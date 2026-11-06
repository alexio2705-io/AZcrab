"""Ночной прогон: настоящий nav.py на сотне случайных лабиринтов с шумами.

    python3 sim/night.py                    # 40 заездов, шум 1, все ядра
    python3 sim/night.py -n 200 --noise 0,1,2 --seed 1000
    python3 sim/night.py -n 32 --noise 0,1,2 --keep bad     # записи - только проблемных
    python3 sim/night.py --compare main . -n 32 --noise 0,1,2
    python3 sim/night.py --compare . .+BLUE_STOP_S=5.3       # тот же код, другой параметр
    python3 sim/night.py -n 300 --sweep gyro_drift_dps=0,0.01,0.02   # чувствительность к шуму
    python3 sim/night.py --one 17 --noise 1 # один заезд (так зовёт себя же)

Каждый заезд - отдельный процесс (свежий nav.py, зависание убивается по
таймауту), 8 минут модельного времени. Мир - sim/mazegen.py по номеру,
железо - maze_sim + fake_hw (плитки, метки, камеры, лента, шумы). Очки - по
регламенту 2026 (sim/scoring.py). Набор заездов - номера лабиринтов от
--seed подряд, каждый со всеми уровнями шума из --noise.

Сравнение "было/стало" (--compare A B): оба варианта на одних и тех же
лабиринтах и шумах. Вариант - это
  .            рабочая копия (pi/ этого репозитория);
  <ветка/коммит>  pi/ из git на этот момент (git archive во временную папку);
  <папка>      другая рабочая копия: папка с nav.py или с pi/nav.py;
и к любому можно дописать +ИМЯ=ЗНАЧЕНИЕ[,ИМЯ=ЗНАЧЕНИЕ] - константы nav.py
(например .+BLUE_STOP_S=5.3). Таблица - средние по метрикам с 95%
доверительными интервалами разности (бутстрэп по парам заездов), список
заездов, где стало хуже, со ссылками на оба повтора.

Что получается в sim/runs/night-<время>/ (или cmp-<время>/):
  index.html          - сводка: очки, время, что сломалось, таблица заездов;
  compare.html        - (сравнение) метрики A против B, где стало хуже;
  results.json        - то же для скриптов;
  seed-N.spec.json    - поле заезда (всегда), чтобы повторить как есть;
  [A-]seed-N-nK.jsonl.gz - запись заезда в формате пульта (pi/dash_web.py):
                      в повторе видно лог, карту, датчики, а ещё ПРАВДУ -
                      где робот был на самом деле, стены, плитки и метки.
--keep all|bad|none: записи всех заездов (~0.5 МБ каждая), только проблемных
(в сравнении - ещё и тех, где B хуже A) или никаких. Если на диске меньше
MIN_FREE_GB, записи перестают сохраняться сами.
Открыть повтор: python3 pi/dash_web.py, ссылки в index.html.
"""
from __future__ import annotations

import argparse
import contextlib
import gzip
import html
import io
import json
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUNS = os.path.join(ROOT, "sim", "runs")
sys.path.insert(0, HERE)

RUN_LIMIT_S = 480.0          # заезд RCJ Maze
REC_PERIOD_S = 0.1           # снимок для пульта - 10 раз в секунду модели
WALL_TIMEOUT_S = 900         # настоящих секунд на заезд, дальше - "завис"
MIN_FREE_GB = 2.0            # меньше свободного места - записи не храним
KITS_NEED = {"damaged": 2, "stable": 1, "unharmed": 0}

# метрики сравнения: ключ, подпись, больше - лучше?
METRICS = [
    ("score", "очки", True), ("score_frac", "доля от максимума", True),
    ("sim_s", "время, с", False), ("coverage", "покрытие", True),
    ("svi", "метки опознаны", True), ("srd", "комплекты", True),
    ("sbv", "синие засчитаны", True), ("blue_repeat", "повторные синие", False),
    ("lop", "LoP", False), ("misid", "ложные мигания", False),
    ("home", "вернулся (доля)", True), ("n_problems", "замечаний", False),
]


# ════════════════════════════════════════════════════════════
#  Один заезд (в своём процессе)
# ════════════════════════════════════════════════════════════
def rec_name(seed, noise, arm=""):
    return f"{arm + '-' if arm else ''}seed-{seed}-n{noise:g}.jsonl.gz"


def run_one(seed, noise_level, out_dir, spec_path=None, arm="", sets=None, noise_set=None):
    """Заезд seed с шумом noise_level. spec_path - поле из файла (иначе
    mazegen.generate(seed)); sets - {константа nav: значение}; noise_set -
    {параметр Noise: значение} поверх уровня шума."""
    import maze_sim as ms
    import fake_hw
    import mazegen
    import live

    nav = ms.nav
    for k, v in (sets or {}).items():
        if not hasattr(nav, k):
            raise SystemExit(f"в nav.py нет {k}")
        setattr(nav, k, v)
    if spec_path:
        with open(spec_path) as f:
            spec = json.load(f)
    else:
        spec = mazegen.generate(seed)
        sp = os.path.join(out_dir, f"seed-{seed}.spec.json")
        if not os.path.exists(sp):
            with open(sp, "w") as f:
                json.dump(spec, f)        # поле заезда - чтобы повторить провал как есть
    world = mazegen.build_world(spec)
    rng_noise = random.Random(seed * 7919 + int(noise_level * 100))
    noise = ms.Noise.level(noise_level, rng_noise)
    for k, v in (noise_set or {}).items():
        setattr(noise, k, v)
    sim = ms.Sim(world, noise=noise, seed=seed, color=True)
    sim.limit = RUN_LIMIT_S
    epoch0 = time.time()
    tee = live.LogTee(io.StringIO(), keep=100000, clock=lambda: epoch0 + (sim.t - sim.t0))
    rec_path = os.path.join(out_dir, rec_name(seed, noise_level, arm))
    rec = Recording(rec_path, sim, nav, tee, mazegen.world_json(spec), spec)
    sim.hooks.append(rec.on_step)

    status, err = "?", None
    cx = cy = 0
    t_wall = time.monotonic()
    with contextlib.redirect_stdout(tee):
        try:
            ms.setup(sim)
            fake_hw.install(sim)
            nav._live["sim"] = f"seed {seed} · шум {noise_level:g}" + (f" · {arm}" if arm else "")
            nav._t_run = nav.time.monotonic()
            cx, cy, hdg, cp, status = nav.flood_fill_run(300)
            if status != "fatal":
                cx, cy, hdg = nav.return_to_start(cx, cy, hdg, cp)
                if (cx, cy) == (0, 0):
                    nav._write_state(cx, cy, hdg, cp, "blinking")
                    nav.blink_start_return()
                    nav._write_state(cx, cy, hdg, cp, "done")
        except ms.SimTimeUp:
            status = "time_up"
            print("── 8 минут модели вышли ──")
        except Exception:           # noqa: BLE001 - любое падение nav.py - находка
            status = "crash"
            err = traceback.format_exc()
            print(err)
    rec.close()
    res = evaluate(spec, sim, nav, status, (cx, cy), tee)
    res.update(seed=seed, noise=noise_level, arm=arm, error=err,
               wall_s=round(time.monotonic() - t_wall, 1),
               rec=os.path.relpath(rec_path, RUNS), size=f'{spec["w"]}x{spec["h"]}',
               features=features_of(spec))
    return res


def features_of(spec):
    out = []
    for r in spec.get("ramps", ()):
        out.append("лестница" if r["kind"] == "stairs" else f"горка{len(r['cells'])}")
    if spec.get("zone"):
        out.append("зона")
    if spec.get("obstacles"):
        out.append(f"препятствия×{len(spec['obstacles'])}")
    return " ".join(out)


class Recording:
    """Запись заезда в формате pi/dash_web.py (строка JSON на снимок) плюс
    правда модели: "world" в первом снимке, "truth" - поза в каждом."""

    def __init__(self, path, sim, nav, tee, world, spec):
        self.f = gzip.open(path, "wt", compresslevel=6)
        self.sim, self.nav, self.tee = sim, nav, tee
        self.world = world
        self.rise = float(spec.get("rise", 0.0))
        self.n = 0
        self.next_t = sim.t
        self.log_seq = 0

    def on_step(self, sim):
        if sim.t < self.next_t:
            return
        self.next_t += REC_PERIOD_S
        try:
            s = self.nav._live_snapshot(self.n)
        except Exception:           # noqa: BLE001 - запись не должна ронять заезд
            return
        s["status"] = self.tee.status
        lines = self.tee.since(self.log_seq)
        if lines:
            s["log"] = lines
            self.log_seq = lines[-1][0]
        lvl = 1 if self.rise and sim.w.height(sim.x, sim.y) > self.rise * 0.75 else 0
        s["truth"] = {"x": round(sim.x), "y": round(sim.y), "th": round(sim.theta, 1),
                      "lvl": lvl, "cell": list(ms_cell(sim)), "kits": dict(sim.kits)}
        if self.n == 0:
            s["truth"]["world"] = self.world
        self.f.write(json.dumps(s, separators=(",", ":"), default=str) + "\n")
        self.n += 1

    def close(self):
        lines = self.tee.since(self.log_seq)
        if lines:
            self.f.write(json.dumps({"t": self.sim.t, "log": lines}, default=str) + "\n")
        self.f.close()


def ms_cell(sim):
    return (int(math.floor(sim.x / 300.0 + 0.5)), int(math.floor(sim.y / 300.0 + 0.5)))


def map_errors(spec, nav):
    """Ошибки карты nav против правды: рёбра клеток, где nav стоял (оба
    этажа: клетка nav с y + 1000 - клетка островка)."""
    import mazegen
    walls = {tuple(e) for e in spec["walls"]}
    high = {tuple(c) for c in spec.get("high", ())}
    rc = mazegen.ramp_cells(spec)
    n = 0
    for (cx, cy), cell in list(nav._maze.items()):
        if (cx, cy) not in nav._visited:
            continue
        lvl = nav.level_of(cy)
        pc = (cx, cy - 1000 * lvl)
        if lvl not in (0, 1) or (pc in high) != (lvl == 1) or pc in rc:
            continue
        if not (0 <= pc[0] < spec["w"] and 0 <= pc[1] < spec["h"]):
            n += 1
            continue
        for d in range(4):
            if cell[d] is None or nav._edge_fixed.get(nav._edge_key(cx, cy, d)) is True:
                continue
            if cell[d] != mazegen.has_wall(walls, pc, d):
                n += 1
    return n


def evaluate(spec, sim, nav, status, nav_end, tee):
    """Что вышло: очки по регламенту (scoring.score) + покрытие, ошибки
    карты, замечания."""
    import mazegen
    import scoring
    reach = mazegen.reachable(spec)
    seen = sim.cells_seen & reach
    end_cell = sim.visits[-1]["cell"] if sim.visits else ms_cell(sim)
    sc = scoring.score(spec, sim.events, sim.visits, end_cell)
    mx = scoring.max_score(spec)
    home = sc["exit"]
    lost = tuple(nav_end) != mazegen.nav_cell(spec, ms_cell(sim))
    map_err = map_errors(spec, nav)
    log_lines = [t for _, _, t in tee.lines]
    warns = sum(1 for t in log_lines if "[WARN]" in t)
    fatals = [t.strip() for t in log_lines if "FATAL" in t or "АВАРИЯ" in t][:3]

    problems = []
    if status == "crash":
        problems.append("nav упал")
    if status == "time_up":
        problems.append("не уложился в 8 мин")
    if lost:
        problems.append("потерял положение")
    if sc["lop_black"]:
        problems.append("LoP: чёрная")
    if sc["lop_blue"]:
        problems.append("LoP: синяя без стоянки")
    if map_err:
        problems.append("ошибки карты")
    if sc["misid"]:
        problems.append("ложная метка")
    if sc["wrong"]:
        problems.append("зрение спутало тип (шум)")
    if sc["dup"]:
        problems.append("метка дважды")
    if sc["bad_blink"]:
        problems.append("мигал на ходу/недолго")
    if sc["missed"]:
        problems.append("пропустил метку")
    if status not in ("crash", "time_up") and not home:
        problems.append("не вернулся")
    if sim.crashes:
        problems.append("упирался в стены")

    out = {
        "status": status, "problems": problems, "n_problems": len(problems),
        "sim_s": round(sim.t - sim.t0, 1),
        "cells": len(reach), "covered": len(seen), "coverage": round(len(seen) / max(1, len(reach)), 3),
        "found": sc["svi"], "max_score": mx, "score_frac": round(sc["score"] / max(1, mx), 3),
        "black_mm": round(sim.black_depth),
        "blue_stops": [e[4]["still_s"] for e in sim.events if e[1] == "blue_left"],
        "home": home, "lost": lost, "end": list(end_cell), "nav_end": list(nav_end),
        "map_err": map_err, "crashes": sim.crashes, "warns": warns, "fatal": fatals,
    }
    out.update(sc)
    return out


# ════════════════════════════════════════════════════════════
#  Варианты nav.py для сравнения
# ════════════════════════════════════════════════════════════
class Arm:
    """Вариант nav.py: папка pi/ и константы поверх. name - для отчёта."""

    def __init__(self, text, label, tmp_root):
        self.text, self.label = text, label
        base, _, sets = text.partition("+")
        self.sets = {}
        for kv in filter(None, sets.split(",")):
            k, _, v = kv.partition("=")
            try:
                self.sets[k.strip()] = json.loads(v)
            except ValueError:
                self.sets[k.strip()] = v
        base = base or "."
        if base == ".":
            self.pi = os.path.join(ROOT, "pi")
            self.desc = "рабочая копия"
        elif os.path.isdir(base):
            p = os.path.abspath(base)
            self.pi = p if os.path.exists(os.path.join(p, "nav.py")) else os.path.join(p, "pi")
            if not os.path.exists(os.path.join(self.pi, "nav.py")):
                raise SystemExit(f"{base}: нет nav.py ни в папке, ни в pi/")
            self.desc = f"папка {base}"
        else:
            rev = subprocess.run(["git", "rev-parse", "--short", base], cwd=ROOT,
                                 capture_output=True, text=True)
            if rev.returncode:
                raise SystemExit(f"{base}: не папка и не ветка/коммит git")
            dst = os.path.join(tmp_root, label)
            os.makedirs(dst)
            arc = subprocess.run(["git", "archive", base, "pi"], cwd=ROOT, capture_output=True)
            subprocess.run(["tar", "-x", "-C", dst], input=arc.stdout, check=True)
            self.pi = os.path.join(dst, "pi")
            self.desc = f"git {base} ({rev.stdout.strip()})"
        if self.sets:
            self.desc += " + " + ", ".join(f"{k}={v}" for k, v in self.sets.items())


# ════════════════════════════════════════════════════════════
#  Много заездов + отчёт
# ════════════════════════════════════════════════════════════
def launch(seed, noise, out_dir, arm=None, noise_set=None):
    cmd = [sys.executable, os.path.abspath(__file__), "--one", str(seed), "--noise", str(noise),
           "--out", out_dir, "--spec", os.path.join(out_dir, f"seed-{seed}.spec.json")]
    env = dict(os.environ)
    if arm is not None:
        cmd += ["--arm", arm.label]
        if arm.sets:
            cmd += ["--set", json.dumps(arm.sets)]
        env["MAZE_PI_DIR"] = arm.pi
    if noise_set:
        cmd += ["--noise-set", json.dumps(noise_set)]
    t0 = time.monotonic()
    base = {"seed": seed, "noise": noise, "arm": arm.label if arm else "",
            "noise_set": noise_set or {}}
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=WALL_TIMEOUT_S, env=env)
    except subprocess.TimeoutExpired:
        return {**base, "status": "hang", "problems": ["завис (настоящее время)"], "n_problems": 1,
                "score": 0, "wall_s": WALL_TIMEOUT_S}
    try:
        res = json.loads(p.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        res = {**base, "status": "crash", "problems": ["процесс упал"], "n_problems": 1, "score": 0,
               "error": (p.stderr or p.stdout)[-3000:], "wall_s": round(time.monotonic() - t0, 1)}
    res["noise_set"] = noise_set or {}
    return res


def free_gb(path):
    return shutil.disk_usage(path).free / 1e9


def drop_rec(out_dir, r):
    p = os.path.join(RUNS, r.get("rec") or "")
    if r.get("rec") and os.path.exists(p):
        os.remove(p)
    r["rec"] = None


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def metric_val(r, key):
    v = r.get(key)
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    return float(v) if isinstance(v, (int, float)) else None


def boot_ci(diffs, n_boot=2000, seed=1):
    """95% доверительный интервал среднего (бутстрэп, детерминированный)."""
    if len(diffs) < 2:
        return (float("nan"), float("nan"))
    rng = random.Random(seed)
    n = len(diffs)
    ms = sorted(sum(diffs[rng.randrange(n)] for _ in range(n)) / n for _ in range(n_boot))
    return ms[int(0.025 * n_boot)], ms[int(0.975 * n_boot) - 1]


def compare_table(pairs):
    """[(метрика, подпись, среднее A, среднее B, разность, ci_lo, ci_hi, вердикт)]."""
    rows = []
    for key, label, up in METRICS:
        a = [metric_val(pa, key) for pa, pb in pairs]
        b = [metric_val(pb, key) for pa, pb in pairs]
        ok = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
        if not ok:
            continue
        d = [y - x for x, y in ok]
        lo, hi = boot_ci(d)
        md = mean(d)
        verdict = "—"
        if not math.isnan(lo) and (lo > 0 or hi < 0):
            better = (md > 0) == up
            verdict = "лучше" if better else "хуже"
        rows.append((key, label, mean([x for x, _ in ok]), mean([y for _, y in ok]), md, lo, hi, verdict))
    return rows


def worse(pa, pb):
    """Почему B хуже A в этом заезде (пустой список - не хуже)."""
    why = []
    if pb.get("score", 0) < pa.get("score", 0):
        why.append(f"очки {pa.get('score')} → {pb.get('score')}")
    if pa.get("home") and not pb.get("home"):
        why.append("перестал возвращаться")
    new = set(pb.get("problems", [])) - set(pa.get("problems", []))
    if new:
        why.append("новое: " + ", ".join(sorted(new)))
    return why


CSS = """:root{color-scheme:dark;--bg:#111110;--card:#1a1a19;--fg:#f2f2ee;--mut:#85847c;--line:rgba(255,255,255,.08);--ok:#3fb27f;--bad:#e5534b}
body{margin:0;padding:24px 16px;background:var(--bg);color:var(--fg);font:13px/1.45 system-ui,sans-serif}
h1{font-size:20px;margin:0 0 4px}h2{font-size:14px;margin:20px 0 8px}p{color:var(--mut);margin:0 0 16px}
.k{display:flex;gap:28px;flex-wrap:wrap;margin:16px 0}.k b{display:block;font:600 26px ui-monospace,Menlo,monospace}.k span{color:var(--mut)}
.wrap{overflow-x:auto}table{border-collapse:collapse;width:100%;background:var(--card)}
th,td{padding:6px 10px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
th{color:var(--mut);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.06em}
td.n{font-family:ui-monospace,Menlo,monospace;text-align:right;white-space:nowrap}td.pr{color:#f0a49f}
.good{color:var(--ok)}.badv{color:var(--bad)}
a{color:#8fbcf3}pre{white-space:pre-wrap;font-size:11px;color:var(--mut)}ul{margin:0 0 16px;padding-left:20px}"""


def link(r, text="повтор"):
    return (f'<a href="http://localhost:8888/?run={html.escape(r["rec"])}">{text}</a>'
            if r.get("rec") else "")


def page(title, body):
    return (f'<!doctype html><html lang="ru"><head><meta charset="utf-8">'
            f'<meta name="viewport" content="width=device-width,initial-scale=1"><title>{html.escape(title)}</title>'
            f'<style>{CSS}</style></head><body>{body}'
            f'<p style="margin-top:16px">Повтор открывается в пульте: <code>python3 pi/dash_web.py</code>, '
            f'потом ссылка «повтор».</p></body></html>')


def runs_table(results):
    def row(r):
        probs = r.get("problems", [])
        err = f'<details><summary>ошибка</summary><pre>{html.escape(r["error"])}</pre></details>' \
            if r.get("error") else ""
        p = r.get("parts", {})
        tip = html.escape(", ".join(f"{k} {v}" for k, v in p.items() if v))
        ns = "".join(f" {k}={v}" for k, v in (r.get("noise_set") or {}).items())
        return (f'<tr><td>{r["seed"]}</td><td>{r["noise"]:g}{html.escape(ns)}</td>'
                f'<td>{html.escape(r.get("size", ""))} {html.escape(r.get("features", ""))}</td>'
                f'<td>{html.escape(r.get("status", ""))}</td>'
                f'<td class="n" title="{tip}">{r.get("score", "")}/{r.get("max_score", "")}</td>'
                f'<td class="n">{r.get("sim_s", "")}</td>'
                f'<td class="n">{r.get("covered", "")}/{r.get("cells", "")}</td>'
                f'<td class="n">{r.get("svi", "")}/{r.get("victims", "")}</td>'
                f'<td class="n">{r.get("srd", "")}/{r.get("kits_need", "")}</td>'
                f'<td class="n">{r.get("sbv", "")}/{r.get("blue_tiles", "")} ↻{r.get("blue_repeat", "")}</td>'
                f'<td class="n">{r.get("lop", "")}</td>'
                f'<td>{"да" if r.get("home") else "нет"}</td>'
                f'<td class="pr">{html.escape(", ".join(probs)) or "—"}{err}</td><td>{link(r)}</td></tr>')

    results = sorted(results, key=lambda r: (-len(r.get("problems", [])), r["seed"], r["noise"]))
    return ('<div class="wrap"><table><tr><th>№</th><th>шум</th><th>поле</th><th>статус</th>'
            '<th>очки</th><th>время, с</th><th>клетки</th><th>метки</th><th>комплекты</th>'
            '<th>синие ↻повт.</th><th>LoP</th><th>дома</th><th>замечания</th><th></th></tr>'
            + "".join(row(r) for r in results) + '</table></div>')


def summary(results):
    from collections import Counter
    n = len(results)
    cnt = Counter(p for r in results for p in r.get("problems", []))
    sc = [r.get("score", 0) for r in results]
    fr = [r["score_frac"] for r in results if "score_frac" in r]
    tm = [r["sim_s"] for r in results if "sim_s" in r]
    k = (f'<div class="k"><div><b>{mean(sc):.0f}</b><span>очков в среднем</span></div>'
         f'<div><b>{mean(fr) * 100:.0f}%</b><span>от максимума поля</span></div>'
         f'<div><b>{mean(tm):.0f} с</b><span>время заезда</span></div>'
         f'<div><b>{sum(1 for r in results if not r.get("problems"))}/{n}</b><span>без замечаний</span></div>'
         f'<div><b>{sum(r.get("svi", 0) for r in results)}/{sum(r.get("victims", 0) for r in results)}</b><span>меток опознано</span></div>'
         f'<div><b>{sum(r.get("blue_repeat", 0) for r in results)}</b><span>повторных синих</span></div>'
         f'<div><b>{sum(r.get("lop", 0) for r in results)}</b><span>LoP</span></div>'
         f'<div><b>{sum(1 for r in results if r.get("home"))}/{n}</b><span>вернулись на старт</span></div></div>')
    top = "".join(f"<li><b>{c}</b> × {html.escape(p)}</li>" for p, c in cnt.most_common())
    by_noise = {}
    for r in results:
        by_noise.setdefault(r["noise"], []).append(r)
    rows = "".join(
        f'<tr><td>{k:g}</td><td class="n">{len(v)}</td><td class="n">{mean([r.get("score", 0) for r in v]):.0f}</td>'
        f'<td class="n">{mean([r.get("score_frac", 0) for r in v]) * 100:.0f}%</td>'
        f'<td class="n">{mean([r.get("sim_s", 0) for r in v]):.0f}</td>'
        f'<td class="n">{sum(r.get("svi", 0) for r in v)}/{sum(r.get("victims", 0) for r in v)}</td>'
        f'<td class="n">{sum(r.get("blue_repeat", 0) for r in v)}</td><td class="n">{sum(r.get("lop", 0) for r in v)}</td>'
        f'<td class="n">{sum(1 for r in v if r.get("home"))}/{len(v)}</td></tr>'
        for k, v in sorted(by_noise.items()))
    tbl = ('<div class="wrap"><table><tr><th>шум</th><th>заездов</th><th>очки</th><th>от макс.</th>'
           '<th>время, с</th><th>метки</th><th>повт. синие</th><th>LoP</th><th>дома</th></tr>'
           + rows + '</table></div>')
    return k + tbl + f'<h2>Что ломалось</h2><ul>{top or "<li>ничего</li>"}</ul>', cnt


def report(results, out_dir, title, sub):
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    head, cnt = summary(results)
    body = (f'<h1>{html.escape(title)}</h1><p>{html.escape(sub)}</p>{head}'
            f'<h2>Заезды</h2>{runs_table(results)}')
    path = os.path.join(out_dir, "index.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(page(title, body))
    return path, cnt


def fmt(v, key):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "—"
    if key in ("score_frac", "coverage", "home"):
        return f"{v * 100:.1f}%"
    return f"{v:.2f}" if abs(v) < 10 else f"{v:.1f}"


def compare_report(pairs, arms, out_dir, sub):
    rows = compare_table(pairs)
    bad = [(pa, pb, worse(pa, pb)) for pa, pb in pairs]
    bad = sorted([x for x in bad if x[2]], key=lambda x: x[1].get("score", 0) - x[0].get("score", 0))
    t = "".join(
        f'<tr><td>{html.escape(label)}</td><td class="n">{fmt(a, key)}</td><td class="n">{fmt(b, key)}</td>'
        f'<td class="n">{fmt(d, key) if key not in ("score_frac", "coverage", "home") else f"{d * 100:+.1f}%"}</td>'
        f'<td class="n">[{fmt(lo, key)} … {fmt(hi, key)}]</td>'
        f'<td class="{"good" if v == "лучше" else "badv" if v == "хуже" else ""}">{v}</td></tr>'
        for key, label, a, b, d, lo, hi, v in rows)
    w = "".join(
        f'<tr><td>{pa["seed"]}</td><td>{pa["noise"]:g}</td><td class="n">{pa.get("score")} → {pb.get("score")}</td>'
        f'<td class="n">{pa.get("sim_s")} → {pb.get("sim_s")}</td>'
        f'<td class="pr">{html.escape("; ".join(why))}</td><td>{link(pa, "A")} {link(pb, "B")}</td></tr>'
        for pa, pb, why in bad)
    body = (f'<h1>Сравнение: A против B</h1><p>{html.escape(sub)}</p>'
            f'<p>A — {html.escape(arms[0].desc)}<br>B — {html.escape(arms[1].desc)}</p>'
            f'<div class="wrap"><table><tr><th>метрика</th><th>A</th><th>B</th><th>B − A</th>'
            f'<th>95% ДИ разности</th><th>итог</th></tr>{t}</table></div>'
            f'<p>ДИ — бутстрэп по {len(pairs)} парам заездов (одно поле, один шум); «лучше/хуже» — '
            f'интервал не задевает ноль.</p>'
            f'<h2>Где B хуже A ({len(bad)})</h2><div class="wrap"><table><tr><th>№</th><th>шум</th>'
            f'<th>очки</th><th>время</th><th>почему</th><th>повтор</th></tr>{w}</table></div>'
            f'<p><a href="index.html">все заезды</a></p>')
    path = os.path.join(out_dir, "compare.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(page("Сравнение A/B", body))
    return path, rows, bad


def parse_sweep(text):
    k, _, vals = text.partition("=")
    return k.strip(), [float(v) for v in vals.split(",") if v.strip()]


def main():
    ap = argparse.ArgumentParser(description="Ночной прогон nav.py на случайных лабиринтах")
    ap.add_argument("-n", type=int, default=40, help="сколько лабиринтов (каждый - со всеми шумами)")
    ap.add_argument("--seed", type=int, default=1, help="первый номер лабиринта")
    ap.add_argument("--noise", default="1", help="уровень шума или список через запятую (0 - идеально)")
    ap.add_argument("-j", "--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--compare", nargs=2, metavar=("A", "B"), help="сравнить два варианта nav.py")
    ap.add_argument("--keep", choices=("all", "bad", "none"), default="all",
                    help="записи заездов: все, только проблемные, никакие (поле - всегда)")
    ap.add_argument("--sweep", help="ПАРАМЕТР=v1,v2,... - шум Noise поверх уровня, по значению на набор")
    ap.add_argument("--features", help="особенности поля через запятую (mazegen.FEATURES; '' - никаких)")
    ap.add_argument("--big", action="store_true", help="большие поля 7-10 x 6-8 (лимит 8 мин ограничивает)")
    ap.add_argument("--one", type=int, help="один заезд в этом процессе (служебное)")
    ap.add_argument("--spec", help="(служебное) поле из файла")
    ap.add_argument("--arm", default="", help="(служебное) метка варианта")
    ap.add_argument("--set", help="(служебное) JSON констант nav.py")
    ap.add_argument("--noise-set", help="(служебное) JSON параметров шума")
    ap.add_argument("--out", help="папка результатов")
    a = ap.parse_args()

    if a.one is not None:
        out = a.out or os.path.join(RUNS, "one")
        os.makedirs(out, exist_ok=True)
        res = run_one(a.one, float(a.noise), out, a.spec, a.arm,
                      json.loads(a.set) if a.set else None,
                      json.loads(a.noise_set) if a.noise_set else None)
        sys.stdout.write("\n" + json.dumps(res, ensure_ascii=False, default=str) + "\n")
        return

    import mazegen
    stamp = time.strftime("%Y-%m-%d_%H-%M")
    out = a.out or os.path.join(RUNS, ("cmp-" if a.compare else "night-") + stamp)
    os.makedirs(out, exist_ok=True)
    levels = [float(x) for x in a.noise.split(",")]
    feats = None if a.features is None else [f for f in a.features.split(",") if f]
    for i in range(a.n):               # поля - заранее: у обоих вариантов одни и те же
        sp = os.path.join(out, f"seed-{a.seed + i}.spec.json")
        if not os.path.exists(sp):
            with open(sp, "w") as f:
                json.dump(mazegen.generate(a.seed + i, features=feats, big=a.big), f)
    sweeps = [None]
    if a.sweep:
        k, vals = parse_sweep(a.sweep)
        sweeps = [{k: v} for v in vals]
    jobs = [(a.seed + i, lv, ns) for i in range(a.n) for lv in levels for ns in sweeps]
    tmp = tempfile.mkdtemp(prefix="maze-arms-")
    arms = [Arm(t, lb, tmp) for t, lb in zip(a.compare, ("A", "B"))] if a.compare else [None]
    keep = a.keep
    sub = (f"{a.n} {'больших ' if a.big else ''}полей × шум {a.noise}{' × ' + a.sweep if a.sweep else ''} · номера с {a.seed} · "
           f"записи: {keep} · настоящий nav.py в модели (sim/night.py)")
    total = len(jobs) * len(arms)
    print(f"Прогон: {total} заездов ({a.n} полей × шум {a.noise}"
          f"{' × ' + str(len(sweeps)) + ' значений' if a.sweep else ''}"
          f"{' × 2 варианта' if a.compare else ''}), {a.jobs} параллельно -> {out}")
    for arm in arms:
        if arm is not None:
            print(f"  {arm.label}: {arm.desc}")

    results, done, t0 = [], 0, time.monotonic()
    low_disk = False
    by_key = {}

    def finished(r):
        nonlocal done, low_disk
        done += 1
        if not low_disk and free_gb(out) < MIN_FREE_GB:
            low_disk = True
            print(f"  [ДИСК] свободно меньше {MIN_FREE_GB:g} ГБ - записи дальше не храню")
        key = (r["seed"], r["noise"], json.dumps(r.get("noise_set") or {}, sort_keys=True))
        if a.compare:
            by_key.setdefault(key, {})[r.get("arm")] = r
            pair = by_key[key]
            if len(pair) == 2:         # судьба записей - когда готовы оба
                pa, pb = pair["A"], pair["B"]
                keep_pair = keep == "all" or (keep == "bad" and (
                    pa.get("problems") or pb.get("problems") or worse(pa, pb)))
                if low_disk or not keep_pair:
                    drop_rec(out, pa)
                    drop_rec(out, pb)
        elif low_disk or keep == "none" or (keep == "bad" and not r.get("problems")):
            drop_rec(out, r)
        results.append(r)
        probs = ", ".join(r.get("problems", [])) or "чисто"
        print(f"  [{done}/{total}] {r.get('arm', '') + ' ' if r.get('arm') else ''}seed {r['seed']} "
              f"шум {r['noise']:g}: {r.get('status')} {r.get('score', '?')} очк, "
              f"{r.get('covered', '?')}/{r.get('cells', '?')} клеток, метки "
              f"{r.get('svi', '?')}/{r.get('victims', '?')} - {probs}", flush=True)

    try:
        with ThreadPoolExecutor(max_workers=a.jobs) as ex:
            futs = [ex.submit(launch, s, k, out, arm, ns) for s, k, ns in jobs for arm in arms]
            for f in futs:
                finished(f.result())
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if a.compare:
        title = "Сравнение " + os.path.basename(out)
        report(results, out, title, sub)
        pairs = [(v["A"], v["B"]) for k, v in sorted(by_key.items()) if len(v) == 2]
        path, rows, bad = compare_report(pairs, arms, out, sub)
        print(f"\nГотово за {time.monotonic() - t0:.0f}с.")
        for key, label, ma, mb, d, lo, hi, v in rows:
            print(f"  {label:20s} A {fmt(ma, key):>8s}  B {fmt(mb, key):>8s}  "
                  f"ДИ разности [{fmt(lo, key)} … {fmt(hi, key)}]  {v}")
        print(f"  B хуже A в {len(bad)} заездах из {len(pairs)}")
        print(f"Отчёт: {path}")
        return
    path, cnt = report(results, out, "Ночной прогон " + os.path.basename(out), sub)
    print(f"\nГотово за {time.monotonic() - t0:.0f}с. Чаще всего: "
          + "; ".join(f"{p} ×{c}" for p, c in cnt.most_common(5)))
    print(f"Отчёт: {path}")


if __name__ == "__main__":
    main()
