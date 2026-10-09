"""Ночной прогон: настоящий nav.py на сотне случайных лабиринтов с шумами.

    python3 sim/night.py                    # 40 заездов, шум 1, все ядра
    python3 sim/night.py -n 200 --noise 0,1,2 --seed 1000
    python3 sim/night.py --one 17 --noise 1 # один заезд (так зовёт себя же)

Каждый заезд - отдельный процесс (свежий nav.py, зависание убивается по
таймауту), 8 минут модельного времени. Мир - sim/mazegen.py по номеру,
железо - maze_sim + fake_hw (плитки, метки, камеры, лента, шумы).

Что получается в sim/runs/night-<время>/:
  index.html        - сводка: что сломалось чаще всего, таблица заездов;
  results.json      - то же для скриптов;
  seed-N.jsonl.gz   - запись заезда в формате пульта (pi/dash_web.py): в
                      повторе видно лог, карту, датчики, а ещё ПРАВДУ - где
                      робот был на самом деле, настоящие стены, плитки и метки.
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
import subprocess
import sys
import time
import traceback
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

RUN_LIMIT_S = 480.0          # заезд RCJ Maze
REC_PERIOD_S = 0.1           # снимок для пульта - 10 раз в секунду модели
WALL_TIMEOUT_S = 900         # настоящих секунд на заезд, дальше - "завис"
KITS_NEED = {"damaged": 2, "stable": 1, "unharmed": 0}
NEAR_VICTIM_MM = 260.0       # робот у метки (центр робота - центр метки)
BLACK_DEEP_MM = 0.0          # центр робота за краем чёрной = больше половины робота на
                             # ней - 'visited', lack of progress (регламент 5.4.4, 5.5.1b)


# ════════════════════════════════════════════════════════════
#  Один заезд (в своём процессе)
# ════════════════════════════════════════════════════════════
def run_one(seed, noise_level, out_dir):
    import maze_sim as ms
    import fake_hw
    import mazegen
    import live

    nav = ms.nav
    spec = mazegen.generate(seed)
    with open(os.path.join(out_dir, f"seed-{seed}.spec.json"), "w") as f:
        json.dump(spec, f)            # поле заезда - чтобы повторить провал как есть
    world = mazegen.build_world(spec)
    rng_noise = __import__("random").Random(seed * 7919 + int(noise_level * 100))
    noise = ms.Noise.level(noise_level, rng_noise)
    sim = ms.Sim(world, noise=noise, seed=seed, color=True)
    sim.limit = RUN_LIMIT_S
    epoch0 = time.time()
    tee = live.LogTee(io.StringIO(), keep=100000, clock=lambda: epoch0 + (sim.t - sim.t0))
    rec_path = os.path.join(out_dir, f"seed-{seed}.jsonl.gz")
    rec = Recording(rec_path, sim, nav, tee, mazegen.world_json(spec))
    sim.hooks.append(rec.on_step)

    status, err = "?", None
    cx = cy = hdg = 0
    t_wall = time.monotonic()
    with contextlib.redirect_stdout(tee):
        try:
            ms.setup(sim)
            fake_hw.install(sim)
            nav._live["sim"] = f"seed {seed} · шум {noise_level:g}"
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
        except Exception as e:      # noqa: BLE001 - любое падение nav.py - находка
            status = "crash"
            err = traceback.format_exc()
            print(err)
    rec.close()
    res = score(spec, sim, nav, status, (cx, cy), tee)
    res.update(seed=seed, noise=noise_level, error=err, wall_s=round(time.monotonic() - t_wall, 1),
               rec=os.path.relpath(rec_path, os.path.join(ROOT, "sim", "runs")),
               size=f'{spec["w"]}x{spec["h"]}')
    return res


class Recording:
    """Запись заезда в формате pi/dash_web.py (строка JSON на снимок) плюс
    правда модели: "world" в первом снимке, "truth" - поза в каждом."""

    def __init__(self, path, sim, nav, tee, world):
        self.f = gzip.open(path, "wt", compresslevel=6)
        self.sim, self.nav, self.tee = sim, nav, tee
        self.world = world
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
        s["truth"] = {"x": round(sim.x), "y": round(sim.y), "th": round(sim.theta, 1),
                      "cell": list(ms_cell(sim)), "kits": dict(sim.kits)}
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


def score(spec, sim, nav, status, nav_end, tee):
    """Что вышло: покрытие, метки, плитки, возврат, ошибки карты."""
    import mazegen
    reach = mazegen.reachable(spec)
    seen = sim.cells_seen & reach
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    walls = {tuple(e) for e in spec["walls"]}

    # метки: мигание у правильной метки правильного типа
    vic = spec["victims"]
    blinks = [e for e in sim.events if e[1] == "blink"]
    found, wrong, false_, dup = set(), 0, 0, 0
    for t, _, x, y, d in blinks:
        tr = d.get("truth")
        if tr is None:
            false_ += 1
            continue
        v = vic[tr]
        import maze_sim as ms
        px, py = ms.Victim(tuple(v["cell"]), v["d"], v["type"], v["along"]).point()
        if math.hypot(px - x, py - y) > NEAR_VICTIM_MM:
            false_ += 1
            continue
        if d.get("type") != v["type"]:
            wrong += 1
            continue
        if tr in found:
            dup += 1
        found.add(tr)
    reach_vic = [i for i, v in enumerate(vic) if tuple(v["cell"]) in reach]
    real_vic = [i for i in reach_vic if vic[i]["type"] in KITS_NEED]
    kits_need = sum(KITS_NEED[vic[i]["type"]] for i in real_vic)
    kits_given = sum(sim.kits.values())

    # карта nav против правды: только ребра клеток, где он стоял
    map_err = 0
    for (cx, cy), cell in list(nav._maze.items()):
        if (cx, cy) not in nav._visited or nav.level_of(cy) != 0:
            continue
        for d in range(4):
            if cell[d] is None or nav._edge_fixed.get(nav._edge_key(cx, cy, d)) is True:
                continue
            if cell[d] != mazegen.has_wall(walls, (cx, cy), d):
                map_err += 1

    black_in = round(sim.black_depth)
    blue = [e[4]["still_s"] for e in sim.events if e[1] == "blue_left"]
    blue_cells = {c for c, t in tiles.items() if t == "blue" and c in reach}
    blue_visited = {tuple(e[4]["cell"]) for e in sim.events if e[1] == "tile" and e[4].get("tile") == "blue"}
    cp_cells = {c for c, t in tiles.items() if t == "silver" and c in reach}
    cp_visited = {tuple(e[4]["cell"]) for e in sim.events if e[1] == "tile" and e[4].get("tile") == "silver"}
    end_cell = ms_cell(sim)
    home = end_cell == (0, 0) and any(e[1] == "finish_blink" for e in sim.events)
    lost = nav_end != end_cell            # nav думает, что он не там, где есть
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
    if black_in > BLACK_DEEP_MM:
        problems.append("заехал на чёрную")
    if map_err:
        problems.append("ошибки карты")
    if false_:
        problems.append("ложная метка")
    if wrong:
        problems.append("зрение спутало тип (шум)")
    if dup:
        problems.append("метка дважды")
    if len(found & set(real_vic)) + wrong < len(real_vic):
        problems.append("пропустил метку")
    if any(s < 5.0 for s in blue) or (blue_cells - blue_visited and len(seen) == len(reach)):
        problems.append("синяя без стоянки")
    if status not in ("crash", "time_up") and not home:
        problems.append("не вернулся")
    if sim.crashes:
        problems.append("упирался в стены")

    return {
        "status": status, "problems": problems,
        "sim_s": round(sim.t - sim.t0, 1),
        "cells": len(reach), "covered": len(seen), "coverage": round(len(seen) / max(1, len(reach)), 3),
        "victims": len(real_vic), "found": len(found & set(real_vic)), "wrong": wrong,
        "false": false_, "dup": dup, "kits_need": kits_need, "kits_given": kits_given,
        "black_mm": black_in, "blue_stops": blue, "checkpoints": f"{len(cp_visited & cp_cells)}/{len(cp_cells)}",
        "home": home, "lost": lost, "end": list(end_cell), "nav_end": list(nav_end),
        "map_err": map_err, "crashes": sim.crashes, "warns": warns, "fatal": fatals,
    }


# ════════════════════════════════════════════════════════════
#  Много заездов + отчёт
# ════════════════════════════════════════════════════════════
def launch(seed, noise, out_dir):
    cmd = [sys.executable, os.path.abspath(__file__), "--one", str(seed), "--noise", str(noise),
           "--out", out_dir]
    t0 = time.monotonic()
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=WALL_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return {"seed": seed, "noise": noise, "status": "hang", "problems": ["завис (настоящее время)"],
                "wall_s": WALL_TIMEOUT_S}
    try:
        res = json.loads(p.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        res = {"seed": seed, "noise": noise, "status": "crash", "problems": ["процесс упал"],
               "error": (p.stderr or p.stdout)[-3000:], "wall_s": round(time.monotonic() - t0, 1)}
    return res


def report(results, out_dir, args):
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    from collections import Counter
    cnt = Counter(p for r in results for p in r.get("problems", []))
    n = len(results)
    clean = sum(1 for r in results if not r.get("problems"))
    cov = [r["coverage"] for r in results if "coverage" in r]
    found = sum(r.get("found", 0) for r in results)
    vics = sum(r.get("victims", 0) for r in results)
    rel = os.path.basename(out_dir)

    def row(r):
        probs = r.get("problems", [])
        link = (f'<a href="http://localhost:8888/?run={html.escape(r["rec"])}">повтор</a>'
                if r.get("rec") else "")
        err = f'<details><summary>ошибка</summary><pre>{html.escape(r["error"])}</pre></details>' \
            if r.get("error") else ""
        return (f'<tr class="{"bad" if probs else "ok"}"><td>{r["seed"]}</td><td>{r["noise"]}</td>'
                f'<td>{html.escape(r.get("size", ""))}</td><td>{html.escape(r.get("status", ""))}</td>'
                f'<td class="n">{r.get("sim_s", "")}</td>'
                f'<td class="n">{r.get("covered", "")}/{r.get("cells", "")}</td>'
                f'<td class="n">{r.get("found", "")}/{r.get("victims", "")}</td>'
                f'<td class="n">{r.get("kits_given", "")}/{r.get("kits_need", "")}</td>'
                f'<td class="n">{r.get("checkpoints", "")}</td>'
                f'<td>{"да" if r.get("home") else "нет"}</td>'
                f'<td>{html.escape(", ".join(probs)) or "—"}{err}</td><td>{link}</td></tr>')

    results = sorted(results, key=lambda r: (-len(r.get("problems", [])), r["seed"]))
    top = "".join(f"<li><b>{c}</b> × {html.escape(p)}</li>" for p, c in cnt.most_common())
    page = f"""<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Ночной прогон</title>
<style>
:root{{color-scheme:dark;--bg:#111110;--card:#1a1a19;--fg:#f2f2ee;--mut:#85847c;--line:rgba(255,255,255,.08);--ok:#3fb27f;--bad:#e5534b}}
body{{margin:0;padding:24px 16px;background:var(--bg);color:var(--fg);font:13px/1.45 system-ui,sans-serif}}
h1{{font-size:20px;margin:0 0 4px}}p{{color:var(--mut);margin:0 0 16px}}
.k{{display:flex;gap:28px;flex-wrap:wrap;margin:16px 0}}.k b{{display:block;font:600 26px ui-monospace,Menlo,monospace}}.k span{{color:var(--mut)}}
.wrap{{overflow-x:auto}}table{{border-collapse:collapse;width:100%;background:var(--card)}}
th,td{{padding:6px 10px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}}
th{{color:var(--mut);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.06em}}
td.n{{font-family:ui-monospace,Menlo,monospace;text-align:right}}tr.bad td:nth-child(11){{color:#f0a49f}}
a{{color:#8fbcf3}}pre{{white-space:pre-wrap;font-size:11px;color:var(--mut)}}ul{{margin:0 0 16px;padding-left:20px}}
</style></head><body>
<h1>Ночной прогон {html.escape(rel)}</h1>
<p>{n} заездов · шум {html.escape(args.noise)} · первый номер {args.seed} · настоящий nav.py в модели (sim/night.py)</p>
<div class="k"><div><b>{clean}/{n}</b><span>без замечаний</span></div>
<div><b>{(sum(cov) / max(1, len(cov)) * 100):.0f}%</b><span>среднее покрытие</span></div>
<div><b>{found}/{vics}</b><span>меток найдено</span></div>
<div><b>{sum(1 for r in results if r.get("home"))}/{n}</b><span>вернулись на старт</span></div></div>
<h2 style="font-size:14px">Что ломалось</h2><ul>{top or "<li>ничего</li>"}</ul>
<div class="wrap"><table><tr><th>№</th><th>шум</th><th>поле</th><th>статус</th><th>время, с</th><th>клетки</th>
<th>метки</th><th>комплекты</th><th>чекпоинты</th><th>дома</th><th>замечания</th><th></th></tr>
{"".join(row(r) for r in results)}</table></div>
<p style="margin-top:16px">Повтор открывается в пульте: <code>python3 pi/dash_web.py</code>, потом ссылка «повтор».</p>
</body></html>"""
    path = os.path.join(out_dir, "index.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(page)
    return path, cnt


def main():
    ap = argparse.ArgumentParser(description="Ночной прогон nav.py на случайных лабиринтах")
    ap.add_argument("-n", type=int, default=40, help="сколько заездов")
    ap.add_argument("--seed", type=int, default=1, help="первый номер лабиринта")
    ap.add_argument("--noise", default="1", help="уровень шума или список через запятую (0 - идеально)")
    ap.add_argument("-j", "--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--one", type=int, help="один заезд в этом процессе (служебное)")
    ap.add_argument("--out", help="папка результатов")
    a = ap.parse_args()

    if a.one is not None:
        res = run_one(a.one, float(a.noise), a.out)
        sys.stdout.write("\n" + json.dumps(res, ensure_ascii=False, default=str) + "\n")
        return

    out = a.out or os.path.join(ROOT, "sim", "runs", "night-" + time.strftime("%Y-%m-%d_%H-%M"))
    os.makedirs(out, exist_ok=True)
    levels = [float(x) for x in a.noise.split(",")]
    jobs = [(a.seed + i, levels[i % len(levels)]) for i in range(a.n)]
    print(f"Ночной прогон: {a.n} заездов, шум {a.noise}, {a.jobs} параллельно -> {out}")
    results, t0 = [], time.monotonic()
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [ex.submit(launch, s, k, out) for s, k in jobs]
        for i, f in enumerate(futs, 1):
            r = f.result()
            results.append(r)
            probs = ", ".join(r.get("problems", [])) or "чисто"
            print(f"  [{i}/{a.n}] seed {r['seed']} шум {r['noise']}: {r.get('status')} "
                  f"{r.get('covered', '?')}/{r.get('cells', '?')} клеток, метки "
                  f"{r.get('found', '?')}/{r.get('victims', '?')} - {probs}", flush=True)
    path, cnt = report(results, out, a)
    print(f"\nГотово за {time.monotonic() - t0:.0f}с. Чаще всего: "
          + "; ".join(f"{p} ×{c}" for p, c in cnt.most_common(5)))
    print(f"Отчёт: {path}")


if __name__ == "__main__":
    main()
