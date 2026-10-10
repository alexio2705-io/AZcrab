"""Разбор пропущенных меток: почему nav.py не мигнул у метки.

    python3 sim/victim_audit.py --dir sim/runs/night-big --noise 0
    python3 sim/victim_audit.py --seed 106 --noise 0 --spec sim/runs/night-big/seed-106.spec.json

Заезд повторяется в этом процессе (как night.run_one, по спецификации поля), а
модель камер подглядывается на каждом такте: когда метка была в кадре
(fake_hw.view - геометрия поля зрения, дальность, этаж), была ли камера
включена (наблюдатель на ходу armed, скан стоя read_wall), сколько пикселей
занимала рамка. По этому каждая пропущенная метка попадает в одну из причин:

  never_in_frame   - ни разу не попала в кадр: робот не вставал боком к этой
                     стене (nav.py) или модель камер не видит, что должна;
  frame_unarmed    - была в кадре только когда камера не смотрела (на ходу
                     наблюдатель выключен, стоя скана этой стены не было);
  watch_small      - наблюдатель видел, но рамка мельче MIN_BOX_PX / решение
                     "не тормозить" (строки [W] в логе);
  still_filtered   - скан стоя был, метка в кадре, но рамка меньше 1/8 кадра
                     (фильтр read_wall);
  read_no_blink    - прочитана, но мигания у неё нет (дубль/приписана не той
                     стене/кадр ушёл).
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import night  # noqa: E402


class Spy:
    """Подглядывает за камерами модели: для каждой метки - когда в кадре,
    включена ли камера, размер рамки, поза робота."""

    def __init__(self, sim, fake_hw):
        self.sim, self.fh = sim, fake_hw
        self.frames = {}         # индекс метки -> [(t, side, px, armed, x, y, th)]
        self.still = {}          # индекс метки -> [(t, side, px, read)]
        self.next_t = 0.0
        self.armed = set()

    def on_step(self, sim):
        if sim.t < self.next_t:
            return
        self.next_t = sim.t + 0.04
        w = self.fh.ms.nav._watcher
        armed = set(getattr(w, "_armed", ()) or ())
        for side in ("left", "right"):
            for i, v, box in self.fh.view(sim, side):
                px = box[3] - box[1]
                self.frames.setdefault(i, []).append(
                    (round(sim.t - sim.t0, 2), side, round(px), side in armed,
                     round(sim.x), round(sim.y), round(sim.theta)))

    def wrap_read(self, det):
        orig = det.read_wall
        spy = self

        def read_wall(cx, cy, wall_dir=None):
            seen = spy.fh.view(spy.sim, det.side)
            res = orig(cx, cy, wall_dir)
            for i, v, box in seen:
                spy.still.setdefault(i, []).append(
                    (round(spy.sim.t - spy.sim.t0, 2), det.side, round(box[3] - box[1]),
                     res is not None and res.get("_truth") == i, (cx, cy, wall_dir)))
            return res
        det.read_wall = read_wall


def audit_run(seed, noise, spec):
    """Повтор заезда со шпионом -> (результат night.score, разбор пропущенных)."""
    import maze_sim as ms
    import fake_hw
    import mazegen
    import live

    nav = ms.nav
    world = mazegen.build_world(spec)
    rng_noise = __import__("random").Random(seed * 7919 + int(noise * 100))
    sim = ms.Sim(world, noise=ms.Noise.level(noise, rng_noise), seed=seed, color=True)
    sim.limit = night.RUN_LIMIT_S
    epoch0 = time.time()
    tee = live.LogTee(io.StringIO(), keep=100000, clock=lambda: epoch0 + (sim.t - sim.t0))
    spy = Spy(sim, fake_hw)
    fake_hw.ms = ms
    sim.hooks.append(spy.on_step)
    status, cx, cy = "?", 0, 0
    with contextlib.redirect_stdout(tee):
        try:
            ms.setup(sim)
            fake_hw.install(sim)
            spy.wrap_read(nav._victim_r)
            spy.wrap_read(nav._victim_l)
            cx, cy, hdg, cp, status = nav.flood_fill_run(300)
            cx, cy, hdg = nav.return_to_start(cx, cy, hdg, cp)
            if (cx, cy) == (0, 0):
                nav.blink_start_return()
        except ms.SimTimeUp:
            status = "time_up"
    res = night.evaluate(spec, sim, nav, status, (cx, cy), tee)
    blinked = {e[4].get("truth") for e in sim.events if e[1] == "blink"}
    reach = mazegen.reachable(spec)
    log = [t for _, _, t in tee.lines]
    out = []
    for i, v in enumerate(spec["victims"]):
        if v["type"] not in night.KITS_NEED or tuple(v["cell"]) not in reach or i in blinked:
            continue
        fr = spy.frames.get(i, [])
        st = spy.still.get(i, [])
        armed = [f for f in fr if f[3]]
        if any(s[3] for s in st):
            why = "read_no_blink"
        elif st:
            why = "still_filtered"
        elif armed:
            why = "watch_small"
        elif fr:
            why = "frame_unarmed"
        else:
            why = "never_in_frame"
        vx, vy = ms.Victim(tuple(v["cell"]), v["d"], v["type"], v["along"]).point()
        # где робот был ближе всего к метке (по центру)
        out.append({"seed": seed, "i": i, "victim": v, "why": why,
                    "frames": len(fr), "armed_frames": len(armed),
                    "max_px": max((f[2] for f in fr), default=0),
                    "max_px_armed": max((f[2] for f in armed), default=0),
                    "still": st[:6], "first_frames": fr[:4],
                    "w_lines": [t.strip() for t in log if "[W]" in t][:0],
                    "point": [round(vx), round(vy)]})
    return res, out


def main():
    ap = argparse.ArgumentParser(description="Почему пропущены метки")
    ap.add_argument("--dir", help="папка ночного прогона (results.json + seed-N.spec.json)")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--spec")
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--arm", help="из сравнения (--compare): только заезды этого варианта (A/B)")
    ap.add_argument("--one", action="store_true", help="служебное: один заезд, JSON в stdout")
    a = ap.parse_args()

    if a.seed is not None:
        spec = json.load(open(a.spec)) if a.spec else __import__("mazegen").generate(a.seed)
        res, out = audit_run(a.seed, a.noise, spec)
        print(json.dumps({"res": res, "missed": out}, ensure_ascii=False, default=str))
        return

    import subprocess
    from concurrent.futures import ThreadPoolExecutor
    results = json.load(open(os.path.join(a.dir, "results.json")))
    todo = sorted({r["seed"] for r in results if r["noise"] == a.noise
                   and (a.arm is None or r.get("arm") == a.arm) and r.get("missed")})

    def go(seed):
        p = subprocess.run([sys.executable, os.path.abspath(__file__), "--seed", str(seed),
                            "--noise", str(a.noise),
                            "--spec", os.path.join(a.dir, f"seed-{seed}.spec.json")],
                           capture_output=True, text=True)
        try:
            return json.loads(p.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            return {"res": {"seed": seed}, "missed": [], "err": p.stderr[-2000:]}

    with ThreadPoolExecutor(max_workers=max(1, (os.cpu_count() or 2) - 1)) as ex:
        allres = list(ex.map(go, todo))
    from collections import Counter
    cnt = Counter()
    for r in allres:
        if r.get("err"):
            print("ОШИБКА", r["res"]["seed"], r["err"][-500:])
        for m in r["missed"]:
            cnt[m["why"]] += 1
            v = m["victim"]
            print(f"seed {m['seed']:4d} метка {m['i']} {v['type']:9s} клетка {tuple(v['cell'])} "
                  f"стена {'NESW'[v['d']]} вдоль {v['along']:+4d}: {m['why']:15s} "
                  f"кадров {m['frames']:3d} (вкл {m['armed_frames']:3d}) "
                  f"рамка до {m['max_px']}px (вкл {m['max_px_armed']}px) стоя {m['still'][:3]}")
    print("Итого:", dict(cnt))


if __name__ == "__main__":
    main()
