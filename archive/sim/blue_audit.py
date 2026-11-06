"""Разбор повторных заездов на синие плитки: какие были неизбежны, какие нет.

    python3 sim/blue_audit.py -n 48 --seed 8000 --noise 0,1
    python3 sim/blue_audit.py --seed 8033 --noise 1 --one      # один заезд, JSON

Регламент (5.6.6): за синюю 30 очков на первом визите, минус 10 за каждый
повторный, не ниже 0; синяя с остатком > 0 ещё даёт 10 в бонусе за выход и
10 в надёжности. Визит - больше половины робота на плитке (5.4.4).

Нижняя граница по настоящему полю. Синяя B - точка сочленения графа клеток
(без чёрных и клеток с препятствием): без неё поле распадается, и k частей,
где есть что посетить, лежат по ту сторону B от старта. Тогда, чтобы обойти
всё и вернуться, на B заезжают не меньше k + 1 раз - k повторов неизбежны.
Если B не сочленение (или за ней ничего) - хватает одного визита.

Заезд повторяется как night.run_one (свежий процесс на заезд), и на каждом
такте запоминается фаза nav.py - для каждого лишнего визита видно, где он
случился: разведка (exploring), путь домой (returning/time) и т.д., какая
клетка была целью и сколько клеток поля ещё не открыто.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import random
import subprocess
import sys
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import night  # noqa: E402


def graph(spec):
    """Клетки, по которым ездят (не чёрные, без препятствий), и соседство."""
    import mazegen
    walls = {tuple(e) for e in spec["walls"]}
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    blocked = mazegen.blocked_cells(spec)
    inside = {(x, y) for x in range(spec["w"]) for y in range(spec["h"])
              if tiles.get((x, y)) != "black" and (x, y) not in blocked}
    adj = {c: [n for _, n in mazegen._neighbors(c, walls, inside)] for c in inside}
    return adj, tiles


def min_visits(spec):
    """{синяя: сколько визитов неизбежно} - см. шапку."""
    import mazegen
    adj, tiles = graph(spec)
    reach = mazegen.reachable(spec)
    out = {}
    for b, t in tiles.items():
        if t != "blue" or b not in reach:
            continue
        seen = {b}
        parts = 0
        for s in adj[b]:
            if s in seen:
                continue
            comp, todo = {s}, [s]
            while todo:
                c = todo.pop()
                for n in adj[c]:
                    if n not in seen and n not in comp:
                        comp.add(n)
                        todo.append(n)
            seen |= comp
            if (0, 0) not in comp and comp & reach:
                parts += 1
        out[b] = parts + 1 if b != (0, 0) else max(1, parts)
    return out


def audit_run(seed, noise, spec):
    import maze_sim as ms
    import fake_hw
    import mazegen
    import live

    nav = ms.nav
    world = mazegen.build_world(spec)
    rng_noise = random.Random(seed * 7919 + int(noise * 100))
    sim = ms.Sim(world, noise=ms.Noise.level(noise, rng_noise), seed=seed, color=True)
    sim.limit = night.RUN_LIMIT_S
    epoch0 = time.time()
    tee = live.LogTee(io.StringIO(), keep=100000, clock=lambda: epoch0 + (sim.t - sim.t0))
    phases = []                 # (t, фаза) при каждой смене

    def on_step(s):
        ph = nav._live.get("phase")
        if not phases or phases[-1][1] != ph:
            phases.append((round(s.t - s.t0, 3), ph))
    sim.hooks.append(on_step)
    status, cx, cy = "?", 0, 0
    with contextlib.redirect_stdout(tee):
        try:
            ms.setup(sim)
            fake_hw.install(sim)
            nav._t_run = nav.time.monotonic()
            cx, cy, hdg, cp, status = nav.flood_fill_run(300)
            if status != "fatal":
                cx, cy, hdg = nav.return_to_start(cx, cy, hdg, cp)
                if (cx, cy) == (0, 0):
                    nav.blink_start_return()
        except ms.SimTimeUp:
            status = "time_up"
    res = night.evaluate(spec, sim, nav, status, (cx, cy), tee)

    def phase_at(t):
        ph = None
        for tp, p in phases:
            if tp > t:
                break
            ph = p
        return ph

    need = min_visits(spec)
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    n_by = Counter()
    extra = []
    for v in sim.visits:
        c = tuple(v["cell"])
        if tiles.get(c) != "blue":
            continue
        n_by[c] += 1
        if n_by[c] > need.get(c, 1):
            extra.append({"cell": list(c), "n": n_by[c], "need": need.get(c, 1),
                          "t": v["t"], "phase": phase_at(v["t"])})
    blues = [{"cell": list(c), "visits": n_by[c], "need": need[c]} for c in need]
    # лог вокруг лишних визитов - чтобы видеть, зачем робот туда поехал
    lines = [(ts - epoch0, txt) for _, ts, txt in tee.lines]
    for e in extra:
        e["log"] = [f"{t:6.1f} {txt.strip()}" for t, txt in lines
                    if e["t"] - 25 <= t <= e["t"] + 3 and txt.strip()][-25:]
    return {"seed": seed, "noise": noise, "score": res["score"], "max": res["max_score"],
            "blue_repeat": res["blue_repeat"], "blue_pts": res["parts"]["blue"],
            "sbv": res["sbv"], "home": res["home"], "status": status,
            "blues": blues, "extra": extra,
            "forced": sum(max(0, min(n_by[c], need[c]) - 1) for c in need)}


def lost_points(r):
    """Очки, потерянные на синих против лучшего возможного при тех же
    неизбежных повторах (30 - 10 за повтор; 4-й визит гасит и бонус, и
    надёжность: ещё 20)."""
    def val(n):
        if n == 0:
            return 0
        p = max(0, 30 - 10 * (n - 1))
        return p + (20 if p > 0 else 0)
    return sum(val(min(b["visits"], b["need"])) - val(b["visits"])
               for b in r["blues"] if b["visits"] > 0)


def main():
    ap = argparse.ArgumentParser(description="Повторные синие: неизбежные и лишние")
    ap.add_argument("-n", type=int, default=32)
    ap.add_argument("--seed", type=int, default=8000)
    ap.add_argument("--noise", default="0")
    ap.add_argument("-j", "--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--one", action="store_true", help="один заезд в этом процессе, JSON в stdout")
    ap.add_argument("--out", help="куда сложить results (jsonl)")
    a = ap.parse_args()

    if a.one:
        import mazegen
        r = audit_run(a.seed, float(a.noise), mazegen.generate(a.seed))
        print("BLUE_AUDIT " + json.dumps(r, ensure_ascii=False))
        return

    noises = [float(x) for x in a.noise.split(",")]
    jobs = [(s, nz) for s in range(a.seed, a.seed + a.n) for nz in noises]

    def go(job):
        s, nz = job
        p = subprocess.run([sys.executable, __file__, "--one", "--seed", str(s), "--noise", str(nz)],
                           capture_output=True, text=True, timeout=night.WALL_TIMEOUT_S)
        for ln in p.stdout.splitlines():
            if ln.startswith("BLUE_AUDIT "):
                return json.loads(ln[len("BLUE_AUDIT "):])
        return {"seed": s, "noise": nz, "error": (p.stderr or p.stdout)[-400:]}

    t0 = time.monotonic()
    with ThreadPoolExecutor(a.jobs) as ex:
        rs = list(ex.map(go, jobs))
    bad = [r for r in rs if "error" in r]
    rs = [r for r in rs if "error" not in r]
    if a.out:
        with open(a.out, "w") as f:
            for r in rs:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")

    n = len(rs)
    rep = sum(r["blue_repeat"] for r in rs)
    forced = sum(r["forced"] for r in rs)
    ex_ = [e for r in rs for e in r["extra"]]
    lost = sum(lost_points(r) for r in rs)
    print(f"заездов {n} ({time.monotonic() - t0:.0f} с), упало {len(bad)}")
    print(f"повторов на синих: {rep} ({rep / max(1, n):.2f} на заезд); неизбежных по полю {forced}, "
          f"лишних {len(ex_)} ({len(ex_) / max(1, n):.2f} на заезд)")
    print(f"очков потеряно на лишних: {lost} ({lost / max(1, n):.1f} на заезд)")
    print("лишние по фазам:", dict(Counter(e["phase"] for e in ex_).most_common()))
    worst = sorted(rs, key=lambda r: -len(r["extra"]))[:12]
    for r in worst:
        if r["extra"]:
            print(f"  поле {r['seed']} шум {r['noise']:g}: лишних {len(r['extra'])}, "
                  + "; ".join(f"{tuple(e['cell'])} #{e['n']}/{e['need']} {e['phase']} t={e['t']:.0f}"
                              for e in r["extra"][:5]))
    for r in bad[:5]:
        print("  ошибка:", r["seed"], r["noise"], r["error"][-200:])


if __name__ == "__main__":
    main()
