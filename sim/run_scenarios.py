"""Сценарии симуляции nav.py: python3 sim/run_scenarios.py [имя ...]

Каждый сценарий - мир (стены, горки, бугорки), разведка flood_fill_run и
возврат return_to_start настоящего nav.py. Полный лог - в sim/logs/<имя>.log,
в консоль - итог: где робот физически, упирался ли в стены, совпадает ли
карта нижнего этажа со стенами мира, записаны ли горки.
"""
from __future__ import annotations

import contextlib
import importlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import maze_sim as ms  # noqa: E402
import sim_trace  # noqa: E402
from maze_sim import CELL, WALL_H, World, nav  # noqa: E402

RAMP_RISE = 140.0           # один этаж: плитка 300 мм под 25°


def corridor_world(bumps=False):
    """Коридор на север (0,0)..(0,3), поворот на восток до (2,3), тупик."""
    w = World()
    w.box([(0, 0), (0, 1), (0, 2), (0, 3), (1, 3), (2, 3)])
    if bumps:
        for yc in (300.0 + 60.0, 600.0 - 40.0):
            w.pieces.append((-150.0, yc - 15.0, 150.0, yc + 15.0, lambda x, y: 15.0))
    return w


def ramp_world():
    """(0,0),(0,1) внизу; горка в (0,2) вверх на север; наверху (0,3),(0,4)
    и поворот на восток в (1,4) - тупик."""
    w = World()
    lo = [(0, 0), (0, 1)]
    hi = [(0, 3), (0, 4), (1, 4)]
    top = RAMP_RISE + WALL_H
    # нижний коридор: стены вокруг (0,0),(0,1), вход в горку открыт
    w.wall(0, 0, 2)
    for c in lo:
        w.wall(*c, 1)
        w.wall(*c, 3)
    # горка: боковые стены от пола до верха верхних стен
    w.wall(0, 2, 1, 0.0, top)
    w.wall(0, 2, 3, 0.0, top)
    w.pieces.append((-150.0, 450.0, 150.0, 750.0,
                     lambda x, y: RAMP_RISE * (y - 450.0) / 300.0))
    # остров верхнего этажа и его стены
    for cx, cy in hi:
        x, y = cx * CELL, cy * CELL
        w.pieces.append((x - 150.0, y - 150.0, x + 150.0, y + 150.0,
                         lambda x, y: RAMP_RISE))
    s = set(hi) | {(0, 2)}
    for cx, cy in hi:
        for d, (dx, dy) in enumerate(((0, 1), (1, 0), (0, -1), (-1, 0))):
            if (cx + dx, cy + dy) not in s:
                w.wall(cx, cy, d, RAMP_RISE, top)
    return w


def brick_world():
    """Коридор (0,0)..(0,3), в (0,2) поперёк - кирпич 50 мм: лидар на 130 мм
    его не видит, проехать нельзя."""
    w = World()
    w.box([(0, 0), (0, 1), (0, 2), (0, 3)])
    w.walls.append((-150.0, 560.0, 150.0, 620.0, 0.0, 50.0))
    return w


def bigbump_world():
    """Коридор как corridor_world, бугорок 25 мм у точки решения клетки
    (0,2): нос задирается выше RAMP_MAP_DEG - кандидат в горку."""
    w = corridor_world()
    w.pieces.append((-150.0, 545.0, 150.0, 575.0, lambda x, y: 25.0))
    return w


def two_ramps_world():
    """Две горки разной длины на один островок: A из (0,1) на север в 1
    плитку (25°), B из (2,1) на север в 2 плитки (13°). Островок:
    (0,3),(0,4),(1,3),(1,4),(2,4)."""
    w = World()
    top = RAMP_RISE + WALL_H
    lo = [(0, 0), (1, 0), (2, 0), (0, 1), (2, 1)]
    hi = [(0, 3), (0, 4), (1, 3), (1, 4), (2, 4)]
    entries = {(0, 1, 0), (2, 1, 0)}
    s = set(lo)
    for cx, cy in lo:
        for d, (dx, dy) in enumerate(((0, 1), (1, 0), (0, -1), (-1, 0))):
            if (cx + dx, cy + dy) not in s and (cx, cy, d) not in entries:
                w.wall(cx, cy, d)
    w.pieces.append((-150.0, 450.0, 150.0, 750.0,
                     lambda x, y: RAMP_RISE * (y - 450.0) / 300.0))
    w.pieces.append((450.0, 450.0, 750.0, 1050.0,
                     lambda x, y: RAMP_RISE * (y - 450.0) / 600.0))
    for cx, cy, d in ((0, 2, 1), (0, 2, 3), (2, 2, 1), (2, 2, 3), (2, 3, 1), (2, 3, 3)):
        w.wall(cx, cy, d, 0.0, top)
    for cx, cy in hi:
        x, y = cx * CELL, cy * CELL
        w.pieces.append((x - 150.0, y - 150.0, x + 150.0, y + 150.0,
                         lambda x, y: RAMP_RISE))
    hs = set(hi)
    for cx, cy in hi:
        for d, (dx, dy) in enumerate(((0, 1), (1, 0), (0, -1), (-1, 0))):
            if (cx + dx, cy + dy) not in hs and (cx, cy, d) not in ((0, 3, 2), (2, 4, 2)):
                w.wall(cx, cy, d, RAMP_RISE, top)
    return w


# имя -> (мир, потолок клеток, ожидаемые горки {(клетка, курс): клетка на другом конце})
SCENARIOS = {
    "flat": (corridor_world, 20, {}),
    "bumps": (lambda: corridor_world(bumps=True), 20, {}),
    # (0,1) -> горка -> первая клетка наверху (0,3) = этаж 1, y=3
    "ramp": (ramp_world, 20, {(0, 1, 0): (0, 1003)}),
    "brick": (brick_world, 20, {}),
    "bigbump": (bigbump_world, 20, {}),
    # обе горки - на один этаж: A -> (0,3), B -> (2,4)
    "two_ramps": (two_ramps_world, 30, {(0, 1, 0): (0, 1003), (2, 1, 0): (2, 1004)}),
}


def wall_truth(w, cx, cy, d):
    """Есть ли в мире стена на ребре клетки нижнего этажа."""
    x, y = cx * CELL, cy * CELL
    px, py = {0: (x, y + 150), 1: (x + 150, y), 2: (x, y - 150), 3: (x - 150, y)}[d]
    return any(x0 <= px <= x1 and y0 <= py <= y1 for x0, y0, x1, y1, *_ in w.walls)


def fresh_nav():
    """Каждый сценарий - с чистым nav: модуль перезагружается на месте."""
    global nav
    importlib.reload(ms.nav)
    ms.nav = nav = sys.modules["nav"]


def run(name, trace=False):
    """Прогон сценария; -> (карта без ошибок, трасса для просмотрщика или None)."""
    make, cap, ramps = SCENARIOS[name]
    w = make()
    sim = ms.Sim(w)
    t0 = sim.t
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    path = os.path.join(HERE, "logs", f"{name}.log")
    rec = None
    if trace:
        rec = sim.rec = sim_trace.Recorder(nav, sim)
        buf = sim_trace.TimedLines(lambda: sim.t)
    else:
        buf = io.StringIO()
    status = "?"
    try:
        with contextlib.redirect_stdout(buf):
            ms.setup(sim)
            if rec is not None:
                nav._write_state = lambda cx, cy, hdg, cp, phase: rec.on_state(phase)
            cx, cy, hdg, cp, status = nav.flood_fill_run(cap)
            if status != "fatal":
                cx, cy, hdg = nav.return_to_start(cx, cy, hdg, cp)
    finally:
        with open(path, "w") as f:
            f.write(buf.getvalue())
    bad = []
    for (cx, cy), cell in sorted(nav._maze.items()):
        if nav.level_of(cy) != 0 or (cx, cy) not in nav._visited:
            continue
        for d in range(4):
            if nav._edge_fixed.get(nav._edge_key(cx, cy, d)) is True:
                continue          # закрыто по факту (упёрлись) - стена нарочно
            if cell[d] is not None and cell[d] != wall_truth(w, cx, cy, d):
                bad.append(f"({cx},{cy}){'NESW'[d]}: карта={'стена' if cell[d] else 'проход'}")
    log = buf.getvalue()
    stops = log.count("торможение с")
    straight = log.count("→ прямо")
    print(f"== {name}: статус {status}, робот физически в ({sim.x:.0f},{sim.y:.0f}) мм, "
          f"курс {sim.theta:.0f}°, упёрся {sim.crashes} раз, max наклон {sim.max_pitch:.0f}°, "
          f"время {sim.t - 1000:.0f}с")
    print(f"   клеток посещено {len(nav._visited)}, прямо без остановки {straight}, "
          f"остановок {stops}, горки {sorted(nav._ramps)}")
    print(f"   FATAL/WARN: {[l.strip()[:90] for l in log.splitlines() if 'FATAL' in l or '[WARN]' in l][:6]}")
    for edge, end in ramps.items():
        got = nav._ramps.get(edge)
        if got is None or got[:2] != end:
            bad.append(f"горка {edge}: ждали {end}, в карте {None if got is None else got[:2]}")
    print(f"   ошибок карты: {len(bad)} {bad[:6]}")
    if rec is None:
        return not bad, None
    rec.on_state("end")
    buf.finish()
    meta = {"status": status, "ok": not bad, "map_errors": bad, "sim_time": round(sim.t - t0, 2),
            "visited": len(nav._visited), "crashes": sim.crashes,
            "max_pitch": round(sim.max_pitch, 1), "ramps": sorted(map(list, nav._ramps))}
    return not bad, sim_trace.build_trace(name, w, rec, buf.lines, meta, t0)


if __name__ == "__main__":
    html = "--html" in sys.argv[1:]
    names = [a for a in sys.argv[1:] if not a.startswith("--")] or list(SCENARIOS)
    failed = []
    traces = {}
    for n in names:
        fresh_nav()
        ok, tr = run(n, trace=html)
        if not ok:
            failed.append(n)
        if tr is not None:
            traces[n] = tr
    print("ВСЕ СЦЕНАРИИ ЧИСТЫЕ" if not failed else f"ОШИБКИ: {failed}")
    if html:
        sys.path.insert(0, os.path.join(HERE, "viewer"))
        import build
        print(f"просмотрщик: {build.write_site(os.path.join(HERE, 'viewer'), traces)}")
    sys.exit(1 if failed else 0)
