"""Сценарии симуляции nav.py: python3 sim/run_scenarios.py [имя ...]

Каждый сценарий - мир (стены, горки, бугорки), разведка flood_fill_run и
возврат return_to_start настоящего nav.py. Полный лог - в sim/logs/<имя>.log,
в консоль - итог: где робот физически, упирался ли в стены, совпадает ли
карта нижнего этажа со стенами мира, записаны ли горки.
"""
from __future__ import annotations

import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import maze_sim as ms  # noqa: E402
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


# имя -> (мир, потолок клеток, ожидаемые горки {(клетка, курс): клетка на другом конце})
SCENARIOS = {
    "flat": (corridor_world, 20, {}),
    "bumps": (lambda: corridor_world(bumps=True), 20, {}),
    # (0,1) -> горка -> первая клетка наверху (0,3) = этаж 1, y=3
    "ramp": (ramp_world, 20, {(0, 1, 0): (0, 1003)}),
}


def wall_truth(w, cx, cy, d):
    """Есть ли в мире стена на ребре клетки нижнего этажа."""
    x, y = cx * CELL, cy * CELL
    px, py = {0: (x, y + 150), 1: (x + 150, y), 2: (x, y - 150), 3: (x - 150, y)}[d]
    return any(x0 <= px <= x1 and y0 <= py <= y1 for x0, y0, x1, y1, *_ in w.walls)


def run(name):
    make, cap, ramps = SCENARIOS[name]
    w = make()
    sim = ms.Sim(w)
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    path = os.path.join(HERE, "logs", f"{name}.log")
    buf = io.StringIO()
    status = "?"
    try:
        with contextlib.redirect_stdout(buf):
            ms.setup(sim)
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
    return not bad
    print(f"   лог: {path}")


if __name__ == "__main__":
    names = sys.argv[1:] or list(SCENARIOS)
    failed = []
    for n in names:
        # каждый сценарий - с чистым nav: модуль перезагружаем
        import importlib
        importlib.reload(ms.nav)
        ms.nav = nav = sys.modules["nav"]
        globals()["nav"] = nav
        if not run(n):
            failed.append(n)
    print("ВСЕ СЦЕНАРИИ ЧИСТЫЕ" if not failed else f"ОШИБКИ: {failed}")
    sys.exit(1 if failed else 0)
