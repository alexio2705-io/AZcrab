"""Случайные лабиринты для ночных прогонов (sim/night.py).

Лабиринт задаётся спецификацией - простым JSON (размер, стены, плитки,
метки, бугорки), а мир модели собирается из неё: так любой заезд можно
повторить по номеру/файлу, не храня сам мир.

Что похоже на поле RoboCup Rescue Maze:
  - сетка W x H клеток 300 мм, старт - клетка (0, 0) в углу, курс на север;
  - стены - лабиринт поиском в глубину, потом часть стен убрана: появляются
    петли и открытые площадки (на настоящем поле их много);
  - чёрные плитки (ямы), синие (стоянка), серебряные (чекпоинты) - не на
    старте и не у самого старта;
  - лежачие полицейские - на плитке, поперёк прохода, до 1 см (регламент
    RCJ Maze 2026, 3.4.1), только на белых клетках;
  - метки на стенах: Φ/Ψ/Ω и иногда ложная цель, не больше одной на стену и
    не на стенах, выходящих на цветную плитку или плитку с бугорком (3.6.3).
Горки и второй этаж генератор пока не делает - для них есть ручные
сценарии в run_scenarios.py.
"""
from __future__ import annotations

import random

import maze_sim as ms

DIRS = ((0, 1), (1, 0), (0, -1), (-1, 0))     # N E S W


def _edge(c, d):
    """Ребро клетки c в сторону d - одним ключом для обеих соседних клеток."""
    x, y = c
    if d == 0:
        return (x, y, "h")           # граница между (x,y) и (x,y+1)
    if d == 2:
        return (x, y - 1, "h")
    if d == 1:
        return (x, y, "v")           # граница между (x,y) и (x+1,y)
    return (x - 1, y, "v")


def generate(seed, w=None, h=None, braid=None, victims=None):
    """Спецификация случайного лабиринта (dict, годится для JSON)."""
    rng = random.Random(seed)
    w = w or rng.randint(4, 7)
    h = h or rng.randint(4, 6)
    braid = rng.uniform(0.15, 0.4) if braid is None else braid
    cells = [(x, y) for x in range(w) for y in range(h)]
    inside = set(cells)

    # 1. лабиринт: DFS - все клетки связаны, стены - всё, что не прорыто
    walls = set()
    for c in cells:
        for d in range(4):
            walls.add(_edge(c, d))
    seen, stack = {(0, 0)}, [(0, 0)]
    while stack:
        c = stack[-1]
        nb = [(d, (c[0] + dx, c[1] + dy)) for d, (dx, dy) in enumerate(DIRS)
              if (c[0] + dx, c[1] + dy) in inside and (c[0] + dx, c[1] + dy) not in seen]
        if not nb:
            stack.pop()
            continue
        d, n = rng.choice(nb)
        walls.discard(_edge(c, d))
        seen.add(n)
        stack.append(n)
    # 2. петли и площадки: убрать часть внутренних стен
    # sorted: порядок множества кортежей со строками зависит от хеширования
    # строк в процессе (PYTHONHASHSEED) - без него один номер давал разные поля
    inner = sorted(e for e in walls if _is_inner(e, w, h))
    rng.shuffle(inner)
    for e in inner[:int(len(inner) * braid)]:
        walls.discard(e)

    # 3. плитки - не на старте и не рядом с ним
    free = [c for c in cells if abs(c[0]) + abs(c[1]) > 1]
    rng.shuffle(free)
    tiles = {}
    n_black = rng.randint(0, max(1, len(cells) // 12))
    for c in free:
        if n_black == 0:
            break
        tiles[c] = "black"
        if not _connected(cells, walls, tiles):
            del tiles[c]          # чёрная не должна отрезать часть поля
            continue
        n_black -= 1
    rest = [c for c in free if c not in tiles]
    for kind, k in (("blue", rng.randint(1, 3)), ("silver", rng.randint(1, 3))):
        for c in rest[:k]:
            tiles[c] = kind
        rest = rest[k:]

    # 4. лежачие полицейские: на белой открытой клетке, поперёк хода, где-то
    # внутри плитки (не на её краю), высота 1 см
    bumps = []
    plain = [c for c in cells if c not in tiles and c != (0, 0)]
    rng.shuffle(plain)
    for c in plain[:rng.randint(0, 3)]:
        axis = rng.choice(("ns", "ew"))       # ход, поперёк которого лежит бугорок
        bumps.append({"cell": list(c), "axis": axis, "off": round(rng.uniform(-90, 90)), "h": 10})
    bump_cells = {tuple(b["cell"]) for b in bumps}

    # 5. метки: стены со стороны клетки, одна на стену; не напротив цветной
    # плитки и плитки с бугорком (регламент 3.6.3)
    faces = [(c, d) for c in cells if c not in tiles and c not in bump_cells
             for d in range(4) if _edge(c, d) in walls]
    rng.shuffle(faces)
    used, vics = set(), []
    n_vic = victims if victims is not None else rng.randint(3, min(9, max(3, len(cells) // 3)))
    for c, d in faces:
        if len(vics) >= n_vic:
            break
        e = _edge(c, d)
        if e in used:
            continue
        used.add(e)
        t = rng.choices(("damaged", "stable", "unharmed", "fake"), (3, 3, 3, 1))[0]
        vics.append({"cell": list(c), "d": d, "type": t, "along": round(rng.uniform(-90, 90))})

    return {
        "seed": seed, "w": w, "h": h,
        "walls": sorted([list(e) for e in walls]),
        "tiles": [[c[0], c[1], t] for c, t in sorted(tiles.items())],
        "victims": vics, "bumps": bumps,
    }


def _is_inner(e, w, h):
    x, y, o = e
    if o == "h":
        return 0 <= y < h - 1 and 0 <= x < w
    return 0 <= x < w - 1 and 0 <= y < h


def _connected(cells, walls, tiles):
    """Все не чёрные клетки достижимы со старта."""
    ok = [c for c in cells if tiles.get(c) != "black"]
    inside = set(ok)
    seen, todo = {(0, 0)}, [(0, 0)]
    while todo:
        c = todo.pop()
        for d, (dx, dy) in enumerate(DIRS):
            n = (c[0] + dx, c[1] + dy)
            if n in inside and n not in seen and _edge(c, d) not in walls:
                seen.add(n)
                todo.append(n)
    return len(seen) == len(ok)


def reachable(spec):
    """Клетки, куда можно доехать со старта (без чёрных)."""
    walls = {tuple(e) for e in spec["walls"]}
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    cells = [(x, y) for x in range(spec["w"]) for y in range(spec["h"])]
    inside = {c for c in cells if tiles.get(c) != "black"}
    seen, todo = {(0, 0)}, [(0, 0)]
    while todo:
        c = todo.pop()
        for d, (dx, dy) in enumerate(DIRS):
            n = (c[0] + dx, c[1] + dy)
            if n in inside and n not in seen and _edge(c, d) not in walls:
                seen.add(n)
                todo.append(n)
    return seen


def has_wall(spec_walls, c, d):
    return _edge(c, d) in spec_walls


def build_world(spec):
    """Мир модели по спецификации."""
    w = ms.World()
    for x, y, o in spec["walls"]:
        if o == "h":
            w.wall(x, y, 0)          # северная стена (x, y)
        else:
            w.wall(x, y, 1)          # восточная стена (x, y)
    for x, y, t in spec["tiles"]:
        w.tiles[(x, y)] = t
    for v in spec["victims"]:
        w.victims.append(ms.Victim(tuple(v["cell"]), v["d"], v["type"], float(v["along"])))
    for b in spec["bumps"]:
        (x, y), h = b["cell"], float(b.get("h", 10))
        if b["axis"] == "ns":        # ход север-юг: бугорок - полоса вдоль x
            yc = y * ms.CELL + b["off"]
            w.pieces.append((x * ms.CELL - 150.0, yc - 15.0, x * ms.CELL + 150.0, yc + 15.0,
                             lambda px, py, h=h: h))
        else:
            xc = x * ms.CELL + b["off"]
            w.pieces.append((xc - 15.0, y * ms.CELL - 150.0, xc + 15.0, y * ms.CELL + 150.0,
                             lambda px, py, h=h: h))
    return w


def world_json(spec):
    """Правда мира для записи пульта: стены рёбрами (как карта nav: клетка ->
    [N,E,S,W]), плитки, метки - чтобы в повторе показать, как было на самом деле."""
    walls = {tuple(e) for e in spec["walls"]}
    maze = {}
    for x in range(spec["w"]):
        for y in range(spec["h"]):
            maze[f"{x},{y}"] = [_edge((x, y), d) in walls for d in range(4)]
    return {"maze": maze, "tiles": spec["tiles"], "victims": spec["victims"],
            "w": spec["w"], "h": spec["h"]}
