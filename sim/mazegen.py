"""Случайные лабиринты для ночных прогонов (sim/night.py).

Лабиринт задаётся спецификацией - простым JSON (размер, стены, плитки,
метки, бугорки, этаж, горки, зона, препятствия), а мир модели собирается из
неё: так любой заезд можно повторить по номеру/файлу, не храня сам мир.

Что похоже на поле RoboCup Rescue Maze 2026 (docs/rules/RCJRescueMaze2026-draft.txt):
  - сетка W x H клеток 300 мм, старт - клетка (0, 0) в углу, курс на север;
  - стены - лабиринт поиском в глубину по каждой области, потом часть стен
    внутри области убрана: появляются петли и открытые площадки;
  - второй этаж (3.1.5, 3.4.6-8): прямоугольный островок у края поля на
    RISE мм выше, к нему - горка в 1 плитку (25°) или 2 (13°), иногда вторая
    горка или лестница (7 ступеней по 20 мм, 2 плитки); вокруг островка и
    горок - стены, проход только по горке;
  - опасная зона (3.5): карман у края поля, обнесён стенами, вход через
    красную плитку; внутри бугорки до 2 см и мусор до 1 см (закреплён -
    модель его не сдвигает); остальное поле проходится без неё;
  - чёрные плитки (ямы), синие (стоянка), серебряные (чекпоинты) - не на
    старте, не у самого старта, не у горок; чёрная не отрезает часть поля;
  - лежачие полицейские - поперёк прохода, до 1 см (3.4.1), на белых клетках;
  - препятствия (3.4.3-4) не ниже 15 см: у стены и не дальше 10 см от неё в
    клетку, или посреди открытой площадки в 20 см от любых стен;
  - метки на стенах: буквы Φ/Ψ/Ω и когнитивные мишени (kind "letter"/"cog"),
    иногда ложные (type "fake"); не больше одной на стену и не на стенах,
    выходящих на цветную плитку, плитку с бугорком/препятствием/мусором,
    горку или лестницу (3.6.3).
Мосты (плитка над плиткой, 3.2.7) модель рельефа не умеет - этаж стоит
рядом с нижним, а не над ним.
"""
from __future__ import annotations

import random

import maze_sim as ms

GEN_VERSION = 2
DIRS = ((0, 1), (1, 0), (0, -1), (-1, 0))     # N E S W
RISE = 140.0               # высота этажа: горка в 1 плитку - 25°
STAIR_STEPS = 8            # площадок лестницы: 7 ступеней по RISE/7 = 20 мм
OBST_H = 200.0             # высота препятствия
FEATURES = ("floor2", "stairs", "zone", "obstacles")


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


def _step(c, d, k=1):
    return (c[0] + DIRS[d][0] * k, c[1] + DIRS[d][1] * k)


def _connected_set(cells):
    """Связно ли множество клеток по соседству (без учёта стен)."""
    cells = set(cells)
    if not cells:
        return True
    first = next(iter(cells))
    seen, todo = {first}, [first]
    while todo:
        c = todo.pop()
        for d in range(4):
            n = _step(c, d)
            if n in cells and n not in seen:
                seen.add(n)
                todo.append(n)
    return len(seen) == len(cells)


def _rect(x0, y0, rw, rh):
    return {(x, y) for x in range(x0, x0 + rw) for y in range(y0, y0 + rh)}


def _place_rect(rng, w, h, sizes, taken, tries=60):
    """Прямоугольник клеток у края поля, не задевающий taken; None - не нашли."""
    for _ in range(tries):
        rw, rh = rng.choice(sizes)
        if rw > w - 1 or rh > h - 1:
            continue
        x0, y0 = rng.randint(0, w - rw), rng.randint(0, h - rh)
        if not (x0 == 0 or y0 == 0 or x0 + rw == w or y0 + rh == h):
            continue
        r = _rect(x0, y0, rw, rh)
        if r & taken:
            continue
        return r
    return None


def _ramp_options(plateau, inside, taken, length):
    """Варианты горки длины length к островку: (bottom, [клетки горки], top, d),
    d - направление подъёма (от подножия к островку)."""
    out = []
    for p in sorted(plateau):
        for d in range(4):
            cells = [_step(p, d, -k) for k in range(length, 0, -1)]
            b = _step(p, d, -(length + 1))
            if all(c in inside and c not in taken for c in cells) and b in inside \
                    and b not in taken:
                out.append((b, cells, p, d))
    return out


def generate(seed, w=None, h=None, braid=None, victims=None, features=None):
    """Спецификация случайного лабиринта (dict, годится для JSON).
    features - что из FEATURES можно ставить (None - всё, по жребию)."""
    rng = random.Random(seed)
    feats = set(FEATURES if features is None else features)
    w = w or rng.randint(4, 7)
    h = h or rng.randint(4, 6)
    braid = rng.uniform(0.15, 0.4) if braid is None else braid
    cells = [(x, y) for x in range(w) for y in range(h)]
    inside = set(cells)
    protected = {(0, 0), (1, 0), (0, 1)}          # старт и его соседи - без особенностей

    # 1. области: островок второго этажа с горками, опасная зона, остальное
    roll_floor2, roll_zone, roll_obst = rng.random(), rng.random(), rng.random()
    high, ramps, zone, entrance = set(), [], set(), None
    taken = set(protected)
    if "floor2" in feats and roll_floor2 < 0.6 and w * h >= 20:
        for _ in range(20):
            plat = _place_rect(rng, w, h, ((2, 2), (2, 3), (3, 2), (3, 3)), taken)
            if plat is None:
                break
            opts = _ramp_options(plat, inside, taken | plat, rng.choice((1, 1, 2)))
            if not opts:
                continue
            b, rc, p, d = rng.choice(opts)
            lower = inside - plat - set(rc)
            if not _connected_set(lower):
                continue
            high = plat
            ramps.append({"kind": "ramp", "cells": [list(c) for c in rc], "d": d,
                          "bottom": list(b), "top": list(p)})
            taken |= plat | set(rc) | {b}
            # вторая дорога наверх: горка или лестница (3.4.8)
            if rng.random() < 0.4:
                kind = "stairs" if "stairs" in feats and rng.random() < 0.6 else "ramp"
                opts2 = [o for o in _ramp_options(plat, inside, taken, 2 if kind == "stairs" else 1)
                         if o[0] not in taken and _connected_set(lower - set(o[1]))]
                if opts2:
                    b2, rc2, p2, d2 = rng.choice(opts2)
                    ramps.append({"kind": kind, "cells": [list(c) for c in rc2], "d": d2,
                                  "bottom": list(b2), "top": list(p2)})
                    taken |= set(rc2) | {b2}
            break
    ramp_cells = {tuple(c) for r in ramps for c in r["cells"]}
    if "zone" in feats and roll_zone < 0.5:
        for _ in range(30):
            z = _place_rect(rng, w, h, ((1, 2), (2, 1), (2, 2), (1, 3), (3, 1)), taken)
            if z is None:
                break
            lower = inside - high - ramp_cells - z
            if not _connected_set(lower):
                continue
            doors = [(o, c) for c in sorted(z) for d in range(4)
                     for o in [_step(c, d)] if o in lower and o not in protected]
            if not doors:
                continue
            entrance = rng.choice(doors)
            zone = z
            taken |= z
            break
    lower = inside - high - ramp_cells - zone

    # 2. лабиринт в каждой области: DFS - все клетки области связаны
    walls = {_edge(c, d) for c in cells for d in range(4)}
    region_of = {}
    for name, reg in (("low", lower), ("high", high), ("zone", zone)):
        for c in reg:
            region_of[c] = name
    for reg in (lower, high, zone):
        if not reg:
            continue
        start = (0, 0) if (0, 0) in reg else min(reg)
        seen, stack = {start}, [start]
        while stack:
            c = stack[-1]
            nb = [(d, _step(c, d)) for d in range(4)
                  if _step(c, d) in reg and _step(c, d) not in seen]
            if not nb:
                stack.pop()
                continue
            d, n = rng.choice(nb)
            walls.discard(_edge(c, d))
            seen.add(n)
            stack.append(n)
    # петли и площадки: убрать часть стен внутри областей
    # sorted: порядок множества кортежей со строками зависит от хеширования
    # строк в процессе (PYTHONHASHSEED) - без него один номер давал разные поля
    for reg, frac in ((lower, braid), (high, braid), (zone, 0.5)):
        inner = sorted(e for e in walls if _inner_of(e, reg))
        rng.shuffle(inner)
        for e in inner[:int(len(inner) * frac)]:
            walls.discard(e)
    # проходы между областями: подножие - горка - островок, вход в зону
    for r in ramps:
        seq = [tuple(r["bottom"])] + [tuple(c) for c in r["cells"]] + [tuple(r["top"])]
        for a, b in zip(seq, seq[1:]):
            walls.discard(_edge(a, r["d"]))
    if entrance is not None:
        o, e = entrance
        d = next(d for d in range(4) if _step(o, d) == e)
        walls.discard(_edge(o, d))

    # 3. плитки - не на старте и рядом, не у горок; красная - вход в зону
    near_ramp = {tuple(r["bottom"]) for r in ramps} | {tuple(r["top"]) for r in ramps}
    free = [c for c in cells if c not in protected and c not in near_ramp
            and c not in ramp_cells and c not in zone]
    rng.shuffle(free)
    tiles = {}
    if entrance is not None:
        tiles[entrance[1]] = "red"
    n_black = rng.randint(0, max(1, len(cells) // 12))
    for c in free:
        if n_black == 0:
            break
        if entrance is not None and c == entrance[0]:
            continue                  # перед входом в зону - не яма
        tiles[c] = "black"
        if not _connected(cells, walls, tiles):
            del tiles[c]              # чёрная не должна отрезать часть поля
            continue
        n_black -= 1
    rest = [c for c in free if c not in tiles]
    for kind, k in (("blue", rng.randint(1, 3)), ("silver", rng.randint(1, 3))):
        for c in rest[:k]:
            tiles[c] = kind
        rest = rest[k:]

    # 4. лежачие полицейские: на белой клетке, поперёк хода, где-то внутри
    # плитки (не на её краю), до 1 см; в зоне - до 2 см (3.5.7)
    bumps, debris = [], []
    plain = [c for c in cells if c not in tiles and c not in protected
             and c not in ramp_cells and c not in near_ramp and c not in zone]
    rng.shuffle(plain)
    for c in plain[:rng.randint(0, 3)]:
        bumps.append({"cell": list(c), "axis": rng.choice(("ns", "ew")),
                      "off": round(rng.uniform(-90, 90)), "h": 10})
    for c in sorted(zone):
        if c in tiles:
            continue
        u = rng.random()
        if u < 0.4:
            bumps.append({"cell": list(c), "axis": rng.choice(("ns", "ew")),
                          "off": round(rng.uniform(-90, 90)), "h": rng.choice((10, 15, 20))})
        elif u < 0.7:                 # мусор до 1 см (3.5.5): пара кубиков
            for _ in range(rng.randint(1, 3)):
                debris.append({"cell": list(c), "x": round(rng.uniform(-100, 100)),
                               "y": round(rng.uniform(-100, 100)), "s": 20, "h": 10})
    bump_cells = {tuple(b["cell"]) for b in bumps} | {tuple(b["cell"]) for b in debris}

    # 5. препятствия: у стены (не дальше 10 см в клетку) или посреди площадки
    obstacles = []
    if "obstacles" in feats and roll_obst < 0.5:
        cand = [c for c in cells if c not in tiles and c not in protected and c not in ramp_cells
                and c not in near_ramp and c not in bump_cells and c not in zone]
        rng.shuffle(cand)
        want = rng.randint(1, 2)
        for c in cand:
            if len(obstacles) >= want:
                break
            o = _obstacle(rng, c, walls, obstacles)
            if o is not None:
                obstacles.append(o)
    obst_cells = {tuple(o["cell"]) for o in obstacles}

    # 6. метки: стены со стороны клетки, одна на стену; не напротив цветной
    # плитки, бугорка, препятствия, горки (регламент 3.6.3)
    faces = [(c, d) for c in cells if c not in tiles and c not in bump_cells
             and c not in obst_cells and c not in ramp_cells
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
        kind = rng.choice(("letter", "cog"))
        vics.append({"cell": list(c), "d": d, "type": t, "along": round(rng.uniform(-90, 90)),
                     "kind": kind})

    return {
        "gen": GEN_VERSION, "seed": seed, "w": w, "h": h,
        "walls": sorted([list(e) for e in walls]),
        "tiles": [[c[0], c[1], t] for c, t in sorted(tiles.items())],
        "victims": vics, "bumps": bumps,
        "high": sorted([list(c) for c in high]), "rise": RISE if high else 0.0,
        "ramps": ramps, "zone": sorted([list(c) for c in zone]),
        "debris": debris, "obstacles": obstacles,
    }


def _obstacle(rng, c, walls, others):
    """Препятствие в клетке c (3.4.4) или None: у стены - вглубь клетки не
    дальше 100 мм от линии стены, посреди - только в открытой площадке, где
    до любых стен 200 мм."""
    x, y = c[0] * ms.CELL, c[1] * ms.CELL
    sides = [d for d in range(4) if _edge(c, d) in walls]
    corners_free = not sides and all(_edge(_step(c, a), b) not in walls
                                     for a in range(4) for b in ((a + 1) % 4, (a + 3) % 4))
    if corners_free and rng.random() < 0.5:
        s = rng.uniform(50, 70) / 2
        ox, oy = x + rng.uniform(-10, 10), y + rng.uniform(-10, 10)
        rect = [ox - s, oy - s, ox + s, oy + s]
        kind = "center"
    elif sides:
        d = rng.choice(sides)
        depth = rng.uniform(30, 90)              # от лицевой стороны стены
        width = rng.uniform(60, 150)
        along = rng.uniform(-150 + 10 + width / 2, 150 - 10 - width / 2)
        face = ms.CELL / 2 - ms.WALL_T / 2
        dx, dy = DIRS[d]
        if dx == 0:                   # стена N/S: вдоль x
            y0 = y + dy * face
            y1 = y0 - dy * depth
            rect = [x + along - width / 2, min(y0, y1), x + along + width / 2, max(y0, y1)]
        else:
            x0 = x + dx * face
            x1 = x0 - dx * depth
            rect = [min(x0, x1), y + along - width / 2, max(x0, x1), y + along + width / 2]
        kind = "wall"
    else:
        return None
    rect = [round(v) for v in rect]
    for o in others:                  # друг от друга - 20 см (3.4.4b)
        a, b = o["rect"], rect
        gap = max(a[0] - b[2], b[0] - a[2], a[1] - b[3], b[1] - a[3])
        if gap < 200:
            return None
    return {"cell": list(c), "kind": kind, "rect": rect, "h": OBST_H}


def _inner_of(e, reg):
    """Ребро e - между двумя клетками области reg."""
    x, y, o = e
    a = (x, y)
    b = (x, y + 1) if o == "h" else (x + 1, y)
    return a in reg and b in reg


def _neighbors(c, walls, inside):
    for d in range(4):
        n = _step(c, d)
        if n in inside and _edge(c, d) not in walls:
            yield d, n


def _connected(cells, walls, tiles):
    """Все не чёрные клетки достижимы со старта."""
    ok = {c for c in cells if tiles.get(c) != "black"}
    seen, todo = {(0, 0)}, [(0, 0)]
    while todo:
        c = todo.pop()
        for _, n in _neighbors(c, walls, ok):
            if n not in seen:
                seen.add(n)
                todo.append(n)
    return len(seen) == len(ok)


def ramp_cells(spec):
    return {tuple(c) for r in spec.get("ramps", ()) for c in r["cells"]}


def blocked_cells(spec):
    """Клетки, куда робот не встанет: препятствие посреди площадки."""
    return {tuple(o["cell"]) for o in spec.get("obstacles", ()) if o["kind"] == "center"}


def reachable(spec):
    """Клетки, куда можно доехать со старта (без чёрных, без плиток горок и
    клеток с препятствием посередине - по ним не встают)."""
    walls = {tuple(e) for e in spec["walls"]}
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    cells = [(x, y) for x in range(spec["w"]) for y in range(spec["h"])]
    blocked = blocked_cells(spec)
    inside = {c for c in cells if tiles.get(c) != "black" and c not in blocked}
    seen, todo = {(0, 0)}, [(0, 0)]
    while todo:
        c = todo.pop()
        for _, n in _neighbors(c, walls, inside):
            if n not in seen:
                seen.add(n)
                todo.append(n)
    return seen - ramp_cells(spec)


def has_wall(spec_walls, c, d):
    return _edge(c, d) in spec_walls


def linear_tiles(spec):
    """Линейные плитки (3.3.1-2): те, по которым ведёт к старту обход по
    левой или правой стене; чёрные - виртуальные стены.

    Обход по стене идёт вдоль связной группы стен (стены соединены
    столбиками в углах плиток) и обходит её со всех свободных сторон: от
    стены к стене через общий столбик, вокруг конца стены. Поэтому
    линейные - плитки, касающиеся (стороной или углом) столбика той группы
    стен, что держит стартовую плитку. Плитки посреди площадки и у
    "островных" стен, не связанных со стенами старта, - плавающие."""
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    parent = {}

    def find(p):
        while parent.setdefault(p, p) != p:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p

    def union(a, b):
        parent[find(a)] = find(b)

    def posts(x, y, o):              # столбики на концах ребра (x, y, o)
        if o == "h":
            return (x, y + 1), (x + 1, y + 1)
        return (x + 1, y), (x + 1, y + 1)

    segs = [tuple(e) for e in spec["walls"]]
    for (x, y), t in tiles.items():
        if t == "black":
            segs += [_edge((x, y), d) for d in range(4)]
    with_wall = set()
    for e in segs:
        a, b = posts(*e)
        union(a, b)
        with_wall |= {a, b}

    def corners(c):
        x, y = c
        return [(x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)]

    roots = {find(p) for p in corners((0, 0)) if p in with_wall}
    return {(x, y) for x in range(spec["w"]) for y in range(spec["h"])
            if tiles.get((x, y)) != "black"
            and any(p in with_wall and find(p) in roots for p in corners((x, y)))}


def level_of_cell(spec, c):
    return 1 if [c[0], c[1]] in spec.get("high", ()) else 0


def nav_cell(spec, c):
    """Клетка мира -> клетка nav.py: этаж 1 - y + LEVEL_STRIDE (nav.record_ramp)."""
    return (c[0], c[1] + 1000 * level_of_cell(spec, c))


def build_world(spec):
    """Мир модели по спецификации."""
    w = ms.World()
    high = {tuple(c) for c in spec.get("high", ())}
    rise = float(spec.get("rise", 0.0))
    rcells = ramp_cells(spec)
    top = ms.WALL_H + rise

    def zr(e):
        """Высота стены на ребре: над этажом - от его пола, у горок и
        между этажами - от нижнего пола до верха верхних стен."""
        x, y, o = e
        a, b = (x, y), ((x, y + 1) if o == "h" else (x + 1, y))
        if a in rcells or b in rcells:
            return 0.0, top
        ha, hb = a in high, b in high
        if ha and hb:
            return rise, top
        if ha or hb:
            inside_a = 0 <= a[0] < spec["w"] and 0 <= a[1] < spec["h"]
            inside_b = 0 <= b[0] < spec["w"] and 0 <= b[1] < spec["h"]
            if not (inside_a and inside_b):
                return rise, top      # внешняя стена островка
            return 0.0, top
        return 0.0, ms.WALL_H

    for x, y, o in spec["walls"]:
        z0, z1 = zr((x, y, o))
        if o == "h":
            w.wall(x, y, 0, z0, z1)          # северная стена (x, y)
        else:
            w.wall(x, y, 1, z0, z1)          # восточная стена (x, y)
    for x, y, t in spec["tiles"]:
        w.tiles[(x, y)] = t
    for v in spec["victims"]:
        w.victims.append(ms.Victim(tuple(v["cell"]), v["d"], v["type"], float(v["along"])))
    half = ms.CELL / 2
    for cx, cy in high:
        x, y = cx * ms.CELL, cy * ms.CELL
        w.pieces.append((x - half, y - half, x + half, y + half, lambda px, py, h=rise: h))
    for r in spec.get("ramps", ()):
        w.ramps.append(r)
        cs = [tuple(c) for c in r["cells"]]
        xs = [c[0] * ms.CELL for c in cs]
        ys = [c[1] * ms.CELL for c in cs]
        x0, x1, y0, y1 = min(xs) - half, max(xs) + half, min(ys) - half, max(ys) + half
        dx, dy = DIRS[r["d"]]
        length = len(cs) * ms.CELL
        # начало подъёма - край первой плитки со стороны подножия
        sx = x0 if dx > 0 else x1 if dx < 0 else 0.0
        sy = y0 if dy > 0 else y1 if dy < 0 else 0.0
        if r["kind"] == "stairs":
            def f(px, py, sx=sx, sy=sy, dx=dx, dy=dy, L=length):
                p = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / L))
                return rise * min(STAIR_STEPS - 1, int(p * STAIR_STEPS)) / (STAIR_STEPS - 1)
        else:
            def f(px, py, sx=sx, sy=sy, dx=dx, dy=dy, L=length):
                p = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / L))
                return rise * p
        w.pieces.append((x0, y0, x1, y1, f))
    for b in spec["bumps"]:
        (x, y), h = b["cell"], float(b.get("h", 10))
        base = rise if (x, y) in high else 0.0
        if b["axis"] == "ns":        # ход север-юг: бугорок - полоса вдоль x
            yc = y * ms.CELL + b["off"]
            rect = (x * ms.CELL - half, yc - 15.0, x * ms.CELL + half, yc + 15.0)
        else:
            xc = x * ms.CELL + b["off"]
            rect = (xc - 15.0, y * ms.CELL - half, xc + 15.0, y * ms.CELL + half)
        w.pieces.append((*rect, lambda px, py, h=base + h: h))
        w.bumps.append({"cell": (x, y), "axis": b["axis"],
                        "pos": (yc if b["axis"] == "ns" else xc)})
    for d in spec.get("debris", ()):
        x, y = d["cell"][0] * ms.CELL + d["x"], d["cell"][1] * ms.CELL + d["y"]
        s = d["s"] / 2
        w.pieces.append((x - s, y - s, x + s, y + s, lambda px, py, h=float(d["h"]): h))
    for o in spec.get("obstacles", ()):
        z0 = rise if tuple(o["cell"]) in high else 0.0
        x0, y0, x1, y1 = o["rect"]
        w.walls.append((x0, y0, x1, y1, z0, z0 + o["h"]))
    return w


def world_json(spec):
    """Правда мира для записи пульта: стены рёбрами (как карта nav: клетка ->
    [N,E,S,W]), плитки, метки - в клетках nav.py (этаж 1 - y + 1000), чтобы в
    повторе показать, как было на самом деле."""
    walls = {tuple(e) for e in spec["walls"]}
    rc = ramp_cells(spec)
    maze = {}
    for x in range(spec["w"]):
        for y in range(spec["h"]):
            if (x, y) in rc:
                continue
            nx, ny = nav_cell(spec, (x, y))
            maze[f"{nx},{ny}"] = [_edge((x, y), d) in walls for d in range(4)]
    tiles = [[*nav_cell(spec, (x, y)), t] for x, y, t in spec["tiles"]]
    vics = [{**v, "cell": list(nav_cell(spec, tuple(v["cell"])))} for v in spec["victims"]]
    return {"maze": maze, "tiles": tiles, "victims": vics, "w": spec["w"], "h": spec["h"],
            "ramps": spec.get("ramps", []), "obstacles": spec.get("obstacles", []),
            "zone": spec.get("zone", [])}
