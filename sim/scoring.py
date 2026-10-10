"""Очки заезда по регламенту RCJ Rescue Maze 2026 (раздел 5.6) - по событиям
модели (maze_sim.Sim.events, Sim.visits) и спецификации поля (mazegen).

Что считается и как (номера пунктов - docs/rules/RCJRescueMaze2026-draft.txt):
  - метка (5.6.1-2): мигание 5 с стоя, когда до метки от корпуса робота не
    больше 15 см. Буква - 5 на линейной плитке / 15 на плавающей,
    когнитивная мишень - 10 / 30 (3.3: линейная - к старту ведёт обход по
    левой или правой стене, чёрные - виртуальные стены). Тип на ленте не
    показывается - спутанный тип метку не отменяет, только комплекты;
  - мигание дальше 15 см от любой настоящей метки (в т.ч. у ложной цели) -
    минус 5 (5.6.14);
  - комплекты (5.6.3-5): комплект в 15 см от опознанной раньше метки;
    одной метке - не больше, чем ей нужно (Φ 2, Ψ 1, Ω 0); 1 - 10 очков,
    2 - 30;
  - синяя (5.6.6): 30 за первое посещение, минус 10 за каждое повторное той
    же плитки, не ниже 0; засчитывается, если хоть раз простоял 5 с;
    ушёл с синей на другую плитку, не простояв 5 с подряд - LoP (5.5.1c);
  - чёрная: посещение (больше половины робота на ней, 5.4.4) - LoP (5.5.1b);
  - чекпоинт 10 (5.6.11), плитка с бугорком 5 (5.6.8), горка 10, лестница
    10 - по одному разу на плитку/горку (5.6.13);
  - горка/лестница пройдена (5.6.9-10): от нижней плитки до верхней (или
    обратно) подряд, и на конечной плитке робот хоть раз целиком;
  - бонус за выход (5.6.12): робот на стартовой плитке и мигнул 10 с -
    10 x SVI + 10 x SBV + 5 x SSN + 5 x SRN;
  - надёжность (5.6.7): 10 x SVI + 10 x SRD + 10 x SBV - 15 x LoP, не меньше 0.
LoP в модели не переносит робота на чекпоинт - он едет дальше, как ехал;
считаются только сами LoP (и их штраф в надёжности).
SBV - число синих плиток, за которые остались очки (> 0).
"""
from __future__ import annotations

import math

import mazegen

KITS_NEED = {"damaged": 2, "stable": 1, "unharmed": 0}
VICTIM_PTS = {("letter", True): 5, ("letter", False): 15, ("cog", True): 10, ("cog", False): 30}
KIT_PTS = {0: 0, 1: 10, 2: 30}
NEAR_MM = 150.0            # 15 см (5.6.1, 5.6.3)
BLINK_MIN_S = 5.0
BLINK_STILL_MM = 15.0      # сдвинулся за мигание больше - не стоял
BLUE_STOP_S = 5.0
FINISH_MIN_S = 10.0
ROBOT_HALF_LEN = 96.5
ROBOT_HALF_W = 62.5


def victim_point(v):
    """Центр метки на стене (как maze_sim.Victim.point), мм."""
    i, j = v["cell"]
    dx, dy = mazegen.DIRS[v["d"]]
    f = 150.0 - 10.0
    return (i * 300.0 + dx * f + (v["along"] if dy else 0.0),
            j * 300.0 + dy * f + (v["along"] if dx else 0.0))


def dist_to_robot(px, py, x, y, th_deg):
    """От точки до корпуса робота (прямоугольник 193 x 125 мм), мм."""
    th = math.radians(th_deg)
    fx, fy = math.sin(th), math.cos(th)
    rx, ry = px - x, py - y
    a = rx * fx + ry * fy               # вперёд
    b = rx * fy - ry * fx               # вправо
    da = max(0.0, abs(a) - ROBOT_HALF_LEN)
    db = max(0.0, abs(b) - ROBOT_HALF_W)
    return math.hypot(da, db)


def ramp_done(r, visits):
    """Горка r пройдена (снизу вверх или сверху вниз) по посещениям."""
    bottom, top = tuple(r["bottom"]), tuple(r["top"])
    mid = {tuple(c) for c in r["cells"]}
    seq = [(v["cell"], v["full"]) for v in visits]
    for i, (c, _) in enumerate(seq):
        if c not in (bottom, top):
            continue
        goal = top if c == bottom else bottom
        for c2, full in seq[i + 1:]:
            if c2 in mid:
                continue
            if c2 == goal and full:
                return True
            break
    return False


def score(spec, events, visits, end_cell):
    """Очки заезда с разбивкой. events - Sim.events, visits - Sim.visits,
    end_cell - клетка мира, где робот кончил (по большей части корпуса)."""
    vic = spec["victims"]
    reach = mazegen.reachable(spec)
    linear = mazegen.linear_tiles(spec)
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    pts = [victim_point(v) for v in vic]
    real = [i for i, v in enumerate(vic) if v["type"] in KITS_NEED]

    # ── метки ─────────────────────────────────────────────
    ident = {}                 # индекс -> время мигания
    misid = wrong = dup = bad_blink = fake_blink = 0
    for t, kind, x, y, d in events:
        if kind != "blink":
            continue
        th = d.get("th", 0.0)
        near = [(dist_to_robot(*pts[i], x, y, th), i) for i in range(len(vic))]
        near = [(dd, i) for dd, i in near if dd <= NEAR_MM]
        near_real = [(dd, i) for dd, i in near if i in real]
        if not near_real:
            misid += 1
            fake_blink += bool(near)
            continue
        if d.get("dur", BLINK_MIN_S) < BLINK_MIN_S - 1e-6 or d.get("moved", 0.0) > BLINK_STILL_MM:
            bad_blink += 1
            continue
        # Две метки в 15 см (угол клетки, старт) - засчитывается та, что
        # робот прочитал (truth от камеры модели); иначе ближайшая ещё не
        # опознанная, иначе ближайшая.
        tr = d.get("truth")
        fresh = [c for c in near_real if c[1] not in ident]
        if tr is not None and any(c[1] == tr for c in near_real):
            i = tr
        else:
            i = min(fresh or near_real)[1]
        if d.get("type") and d["type"] != vic[i]["type"]:
            wrong += 1
        if i in ident:
            dup += 1
            continue
        ident[i] = t
    vpts = 0
    floating_found = 0
    for i in ident:
        v = vic[i]
        lin = tuple(v["cell"]) in linear
        floating_found += not lin
        vpts += VICTIM_PTS[(v.get("kind", "letter"), lin)]

    # ── комплекты ─────────────────────────────────────────
    kits_at = {}
    kits_total = kits_lost = 0
    for t, kind, x, y, d in events:
        if kind != "kit":
            continue
        kits_total += 1
        kx, ky = d.get("at", (x, y))
        cand = [(math.hypot(pts[i][0] - kx, pts[i][1] - ky), i) for i, ti in ident.items() if ti < t]
        cand = [c for c in cand if c[0] <= NEAR_MM]
        if not cand:
            kits_lost += 1
            continue
        i = min(cand)[1]
        kits_at[i] = kits_at.get(i, 0) + 1
    srd = 0
    kpts = 0
    for i, n in kits_at.items():
        k = min(n, KITS_NEED[vic[i]["type"]])
        srd += k
        kpts += KIT_PTS[k]
    kits_need = sum(KITS_NEED[vic[i]["type"]] for i in real if tuple(vic[i]["cell"]) in reach)

    # ── плитки ────────────────────────────────────────────
    lop_black = lop_blue = 0
    blue_visits = {}
    blue_ok = set()
    for k, v in enumerate(visits):
        tile = tiles.get(v["cell"], "white")
        if tile == "black":
            lop_black += 1
        if tile == "blue":
            blue_visits[v["cell"]] = blue_visits.get(v["cell"], 0) + 1
            if v["still"] >= BLUE_STOP_S:
                blue_ok.add(v["cell"])
            elif k + 1 < len(visits):          # ушёл на другую, не простояв
                lop_blue += 1
    blue_pts = {c: (max(0, 30 - 10 * (n - 1)) if c in blue_ok else 0) for c, n in blue_visits.items()}
    sbv = sum(1 for p in blue_pts.values() if p > 0)
    blue_repeat = sum(n - 1 for n in blue_visits.values())
    cp = {v["cell"] for v in visits if tiles.get(v["cell"]) == "silver"}
    bumps = {tuple(d["cell"]) for _, kind, _, _, d in events if kind == "bump_cross"}
    ramps = spec.get("ramps", [])
    srn = sum(1 for r in ramps if r["kind"] == "ramp" and ramp_done(r, visits))
    ssn = sum(1 for r in ramps if r["kind"] == "stairs" and ramp_done(r, visits))

    # ── выход ─────────────────────────────────────────────
    fin = [(t, x, y, d) for t, kind, x, y, d in events if kind == "finish_blink"]
    exit_ok = bool(fin) and tuple(end_cell) == (0, 0) and fin[-1][3].get("s", 0) >= FINISH_MIN_S \
        and fin[-1][3].get("moved", 0.0) <= BLINK_STILL_MM
    svi = len(ident)
    lop = lop_black + lop_blue
    parts = {
        "victims": vpts, "kits": kpts, "blue": sum(blue_pts.values()),
        "checkpoints": 10 * len(cp), "bumps": 5 * len(bumps),
        "ramps": 10 * srn, "stairs": 10 * ssn,
        "exit": (10 * svi + 10 * sbv + 5 * ssn + 5 * srn) if exit_ok else 0,
        "reliability": max(0, 10 * svi + 10 * srd + 10 * sbv - 15 * lop),
        "misid": -5 * misid,
    }
    total = max(0, sum(parts.values()))
    reach_real = [i for i in real if tuple(vic[i]["cell"]) in reach]
    return {
        "score": total, "parts": parts,
        "svi": svi, "victims": len(reach_real), "floating_found": floating_found,
        "misid": misid, "fake_blink": fake_blink, "wrong": wrong, "dup": dup, "bad_blink": bad_blink,
        "srd": srd, "kits_need": kits_need, "kits_given": kits_total, "kits_lost": kits_lost,
        "sbv": sbv, "blue_tiles": sum(1 for t in tiles.values() if t == "blue"),
        "blue_repeat": blue_repeat, "lop": lop, "lop_black": lop_black, "lop_blue": lop_blue,
        "checkpoints": len(cp), "bump_tiles": len(bumps), "srn": srn, "ssn": ssn,
        "ramps_total": sum(1 for r in ramps if r["kind"] == "ramp"),
        "stairs_total": sum(1 for r in ramps if r["kind"] == "stairs"),
        "exit": exit_ok, "t_exit": round(fin[-1][0], 1) if exit_ok else None,
        "missed": [i for i in reach_real if i not in ident],
    }


def max_score(spec):
    """Сколько можно было набрать на этом поле (всё найдено, ни одного LoP,
    вернулся) - для доли от максимума в отчёте."""
    vic = spec["victims"]
    reach = mazegen.reachable(spec)
    linear = mazegen.linear_tiles(spec)
    tiles = {(x, y): t for x, y, t in spec["tiles"]}
    real = [v for v in vic if v["type"] in KITS_NEED and tuple(v["cell"]) in reach]
    svi = len(real)
    vp = sum(VICTIM_PTS[(v.get("kind", "letter"), tuple(v["cell"]) in linear)] for v in real)
    srd = sum(KITS_NEED[v["type"]] for v in real)
    kp = sum(KIT_PTS[KITS_NEED[v["type"]]] for v in real)
    sbv = sum(1 for c, t in tiles.items() if t == "blue" and c in reach)
    cp = sum(1 for c, t in tiles.items() if t == "silver" and c in reach)
    bumps = len({tuple(b["cell"]) for b in spec["bumps"] if tuple(b["cell"]) in reach})
    ramps = spec.get("ramps", [])
    srn = sum(1 for r in ramps if r["kind"] == "ramp")
    ssn = sum(1 for r in ramps if r["kind"] == "stairs")
    return (vp + kp + 30 * sbv + 10 * cp + 5 * bumps + 10 * (srn + ssn)
            + 10 * svi + 10 * sbv + 5 * ssn + 5 * srn + 10 * svi + 10 * srd + 10 * sbv)
