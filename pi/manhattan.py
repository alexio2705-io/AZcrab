"""Курс по стенам ("манхэттен"): стены поля идут только по двум направлениям,
под 90° друг к другу, поэтому по точкам лидара видно, на сколько робот
повёрнут относительно сетки стен - с точностью до 90°. Сравнение с
гироскопом даёт его уход (nav.py, _measure_cell -> YAW_ORIGIN).

Чистая логика, без железа: pi/tests/test_manhattan.py.

Точки - как corridor.corridor_points(): [x вперёд, y вправо] мм в системе
робота. Угол - по часовой (как курс в nav.py): робот повёрнут на +b, значит
стены в его системе повёрнуты на -b.

Способ (split-and-merge): точки в порядке обхода лидара (по углу) режутся
на цепочки по разрывам, цепочка - пополам в точке, дальше всего отстоящей
от хорды, пока не станет ровной; длинные ровные отрезки - стены, по каждой
прямая наименьших квадратов (главная ось). Направление стены по модулю 90° -
кандидат курса с весом по длине и числу точек; итог - взвешенная медиана,
уточнённая средним по согласным с ней стенам.
"""
from __future__ import annotations

import math

MAX_STEP_MM = 35.0      # соседние точки дальше этого - разрыв, разные стены
SPLIT_MM = 20.0         # точка дальше этого от хорды - излом (угол, столбик)
MIN_LEN_MM = 80.0       # отрезок короче - не стена
MIN_PTS = 10            # и не меньше стольких точек
MIN_WALLS_MM = 200.0    # суммарно согласных стен меньше - не верим
KEEP_DEG = 3.0          # согласные с медианой стены - в пределах этого


def _wrap90(a):
    """Угол -> (-45, 45]."""
    a = (a + 45.0) % 90.0 - 45.0
    return 45.0 if a == -45.0 else a


def _chains(pts):
    p = sorted(pts, key=lambda q: math.atan2(q[1], q[0]))
    out, cur = [], []
    for q in p:
        if cur and math.hypot(q[0] - cur[-1][0], q[1] - cur[-1][1]) > MAX_STEP_MM:
            out.append(cur)
            cur = []
        cur.append(q)
    if cur:
        out.append(cur)
    return out


def _split(c, out):
    if len(c) < MIN_PTS:
        return
    (x0, y0), (x1, y1) = c[0], c[-1]
    L = math.hypot(x1 - x0, y1 - y0)
    if L < 1e-6:
        return
    nx, ny = -(y1 - y0) / L, (x1 - x0) / L
    dev = [abs((q[0] - x0) * nx + (q[1] - y0) * ny) for q in c]
    k = max(range(len(c)), key=dev.__getitem__)
    if dev[k] > SPLIT_MM and 0 < k < len(c) - 1:
        _split(c[:k + 1], out)
        _split(c[k:], out)
    else:
        out.append(c)


def segments(pts):
    """Ровные отрезки стен: [(угол по модулю 90, длина мм, точек)]."""
    segs = []
    for ch in _chains(pts):
        _split(ch, segs)
    out = []
    for c in segs:
        n = len(c)
        mx = sum(q[0] for q in c) / n
        my = sum(q[1] for q in c) / n
        sxx = sum((q[0] - mx) ** 2 for q in c)
        syy = sum((q[1] - my) ** 2 for q in c)
        sxy = sum((q[0] - mx) * (q[1] - my) for q in c)
        ang = 0.5 * math.atan2(2 * sxy, sxx - syy)          # главная ось
        L = math.hypot(c[-1][0] - c[0][0], c[-1][1] - c[0][1])
        if L >= MIN_LEN_MM and n >= MIN_PTS:
            out.append((_wrap90(math.degrees(ang)), L, n))
    return out


def wall_angle(pts):
    """На сколько градусов (по часовой) робот повёрнут относительно сетки
    стен (в пределах ±45), или None - ровных стен видно мало или они не
    согласны между собой."""
    segs = segments(pts)
    if not segs:
        return None
    # вес стены ~ обратная дисперсия наклона: точек x длина^2
    w = [(a, n * L * L, L) for a, L, n in segs]
    ref = max(w, key=lambda t: t[1])[0]
    rel = sorted((_wrap90(a - ref), wt) for a, wt, _ in w)
    tot, acc, med = sum(wt for _, wt in rel), 0.0, 0.0
    for r, wt in rel:
        acc += wt
        if acc >= tot / 2:
            med = ref + r
            break
    keep = [(_wrap90(a - med), wt, L) for a, wt, L in w if abs(_wrap90(a - med)) <= KEEP_DEG]
    if sum(L for _, _, L in keep) < MIN_WALLS_MM:
        return None
    m = sum(r * wt for r, wt, _ in keep) / sum(wt for _, wt, _ in keep)
    # стены в системе робота повёрнуты на минус курс
    return _wrap90(-(med + m))
