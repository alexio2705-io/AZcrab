#!/usr/bin/env python3
"""Расстояния до передней, правой, левой и задней стен.

Геометрия замера повторяет nav.py: те же маски на детали робота, те же
окна и проекции, та же медиана. Задней стены в nav.py нет, она добавлена
здесь - см. BACK_* и комментарий про маску задней зоны.

Запуск на малине:  ~/maze/venv/bin/python ~/maze/walls.py
"""

from __future__ import annotations

import math
import time

from ld19 import DEFAULT_PORT, BAUDRATE, ZERO_DIR, LD19

# ── пороги «стена есть» (nav.py) ─────────────────────────────
WALL_THRESH = 260   # перёд
SIDE_THRESH = 210   # борта
BACK_THRESH = 260   # зад, по аналогии с передом

# ── окна и ограничения проекции (nav.py) ─────────────────────
# SIDE_BACK_LIMIT сознательно мал (было 40): боковой замер участвует в решении
# "поворачивать или ехать дальше", которое nav.py принимает за DECIDE_LEAD_MM
# до центра клетки. При 40мм заднее окно упиралось ровно в границу предыдущей
# клетки, и робот принимал решение по ЕЁ стене - то есть проезжал мимо
# поворота на клетку дальше. С 10мм появляется запас в 30мм.
FRONT_WIN, SIDE_WIN, BACK_WIN = 22, 32, 22
SIDE_FWD_LIMIT, SIDE_BACK_LIMIT = 85, 10

# Отдельное, СИММЕТРИЧНОЕ заднее ограничение для замера СТОЯ. Обоснование
# SIDE_BACK_LIMIT=10 - "окно замера должно целиком лежать уже в НОВОЙ клетке" -
# относится к решению, которое принимается НА ХОДУ, за DECIDE_LEAD_MM до
# центра. Робот, который уже стоит в центре клетки, окном ±85мм за её
# пределы (300мм) не выходит вовсе, а точек получает заметно больше.
#
# Это важнее, чем кажется. Маски ручек (±12° вокруг 90/270) вырезали середину
# бокового окна, и от ±32° осталась ОДНОСТОРОННЯЯ дуга; вот сколько целых
# градусов доживает до медианы для плоской боковой стены:
#
#     стена, мм | на ходу (10) | стоя (85)
#         140   |     20       |    30
#         180   |     14       |    18
#         210   |     11       |    11     <- это и есть SIDE_THRESH
#         240   |      9       |     9
#
# То есть симметричное окно помогает центровке (140..180мм), а у самого порога
# "есть стена / нет стены" не помогает ничем: там дугу режет уже не
# SIDE_BACK_LIMIT, а сам SIDE_WIN вместе с маской ручки. Расширять SIDE_WIN
# нельзя - окно уедет в соседнюю клетку. Поэтому на пороге замер держится на
# ~11 точках, и если ручка на самом деле уже 12°, полуширину маски стоит
# уменьшить: ложное "справа открыто" - это поворот в стену.
SIDE_BACK_LIMIT_STILL = 85
FRONT_LAT_LIMIT = BACK_LAT_LIMIT = 130
DIST_MAX = 3000

# ── контур самого робота (замерено) ──────────────────────────
# Прежде был плоский DIST_MIN=80: при ширине 125мм и окне борта 90°±32°
# лучевая дальность до стены, прижатой вплотную к борту, — 64..74мм (см.
# геометрию ниже), то есть ЦЕЛИКОМ ниже 80 - стена у самого борта пропадала
# из замера полностью, и центровке становилось не от чего отруливать.
#
# Вместо константы - порог по контуру прямоугольного корпуса: под углом a
# луч выходит за пределы корпуса на дистанции min(HALF_LEN/|cos a|,
# HALF_WID/|sin a|) - меньше этого лидар видит не стену, а свой же корпус.
# SELF_MARGIN - запас на то, что лидар не идеально в центре и на ширину луча.
ROBOT_LEN_MM = 193.0
ROBOT_WID_MM = 137.0
HALF_LEN = ROBOT_LEN_MM / 2.0
HALF_WID = ROBOT_WID_MM / 2.0
# Было 15 - вместе с геометрией корпуса это давало слепую зону ~110-119мм
# в переднем окне, которая почти вплотную примыкала к аварийному порогу
# FRONT_EMERGENCY_MM в nav.py: лидар переставал видеть стену буквально за
# несколько мм до того, как должен был сработать аварийный стоп. 5мм - тот
# же запас, что был в самом первом варианте (DIST_MIN=80 плоским порогом
# на предыдущем роботе), уже достаточно против шума лидара по краю корпуса.
SELF_MARGIN = 5.0


def self_outline_mm(deg: float) -> float:
    """Дистанция, на которой луч под углом deg покидает корпус робота."""
    rad = math.radians(deg)
    c, s = abs(math.cos(rad)), abs(math.sin(rad))
    r_len = HALF_LEN / c if c > 1e-6 else float('inf')
    r_wid = HALF_WID / s if s > 1e-6 else float('inf')
    return min(r_len, r_wid)

# ── зоны, где лидар видит сам робот (nav.py) ─────────────────
MASKS = [
    {'center':  90, 'hw': 12},   # правая ручка
    {'center': 270, 'hw': 12},   # левая ручка
    {'center': 180, 'hw': 55},   # задняя зона
    # Леска ручки относительно стрелки LD19: raw +15° и её отражение
    # относительно вертикальной оси raw 165°. При ZERO_DIR=90° это 105°
    # и 255° в системе робота. Полуширина 7° покрывает крепёжный допуск.
    {'center': 105, 'hw':  7},
    {'center': 255, 'hw':  7},
    {'center':  25, 'hw': 10},   # правая передняя стойка
    {'center': 335, 'hw': 10},   # левая передняя стойка
]
# Задняя маска закрывает 125..235 - ровно там, где должна быть задняя стена.
# Поэтому для 'back' она снимается, а остальные маски остаются.
REAR_MASK = MASKS[2]

# Леска даёт устойчивую ложную "левую стену": robot 285..302°
# (raw LD19 195..212°), дистанция 70..86 мм. Угловая маска здесь была бы
# плохой: в том же секторе лидар видит настоящую левую стену. Поэтому
# отбрасываем только ближний след лески с запасом, а дальние точки сохраняем.
ROPE_LEFT_CENTER_DEG = 294.0
ROPE_LEFT_HALF_WIDTH_DEG = 12.0
# Было 105. Зона лески (282..306°) полностью накрывает ту дугу, что остаётся
# от левого борта после масок ручек (282.1..301.2° для стены на 140мм), так
# что этот порог - не "сколько отрезать лески", а "насколько близко левая
# стена становится невидимой". При 105мм по лучу под 294° слепой оказывалась
# стена ближе 96мм по нормали - при рабочем SIDE_CENTER_MM=140 это всего 44мм
# сноса, вполне достижимо, и центровка теряла опору ровно тогда, когда она
# нужнее всего. Справа такой дыры нет: там фильтр другой (ROPE_EDGE_ZONES),
# то есть фильтрация была несимметричной.
#
# Замеренный след лески - 70..86мм по лучу, поэтому 92 оставляет ей запас 6мм
# и сдвигает слепую полосу к 68..84мм по нормали, то есть вплотную к
# собственному габариту (self_outline под 294° даёт 68.5мм).
#
# Опорной фильтрацией, как для ROPE_EDGE_ZONES, эту зону не спасти: в окне
# ±32° от 270 вне 282..306 и в пределах ROPE_SUPPORT_RADIUS_DEG от 294
# немаскированных углов не остаётся ВООБЩЕ - поддержке взяться неоткуда.
ROPE_LEFT_MAX_MM = 92.0

# Край лески изредка даёт более дальний ответ, который не попадает
# под ROPE_LEFT_MAX_MM. Не выкидываем весь боковой замер: для каждого такого
# луча ищем рядом продолжение той же плоскости. У стены оно есть, у одиночного
# блика лески — нет. Правая зона нужна для такого же краевого ответа.
ROPE_EDGE_ZONES = (
    (66.5, 3.0),
    (300.0, 3.0),
)
ROPE_EDGE_MAX_MM = 170.0
ROPE_SUPPORT_RADIUS_DEG = 15.0
ROPE_SUPPORT_PROJ_TOL_MM = 18.0
ROPE_SUPPORT_MIN_POINTS = 2


def is_masked(deg: float, masks=MASKS) -> bool:
    for m in masks:
        d = abs(deg - m['center'])
        if d > 180:
            d = 360 - d
        if d <= m['hw']:
            return True
    return False


def is_left_rope(deg: float, dist: float) -> bool:
    """True only for the short close-range trace of the left handle rope."""
    off = abs(deg - ROPE_LEFT_CENTER_DEG)
    if off > 180.0:
        off = 360.0 - off
    return off <= ROPE_LEFT_HALF_WIDTH_DEG and dist <= ROPE_LEFT_MAX_MM


def _angle_off(deg: float, center: float) -> float:
    off = abs(deg - center)
    return 360.0 - off if off > 180.0 else off


def _side_projection(deg: float, dist: float) -> float:
    return abs(dist * math.sin(math.radians(deg)))


def without_unsupported_rope_edges(points: dict) -> dict:
    """Remove only isolated rope-edge returns; keep a continuous wall."""
    result = dict(points)
    for center, half_width in ROPE_EDGE_ZONES:
        for angle, dist in points.items():
            if (dist > ROPE_EDGE_MAX_MM
                    or _angle_off(angle, center) > half_width):
                continue
            projected = _side_projection(angle, dist)
            support = 0
            for other_angle, other_dist in points.items():
                if (other_angle == angle
                        or _angle_off(other_angle, center) <= half_width
                        or _angle_off(angle, other_angle) > ROPE_SUPPORT_RADIUS_DEG):
                    continue
                if abs(_side_projection(other_angle, other_dist)
                       - projected) <= ROPE_SUPPORT_PROJ_TOL_MM:
                    support += 1
            if support < ROPE_SUPPORT_MIN_POINTS:
                result.pop(angle, None)
    return result


# Боковая стена должна быть ПЛОСКОСТЬЮ, а не горсткой точек. Ложные "стены
# там, где их нет" почти все собраны из 1-9 точек у краёв ручки перед
# лидаром (слева ~283-302°, справа ~58-68°): луч цепляет край ручки и стену
# за ней сразу и даёт промежуточную дальность, 85-135мм по боку. По одной
# дальности это не отличить от стены, прижатой к борту, но настоящая стена
# даёт ровную линию: даже на пороге SIDE_THRESH это ~11 точек с одинаковым
# боковым расстоянием, растянутых по дуге. Призрак - либо разброс, либо
# узкая кучка в 4-7° у края ручки. Проверено на 2900 событиях
# wall_debug.log: из ложных стен по 1-4 точкам правило убирает все, кроме
# одной; живые стены на 118 и 131мм проходят в 100% оборотов.
#
# Первая версия сравнивала все точки с ОДНОЙ медианой бокового расстояния, и
# это теряло настоящие стены в двух случаях (видно по тому же логу):
#  - робот стоит к стене под углом: боковое расстояние вдоль стены линейно
#    растёт, при перекосе ~10° в окне стоя это уже ±15мм, и точки с краёв
#    вылетают из допуска;
#  - в окне две поверхности - передняя дуга (58-77°) видит одно расстояние,
#    задняя (113-122°) другое: медиана ложится МЕЖДУ ними, и в допуск не
#    попадает ни та, ни другая - стена отбрасывается целиком.
# Поэтому плоскость теперь - прямая в координатах (вперёд, вбок) с наклоном
# не круче SIDE_FLAT_MAX_SKEW_DEG (перпендикулярный торец стены и разброс
# лески по-прежнему не проходят), а из нескольких таких берётся та, на
# которую легло больше всего точек. На 3000 событиях лога: стены по 15+
# точкам проходят в 100% оборотов (было 94%), по 10-14 - 92% (86%), призраки
# по 1-4 точкам по-прежнему отбрасываются все.
SIDE_FLAT_TOL_MM = 15.0      # "та же плоскость": отклонение от прямой
SIDE_FLAT_MIN_POINTS = 5     # столько точек должно лечь в плоскость
SIDE_FLAT_MIN_SPAN_DEG = 6.0  # и растянуться хотя бы на столько по дуге
SIDE_FLAT_MAX_SKEW_DEG = 20.0  # перекос робота к стене, который ещё стена
# Стена идёт ВДОЛЬ борта, поэтому её точки растянуты по ходу робота. Кирпич
# (препятствие) впереди-сбоку даёт торец ПОПЕРЁК: замерено на поле - точки на
# 296-302° с "вперёд" 70-82мм и боком 125-155мм, то есть по ходу всего
# 12-22мм, и по одной боковой дальности он сходил за стену. Настоящая стена
# даже на самом пороге SIDE_THRESH (окно 283-292°) растянута на ~37мм.
SIDE_FLAT_MIN_EXTENT_MM = 30.0
_SKEW_TAN = math.tan(math.radians(SIDE_FLAT_MAX_SKEW_DEG))
# Наклон прямой-кандидата - по точке и её соседу через столько точек по дуге.
_SKEW_STEP = 3


def _side_plane(pts):
    """pts - [(угол, вперёд_мм, вбок_мм)] одного борта. Боковое расстояние
    (медиана по точкам плоскости) до стены, на которую легло больше всего
    точек, или None, если ни одна прямая не похожа на стену."""
    if len(pts) < SIDE_FLAT_MIN_POINTS:
        return None
    pts = sorted(pts)
    # Кандидаты: через каждую точку - параллельно роботу и с наклоном к
    # соседу по дуге. Точек в окне борта не больше ~30, так что перебор
    # всех кандидатов - доли миллисекунды на оборот.
    cands = []
    for i, (_, x0, l0) in enumerate(pts):
        cands.append((0.0, x0, l0))
        if i + _SKEW_STEP < len(pts):
            dx = pts[i + _SKEW_STEP][1] - x0
            if abs(dx) >= 5.0:
                k = (pts[i + _SKEW_STEP][2] - l0) / dx
                cands.append((max(-_SKEW_TAN, min(_SKEW_TAN, k)), x0, l0))
    best = None
    for k, x0, l0 in cands:
        inl = [(a, x, lat) for a, x, lat in pts
               if abs(lat - (l0 + k * (x - x0))) <= SIDE_FLAT_TOL_MM]
        if (len(inl) < SIDE_FLAT_MIN_POINTS
                or inl[-1][0] - inl[0][0] < SIDE_FLAT_MIN_SPAN_DEG):
            continue
        xs = [x for _, x, _ in inl]
        if max(xs) - min(xs) < SIDE_FLAT_MIN_EXTENT_MM:
            continue
        key = (-len(inl), _median([lat for _, _, lat in inl]))
        if best is None or key < best:
            best = key
    return None if best is None else best[1]


def _median(lst):
    if not lst:
        return None
    s = sorted(lst)
    return s[len(s) // 2]


def wall_dist(filt: dict, axis: str, side_back_limit: float = SIDE_BACK_LIMIT):
    """Дистанция до стены по оси: 'front' | 'right' | 'left' | 'back'.

    Точки в окне вокруг оси проецируются на неё, поперечный разброс
    ограничивается, результат - медиана. Точь-в-точь _wall_dist из nav.py,
    плюс ветка 'back'.

    side_back_limit - насколько далеко НАЗАД от лидара разрешено брать точки
    для бортов: SIDE_BACK_LIMIT для замера на ходу, SIDE_BACK_LIMIT_STILL для
    замера стоя (см. комментарий к константам).

    Возвращает (дистанция_мм_или_None, число_точек_в_медиане) - число нужно,
    чтобы отличить "стены нет" от "в дуге не осталось точек": после масок
    ручек боковая дуга тонкая, и эти два случая ведут к противоположным
    решениям.

    Для бортов дистанция - до плоскости, которую нашёл _side_plane(); если
    точки в плоскость не складываются - это не стена: дистанция None, число
    точек остаётся для диагностики.
    """
    center, win = {'front': (0.0, FRONT_WIN), 'right': (90.0, SIDE_WIN),
                   'back': (180.0, BACK_WIN), 'left': (270.0, SIDE_WIN)}[axis]
    vals = []
    side_pts = []
    for a, d in filt.items():
        off = abs(a - center)
        if off > 180.0:
            off = 360.0 - off
        if off > win:
            continue
        rad = math.radians(a)
        fwd_s = d * math.cos(rad)
        lat = abs(d * math.sin(rad))
        if axis in ('front', 'back'):
            if lat > FRONT_LAT_LIMIT:
                continue
            proj = abs(fwd_s)
        else:
            if fwd_s > SIDE_FWD_LIMIT or fwd_s < -side_back_limit:
                continue
            proj = lat
        if proj > 0:
            vals.append(proj)
            # Угол - непрерывный по дуге борта: левый борт лежит через 270,
            # без перехода через 0, правый - через 90.
            side_pts.append((a, fwd_s, proj))
    if axis in ('right', 'left'):
        return _side_plane(side_pts), len(vals)
    return _median(vals), len(vals)


def filtered(rev):
    """Оборот -> (все маски, без задней маски) - {угол: дистанция} после
    фильтров корпуса, масок и лески. Общее для measure() и точек коридора,
    которые lidar_proc.py отдаёт nav.py (corridor.corridor_points)."""
    full, no_rear = {}, {}
    rear_only = [m for m in MASKS if m is not REAR_MASK]
    for angle, dist, _ in rev:
        if (dist > DIST_MAX or dist <= self_outline_mm(angle) + SELF_MARGIN
                or is_left_rope(angle, dist)):
            continue
        key = round(angle)
        if not is_masked(angle):
            full[key] = dist
        if not is_masked(angle, rear_only):
            no_rear[key] = dist
    return without_unsupported_rope_edges(full), without_unsupported_rope_edges(no_rear)


def measure(rev, filt=None) -> dict:
    """Оборот точек -> дистанции до стен в мм (None = нет стены).

    Ключи 'front'/'right'/'left'/'back' - замер ДЛЯ ДВИЖЕНИЯ (асимметричное
    боковое окно, см. SIDE_BACK_LIMIT). Ключи 'right_still'/'left_still' - тот
    же оборот, но симметричным окном: он корректен только когда робот стоит в
    центре клетки, зато там, где стена близко, даёт в полтора раза больше
    точек. Ключи '*_n' - сколько точек вошло в каждую медиану.

    filt - уже готовый filtered(rev), если вызывающему он нужен и сам.
    """
    full, no_rear = filt if filt is not None else filtered(rev)

    front, front_n = wall_dist(full, 'front')
    right, right_n = wall_dist(full, 'right')
    left,  left_n  = wall_dist(full, 'left')
    back,  back_n  = wall_dist(no_rear, 'back')
    right_s, right_s_n = wall_dist(full, 'right', SIDE_BACK_LIMIT_STILL)
    left_s,  left_s_n  = wall_dist(full, 'left',  SIDE_BACK_LIMIT_STILL)
    return {'front': front, 'right': right, 'left': left, 'back': back,
            'right_still': right_s, 'left_still': left_s,
            'front_n': front_n, 'right_n': right_n, 'left_n': left_n,
            'back_n': back_n, 'right_still_n': right_s_n,
            'left_still_n': left_s_n}


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="расстояния до стен")
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("-b", "--baud", type=int, default=BAUDRATE)
    ap.add_argument("--zero-dir", type=float, default=ZERO_DIR)
    ap.add_argument("-n", type=int, default=0, help="сколько замеров (0 = без конца)")
    args = ap.parse_args()

    thr = {'front': WALL_THRESH, 'right': SIDE_THRESH,
           'left': SIDE_THRESH, 'back': BACK_THRESH}

    def fmt(axis, v):
        if v is None:
            return f"{axis} {'—':>6}     "
        return f"{axis} {v:6.0f}мм {'стена' if v < thr[axis] else ' ... '}"

    with LD19(args.port, args.baud, zero_dir=args.zero_dir) as lidar:
        n = 0
        for rev in lidar.revolutions():
            w = measure(rev)
            print("  ".join(fmt(a, w[a])
                            for a in ('front', 'right', 'back', 'left')),
                  flush=True)
            n += 1
            if args.n and n >= args.n:
                break


if __name__ == "__main__":
    main()
