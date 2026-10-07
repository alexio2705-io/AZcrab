"""Стены клеток впереди по точкам лидара на ходу - для проезда прямых без
остановок (drive_cells с планом, см. nav.py). Чистая логика: без железа,
покрыта pi/tests/test_corridor.py. Дизайн: docs/superpowers/specs/
2026-10-07-nonstop-straights-design.md.

Система пути: X - мм вдоль хода от центра клетки, где поездка началась
(одометрия без обнулений), Y - мм вбок, вправо > 0. Углы лидара - как в
walls.py: 0° вперёд, по часовой, 90° - правый борт.

Передние стойки робота (маски 25±10 и 335±10 в walls.MASKS) закрывают
боковую стену дальше ~21 см вперёд: стену следующей клетки лидар видит
только за последние ~20 см до точки решения, 2-3 оборота. Поэтому решение -
не по голосам оборотов, а по точкам и по тому, какую длину стены реально
было видно.
"""
from __future__ import annotations

import math

CELL_MM = 300.0
HALF_CELL_MM = 150.0
WALL_LAT_MM = 150.0       # боковая стена от оси хода, робот по центру клетки
BAND_LO_MM = 100.0        # полоса "это стена этой клетки" вбок; 210 - это SIDE_THRESH
BAND_HI_MM = 210.0
SEG_HALF_MM = 100.0       # средняя часть стены вдоль хода: без столбиков и концов соседних
VIEW_MAX_MM = 600.0       # дальше по коридору не смотрим
SPAN_STEP_MM = 5.0        # шаг, которым меряется видимая длина стены
WALL_MIN_PTS = 8          # "стена есть": столько точек в полосе...
WALL_MAX_IQR_MM = 20.0    # ...и они лежат в линию (межквартильный разброс вбок)
OPEN_MIN_SPAN_MM = 120.0  # "проход": видно хотя бы столько из 200мм...
OPEN_MAX_PTS = 2          # ...и точек в полосе почти нет
POST_TOL_MM = 40.0        # поправка по столбику - только если ошибка не больше
POST_MIN_PTS = 3          # столько точек стены у её конца
# Полтолщины стены: робот по центру клетки - в 140 мм от неё (SIDE_CENTER_MM),
# сетка - в 150. Стена кончается на столбике - на столько ЗА границей клеток,
# начинается - на столько ДО неё; без этого каждая поправка тянула бы путь
# на 5 мм в одну сторону.
WALL_HALF_T_MM = 10.0

# Коридор, который lidar_proc.py отдаёт в nav.py: в системе робота, мм.
CORR_X_MIN, CORR_X_MAX = -100.0, 600.0
CORR_Y_MIN, CORR_Y_MAX = 60.0, 320.0


def corridor_points(filt):
    """{угол: дистанция} оборота (после фильтров walls.filtered) -> точки
    боковых коридоров [x вперёд, y вправо] целыми мм - так их и шлёт
    lidar_proc.py (JSON 10 раз в секунду, поэтому коротко)."""
    out = []
    for a, d in filt.items():
        r = math.radians(a)
        x, y = d * math.cos(r), d * math.sin(r)
        if CORR_X_MIN <= x <= CORR_X_MAX and CORR_Y_MIN <= abs(y) <= CORR_Y_MAX:
            out.append([round(x), round(y)])
    return out


def point_time(t_end, trev, x, y):
    """Время замера точки: оборот длиной trev кончается на шве сзади (180°)
    в t_end, углы растут по часовой."""
    deg = math.degrees(math.atan2(y, x)) % 360.0
    return t_end - trev * (1.0 - ((deg - 180.0) % 360.0) / 360.0)


def to_path_frame(pts, t_end, trev, travel_at, yaw_err_deg):
    """Точки оборота [(x вперёд, y вправо)] в системе робота -> [(X, Y)] в
    системе пути. travel_at(t) - путь в мм на момент t (история одометрии):
    за оборот робот на 55см/с проезжает 55мм, поэтому время - по точке.
    yaw_err_deg - курс робота минус курс хода (по часовой > 0)."""
    a = math.radians(yaw_err_deg)
    c, s = math.cos(a), math.sin(a)
    out = []
    for x, y in pts:
        tr = travel_at(point_time(t_end, trev, x, y))
        out.append((tr + x * c - y * s, y * c + x * s))
    return out


def _masked(deg, masks):
    return any(abs((deg - center + 180.0) % 360.0 - 180.0) <= hw for center, hw in masks)


def visible_steps(x_robot, seg_lo, seg_hi, side, masks):
    """Индексы шагов SPAN_STEP_MM на отрезке [seg_lo, seg_hi] боковой стены
    (WALL_LAT_MM вбок), которые лидар из x_robot видит: луч не под маской и
    не дальше VIEW_MAX_MM. Множество - чтобы видимое за разные обороты
    объединялось без двойного счёта."""
    out = set()
    i = math.ceil(seg_lo / SPAN_STEP_MM)
    while i * SPAN_STEP_MM <= seg_hi:
        dx = i * SPAN_STEP_MM - x_robot
        if math.hypot(dx, WALL_LAT_MM) <= VIEW_MAX_MM:
            deg = math.degrees(math.atan2(WALL_LAT_MM, dx))
            if side == "left":
                deg = 360.0 - deg
            if not _masked(deg, masks):
                out.add(i)
        i += 1
    return out


class CellSides:
    """Накопитель по клеткам впереди: для (номер клетки, сторона) - поперечные
    дистанции точек в полосе стены и видимые участки её средней части; плюс
    X всех точек полосы по сторонам - для столбиков (post_error)."""

    def __init__(self, masks):
        self.masks = masks
        self.lat = {}       # (k, side) -> [|Y|]
        self.seen = {}      # (k, side) -> {индекс шага}
        self.band_x = {"right": [], "left": []}

    def add_scan(self, path_pts, x_robot, cells):
        """cells - [(номер клетки, X её центра)], которые сейчас впереди."""
        for X, Y in path_pts:
            lat = abs(Y)
            if not BAND_LO_MM <= lat <= BAND_HI_MM:
                continue
            side = "right" if Y > 0 else "left"
            self.band_x[side].append(X)
            for k, c in cells:
                if abs(X - c) <= SEG_HALF_MM:
                    self.lat.setdefault((k, side), []).append(lat)
        for k, c in cells:
            for side in ("right", "left"):
                self.seen.setdefault((k, side), set()).update(
                    visible_steps(x_robot, c - SEG_HALF_MM, c + SEG_HALF_MM, side, self.masks))

    def result(self, k, side):
        """(стена: True / проход: False / не уверен: None, точек, видно мм)."""
        lat = sorted(self.lat.get((k, side), []))
        span = len(self.seen.get((k, side), ())) * SPAN_STEP_MM
        n = len(lat)
        if n >= WALL_MIN_PTS and lat[(3 * n) // 4] - lat[n // 4] <= WALL_MAX_IQR_MM:
            return True, n, span
        if n <= OPEN_MAX_PTS and span >= OPEN_MIN_SPAN_MM:
            return False, n, span
        return None, n, span

    def prune(self, x_min):
        """Забыть точки полосы позади x_min - на длинной прямой их копится много."""
        for side in self.band_x:
            self.band_x[side] = [x for x in self.band_x[side] if x >= x_min]


def post_error(xs, boundary, wall_before, wall_after):
    """Ошибка одометрии вдоль хода по столбику (как в micromouse): где боковая
    стена кончается (стена до границы клеток, проход после) или начинается
    (наоборот), против ожидаемой границы. xs - X точек полосы этой стороны.

    Возвращает e в мм (> 0 - стена кончается/начинается дальше, чем ждали:
    робот на самом деле не доехал столько, сколько показывает одометрия,
    цель надо отодвинуть на e) или None: стороны не различаются, у конца
    меньше POST_MIN_PTS точек или |e| > POST_TOL_MM."""
    if wall_before is None or wall_after is None or wall_before == wall_after:
        return None
    near = sorted(x for x in xs if abs(x - boundary) <= SEG_HALF_MM)
    if not near:
        return None
    if wall_before:
        end = near[-1]
        support = [x for x in near if x >= end - 30.0]
        expected = boundary + WALL_HALF_T_MM
    else:
        end = near[0]
        support = [x for x in near if x <= end + 30.0]
        expected = boundary - WALL_HALF_T_MM
    if len(support) < POST_MIN_PTS:
        return None
    e = end - expected
    return e if abs(e) <= POST_TOL_MM else None
