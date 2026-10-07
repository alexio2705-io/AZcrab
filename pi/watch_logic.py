"""Чистая логика наблюдателя меток жертв на ходу (victim_watch.py, nav.py).

Без железа и без torch - импортируется где угодно, покрыта
pi/tests/test_watch_logic.py. Дизайн: docs/superpowers/specs/
2026-10-07-victim-watch-on-the-move-design.md.

Координаты. Положение метки вдоль стены считается в мировых мм, а стена
задаётся ключом (направление, линия сетки, этаж): одна и та же стена
соседних клеток - один ключ, и метка на стыке клеток не задваивается.
Вдоль стен E/W ось - на север (+y), вдоль стен N/S - на восток (+x).
"""
from __future__ import annotations

import math

CELL_MM = 300.0

# Камеры: OV5647 с рыбьим глазом 130°, кадр 320x320. Шлейф смотрит назад,
# поэтому ось "вдоль стены" - вертикаль кадра, верх кадра - вперёд по ходу.
# Пересчёт смещения в мм нелинейный: s = D*tan(u/F), u - пиксели от центра
# кадра. F и D подобраны под "с центра клетки в кадр помещается ~180мм
# стены" (край кадра = ±90мм) и уточняются линейкой на поле.
FRAME_PX = 320
CAM_F_PX = 235.0       # px на радиан
CAM_D_MM = 111.0       # объектив -> стена, мм
CAM_FWD_SIGN = +1      # +1: верх кадра - вперёд; перепутан - см. [WARN] в nav.py
CAM_BACK_MM = 25.0     # объектив сдвинут назад от центра робота

# Метка на ходу - клетке, над которой робот: пройдено меньше этого от
# центра исходной клетки - она, иначе следующая.
ATTRIB_MM = 150.0


def px_to_mm(cy_frac):
    """Смещение точки кадра вдоль стены от оси объектива, мм (вперёд > 0).
    cy_frac - y в долях высоты кадра (0 - верх): годится для любого
    разрешения, и 320, и 960."""
    u = (0.5 - cy_frac) * FRAME_PX * CAM_FWD_SIGN
    return CAM_D_MM * math.tan(u / CAM_F_PX)


def mark_pos_mm(traveled_mm, offset_mm):
    """Где метка вдоль хода, мм от центра исходной клетки: центр робота
    проехал traveled_mm, объектив на CAM_BACK_MM сзади него, метка смещена
    от оси объектива на offset_mm (см. px_to_mm)."""
    return traveled_mm - CAM_BACK_MM + offset_mm


def wall_of(hdg, side):
    """Абсолютное направление стены, на которую смотрит камера side."""
    return (hdg + (1 if side == "right" else 3)) % 4


def level_of(cy, stride):
    """Этаж клетки по её cy (см. LEVEL_STRIDE в nav.py)."""
    return int(math.floor((cy + stride / 2) / stride))


def world_along(cell, hdg, wall_dir, pos_mm, stride):
    """(ключ стены, мировая координата вдоль неё в мм) для метки на стене
    wall_dir, которая в pos_mm (mark_pos_mm) от центра cell по ходу hdg.
    wall_dir - боковая стена хода, wall_of(hdg, ...)."""
    cx, cy = cell
    sgn = 1 if hdg in (0, 1) else -1          # N и E - по оси, S и W - против
    if wall_dir in (1, 3):                    # стены E/W, ось вдоль - y
        level = level_of(cy, stride)
        key = (wall_dir, cx, level)
        center = (cy - level * stride) * CELL_MM
    else:                                     # стены N/S, ось вдоль - x
        key = (wall_dir, cy)
        center = cx * CELL_MM
    return key, center + sgn * pos_mm


def attribute(cell, next_cell, hdg, side, traveled_mm):
    """Метка, прочитанная на ходу: (клетка, стена). Клетка - по одометрии
    робота (см. ATTRIB_MM), стена - сбоку, со стороны камеры."""
    return (cell if traveled_mm < ATTRIB_MM else next_cell), wall_of(hdg, side)


DEDUP_MM = 150.0       # повтор ближе этого на той же стене - та же метка
CREEP_MAX_MM = 150.0   # подъезд к метке дальше этого - оценке не верим
EDGE_MARGIN = 0.10     # "рамка целиком в кадре" - с таким полем от краёв


class MarkRegistry:
    """Где на стенах уже есть метки: 'read' - прочитана (на ходу или стоя),
    'rejected' - наблюдатель сработал, а стоя ничего не подтвердилось. Ни
    то, ни другое больше не останавливает робота (каждая лишняя остановка -
    ~2с), а прочитанная ещё и не репортится второй раз."""

    def __init__(self, dedup_mm=DEDUP_MM):
        self.dedup_mm = dedup_mm
        self.marks = []          # (ключ стены, мм вдоль неё, вид)

    def add(self, key, along, kind):
        self.marks.append((key, along, kind))

    def near(self, key, along):
        """Ближайшая запись на той же стене ближе dedup_mm или None."""
        best = None
        for k, a, kind in self.marks:
            d = abs(a - along)
            if k == key and d < self.dedup_mm and (best is None or d < abs(best[1] - along)):
                best = (k, a, kind)
        return best


def decide_stop(conf, side_px, key, along, registry, busy, conf_min, min_box_px):
    """Тормозить ли ради рамки наблюдателя: (да/нет, причина для лога).
    min_box_px - тот же фильтр дальних стен, что в VictimDetector.scan()."""
    if conf < conf_min:
        return False, "conf"
    if side_px < min_box_px:
        return False, "мелкая"
    if busy:
        return False, "уже стоим"
    dup = registry.near(key, along)
    if dup is not None:
        return False, "дубль" if dup[2] == "read" else "отбракована раньше"
    return True, "стоп"


def stop_point_cm(traveled_cm, v_cms, decel_cms2):
    """Где встанет робот, если тормозить с v_cms с постоянным замедлением."""
    return traveled_cm + v_cms * v_cms / (2.0 * decel_cms2)


def creep_to_mark(box_y, est_mm, tol_mm, max_mm=CREEP_MAX_MM):
    """Сколько подъехать, чтобы метка встала в центр кадра: (мм или None,
    если такому подъезду не верим; источник). box_y - (y1, y2) рамки со
    стоячего кадра в долях высоты или None (метки в кадре уже нет - обычно
    робот проехал её на тормозном пути); тогда est_mm - оценка по
    координате, запомненной на ходу."""
    if box_y is not None:
        y1, y2 = box_y
        off = px_to_mm((y1 + y2) / 2.0)
        inside = y1 >= EDGE_MARGIN and y2 <= 1.0 - EDGE_MARGIN
        mm, src = (0.0 if inside and abs(off) < tol_mm else off), "рамка"
    else:
        mm, src = est_mm, "оценка"
    if abs(mm) > max_mm:
        return None, src
    return mm, src
