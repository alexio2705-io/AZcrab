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
