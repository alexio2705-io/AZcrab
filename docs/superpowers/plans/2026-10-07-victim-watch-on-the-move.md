# Чтение меток жертв на ходу — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Пока робот едет по ровному в `drive_cells`, фоновый наблюдатель
смотрит боковыми камерами (только со стороны стены) быстрым YOLO, при находке
робот штатно тормозит, подъезжает к метке, читает её существующей логикой
`scan()`, репортит жертву на месте и продолжает ту же поездку.

**Architecture:** Инференс — в отдельном процессе `watch_proc.py` (свой GIL,
ядра 2–3), камеры остаются в nav: потоки `victim_watch.VictimWatcher` кладут
кадры в общую память. Вся проверяемая без железа логика (px→мм рыбьего глаза,
мировые координаты, реестр меток, решение «стоп», подъезд, таймеры, история
одометрии, статистика) — в `watch_logic.py` под unittest. `nav.drive_cells`
включает наблюдатель, тормозит к метке тем же профилем и после паузы
продолжает поездку без сброса одометрии.

**Tech Stack:** Python 3.13 (Pi 5), picamera2, ultralytics YOLO26n, torch,
numpy, `multiprocessing.shared_memory`, stdlib `unittest`, pyright 1.1.414.

**Spec:** `docs/superpowers/specs/2026-10-07-victim-watch-on-the-move-design.md`

## Global Constraints

- Скорости, пороги торможения, геометрию клетки (`MOVE_*`, `BRAKE_DECEL_CMS2`,
  `RAMP_UP_S`, `STOP_LEAD_S`, `ODO_STALL_S`, `CELL_TIMEOUT_S`, `SIDE_THRESH`,
  `ALIGN_*`, `CELL_MM`…) не менять.
- Протокол ESP↔Pi и прошивку не трогать (`src/`, `include/`, `TELEM_*`, `_cmd`).
- Сканы стоя (`arrive` → `scan_cell`, после поворотов) — как есть; добавляется
  только дедуп по положению. `report_victim` для них не поворачивается обратно.
- `--no-vision` полностью отключает наблюдатель (воркер не запускается).
- Быстрый проход: conf 0,25 (`CONF_FAST`), фильтр `MIN_BOX_PX = 40`;
  выдержка на ходу ≤ 3 мс; цель — 10 Гц на камеру.
- Дедуп по положению: ближе 15 см на той же стене; приписывание клетке:
  пройдено < 150 мм — исходная, иначе следующая; стена = курс ±1.
- Комментарии и сообщения лога — по-русски, в стиле файла. Комментарии
  объясняют «почему», как в окружающем коде.
- Тесты: stdlib `unittest`, `pi/tests/`, запуск из корня репозитория:
  `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`.
- После каждой задачи: тесты зелёные, `pyright` — не больше 16 ошибок
  (исходная база: 16 errors, 26 warnings; новые предупреждения о
  неразрешённых импортах cv2/torch/numpy на Маке допустимы).
- Одна задача — один коммит; сообщение по-английски в стиле истории
  репозитория, последняя строка `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
  В коммит добавлять только файлы своей задачи (`git add <пути>`).

## Review Focus

1. **Знак оси камеры (`CAM_FWD_SIGN`) перепутан** — подъезд уводит от метки.
   Ожидание: после подъезда по рамке повторный кадр сравнивается с первым, при
   удалении от центра — `[WARN] … знак оси камеры?`, чтение всё равно идёт.
   Тест в Задаче 2 закрепляет «рамка выше центра ⇒ подъезд вперёд (>0)».
2. **Воркер умер или замолчал посреди заезда.** Ожидание: `[WARN]`,
   наблюдатель выключен до конца заезда, поездки идут как раньше. Тест с
   молчащим фейковым воркером — Задача 7.
3. **`SensorTimestamp` не в часах `time.monotonic`.** Ожидание: время кадра =
   момент возврата кадра, `[WARN]` один раз. Тест с фейковой камерой,
   отдающей чужие часы, — Задача 7.
4. **Метку заметили, когда робот уже тормозит к центру** (точка остановки за
   центром). Ожидание: штатная остановка в центре, чтение там же, без лишнего
   разгона. Проверяется сценарием «метка на стыке клеток» в полевом плане
   (Задача 13).
5. **Пауза/подъём во время мигания или поворота посреди клетки.** Ожидание:
   сразу выход из `drive_cells` с той же причиной, без выдачи и без поворота
   обратно. Проверяется на поле (Задача 13): нажать BTN_1 во время мигания.

---

## Файлы

| Файл | Что | Задачи |
|---|---|---|
| `pi/watch_logic.py` (новый) | чистая логика | 1–3 |
| `pi/tests/test_watch_logic.py` (новый) | её тесты | 1–3 |
| `pi/dataset_web.py` | `Camera.fast_exposure/restore_exposure/try_latest` | 4 |
| `victim_detect.py` | `read_wall`, `best_fast`, `box_y` в результате | 5 |
| `pi/watch_proc.py` (новый) | процесс-воркер YOLO | 6 |
| `pi/victim_watch.py` (новый) | `VictimWatcher` | 7 |
| `pi/tests/test_victim_watch.py`, `pi/tests/fake_watch_proc.py` (новые) | тест наблюдателя с фейковым воркером | 7 |
| `pi/nav.py` | обёртка `drive_cells`, джиттер, дедуп в `scan_sides`, наблюдатель в цикле, репорт посреди клетки, `main` | 8–12 |

На Pi всё лежит плоско в `~/maze/` (как сейчас `nav.py`, `victim_detect.py`,
`dataset_web.py`, `lidar_proc.py`).

---

### Задача 0: Отделить незакоммиченные правки пользователя

Рабочее дерево содержит чужие незакоммиченные правки в `pi/nav.py`,
`pi/dataset_web.py` и других файлах. Задачи 4, 8–12 правят эти же файлы.

- [ ] **Шаг 1:** Сделать то, что решил пользователь (спросить, если не
  решено): либо пользователь коммитит сам, либо одним коммитом «как есть»:

```bash
git add .gitignore camera_web/app.py camera_web/templates/index.html map_web.py pi/colors_page.py pi/dataset_web.py pi/ld19_web.py pi/nav.py pi/tile_calib.py pi/victim_web.py templates_detect/index.html
git commit -m "WIP from the working tree before victim watch: push-through, turn() result, web pages

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

- [ ] **Шаг 2:** Закоммитить правку спецификации (про `_write_state`) и этот план:

```bash
git add docs/superpowers/specs/2026-10-07-victim-watch-on-the-move-design.md docs/superpowers/plans/2026-10-07-victim-watch-on-the-move.md
git commit -m "Plan: victim watch on the move

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 1: watch_logic — геометрия

**Files:**
- Create: `pi/watch_logic.py`
- Test: `pi/tests/test_watch_logic.py`

**Interfaces:**
- Produces: `FRAME_PX`, `CAM_F_PX`, `CAM_D_MM`, `CAM_FWD_SIGN`, `CAM_BACK_MM`,
  `CELL_MM`, `ATTRIB_MM`; `px_to_mm(cy_frac: float) -> float`;
  `mark_pos_mm(traveled_mm: float, offset_mm: float) -> float`;
  `wall_of(hdg: int, side: str) -> int`; `level_of(cy: int, stride: int) -> int`;
  `world_along(cell: tuple[int,int], hdg: int, wall_dir: int, pos_mm: float, stride: int) -> tuple[tuple, float]`;
  `attribute(cell, next_cell, hdg: int, side: str, traveled_mm: float) -> tuple[tuple[int,int], int]`.

- [ ] **Шаг 1: Написать падающий тест** — `pi/tests/test_watch_logic.py`:

```python
"""Тесты чистой логики наблюдателя меток (pi/watch_logic.py).

Запуск из корня репозитория:
    python3 -m unittest discover -s pi/tests -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import watch_logic as wl  # noqa: E402

STRIDE = 1000


class TestGeometry(unittest.TestCase):
    def test_px_to_mm_center_is_zero(self):
        self.assertAlmostEqual(wl.px_to_mm(0.5), 0.0)

    def test_px_to_mm_edges_are_90mm(self):
        self.assertAlmostEqual(wl.px_to_mm(0.0), 90.0, delta=1.0)
        self.assertAlmostEqual(wl.px_to_mm(1.0), -90.0, delta=1.0)

    def test_px_to_mm_top_is_forward(self):
        # верх кадра - вперёд по ходу (шлейф смотрит назад)
        self.assertGreater(wl.px_to_mm(0.2), 0.0)

    def test_px_to_mm_monotonic_and_fisheye(self):
        vals = [wl.px_to_mm(i / 10) for i in range(11)]
        self.assertTrue(all(a > b for a, b in zip(vals, vals[1:])))
        center_step = vals[5] - vals[6]
        edge_step = vals[0] - vals[1]
        self.assertGreater(edge_step, 1.4 * center_step)   # у края мм/px крупнее

    def test_px_to_mm_quarter(self):
        self.assertAlmostEqual(wl.px_to_mm(0.25), 39.3, delta=0.5)

    def test_mark_pos_accounts_for_camera_back_offset(self):
        self.assertAlmostEqual(wl.mark_pos_mm(200.0, 30.0), 205.0)

    def test_wall_of(self):
        self.assertEqual(wl.wall_of(0, "right"), 1)
        self.assertEqual(wl.wall_of(0, "left"), 3)
        self.assertEqual(wl.wall_of(3, "right"), 0)
        self.assertEqual(wl.wall_of(2, "left"), 1)

    def test_level_of(self):
        self.assertEqual(wl.level_of(0, STRIDE), 0)
        self.assertEqual(wl.level_of(1003, STRIDE), 1)
        self.assertEqual(wl.level_of(-998, STRIDE), -1)

    def test_world_along_heading_north_east_wall(self):
        key, along = wl.world_along((0, 0), 0, 1, 205.0, STRIDE)
        self.assertEqual(key, (1, 0, 0))
        self.assertAlmostEqual(along, 205.0)

    def test_same_mark_from_opposite_directions(self):
        # едем на север из (0,0): метка на восточной стене в 205мм впереди;
        # едем на юг из (0,1): та же метка - левой камерой, в 95мм впереди
        k1, a1 = wl.world_along((0, 0), 0, 1, 205.0, STRIDE)
        k2, a2 = wl.world_along((0, 1), 2, 1, 95.0, STRIDE)
        self.assertEqual(k1, k2)
        self.assertAlmostEqual(a1, a2)

    def test_world_along_east_west_headings_on_north_south_walls(self):
        k1, a1 = wl.world_along((2, 3), 1, 0, 100.0, STRIDE)
        k2, a2 = wl.world_along((3, 3), 3, 0, 200.0, STRIDE)
        self.assertEqual(k1, (0, 3))
        self.assertEqual(k1, k2)
        self.assertAlmostEqual(a1, 700.0)
        self.assertAlmostEqual(a2, 700.0)

    def test_neighbour_cells_share_wall_key(self):
        k1, _ = wl.world_along((0, 0), 0, 1, 0.0, STRIDE)
        k2, _ = wl.world_along((0, 1), 0, 1, 0.0, STRIDE)
        self.assertEqual(k1, k2)

    def test_opposite_faces_differ(self):
        k_east, _ = wl.world_along((0, 0), 0, 1, 0.0, STRIDE)   # восточная (0,0)
        k_west, _ = wl.world_along((1, 0), 0, 3, 0.0, STRIDE)   # западная (1,0)
        self.assertNotEqual(k_east, k_west)

    def test_levels_differ(self):
        k0, a0 = wl.world_along((0, 0), 0, 1, 50.0, STRIDE)
        k1, a1 = wl.world_along((0, 1000), 0, 1, 50.0, STRIDE)
        self.assertNotEqual(k0, k1)
        self.assertAlmostEqual(a0, a1)

    def test_attribute_boundary_150mm(self):
        self.assertEqual(wl.attribute((0, 0), (0, 1), 0, "right", 149.9), ((0, 0), 1))
        self.assertEqual(wl.attribute((0, 0), (0, 1), 0, "right", 150.0), ((0, 1), 1))
        self.assertEqual(wl.attribute((0, 0), (0, 1), 0, "left", 10.0), ((0, 0), 3))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Шаг 2: Запустить — должен упасть**

Run: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`
Expected: ERROR `ModuleNotFoundError: No module named 'watch_logic'`

- [ ] **Шаг 3: Реализация** — `pi/watch_logic.py`:

```python
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
```

- [ ] **Шаг 4: Запустить — зелёные**

Run: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`
Expected: `OK` (15 тестов)

- [ ] **Шаг 5: pyright**

Run: `pyright 2>&1 | tail -1`
Expected: `16 errors, …` (не больше)

- [ ] **Шаг 6: Коммит**

```bash
git add pi/watch_logic.py pi/tests/test_watch_logic.py
git commit -m "watch_logic: fisheye px->mm, wall keys and cell attribution for marks seen on the move

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 2: watch_logic — реестр, решение «стоп», подъезд

**Files:**
- Modify: `pi/watch_logic.py` (дописать в конец)
- Test: `pi/tests/test_watch_logic.py` (дописать классы перед `if __name__`)

**Interfaces:**
- Consumes: `px_to_mm` (Задача 1).
- Produces: `DEDUP_MM`, `CREEP_MAX_MM`, `EDGE_MARGIN`;
  `class MarkRegistry(dedup_mm=DEDUP_MM)` с `add(key, along, kind: 'read'|'rejected')`,
  `near(key, along) -> tuple[key, along, kind] | None`;
  `decide_stop(conf, side_px, key, along, registry, busy, conf_min, min_box_px) -> tuple[bool, str]`
  (причины: `'conf'`, `'мелкая'`, `'уже стоим'`, `'дубль'`, `'отбракована раньше'`, `'стоп'`);
  `stop_point_cm(traveled_cm, v_cms, decel_cms2) -> float`;
  `creep_to_mark(box_y: tuple[float,float] | None, est_mm: float, tol_mm: float, max_mm=CREEP_MAX_MM) -> tuple[float | None, str]`
  (источник `'рамка'` или `'оценка'`).

- [ ] **Шаг 1: Падающие тесты** — дописать в `pi/tests/test_watch_logic.py`:

```python
class TestRegistry(unittest.TestCase):
    def setUp(self):
        self.reg = wl.MarkRegistry()
        self.key = (1, 0, 0)
        self.reg.add(self.key, 205.0, "read")

    def test_near_within_15cm(self):
        self.assertEqual(self.reg.near(self.key, 354.9), (self.key, 205.0, "read"))

    def test_not_near_at_15cm(self):
        self.assertIsNone(self.reg.near(self.key, 355.0))

    def test_other_wall_is_not_near(self):
        self.assertIsNone(self.reg.near((3, 1, 0), 205.0))

    def test_nearest_wins(self):
        self.reg.add(self.key, 300.0, "rejected")
        self.assertEqual(self.reg.near(self.key, 290.0)[2], "rejected")


class TestDecideStop(unittest.TestCase):
    def setUp(self):
        self.reg = wl.MarkRegistry()
        self.key = (1, 0, 0)

    def decide(self, conf=0.6, side_px=60, along=500.0, busy=False):
        return wl.decide_stop(conf, side_px, self.key, along, self.reg, busy, 0.25, 40)

    def test_stop(self):
        self.assertEqual(self.decide(), (True, "стоп"))

    def test_low_conf(self):
        self.assertEqual(self.decide(conf=0.2), (False, "conf"))

    def test_small_box(self):
        self.assertEqual(self.decide(side_px=39), (False, "мелкая"))

    def test_busy(self):
        self.assertEqual(self.decide(busy=True), (False, "уже стоим"))

    def test_duplicate_of_read(self):
        self.reg.add(self.key, 450.0, "read")
        self.assertEqual(self.decide(), (False, "дубль"))

    def test_duplicate_of_rejected(self):
        self.reg.add(self.key, 560.0, "rejected")
        self.assertEqual(self.decide(), (False, "отбракована раньше"))


class TestStopAndCreep(unittest.TestCase):
    def test_stop_point(self):
        self.assertAlmostEqual(wl.stop_point_cm(10.0, 55.0, 300.0), 10.0 + 3025.0 / 600.0)

    def test_stop_point_standing(self):
        self.assertAlmostEqual(wl.stop_point_cm(12.0, 0.0, 300.0), 12.0)

    def test_creep_box_centered_no_move(self):
        self.assertEqual(wl.creep_to_mark((0.45, 0.55), 999.0, 15.0), (0.0, "рамка"))

    def test_creep_box_above_center_goes_forward(self):
        mm, src = wl.creep_to_mark((0.2, 0.4), 0.0, 15.0)
        self.assertEqual(src, "рамка")
        self.assertAlmostEqual(mm, 31.0, delta=0.5)

    def test_creep_box_cut_by_edge(self):
        mm, src = wl.creep_to_mark((0.0, 0.2), 0.0, 15.0)
        self.assertAlmostEqual(mm, wl.px_to_mm(0.1))
        self.assertGreater(mm, 0.0)

    def test_creep_by_estimate_backwards(self):
        self.assertEqual(wl.creep_to_mark(None, -95.0, 15.0), (-95.0, "оценка"))

    def test_creep_too_far_rejected(self):
        self.assertEqual(wl.creep_to_mark(None, 160.0, 15.0), (None, "оценка"))
```

- [ ] **Шаг 2: Запустить — должны упасть**

Run: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`
Expected: ERROR `AttributeError: module 'watch_logic' has no attribute 'MarkRegistry'`

- [ ] **Шаг 3: Реализация** — дописать в конец `pi/watch_logic.py`:

```python
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
```

- [ ] **Шаг 4: Запустить — зелёные**

Run: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`
Expected: `OK` (32 теста)

- [ ] **Шаг 5: pyright** — `pyright 2>&1 | tail -1` → не больше 16 errors.

- [ ] **Шаг 6: Коммит**

```bash
git add pi/watch_logic.py pi/tests/test_watch_logic.py
git commit -m "watch_logic: mark registry, stop decision and creep-to-mark

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 3: watch_logic — таймеры, история одометрии, статистика, стороны

**Files:**
- Modify: `pi/watch_logic.py` (дописать в конец; `import bisect` — к импортам)
- Test: `pi/tests/test_watch_logic.py` (дописать)

**Interfaces:**
- Produces: `WATCH_SIDE_HOLD_S = 0.2`;
  `watch_allowed(ramp_state, bump_on, assist_mode, pitch_delta, rearm_deg, wall_forced) -> bool`;
  `shift_timers(times: tuple[float, ...], pause_s: float) -> tuple[float, ...]`;
  `class OdoHistory(keep_s=1.0)` с `add(t, d)`, `at(t) -> float | None`;
  `class Stats` с `add(x)`, `pct(p) -> float | None`, `fmt() -> str`;
  `class SideHold(hold_s=WATCH_SIDE_HOLD_S)` с `update(t, present: dict[str, bool]) -> set[str]`, `reset()`.

- [ ] **Шаг 1: Падающие тесты** — дописать:

```python
class TestWatchAllowed(unittest.TestCase):
    def ok(self, **kw):
        a = dict(ramp_state=None, bump_on=False, assist_mode=None, pitch_delta=0.5,
                 rearm_deg=1.2, wall_forced=False)
        a.update(kw)
        return wl.watch_allowed(**a)

    def test_flat(self):
        self.assertTrue(self.ok())

    def test_blocked(self):
        for kw in (dict(ramp_state="cand"), dict(ramp_state="exit"), dict(bump_on=True),
                   dict(assist_mode="kick"), dict(pitch_delta=1.3), dict(pitch_delta=-1.3),
                   dict(wall_forced=True)):
            with self.subTest(**kw):
                self.assertFalse(self.ok(**kw))


class TestShiftTimers(unittest.TestCase):
    def test_shift_skips_unarmed_zero(self):
        self.assertEqual(wl.shift_timers((10.0, 0.0, 5.0), 2.5), (12.5, 0.0, 7.5))


class TestOdoHistory(unittest.TestCase):
    def test_empty(self):
        self.assertIsNone(wl.OdoHistory().at(1.0))

    def test_interpolate_and_clamp(self):
        h = wl.OdoHistory()
        h.add(0.0, 0.0)
        h.add(0.1, 5.0)
        h.add(0.2, 10.0)
        self.assertAlmostEqual(h.at(0.15), 7.5)
        self.assertAlmostEqual(h.at(-1.0), 0.0)
        self.assertAlmostEqual(h.at(5.0), 10.0)

    def test_trims_old(self):
        h = wl.OdoHistory(keep_s=1.0)
        for i in range(21):
            h.add(i * 0.1, float(i))
        self.assertGreaterEqual(h.t[0], 2.0 - 1.0 - 1e-9)
        self.assertAlmostEqual(h.at(0.0), h.d[0])


class TestStats(unittest.TestCase):
    def test_percentiles(self):
        s = wl.Stats()
        for i in range(1, 101):
            s.add(float(i))
        self.assertEqual(s.pct(50), 50.0)
        self.assertEqual(s.pct(95), 95.0)
        self.assertIn("max 100.0", s.fmt())
        self.assertIn("n=100", s.fmt())

    def test_empty(self):
        self.assertIsNone(wl.Stats().pct(50))
        self.assertEqual(wl.Stats().fmt(), "—")


class TestSideHold(unittest.TestCase):
    def test_hold(self):
        h = wl.SideHold(0.2)
        self.assertEqual(h.update(0.0, {"right": True, "left": False}), {"right"})
        self.assertEqual(h.update(0.1, {"right": False, "left": False}), {"right"})
        self.assertEqual(h.update(0.25, {"right": False, "left": False}), set())

    def test_reset(self):
        h = wl.SideHold(0.2)
        h.update(0.0, {"left": True})
        h.reset()
        self.assertEqual(h.update(0.05, {"left": False}), set())
```

- [ ] **Шаг 2: Запустить — должны упасть**

Run: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`
Expected: ERROR `AttributeError: module 'watch_logic' has no attribute 'watch_allowed'`

- [ ] **Шаг 3: Реализация** — в `pi/watch_logic.py` к импортам `import bisect`, в конец:

```python
# Сторона включена ещё столько после того, как лидар перестал видеть стену:
# на стыке клеток боковое окно мигает, а камере дёргаться незачем.
WATCH_SIDE_HOLD_S = 0.2


def watch_allowed(ramp_state, bump_on, assist_mode, pitch_delta, rearm_deg, wall_forced):
    """Наблюдатель - только на ровном ходу: не горка и не кандидат в неё, не
    всплеск наклона, не ассист моментом, не экстренное торможение у стены."""
    return (ramp_state is None and not bump_on and assist_mode is None
            and abs(pitch_delta) < rearm_deg and not wall_forced)


def shift_timers(times, pause_s):
    """Отметки времени, сдвинутые на паузу остановки у метки; 0.0 значит
    "не взведено" (так их и заводит drive_cells) - такие не трогаем."""
    return tuple(t + pause_s if t else t for t in times)


class OdoHistory:
    """(время, путь) за последние keep_s: путь на момент экспозиции кадра,
    а не на момент ответа воркера - на 55см/с разница 2-5см."""

    def __init__(self, keep_s=1.0):
        self.keep_s = keep_s
        self.t = []
        self.d = []

    def add(self, t, d):
        self.t.append(t)
        self.d.append(d)
        cut = 0
        while cut < len(self.t) - 1 and self.t[cut] < t - self.keep_s:
            cut += 1
        if cut:
            del self.t[:cut]
            del self.d[:cut]

    def at(self, t):
        if not self.t:
            return None
        if t <= self.t[0]:
            return self.d[0]
        if t >= self.t[-1]:
            return self.d[-1]
        i = bisect.bisect_right(self.t, t)
        t0, t1 = self.t[i - 1], self.t[i]
        d0, d1 = self.d[i - 1], self.d[i]
        return d0 + (d1 - d0) * (t - t0) / (t1 - t0)


class Stats:
    """Выборка для p50/p95/max: частота, задержки, такт цикла."""

    def __init__(self):
        self.v = []

    def add(self, x):
        self.v.append(x)

    def pct(self, p):
        if not self.v:
            return None
        s = sorted(self.v)
        return s[max(0, math.ceil(p / 100.0 * len(s)) - 1)]

    def fmt(self):
        if not self.v:
            return "—"
        return (f"p50 {self.pct(50):.1f} p95 {self.pct(95):.1f} "
                f"max {max(self.v):.1f} (n={len(self.v)})")


class SideHold:
    """Стороны, где есть стена, с удержанием hold_s (см. WATCH_SIDE_HOLD_S)."""

    def __init__(self, hold_s=WATCH_SIDE_HOLD_S):
        self.hold_s = hold_s
        self.last = {}

    def update(self, t, present):
        for side, p in present.items():
            if p:
                self.last[side] = t
        return {s for s, tl in self.last.items() if t - tl <= self.hold_s}

    def reset(self):
        self.last.clear()
```

- [ ] **Шаг 4: Запустить — зелёные**

Run: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`
Expected: `OK` (42 теста)

- [ ] **Шаг 5: pyright** — не больше 16 errors.

- [ ] **Шаг 6: Коммит**

```bash
git add pi/watch_logic.py pi/tests/test_watch_logic.py
git commit -m "watch_logic: pause timer shift, odometry-at-frame history, stats, side hold

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 4: Camera — быстрая выдержка и кадр без ожидания

**Files:**
- Modify: `pi/dataset_web.py` — класс `Camera`, вставить после `auto_exposure()` (≈ стр. 177) и после `latest()` (≈ стр. 270).

**Interfaces:**
- Produces: `Camera.fast_exposure(max_us: int) -> tuple[int, float] | None`;
  `Camera.restore_exposure() -> None`;
  `Camera.try_latest() -> tuple[ndarray | None, dict]`.

- [ ] **Шаг 1: Вставить после `auto_exposure()`:**

```python
    def fast_exposure(self, max_us):
        """Кадры на ходу: выдержка не длиннее max_us (смаз), остаток яркости,
        выбранной автоэкспозицией, - усилением. Наоборот к auto_exposure():
        там шум важнее смаза. Сумма - с текущего кадра; зовётся из потока
        наблюдателя, не из петли управления (capture_metadata ждёт кадр).
        Возвращает (выдержка, усиление) или None - камера занята снимком."""
        if self.cam is None or not self.cap_lock.acquire(timeout=1.0):
            return None
        try:
            md = self.cam.capture_metadata()
            total = (md.get('ExposureTime', 10000) * md.get('AnalogueGain', 1.0)
                     * md.get('DigitalGain', 1.0))
            exp = int(max(100, min(max_us, total)))
            gain = max(1.0, total / exp)
            self.cam.set_controls({'AeEnable': False, 'ExposureTime': exp,
                                   'AnalogueGain': float(gain)})
            return exp, gain
        finally:
            self.cap_lock.release()

    def restore_exposure(self):
        """Назад к camera_settings.json после fast_exposure()."""
        self.apply(cam_settings)
```

- [ ] **Шаг 2: Вставить после `latest()`:**

```python
    def try_latest(self):
        """(кадр, метаданные) без ожидания cap_lock: камера занята снимком
        или его фоновым возвратом (settle_async) - (None, {}), наблюдатель
        просто пропустит кадр. Метаданные нужны ради SensorTimestamp."""
        if self.cam is None or not self.cap_lock.acquire(blocking=False):
            return None, {}
        try:
            req = self.cam.capture_request()
            try:
                return req.make_array('main'), req.get_metadata()
            finally:
                req.release()
        finally:
            self.cap_lock.release()
```

- [ ] **Шаг 3: Проверки** — тесты: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests` → `OK`; `pyright 2>&1 | tail -1` → не больше 16 errors.

- [ ] **Шаг 4: Коммит**

```bash
git add pi/dataset_web.py
git commit -m "Camera: short-exposure mode for frames on the move, non-blocking try_latest

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 5: VictimDetector — read_wall, best_fast, положение рамки

**Files:**
- Modify: `victim_detect.py:108-225` (метод `scan`)

**Interfaces:**
- Produces: `VictimDetector.read_wall(cell_x, cell_y, wall_dir) -> dict | None`
  (результат как у `scan()`, плюс ключ `'box_y': (y1, y2)` — рамка в долях
  высоты кадра); `VictimDetector.scan()` — прежнее поведение (дедуп + `read_wall`);
  `VictimDetector.best_fast() -> tuple[str, float, tuple[float, float]] | None`.

- [ ] **Шаг 1: Разделить scan.** Строки `key = (cell_x, cell_y, wall_dir)` …
  `self.scanned.add(key)` (≈120–123) остаются в `scan()`, после них — вызов
  `read_wall`; весь остальной код тела `scan()` (от комментария
  `# ── Проход 1` до `return result`) переносится без изменений в новый метод
  `read_wall` сразу под `scan()`. Итог начала:

```python
    def scan(self, cell_x: int, cell_y: int, wall_dir=None):
        """
        (прежний docstring без изменений)
        """
        key = (cell_x, cell_y, wall_dir)
        if key in self.scanned:
            return None
        self.scanned.add(key)
        return self.read_wall(cell_x, cell_y, wall_dir)

    def read_wall(self, cell_x: int, cell_y: int, wall_dir=None):
        """То же чтение, что в scan(), но без дедупа по (клетка, стена): по
        сигналу наблюдателя на ходу стену исходной клетки читаем ещё раз -
        стоя в центре её уже смотрели, но метка была вне кадра. В результате
        дополнительно 'box_y' - (y1, y2) рамки в долях высоты кадра, по нему
        nav.py считает, где метка вдоль стены (дедуп по положению)."""
        # ── Проход 1: обычный кадр — по нему решаем, что за метка ──
        ...перенесённый код без изменений...
```

- [ ] **Шаг 2: Добавить `box_y` в результат.** В `read_wall` в словаре
  `result = {` после строки `'checksum': checksum,` добавить:

```python
            'box_y':    (round(xyxy[1] / still.shape[0], 4), round(xyxy[3] / still.shape[0], 4)),
```

- [ ] **Шаг 3: Добавить `best_fast` после `read_wall`:**

```python
    def best_fast(self):
        """Стоя, после остановки по сигналу наблюдателя: свежий кадр 320,
        лучшая рамка conf >= CONF_FAST не мельче MIN_BOX_PX. Возвращает
        (класс, уверенность, (y1, y2) в долях высоты кадра) или None."""
        self.cam.latest()                  # сброс кадра, снятого ещё на ходу
        frame = self.cam.latest()
        found = self._best(frame, CONF_FAST)
        if found is None:
            return None
        cls_name, conf, box = found
        if max(box[2] - box[0], box[3] - box[1]) < MIN_BOX_PX:
            return None
        h = frame.shape[0]
        return cls_name, conf, (box[1] / h, box[3] / h)
```

- [ ] **Шаг 4: Проверки** — тесты `OK`; `pyright 2>&1 | tail -1` → не больше 16 errors.
  Вручную: `git diff victim_detect.py` — тело чтения перенесено целиком, кроме
  трёх строк дедупа; поведение `scan()` прежнее.

- [ ] **Шаг 5: Коммит**

```bash
git add victim_detect.py
git commit -m "VictimDetector: read_wall() without wall dedup, best_fast(), box position in result

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 6: watch_proc.py — процесс-воркер YOLO

**Files:**
- Create: `pi/watch_proc.py`

**Interfaces:**
- Produces (протокол для Задачи 7): запуск
  `python -u watch_proc.py МОДЕЛЬ ПАПКА CONF idx:shm_name [idx:shm_name]`;
  stdin — строки `"idx fid arm"`; stdout — сначала `{"ready": true}`, затем на
  каждый запрос `{"cam": idx, "id": fid, "arm": arm, "boxes": [{"cls": str, "conf": float, "xyxy": [x1,y1,x2,y2]}], "ms": float, "img": str|null}`.
  Кадр — `uint8` 320×320×3 BGR в общей памяти камеры `idx`.

- [ ] **Шаг 1: Создать `pi/watch_proc.py`:**

```python
#!/usr/bin/env python3
"""YOLO наблюдателя меток на ходу - отдельным процессом (см. victim_watch.py).

Свой GIL и свои ядра: петля drive_cells в nav.py (200 Гц) и поток телеметрии
не делят интерпретатор с инференсом. Раньше после YOLO в том же процессе
лидар отдавал пустой перед - именно из-за этого. Ядра 2-3, nice +5, 2 потока
torch: ядра 0-1 остаются nav.py и lidar_proc.py.

Запуск (это делает victim_watch.py):
    watch_proc.py МОДЕЛЬ ПАПКА CONF idx:shm [idx:shm]
stdin:  "idx номер_кадра номер_включения" - кадр 320x320 BGR уже лежит в
        общей памяти камеры idx (0 - правая, 1 - левая, как cam_idx);
stdout: {"ready": true} после прогрева, дальше по строке JSON на запрос:
        {"cam", "id", "arm", "boxes": [{"cls", "conf", "xyxy"}], "ms", "img"}.
Каждый кадр с рамкой сохраняется в ПАПКУ с нарисованными рамками - по ним
подбирают порог (каждая лишняя остановка стоит ~2с).
"""
import json
import os
import sys
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'

from multiprocessing import shared_memory  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402

IMGSZ = 320
FRAME_SHAPE = (IMGSZ, IMGSZ, 3)
TORCH_THREADS = 2
CPU_CORES = {2, 3}
NICE = 5
CAM_TAG = {0: 'r', 1: 'l'}


def save(frame, boxes, save_dir, idx):
    """Кадр с рамками в папку детекций; имя - время, камера, лучший класс."""
    shown = frame.copy()
    for b in boxes:
        x1, y1, x2, y2 = (int(v) for v in b['xyxy'])
        cv2.rectangle(shown, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.putText(shown, f"{b['cls']} {b['conf']:.2f}", (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    best = max(boxes, key=lambda b: b['conf'])
    t = time.time()
    name = (f"w_{time.strftime('%H-%M-%S', time.localtime(t))}.{int(t * 1000) % 1000:03d}"
            f"_{CAM_TAG.get(idx, idx)}_{best['cls']}_{best['conf']:.2f}.jpg")
    cv2.imwrite(os.path.join(save_dir, name), shown)
    return name


def main():
    model_path, save_dir, conf = sys.argv[1], sys.argv[2], float(sys.argv[3])
    slots = {}
    for arg in sys.argv[4:]:
        idx, name = arg.split(':', 1)
        # track=False: память создал и освободит nav.py, не наш resource_tracker
        slots[int(idx)] = shared_memory.SharedMemory(name=name, track=False)
    frames = {i: np.ndarray(FRAME_SHAPE, np.uint8, buffer=s.buf) for i, s in slots.items()}

    try:
        os.sched_setaffinity(0, CPU_CORES)
    except (AttributeError, OSError) as e:
        print(f'[VW] привязка к ядрам {CPU_CORES} не удалась: {e}', file=sys.stderr, flush=True)
    os.nice(NICE)
    import torch
    torch.set_num_threads(TORCH_THREADS)
    from ultralytics import YOLO
    model = YOLO(model_path)
    os.makedirs(save_dir, exist_ok=True)
    model.predict(np.zeros(FRAME_SHAPE, np.uint8), imgsz=IMGSZ, conf=conf, verbose=False)
    print(json.dumps({'ready': True}), flush=True)

    for line in sys.stdin:
        try:
            idx, fid, arm = (int(v) for v in line.split())
        except ValueError:
            continue
        if idx not in frames:
            continue
        frame = frames[idx].copy()
        t0 = time.monotonic()
        res = model.predict(frame, imgsz=IMGSZ, conf=conf, verbose=False)[0]
        ms = (time.monotonic() - t0) * 1000.0
        boxes = [{'cls': str(model.names[int(b.cls[0])]),
                  'conf': round(float(b.conf[0]), 3),
                  'xyxy': [round(float(v), 1) for v in b.xyxy[0].tolist()]}
                 for b in res.boxes]
        img = save(frame, boxes, save_dir, idx) if boxes else None
        print(json.dumps({'cam': idx, 'id': fid, 'arm': arm, 'boxes': boxes,
                          'ms': round(ms, 1), 'img': img}), flush=True)


if __name__ == '__main__':
    main()
```

- [ ] **Шаг 2: Проверки** — `/opt/homebrew/bin/python3 -m py_compile pi/watch_proc.py`
  (без вывода); тесты `OK`; `pyright 2>&1 | tail -1` → не больше 16 errors.

- [ ] **Шаг 3: Коммит**

```bash
git add pi/watch_proc.py
git commit -m "watch_proc: YOLO worker process for the on-the-move victim watcher

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 7: VictimWatcher (сторона nav) и его тест с фейковым воркером

**Files:**
- Create: `pi/victim_watch.py`
- Create: `pi/tests/fake_watch_proc.py`
- Test: `pi/tests/test_victim_watch.py`

**Interfaces:**
- Consumes: протокол `watch_proc.py` (Задача 6); `Camera.fast_exposure`,
  `restore_exposure`, `try_latest` (Задача 4); `watch_logic.Stats` (Задача 3).
- Produces: `@dataclass WatchBox(side: str, cls: str, conf: float, xyxy: tuple, t_frame: float, image: str | None)`;
  `VictimWatcher(cams: dict[str, Camera], model_path: str, save_dir: str, conf: float, script: str | None = None)`
  с атрибутами `dead: bool`, `ready: threading.Event` и методами
  `arm(sides: set[str]) -> bool` (вызывать каждый такт; True — хоть одна
  сторона включена), `poll() -> list[WatchBox]`, `wait_idle(timeout=IDLE_WAIT_S)`,
  `take_stats() -> str | None`, `close()`.

Тестам нужен numpy, на Маке его нет — ставим во временное окружение в
scratchpad (не в проект). Без numpy тест сам пропускается.

- [ ] **Шаг 1: Окружение для теста**

```bash
python3 -m venv "$SCRATCH/venv" && "$SCRATCH/venv/bin/pip" install -q numpy
```
(`$SCRATCH` — каталог scratchpad сессии.)

- [ ] **Шаг 2: Фейковый воркер** — `pi/tests/fake_watch_proc.py`:

```python
"""Фейковый watch_proc.py для test_victim_watch.py: тот же протокол, без YOLO.
FAKE_SILENT=1 - готов, но на запросы не отвечает (воркер завис)."""
import json
import os
import sys

print(json.dumps({'ready': True}), flush=True)
for line in sys.stdin:
    idx, fid, arm = (int(v) for v in line.split())
    if os.environ.get('FAKE_SILENT'):
        continue
    print(json.dumps({'cam': idx, 'id': fid, 'arm': arm, 'ms': 1.0, 'img': 'w_test.jpg',
                      'boxes': [{'cls': 'phi', 'conf': 0.9, 'xyxy': [100, 100, 160, 160]}]}),
          flush=True)
```

- [ ] **Шаг 3: Падающий тест** — `pi/tests/test_victim_watch.py`:

```python
"""VictimWatcher с фейковой камерой и фейковым воркером (нужен numpy)."""
import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

try:
    import numpy as np
except ImportError:          # на Маке без окружения из плана - пропуск
    np = None

FAKE = os.path.join(HERE, 'fake_watch_proc.py')


class FakeCam:
    def __init__(self, ts_offset_s=0.0):
        self.ts_offset_s = ts_offset_s
        self.fast = 0
        self.restored = 0

    def fast_exposure(self, max_us):
        self.fast += 1
        return max_us, 2.0

    def restore_exposure(self):
        self.restored += 1

    def try_latest(self):
        ts = int((time.monotonic() + self.ts_offset_s) * 1e9)
        return np.zeros((320, 320, 3), np.uint8), {'SensorTimestamp': ts}


def wait_for(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


@unittest.skipIf(np is None, 'нужен numpy')
class TestVictimWatcher(unittest.TestCase):
    def make(self, cam=None):
        from victim_watch import VictimWatcher
        self.cam = cam or FakeCam()
        w = VictimWatcher({'right': self.cam}, 'model.pt', '/tmp', 0.25, script=FAKE)
        self.addCleanup(w.close)
        self.assertTrue(w.ready.wait(5.0))
        return w

    def test_boxes_arrive_with_frame_time(self):
        w = self.make()
        t_arm = time.monotonic()
        self.assertTrue(w.arm({'right'}))
        got = []
        self.assertTrue(wait_for(lambda: got.extend(w.poll()) or got))
        b = got[0]
        self.assertEqual((b.side, b.cls), ('right', 'phi'))
        self.assertGreaterEqual(b.t_frame, t_arm - 0.05)
        self.assertEqual(self.cam.fast, 1)

    def test_unknown_side_and_disarm_restore(self):
        w = self.make()
        self.assertFalse(w.arm({'left'}))          # левой камеры нет
        w.arm({'right'})
        self.assertTrue(wait_for(lambda: self.cam.fast == 1))
        w.arm(set())
        w.wait_idle(1.0)
        self.assertEqual(self.cam.restored, 1)

    def test_stale_results_dropped_after_rearm(self):
        w = self.make()
        w.arm({'right'})
        time.sleep(0.3)
        w.arm(set())
        w.wait_idle(1.0)
        time.sleep(0.1)                             # ответы старого включения пришли
        t_rearm = time.monotonic()
        w.arm({'right'})
        for b in w.poll():
            self.assertGreaterEqual(b.t_frame, t_rearm - 0.05)

    def test_foreign_clock_falls_back_to_now(self):
        w = self.make(FakeCam(ts_offset_s=-1000.0))
        w.arm({'right'})
        got = []
        self.assertTrue(wait_for(lambda: got.extend(w.poll()) or got))
        self.assertLess(abs(got[0].t_frame - time.monotonic()), 1.0)

    def test_silent_worker_marks_dead(self):
        os.environ['FAKE_SILENT'] = '1'
        self.addCleanup(os.environ.pop, 'FAKE_SILENT', None)
        w = self.make()
        w.arm({'right'})
        self.assertTrue(wait_for(lambda: (w.poll(), w.dead)[1], timeout=4.0))
        self.assertFalse(w.arm({'right'}))

    def test_stats_line(self):
        w = self.make()
        w.arm({'right'})
        time.sleep(0.5)
        w.poll()
        line = w.take_stats()
        self.assertIsNotNone(line)
        self.assertIn('right', line)
        self.assertIn('Гц', line)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Шаг 4: Запустить — должен упасть**

Run: `"$SCRATCH/venv/bin/python" -m unittest discover -s pi/tests -v`
Expected: ERROR `ModuleNotFoundError: No module named 'victim_watch'`

- [ ] **Шаг 5: Реализация** — `pi/victim_watch.py`:

```python
"""Наблюдатель меток жертв на ходу - сторона nav.py.

Кадры снимают потоки здесь (picamera2 не открыть из двух процессов, а скан
стоя - VictimDetector в nav.py - пользуется теми же камерами), а YOLO крутится
в отдельном процессе watch_proc.py: со своим GIL и на своих ядрах. Цена для
nav.py - копия 300 КБ в общую память и строка в pipe на кадр.

nav.py на каждом такте drive_cells говорит arm(стороны) и забирает рамки
poll(). Кадр с камеры - не чаще WATCH_HZ и только когда воркер закончил
предыдущий кадр этой камеры: очереди нет, всегда свежий кадр. На время
включения камера переходит на короткую выдержку (Camera.fast_exposure), при
выключении - обратно, до любого снимка стоя.

Метка на 55см/с целиком в кадре ~0,24с: 10 Гц - это 2-3 кадра на метку.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from multiprocessing import shared_memory

import numpy as np

from watch_logic import Stats

WATCH_HZ = 10.0
FAST_EXPOSURE_US = 3000        # смаз на 55см/с ~1,7мм
RESPONSE_TIMEOUT_S = 2.0       # воркер молчит дольше - считаем мёртвым
IDLE_WAIT_S = 0.2
TS_SKEW_MAX_S = 0.5            # SensorTimestamp дальше от monotonic - чужие часы
FRAME_SHAPE = (320, 320, 3)
SIDE_IDX = {"right": 0, "left": 1}   # как cam_idx в victim_detect.py


@dataclass
class WatchBox:
    side: str
    cls: str
    conf: float
    xyxy: tuple
    t_frame: float          # time.monotonic() кадра
    image: str | None       # картинка в папке детекций (сохранил воркер)


class VictimWatcher:
    def __init__(self, cams, model_path, save_dir, conf, script=None):
        self.cams = dict(cams)           # {"right"/"left": Camera}
        self.dead = False
        self.ready = threading.Event()
        self._closing = False
        self._lock = threading.Lock()
        self._armed = {s: False for s in self.cams}
        self._wake = {s: threading.Event() for s in self.cams}
        self._idle = {s: threading.Event() for s in self.cams}
        self._free = {s: threading.Event() for s in self.cams}   # воркер свободен для камеры
        self._pending = {}               # side -> (fid, t_frame, arm_id, t_sent)
        self._out = []                   # (arm_id, t_frame, [WatchBox])
        self._arm_id = 0
        self._any_armed = False
        self._fid = 0
        self._ts_warned = False
        self._failed = set()
        self._frames_n = {s: 0 for s in self.cams}
        self._armed_s = {s: 0.0 for s in self.cams}
        self._armed_t = {s: None for s in self.cams}
        self.latency = Stats()
        self.infer = Stats()
        self._t_start = time.monotonic()

        nbytes = int(np.prod(FRAME_SHAPE))
        self._shm = {s: shared_memory.SharedMemory(create=True, size=nbytes) for s in self.cams}
        self._buf = {s: np.ndarray(FRAME_SHAPE, np.uint8, buffer=self._shm[s].buf)
                     for s in self.cams}
        script = script or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "watch_proc.py")
        slots = [f"{SIDE_IDX[s]}:{self._shm[s].name}" for s in self.cams]
        self._proc = subprocess.Popen(
            [sys.executable, "-u", script, model_path, save_dir, str(conf), *slots],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        for s in self.cams:
            self._idle[s].set()
            self._free[s].set()
        threading.Thread(target=self._reader, daemon=True).start()
        for s in self.cams:
            threading.Thread(target=self._cam_loop, args=(s,), daemon=True).start()

    # ── управление из петли drive_cells ─────────────────────────
    def arm(self, sides):
        """Какие стороны смотреть (каждый такт). Возвращает, включено ли хоть
        что-то. Переход "ничего" -> "что-то" - новый номер включения: ответы
        воркера на кадры прошлого включения poll() отбросит."""
        if self.dead or not self.ready.is_set():
            sides = set()
        sides = set(sides) & (set(self.cams) - self._failed)
        now = time.monotonic()
        with self._lock:
            any_now = bool(sides)
            if any_now and not self._any_armed:
                self._arm_id += 1
            self._any_armed = any_now
            for s in self.cams:
                on = s in sides
                if on == self._armed[s]:
                    continue
                self._armed[s] = on
                if on:
                    self._armed_t[s] = now
                else:
                    t = self._armed_t[s]
                    if t is not None:
                        self._armed_s[s] += now - t
                    self._armed_t[s] = None
                self._wake[s].set()
        return any_now

    def poll(self):
        """Рамки, пришедшие с прошлого вызова (только текущего включения).
        Здесь же - сторож воркера: процесс умер или молчит дольше
        RESPONSE_TIMEOUT_S - наблюдатель выключается до конца заезда."""
        now = time.monotonic()
        with self._lock:
            out, self._out = self._out, []
            arm_id = self._arm_id
            stale = any(self._armed[s] and now - p[3] > RESPONSE_TIMEOUT_S
                        for s, p in self._pending.items())
        if self._proc.poll() is not None:
            self._die(f"воркер завершился (код {self._proc.returncode})")
        elif stale:
            self._die(f"воркер не ответил за {RESPONSE_TIMEOUT_S:.0f}с")
        boxes = []
        for a, t_frame, bxs in out:
            if a != arm_id:
                continue
            self.latency.add((now - t_frame) * 1000.0)
            boxes.extend(bxs)
        return boxes

    def wait_idle(self, timeout=IDLE_WAIT_S):
        """Ждёт, пока выключенные камеры отдадут кадр и вернут выдержку."""
        end = time.monotonic() + timeout
        for s in self.cams:
            self._idle[s].wait(max(0.0, end - time.monotonic()))

    def take_stats(self):
        """Строка статистики с прошлого вызова или None, если не включался."""
        now = time.monotonic()
        with self._lock:
            parts = []
            for s in self.cams:
                t = self._armed_s[s]
                if self._armed_t[s] is not None:
                    t += now - self._armed_t[s]
                    self._armed_t[s] = now
                if t > 0.05:
                    parts.append(f"{s} {self._frames_n[s] / t:.1f}Гц ({self._frames_n[s]} кадров)")
                self._armed_s[s] = 0.0
                self._frames_n[s] = 0
            lat, inf = self.latency, self.infer
            self.latency, self.infer = Stats(), Stats()
        if not parts:
            return None
        return (f"    [VW] {', '.join(parts)}; кадр→рамки {lat.fmt()} мс; "
                f"инференс {inf.fmt()} мс")

    def close(self):
        self._closing = True
        self.arm(set())
        self.dead = True
        for s in self.cams:
            self._wake[s].set()
        self.wait_idle(1.0)
        try:
            if self._proc.stdin is not None:
                self._proc.stdin.close()
            self._proc.terminate()
            self._proc.wait(timeout=1.0)
        except (OSError, subprocess.TimeoutExpired):
            self._proc.kill()
        self._buf.clear()               # иначе shm.close(): "exported pointers exist"
        for shm in self._shm.values():
            try:
                shm.close()
                shm.unlink()
            except (OSError, BufferError):
                pass

    # ── потоки ──────────────────────────────────────────────────
    def _die(self, why):
        if self.dead:
            return
        self.dead = True
        with self._lock:
            for s in self.cams:
                self._armed[s] = False
                self._wake[s].set()
        if not self._closing:
            print(f"\n  [WARN] наблюдатель меток: {why} - выключен до конца заезда", flush=True)

    def _fail(self, side, e):
        self._failed.add(side)
        with self._lock:
            self._armed[side] = False
        print(f"\n  [WARN] наблюдатель меток: камера {side} - {e}; сторона выключена",
              flush=True)

    def _frame_time(self, md):
        now = time.monotonic()
        ts = md.get("SensorTimestamp")
        if ts:
            t = ts / 1e9
            if abs(now - t) <= TS_SKEW_MAX_S:
                return t
        if not self._ts_warned:
            self._ts_warned = True
            print(f"\n  [WARN] наблюдатель меток: SensorTimestamp не в часах monotonic "
                  f"({ts}) - время кадра = момент его получения", flush=True)
        return now

    def _reader(self):
        idx_side = {v: k for k, v in SIDE_IDX.items()}
        stdout = self._proc.stdout
        assert stdout is not None
        for line in stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if msg.get("ready"):
                print(f"[VW] воркер готов за {time.monotonic() - self._t_start:.1f}с", flush=True)
                self.ready.set()
                continue
            side = idx_side.get(msg.get("cam"))
            if side not in self.cams:
                continue
            with self._lock:
                p = self._pending.get(side)
                if p is None or p[0] != msg.get("id"):
                    continue
                del self._pending[side]
                _, t_frame, arm_id, _ = p
                self.infer.add(float(msg.get("ms", 0.0)))
                boxes = [WatchBox(side, str(b["cls"]), float(b["conf"]),
                                  tuple(b["xyxy"]), t_frame, msg.get("img"))
                         for b in msg.get("boxes", [])]
                self._out.append((arm_id, t_frame, boxes))
            self._free[side].set()
        self._die("воркер закрыл вывод")

    def _cam_loop(self, side):
        cam = self.cams[side]
        fast = False
        log_exposure = None
        next_t = 0.0
        while not self.dead and side not in self._failed:
            if not self._armed[side]:
                if fast:
                    try:
                        cam.restore_exposure()
                    except Exception as e:
                        self._fail(side, e)
                        break
                    fast = False
                self._idle[side].set()
                self._wake[side].wait(0.5)
                self._wake[side].clear()
                continue
            self._idle[side].clear()
            try:
                if not fast:
                    got = cam.fast_exposure(FAST_EXPOSURE_US)
                    if got is None:
                        time.sleep(0.05)
                        continue
                    fast, log_exposure = True, got
                now = time.monotonic()
                if now < next_t:
                    time.sleep(next_t - now)
                    continue
                if not self._free[side].wait(0.1):
                    continue
                frame, md = cam.try_latest()
            except Exception as e:
                self._fail(side, e)
                break
            if frame is None:
                time.sleep(0.02)
                continue
            next_t = time.monotonic() + 1.0 / WATCH_HZ
            t_frame = self._frame_time(md)
            if log_exposure is not None:
                exp, gain = log_exposure
                log_exposure = None
                print(f"\n    [VW] {side}: выдержка {exp}мкс x{gain:.1f}, "
                      f"яркость {float(frame.mean()):.0f}", flush=True)
            with self._lock:
                if not self._armed[side] or self.dead:
                    continue
                self._fid += 1
                fid, arm_id = self._fid, self._arm_id
                self._buf[side][...] = frame
                self._pending[side] = (fid, t_frame, arm_id, time.monotonic())
                self._free[side].clear()
                self._frames_n[side] += 1
            try:
                stdin = self._proc.stdin
                assert stdin is not None
                stdin.write(f"{SIDE_IDX[side]} {fid} {arm_id}\n")
                stdin.flush()
            except (OSError, ValueError):
                self._die("pipe воркера закрыт")
                break
        if fast:
            try:
                cam.restore_exposure()
            except Exception:
                pass
        self._idle[side].set()
```

- [ ] **Шаг 6: Запустить — зелёные**

Run: `"$SCRATCH/venv/bin/python" -m unittest discover -s pi/tests -v`
Expected: `OK` (42 + 6 тестов). И без numpy:
`/opt/homebrew/bin/python3 -m unittest discover -s pi/tests` → `OK (skipped=6)`.

- [ ] **Шаг 7: pyright** — не больше 16 errors.

- [ ] **Шаг 8: Коммит**

```bash
git add pi/victim_watch.py pi/tests/fake_watch_proc.py pi/tests/test_victim_watch.py
git commit -m "VictimWatcher: camera threads feed the YOLO worker through shared memory

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 8: nav — drive_cells(cell, hdg), джиттер такта, общие глобалы

**Files:**
- Modify: `pi/nav.py`: импорты (≈84–88), глобалы у `_scanned_walls` (≈923),
  `level_of` (≈1881), `drive_cells` (≈2569–2570, цикл ≈2721–2723),
  вызовы в `flood_fill_run` (≈3690) и `return_to_start` (≈3860), `main` finally.

**Interfaces:**
- Consumes: `watch_logic.Stats`, `watch_logic.MarkRegistry`, `watch_logic.level_of`.
- Produces: `drive_cells(max_cells, expect_ramp=False, cell=None, hdg=None)`;
  глобалы `_watcher` (None), `_marks: MarkRegistry`, `_tick_ms: dict[str, Stats]`;
  `tick_report() -> str`; внутри `_drive_cells` переменная `watch_armed` (bool).

- [ ] **Шаг 1: Импорты.** После `import tile_classify`:

```python
import watch_logic
```
и строку `from victim_detect import VictimDetector` заменить на:

```python
from victim_detect import VictimDetector, CONF_FAST, MIN_BOX_PX, YOLO_MODEL_PATH, SAVE_DIR
```

- [ ] **Шаг 2: Глобалы** — сразу после `_scanned_walls = set()`:

```python
# Наблюдатель меток на ходу (victim_watch.py): None при --no-vision, если не
# поднялся или умер. _marks - где на стенах уже прочитаны (или отбракованы)
# метки: повтор ближе 15см не останавливает робота и не репортится ни на
# ходу, ни при скане стоя (см. watch_logic.MarkRegistry).
_watcher = None
_marks = watch_logic.MarkRegistry()
# Интервал между тактами drive_cells, мс - с наблюдателем и без: им видно,
# мешает ли инференс петле управления (итог - в конце заезда).
_tick_ms = {"вкл": watch_logic.Stats(), "выкл": watch_logic.Stats()}
```

- [ ] **Шаг 3: `level_of`** — тело заменить на (формула та же, общая с watch_logic):

```python
    return watch_logic.level_of(cy, LEVEL_STRIDE)
```

- [ ] **Шаг 4: Обёртка.** Строки

```python
@_timed("ход")
def drive_cells(max_cells, expect_ramp=False):
```
заменить на:

```python
@_timed("ход")
def drive_cells(max_cells, expect_ramp=False, cell=None, hdg=None):
    """См. _drive_cells. cell, hdg - клетка, из которой едем, и курс по карте:
    без них наблюдатель меток на ходу не включается (victim_watch.py). Что бы
    ни остановило поездку, наблюдатель после неё выключен."""
    try:
        return _drive_cells(max_cells, expect_ramp, cell, hdg)
    finally:
        if _watcher is not None:
            _watcher.arm(set())
            line = _watcher.take_stats()
            if line:
                print(line)


def _drive_cells(max_cells, expect_ramp, cell, hdg):
```
(docstring прежнего `drive_cells` остаётся на `_drive_cells`.)

- [ ] **Шаг 5: Джиттер.** Перед `t0 = time.monotonic()` (≈2689) добавить
  `watch_armed = False`, а строки

```python
        now_m = time.monotonic()
        loop_dt = min(0.05, now_m - last_loop_t)
```
заменить на:

```python
        now_m = time.monotonic()
        _tick_ms["вкл" if watch_armed else "выкл"].add((now_m - last_loop_t) * 1000.0)
        loop_dt = min(0.05, now_m - last_loop_t)
```

- [ ] **Шаг 6: `tick_report`** — сразу после `time_report()`:

```python
def tick_report():
    """Такт drive_cells с наблюдателем меток и без - см. _tick_ms."""
    return "  ⏱ такт drive_cells, мс: " + "; ".join(
        f"наблюдатель {k}: {s.fmt()}" for k, s in _tick_ms.items())
```
и в `main()` в `finally` сразу после
`print(time_report({}, time.monotonic() - t_run, prefix="⏱ весь заезд"))`:

```python
        print(tick_report())
```

- [ ] **Шаг 7: Вызовы.** В `flood_fill_run` и `return_to_start` строку
  `n, reason = drive_cells(1, expect_ramp=(cx, cy, hdg) in _ramps)` заменить на:

```python
        n, reason = drive_cells(1, expect_ramp=(cx, cy, hdg) in _ramps,
                                cell=(cx, cy), hdg=hdg)
```

- [ ] **Шаг 8: Проверки** — тесты `OK`; `pyright 2>&1 | tail -1` → не больше
  16 errors; `grep -n "drive_cells(" pi/nav.py` — оба вызова с `cell=`.

- [ ] **Шаг 9: Коммит**

```bash
git add pi/nav.py
git commit -m "nav: drive_cells takes cell and heading, loop tick jitter stats

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 9: nav — дедуп по положению при скане стоя

**Files:**
- Modify: `pi/nav.py` — `scan_sides` (≈3411–3428), новая функция `_mark_world` перед `scan_sides`.

**Interfaces:**
- Consumes: `result['box_y']` (Задача 5), `_marks` (Задача 8), `watch_logic.px_to_mm/mark_pos_mm/world_along`.
- Produces: `_mark_world(result: dict, cell, hdg: int, wall_dir: int, traveled_mm: float) -> tuple[key, float]`.

- [ ] **Шаг 1: `_mark_world`** — перед `@_timed("камеры")` у `scan_sides`:

```python
def _mark_world(result, cell, hdg, wall_dir, traveled_mm):
    """(ключ стены, мм вдоль неё) прочитанной метки - по её рамке на кадре;
    traveled_mm - сколько робот проехал от центра cell курсом hdg (стоя в
    центре - 0)."""
    y1, y2 = result["box_y"]
    pos = watch_logic.mark_pos_mm(traveled_mm, watch_logic.px_to_mm((y1 + y2) / 2.0))
    return watch_logic.world_along(cell, hdg, wall_dir, pos, LEVEL_STRIDE)
```

- [ ] **Шаг 2: Дедуп** — в `scan_sides`, в цикле `for side, det, wall_dir in jobs:`
  сразу после `if result is None: continue` вставить:

```python
        # Та же метка могла быть прочитана на ходу из соседней клетки (или
        # с другой стороны стыка) - второй раз не репортим.
        key, along = _mark_world(result, (cx, cy), hdg, wall_dir, 0.0)
        dup = _marks.near(key, along)
        if dup is not None and dup[2] == "read":
            print(f"    [W] {side}: {result['label']} в {along:.0f}мм - уже прочитана "
                  f"рядом ({dup[1]:.0f}мм), не репортю")
            continue
        _marks.add(key, along, "read")
```

- [ ] **Шаг 3: Проверки** — тесты `OK`; pyright не больше 16 errors.

- [ ] **Шаг 4: Коммит**

```bash
git add pi/nav.py
git commit -m "nav: standing scans skip marks already read within 15 cm on the same wall

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 10: nav — наблюдатель в цикле drive_cells: стоп, подъезд, чтение, продолжение

**Files:**
- Modify: `pi/nav.py` — `_drive_cells` (подготовка перед циклом ≈2685, история
  одометрии ≈2724, блок наблюдателя после якоря лидара ≈2902, `mark_first`
  перед `rem_eff` ≈2999, ветка остановки ≈3001, профиль ≈3041); новые функции
  `_interrupt_reason`, `_watch_decide`, `_reject_mark`, `_mark_stop` после `align_at_wall`.

**Interfaces:**
- Consumes: `_watcher.arm/poll/wait_idle/dead` (Задача 7), `det.best_fast/read_wall/scanned`
  (Задача 5), `_mark_world` (Задача 9), `watch_logic.*` (Задачи 1–3).
- Produces: `_interrupt_reason() -> str | None`;
  `_mark_stop(m: dict, cell, hdg, target_yaw, base_cm) -> str | None`.

- [ ] **Шаг 1: Вспомогательные функции** — после `align_at_wall` (до `KITS_BY_TYPE`):

```python
def _interrupt_reason():
    """Причина бросить поездку посреди остановки у метки - те же, что
    проверяет такт drive_cells."""
    if _lifted.is_set():
        return "lifted"
    if get_esp_paused():
        return "paused"
    if not telem_fresh():
        return "link_lost"
    return None


def _watch_decide(b, cell, hdg, odo_t, busy):
    """Рамка наблюдателя -> метка, ради которой тормозить ({"side", "pos_mm"}),
    или None. Каждое срабатывание - строка [W] с картинкой: по ним подбирают
    порог, лишняя остановка стоит ~2с."""
    x1, y1, x2, y2 = b.xyxy
    side_px = max(x2 - x1, y2 - y1)
    t_cm = odo_t.at(b.t_frame) or 0.0
    off = watch_logic.px_to_mm((y1 + y2) / 2.0 / watch_logic.FRAME_PX)
    pos = watch_logic.mark_pos_mm(t_cm * 10.0, off)
    key, along = watch_logic.world_along(cell, hdg, watch_logic.wall_of(hdg, b.side),
                                         pos, LEVEL_STRIDE)
    stop, why = watch_logic.decide_stop(b.conf, side_px, key, along, _marks, busy,
                                        CONF_FAST, MIN_BOX_PX)
    print(f"\n    [W] {b.side} {b.cls} {b.conf:.2f} {side_px:.0f}px  путь {t_cm * 10:.0f}мм  "
          f"метка {pos:+.0f}мм (стена {key} {along:.0f}мм) → {why}  {b.image or ''}")
    return {"side": b.side, "pos_mm": pos} if stop else None


def _reject_mark(cell, hdg, side, pos_mm):
    """Наблюдатель сработал, а стоя не подтвердилось - больше здесь не стоим."""
    key, along = watch_logic.world_along(cell, hdg, watch_logic.wall_of(hdg, side),
                                         pos_mm, LEVEL_STRIDE)
    _marks.add(key, along, "rejected")


@_timed("камеры")
def _mark_stop(m, cell, hdg, target_yaw, base_cm):
    """Остановка на метку, замеченную на ходу (см. drive_cells): найти её стоя,
    подъехать так, чтобы была целиком в кадре, прочитать read_wall() и
    приписать клетке. Робот уже затормозил профилем; одометрия не обнулялась,
    base_cm - пройдено от центра исходной клетки. Возвращает причину бросить
    поездку ('paused'/'lifted'/'link_lost') или None - ехать дальше."""
    side = m["side"]
    det = _victim_r if side == "right" else _victim_l
    l0, r0 = get_odo()

    def trav_mm():
        cl, cr = get_odo()
        return (base_cm + ((cl - l0) + (cr - r0)) / 2.0) * 10.0

    settle_brake(0.06)
    if _watcher is not None:
        _watcher.wait_idle()
    r = _interrupt_reason()
    if r or det is None:
        return r

    def look():
        try:
            return det.best_fast()
        except Exception as e:
            print(f"    [WARN] кадр стоя ({side}): {e}")
            return None

    # 1. Где метка сейчас. Обычно робот проехал её на тормозном пути и в
    # кадре её уже нет - тогда подъезд по координате, запомненной на ходу.
    found = look()
    est = m["pos_mm"] - watch_logic.mark_pos_mm(trav_mm(), 0.0)
    creep, src = watch_logic.creep_to_mark(None if found is None else found[2], est,
                                           ALIGN_TOL_MM)
    if creep is None:
        print(f"    метка: нужен подъезд {est:+.0f}мм ({src}) - не верю, отбраковываю")
        _reject_mark(cell, hdg, side, m["pos_mm"])
        return None
    # 2. Подъезд
    if creep:
        print(f"    метка: подъезд {creep:+.0f}мм по {src}")
        creep_mm(creep, target_yaw)
        r = _interrupt_reason()
        if r:
            return r
        if found is not None:
            again = look()
            if again is not None:
                before = abs(watch_logic.px_to_mm(sum(found[2]) / 2.0))
                after = abs(watch_logic.px_to_mm(sum(again[2]) / 2.0))
                if after > before:
                    print(f"    [WARN] после подъезда метка дальше от центра кадра "
                          f"({before:.0f} -> {after:.0f}мм) - знак оси камеры? "
                          f"(watch_logic.CAM_FWD_SIGN)")
    # 3. Чтение - та же логика, что стоя в центре, без дедупа по стене
    trav = trav_mm()
    tcell, wall_dir = watch_logic.attribute(cell, _next_cell(cell[0], cell[1], hdg),
                                            hdg, side, trav)
    try:
        result = det.read_wall(tcell[0], tcell[1], wall_dir)
    except Exception as e:
        print(f"    [WARN] чтение метки ({side}): {e}")
        result = None
    if result is None:
        print(f"    метка не подтвердилась - место отбраковано ({side}, {trav:.0f}мм)")
        _reject_mark(cell, hdg, side, m["pos_mm"])
        return _interrupt_reason()
    key, along = _mark_world(result, cell, hdg, wall_dir, trav)
    _marks.add(key, along, "read")
    _scanned_walls.add((tcell[0], tcell[1], wall_dir))
    det.scanned.add((tcell[0], tcell[1], wall_dir))
    _victim_log.append({**result, "side": side, "wall_dir": wall_dir, "on_move": True,
                        "ts": time.strftime("%H:%M:%S")})
    print(f"    метка на ходу: {result['label']} → клетка {tcell}, стена "
          f"{_DIR_NAME[wall_dir]} (проехали {trav:.0f}мм)")
    return _interrupt_reason()
```

- [ ] **Шаг 2: Подготовка перед циклом** — после `forced_from_cm = 0.0`
  (и рядом с `watch_armed = False` из Задачи 8):

```python
    # Наблюдатель меток на ходу (victim_watch.py): только на ровном и с той
    # стороны, где лидар видит стену. vmark - метка, ради которой тормозим
    # ({"side", "pos_mm", "stop_cm"}); пока она есть, наблюдатель выключен.
    watch_on = (_watcher is not None and cell is not None and hdg is not None
                and not expect_ramp)
    side_hold = watch_logic.SideHold()
    odo_t = watch_logic.OdoHistory()
    vmark = None
```

- [ ] **Шаг 3: История пути** — сразу после строки `odo_hist.append((now_m, traveled_cm))`:

```python
        odo_t.add(now_m, traveled_cm)
```

- [ ] **Шаг 4: Блок наблюдателя** — сразу после вычисления
  `f_est = None if f_anchor is None else …` (перед комментарием
  `# Не глушится и на склоне`):

```python
        # ── Наблюдатель меток ─────────────────────────────────
        # Только на ровном (не горка, не всплеск, не ассист) и только к
        # стене. Заметил - тормозим тем же профилем к stop_point (ниже, см.
        # mark_first), а наблюдатель гасим сразу: камера вернётся к обычной
        # выдержке, пока робот ещё тормозит.
        if watch_on:
            if _watcher.dead:
                watch_on = watch_armed = False
            else:
                sides = side_hold.update(now_m, {
                    "right": wall_present(walls["right"], SIDE_THRESH),
                    "left": wall_present(walls["left"], SIDE_THRESH)})
                flat = watch_logic.watch_allowed(ramp_state, bump_on, assist_mode,
                                                 pitch_delta, OBSTACLE_REARM_DEG, wall_forced)
                watch_armed = _watcher.arm(sides if flat and vmark is None else set())
                for b in _watcher.poll():
                    m = _watch_decide(b, cell, hdg, odo_t, vmark is not None)
                    if m is not None:
                        m["stop_cm"] = watch_logic.stop_point_cm(traveled_cm, fwd,
                                                                 BRAKE_DECEL_CMS2)
                        vmark = m
                        watch_armed = _watcher.arm(set())
                        print(f"    метка на ходу: торможу, встану через "
                              f"{(m['stop_cm'] - traveled_cm) * 10:.0f}мм")
```

- [ ] **Шаг 5: `mark_first`** — перед строкой `rem_eff = rem - fwd * STOP_LEAD_S`:

```python
        # Точка остановки у метки ближе цели - тормозим к ней. Иначе робот
        # штатно встанет в центре (или у стены), и метку читаем там же.
        mark_first = vmark is not None and vmark["stop_cm"] - traveled_cm < rem
        if mark_first:
            rem = vmark["stop_cm"] - traveled_cm
```

- [ ] **Шаг 6: Ветка остановки.** Условие
  `if stopping and rem_eff <= 0.0 and ramp_state != 'cand':` заменить на
  `if (stopping or mark_first) and rem_eff <= 0.0 and ramp_state != 'cand':`
  и первой строкой тела (перед `if wall_forced and traveled_cm < …`) вставить:

```python
            if vmark is not None and not (wall_forced
                                          and traveled_cm < forced_from_cm + cell_cm * 0.5):
                # Стоим у метки (или в центре, если он был ближе): читаем её
                # здесь, потом - та же поездка дальше, без Z и без новой цели.
                t_stop = time.monotonic()
                out = _mark_stop(vmark, cell, hdg, target_yaw, traveled_cm)
                vmark = None
                if out in ("lifted", "paused"):
                    return cells, out
                if out == "link_lost":
                    reason = out
                    break
                pause_s = time.monotonic() - t_stop
                # Таймер клетки и сторожа застревания - на длительность паузы,
                # иначе после неё сразу timeout/odo_fault.
                (t_cell, last_progress_t, assist_gap_until, assist_t0, assist_total_t0,
                 assist_progress_t) = watch_logic.shift_timers(
                    (t_cell, last_progress_t, assist_gap_until, assist_t0, assist_total_t0,
                     assist_progress_t), pause_s)
                # Подъезд и развороты уже в энкодерах - добираем их в путь сразу.
                cl, cr = get_odo()
                traveled_cm += ((cl - last_l) + (cr - last_r)) / 2.0
                last_l, last_r = cl, cr
                last_progress_cm = traveled_cm
                t0 = last_loop_t = time.monotonic()     # новый плавный разгон с места
                fwd = last_steer = 0.0
                odo_hist.clear()
                f_anchor = None                         # перед - заново со свежего оборота
                last_seq = _scan_cnt
                side_hold.reset()
                left_cm = boundary_cm - traveled_cm
                print(f"    продолжаю: пауза {pause_s:.1f}с, до центра {left_cm * 10:.0f}мм")
                if stopping and left_cm * 10.0 < ALIGN_TOL_MM:
                    # Уже в центре или подъезд увёл за него - остаток доводкой.
                    if abs(left_cm) > 0.2:
                        creep_mm(left_cm * 10.0, target_yaw)
                    cells += 1
                    break
                telem_seq = wait_telem(telem_seq)
                continue
```

- [ ] **Шаг 7: Профиль** — строку `if stopping and ramp_state != 'cand':` в
  блоке `# ── Профиль скорости` заменить на
  `if (stopping or mark_first) and ramp_state != 'cand':`.

- [ ] **Шаг 8: Выключение после цикла** — сразу после выхода из `while True:`
  (перед комментарием `# Гребень уже позади`):

```python
    if watch_on:
        _watcher.arm(set())     # доводке и скану стоя камера нужна в обычном режиме
```

- [ ] **Шаг 9: Проверки** — тесты `OK`; `pyright 2>&1 | tail -1` → не больше
  16 errors; `/opt/homebrew/bin/python3 -m py_compile pi/nav.py`. Пока
  `_watcher` нигде не создаётся (Задача 12), поведение прежнее.

- [ ] **Шаг 10: Коммит**

```bash
git add pi/nav.py
git commit -m "nav: stop for marks seen on the move, read them and resume the same drive

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 11: nav — жертва посреди клетки

**Files:**
- Modify: `pi/nav.py` — `report_victim` (≈3462–3472), новые `_blink_victim`,
  `_report_on_move`; конец `_mark_stop` (Задача 10).

**Interfaces:**
- Consumes: `_interrupt_reason` (Задача 10), `turn`, `turn_to`, `dispense_kit`, `DIR_DELTA`.
- Produces: `_blink_victim(result, side) -> int`;
  `_report_on_move(result, side, wall_dir, hdg, target_yaw) -> str | None`.

- [ ] **Шаг 1: `_blink_victim`** — перед `@_timed("жертва")` у `report_victim`:

```python
def _blink_victim(result, side):
    """Индикация лентой (как требуют правила - 5+с неподвижно). Возвращает,
    сколько комплектов выдавать: сколько нужно, но не больше, чем осталось."""
    need = KITS_BY_TYPE.get(result["type"], 0)
    n_kits = min(need, kits_left())
    short = f" (нужно {need}, в запасе {kits_left()})" if n_kits < need else ""
    print(f"    ▓ {result['label']} ({result['type']}, {side}): индикация "
          f"{VICTIM_STOP_S + MARGIN_S:.1f}с, аптечек={n_kits}{short}")
    if _leds is not None:
        hold_while(blink_pink, _leds, stop=get_esp_paused)
    return n_kits
```

- [ ] **Шаг 2: `report_victim`** — строки от `need = KITS_BY_TYPE.get(...)` до
  `hold_while(blink_pink, _leds, stop=get_esp_paused)` включительно заменить на:

```python
    n_kits = _blink_victim(result, side)
```

- [ ] **Шаг 3: `_report_on_move`** — сразу после `report_victim`:

```python
@_timed("жертва")
def _report_on_move(result, side, wall_dir, hdg, target_yaw):
    """Жертва, найденная посреди клетки (см. _mark_stop): мигание; если есть
    что выдавать - спиной к стене, выдача и поворот ровно обратно на
    target_yaw, потому что поездка продолжается. align_at_wall не нужен:
    робот и так в ~14см от стены. Возвращает причину бросить поездку или None."""
    n_kits = _blink_victim(result, side)
    r = _interrupt_reason()
    if r or n_kits == 0:
        return r
    ok = turn(DIR_DELTA[((wall_dir + 2) % 4 - hdg) % 4])
    r = _interrupt_reason()
    if r:
        return r
    if ok:
        for _ in range(n_kits):
            r = _interrupt_reason()
            if r:
                return r
            dispense_kit()
    else:
        print("    не развернулся спиной к метке - комплекты не выдаю")
    for _ in range(2):
        if turn_to(target_yaw):
            return None
        r = _interrupt_reason()
        if r:
            return r
    print("    [WARN] не вернулся на курс - доведёт руль drive_cells")
    return None
```

- [ ] **Шаг 4: Вызов** — в `_mark_stop` последнюю строку `return _interrupt_reason()`
  (после `print(f"    метка на ходу: …")`) заменить на:

```python
    r = _interrupt_reason()
    if r:
        return r
    if result.get("blink"):
        return _report_on_move(result, side, wall_dir, hdg, target_yaw)
    return None
```

- [ ] **Шаг 5: Проверки** — тесты `OK`; pyright не больше 16 errors;
  `git diff pi/nav.py` — `report_victim` выводит ту же строку `▓ …` и мигает так же.

- [ ] **Шаг 6: Коммит**

```bash
git add pi/nav.py
git commit -m "nav: report victims found mid-cell and turn back to the drive heading

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 12: nav — запуск и остановка наблюдателя в main

**Files:**
- Modify: `pi/nav.py` — импорт, `main()` (≈3986, блок `if use_vision:` ≈4060, после
  `wait_start_button()`, `finally`), шапка модуля (абзац про `--no-vision` ≈52–56).

**Interfaces:**
- Consumes: `VictimWatcher` (Задача 7), `YOLO_MODEL_PATH`, `SAVE_DIR`, `CONF_FAST` (Задача 8).

- [ ] **Шаг 1: Импорт** — после `import watch_logic`:

```python
from victim_watch import VictimWatcher
```

- [ ] **Шаг 2: `global`** — в `main()` строку
  `global _link, YAW_ORIGIN, _victim_r, _victim_l, _leds` дополнить `, _watcher`.

- [ ] **Шаг 3: Запуск** — в `if use_vision:` сразу после блока левой камеры
  (перед `print("Инициализация ленты индикации...")`):

```python
        cams = {s: d.cam for s, d in (("right", _victim_r), ("left", _victim_l))
                if d is not None}
        if cams:
            # Воркер грузит YOLO в своём процессе, пока здесь поднимается
            # лента и ждём кнопку; arm() до его готовности ничего не делает.
            print("Запуск наблюдателя меток на ходу (YOLO отдельным процессом)...")
            try:
                _watcher = VictimWatcher(cams, YOLO_MODEL_PATH, SAVE_DIR, CONF_FAST)
            except Exception as e:
                print(f"  [WARN] наблюдатель не поднялся: {e}")
                _watcher = None
```

- [ ] **Шаг 4: Готовность** — сразу после `wait_start_button()`:

```python
    if _watcher is not None and not _watcher.ready.is_set():
        print("  [WARN] наблюдатель меток ещё грузится - включится, когда будет готов")
```

- [ ] **Шаг 5: Остановка** — в `finally` перед `if _victim_r is not None: _victim_r.close()`:

```python
        if _watcher is not None:
            _watcher.close()
```

- [ ] **Шаг 6: Шапка** — в docstring модуля после предложения про `--no-vision`
  дописать абзац:

```text
На ходу те же камеры смотрит наблюдатель меток (victim_watch.py, YOLO -
отдельным процессом watch_proc.py): только на ровном и со стороны стены.
Заметил - робот штатно тормозит, подъезжает к метке, читает её как стоя
(VictimDetector.read_wall), репортит жертву на месте и доезжает до центра
клетки той же поездкой. --no-vision выключает и его.
```

- [ ] **Шаг 7: Проверки** — тесты `OK`; `pyright 2>&1 | tail -1` → не больше 16 errors;
  `/opt/homebrew/bin/python3 -m py_compile pi/nav.py pi/victim_watch.py pi/watch_proc.py pi/watch_logic.py`.

- [ ] **Шаг 8: Коммит**

```bash
git add pi/nav.py
git commit -m "nav: start the on-the-move victim watcher with the detectors, close it on exit

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Задача 13: Проверка перед «готово» и полевой план

- [ ] **Шаг 1:** superpowers:verification-before-completion — заново прогнать:
  `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests -v`,
  `"$SCRATCH/venv/bin/python" -m unittest discover -s pi/tests -v`,
  `pyright 2>&1 | tail -1`, `py_compile` всех изменённых `.py`.
- [ ] **Шаг 2:** `/code-review high` на весь дифф от коммита спецификации
  (`git diff a626a52..HEAD`), исправить подтверждённое отдельными коммитами.
- [ ] **Шаг 3:** Дать пользователю полевой план (в чат, команды в локальной
  форме для SSH-сессии на Pi, venv `~/maze/venv`): копирование файлов,
  калибровка `CAM_F_PX`/`CAM_D_MM`/`CAM_FWD_SIGN` линейкой, сценарии — метка
  у края стены, метка на стыке двух клеток, жертва с 2 аптечками посреди
  клетки, проезд без меток (ложные остановки, время клетки по `⏱`),
  `--no-vision`, пауза BTN_1 во время мигания; что смотреть в логе
  (`[W]`, `[VW]`, `метка на ходу`, `продолжаю: пауза`, `⏱ такт drive_cells`,
  `[WARN]`), и где картинки (`~/maze/detections/w_*.jpg`).
