# Остановки только на поворотах — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Разведка и возврат на старт проезжают прямые участки (известные и новые клетки) без остановок; робот встаёт только там, где нужен поворот, на синей клетке или когда не уверен в стенах.

**Architecture:** `lidar_proc.py` отдаёт точки боковых коридоров; чистый `corridor.py` переводит их в систему пути и копит по клеткам впереди «стена/проход/не уверен» и столбики; чистый `runplan.py` (BFS из nav) решает «прямо или стоп»; `drive_cells` в точке решения спрашивает план, а проеханные клетки записывает в карту на ходу.

**Tech Stack:** Python 3.11 (Pi venv), stdlib `unittest`, pyright 1.1.414.

**Spec:** `docs/superpowers/specs/2026-10-07-nonstop-straights-design.md`

## Global Constraints

- Скорости и пороги (`MOVE_*`, `BRAKE_DECEL_CMS2`, `MARK_BRAKE_DECEL_CMS2`, `SIDE_THRESH`, `WALL_THRESH`, `DECIDE_LEAD_MM`, маски лидара) не менять.
- Протокол ESP↔Pi и прошивку не трогать.
- Поворот — стоя в центре клетки; там `arrive` (замер стоя, плитка, скан камерами) как сейчас.
- `drive_cells` без `plan` ведёт себя как сейчас (`challenge.py`).
- Порог «стена»: ≥ 8 точек в полосе 100..210 мм вбок, межквартильный разброс ≤ 20 мм; «проход»: видно ≥ 120 мм из 200, точек ≤ 2; иначе «не уверен» → стоп.
- Поправка по столбику: |e| ≤ 40 мм, сдвиг на e/2, не чаще раза на границу клеток.
- Тесты: `/opt/homebrew/bin/python3 -m unittest discover -s pi/tests` (на Маке), pyright — не больше 16 errors. Комментарии/лог — по-русски. Один шаг — один коммит, `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Горка, начавшаяся до точки решения** (кандидат в горку при `rem ≤ DECIDE_LEAD_MM`): решение откладывается до выяснения, иначе клетку перед горкой записали бы посещённой. Проверка — полевой план, сценарий «горка посреди прямой».
2. **Известная горка дальше по прямой**: план останавливается перед её ребром, чтобы следующий заезд шёл с `expect_ramp`. Тест в Задаче 4 (`_DrivePlan` не тестируется на Маке — логика `go_straight` с цепочкой рёбер; горка — в полевом плане).
3. **Возврат на старт**: цель — сама клетка N → стоп, а не поиск старта дальше. Тест `test_home_goal_at_start_stops` (Задача 3).
4. **Лидар не прислал точек** (процесс перезапускается): у N нет данных → «не уверен» → стоп. Тест `test_no_points_is_unsure` (Задача 1).
5. **Ошибка курса на ходу** (робот довернулся на несколько градусов): точки поворачиваются в систему пути. Тест `test_to_path_frame_rotates_by_yaw_error` (Задача 1).

---

### Задача 1: corridor.py — точки в системе пути, видимость, стороны клеток

**Files:**
- Create: `pi/corridor.py`
- Test: `pi/tests/test_corridor.py`

**Interfaces:**
- Produces: константы `CELL_MM, HALF_CELL_MM, WALL_LAT_MM, BAND_LO_MM, BAND_HI_MM, SEG_HALF_MM, VIEW_MAX_MM, SPAN_STEP_MM, WALL_MIN_PTS, WALL_MAX_IQR_MM, OPEN_MIN_SPAN_MM, OPEN_MAX_PTS, POST_TOL_MM, POST_MIN_PTS, CORR_X_MIN, CORR_X_MAX, CORR_Y_MIN, CORR_Y_MAX`;
  `corridor_points(filt: dict[int, float]) -> list[list[int]]`;
  `point_time(t_end, trev, x, y) -> float`;
  `to_path_frame(pts, t_end, trev, travel_at, yaw_err_deg) -> list[tuple[float, float]]`;
  `visible_steps(x_robot, seg_lo, seg_hi, side, masks) -> set[int]`;
  `class CellSides(masks)`: `add_scan(path_pts, x_robot, cells: list[tuple[int, float]])`, `result(k, side) -> tuple[bool | None, int, float]`, `band_x: dict[str, list[float]]`, `prune(x_min)`.
  `masks` — последовательность `(center_deg, half_width_deg)`.

- [ ] **Шаг 1: Тест** — `pi/tests/test_corridor.py`: синтетический оборот генерируется лучами по 1° до прямых стен `Y=±150` на заданных отрезках, с масками как в `walls.MASKS`; проверки: `to_path_frame` (статично и с ошибкой курса), `visible_steps` (с позиции 190 видно ≥ 120 мм средней части следующей клетки, с позиции 0 — меньше), `CellSides.result` (стена → True, проход → False, мало данных → None, нет точек → None), `corridor_points` (фильтр коридора).
- [ ] **Шаг 2:** запустить — `ModuleNotFoundError: corridor`.
- [ ] **Шаг 3:** реализация `pi/corridor.py` (код — в коммите задачи; формулы как в спецификации: `X = travel(t_точки) + x·cos a − y·sin a`, `Y = y·cos a + x·sin a`, время точки `t_end − trev·(1 − ((угол−180) mod 360)/360)`).
- [ ] **Шаг 4:** тесты зелёные, pyright ≤ 16.
- [ ] **Шаг 5:** коммит `corridor: side walls of cells ahead from lidar points on the move`.

### Задача 2: corridor.py — столбики

**Files:** Modify `pi/corridor.py`; Test `pi/tests/test_corridor.py`

**Interfaces:** Produces `post_error(xs: list[float], boundary: float, wall_before: bool | None, wall_after: bool | None) -> float | None` (e > 0 — конец/начало стены дальше ожидаемой границы).

- [ ] **Шаг 1: Тест:** стена кончается на 465 при границе 450 → 15; начинается на 440 → −10; |e| = 50 → None; одинаковые стороны → None; < 3 точек у конца → None.
- [ ] **Шаг 2:** RED (`AttributeError`).
- [ ] **Шаг 3:** реализация.
- [ ] **Шаг 4:** зелёные, pyright ≤ 16.
- [ ] **Шаг 5:** коммит `corridor: odometry error from wall ends at cell boundaries`.

### Задача 3: runplan.py — BFS и решение «прямо/стоп»; nav.find_path_bfs — обёртка

**Files:** Create `pi/runplan.py`; Test `pi/tests/test_runplan.py`; Modify `pi/nav.py` (`find_path_bfs`).

**Interfaces:** Produces `bfs_path(start, hdg, open_dirs, next_cell, goal_test, side_order) -> list[int] | None`; `go_straight(start, hdg, open_dirs, next_cell, goal_test, side_order) -> bool`.

- [ ] **Шаг 1: Тест** на лабиринте-словаре: путь как у nav (L-образный), коридор → прямо, справа открыто → стоп (правая рука первой), тупик → стоп, `test_home_goal_at_start_stops` (цель — сама start → стоп), нет маршрута → стоп.
- [ ] **Шаг 2:** RED.
- [ ] **Шаг 3:** `runplan.py`; `nav.find_path_bfs` → `return runplan.bfs_path(start, hdg, open_dirs, _next_cell, goal_test, SIDE_ORDER[_side_pref])` (docstring остаётся).
- [ ] **Шаг 4:** зелёные, `py_compile nav.py`, pyright ≤ 16.
- [ ] **Шаг 5:** коммит `runplan: BFS out of nav.py, straight-or-stop decision`.

### Задача 4: watch_logic — индекс клетки по пути; nav — `_cell_at`

**Files:** Modify `pi/watch_logic.py` (`attribute` → `cell_index`), `pi/tests/test_watch_logic.py`, `pi/nav.py` (`_mark_stop`, новая `_cell_at`).

**Interfaces:** Produces `watch_logic.cell_index(traveled_mm) -> int` (`floor((traveled_mm + ATTRIB_MM) / CELL_MM)`); `nav._cell_at(cell, hdg, k) -> tuple[int, int]`.

- [ ] **Шаг 1: Тест:** 0→0, 149.9→0, 150→1, 449.9→1, 450→2, −10→0; тест `attribute` удаляется.
- [ ] **Шаг 2:** RED. **Шаг 3:** реализация; `_mark_stop`: `tcell = _cell_at(cell, hdg, watch_logic.cell_index(trav))`, `wall_dir = watch_logic.wall_of(hdg, side)`.
- [ ] **Шаг 4:** зелёные, pyright ≤ 16. **Шаг 5:** коммит `Marks read on the move are attributed to any cell along a multi-cell drive`.

### Задача 5: точки коридора из lidar_proc в nav

**Files:** Modify `pi/walls.py` (`filtered`, `measure(rev, filt=None)`), `pi/lidar_proc.py` (`"pts"`, `"trev"`), `pi/nav.py` (`_lidar_thread`, `_scan_pts`, `_new_scans`, `_LIDAR_MASKS`).

**Interfaces:** Produces `nav._scan_pts: deque[(cnt, t_end, trev, pts)]`, `nav._new_scans(after_cnt) -> list`, `nav._LIDAR_MASKS: tuple[(center, hw)]`.

- [ ] Без unit-теста (железо); проверки: `py_compile`, pyright ≤ 16, на Pi — `lidar_proc.py` печатает строку с `pts` (стоя).
- [ ] Коммит `lidar_proc: corridor points per revolution for nav`.

### Задача 6: drive_cells с планом

**Files:** Modify `pi/nav.py` (`drive_cells`/`_drive_cells`).

**Interfaces:** Consumes `corridor.*`, `_new_scans`, `_LIDAR_MASKS`; plan-объект с методами `known_side(k, side) -> bool|None`, `decide(k, sides_res, front) -> bool`, `passed(k, walls_rel, tile)`.

- [ ] Накопление `CellSides` по новым оборотам (не на склоне), голоса передней стены N, цвета плитки над N, поправка по столбикам, решение через `plan.decide` (отложено при `ramp_state == 'cand'`), `plan.passed` при проезде; горка после нескольких клеток — `cells + 1`, а не 1.
- [ ] `py_compile`, pyright ≤ 16, тесты зелёные. Коммит `drive_cells: straight-or-stop from a plan, cells ahead from lidar, posts`.

### Задача 7: план в flood_fill_run и return_to_start

**Files:** Modify `pi/nav.py` (`_DrivePlan`, вызовы).

- [ ] `_DrivePlan(cell, hdg, goal_test, home, on_pass)`; разведка — фронтир, без остановки на самой N; возврат — стоп на старте; стоп перед известной горкой; «не уверен» по стороне без знания карты → стоп; `on_pass` — чекпоинт на ходу и `_write_state`.
- [ ] Вызовы `drive_cells(max(1, max_cells - total), …, plan=plan)` и обработка `n` клеток.
- [ ] `py_compile`, pyright ≤ 16. Коммит `Explore and return without stopping on straights`.

### Задача 8: проверка, ревью, заливка, полевой план

- [ ] verification-before-completion, `/code-review high` на дифф от `2255734`, исправления с тестами.
- [ ] Заливка на Pi (`nav.py walls.py lidar_proc.py corridor.py runplan.py watch_logic.py`) с `.bak`, `py_compile` на Pi, смоук `lidar_proc` стоя.
- [ ] Полевой план пользователю.
