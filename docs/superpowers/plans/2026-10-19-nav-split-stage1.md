# Разбор nav.py, этап 1 (перенос): план

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** разложить `pi/nav.py` по небольшим модулям, не меняя ни одной строки логики.

**Architecture:** перенос делает скрипт по AST. Он режет nav.py на куски верхнего уровня (функция или константа вместе с комментарием над ней), раскладывает их по модулям по таблице ниже и переписывает обращения к именам: общие переменные становятся `state.x`, имена из другого модуля — `модуль.имя`. Строки `global` удаляются. Независимый второй скрипт сверяет каждую функцию с исходной.

**Tech Stack:** Python 3.11 на Pi. Инструменты (скрипт переноса, сверка, pyflakes) работают локально в scratchpad и в репозиторий не попадают.

**Spec:** `docs/superpowers/specs/2026-10-19-nav-split-design.md`

## Global Constraints

- Тела функций дословные: меняются только имена (`_maze` → `state.maze`, `turn_to` → `turning.turn_to`) и удаляются строки `global`.
- Между проектными модулями только `import модуль`, без `from модуль import`.
- `config.py`, `state.py`, `timing.py` не импортируют других модулей проекта, кроме друг друга в порядке config → state → timing. Только в них может быть то, что нужно при импорте (декоратор `@_timed`, значения по умолчанию у параметров, начальные значения переменных).
- Симулятор не запускать. `sim/`, `pi/challenge.py`, `src/` не трогать.
- Python 3.11: никакого нового синтаксиса.

## Раскладка

| Модуль | Функции |
|---|---|
| `config.py` | только константы, нужные двум и более модулям или при импорте |
| `state.py` | все изменяемые переменные верхнего уровня nav.py без `_` в начале (`_state` → `telem`), плюс `YAW_ORIGIN`, `PER_CELL_STOP`, `CHECKPOINT_TILE`, `USE_COLOR_SENSOR` |
| `timing.py` | `_timed`, `time_report`, `tick_report` |
| `esp_link.py` | `crc16`, `link_report`, `link_errors`, `_link_reopen`, `_telem_thread`, `get_yaw`, `get_yaw_rate`, `get_yaw_and_rate`, `get_pitch`, `get_roll`, `get_accel`, `get_az`, `telem_age`, `telem_fresh`, `wait_telem`, `get_odo`, `get_z_cnt`, `wait_odo_zeroed`, `encoders_online`, `encoder_sides`, `get_color`, `get_rgbc`, `get_esp_paused`, `resume_esp`, `interrupted`, `_cmd`, `_drive_cmd`, `_drive_cmd_now`, `motors`, `torque`, `stop`, `zero_odo`, `brake`, `settle_brake`, `hold_while` |
| `bumper.py` | `get_bumper`, `bumper_side`; константы `BUMPER_*`, `BUMP_*` |
| `lidar_walls.py` | `_new_scans`, `_lidar_spawn`, `_lidar_thread`, `get_walls`, `get_walls_still`, `get_wall_points`, `wall_present`, `wait_scan`, `_walls_cached`, `measure_walls`, `measure_front_confirmed`, `_measure_cell` |
| `tiles.py` | `black_threshold`, `tile_color`, `_tile_color`, `blue_stop`, `load_tile_calib`, `calibrate_white` |
| `ramps.py` | `tilt_deg`, `is_tilted`, `record_ramp`, `level_of`, `_ramp_measure` |
| `maze_map.py` | `_maze_cell`, `_cell_at`, `_next_cell`, `_walls_match`, `_set_wall`, `_edge_key`, `vote_wall`, `close_edge`, `open_edge`, `set_cell_walls`, `block_direction`, `open_dirs`, `find_path_bfs`, `find_frontier_path`, `_walls_rel_map` |
| `turning.py` | `angle_err`, `snap_target`, `turn`, `gyro_heading`, `turn_to`, `align_at_wall` |
| `driving.py` | `drive_cells`, `_drive_cells`, `creep_mm`, `creep_soft` |
| `marks.py` | `_interrupt_reason`, `_watch_decide`, `_reject_mark`, `_mark_stop`, `_mark_world`, `_read_wall`, `_reread_aligned`, `_front_reread`, `_reset_unread`, `_decide_unread`, `_corner_dup`, `_turn_marks` |
| `victims.py` | `dispense`, `kits_left`, `dispense_kit`, `_front_back_step`, `_scan_front`, `scan_sides`, `scan_cell`, `_blink_victim`, `report_victim`, `_report_on_move` |
| `recovery.py` | `wait_start_button`, `handle_lift`, `detect_heading`, `restart_from_checkpoint`, `wait_for_robot` |
| `explore.py` | `_DrivePlan`, `_arrived_cell`, `flood_fill_run`, `return_to_start` |
| `live.py` | `leds_lifted`, `leds_sleep`, `_lift_thread`, `_map_dict`, `_note_state`, `_note_phase`, `_write_state`, `blink_start_return`, `_local_ip`, `start_map_web`, `_r`, `_live_snapshot`, `start_live`, `start_dash_web` |
| `nav.py` | докстринг, `_on_hangup`, `main`, `if __name__ == "__main__"` |

Константа уходит в тот модуль, чьи функции её используют. Если её используют несколько модулей или она нужна при импорте другого модуля, она уходит в `config.py` (кроме `BUMPER_*`/`BUMP_*`, они в `bumper.py`, если это не ломает импорт).

## Review Focus

- Локальная переменная с тем же именем, что у функции или константы nav.py (`stop`, `turn`…): её нельзя переименовать. Области видимости определяются через `symtable`, а не по тексту.
- Имена внутри f-строк (`f"{BUMPER_BACKOFF_MM:.0f}"`) переименовываются так же, как в обычном коде.
- Русский текст в строке: позиции из AST — байтовые, замена делается по байтам строки.
- Порядок импорта: обращение к другому модулю при импорте (декоратор, значение по умолчанию, начальное значение) даёт `AttributeError` из-за кругового импорта. Это ловит пробный импорт с заглушками железа.
- Функция, которая присваивала общую переменную без `global` (то есть локальную с тем же именем), остаётся локальной.

---

### Task 1: скрипт переноса

**Files:**
- Create: `<scratchpad>/split_nav.py` (не в репозитории)
- Create: `pi/config.py`, `pi/state.py`, `pi/timing.py`, `pi/esp_link.py`, `pi/bumper.py`, `pi/lidar_walls.py`, `pi/tiles.py`, `pi/ramps.py`, `pi/maze_map.py`, `pi/turning.py`, `pi/driving.py`, `pi/marks.py`, `pi/victims.py`, `pi/recovery.py`, `pi/explore.py`, `pi/live.py`
- Modify: `pi/nav.py` (остаются докстринг, `_on_hangup`, `main`)

- [ ] **Step 1:** скрипт читает `pi/nav.py` из коммита `32ca25c` (`git show 32ca25c:pi/nav.py`), а не из рабочей копии, чтобы его можно было перезапускать.
- [ ] **Step 2:** каждая функция из таблицы попадает ровно в один модуль, иначе скрипт падает с именем функции. Неизвестная функция или лишняя запись в таблице — тоже ошибка.
- [ ] **Step 3:** каждый модуль начинается с докстринга в одну строку по-русски, затем `from __future__ import annotations`, внешние импорты, которые он использует (из шапки nav.py), затем `import config`, `import state` и нужные проектные модули.
- [ ] **Step 4:** запустить скрипт. Ожидается: 17 файлов, отчёт «функций N, констант M, переменных состояния K», без ошибок.

### Task 2: независимая сверка

**Files:**
- Create: `<scratchpad>/check_split.py`

- [ ] **Step 1:** для каждой функции и класса исходного nav.py найти его в новом модуле. В новом AST заменить `модуль.имя` → `имя`, `state.x` → исходное имя. В исходном AST удалить `global`. Сравнить `ast.dump` без позиций. Ожидается: «совпало 155 из 155».
- [ ] **Step 2:** то же для каждой константы (значение) и каждой переменной состояния (начальное значение).

### Task 3: компиляция, pyflakes, тесты, пробный импорт

- [ ] **Step 1:** `python3 -m py_compile pi/*.py`. Ожидается: без вывода.
- [ ] **Step 2:** pyflakes в `<scratchpad>/venv` по новым модулям. Ожидается: ни одного `undefined name`.
- [ ] **Step 3:** `python3 -m unittest discover -s pi/tests`. Ожидается: `OK` (пропуски «нужен numpy» — норма).
- [ ] **Step 4:** пробный импорт `import nav` с заглушками `serial`, `RPi.GPIO` и тяжёлых модулей камеры в `sys.modules`. Ожидается: импорт проходит, у каждого модуля есть все функции из таблицы.

### Task 4: коммит

- [ ] `git add pi/*.py && git commit` с сообщением «pi: split nav.py into small modules, code moved verbatim».

### Task 5: заливка на Pi

- [ ] По скиллу `pi_connect`: сравнить файлы на Pi с исходной точкой, сделать `.bak`, залить новые модули и nav.py, на Pi выполнить `python -c "import nav"` в venv. Дать пользователю обычную команду запуска.
