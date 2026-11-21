# Где что в коде робота (pi/)

Запуск: `nav.py` → `main()`: `boot.start()` (железо) → `explore.flood_fill_run()` (обход) →
`explore.return_to_start()` → мигание на старте → `boot.stop()` (уборка).
Задания технического испытания — `task_*.py` из глаголов `actions.py` (см. «Как написать задание»).
Модули вызывают друг друга как `модуль.функция()`: по имени сразу видно, в каком файле искать.
Общие переменные робота (карта, телеметрия, флаги) — `state.x`; общие константы — `config.X`;
константы одного модуля — вверху этого модуля.

## Хочу поменять…

| Что | Файл | Функция / константы |
|---|---|---|
| Реакцию на удар бампером | `bumper.py` | `check_bumper_while_driving` (на ходу), `back_off_from_wall`, `dodge_side_hit`; `BUMPER_*` вверху |
| Сколько раз объезжать боковой удар | `bumper.py` | `BUMPER_SIDE_RETRIES` (решение — в `explore._bumper_side`) |
| Один шаг к цели (поворот, поездка, разбор остановки) | `explore.py` | `step` → `_turn_toward` → `_drive` → `_handle_stop`; `go_to` — шаги до цели |
| Что делать, когда застряли или пути нет | `explore.py` | `MAX_STALLS`, `_restart`, `_no_path`; `maze_map.forget_soft_blocks` |
| Что робот делает на плитке цвета X | `tile_rules.py` | таблица `RULES`: роль (`NORMAL`/`WALL`/`STOP`/`CHECKPOINT`) и реакция |
| Какие метки — жертвы, сколько комплектов | `config.py` | `VICTIM_CLASSES`, `CLASS_TO_VICTIM`, `KITS_BY_TYPE`, `add_victim_class` |
| Запуск железа и уборку | `boot.py` | `start`, `stop`, `run` (то, что было в начале и в `finally` у `nav.main`) |
| Глаголы для заданий | `actions.py` | шпаргалка — в начале файла |
| Доворот на метку с поворота | `marks.py` | `_turn_marks` и её 4 шага `_best_turn_frames` → `_turn_mark_place` → `_read_turn_mark` → `_record_turn_mark` |
| Остановку у метки, замеченной на ходу | `marks.py` | `watch_marks_while_driving` (заметил), `_mark_stop` (стоя у неё) |
| Перечитывание передней метки | `marks.py` | `_front_reread`, `FRONT_*` |
| Порог «стена есть» | `walls.py` | `WALL_THRESH`, `SIDE_THRESH`, `BACK_THRESH` |
| Как лидар меряет стены | `walls.py` (окна, маски) → `lidar_walls.py` | описание цепочки — в начале `lidar_walls.py` |
| Поворот / доворот на курс | `turning.py` | `turn_to`, `turn`, `align_at_wall` (доводка у стены) |
| Застрял на повороте (профиль, порог) | `turn_unstick.py` | ступени `Unstick.STAGES`: момент, раскачка, одной стороной, откат; токи и время вверху модуля |
| Езду по клеткам | `driving.py` | `run_tick` — список шагов такта по порядку |
| Отъезды после остановки (горка, препятствие, чёрная) | `driving.py` | `finish_trip` |
| Короткий подъезд/отъезд на N мм | `driving.py` | `creep_mm`, `creep_soft` |
| Скорость, разгон, торможение | `drive_motion.py` | `speed_profile`; `MOVE_*` в `drive_trip.py`, `BRAKE_DECEL_CMS2` в `config.py` |
| Центровку по стенам и руль | `drive_motion.py` | `wall_offset`, `steering`; `WALL_*` там же, `GYRO_*` в `config.py` |
| Горки и ассист моментом | `drive_ramp.py` | `track_slope`, `update_assist`; `RAMP_*`, `OBSTACLE_*` |
| Решение «ехать дальше или стоп» у границы клетки | `drive_front.py` | `decide_at_boundary` |
| Аварийные остановки (связь, энкодеры, стена вплотную) | `drive_checks.py` | `check_abort`, `check_progress`, `check_wall_too_close` |
| Цвет плитки | `floor_tiles.py` | `tile_color`, `check_floor_while_driving` (на ходу) |
| Жертвы стоя, аптечки | `victims.py` | `scan_sides`, `report_victim` |
| Карту и путь | `maze_map.py` | `vote_wall`, `find_path_bfs` |
| Связь с ESP, моторы | `esp_link.py` | `motors`, `stop`, `brake`, `get_yaw` |

## Как устроена поездка (`driving.py`)

`_drive_cells` = `drive_trip.start_trip()` → в цикле `run_tick()` → `finish_trip()`.
Всё, что поездка помнит, лежит в объекте `trip` (`trip.traveled_cm`, `trip.reason`, …); поля и
комментарии к ним — в `drive_trip.start_trip()`.
`run_tick()` вызывает шаги по порядку. Шаг возвращает:
- `None` — дальше, следующий шаг;
- `"stop"` — закончить поездку (причина в `trip.reason`), потом `finish_trip()`;
- `"next_tick"` — сразу следующий такт;
- `"quit"` — выйти сразу, без доводки.

Новый шаг — функция `шаг(trip)` и строка в списке `run_tick()`.

## Что сломалось при разборе

`sim/` и `pi/challenge.py` подменяли функции внутри старого `nav.py`, поэтому пока не работают.

## Как написать задание на площадке

Задание — один файл `pi/task_<имя>.py` на 30–60 строк. Образцы: `task_tomsk.py` (порядок зон, таймер,
финиш на старте), `task_colors.py` (роли цветов, буквы как жертвы, финиш на чёрной), `task_cog.py`
(порядок по кольцам меток, финиш на серебряной). Список глаголов с описанием — в начале `actions.py`.

1. **Скопировать ближайший образец** в `pi/task_<имя>.py`. В шапке написать условие словами и что
   робот делает — пригодится через полчаса, когда будешь искать ошибку.
2. **Плитки** — строки таблицы `tile_rules.RULES` в начале файла:
   `tile_rules.RULES[config.TILE_RED] = tile_rules.Rule(tile_rules.STOP, react=моя_функция)`.
   Роль: `NORMAL` проехать, `WALL` не заезжать, `STOP` встать в центре, `CHECKPOINT` чекпоинт.
   Реакция `react(клетка)` — по приезде; `every_visit=False` — один раз. Чтобы на плитке что-то
   сделать, робот должен на ней остановиться: роль `STOP` или поездка к ней (`go_to_cell`).
3. **Метки** — до запуска: `config.add_victim_class("H", kits=1)` делает H жертвой;
   `config.KITS_BY_TYPE` — комплекты за тип; `config.cog_marks_not_victims()` — когнитивные только
   читать.
4. **Сценарий** — функция `scenario(run)`: `w = actions.new_walk()`, `actions.look_around(w)`,
   дальше глаголы: `explore_until`, `go_to_cell` / `go_to_color` / `visit_targets`, `blink`,
   `give_kits`, `stand_motors_off`. Таймер — `actions.Timer(лимит)`, «пора домой» —
   `timer.time_to_go(w, клетка, extra_s=...)`.
5. **Запуск железа** — последней строкой `boot.run(scenario)`; без камер (лента остаётся) —
   `boot.run(scenario, cameras=False)`.
6. **Проверка до поля:** `python3 -m py_compile pi/task_<имя>.py`, логику без железа — в
   `task_logic.py` с тестом в `pi/tests/`. На Pi: `~/maze/venv/bin/python ~/maze/task_<имя>.py`.
7. **Не хватает глагола** — дописать его в `actions.py` (короткая функция + строка в шпаргалке), а не
   логику в задании.

Правила проекта действуют и в заданиях: заезд кончается только на финише (пути нет — перезапуск с
чекпоинта, а не выход), робот всегда решает, комплекты — сразу у жертвы.
