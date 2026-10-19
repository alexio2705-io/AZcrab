# Где что в коде робота (pi/)

Запуск: `nav.py` → `main()` → `explore.flood_fill_run()` (обход) → `explore.return_to_start()`.
Модули вызывают друг друга как `модуль.функция()`: по имени сразу видно, в каком файле искать.
Общие переменные робота (карта, телеметрия, флаги) — `state.x`; общие константы — `config.X`;
константы одного модуля — вверху этого модуля.

## Хочу поменять…

| Что | Файл | Функция / константы |
|---|---|---|
| Реакцию на удар бампером | `bumper.py` | `check_bumper_while_driving` (на ходу), `back_off_from_wall`, `dodge_side_hit`; `BUMPER_*` вверху |
| Сколько раз объезжать боковой удар | `bumper.py` | `BUMPER_SIDE_RETRIES` (решение — в `explore.flood_fill_run`) |
| Доворот на метку с поворота | `marks.py` | `_turn_marks` и её 4 шага `_best_turn_frames` → `_turn_mark_place` → `_read_turn_mark` → `_record_turn_mark` |
| Остановку у метки, замеченной на ходу | `marks.py` | `watch_marks_while_driving` (заметил), `_mark_stop` (стоя у неё) |
| Перечитывание передней метки | `marks.py` | `_front_reread`, `FRONT_*` |
| Порог «стена есть» | `walls.py` | `WALL_THRESH`, `SIDE_THRESH`, `BACK_THRESH` |
| Как лидар меряет стены | `walls.py` (окна, маски) → `lidar_walls.py` | описание цепочки — в начале `lidar_walls.py` |
| Поворот / доворот на курс | `turning.py` | `turn_to`, `turn`, `align_at_wall` (доводка у стены) |
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
