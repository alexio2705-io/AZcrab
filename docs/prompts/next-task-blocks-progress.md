# Ход работ: кубики для технических испытаний

Промпт — `docs/prompts/next-task-blocks.md`. Ветка `nav-split`.

Проверка после каждой задачи: `python3 -m unittest discover -s pi/tests`, `pyright` по изменённым
файлам (сравнение с HEAD — новых ошибок нет), `python3 -m py_compile`.

Тесты модулей, которые тянут `state` (а он — numpy), идут через `pi/tests/fakes.py`: на время
импорта `state`, `lidar_walls` и т. п. подменяются пустыми модулями.

## 1. Один шаг движения (`pi/explore.py`)

- `explore.Walk` — где робот (клетка, курс, чекпоинт, фаза пульта) и счётчики неудач на ребре.
- `explore.step(w, goal_test, home, arrive)` — один шаг: дождаться робота → путь BFS → поворот
  (`_turn_toward` / `_turn_once`) → поездка (`_drive`) → разбор остановки (`_handle_stop`,
  `_close_ahead`, `_obstacle_stall`, `_bumper_side`). Возвращает `"goal"` / `"no_path"` / `"step"`.
- `explore.go_to(w, goal_test, arrive)` — шаги до цели; пути нет → `_no_path`.
- `arrive_explore` и `arrive_home` — что делать по приезде; передаются в шаг снаружи и вызываются
  там же, где раньше (между поездкой и разбором причины).
- `flood_fill_run` — 22 строки, `return_to_start` — 12.
- `nav.py`: убрана проверка `status != "fatal"`, возврат и мигание идут всегда.

Аварийных выходов нет:
- `MAX_STALLS` шагов без продвижения → `recovery.restart_from_checkpoint` (фаза `sleep` на пульте,
  лента синяя, ждём кнопку, карта та же).
- Пути к цели нет (в возврате и в `go_to`) → перезапуск с чекпоинта; если и оттуда пути нет →
  `maze_map.forget_soft_blocks()`: рёбра из `state.soft_closed` (препятствие, бампер, одометрия,
  кирпич у входа) снова решают голоса лидара, клетки за ними выходят из `state.blocked`. Чёрные
  клетки (`state.black_cells`) не забываются. Если пути нет и после этого — следующий шаг снова
  перезапускает с чекпоинта: заезд не кончается.
- Статуса `"fatal"` у разведки нет.

Что изменилось в возврате (взято из разведки):
1. Препятствие (`obstacle_stall`/`ramp_stall`) — до `OBSTACLE_RETRIES` повторов, раньше ребро
   закрывалось сразу.
2. Боковой удар бампером — до `bumper.BUMPER_SIDE_RETRIES` объездов, раньше закрывалось после одного.
3. Разворот на 180° — двумя поворотами по 90°, раньше одним.
4. После каждого поворота — скан стен камерами (`victims.scan_cell`), раньше не было.
5. Счётчик `MAX_STALLS` (раньше в возврате не было) → перезапуск с чекпоинта.
6. Чекпоинт, проеханный на ходу (`_DrivePlan.checkpoint`), запоминается и в возврате.
7. Подъём робота (`lifted`) сбрасывает счётчики неудач и печатает «Продолжаем с чекпоинта».
8. Печать «─── (x,y) N → E  k/300 ───» и хронометраж шага — теперь и в возврате.
9. Сообщения в лог при `odo_fault`, `front_obstacle`, `wall_close`, `black` — те же, что в разведке.

Что изменилось в разведке: только конец по `MAX_STALLS` (перезапуск вместо выхода) и запись
закрытых рёбер в `state.soft_closed` / `state.black_cells`.

Тесты: `pi/tests/test_forget_blocks.py` (ребро препятствия забывается, чёрная остаётся,
после забывания стену решают голоса лидара).

## 2. Правила плиток (`pi/tile_rules.py`)

- `RULES` — цвет → `Rule(роль, react, every_visit)`. Роли: `NORMAL`, `WALL`, `STOP`, `CHECKPOINT`.
  Обычный заезд: чёрная — `WALL`, синяя — `STOP` + `floor_tiles.blue_stop` на каждом визите,
  серебряная — `CHECKPOINT`, красная — `NORMAL`.
- `--red-checkpoint` теперь `tile_rules.use_checkpoint(TILE_RED)`: красная — чекпоинт, серебряная —
  обычная, `state.CHECKPOINT_TILE` выставляется там же (его читают калибровка и печать).
- На таблицу переведены: `floor_tiles.check_floor_while_driving` (стена → откат, стоп → центр,
  чекпоинт на ходу), `drive_front` (`tile_rules.must_stop`), `explore.arrive_explore` и
  `arrive_home` (`tile_rules.on_arrive` вместо `blue_stop`), чекпоинт стоя.
- `state.tile_map` пишется: по приезде (`on_arrive`), на ходу (`check_floor_while_driving`, клетка
  `trip.plan.cell(k_under)`), при откате от стены (`explore._close_ahead`, клетка за ребром, цвет —
  `state.last_wall_tile`).
- Решено по ходу: память «однажды подтверждённому верим» (была `blue_cells`) обобщена в
  `state.react_cells` (клетка → цвет, по которому уже была реакция). `tile_map` — просто последний
  известный цвет, без «залипания»: иначе ложное чтение на ходу у края клетки включало бы стоянку на
  белой. `blue_cells` остался (его пишет `blue_stop`, читает дашборд).
- Поведение обычного заезда не менялось; в лог чёрной добавлен цвет плитки.

Тесты: `pi/tests/test_tile_rules.py` (таблица обычного заезда, красный чекпоинт, запись `tile_map`,
реакция на каждом визите и по памяти, правило «один раз»).

## 3. Глаголы (`pi/actions.py`)

- Шпаргалка всех глаголов — в начале `actions.py`.
- Стоя: `blink` (новая `leds.blink_n`, `blink_pink` не тронута), `blink_victim`, `blink_finish`,
  `stand`, `stand_motors_off`, `give_kits(w, n, wall_dir=None)`.
- `victims.give_kits(n, wall_dir, hdg)` вынесена из `report_victim` дословно; разворот спиной к
  стене — `victims._back_to_wall`. `report_victim` зовёт `give_kits` — поведение то же.
- Ехать: `start_walk`, `explore_until(w, done)` (цикл разведки с внешним условием),
  `go_to_cell`, `go_to_color` (ближайшая по пути клетка цвета, через `explore.go_to`).
  `explore._unvisited` стала `explore.unvisited` — её зовёт `explore_until`.
- Время: `Timer(limit_s)` — `elapsed`, `left`, `trip_s`, `time_to_go`. Среднее на клетку —
  `sec_per_cell`: статьи хронометража «ход + поворот + доводка + лидар» / клеток проехано, до 3 клеток
  — 4 с. К оценке пути запас ×1,3. Пути по карте нет — оценка по прямой (решать надо всегда).
- Найдено: `cells_of(color)` (`state.tile_map`), `marks_found(classes)` (`state.victim_log`).

Тесты: `pi/tests/test_actions.py` (среднее на клетку, длина пути, таймер и «пора ехать»,
клетки по цвету, метки по классам).
