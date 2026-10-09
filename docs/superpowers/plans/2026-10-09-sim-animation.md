# Анимация прогона симулятора — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** запись трассы прогона `nav.py` в симуляторе и локальная HTML-страница
на Маке, которая её подробно проигрывает.

**Architecture:** `maze_sim.Sim` получает необязательный регистратор `rec`
(хуки на шаг модели, оборот лидара, сырые команды). `sim/trace.py` — чистые
функции (разбор лога, прореживание, мир, правда по рёбрам, снимки карты,
JSON) и `Recorder`. `sim/viewer/build.py` вставляет JSON в
`sim/viewer/template.html`. `run_scenarios.py --html` связывает всё.

**Tech Stack:** Python 3 stdlib (unittest, json, re); HTML/CSS/JS без сборки,
canvas 2D; three.js r149 `three.min.js` (классический скрипт) в
`sim/viewer/vendor/`.

**Spec:** `docs/superpowers/specs/2026-10-09-sim-animation-design.md`

## Global Constraints

- Логику `pi/nav.py`, `pi/corridor.py`, `pi/runplan.py`, прошивку `src/`, скорости не менять.
- `python3 sim/run_scenarios.py` без флагов: тот же вывод, логи байт-в-байт как до изменений, «ВСЕ СЦЕНАРИИ ЧИСТЫЕ».
- `python3 -m unittest discover -s pi/tests` зелёный; pyright — не больше 16 ошибок.
- Ничего не публиковать; страница работает с `file://` без интернета.
- Один шаг — один коммит, в конце сообщения `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Отступление от спеки (обосновано)

Chrome не грузит `<script type="module" src=…>` со страницы `file://` (CORS,
origin `null`), поэтому вместо ES-модуля и import map — классический
`three.min.js` r149 (последняя версия без предупреждения об устаревании
UMD-сборки) через `<script src="vendor/three.min.js">`: классические скрипты с
`file://` грузят и Chrome, и Safari. Точки лидара в трассе — целые мм (объём),
поза — 0,1 мм, как в спеке.

## Review Focus

1. Строка лога без перевода строки в конце прогона (последний `\r`-прогресс) — должна попасть в события, а не потеряться (тест в Task 2).
2. Текст лога с `</script>` или `<!--` ломает страницу — данные экранируются `<` → `<` (тест в Task 5).
3. Повторный `_write_state` без изменений карты — снимок не дублируется (тест в Task 3).
4. Поза между кадрами при курсе через 0°/360° — интерполяция по кратчайшей дуге, без оборота робота на месте (проверка в Task 6 вручную: сценарий с курсом 37°±90°…).
5. Сценарий без горок / без верхнего этажа — карта этажа 1 пустая, просмотрщик не падает (Task 7, `flat`).

---

### Task 1: Хуки регистратора в `maze_sim.Sim`

**Files:**
- Modify: `sim/maze_sim.py` (`Sim.__init__`, `Sim.write`, `Sim.step`, `Sim._scan`)
- Test: `sim/tests/test_sim_hooks.py`

**Interfaces:**
- Produces: `Sim.rec` (None или объект с `on_step(sim, pitch)`, `on_scan(sim, hits, pts)`); `Sim.cmd_raw` ∈ `"M"|"T"|"B"`; `hits` — список `(x, y, z, is_wall)` мировых концов лучей; `pts` — `corridor_points` (система робота).

- [ ] **Step 1: тест** — `Sim` с фейковым регистратором в коридоре `corridor_world()`: `sim.write(b"T:20,20,30*")` → `cmd_raw == "T"`; `b"B*"` → `"B"`, `cmd` = `("M",0,0,0)`; `sim.advance(0.2)` → `on_step` вызван 40 раз, `on_scan` ≥ 1 раз, у хита 4 поля, есть и `is_wall=True`, и `False`.
- [ ] **Step 2:** запустить `python3 -m unittest discover -s sim/tests` — падает (`cmd_raw` нет).
- [ ] **Step 3:** реализация:
  - `__init__`: `self.cmd_raw = "M"`, `self.rec = None`;
  - `write`: `cmd_raw = "M"/"T"/"B"` по команде;
  - `step`: после `_telemetry` — `if self.rec is not None: self.rec.on_step(self, p)`;
  - `_scan`: результат `_ray_walls` в `sw`, `s = _ray_terrain(..., min(sw,1500) if sw<4000 else 1500)` (как было), при `rec` — `hits.append((x+dx*s, y+dy*s, z+s*te, s == sw))`; `pts = corridor.corridor_points(filt[0])` один раз, в `_scan_pts` и в `rec.on_scan(self, hits, pts)`.
- [ ] **Step 4:** тесты зелёные; `python3 sim/run_scenarios.py` → «ВСЕ СЦЕНАРИИ ЧИСТЫЕ», `diff -r` логов с копией до изменений пуст.
- [ ] **Step 5:** коммит `sim: optional trace recorder hooks in the model`.

### Task 2: Строки лога со временем и разбор в события

**Files:**
- Create: `sim/trace.py`
- Test: `sim/tests/test_trace_log.py`

**Interfaces:**
- Produces: `TimedLines(clock)` — файлоподобный (`write`, `flush`, `getvalue`, `finish()`), `.lines: list[(t, str)]`; `parse_line(text) -> dict` (`type` + поля); `parse_log(lines, t0=0.0) -> list[dict]` (`t`, `text`, `type`, поля).

- [ ] **Step 1: тесты**:
  - `TimedLines`: `write("\n    → кл…\n")` при t=1 → одна строка с t=1; прогресс `"\r    кл.1 ост. 300мм … spd=0   "` + `"\r    кл.1 ост. 297мм …"` при t=1 и t=2 → две строки с t=1 и t=2; `getvalue()` — весь текст как есть; хвост без `\n` попадает в `lines` после `finish()`; пустые строки не попадают.
  - `parse_line` на реальных строках (из `sim/logs`): `step` (`─── (2,1004) N → W  4/30 ───` → cell `[2,1004]`, from `N`, to `W`); `decide` (`→ кл.(1, 0): R=есть(68т,175мм) L=есть(68т,175мм) F=нет(0:5) → прямо (новая)` → cell `[1,0]`, `straight=True`, reason `новая`, sides `{"R":"есть","L":"есть","F":"нет"}`; и вариант `стоп (известна)`, `F=?(…)`); `post`, `progress`, `brake`, `at_wall`, `turn`, `turn_done`, `move`, `ramp_detect` (`горка: pitch=+20.2°, момент 1.9A …`), `ramp_measure`, `ramp_up`, `ramp_down`, `ramp_done` (`⛰ ГОРКА вверх: ~1 плит., (0,1) → (0,1003), …` → from `[0,1]`, to `[0,1003]`, up True), `not_ramp`, `obstacle`, `bumper`, `backoff`, `align` (`[выр] ок, 140мм`), прочее → `other`.
  - `parse_log`: `t` округлено до 1 мс за вычетом `t0`, `text` без крайних пробелов.
- [ ] **Step 2:** тесты падают (нет модуля).
- [ ] **Step 3:** реализация — таблица `(тип, regex, поля)` в порядке: `step, decide, post, progress, brake, at_wall, turn, turn_done, move, ramp_detect, ramp_measure, ramp_up, ramp_down, ramp_done, not_ramp, obstacle, bumper, backoff, align`; `re.search`, первое совпадение.
- [ ] **Step 4:** тесты зелёные.
- [ ] **Step 5:** коммит `sim: timed log lines and log events for the trace`.

### Task 3: Прореживание позы, мир, правда по рёбрам, снимки карты

**Files:**
- Modify: `sim/trace.py`
- Test: `sim/tests/test_trace_data.py`

**Interfaces:**
- Produces:
  - `thin_poses(samples, keep_times=(), dt=0.02) -> list[dict]` — сэмпл: `t,x,y,z,th,p,vl,vr,m,sl,sr,cap,b,f`;
  - `pose_columns(poses, t0) -> dict[str, list|str]` (колонки; `m` — строка из букв, `b` — 0/1, `f` — число или None);
  - `sample_terrain(x0,y0,x1,y1,f,step=10.0) -> dict` (`{"kind":"flat","rect",h}` или `{"kind":"slope","rect","dir":"N|E|S|W","h0","h1"}`);
  - `world_json(walls, pieces) -> dict` (`walls`, `terrain`, `cell`);
  - `level_of(cy)`, `edge_truth(walls, height, cx, cy, d) -> bool`, `truth_rows(walls, height, cells) -> list[[cx,cy,n,e,s,w]]`;
  - `map_snapshot(maze, visited, ramps, blocked, edge_fixed, phase) -> dict` (`maze` строки `[cx,cy,n,e,s,w]` 1/0/None, `visited`, `ramps` `[cx,cy,d,bx,by,n,dir]`, `blocked`, `fixed` `[cx,cy,d]`, `phase`), всё отсортировано.

- [ ] **Step 1: тесты**:
  - `thin_poses`: 1 с сэмплов по 5 мс одного режима → 51 кадр (каждый 4-й плюс последний); смена `m` в середине → кадр на смене; `b` переключился → кадр; `keep_times=[0.0123]` → кадр с t ≥ 0.0123 сохранён; первый и последний всегда.
  - `sample_terrain`: остров `lambda x,y: 140` → flat h=140; горка `(-150,450,150,750, 140*(y-450)/300)` → slope dir `N`, h0=0, h1=140; горка вниз на восток → dir `W`; бугорок 15 → flat 15.
  - `edge_truth`: для всех миров `SCENARIOS` и клеток x∈[-1,3], y∈[-1,5], d∈0..3 на этаже 0 совпадает с `run_scenarios.wall_truth`; `ramp_world` этаж 1: `(0,1003)` W/E стена, S и N проход; `(1,1004)` N/E/S стена, W проход.
  - `map_snapshot`: True/False/None → 1/0/None, `_edge_fixed` с frozenset из двух концов → оба `[cx,cy,d]` в `fixed`; результат детерминирован при перемешанных входах.
- [ ] **Step 2:** тесты падают.
- [ ] **Step 3:** реализация (функции выше; `edge_truth`: этаж `level_of(cy) = floor((cy+500)/1000)`, мировая клетка `(cx, cy - lv*1000)`, точка середины ребра, `z = height(центр клетки) + 40`, стена — прямоугольник содержит точку и `z0 ≤ z ≤ z1`).
- [ ] **Step 4:** тесты зелёные.
- [ ] **Step 5:** коммит `sim: pose thinning, world and per-floor wall truth for the trace`.

### Task 4: `Recorder`, сборка JSON и `run_scenarios.py --html` (данные)

**Files:**
- Modify: `sim/trace.py` (`Recorder`, `build_trace`, `dump_trace`)
- Modify: `sim/run_scenarios.py` (`fresh_nav()`, `run(name, trace=False)` → `(ok, trace|None)`, флаг `--html`)
- Modify: `sim/.gitignore` (`viewer/data/`, `viewer/*.html`, `!viewer/template.html`)
- Test: `sim/tests/test_trace_run.py`

**Interfaces:**
- Consumes: Task 1 хуки, Task 2–3 функции.
- Produces: `Recorder(nav, sim)` с `on_step`, `on_scan`, `on_state(phase)`, `samples`, `scans`, `maps`; `build_trace(name, world, rec, lines, meta, t0) -> dict` (ключи `meta, world, truth, poses, scans, events, maps`); `dump_trace(trace) -> str` (`sort_keys`, компактно, `ensure_ascii=False`); `run_scenarios.fresh_nav()`; `run_scenarios.run(name, trace=False)`.

- [ ] **Step 1: тест** — `fresh_nav(); ok1, tr1 = run("flat", trace=True)`, ещё раз → `dump_trace(tr1) == dump_trace(tr2)`; `tr1` содержит `poses.t` с шагом ≤ 0,02 с, `scans` ≥ 10 с `hits` и `corr`, событий `decide` ≥ 1, `maps` ≥ 1, `truth` не пуст, `meta.status == "explored"`.
- [ ] **Step 2:** падает.
- [ ] **Step 3:** реализация. `Recorder.on_scan` переводит `pts` (x вперёд, y вправо) в мир: `fwd = (sin θ, cos θ)`, `right = (cos θ, −sin θ)`. `on_state` пишет снимок только если он отличается от предыдущего (без `phase`… с `phase` — смена фазы тоже снимок). В `run` при `trace=True`: `rec = Recorder(nav, sim)`, `sim.rec = rec`, stdout → `TimedLines(lambda: sim.t)`, после `ms.setup(sim)` — `nav._write_state = lambda cx, cy, hdg, cp, phase: rec.on_state(phase)`; в конце `rec.on_state("end")`, `lines.finish()`; лог пишется из `getvalue()` как прежде. `--html` пишет `sim/viewer/data/<имя>.json` (страницы — Task 5).
- [ ] **Step 4:** тесты зелёные; без флага логи как до изменений.
- [ ] **Step 5:** коммит `sim: record a deterministic JSON trace per scenario (--html)`.

### Task 5: Генератор страниц, three.js, каркас страницы и вид сверху

**Files:**
- Create: `sim/viewer/build.py`, `sim/viewer/template.html`, `sim/viewer/vendor/three.min.js`
- Modify: `sim/run_scenarios.py` (`--html` вызывает `build.write_site`)
- Test: `sim/tests/test_viewer_build.py`

**Interfaces:**
- Produces: `page_html(template, trace_json, names) -> str` (подставляет `__TRACE__` и `__SCENARIOS__`, экранирует `<` → `<`); `index_html(rows) -> str`; `write_site(out_dir, traces) -> str` (путь к index.html).

- [ ] **Step 1: тест** — `page_html` с текстом `"</script><!--"` в данных: в результате нет `</script><!--` внутри данных, `json.loads` извлечённого блока возвращает исходную строку; `index_html` содержит ссылки `flat.html` и т.д.
- [ ] **Step 2:** падает.
- [ ] **Step 3:** `build.py`; `curl` r149 `three.min.js` с jsdelivr в `vendor/`; шаблон: CSS-переменные (светлая/тёмная), сетка (шапка; вид сверху | 3D | правая колонка карта+консоль; низ — профиль и графики), загрузка данных, общий таймлайн (`cur`, пуск/пауза, скорость, шкала, события ←/→ с фильтром, чекбоксы слоёв), интерполяция позы (курс по кратчайшей дуге), вид сверху целиком (стены по высоте, горки градиент+стрелка, острова, бугорки, робот 193×125 с носом и колёсами, след по скорости, лучи, точки коридора, метки решений с подсказкой, столбики, ассист, бампер; масштаб/сдвиг/следить).
- [ ] **Step 4:** тесты зелёные; `--html` собирает; страница открывается во встроенном браузере, анимация идёт, ошибок в консоли нет.
- [ ] **Step 5:** коммит `sim viewer: page generator, vendored three.js, top view and timeline`.

### Task 6: 3D-вид

**Files:** Modify: `sim/viewer/template.html`

- [ ] **Step 1:** сцена three.js: пол, стены коробками `z0..z1`, острова и бугорки — коробки до `h`, горки — наклонная плита по `dir/h0/h1`; робот — группа (корпус, нос, 4 колеса), `rotation.order="YXZ"`, `y = −θ`, `x = pitch`; лучи последнего оборота — `LineSegments`; камера сзади-сверху следует, облёт мышью, клавиша `C` — вернуть следование; `ResizeObserver`.
- [ ] **Step 2:** проверка во встроенном браузере на `ramp` и `two_ramps`: корпус наклоняется на горке, камера следует, ошибок нет.
- [ ] **Step 3:** коммит `sim viewer: 3D view following the robot`.

### Task 7: Карта глазами робота, профиль, графики, консоль, оглавление

**Files:** Modify: `sim/viewer/template.html`, `sim/viewer/build.py` (оглавление со сводкой)

- [ ] **Step 1:** карта: снимок на `cur`, этаж авто (по последнему `step`) или 0/1; сравнение с `truth` (совпало — сплошная, лишняя — красная, пропущенная — оранжевый пунктир, неизвестно — серая, `fixed` — фиолетовая), посещённые, горки, заблокированные; робот на своём этаже.
- [ ] **Step 2:** профиль z от пройденного пути с курсором; графики по времени: скорость (vl, vr, средняя, уставка), pitch, полоса M/T/B, передняя стена; клик/перетаскивание — перемотка.
- [ ] **Step 3:** консоль: все строки, цвет по типу, подсветка текущей и прокрутка, клик — перемотка, скрыть `progress`.
- [ ] **Step 4:** оглавление: таблица сценариев (статус, время, клеток, горки, ошибки карты, упирался).
- [ ] **Step 5:** проверка на `flat` (нет этажа 1) и `two_ramps`; коммит `sim viewer: robot map, profile, charts, synced log console, index`.

### Task 8: Проверка целиком

- [ ] `python3 -m unittest discover -s pi/tests`, `-s sim/tests` — зелёные.
- [ ] `python3 sim/run_scenarios.py` — «ВСЕ СЦЕНАРИИ ЧИСТЫЕ», логи совпадают с копией до изменений.
- [ ] `python3 sim/run_scenarios.py --html` — путь к оглавлению напечатан.
- [ ] `pyright` — ≤ 16 ошибок.
- [ ] `open sim/viewer/index.html` в Safari и Chrome: анимация, 3D, графики.
