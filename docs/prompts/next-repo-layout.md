# Промпт: переезд и раскладка репозитория робота

Отвечай по-русски. Код в чат не выводи — пиши в файлы, в чате объясняй словами. Каждый шаг описывай: что сделал, чем проверил, что получилось.

## Контекст

- Карта кода: `docs/CODEMAP.md` (генерирует `.claude/hooks/codemap.py`), ручной указатель — `docs/robot_code_map.md`.
- Робот RoboCup Junior Rescue Maze: Raspberry Pi 5 (`pi/`, Python 3.11 на Pi) ведёт навигацию по лидару LD19, камеры/YOLO; ESP32-S3 (`src/`, `include/`, ESP-IDF 5.5 через PlatformIO `espressif32@6.12.0`, плата `4d_systems_esp32s3_gen4_r8n16`) — моторы и датчики. Регламент — `docs/rules/`.
- Репозиторий сейчас лежит в `~/Documents/PlatformIO/Projects/MAZE2027`. `~/Documents` синхронизируется с iCloud (`~/Library/Mobile Documents/com~apple~CloudDocs/Documents` существует), в Finder висит ошибка синхронизации. В iCloud уезжают `.git`, `.pio` (163 МБ) и `sim/runs` (3,5 ГБ). Локально выгруженных файлов нет (проверено: 0 dataless), перемещать безопасно.
- У пользователя есть общая папка `~/maze2026` (вне iCloud): `printed_circuit_boards/`, `tdp/`, `poster2026/`, `video/`, `main_programs/`, `test_programs/`, `documentation/`, `.venv/`.
- Корень репозитория сейчас — смесь трёх миров:
  - прошивка ESP: `platformio.ini`, `CMakeLists.txt`, `sdkconfig.4d_systems_esp32s3_gen4_r8n16`, `src/`, `include/`, `components/` (449 файлов), `compile_commands.json` (генерируется), `lib/` и `test/` (в обоих только README от PlatformIO);
  - код Pi вне `pi/`: `map_web.py`, `victim_detect.py`, `detect_app.py`, `victim_static_test.py`, `templates_detect/`, `camera_web/`. `.claude/skills/pi_connect/deploy.py` кладёт и `pi/X`, и `X` из корня в `~/maze/X` на Pi, то есть на Pi они уже лежат рядом с модулями `pi/`. Связи: `detect_app.py` и `map_web.py` импортируют `victim_detect`; `victim_static_test.py` тоже; тесты `pi/tests/test_continuous_safety.py`, `test_drift.py`, `test_nose_down_wall.py`, `test_reliability.py` упоминают эти модули; `pi/tests/fakes.py` кладёт `pi/` в `sys.path`;
  - прочее: `sim/` (симулятор больше не используется, `sim/runs` 3,5 ГБ в `.gitignore`), `logs/` (в `.gitignore`), `__pycache__/`, `pyrightconfig.json` (`include: ["pi", "*.py"]`), `docs/`, `CLAUDE.md`.
- Пути, зашитые в настройке Claude: `.claude/skills/esp-flash/SKILL.md` (команда `cd ~/Documents/PlatformIO/Projects/MAZE2027 && … pio run -t upload`), задача по расписанию `~/.claude/scheduled-tasks/claude-config-gc-monthly/SKILL.md`, память `~/.claude/projects/-Users-sanechek-Documents-PlatformIO-Projects-MAZE2027/memory/`. Хуки (`.claude/hooks/esp_build.py`, `guard_rules.py` — проверяет `src/main.c`, `codemap.py` — группы путей, `block_generated.py`), правила `.claude/rules/esp-idf.md` (`paths: src/**, include/**, platformio.ini, CMakeLists.txt`) и `pi-python.md` (`paths: pi/**, *.py`), скиллы `maze-verify`, `esp-flash` ссылаются на `src/`, `include/`.

## Цель (предложение, утверждено пользователем — см. «Открытые вопросы»)

```
~/maze2026/robot/            ← git-репозиторий (вне iCloud, рядом с платами и TDP)
  CLAUDE.md  README.md  .gitignore  pyrightconfig.json  .claude/
  firmware/                  ← всё для ESP32-S3, PlatformIO-проект
    platformio.ini  CMakeLists.txt  sdkconfig.*  src/  include/  components/
  pi/                        ← весь код Raspberry Pi, включая бывшие корневые скрипты
    map_web.py  victim_detect.py  detect_app.py  victim_static_test.py
    templates_detect/  camera_web/  …
  docs/                      ← тексты; картинки уже в docs/assets/
  archive/sim/               ← симулятор без runs, на случай если понадобится
  logs/                      ← в .gitignore
```

Корень после переезда: 4 папки + 3–4 служебных файла.

## Правила проекта

- Заезд кончается только на старте (никаких FATAL); комплекты сразу у жертвы; YOLO только в сторону стены.
- Техчелленж (`pi/challenge.py`, `pi/letters_cv.py`, `docs/tech_challenges.md`) не трогать — только переносить вместе со всеми, если он в перемещаемой папке.
- Только переезд и раскладка: **логику кода не менять,** не рефакторить, не «улучшать заодно». Меняются пути, импорты и конфиги, ничего больше.
- Проверка — maze-verify (юнит-тесты, сборка ESP); симулятор не используется и не запускается.
- Перемещать через `git mv`, чтобы сохранить историю. Ничего не удалять насовсем: лишнее — в Корзину macOS (`mv … ~/.Trash/`) после подтверждения пользователя.
- В дереве есть незакоммиченные правки пользователя в `pi/`, `src/`, `include/`. Их не коммитить без вопроса и не терять: перед началом спросить, закоммитить их отдельным коммитом или оставить как есть (тогда `git mv` переносит и их).
- Не прошивать ESP и не заливать на Pi без команды пользователя.

## Что сделано (не ломать)

- Коммит `6f9d0e9` (ветка `chore/light-worker-agent`): карта кода и её хук, правила по путям, проектные скиллы и хуки в git, симулятор убран из процесса.
- Тесты: `python3 -m unittest discover -s pi/tests` — 376 тестов, OK (29 пропусков «нужен numpy» — норма).

## Задачи

### Этап A — раскладка внутри репозитория (сессия открыта в старой папке)

1. **Снимок «до».** `pio run` в корне (запомнить размер `.pio/build/*/firmware.bin` и что сборка зелёная), юнит-тесты (число тестов и OK). Записать в progress.
2. **Прошивка в `firmware/`.** `git mv platformio.ini CMakeLists.txt sdkconfig.4d_systems_esp32s3_gen4_r8n16 src include components firmware/`. `lib/` и `test/` (только README) — в Корзину после подтверждения. Старую `.pio/` и `compile_commands.json` удалить из корня (генерируемые, 163 МБ), пересобрать в `firmware/`: `pio run -d firmware`, затем `pio run -d firmware -t compiledb`. **Готово:** сборка зелёная, размер `firmware.bin` совпадает с «до» (±несколько байт из-за времени сборки — сказать честно, если больше).
3. **Код Pi в `pi/`.** `git mv map_web.py victim_detect.py detect_app.py victim_static_test.py templates_detect camera_web pi/`. Проверить импорты (`sys.path` в `detect_app.py`, `template_folder='templates_detect'` в нём же — путь относительно файла, должен остаться верным), тесты `pi/tests`, `deploy.py` (корень → `~/maze/X` и `pi/X` → `~/maze/X`: на Pi расклад не меняется — подтвердить по коду, не по памяти). Проверить `pi_connect/SKILL.md`, `yolo-retrain/SKILL.md` и команды запуска, где упоминаются эти файлы из корня. **Готово:** тесты OK, число тестов как «до», все ссылки обновлены (`git grep` по именам файлов).
4. **Симулятор в архив.** `git mv sim archive/sim`; `sim/runs` (3,5 ГБ, не в git) — спросить: в Корзину или вынести вне репозитория. `.gitignore`: `archive/sim/runs/`.
5. **Настройка Claude под новую раскладку.** Обновить: `.claude/hooks/esp_build.py` и `guard_rules.py` (пути `src/` → `firmware/src/`), `codemap.py` (группы), `.claude/rules/esp-idf.md` (`paths: firmware/**`), `pi-python.md` (`paths: pi/**`), `pyrightconfig.json` (`include: ["pi"]`, exclude `firmware`, `archive`), скиллы `esp-flash` и `maze-verify` (команды `pio run -d firmware …`), `CLAUDE.md` (пути, раздел о структуре — 3–4 строки), `docs/robot_code_map.md` при необходимости. Пересобрать `docs/CODEMAP.md`. **Готово:** `git grep -nE '(^|[^/])\b(src|include)/'` в `.claude/`, `CLAUDE.md`, `docs/` не находит старых путей прошивки; хуки отрабатывают на пробной правке (`firmware/src/*.c` → сборка, `pi/*.py` → тесты).
6. **README.md** в корне: 10–15 строк — что за репозиторий, четыре папки, как собрать прошивку и залить на Pi.
7. **Коммит** одним-двумя коммитами (только после «да» пользователя), без чужих правок, если пользователь решил их не включать.

### Этап B — переезд папки (после коммита этапа A)

8. Дать пользователю одну команду переезда (`mv ~/Documents/PlatformIO/Projects/MAZE2027 ~/maze2026/robot`) и попросить закрыть VS Code с проектом перед ней. Сам переезд не выполнять из сессии, которая работает в этой папке.
9. Подготовить (и описать в progress) действия после переезда, которые выполнит **новая сессия, открытая в `~/maze2026/robot`**:
   - память Claude: скопировать `~/.claude/projects/-Users-sanechek-Documents-PlatformIO-Projects-MAZE2027/memory/` в папку проекта под новым путём (`-Users-sanechek-maze2026-robot`), старую оставить до проверки;
   - путь в `esp-flash/SKILL.md` и в задаче `claude-config-gc-monthly` (через `update_scheduled_task`);
   - `pio run -d firmware` и `-t compiledb` (абсолютные пути в `compile_commands.json`), юнит-тесты;
   - VS Code + PlatformIO: открыть папку `firmware/` (или workspace-файл с ней) — PlatformIO IDE ищет `platformio.ini` в корне рабочей папки;
   - убедиться, что в `~/Documents/PlatformIO/Projects/` и в iCloud не осталось копии.

## Открытые вопросы (спросить в начале, без ответа не делать)

1. Имя и место: `~/maze2026/robot` или другое (`~/maze2026/MAZE2027`, `~/maze2026/main_programs/...`)?
2. Прошивку — в `firmware/` (чистый корень, `pio run -d firmware`, VS Code открывать на `firmware/`) или оставить в корне (проще для VS Code, корень грязнее)?
3. Незакоммиченные правки в `pi/`, `src/`, `include/`: закоммитить отдельно до переезда или переносить как есть?
4. `sim/runs` (3,5 ГБ): в Корзину или вынести вне репозитория? Нужен ли `archive/sim` вообще или весь `sim/` — в Корзину?
5. `logs/`: оставить в корне (в `.gitignore`) или складывать вне репозитория, например `~/maze2026/robot-logs/`?
