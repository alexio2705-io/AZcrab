# MAZE2027: программная часть робота

Программная часть робота для RoboCup Junior Rescue Maze. Железо (платы, модели, документы) лежит рядом, вне этого репозитория.

- `firmware/` — прошивка ESP32-S3 (ESP-IDF через PlatformIO): моторы, датчики, связь с Pi.
- `pi/` — весь код Raspberry Pi 5: навигация по лидару, YOLO по меткам жертв, веб-карта и дашборд.
- `docs/` — описания, карта кода (`docs/CODEMAP.md`) и правила проекта.
- `archive/sim/` — старый симулятор, не используется.

Сборка прошивки:

```bash
pio run -d firmware
```

Прошивка ESP (при подключённой плате):

```bash
pio run -d firmware -t upload
```

Юнит-тесты Pi:

```bash
python3 -m unittest discover -s pi/tests
```

Заливка файлов на Pi (ключ по SSH, копия в `~/maze/`):

```bash
python3 .claude/skills/pi_connect/deploy.py HOST pi/nav.py
```

Правила работы с репозиторием и роботом — в `CLAUDE.md`.
