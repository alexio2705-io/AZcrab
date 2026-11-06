---
name: esp-flash
description: Использовать, когда надо собрать, прошить или послушать ESP32-S3 робота MAZE2027 — «прошей», «залей прошивку на есп», «подключил есп, прошивай», «закрыл, прошивай», «открой монитор», «что пишет есп», правки в firmware/src/ или firmware/include/, тест моторов/кнопок/сервы/шин на ESP.
---

# Прошивка ESP32-S3 (MAZE2027)

ESP-IDF через PlatformIO, окружение `4d_systems_esp32s3_gen4_r8n16`. `pio` не в PATH:
`~/.platformio/penv/bin/pio`. Всё запускать из корня репозитория.

## Перед сборкой

Тестовые переключатели в `firmware/src/main.c` перекрывают лабиринт. Для заезда должно быть:
`BUTTON_TEST 0`, `DRIVE_TEST 0`, `RUN_MAZE 1`. Включать тест — только по просьбе
пользователя, а после теста вернуть и сказать об этом (лучше отдельным коммитом).
Режим теста шин — `TEST_TURN` в `firmware/src/drive_test.c` (1 — поворот на `TURN_TEST_DEG`, 0 — проезд `DRIVE_TEST_CM`).

Менялся пакет телеметрии (`firmware/include/robot_link.h`, `firmware/src/robot_link.c`) → поднять
`TELEM_VER` и синхронно `TELEM_VER`/`FMT` в `pi/nav.py`, потом залить на Pi
(pi_connect). Иначе Pi видит 0 Гц.

## Сборка и прошивка

```bash
~/.platformio/penv/bin/pio run -d firmware 2>&1 | grep -iE "error|warning" | grep -v "Flash memory size mismatch"
ls /dev/cu.usbmodem*
lsof /dev/cu.usbmodem* 2>/dev/null
~/.platformio/penv/bin/pio run -d firmware -t upload --upload-port /dev/cu.usbmodem101 2>&1 | tail -12
```

- Предупреждение `Flash memory size mismatch` — норма, не чинить.
- Нет `/dev/cu.usbmodem*` → ESP не подключена по USB-C: попросить подключить.
- `lsof` показывает процесс → у пользователя открыт монитор (VS Code). **Не убивать** его процесс: попросить закрыть и ждать «закрыл».
- Не подключается к чипу → попросить: зажать BOOT, нажать RST, отпустить BOOT; повторить.
- Порт бывает `usbmodem101` или `usbmodem1101` — брать тот, что в `ls`.

## Послушать вывод

Интерактивный `pio device monitor` здесь не работает (нужен терминал). Читать скриптом:

```bash
~/.platformio/penv/bin/python .claude/skills/esp-flash/serial_read.py 20 'BTN|MOTOR_CAN|E \('
```

После прошивки ESP перезагружается — скрипт ждёт порт сам. Начало загрузки
(BNO08x, `MOTOR_CAN: Probing ids`) — первые ~4 с: читать сразу после прошивки.

## Если пользователь прошивает сам

Дать одну команду из корня проекта:

```bash
cd ~/Documents/PlatformIO/Projects/MAZE2027 && ~/.platformio/penv/bin/pio run -d firmware -t upload && ~/.platformio/penv/bin/pio device monitor
```

## Частые ошибки

| Симптом | Что на самом деле |
|---|---|
| `MOTOR_CAN: id N: NO REPLY` | сначала питание моторов и CAN-провод к этому id, потом код |
| кнопка не меняется в выводе | дважды оказывалась мёртвая кнопка — сначала просить прозвонить/нажать BOOT для сравнения |
| Pi: «0 Гц», «жду пакеты N байт версии V» | разные `TELEM_VER` на ESP и Pi |
