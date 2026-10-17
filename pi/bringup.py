#!/usr/bin/env python3
"""Проверка Pi после включения: всё ли на месте перед заездом. Только читает.

    ~/maze/venv/bin/python ~/maze/bringup.py            # чек-лист
    ~/maze/venv/bin/python ~/maze/bringup.py --hashes   # sha1 файлов - сверить с репозиторием

Что смотрит: Python и модули venv, I2C и дисплей (отвечает ли на шине, поднят
ли демон и автозапуск), связь с ESP (идут ли пакеты нужной версии и с какой
частотой), лидар LD19 (идут ли пакеты), камеры, кнопки старта, калибровку
плиток, модель YOLO, питание и температуру, место на диске.

Ничего не меняет и не запускает моторы: порт ESP только читается (команды не
шлются), дисплею уходит NOP - команда «ничего не делать», чтобы узнать, есть
ли он на шине. Если nav.py или tile_calib.py уже работают, порты ESP и
лидара заняты ими - эти пункты пропускаются, а не ломают заезд.

Итог - строка на пункт: OK / WARN / FAIL, код выхода 1, если есть FAIL.
Демон дисплея запущен - итог показывается и на экране робота.

Сверка файлов с репозиторием (с мака):
    ssh madness@<ip> '~/maze/venv/bin/python ~/maze/bringup.py --hashes' > /tmp/pi.sha1
    (cd pi && shasum *.py dash/*.* | sort -k2) | diff - <(sort -k2 /tmp/pi.sha1)
"""
from __future__ import annotations

import glob
import hashlib
import os
import re
import shutil
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

MAZE = os.path.expanduser("~/maze")
I2C_DEV = "/dev/i2c-1"
LIDAR_PORT = "/dev/ttyAMA3"
LIDAR_BAUD = 230400
LIDAR_HEADER = b"\x54\x2c"        # начало пакета LD19
# как в nav.py.wait_start_button: у GPIO25 своя RC-цепь (без подтяжки), у GPIO27 - внутренняя
BUTTONS = {25: ("старт (правая рука)", False), 27: ("старт (левая рука)", True)}
MODULES = ("serial", "RPi.GPIO", "cv2", "numpy", "flask")

results = []                      # (уровень, пункт, подробности)


def report(level, what, detail=""):
    results.append((level, what, detail))
    mark = {"OK": "  OK ", "WARN": "WARN ", "FAIL": "FAIL ", "SKIP": "  -- "}[level]
    print(f"{mark} {what}" + (f": {detail}" if detail else ""), flush=True)


def run(cmd, timeout=5.0):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()
    except (OSError, subprocess.SubprocessError) as e:
        return None, str(e)


def busy_processes():
    """Кто уже держит порты: nav.py / tile_calib.py / color.py / telem.py."""
    _, out = run(["pgrep", "-af", "python"])
    names = ("nav.py", "tile_calib.py", "color.py", "telem.py", "lift.py",
             "servo_test.py", "ld19_web.py", "raw_scan.py")
    return [ln for ln in out.splitlines() if any(n in ln for n in names)
            and "bringup.py" not in ln]


# ── пункты ─────────────────────────────────────────────────────
def check_python():
    v = sys.version_info
    report("OK" if v >= (3, 9) else "WARN", "Python", f"{v.major}.{v.minor} ({sys.executable})")
    missing = []
    for m in MODULES:
        try:
            __import__(m)
        except Exception:               # noqa: BLE001 - любой сбой импорта - «нет модуля»
            missing.append(m)
    if missing:
        report("FAIL", "модули", "нет: " + ", ".join(missing) + " (venv не тот?)")
    else:
        report("OK", "модули", ", ".join(MODULES))


def check_display():
    import oled
    if not os.path.exists(I2C_DEV):
        report("FAIL", "I2C", f"нет {I2C_DEV} - включи: sudo raspi-config nonint do_i2c 0, перезагрузка")
        return
    if not os.access(I2C_DEV, os.R_OK | os.W_OK):
        report("FAIL", "I2C", f"нет прав на {I2C_DEV} - пользователь не в группе i2c")
        return
    running = oled.daemon_running()
    if running:
        report("OK", "дисплей", "демон работает (шину держит он, на ней не проверяю)")
    else:
        try:
            fd = os.open(I2C_DEV, os.O_RDWR)
            try:
                found = None
                for a in oled.ADDRS:
                    oled.fcntl.ioctl(fd, oled.I2C_SLAVE, a)
                    try:
                        os.write(fd, b"\x00\xe3")      # NOP
                        found = a
                        break
                    except OSError:
                        continue
            finally:
                os.close(fd)
            if found is None:
                report("FAIL", "дисплей", "на 0x3C/0x3D никто не отвечает - разъём J14, питание")
            else:
                report("WARN", "дисплей", f"на шине (0x{found:02X}), но демон не запущен "
                                          f"(поднимет nav.py; или systemctl start maze-oled)")
        except OSError as e:
            report("FAIL", "дисплей", f"{e}")
    _, out = run(["systemctl", "is-enabled", "maze-oled"])
    if out.strip() == "enabled":
        report("OK", "автозапуск дисплея", "maze-oled.service включён")
    else:
        report("WARN", "автозапуск дисплея",
               "сервис не установлен - IP после загрузки не покажется (см. maze-oled.service)")


def read_port(port, baud, seconds):
    import serial
    s = serial.Serial(port, baud, timeout=0.1)
    try:
        s.reset_input_buffer()
        buf = bytearray()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            buf += s.read(4096)
        return bytes(buf)
    finally:
        s.close()


def count_packets(data, magic, ver, size):
    """-> (пакетов нужной версии, начал пакета с другой версией) в сыром потоке."""
    good = bad_ver = 0
    i = 0
    while True:
        i = data.find(magic, i)
        if i < 0 or i + size > len(data):
            return good, bad_ver
        if data[i + len(magic)] == ver:
            good += 1
            i += size
        else:
            bad_ver += 1
            i += len(magic)


def check_esp(busy):
    import color as link            # раскладка пакета - та же, что в nav.py
    if busy:
        report("SKIP", "ESP", "порт занят: " + busy[0][:60])
        return
    try:
        data = read_port(link.LINK_PORT, link.LINK_BAUD, 1.0)
    except Exception as e:          # noqa: BLE001
        report("FAIL", "ESP", f"{link.LINK_PORT}: {e}")
        return
    if not data:
        report("FAIL", "ESP", "за 1 с ни байта - ESP выключена, провод, или прошивка висит")
        return
    good, bad_ver = count_packets(data, link.MAGIC, link.TELEM_VER, link.PKT_SIZE)
    if good == 0:
        report("FAIL", "ESP", f"{len(data)} байт, но ни одного пакета версии {link.TELEM_VER} "
                              f"({bad_ver} с другой) - прошивка и .py разошлись")
    elif good < 120:
        report("WARN", "ESP", f"{good} пакетов/с (ожидается ~200)")
    else:
        report("OK", "ESP", f"{good} пакетов/с, версия {link.TELEM_VER}")


def check_lidar(busy):
    if busy:
        report("SKIP", "лидар", "порт занят: " + busy[0][:60])
        return
    if not os.path.exists(LIDAR_PORT):
        report("FAIL", "лидар", f"нет {LIDAR_PORT} - UART3 не включён в config.txt")
        return
    try:
        data = read_port(LIDAR_PORT, LIDAR_BAUD, 1.0)
    except Exception as e:          # noqa: BLE001
        report("FAIL", "лидар", f"{LIDAR_PORT}: {e}")
        return
    n = data.count(LIDAR_HEADER)
    if n == 0:
        report("FAIL", "лидар", f"{len(data)} байт за 1 с, пакетов LD19 нет - питание мотора, кабель")
    elif n < 300:
        report("WARN", "лидар", f"{n} пакетов/с (обычно ~450)")
    else:
        report("OK", "лидар", f"{n} пакетов/с")


from watch_logic import CAM_NAMES      # noqa: E402  номер камеры -> сторона


def frame_ok(output):
    """Вывод rpicam-hello -> пришли ли кадры. Код выхода не помогает: при
    «Camera frontend has timed out» rpicam-hello выходит с 0."""
    low = output.lower()
    return "timed out" not in low and "timeout" not in low


def check_cameras(busy):
    if busy:
        report("SKIP", "камеры", "заняты: " + busy[0][:60])
        return
    rc, out = run(["rpicam-hello", "--list-cameras"], timeout=10.0)
    if rc is None:
        report("WARN", "камеры", "rpicam-hello не найден - не проверить")
        return
    idx = [int(m.group(1)) for m in (re.match(r"^\s*(\d+)\s*:", ln) for ln in out.splitlines()) if m]
    if not idx:
        report("FAIL", "камеры", "ни одной в списке - шлейфы")
        return
    # Видна в списке - ещё не значит, что работает: камера с отошедшим
    # шлейфом отвечает по I2C, но кадров не шлёт. Снимаем по секунде с каждой.
    for i in sorted(set(idx) | {0, 1}):
        name = f"камера {i} ({CAM_NAMES.get(i, '?')})"
        if i not in idx:
            report("FAIL", name, "нет в списке - шлейф или разъём")
            continue
        rc, out = run(["rpicam-hello", "--camera", str(i), "-t", "1000", "--nopreview"], timeout=15.0)
        if rc is None:
            report("FAIL", name, "rpicam-hello завис")
        elif frame_ok(out):
            report("OK", name, "кадры идут")
        else:
            report("FAIL", name, "видна, но кадров нет (Camera frontend timed out) - шлейф/разъём")


def check_buttons():
    try:
        import RPi.GPIO as GPIO
    except Exception as e:          # noqa: BLE001
        report("WARN", "кнопки", f"RPi.GPIO: {e}")
        return
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        stuck = []
        for pin, (name, pull) in BUTTONS.items():
            if pull:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            else:
                GPIO.setup(pin, GPIO.IN)
            if GPIO.input(pin) == GPIO.LOW:
                stuck.append(f"GPIO{pin} ({name})")
        if stuck:
            report("WARN", "кнопки", "нажаты прямо сейчас (залипла?): " + ", ".join(stuck))
        else:
            report("OK", "кнопки", "отпущены")
    except Exception as e:          # noqa: BLE001 - nav.py мог занять GPIO
        report("WARN", "кнопки", f"{e}")
    finally:
        try:
            GPIO.cleanup(list(BUTTONS))
        except Exception:           # noqa: BLE001
            pass


def check_calib():
    import tile_classify as tc
    s = tc.load()
    have = {k: len(v) for k, v in s.items()}
    if not {"white", "black"} <= have.keys():
        report("FAIL", "калибровка плиток", f"нет белой/чёрной в {tc.CALIB_PATH} - запусти tile_calib.py")
        return
    miss = [tc.NAMES[k].split(" (")[0] for k in ("blue", "silver", "red") if k not in have]
    close = [(a, b, d) for a, b, d in tc.separation(s) if d < tc.SEP_WARN]
    detail = ", ".join(f"{k}×{n}" for k, n in have.items())
    if miss or close:
        extra = (" нет: " + ", ".join(miss) if miss else "") + \
                (f"; почти не различимы: {close[0][0]}/{close[0][1]}" if close else "")
        report("WARN", "калибровка плиток", detail + extra)
    else:
        report("OK", "калибровка плиток", detail)


def check_model():
    models = glob.glob(os.path.join(MAZE, "models", "*.pt")) + \
        glob.glob(os.path.join(MAZE, "models", "*_ncnn_model"))
    want = os.environ.get("MAZE_MODEL", "B")
    if not models:
        report("FAIL", "модель YOLO", f"в {MAZE}/models пусто")
        return
    names = sorted(os.path.basename(m) for m in models)
    has = any(f"maze_v26n_{want}" in n for n in names)
    report("OK" if has else "FAIL", "модель YOLO",
           f"нужна maze_v26n_{want} (MAZE_MODEL) - " + ("есть" if has else "НЕТ") +
           f"; всего {len(names)}")


def check_power():
    _, out = run(["vcgencmd", "get_throttled"])
    try:
        thr = int(out.partition("=")[2], 16)
    except ValueError:
        report("WARN", "питание", "vcgencmd не ответил")
        thr = None
    if thr is not None:
        if thr & 0x1:
            report("FAIL", "питание", "напряжение низкое прямо сейчас")
        elif thr & 0x10000:
            report("WARN", "питание", f"с загрузки были просадки (0x{thr:x})")
        else:
            report("OK", "питание", "просадок не было")
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            t = int(f.read()) / 1000.0
        report("OK" if t < 70 else "WARN", "температура CPU", f"{t:.0f}°C")
    except (OSError, ValueError):
        pass
    free = shutil.disk_usage(MAZE if os.path.isdir(MAZE) else "/").free / 1e9
    report("OK" if free > 2 else "WARN", "место на диске", f"{free:.1f} ГБ свободно")


def check_network():
    _, out = run(["hostname", "-I"])
    ips = [a for a in out.split() if "." in a]
    report("OK" if ips else "WARN", "сеть", " ".join(ips) or "IP нет")


def hashes():
    files = sorted(glob.glob(os.path.join(HERE, "*.py")) + glob.glob(os.path.join(HERE, "dash", "*.*")))
    for p in files:
        if os.path.isfile(p):
            with open(p, "rb") as f:
                rel = os.path.relpath(p, HERE)
                print(f"{hashlib.sha1(f.read()).hexdigest()}  {rel}")


def main():
    if "--hashes" in sys.argv:
        hashes()
        return 0
    print(f"Проверка {socket.gethostname()} - {time.strftime('%Y-%m-%d %H:%M:%S')}")
    busy = busy_processes()
    if busy:
        print("  (уже работают: " + "; ".join(b[:50] for b in busy) + ")")
    for fn in (check_network, check_python, check_display, lambda: check_esp(busy),
               lambda: check_lidar(busy), lambda: check_cameras(busy), check_buttons, check_calib,
               check_model, check_power):
        try:
            fn()
        except Exception as e:      # noqa: BLE001 - один пункт не должен ронять весь чек-лист
            report("FAIL", getattr(fn, "__name__", "пункт"), f"{type(e).__name__}: {e}")
    n = {k: sum(1 for r in results if r[0] == k) for k in ("OK", "WARN", "FAIL", "SKIP")}
    print(f"\nИтого: OK {n['OK']}, WARN {n['WARN']}, FAIL {n['FAIL']}, пропущено {n['SKIP']}")
    try:
        import oled
        bad = [f"{w}: {d}" for lvl, w, d in results if lvl == "FAIL"] + \
              [f"{w}: {d}" for lvl, w, d in results if lvl == "WARN"]
        title = "ПРОВЕРКА: ВСЁ ОК" if not bad else f"ОШИБОК {n['FAIL']}, ЗАМЕЧ {n['WARN']}"
        oled.say(title, bad[:3] or [f"OK {n['OK']} пунктов"], small=True,
                 alert=bool(n["FAIL"]), ttl=20.0)
    except Exception:               # noqa: BLE001
        pass
    return 1 if n["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main())
