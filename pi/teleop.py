#!/usr/bin/env python3
"""Ручной пульт моторов с телефона: джойстик на странице -> "M:л,п" на ESP.

Открыть с телефона в той же Wi-Fi сети:  http://<IP робота>:8070/
Палец на джойстике - едет, отпустил - стоп. Ползунок - предел скорости.

Безопасность (по очереди, что сработает первым):
  - страница шлёт команду каждые 100 мс, пока палец на джойстике, и стоп,
    когда палец убран или вкладка ушла в фон;
  - здесь: нет команды с телефона дольше STALE_S - тормоз ("B");
  - на ESP: нет "M:" дольше 0,5 с на ходу - тормоз сам (CMD_TIMEOUT_US).

Запуск (nav.py при этом должен быть остановлен - порт один):
    ~/maze/venv/bin/python ~/maze/teleop.py
Ctrl+C - тормоз и выключение моторов ("X"), как после заезда.
Только стандартная библиотека и pyserial.
"""
from __future__ import annotations

import argparse
import binascii
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HTTP_PORT = 8070
LINK_PORT = "/dev/serial0"
LINK_BAUD = 921600
MAGIC = b"\xAA\x55"     # начало пакета телеметрии ESP (robot_link.c)
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "teleop", "index.html")

SEND_HZ = 20            # "M:" на ESP; у ESP таймаут 0,5 с - запас в 10 раз
STALE_S = 0.3           # нет команды с телефона дольше - стоп
MAX_CMS = 60.0          # потолок ползунка скорости, см/с на ободе
TURN_SHARE = 0.6        # джойстик вбок до упора - разворот на месте на 0,6 скорости


def mix(x, y, speed):
    """Джойстик (x вправо, y вперёд, оба -1..1) -> скорости колёс, см/с."""
    x = max(-1.0, min(1.0, float(x)))
    y = max(-1.0, min(1.0, float(y)))
    speed = max(0.0, min(MAX_CMS, float(speed)))
    l, r = y + TURN_SHARE * x, y - TURN_SHARE * x
    big = max(1.0, abs(l), abs(r))       # вперёд+вбок не должно превышать предел
    return l / big * speed, r / big * speed


class Target:
    """Последняя команда с телефона; протухшая значит стоп."""

    def __init__(self):
        self.lock = threading.Lock()
        self.lr = (0.0, 0.0)
        self.t = 0.0

    def set(self, l, r, now):
        with self.lock:
            self.lr, self.t = (l, r), now

    def get(self, now):
        with self.lock:
            return self.lr if now - self.t <= STALE_S else (0.0, 0.0)


def nav_running():
    """Запущен ли nav.py - он держит тот же порт, команды бы перемешались."""
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                if b"nav.py" in f.read():
                    return True
        except OSError:
            pass
    return False


class Esp:
    """Порт ESP: команды с CRC и счёт пакетов телеметрии (жив ли ESP)."""

    def __init__(self, link):
        self.link = link
        self.write_lock = threading.Lock()
        self.packets = 0
        self.hz = 0

    def cmd(self, s):
        b = s.encode()
        with self.write_lock:
            self.link.write(b + b"*%04X\n" % binascii.crc_hqx(b, 0xFFFF))   # CRC - robot_link.h

    def read_loop(self, stop):
        import serial
        t0 = time.monotonic()
        tail = b""          # последний байт прошлого куска: маркер мог разрезаться
        while not stop.is_set():
            try:
                chunk = tail + self.link.read(self.link.in_waiting or 1)
            except (serial.SerialException, OSError):
                time.sleep(0.1)     # порт на миг открыл oled.py (battery_probe) - переждать
                continue
            self.packets += chunk.count(MAGIC)
            tail = chunk[-1:]
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.hz, self.packets, t0 = self.packets, 0, now


def send_loop(esp, target, stop):
    """SEND_HZ раз в секунду: едем - "M:", только что остановились - "B"."""
    moving = False
    while not stop.wait(1.0 / SEND_HZ):
        l, r = target.get(time.monotonic())
        if l or r:
            esp.cmd(f"M:{l:.1f},{r:.1f}")
            moving = True
        elif moving:
            esp.cmd("B")
            moving = False


def make_handler(esp, target):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path != "/":
                return self._send(404, b"not found", "text/plain")
            with open(PAGE, "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")

        def do_POST(self):
            if self.path != "/drive":
                return self._send(404, b"not found", "text/plain")
            try:
                req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                l, r = mix(req["x"], req["y"], req["speed"])
            except (ValueError, KeyError, TypeError):
                return self._send(400, b"bad request", "text/plain")
            target.set(l, r, time.monotonic())
            body = json.dumps({"l": round(l, 1), "r": round(r, 1), "esp_hz": esp.hz})
            self._send(200, body.encode(), "application/json")

    return Handler


def main():
    ap = argparse.ArgumentParser(description="ручной пульт моторов с телефона")
    ap.add_argument("--port", type=int, default=HTTP_PORT, help="порт страницы")
    ap.add_argument("--serial", default=LINK_PORT)
    args = ap.parse_args()

    if nav_running():
        sys.exit("nav.py запущен и держит порт - останови его сначала")

    import serial
    link = serial.Serial(args.serial, LINK_BAUD, timeout=0.1)
    esp, target, stop = Esp(link), Target(), threading.Event()
    threading.Thread(target=esp.read_loop, args=(stop,), daemon=True).start()
    time.sleep(1.1)
    if esp.hz == 0:
        sys.exit(f"с {args.serial} нет телеметрии - ESP выключен, не та прошивка или не тот порт")
    print(f"ESP на связи ({esp.hz} пакетов/с)")
    esp.cmd("W")                          # моторы включаются только по "W"
    threading.Thread(target=send_loop, args=(esp, target, stop), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(esp, target))
    print(f"пульт: http://<IP робота>:{args.port}/   Ctrl+C - выход")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        esp.cmd("B")
        time.sleep(0.2)
        esp.cmd("X")                      # выключить: не пищат, не греются
        link.flush()
        link.close()
        print("\nмоторы выключены")


if __name__ == "__main__":
    main()
