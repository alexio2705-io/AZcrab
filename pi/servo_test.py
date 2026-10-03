#!/usr/bin/env python3
"""Проверка сервы дозатора с малины - те же команды, что шлёт nav.py.

Серва висит на ESP (GPIO10, servo.c), малина только просит её по UART:
"K:L" - выдать из левого отсека, "K:R" - из правого. ESP сам отводит серву
в крайнее положение, держит SERVO_HOLD_MS и возвращает в центр - около
секунды, и всё это время команды не принимает (см. robot_link.h).

Запуск (nav.py при этом должен быть остановлен - порт один):
    ~/maze/venv/bin/python ~/maze/servo_test.py            # клавишами: l / r / q
    ~/maze/venv/bin/python ~/maze/servo_test.py L R L      # по очереди и выйти
    ~/maze/venv/bin/python ~/maze/servo_test.py R -n 5     # пять раз подряд
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import serial

LINK_PORT = "/dev/serial0"
LINK_BAUD = 921600
MAGIC = b"\xAA\x55"     # начало пакета телеметрии ESP (robot_link.c)
DISPENSE_MOVE_S = 1.3   # как в nav.py: ход сервы туда-обратно с запасом


def nav_running() -> bool:
    """Запущен ли nav.py - он держит тот же порт, команды бы перемешались."""
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                if b"nav.py" in f.read():
                    return True
        except OSError:
            pass
    return False


def esp_alive(link: serial.Serial, wait_s: float = 1.0) -> int:
    """Сколько пакетов телеметрии пришло за wait_s - ESP жив и линк цел."""
    link.reset_input_buffer()
    end = time.monotonic() + wait_s
    buf = b""
    while time.monotonic() < end:
        buf += link.read(link.in_waiting or 1)
    return buf.count(MAGIC)


def dispense(link: serial.Serial, side: str) -> None:
    name = "левый" if side == "L" else "правый"
    print(f"  K:{side} - {name} отсек...", end="", flush=True)
    link.write(f"K:{side}\n".encode())
    link.flush()
    time.sleep(DISPENSE_MOVE_S)
    print(" готово")


def interactive(link: serial.Serial) -> None:
    import termios
    import tty

    print("l - левый, r - правый, q - выход")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            ch = sys.stdin.read(1).lower()
            if ch in ("q", "\x03", "\x1b"):
                break
            if ch in ("l", "r"):
                dispense(link, ch.upper())
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    ap = argparse.ArgumentParser(description="проверка сервы дозатора")
    ap.add_argument("sides", nargs="*", type=str.upper, choices=["L", "R"],
                    help="какие отсеки по очереди; без них - клавишами")
    ap.add_argument("-n", type=int, default=1, help="сколько раз повторить список")
    ap.add_argument("-p", "--port", default=LINK_PORT)
    args = ap.parse_args()

    if nav_running():
        sys.exit("nav.py запущен и держит порт - останови его сначала")

    with serial.Serial(args.port, LINK_BAUD, timeout=0.1) as link:
        n = esp_alive(link)
        if n == 0:
            sys.exit(f"с {args.port} не пришло ни одного пакета телеметрии - "
                     f"ESP выключен, не та прошивка или не тот порт")
        print(f"ESP на связи ({n} пакетов/с)")
        if args.sides:
            for _ in range(args.n):
                for side in args.sides:
                    dispense(link, side)
        else:
            interactive(link)


if __name__ == "__main__":
    main()
