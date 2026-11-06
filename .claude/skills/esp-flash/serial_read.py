#!/usr/bin/env python3
"""Читать вывод ESP по USB заданное время - без интерактивного монитора.

    ~/.platformio/penv/bin/python .claude/skills/esp-flash/serial_read.py [СЕК] [РЕГЕКСП] [--port PORT]

По умолчанию 15 с, порт - первый /dev/cu.usbmodem*. Переживает перезагрузку
ESP (порт пропадает и появляется снова). РЕГЕКСП - печатать только
подходящие строки (например 'BTN|MOTOR_CAN|E \\('). Скорость не важна:
у ESP32-S3 это USB-CDC.
"""
import glob
import re
import sys
import time

import serial  # есть в ~/.platformio/penv


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    return ports[0] if ports else None


def main():
    args = sys.argv[1:]
    port = None
    if "--port" in args:
        i = args.index("--port")
        port = args[i + 1]
        del args[i:i + 2]
    secs = float(args[0]) if args else 15.0
    pat = re.compile(args[1]) if len(args) > 1 else None

    t0 = time.time()
    s = None
    while time.time() - t0 < secs:
        if s is None:
            p = port or find_port()
            if not p:
                time.sleep(0.3)
                continue
            try:
                s = serial.Serial(p, 921600, timeout=0.2)
                print(f"--- {time.time() - t0:5.1f}с: открыт {p}", flush=True)
            except serial.SerialException as e:
                if "Resource busy" in str(e) or "Errno 16" in str(e):
                    print(f"[ЗАНЯТ] {p}: открыт монитор - попросить пользователя закрыть его")
                    return 2
                time.sleep(0.3)
                continue
        try:
            line = s.readline().decode(errors="replace").rstrip()
        except (serial.SerialException, OSError):
            print(f"--- {time.time() - t0:5.1f}с: порт пропал (перезагрузка?)", flush=True)
            s = None
            continue
        if line and (pat is None or pat.search(line)):
            print(line, flush=True)
    if s is None and not (port or find_port()):
        print("[НЕТ ПОРТА] /dev/cu.usbmodem* не найден - ESP не подключена по USB")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
