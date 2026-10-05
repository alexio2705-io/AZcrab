#!/usr/bin/env python3
"""Телеметрия ESP в консоль - для отладки линка, кнопок и датчиков.

Читает тот же пакет, что nav.py (раскладка FMT и TELEM_VER оттуда), и
печатает строку 5 раз в секунду. Смена статуса ESP, BTN_1 или концевиков
печатается сразу отдельной строкой с ">>>" - короткое нажатие между
обычными строками не теряется.

Запуск (nav.py при этом должен быть остановлен - порт один):
    ~/maze/venv/bin/python ~/maze/telem.py
    ~/maze/venv/bin/python ~/maze/telem.py --hz 20
Выход - Ctrl+C.
"""
import argparse
import struct
import time

import serial

LINK_PORT = "/dev/serial0"
LINK_BAUD = 921600
MAGIC = b"\xAA\x55"
# ДОЛЖНО СОВПАДАТЬ с telem_t и TELEM_VER в robot_link.c (и с nav.py).
TELEM_VER = 2
FMT = "<BfffffffBHHHHffBBB"
PKT_SIZE = len(MAGIC) + struct.calcsize(FMT)

COLORS = {0: "?", 1: "WHITE", 2: "BLUE", 3: "BLACK", 4: "MIRROR", 5: "RED"}


def flags_str(fl):
    return (f"BTN1={'ДА ' if fl & 16 else 'нет'} "
            f"C1={'ДА ' if fl & 1 else 'нет'} C2={'ДА ' if fl & 2 else 'нет'} "
            f"encL={'ok' if fl & 4 else '--'} encR={'ok' if fl & 8 else '--'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=LINK_PORT)
    ap.add_argument("--hz", type=float, default=5.0, help="строк в секунду")
    args = ap.parse_args()

    link = serial.Serial(args.port, LINK_BAUD, timeout=0.1)
    print(f"{args.port} @ {LINK_BAUD}, жду пакеты {PKT_SIZE} байт версии {TELEM_VER}. "
          f"Ctrl+C - выход", flush=True)

    buf = bytearray()
    t0 = time.monotonic()
    t_print = 0.0
    t_rate = t0
    good = bad_crc = 0
    bad_ver = {}
    rate_cnt = 0
    hz = 0.0
    last_events = None

    try:
        while True:
            buf += link.read(max(1, min(link.in_waiting, 4096)))
            while len(buf) >= PKT_SIZE:
                if buf[0] != MAGIC[0] or buf[1] != MAGIC[1]:
                    del buf[0]
                    continue
                body = bytes(buf[2:PKT_SIZE])
                # Случайный маркер в мусоре: при ошибке съедаем 1 байт, а не
                # весь кадр. Чужая версия - кадр, за которым сразу маркер.
                if body[0] != TELEM_VER:
                    if (len(buf) >= PKT_SIZE + 2 and buf[PKT_SIZE] == MAGIC[0]
                            and buf[PKT_SIZE + 1] == MAGIC[1]):
                        bad_ver[body[0]] = bad_ver.get(body[0], 0) + 1
                    del buf[0]
                    continue
                crc = 0
                for b in body[:-1]:
                    crc ^= b
                if crc != body[-1]:
                    bad_crc += 1
                    del buf[0]
                    continue
                del buf[:PKT_SIZE]
                (_v, yaw, yaw_rate, pitch, roll, ax, ay, az, color,
                 cr, cg, cb, cc, lcm, rcm, status, fl, _c) = struct.unpack(FMT, body)
                good += 1
                rate_cnt += 1

                now = time.monotonic()
                t = now - t0
                if now - t_rate >= 1.0:
                    hz = rate_cnt / (now - t_rate)
                    rate_cnt = 0
                    t_rate = now

                st = "PAUSED" if status else "RUN"
                events = (status, fl & 16, fl & 1, fl & 2)
                if events != last_events:
                    if last_events is not None:
                        print(f">>> {t:7.2f}s  статус={st}  {flags_str(fl)}", flush=True)
                    last_events = events

                if now - t_print >= 1.0 / args.hz:
                    t_print = now
                    print(f"{t:7.2f}s {hz:4.0f}Гц [{st:6}] "
                          f"yaw={yaw:6.1f} rate={yaw_rate:+6.1f} "
                          f"pitch={pitch:+5.1f} roll={roll:+5.1f} "
                          f"acc=({ax:+5.2f},{ay:+5.2f},{az:+5.2f}) "
                          f"цвет={COLORS.get(color, color):6} "
                          f"rgbc=({cr},{cg},{cb},{cc}) "
                          f"L={lcm:7.1f} R={rcm:7.1f}см  {flags_str(fl)}",
                          flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        print(f"\nхороших пакетов {good}, битых CRC {bad_crc}", end="")
        if bad_ver:
            print(f", ЧУЖАЯ ВЕРСИЯ {bad_ver} - прошивка ESP и .py разных версий",
                  end="")
        print(flush=True)


if __name__ == "__main__":
    main()
