#!/usr/bin/env python3
"""ESP<->RPi UART link test. Протокол см. link_test.h на ESP.

Один порт на оба направления - подтверждено по нетлисту платы: ESP GPIO3(TX)
/GPIO9(RX) через резисторы R8/R23 идут на Pi GPIO15/GPIO14 (физ. пины 10/8) -
это штатный /dev/serial0, никакой dtoverlay не нужен, он поднят из коробки.

    /dev/serial0  - телеметрия ESP -> Pi (энкодеры, цвет, концевики, гироскоп)
                    И команды Pi -> ESP, один порт
    /dev/ttyAMA3  - сам лидар LD19        (dtoverlay=uart3-pi5), только с --lidar

По умолчанию Pi отвечает ESP случайным числом ("D:<0..1000>") - просто чтобы
подтвердить, что обратный канал работает, лидар для этого не нужен. Флаг
--lidar переключает источник на реальную дистанцию до передней стены.

Запуск:  ~/maze/venv/bin/python ~/maze/link_test.py
"""

from __future__ import annotations

import random
import struct
import sys
import threading
import time

import serial

from ld19 import DEFAULT_PORT as LIDAR_PORT, BAUDRATE as LIDAR_BAUD, LD19
from walls import measure

LINK_PORT = "/dev/serial0"
UART_BAUD = 921600

MAGIC = bytes([0xAA, 0x55])
# yaw pitch color clicker1 clicker2 encL encR lidar_echo enc_online crc
FMT = "<ffBBBfffBB"
BODY_SIZE = struct.calcsize(FMT)          # 25
PKT_SIZE = len(MAGIC) + BODY_SIZE         # 27
COLORS = {0: "?", 1: "WHITE", 2: "BLUE", 3: "BLACK", 4: "MIRROR", 5: "RED"}

CMD_HZ = 8  # как часто шлём "D:<фронт>" на ESP


def read_packet(port: serial.Serial, timeout: float = 0.3):
    """Синхронизация по magic 0xAA 0x55, разбор, проверка CRC. dict или None."""
    deadline = time.monotonic() + timeout
    prev = 0
    while time.monotonic() < deadline:
        b = port.read(1)
        if not b:
            continue
        cur = b[0]
        if prev == MAGIC[0] and cur == MAGIC[1]:
            break
        prev = cur
    else:
        return None

    body = port.read(BODY_SIZE)
    if len(body) < BODY_SIZE:
        return None

    crc_calc = 0
    for x in body[:-1]:
        crc_calc ^= x
    if crc_calc != body[-1]:
        return None

    yaw, pitch, color, c1, c2, encl, encr, lidar_echo, enc_online, _crc = \
        struct.unpack(FMT, body)
    return {
        "yaw": yaw, "pitch": pitch,
        "color": COLORS.get(color, "?"), "color_id": color,
        "clicker1": bool(c1), "clicker2": bool(c2),
        "enc_left": encl, "enc_right": encr,
        "enc_left_online": bool(enc_online & 1),
        "enc_right_online": bool(enc_online & 2),
        "lidar_echo": lidar_echo,
    }


def send_cmd(port: serial.Serial, text: str, lock: threading.Lock) -> None:
    with lock:
        port.write((text + "\n").encode())
        port.flush()


def telem_loop(port: serial.Serial, stop: threading.Event) -> None:
    n = 0
    t0 = time.monotonic()
    while not stop.is_set():
        pkt = read_packet(port)
        if pkt is None:
            continue
        n += 1
        if n % 4 == 0:  # печатаем не каждый пакет - иначе консоль не читается
            hz = n / (time.monotonic() - t0)
            el = f"{pkt['enc_left']:.1f}" if pkt["enc_left_online"] else "off"
            er = f"{pkt['enc_right']:.1f}" if pkt["enc_right_online"] else "off"
            print(f"[TELEM {hz:4.1f}Hz] yaw={pkt['yaw']:6.1f} pitch={pkt['pitch']:5.1f}  "
                  f"цвет={pkt['color']:6s}  C1={int(pkt['clicker1'])} C2={int(pkt['clicker2'])}  "
                  f"encL={el:>6s} encR={er:>6s}  эхо={pkt['lidar_echo']:.0f}",
                  flush=True)


def lidar_cmd_loop(port: serial.Serial, lock: threading.Lock, stop: threading.Event) -> None:
    """Читает лидар, шлёт на ESP дистанцию до передней стены как 'D:<мм>'."""
    with LD19(LIDAR_PORT, LIDAR_BAUD) as lidar:
        last_send = 0.0
        for rev in lidar.revolutions():
            if stop.is_set():
                return
            w = measure(rev)
            front = w["front"]
            now = time.monotonic()
            if front is not None and now - last_send >= 1.0 / CMD_HZ:
                send_cmd(port, f"D:{front:.0f}", lock)
                last_send = now


def random_cmd_loop(port: serial.Serial, lock: threading.Lock, stop: threading.Event) -> None:
    """Без лидара: просто шлёт на ESP случайное число - подтверждает обратный канал."""
    while not stop.is_set():
        send_cmd(port, f"D:{random.uniform(0, 1000):.0f}", lock)
        stop.wait(1.0 / CMD_HZ)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="ESP<->RPi link test")
    ap.add_argument("--port", default=LINK_PORT)
    ap.add_argument("--lidar", action="store_true",
                    help="слать реальную дистанцию с лидара вместо случайного числа")
    ap.add_argument("--no-reply", action="store_true",
                    help="не слать 'D:' вообще - только слушать телеметрию ESP")
    args = ap.parse_args()

    print(f"Открываю {args.port} (телеметрия + команды, один порт)...")
    try:
        link = serial.Serial(args.port, UART_BAUD, timeout=0.3)
    except serial.SerialException as e:
        print(f"Ошибка порта: {e}")
        sys.exit(1)

    write_lock = threading.Lock()  # порт один на чтение и запись из двух потоков
    stop = threading.Event()
    threads = [threading.Thread(target=telem_loop, args=(link, stop), daemon=True)]
    if not args.no_reply:
        reply_fn = lidar_cmd_loop if args.lidar else random_cmd_loop
        threads.append(threading.Thread(target=reply_fn,
                                         args=(link, write_lock, stop), daemon=True))
    for t in threads:
        t.start()

    source = "фронтальной дистанции лидара" if args.lidar else "случайного числа"
    print("Работаю. Ctrl+C для выхода.\n"
          f"  'D:<число>' идёт на ESP каждые ~{1000.0/CMD_HZ:.0f}мс, источник - {source}\n"
          "  'эхо' в строке телеметрии - то же число, вернувшееся с ESP\n")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        send_cmd(link, "Z", write_lock)  # на прощание - проверка команды без числа
        time.sleep(0.1)
        link.close()


if __name__ == "__main__":
    main()
