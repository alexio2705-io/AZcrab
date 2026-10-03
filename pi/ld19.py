#!/usr/bin/env python3
"""Чтение лидара LDROBOT LD19 на Raspberry Pi.

Подключение по схеме:
    J2.1 raw_5v   -> питание лидара (5 В, ~250 мА при старте мотора)
    J2.2 GND      -> общая земля
    J2.4 TX_LIDAR -> RPi pin 21 = GPIO9 = RXD3  (лидар только передаёт)

На Raspberry Pi 5 (RP1) GPIO8/9 - это UART3, поэтому в config.txt нужна
строка `dtoverlay=uart3-pi5`; порт появляется как /dev/ttyAMA3.
Скорость 230400, 8N1. На Pi 4 те же пины - это UART4 (`dtoverlay=uart4`,
/dev/ttyAMA4), порт задаётся флагом -p.

Протокол: пакет 47 байт, 12 точек на пакет, ~5000 точек/с (~10 Гц оборот).

Углы на выходе - в системе робота: 0 = перёд, 90 = право, 180 = зад,
270 = лево. Пересчёт из сырого угла лидара:
    robot = (ZERO_DIR + raw) % 360,  ZERO_DIR = 90
Нулевой угол лидара - это стрелка на его корпусе, и на этом роботе она
смотрит в правый борт, отсюда ZERO_DIR = 90. Углы растут в ту же сторону,
что и в системе робота (по часовой), поэтому MIRROR = False.

Проверка на месте: поставить робота так, чтобы стена была только с одной
стороны, и убедиться, что walls.py показывает её с той же стороны. Если
борта поменялись местами - зеркалит, переключается MIRROR; если стена
уехала на 90° - врёт ZERO_DIR.
"""

from __future__ import annotations

import struct
import time
from collections import namedtuple
from dataclasses import dataclass

import serial

HEADER = 0x54
VER_LEN = 0x2C  # тип пакета 1 (верхние 3 бита) + 12 точек (нижние 5)
POINTS_PER_PACKET = 12
PACKET_LEN = 47
BAUDRATE = 230400
DEFAULT_PORT = "/dev/ttyAMA3"
# ориентация лидара на роботе: robot_angle = (90 + raw) % 360
ZERO_DIR = 90.0    # угол в системе робота, куда смотрит нулевой угол лидара
MIRROR = False     # True: углы лидара растут против часовой в системе робота


def _make_crc_table(poly: int = 0x4D) -> tuple:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _make_crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


# angle - градусы 0..360, distance - мм (0 = нет измерения), intensity - 0..255
Point = namedtuple("Point", "angle distance intensity")


@dataclass
class Packet:
    speed: int          # скорость вращения, град/с
    start_angle: float  # градусы
    end_angle: float    # градусы
    timestamp: int      # мс, переполняется на 30000
    points: list        # список Point


def to_robot(raw: float, zero_dir: float = ZERO_DIR,
             mirror: bool = MIRROR) -> float:
    """Сырой угол лидара -> угол в системе робота (0 перёд, 90 право)."""
    return ((zero_dir - raw) if mirror else (zero_dir + raw)) % 360.0


def parse_packet(frame: bytes, zero_dir: float = ZERO_DIR,
                 mirror: bool = MIRROR) -> Packet:
    """Разбор проверенного по CRC кадра длиной PACKET_LEN.

    Углы точек сразу переводятся в систему робота (см. to_robot).
    """
    speed, start_raw = struct.unpack_from("<HH", frame, 2)
    end_raw, timestamp = struct.unpack_from("<HH", frame, 42)

    start = start_raw / 100.0
    end = end_raw / 100.0
    # угол монотонно растёт, но может перескочить через 360
    span = (end_raw - start_raw) % 36000 / 100.0
    step = span / (POINTS_PER_PACKET - 1)

    points = []
    for i in range(POINTS_PER_PACKET):
        dist, intensity = struct.unpack_from("<HB", frame, 6 + i * 3)
        angle = to_robot(start + step * i, zero_dir, mirror)
        points.append(Point(angle, dist, intensity))

    return Packet(speed, to_robot(start, zero_dir, mirror),
                  to_robot(end, zero_dir, mirror), timestamp, points)


class LD19:
    """Потоковый парсер LD19 поверх последовательного порта."""

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = BAUDRATE,
                 timeout: float = 1.0, zero_dir: float = ZERO_DIR,
                 mirror: bool = MIRROR):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.zero_dir, self.mirror = zero_dir, mirror
        self._buf = bytearray()

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> "LD19":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def read_packet(self) -> Packet | None:
        """Один валидный пакет. None - таймаут порта (лидар молчит)."""
        while True:
            if len(self._buf) >= PACKET_LEN:
                i = self._buf.find(bytes([HEADER, VER_LEN]))
                if i < 0:
                    # заголовка нет; последний байт может быть началом кадра
                    del self._buf[:-1]
                else:
                    del self._buf[:i]
                    if len(self._buf) >= PACKET_LEN:
                        frame = bytes(self._buf[:PACKET_LEN])
                        if crc8(frame[:-1]) == frame[-1]:
                            del self._buf[:PACKET_LEN]
                            return parse_packet(frame, self.zero_dir,
                                                self.mirror)
                        del self._buf[:2]  # ложный заголовок, ищем дальше
                        continue

            chunk = self.ser.read(self.ser.in_waiting or 1)
            if not chunk:
                return None
            self._buf += chunk

    def packets(self):
        while True:
            pkt = self.read_packet()
            if pkt is None:
                continue
            yield pkt

    def points(self):
        for pkt in self.packets():
            yield from pkt.points

    def revolutions(self):
        """Полные обороты как списки Point, без огрубления в сектора.

        Оборот считается завершённым, когда угол перескочил через 0.
        """
        pts = []
        prev_angle = 0.0
        partial = True  # подключились в середине оборота - первый неполный
        for pkt in self.packets():
            for p in pkt.points:
                # при MIRROR углы убывают, поэтому ловим скачок в любую сторону
                if abs(p.angle - prev_angle) > 180.0:  # прошли через ноль
                    if not partial:
                        yield pts
                    partial = False
                    pts = []
                prev_angle = p.angle
                pts.append(p)

    def scans(self, bins: int = 360, min_intensity: int = 0):
        """Полные обороты как список из `bins` дистанций в мм (0 = нет данных)."""
        k = bins / 360.0
        for pts in self.revolutions():
            scan = [0] * bins
            for angle, dist, intensity in pts:
                if dist and intensity >= min_intensity:
                    scan[int(angle * k) % bins] = dist
            yield scan


def _sector(scan, center: int, width: int = 20) -> int:
    """Медиана валидных дистанций в секторе вокруг `center` градусов."""
    vals = sorted(d for i in range(center - width // 2, center + width // 2 + 1)
                  for d in (scan[i % len(scan)],) if d)
    return vals[len(vals) // 2] if vals else 0


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="LD19 lidar dump")
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("-b", "--baud", type=int, default=BAUDRATE)
    ap.add_argument("--zero-dir", type=float, default=ZERO_DIR,
                    help="куда смотрит нулевой угол лидара, град")
    args = ap.parse_args()

    with LD19(args.port, args.baud, zero_dir=args.zero_dir) as lidar:
        t0 = time.monotonic()
        n = 0
        for scan in lidar.scans():
            n += 1
            hz = n / (time.monotonic() - t0)
            print(f"{hz:4.1f} Гц  "
                  f"front {_sector(scan, 0):5d}  "
                  f"right {_sector(scan, 90):5d}  "
                  f"back {_sector(scan, 180):5d}  "
                  f"left {_sector(scan, 270):5d}  мм", flush=True)


if __name__ == "__main__":
    main()
