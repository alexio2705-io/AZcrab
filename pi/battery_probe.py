"""Напряжение батареи для меню экрана, пока nav.py не запущен.

Телеметрию ESP в меню никто не читает, поэтому демон экрана раз в PERIOD_S
сам открывает порт на WINDOW_S, берёт пакеты v4 (protocol.py) и закрывает
его. Порт держит другая программа (nav.py, telem.py, tile_calib.py...) -
не трогаем: два читателя одного порта делят байты, и оба теряют пакеты.
Напряжение - наименьшее из свежих у четырёх моторов (их шина питания).
"""
from __future__ import annotations

import binascii
import os
import struct
import threading
import time

from protocol import FMT, MAGIC, PKT_SIZE, TELEM_VER

PORT = "/dev/serial0"
BAUD = 921600
WINDOW_S = 0.3          # 60 пакетов при 200 Гц - хватает с запасом
PERIOD_S = 5.0
FRESH_S = 15.0          # старше - в меню "--.-V"
DV_FIELDS = slice(21, 25)   # motor_dv в struct.unpack(FMT): 0.1 В, 0 - нет ответа


def port_in_use(port=PORT, proc="/proc"):
    """Открыт ли порт другим процессом (по /proc/*/fd)."""
    target = os.path.realpath(port)
    me = str(os.getpid())
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit() and p != me]
    except OSError:
        return False
    for pid in pids:
        fd_dir = os.path.join(proc, pid, "fd")
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                    return True
            except OSError:
                continue
    return False


def volts_from_bytes(buf):
    """Наименьшее напряжение моторов (В) из пакетов в buf или None."""
    best = None
    i = buf.find(MAGIC)
    while 0 <= i <= len(buf) - PKT_SIZE:
        pkt = buf[i:i + PKT_SIZE]
        crc = pkt[-2] | pkt[-1] << 8
        if pkt[2] == TELEM_VER and binascii.crc_hqx(bytes(pkt[2:-2]), 0xFFFF) == crc:
            dv = [v for v in struct.unpack(FMT, bytes(pkt[2:]))[DV_FIELDS] if v]
            if dv:
                best = min(dv) / 10.0
            i = buf.find(MAGIC, i + PKT_SIZE)
        else:
            i = buf.find(MAGIC, i + 1)
    return best


def read_volts(port=PORT, window_s=WINDOW_S):
    """Открыть порт на window_s и вернуть напряжение или None."""
    import serial
    try:
        with serial.Serial(port, BAUD, timeout=0.05) as link:
            buf = bytearray()
            end = time.monotonic() + window_s
            while time.monotonic() < end:
                buf += link.read(4096)
    except (serial.SerialException, OSError):
        return None
    return volts_from_bytes(buf)


class BatteryProbe:
    """Фоновый замер. wanted - меню на экране и никто не занят; volts(now) -
    последнее напряжение или None, если оно старше FRESH_S."""

    def __init__(self, start=True):
        self.wanted = False
        self._v, self._t = None, -1e9
        if start:
            threading.Thread(target=self._run, daemon=True, name="oled-battery").start()

    def volts(self, now):
        return self._v if now - self._t <= FRESH_S else None

    def _run(self):
        while True:
            if self.wanted and not port_in_use():
                v = read_volts()
                if v is not None:
                    self._v, self._t = v, time.monotonic()
            time.sleep(PERIOD_S)
