#!/usr/bin/env python3
"""LD19 в отдельном процессе: читает порт, считает стены (walls.measure) и
построчно отдаёт результат в stdout как JSON - nav.py читает его своим
потоком (см. _lidar_thread там).

Зачем отдельный процесс. Раньше лидар читался потоком внутри nav.py, на чистом
Python, и делил GIL с YOLO, телеметрией и петлёй моторов. Пока шёл скан
жертвы, поток не успевал забирать порт: 230400 бод - это ~23 КБ/с, буфер
порта переполнялся за доли секунды, и обороты приходили с дырами - бока
после скана читались пустыми, и в карту шло "стены нет". Здесь свой
интерпретатор и свой GIL: разбор пакетов (CRC по байту) и walls.measure()
больше никому не мешают, а nav.py получает ~10 коротких строк в секунду.

Неполные обороты (потеряли пакеты - в оборот вошла дыра по углу) не
отдаются вовсе: лучше на 100мс позже, чем замер, в котором пропуск точек
выглядит как "стены нет". Сколько таких было - в поле "dropped".

Строка на оборот:
    {"w": <словарь walls.measure()>, "npts": N, "dropped": K, "t": T, "dbg": ...}
"t" - time.monotonic() на момент конца оборота (шов сзади, см. ld19.SEAM_DEG):
перед робота отсканирован ~50 мс назад, зад - в самом конце.
"dbg" - только когда борт перешёл из "нет стены" в "есть стену": сторона и
сырые точки ±40° вокруг неё, для wall_debug.log в nav.py.

Запуск вручную (проверка):  ~/maze/venv/bin/python ~/maze/lidar_proc.py
"""

from __future__ import annotations

import json
import sys
import time

from ld19 import DEFAULT_PORT, BAUDRATE, LD19
from walls import measure, SIDE_THRESH

# Шаг между соседними точками ~0.8°, пакет из 12 точек - ~10°. Дыра больше
# этого - потерян хотя бы один пакет.
MAX_GAP_DEG = 5.0
# Оборот короче этой доли от обычного - тоже неполный (например, первый
# после подключения или разрыв на стыке оборотов, который дырой не виден).
MIN_PTS_FRAC = 0.8
DBG_HALF_DEG = 40.0
# Нет пакетов дольше - процесс падает, nav.py его перезапускает.
SILENCE_S = 2.0
# Столько коротких оборотов подряд (~2с) - "обычное" число точек пересчитывается.
SHORT_RESET = 20


def _max_gap(rev) -> float:
    """Самая большая дыра по углу внутри оборота, включая стык 360->0."""
    angs = sorted(p[0] for p in rev)
    gap = angs[0] + 360.0 - angs[-1]
    for a, b in zip(angs, angs[1:]):
        if b - a > gap:
            gap = b - a
    return gap


def _side_pts(rev, center):
    out = []
    for a, d, _ in rev:
        off = abs(a - center)
        if min(off, 360.0 - off) <= DBG_HALF_DEG:
            out.append((round(a), d))
    out.sort()
    return out


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUDRATE
    out = sys.stdout
    dropped = 0
    typical = None          # обычное число точек в обороте (скользящее)
    short_run = 0           # коротких оборотов подряд
    prev_wr = prev_wl = False
    with LD19(port, baud) as lidar:
        for rev in lidar.revolutions(SILENCE_S):
            n = len(rev)
            if n == 0:
                continue
            if typical is None:
                typical = float(n)
            short = n < MIN_PTS_FRAC * typical
            if short:
                # Много коротких подряд - это уже не потери пакетов, а
                # завышенное "обычное" (разовый всплеск числа точек): без
                # сброса оно не падало бы, и отбрасывались бы все обороты.
                short_run += 1
                if short_run >= SHORT_RESET:
                    typical, short_run, short = float(n), 0, False
            else:
                short_run = 0
                # Только по полным оборотам, иначе серия битых тянула бы
                # "обычное" вниз и переставала бы считаться битой.
                typical += 0.1 * (n - typical)
            if short or _max_gap(rev) > MAX_GAP_DEG:
                dropped += 1
                continue

            w = measure(rev)
            msg = {"w": w, "npts": n, "dropped": dropped, "t": time.monotonic()}

            wr = w["right"] is not None and w["right"] < SIDE_THRESH
            wl = w["left"] is not None and w["left"] < SIDE_THRESH
            if (wr and not prev_wr) or (wl and not prev_wl):
                side, center = ("RIGHT", 90.0) if (wr and not prev_wr) else ("LEFT", 270.0)
                msg["dbg"] = {"side": side, "pts": _side_pts(rev, center)}
            prev_wr, prev_wl = wr, wl

            out.write(json.dumps(msg) + "\n")
            out.flush()


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        # Ctrl+C приходит всей группе процессов, а закрытый pipe - это
        # nav.py уже вышел. И то и другое - штатный конец, без трассы. stdout
        # подменяем на /dev/null, иначе его сброс при выходе снова упадёт
        # на закрытом pipe и напечатает "Exception ignored".
        import os
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
