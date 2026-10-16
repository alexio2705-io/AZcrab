#!/usr/bin/env python3
"""Видео для дисплея робота (oled.py): ролик -> кадры 128x64 по 1 биту.

    python3 pi/oled_video.py кот.mp4 cat.oledv --start 0 --dur 16.3 --fps 10
    ~/maze/venv/bin/python ~/maze/oled.py --play ~/maze/cat.oledv     # на Pi, по кругу
    ~/maze/venv/bin/python ~/maze/oled.py --stop

Перегонка - на маке (нужен ffmpeg), на Pi нужен только load() - стандартная
библиотека, как и весь oled.py.

Как кадр становится однобитным. OLED светит белым на чёрном, поэтому светлый
фон ролика (стикер на белом) не рисуется: всё почти белое, что связано с
краем кадра, - фон, гаснет. Остальное - яркость через упорядоченный дизеринг
(матрица Байера 4x4): узор привязан к пикселям, а не к кадру, поэтому от кадра
к кадру не мерцает, как мерцала бы диффузия ошибки.

Файл: b"OLV1", кадров в секунду (1 байт), дальше кадры по 1024 байта в
раскладке контроллера (как Canvas.buf).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

W, H = 128, 64
MAGIC = b"OLV1"
BAYER = [[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]]


def check_output(dst, src=None):
    """Куда писать кадры - только в .oledv, и только поверх такого же файла.
    Иначе можно затереть исходный ролик (так однажды и вышло: старый
    генератор понял аргументы по-другому и записал кадры в .mp4)."""
    dst = os.path.abspath(os.path.expanduser(dst))
    if not dst.endswith(".oledv"):
        raise SystemExit(f"куда писать - должен быть файл .oledv, а не {dst}")
    if src is not None and os.path.abspath(os.path.expanduser(src)) == dst:
        raise SystemExit("исходник и результат - один и тот же файл")
    if os.path.exists(dst):
        with open(dst, "rb") as f:
            if f.read(4) != MAGIC:
                raise SystemExit(f"{dst} существует и это не видео дисплея - не перезаписываю")
    return dst


def load(path):
    """-> (кадров в секунду, [кадр 1024 байта, ...])."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != MAGIC:
        raise ValueError(f"{path}: не видео дисплея (нет {MAGIC!r})")
    fps = data[4]
    body = data[5:]
    n = len(body) // (W * H // 8)
    return fps, [body[i * 1024:(i + 1) * 1024] for i in range(n)]


def _sobel(g, w, h, x, y):
    def p(xx, yy):
        return g[min(h - 1, max(0, yy)) * w + min(w - 1, max(0, xx))]
    gx = (p(x + 1, y - 1) + 2 * p(x + 1, y) + p(x + 1, y + 1)
          - p(x - 1, y - 1) - 2 * p(x - 1, y) - p(x - 1, y + 1))
    gy = (p(x - 1, y + 1) + 2 * p(x, y + 1) + p(x + 1, y + 1)
          - p(x - 1, y - 1) - 2 * p(x, y - 1) - p(x + 1, y - 1))
    return (gx * gx + gy * gy) ** 0.5


def frame_bits(gray, w, h, bg_level=232, lo=70, hi=215, outline=True, edges=None, dark=None,
               speck=3):
    """Кадр w x h (байты яркости) -> 1024 байта для экрана, по центру.
    lo..hi растягивается на всю яркость: шерсть средней яркости иначе вся
    уходит в одинаковый узор. outline - силуэт обведён (край у фона горит).
    edges - порог перепада яркости (Собель): тогда вместо заливки горят только
    контуры - силуэт и резкие перепады внутри (глаза, нос, полоски).
    dark - «мультяшный» вид: силуэт горит целиком, гаснет только темнее
    dark (глаза, нос, тёмные полоски); тёмные пятна меньше speck пикселей
    не гасим - это шум сжатия, от него морда рябит."""
    # фон: почти белое, связанное с краем
    bg = bytearray(w * h)
    todo = [(x, y) for x in range(w) for y in (0, h - 1)] + \
           [(x, y) for y in range(h) for x in (0, w - 1)]
    while todo:
        x, y = todo.pop()
        i = y * w + x
        if bg[i] or gray[i] < bg_level:
            continue
        bg[i] = 1
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < w and 0 <= ny < h and not bg[ny * w + nx]:
                todo.append((nx, ny))
    # Не фон - это кот и мелкие отдельные пятна (значок соцсети и т.п.):
    # оставляем только самый большой связный кусок, остальное - фон.
    seen = bytearray(w * h)
    best = []
    for start in range(w * h):
        if bg[start] or seen[start]:
            continue
        comp, todo = [], [start]
        seen[start] = 1
        while todo:
            i = todo.pop()
            comp.append(i)
            x, y = i % w, i // w
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                j = ny * w + nx
                if 0 <= nx < w and 0 <= ny < h and not bg[j] and not seen[j]:
                    seen[j] = 1
                    todo.append(j)
        if len(comp) > len(best):
            best = comp
    keep = set(best)
    for i in range(w * h):
        if not bg[i] and i not in keep:
            bg[i] = 1
    off = None
    if dark is not None:
        # тёмные пятна внутри силуэта; мелкие (шум) - не считаем
        off = bytearray(w * h)
        seen2 = bytearray(w * h)
        for s0 in range(w * h):
            if bg[s0] or seen2[s0] or gray[s0] >= dark:
                continue
            comp, todo = [], [s0]
            seen2[s0] = 1
            while todo:
                i = todo.pop()
                comp.append(i)
                x, y = i % w, i // w
                for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                    j = ny * w + nx
                    if (0 <= nx < w and 0 <= ny < h and not bg[j] and not seen2[j]
                            and gray[j] < dark):
                        seen2[j] = 1
                        todo.append(j)
            if len(comp) >= speck:
                for i in comp:
                    off[i] = 1
    out = bytearray(W * H // 8)
    x0, y0 = (W - w) // 2, (H - h) // 2
    for y in range(h):
        for x in range(w):
            i = y * w + x
            if bg[i]:
                continue
            edge = outline and any(
                0 <= nx < w and 0 <= ny < h and bg[ny * w + nx]
                for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)))
            if off is not None:
                on = not off[i]
            elif edges is not None:
                on = edge or _sobel(gray, w, h, x, y) > edges
            else:
                v = min(1.0, max(0.0, (gray[i] - lo) / (hi - lo)))
                on = edge or v * 16 > BAYER[y % 4][x % 4] + 0.5
            if on:
                sx, sy = x0 + x, y0 + y
                out[(sy >> 3) * W + sx] |= 1 << (sy & 7)
    return bytes(out)


def make(src, dst, start=0.0, dur=None, fps=10, crop=None, height=64, bg_level=232,
         lo=70, hi=215, edges=None, dark=None):
    """Ролик -> файл кадров. crop - "w:h:x:y" для ffmpeg (по исходнику)."""
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                            "-show_entries", "stream=width,height", "-of", "csv=p=0", src],
                           capture_output=True, text=True, check=True).stdout.split(",")
    sw, sh = int(probe[0]), int(probe[1])
    if crop:
        cw, ch = (int(float(v)) for v in crop.split(":")[:2])
    else:
        cw, ch = sw, sh
    h = min(H, height)
    w = min(W, round(cw * h / ch / 2) * 2)
    dst = check_output(dst, src)
    vf = (f"crop={crop}," if crop else "") + f"fps={fps},scale={w}:{h}:flags=area,format=gray"
    cmd = ["ffmpeg", "-v", "error", "-ss", str(start)] + (["-t", str(dur)] if dur else []) + \
          ["-i", src, "-vf", vf, "-f", "rawvideo", "-"]
    raw = subprocess.run(cmd, capture_output=True, check=True).stdout
    size = w * h
    frames = [frame_bits(raw[i:i + size], w, h, bg_level, lo, hi, edges=edges, dark=dark)
              for i in range(0, len(raw) - size + 1, size)]
    with open(dst, "wb") as f:
        f.write(MAGIC + bytes([fps]) + b"".join(frames))
    return len(frames), w, h


def main():
    ap = argparse.ArgumentParser(description="Ролик -> видео для дисплея робота")
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--dur", type=float)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--crop", help="w:h:x:y по исходнику (ffmpeg crop)")
    ap.add_argument("--bg", type=int, default=232, help="ярче этого у края - фон, гаснет")
    ap.add_argument("--lo", type=int, default=70, help="темнее этого - не горит")
    ap.add_argument("--hi", type=int, default=215, help="ярче этого - горит целиком")
    ap.add_argument("--edges", type=float, help="только контуры: порог перепада яркости (~150-300)")
    ap.add_argument("--dark", type=int, help="мультяшно: силуэт горит, гаснет темнее этого (~60-100)")
    a = ap.parse_args()
    n, w, h = make(a.src, a.dst, a.start, a.dur, a.fps, a.crop, bg_level=a.bg, lo=a.lo, hi=a.hi,
                   edges=a.edges, dark=a.dark)
    print(f"{a.dst}: {n} кадров {w}x{h} по центру, {a.fps} к/с, {n / a.fps:.1f} с")


if __name__ == "__main__":
    sys.exit(main())
