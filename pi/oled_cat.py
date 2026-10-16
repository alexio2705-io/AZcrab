#!/usr/bin/env python3
"""Кот для дисплея робота - нарисован кодом под экран 128x64 по 1 биту.

    python3 pi/oled_cat.py ~/maze/cat.oledv            # собрать анимацию
    ~/maze/venv/bin/python ~/maze/oled.py --play ~/maze/cat.oledv

Почему не из ролика: на 128x64 по одному биту обработанное видео кота
читается как пятно. Здесь силуэт светится целиком, а глаза, нос, рот и
полоски - тёмные, как у мультяшного стикера; усы торчат наружу.

Что происходит (8 с по кругу, 10 кадров в секунду): кот покачивается,
моргает, дёргает ухом, потом довольно щурится, краснеет и над ним
всплывает сердечко. Только стандартная библиотека и oled.Canvas.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import oled  # noqa: E402
import oled_video  # noqa: E402

FPS = 10
SECONDS = 8.0
CX = 64.0


def fill_ellipse(cv, cx, cy, rx, ry, on=True):
    for y in range(int(cy - ry) - 1, int(cy + ry) + 2):
        for x in range(int(cx - rx) - 1, int(cx + rx) + 2):
            if ((x - cx) / rx) ** 2 + ((y - cy) / ry) ** 2 <= 1.0:
                cv.pixel(x, y, on)


def fill_poly(cv, pts, on=True):
    """Заливка многоугольника (по центрам пикселей)."""
    ys = [p[1] for p in pts]
    for y in range(int(min(ys)), int(max(ys)) + 1):
        yc = y + 0.5
        xs = []
        for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
            if (y0 <= yc < y1) or (y1 <= yc < y0):
                xs.append(x0 + (yc - y0) * (x1 - x0) / (y1 - y0))
        xs.sort()
        for a, b in zip(xs[::2], xs[1::2]):
            for x in range(int(math.ceil(a - 0.5)), int(math.floor(b - 0.5)) + 1):
                cv.pixel(x, y, on)


def line(cv, x0, y0, x1, y1, on=True):
    n = max(1, int(max(abs(x1 - x0), abs(y1 - y0))))
    for k in range(n + 1):
        cv.pixel(round(x0 + (x1 - x0) * k / n), round(y0 + (y1 - y0) * k / n), on)


def heart(cv, x, y, s=1.0):
    fill_ellipse(cv, x - 2.2 * s, y, 2.6 * s, 2.4 * s)
    fill_ellipse(cv, x + 2.2 * s, y, 2.6 * s, 2.4 * s)
    fill_poly(cv, [(x - 4.8 * s, y + 0.6 * s), (x + 4.8 * s, y + 0.6 * s), (x, y + 6 * s)])


def cat(cv, t):
    """Кадр в момент t (с) от начала цикла."""
    happy = 4.2 <= t < 6.6
    bob = 1.6 * math.sin(2 * math.pi * t / 1.6) + (1.2 * math.sin(2 * math.pi * t / 0.4) if happy else 0)
    squash = 0.8 * math.sin(2 * math.pi * t / 1.6)      # чуть шире, когда ниже
    hy = 27 + bob                                          # центр головы
    by = 47 + bob * 0.6                                    # центр тела

    # тело и лапки
    fill_ellipse(cv, CX, by, 21 + squash, 13)
    for fx in (CX - 9, CX + 9):
        fill_ellipse(cv, fx, by + 12, 5, 3.5)
    # уши (левое иногда дёргается)
    twitch = -3 if 2.4 <= t < 2.7 else 0
    for side in (-1, 1):
        tip_dx = twitch if side < 0 else 0
        base_out, base_in, tip = (CX + side * 27, hy - 6), (CX + side * 9, hy - 16), (CX + side * 22 + tip_dx, hy - 25)
        fill_poly(cv, [base_out, tip, base_in])
        # тёмная серединка уха
        inner = [(base_out[0] - side * 4, base_out[1] - 3), (tip[0] - side * 1.5, tip[1] + 6),
                 (base_in[0] + side * 4, base_in[1] + 1)]
        fill_poly(cv, inner, on=False)
    # голова
    fill_ellipse(cv, CX, hy, 28 + squash, 19)
    # полоски на лбу
    for dx, top in ((-6, 0), (0, -1), (6, 0)):
        line(cv, CX + dx, hy - 17 + top, CX + dx, hy - 12, on=False)

    # глаза
    ey = hy + 1
    blink = 1.1 <= t < 1.3 or 3.5 <= t < 3.7 or 7.2 <= t < 7.4
    for ex in (CX - 11, CX + 11):
        if happy:                                          # ^ ^
            line(cv, ex - 5, ey + 2, ex, ey - 2, on=False)
            line(cv, ex, ey - 2, ex + 5, ey + 2, on=False)
            line(cv, ex - 5, ey + 3, ex, ey - 1, on=False)
            line(cv, ex, ey - 1, ex + 5, ey + 3, on=False)
        elif blink:
            line(cv, ex - 5, ey + 1, ex + 5, ey + 1, on=False)
            line(cv, ex - 5, ey + 2, ex + 5, ey + 2, on=False)
        else:
            fill_ellipse(cv, ex, ey, 4.6, 5.6, on=False)
            fill_ellipse(cv, ex + 1.6, ey - 2.2, 1.4, 1.4)   # блик
    # нос и рот «ω»
    ny = hy + 7
    fill_poly(cv, [(CX - 2.5, ny), (CX + 2.5, ny), (CX, ny + 2.5)], on=False)
    for side in (-1, 1):
        cv.pixel(round(CX + side * 1), round(ny + 3), False)
        cv.pixel(round(CX + side * 2), round(ny + 4), False)
        cv.pixel(round(CX + side * 3), round(ny + 4), False)
        cv.pixel(round(CX + side * 4), round(ny + 3), False)
    # румянец
    if happy:
        for side in (-1, 1):
            bx = CX + side * 18
            for k in range(3):
                line(cv, bx - 3 + k * 3, ny + 2, bx - 1 + k * 3, ny - 1, on=False)
    # усы - наружу от щёк, светятся на чёрном
    for side in (-1, 1):
        x0 = CX + side * 27
        for dy, slope in ((-1, -2), (2, 0), (5, 2)):
            line(cv, x0, ny - 2 + dy, x0 + side * 12, ny - 2 + dy + slope)
    # сердечко всплывает, пока кот доволен
    if happy:
        k = (t - 4.2) / 2.4
        heart(cv, CX + 46 + 2 * math.sin(k * 6), 26 - 24 * k, 1.0 + 0.3 * k)


def frames():
    out = []
    n = int(SECONDS * FPS)
    for i in range(n):
        cv = oled.Canvas()
        cat(cv, i / FPS)
        out.append(bytes(cv.buf))
    return out


def main():
    dst = sys.argv[1] if len(sys.argv) > 1 else "cat.oledv"
    fr = frames()
    with open(os.path.expanduser(dst), "wb") as f:
        f.write(oled_video.MAGIC + bytes([FPS]) + b"".join(fr))
    print(f"{dst}: {len(fr)} кадров, {FPS} к/с, {len(fr) / FPS:.1f} с")


if __name__ == "__main__":
    main()
