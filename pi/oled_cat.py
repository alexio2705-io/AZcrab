#!/usr/bin/env python3
"""Кот для дисплея робота: движение - из ролика, черты - нарисованные.

    python3 pi/oled_cat.py ролик.mp4 cat.oledv --dur 15.9 --crop 944:1040:0:30 --pad-top 100 --style v2b

--dur - где кончается круг: там, где поза ближе всего к первому кадру. У
ролика с котом это 15.8 с; дальше начинается последнее «сплющивание»,
которое ролик обрывает, и на повторе кот рывком прыгал в начало.
    ~/maze/venv/bin/python ~/maze/oled.py --play ~/maze/cat.oledv

Перегонка - на маке (нужен ffmpeg), на Pi копируется готовый .oledv.
Результат - только .oledv (oled_video.check_output): исходник не затереть.

Почему так. Ролик, пережатый в 128x64 по одному биту, читается как пятно:
серая шерсть уходит в узор точек. Кот, нарисованный с нуля, читается, но
двигается не как в ролике - поворот головы и прищур из силуэта не снять.
Здесь каждый кадр собран из двух частей:

  - силуэт - прямо из кадра ролика (фон - светлое, связанное с краем), горит
    целиком: прыжки, сплющивание и наклоны - точь-в-точь;
  - вид «стикер»: силуэт обведён тёмной линией внутри края, глаза крупные
    с двумя бликами (из четырёх вариантов рисунка выбран этот);
  - черты - нарисованы там, где они в кадре: глаза ищутся как два тёмных
    пятна в верхней половине силуэта (открыты - крупные, прищур - тонкие
    полоски), от них - нос «ω», полоски на лбу, румянец (розовое в кадре) и
    усы. Голову повернул - глаза и нос сместились вместе с ней.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import oled  # noqa: E402
import oled_video  # noqa: E402

UP = 4                     # кадр разбирается в UP раз крупнее экранного


def fill_ellipse(cv, cx, cy, rx, ry, on=True):
    for y in range(int(cy - ry) - 1, int(cy + ry) + 2):
        for x in range(int(cx - rx) - 1, int(cx + rx) + 2):
            if ((x - cx) / rx) ** 2 + ((y - cy) / ry) ** 2 <= 1.0:
                cv.pixel(x, y, on)


def fill_poly(cv, pts, on=True):
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


def background(lum, w, h, level):
    """Светлое, связанное с краем кадра, плюс всё, кроме самого большого
    куска (значок соцсети и т.п.)."""
    bg = bytearray(w * h)
    todo = [(x, y) for x in range(w) for y in (0, h - 1)] + [(x, y) for y in range(h) for x in (0, w - 1)]
    while todo:
        x, y = todo.pop()
        i = y * w + x
        if bg[i] or lum[i] < level:
            continue
        bg[i] = 1
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < w and 0 <= ny < h and not bg[ny * w + nx]:
                todo.append((nx, ny))
    seen = bytearray(w * h)
    best = []
    for s0 in range(w * h):
        if bg[s0] or seen[s0]:
            continue
        comp, st = [], [s0]
        seen[s0] = 1
        while st:
            i = st.pop()
            comp.append(i)
            x, y = i % w, i // w
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                j = ny * w + nx
                if 0 <= nx < w and 0 <= ny < h and not bg[j] and not seen[j]:
                    seen[j] = 1
                    st.append(j)
        if len(comp) > len(best):
            best = comp
    keep = set(best)
    for i in range(w * h):
        if not bg[i] and i not in keep:
            bg[i] = 1
    return bg


def analyse(rgb, w, h, bg_level=225, eye_dark=60):
    """Кадр (RGB, в UP раз крупнее экранного) -> силуэт, глаза, румянец."""
    lum = [(rgb[3 * i] * 3 + rgb[3 * i + 1] * 6 + rgb[3 * i + 2]) // 10 for i in range(w * h)]
    bg = background(lum, w, h, bg_level)
    pts = [(i % w, i // w) for i in range(w * h) if not bg[i]]
    top = min(p[1] for p in pts)
    hh = max(p[1] for p in pts) - top
    cx = sum(p[0] for p in pts) / len(pts)
    y0, y1 = top + 0.22 * hh, top + 0.62 * hh
    sides = {-1: [], 1: []}
    for y in range(int(y0), int(y1)):
        for x in range(w):
            i = y * w + x
            if not bg[i] and lum[i] < eye_dark:
                sides[-1 if x < cx else 1].append((x, y))
    eyes = {}
    for s, p in sides.items():
        if len(p) >= 6:
            eyes[s] = (sum(q[0] for q in p) / len(p), sum(q[1] for q in p) / len(p), len(p))
    pink = sum(1 for i in range(w * h) if not bg[i]
               and rgb[3 * i] > 200 and rgb[3 * i + 1] < 170 and rgb[3 * i + 2] < 200)
    return {"bg": bg, "eyes": eyes, "pink": pink}


def sticker_outline(cv):
    """Как у наклейки в ролике: тёмная линия на пиксель внутри края
    силуэта, сам край остаётся светлым."""
    lit = {(x, y) for y in range(oled.H) for x in range(oled.W) if cv.get(x, y)}

    def edge(m):
        return {(x, y) for (x, y) in m
                if any(p not in m for p in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)))}
    for x, y in edge(lit - edge(lit)):
        cv.pixel(x, y, False)


# Как рисовать кота. Выбирали по картинкам на одних и тех же кадрах:
#   smooth - сколько раз сгладить край силуэта (большинство 3x3: срезает
#            зубчики и бугры, из-за которых кот был похож на кляксу);
#   neck   - тело ниже морды уже головы (голова читается отдельно);
#   eye, nose, mouth - размер глаз, носа и рта «ω» от расстояния между глазами;
#   squint_w - толщина прищура «^ ^», whisk - длина усов, пикс.
STYLES = {
    "v2": dict(smooth=2, neck=False, eye=0.34, squint_w=2, nose=0.15, mouth=0.26, whisk=12),
    "v3": dict(smooth=2, neck=True, eye=0.34, squint_w=2, nose=0.15, mouth=0.26, whisk=12),
    # выбран: как v2, черты чуть крупнее; собирать с --pad-top 100 (кот крупнее на экране)
    "v2b": dict(smooth=2, neck=False, eye=0.37, squint_w=2, nose=0.16, mouth=0.28, whisk=12),
}


def _smooth(m, times):
    for _ in range(times):
        m = {(x, y) for x in range(oled.W) for y in range(oled.H)
             if sum((x + i, y + j) in m for i in (-1, 0, 1) for j in (-1, 0, 1)) >= 5}
    return m


def _narrow_body(m, start=0.6, factor=0.84, ramp=6):
    """Ниже start высоты силуэта - уже к центру, плавно за ramp строк."""
    ys = [p[1] for p in m]
    xs = [p[0] for p in m]
    y0, y1 = min(ys), max(ys)
    cx = (min(xs) + max(xs)) / 2
    ycut = y0 + start * (y1 - y0)
    rows = {}
    for x, y in m:
        rows.setdefault(y, []).append(x)
    out = set()
    for x, y in m:
        if y < ycut:
            out.add((x, y))
            continue
        f = 1 - (1 - factor) * min(1.0, (y - ycut) / ramp)
        if cx + (min(rows[y]) - cx) * f <= x <= cx + (max(rows[y]) - cx) * f:
            out.add((x, y))
    return out


def draw_frame(a, w, h, prev, open_px, blush_px, style=None):
    """Экранный кадр: силуэт (блоки UP x UP) + нарисованные черты."""
    st = STYLES["v2"] if style is None else style
    cv = oled.Canvas()
    sw, sh = w // UP, h // UP
    x0, y0 = (oled.W - sw) // 2, (oled.H - sh) // 2
    bg = a["bg"]
    m = set()
    for y in range(sh):
        for x in range(sw):
            n = sum(1 for dy in range(UP) for dx in range(UP)
                    if not bg[(y * UP + dy) * w + x * UP + dx])
            if n * 2 > UP * UP:
                m.add((x0 + x, y0 + y))
    m = _smooth(m, int(st["smooth"]))
    if st["neck"] and m:
        m = _narrow_body(m)
    for x, y in m:
        cv.pixel(x, y)
    sticker_outline(cv)

    eyes = a["eyes"]
    if len(eyes) == 2:
        (lx, ly, ln), (rx, ry, rn) = eyes[-1], eyes[1]
        is_open = min(ln, rn) >= open_px
        lx, ly, rx, ry = x0 + lx / UP, y0 + ly / UP, x0 + rx / UP, y0 + ry / UP
    elif prev is not None:                      # глаз не нашлось - прищур там же
        (lx, ly), (rx, ry), _ = prev
        is_open = False
    else:
        return cv, None
    d = max(8.0, math.hypot(rx - lx, ry - ly))
    ux, uy = (rx - lx) / d, (ry - ly) / d       # вдоль глаз
    vx, vy = -uy, ux                            # вниз по морде
    mx, my = (lx + rx) / 2, (ly + ry) / 2

    def at(au, av):
        return mx + au * d * ux + av * d * vx, my + au * d * uy + av * d * vy

    eo = st["eye"] * d
    for ex, ey in ((lx, ly), (rx, ry)):
        if is_open:                             # крупные глаза, большой и маленький блик
            fill_ellipse(cv, ex, ey, eo * 0.85, eo, on=False)
            fill_ellipse(cv, ex + 0.3 * eo, ey - 0.38 * eo, max(1.0, 0.33 * eo), max(1.0, 0.33 * eo))
            fill_ellipse(cv, ex - 0.3 * eo, ey + 0.4 * eo, max(0.7, 0.14 * eo), max(0.7, 0.14 * eo))
        else:                                   # довольный прищур «^ ^»
            for k in range(int(st["squint_w"])):
                ax, ay = ex - eo * ux + (0.35 * eo + k) * vx, ey - eo * uy + (0.35 * eo + k) * vy
                tx, ty = ex - (0.45 * eo - k) * vx, ey - (0.45 * eo - k) * vy
                bx, by = ex + eo * ux + (0.35 * eo + k) * vx, ey + eo * uy + (0.35 * eo + k) * vy
                line(cv, ax, ay, tx, ty, on=False)
                line(cv, tx, ty, bx, by, on=False)
    s = st["nose"] * d                          # нос
    nx, ny = at(0, 0.48)
    fill_poly(cv, [(nx - s * ux, ny - s * uy), (nx + s * ux, ny + s * uy),
                   (nx + 0.9 * s * vx, ny + 0.9 * s * vy)], on=False)
    mw = st["mouth"] * d                        # рот «ω» - две дуги от носа
    for side in (-1, 1):
        cxm, cym = nx + side * 0.5 * mw * ux + 0.9 * s * vx, ny + side * 0.5 * mw * uy + 0.9 * s * vy
        for k in range(9):
            t = math.pi * k / 8
            cv.pixel(round(cxm - side * 0.5 * mw * math.cos(t) * ux + 0.45 * mw * math.sin(t) * vx),
                     round(cym - side * 0.5 * mw * math.cos(t) * uy + 0.45 * mw * math.sin(t) * vy), False)
    for k in (-1, 0, 1):                        # полоски на лбу
        bx, by = at(k * 0.3, -0.62)
        line(cv, bx - 0.13 * d * vx, by - 0.13 * d * vy, bx + 0.13 * d * vx, by + 0.13 * d * vy, on=False)
    if a["pink"] > blush_px:                    # румянец
        for side in (-1, 1):
            cxb, cyb = at(side * 0.8, 0.38)
            for k in (-1, 0, 1):
                line(cv, cxb + k * 2.2 - 1, cyb + 1.5, cxb + k * 2.2 + 1, cyb - 1.5, on=False)
    yy = int(round(ny))                         # усы - от края силуэта наружу, с изгибом
    if 0 <= yy < oled.H:
        row = [x for x in range(oled.W) if cv.get(x, yy)]
        if row:
            L = st["whisk"]
            for side, edge in ((-1, min(row)), (1, max(row))):
                for dy, bend in ((-2, -3), (1, 0), (4, 3)):
                    pts = [(edge + side * (1 + L * t), yy + dy + bend * t * t) for t in (0, 0.33, 0.66, 1.0)]
                    for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
                        line(cv, xa, ya, xb, yb)
    return cv, ((lx, ly), (rx, ry), is_open)


def sway_offsets(shots, w, fps, gain=3.0, max_px=4.0, level=0.35, min_s=0.4):
    """Добавочный сдвиг вбок для каждого кадра, пикс. экрана.

    В ролике есть участки, где кот, как стикер, быстро переключается между
    позами «наклон» и «прямо» - центр силуэта скачет всего на ~1 пикс.
    экрана, и покачивание почти не видно. На таких участках (разброс центра
    в окне ±0.15 с больше level, дольше min_s) кадр целиком сдвигается так,
    чтобы размах стал в gain раз больше. Тайминг и позы - ролика: ничего не
    придумывается, кадр не поворачивается и не режется на строки."""
    cx = []
    for s in shots:
        xs = [i % w for i, b in enumerate(s["bg"]) if not b]
        cx.append(sum(xs) / len(xs) / UP if xs else 0.0)
    half = max(1, round(0.15 * fps))
    spread = []
    for k in range(len(cx)):
        win = cx[max(0, k - half):k + half + 1]
        mu = sum(win) / len(win)
        spread.append((sum((v - mu) ** 2 for v in win) / len(win)) ** 0.5)
    out = [0.0] * len(shots)
    k = 0
    while k < len(spread):
        if spread[k] <= level:
            k += 1
            continue
        j = k
        while j < len(spread) and spread[j] > level:
            j += 1
        if j - k >= min_s * fps:
            base = sum(cx[k:j]) / (j - k)
            for i in range(k, j):
                out[i] = max(-max_px, min(max_px, (gain - 1) * (cx[i] - base)))
        k = j
    return out


def shift_frame(cv, dx):
    """Кадр целиком на dx пикс. вбок."""
    dx = int(round(dx))
    if dx == 0:
        return cv
    out = oled.Canvas()
    for y in range(oled.H):
        for x in range(oled.W):
            if cv.get(x, y):
                out.pixel(x + dx, y)
    return out


def make(src, dst, start=0.0, dur=None, fps=10, crop=None, height=64, pad_top=0, style="v2b",
         sway_gain=1.0):
    """pad_top - белое поле над кадром (пикс. исходника): кот меньше, сверху
    воздух. style - ключ STYLES."""
    dst = oled_video.check_output(dst, src)
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                            "stream=width,height", "-of", "csv=p=0", src],
                           capture_output=True, text=True, check=True).stdout.split(",")
    if crop:
        cw, ch = (int(float(v)) for v in crop.split(":")[:2])
    else:
        cw, ch = int(probe[0]), int(probe[1])
    ch += pad_top
    sh = min(oled.H, height)
    sw = min(oled.W, round(cw * sh / ch / 2) * 2)
    w, h = sw * UP, sh * UP
    vf = (f"crop={crop}," if crop else "") + \
        (f"pad=iw:ih+{pad_top}:0:{pad_top}:white," if pad_top else "") + f"fps={fps},scale={w}:{h}:flags=area"
    raw = subprocess.run(["ffmpeg", "-v", "error", "-ss", str(start)] + (["-t", str(dur)] if dur else []) +
                         ["-i", src, "-vf", vf, "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                         capture_output=True, check=True).stdout
    size = w * h * 3
    shots = [analyse(raw[i:i + size], w, h) for i in range(0, len(raw) - size + 1, size)]
    # Силуэт «провалился» (белая грудка слилась с белым фоном - бывает в
    # прыжке): ещё раз со строгим порогом фона, не помогло - силуэт соседа.
    areas = [w * h - sum(s["bg"]) for s in shots]
    med = sorted(areas)[len(areas) // 2]
    for k, s in enumerate(shots):
        if areas[k] >= 0.9 * med:
            continue
        again = analyse(raw[k * size:(k + 1) * size], w, h, bg_level=245)
        if w * h - sum(again["bg"]) >= 0.9 * med:
            shots[k] = again
        elif k > 0:
            s["bg"] = shots[k - 1]["bg"]
    # «открытый» глаз - крупнее середины между прищуром и открытым в этом ролике
    sizes = sorted(min(s["eyes"][-1][2], s["eyes"][1][2]) for s in shots if len(s["eyes"]) == 2)
    open_px = (sizes[len(sizes) // 5] + sizes[-max(1, len(sizes) // 5)]) / 2 if sizes else 1e9
    blush_px = 10 * UP * UP * (sw / 58) ** 2      # порог подбирался на коте шириной 58
    sway = sway_offsets(shots, w, fps, gain=sway_gain) if sway_gain > 1 else [0.0] * len(shots)
    frames, prev = [], None
    for s, dx in zip(shots, sway):
        cv, prev = draw_frame(s, w, h, prev, open_px, blush_px, STYLES[style])
        frames.append(bytes(shift_frame(cv, dx).buf))
    with open(dst, "wb") as f:
        f.write(oled_video.MAGIC + bytes([fps]) + b"".join(frames))
    return dst, len(frames)


def main():
    ap = argparse.ArgumentParser(description="Кот для дисплея: движение из ролика, черты нарисованы")
    ap.add_argument("src", help="ролик (только читается)")
    ap.add_argument("dst", help="результат, .oledv")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--dur", type=float)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--crop", help="w:h:x:y по исходнику (ffmpeg crop)")
    ap.add_argument("--pad-top", type=int, default=0, help="белое поле над кадром, пикс. исходника")
    ap.add_argument("--style", choices=sorted(STYLES), default="v2b")
    ap.add_argument("--sway", type=float, default=1.0,
                    help="во сколько раз усилить покачивания (1 - как в ролике; 3 пробовали - хуже)")
    a = ap.parse_args()
    dst, n = make(a.src, a.dst, a.start, a.dur, a.fps, a.crop, pad_top=a.pad_top, style=a.style,
                  sway_gain=a.sway)
    print(f"{dst}: {n} кадров, {a.fps} к/с, {n / a.fps:.1f} с")


if __name__ == "__main__":
    main()
