"""Кот «говорит», какую букву прочитал робот: слева кот, справа буква во
всю высоту экрана. У каждой буквы своё настроение кота:

  Φ (раненая)    - грустит: уши прижаты, брови домиком, веки опущены, рот
                   уголками вниз, слёзы ручьями и капают с подбородка; сидит
                   ниже обычного, медленно оседает и всхлипывает;
  Ψ (стабильная) - спокоен: обычная анимация кота, рядом ровно бьётся сердце;
  Ω (целая)      - радуется: прищур ^^, прыгает, вокруг мерцают искры.

Кот - кадры того же ролика, что крутится после загрузки (cat.oledv,
oled_cat.py), сдвинутые влево; сам ролик не меняется, всё грустное
дорисовывается поверх при показе. Черты лица (глаза, рот, макушка) измерены
на кадре 0 этого ролика: пересоберут ролик - перемерить (см. тест
test_moods_fit_cat_frame).

Только рисование по Canvas из oled.py; время t - секунды с появления метки.
"""
from __future__ import annotations

import math
import random

W, H = 128, 64
FPS = 10                       # кадров в секунду, пока показываем метку
LOOP_S = 3.0                   # всё повторяется раз в столько
VIDEO_FPS = 10                 # у cat.oledv
DX = -34                       # кот сдвинут влево: справа место букве
CAT_XMAX = 66                  # правее - буква, усы обрезаются
LETTER_CX, LETTER_W = 98, 4.5

# Кадр 0 ролика, координаты до сдвига
EYE_L, EYE_R = (56.5, 33.5), (75.5, 33.5)
MOUTH = (66, 44)
HEAD_X0, HEAD_X1 = 49, 82      # левый и правый край головы у макушки
SQUINT = range(96, 106)        # прищур ^^ в ролике

_rnd = random.Random(5)
# Искры - только там, где фон: левее кота и между котом и буквой.
SPARKS = [(_rnd.choice([_rnd.randrange(2, 11), _rnd.randrange(56, 66)]),
           _rnd.randrange(3, 40), _rnd.random()) for _ in range(7)]

_lit = {}
_lit_src = None


def lit(frames, i):
    """Горящие пиксели кадра i списком (x, y) - кадр рисуется со сдвигом
    каждый раз, а перебирать все 8192 точки на Pi 5 раз в 100 мс дорого."""
    global _lit_src
    if frames is not _lit_src:
        _lit.clear()
        _lit_src = frames
    i %= len(frames)
    pts = _lit.get(i)
    if pts is None:
        buf = frames[i]
        pts = [(x, (p << 3) + b) for p in range(H // 8) for x in range(W)
               for b in range(8) if buf[p * W + x] >> b & 1]
        _lit[i] = pts
    return pts


def put_cat(cv, frames, i, dy=0):
    for x, y in lit(frames, i):
        x += DX
        y += dy
        if 0 <= x <= CAT_XMAX and 0 <= y < H:
            cv.pixel(x, y)


def stroke(cv, x0, y0, x1, y1, w, on=True):
    n = max(1, int(max(abs(x1 - x0), abs(y1 - y0))))
    for k in range(n + 1):
        cv.disc(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, w / 2, on)


def arc(cv, cx, cy, rx, ry, a0, a1, w, on=True):
    n = max(8, int(abs(a1 - a0) / 360.0 * 2 * math.pi * max(rx, ry)))
    for k in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * k / n)
        cv.disc(cx + rx * math.cos(a), cy + ry * math.sin(a), w / 2, on)


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
            for x in range(int(round(a)), int(round(b))):
                cv.pixel(x, y, on)


# ── Φ: грустный ───────────────────────────────────────────────────
def droop_ears(cv, dy):
    """Стоячие ушки с макушки стёрты, вместо них - прижатые, свисают в стороны."""
    for y in range(13 + dy, 21 + dy):
        cv.fill(44 + DX, y, 46, 1, False)
    cv.fill(50 + DX, 19 + dy, 31, 1)                        # новая макушка
    cv.pixel(HEAD_X0 + DX, 20 + dy)
    cv.pixel(HEAD_X1 - 1 + DX, 20 + dy)
    for s in (1, -1):                                       # 1 - левое ухо
        ax = (HEAD_X0 if s == 1 else HEAD_X1) + DX
        top, bot, tip = (ax + 3 * s, 20 + dy), (ax, 28 + dy), (ax - 10 * s, 31 + dy)
        fill_poly(cv, [(top[0] + s, top[1] - 1), (bot[0] + s, bot[1] + 1),
                       (tip[0] - s, tip[1] + 1)])
        for a, b in ((top, tip), (tip, bot), (bot, top)):
            stroke(cv, a[0] + 0.5 * s, a[1] + 0.5, b[0] + 0.5 * s,
                   b[1] - 0.5 if b is tip else b[1] + 0.5, 1.0, False)
        stroke(cv, ax - s, 24 + dy, tip[0] + 4 * s, tip[1] - 2, 1.0, False)   # складка


def sad_face(cv, dy, blink=False):
    for (cx, cy), s in ((EYE_L, 1), (EYE_R, -1)):          # s: 1 - левый глаз
        cx += DX
        cy += dy
        # веко: выше линии «у носа выше, снаружи ниже» - шерсть
        for y in range(int(cy) - 7, int(cy) + 7):
            for x in range(int(cx) - 6, int(cx) + 7):
                lid = cy + 3 if blink else cy - 1 - (x - cx) * s * 0.45
                if y < lid:
                    cv.pixel(x, y)
        if blink:
            stroke(cv, cx - 6 * s, cy + 3, cx + 6 * s, cy + 3, 1.6, False)
        else:
            stroke(cv, cx - 6 * s, cy + 1.5, cx + 6 * s, cy - 4, 1.6, False)
        stroke(cv, cx - 5 * s, cy - 6, cx + 4 * s, cy - 11, 1.8, False)    # бровь домиком
    mx, my = MOUTH[0] + DX, MOUTH[1] + dy
    cv.fill(int(mx) - 6, int(my) - 2, 13, 7)                # стёрли «ω»
    cv.fill(int(mx) - 1, int(my) - 2, 3, 2, False)          # нос
    arc(cv, mx, my + 5, 4.5, 3.5, 200, 340, 1.4, False)    # рот уголками вниз


def drop(cv, x, y):
    """Капля: светлая в тёмной обводке - видна и на коте, и на фоне."""
    cv.disc(x, y + 2, 3.4, False)
    for yy in range(y - 3, y + 1):
        cv.fill(x - 1, yy, 3, 1, False)
    cv.disc(x, y + 2, 2.2)
    cv.pixel(x, y - 1)
    cv.pixel(x, y)


def stream(cv, x, y0, y1, t, off):
    """Ручей по щеке, по нему бегут блики."""
    for y in range(int(y0), int(y1)):
        cv.fill(x - 2, y, 5, 1, False)
        cv.fill(x - 1, y, 3, 1)
        if (y * 0.5 - t * 12 + off) % 4 < 1:
            cv.pixel(x, y, False)


def sad(cv, frames, t):
    dy = 3 + round(1.5 - 1.5 * math.cos(2 * math.pi * t / LOOP_S))
    ph = t % 1.5
    if ph < 0.3:                                            # всхлип
        dy += (0, -1, 0)[min(2, int(ph / 0.1))]
    put_cat(cv, frames, 0, dy)
    droop_ears(cv, dy)
    sad_face(cv, dy, blink=2.0 <= t % LOOP_S < 2.2)
    for k, (ex, ey) in enumerate((EYE_L, EYE_R)):
        x = int(ex + DX) + (-3 if k == 0 else 3)
        stream(cv, x, ey + dy + 3, ey + dy + 16, t, k * 2)
        for j in range(2):                                  # капли с подбородка
            p = (t * 1.4 + j * 0.5 + k * 0.25) % 1.0
            y = int(ey + dy + 17 + p * 18)
            if y < H - 2:
                drop(cv, x, y)


# ── Ψ: спокойный ──────────────────────────────────────────────────
def heart(cv, x, y, s):
    cv.disc(x - s, y, s)
    cv.disc(x + s, y, s)
    fill_poly(cv, [(x - 2 * s - 0.5, y + 0.5), (x + 2 * s + 0.5, y + 0.5), (x, y + 2.6 * s)])


def calm(cv, frames, t):
    put_cat(cv, frames, int(t * VIDEO_FPS))
    p = t % 1.0                                             # «тук-тук» раз в секунду
    heart(cv, 60, 14, 3.4 if p < 0.1 or 0.25 <= p < 0.35 else 2.4)


# ── Ω: радостный ──────────────────────────────────────────────────
def sparkle(cv, x, y, r):
    cv.fill(x - r, y, 2 * r + 1, 1)
    cv.fill(x, y - r, 1, 2 * r + 1)
    if r > 2:
        for dx, dy in ((-1, -1), (1, 1), (1, -1), (-1, 1)):
            cv.pixel(x + dx, y + dy)


def happy(cv, frames, t):
    p = (t / 0.75) % 1.0                                    # прыжок раз в 0.75 с
    put_cat(cv, frames, SQUINT[int(t * VIDEO_FPS) % len(SQUINT)], -round(8 * math.sin(math.pi * p)))
    for x, y, o in SPARKS:
        b = (t * 1.6 + o) % 1.0
        if b < 0.5:
            r = 1 + round(2 * math.sin(math.pi * b / 0.5))
            if not any(cv.get(xx, yy) for xx in range(x - r, x + r + 1)
                       for yy in range(y - r, y + r + 1) if 0 <= xx < W and 0 <= yy < H):
                sparkle(cv, x, y, r)


MOODS = {"Φ": sad, "Ψ": calm, "Ω": happy}


def draw(cv, ch, frames, t):
    """Кот с настроением для буквы ch (без самой буквы). -> False, если для
    этой буквы кота нет."""
    f = MOODS.get(ch)
    if f is None or not frames:
        return False
    f(cv, frames, t % LOOP_S if f is not calm else t)
    return True
