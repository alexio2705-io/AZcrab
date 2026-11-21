#!/usr/bin/env python3
"""Буквы H / S / U на стене без нейросети - запасной вариант для технического
испытания, пока YOLO (maze_v26n) знает только Φ/Ψ/Ω и когнитивные цели.

Идея - топология штрихов, а не шаблон: она не зависит от шрифта, толщины
линий и масштаба.
  * центральная ВЕРТИКАЛЬ буквы пересекает у S три штриха (верхняя дуга,
    середина, нижняя дуга), у H - один короткий (перемычка) посередине,
    у U - один короткий внизу;
  * верхняя горизонталь у H и U - два штриха (две ножки).
Поворот на 90/180/270 перебирается: у повёрнутой H центральная вертикаль
идёт вдоль перемычки - один ДЛИННЫЙ штрих, это отсекается порогом длины.
Φ и Ψ (вертикальный стержень во всю высоту) и Ω (штрих только сверху) по
этим правилам не проходят - ложной H/S/U на старых метках не будет.

Ограничения: буква должна быть одна в кадре и ощутимо крупной (соседняя
стена, а не дальняя), сильная перспектива (камера под углом к стене) не
выправляется.

Проверка на кадре с камеры:
    ~/maze/venv/bin/python ~/maze/letters_cv.py снимок.jpg
"""

from __future__ import annotations

import sys

import cv2
import numpy as np

# Кандидат в буквы: доля площади кадра, которую занимает рамка контура.
# Буква 4см на соседней стене - крупная; мелочь - дальняя стена или мусор.
MIN_BOX_FRAC = 0.01
MAX_BOX_FRAC = 0.6
# Отношение сторон рамки (ширина/высота) - буква примерно квадратная.
MIN_ASPECT, MAX_ASPECT = 0.35, 2.8
# Доля тёмного в рамке: у буквы штрихи, а не сплошное пятно (чёрная плитка,
# тень в углу) и не пара точек.
MIN_FILL, MAX_FILL = 0.15, 0.75
NORM = 64                 # сторона нормализованной буквы, пикселей
SHORT_RUN = 0.40          # "короткий" штрих - меньше этой доли высоты
# Сколько соседних линий сканирования голосует (по обе стороны от основной):
# одна линия может попасть ровно в стык или в дырку от шума.
SCAN_SPREAD = (-0.06, -0.03, 0.0, 0.03, 0.06)
MAX_S_MID = 0.85          # перемычка S по середине - не во всю ширину
MIN_S_SYM = 0.42          # IoU буквы с собой же, повёрнутой на 180°
MAX_LEG = 0.35            # ножка H/U - узкий штрих
LEG_DRIFT = 0.10          # насколько может гулять край прямой ножки
MIN_HOLE_FRAC = 0.01      # полость меньше этой доли рамки - шум, не дырка
MAX_DESKEW_DEG = 20       # больший наклон не выпрямляем - это уже не "криво"
# Где проверять ножки: у H - выше и ниже перемычки, у U - выше скругления.
LEG_ROWS = {"H": (0.12, 0.3, 0.7, 0.88), "U": (0.1, 0.3, 0.5)}


def _runs(line):
    """Отрезки подряд идущих единиц в 0/1-массиве: [(начало, длина), ...]."""
    d = np.diff(np.concatenate(([0], line.astype(np.int8), [0])))
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1)
    # Шум в 1-2 пикселя - не штрих.
    return [(s, e - s) for s, e in zip(starts, ends) if e - s >= 2]


def _vote(fn):
    """Значение fn(сдвиг) по большинству линий SCAN_SPREAD (None - нет)."""
    vals = [fn(off) for off in SCAN_SPREAD]
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    best = max(set(vals), key=vals.count)
    return best if vals.count(best) * 2 > len(SCAN_SPREAD) else None


def classify_glyph(g):
    """g - квадрат NORM x NORM, 1 = чернила, буква вписана вплотную.
    -> 'H' / 'S' / 'U' / None, только для ровно стоящей буквы."""
    n = NORM

    def vertical(off):
        x = int(round(n * (0.5 + off)))
        runs = _runs(g[:, x])
        if len(runs) == 3 and all(ln < SHORT_RUN * n for _, ln in runs):
            return "S"
        if len(runs) == 1:
            start, ln = runs[0]
            if ln >= SHORT_RUN * n:
                return None
            mid = (start + ln / 2) / n
            if 0.3 < mid < 0.7:
                return "H"
            if mid >= 0.7:
                return "U"
        return None

    letter = _vote(vertical)
    if letter == "S":
        # Середина S - одна короткая косая перемычка. У повёрнутых Φ/Ψ
        # центральная вертикаль тоже даёт три штриха, но горизонталь по
        # середине идёт вдоль стержня - один штрих во всю ширину.
        mid = _runs(g[n // 2, :])
        if len(mid) != 1 or mid[0][1] > MAX_S_MID * n:
            return None
        # S переходит сама в себя при повороте на 180°, повёрнутая Ψ - нет
        # (стержень с одной стороны, рога с другой).
        rot = np.rot90(g, 2)
        if (g & rot).sum() < MIN_S_SYM * (g | rot).sum():
            return None
    elif letter in ("H", "U") and not _straight_legs(g, LEG_ROWS[letter]):
        return None
    return letter


def _straight_legs(g, rows):
    """Две прямые вертикальные ножки на высотах rows (доли высоты): на каждой
    ровно два узких штриха, и их края не гуляют. Отсекает перевёрнутую Ω
    (талия уже лапок) и прочие дуги, похожие на U по центральной линии."""
    n = NORM
    lefts, rights = [], []
    for fy in rows:
        runs = _runs(g[int(fy * n), :])
        if len(runs) != 2 or any(ln > MAX_LEG * n for _, ln in runs):
            return False
        lefts.append(runs[0][0])
        rights.append(runs[1][0] + runs[1][1])
    return (max(lefts) - min(lefts) <= LEG_DRIFT * n
            and max(rights) - min(rights) <= LEG_DRIFT * n)


def _normalize(mask):
    """Обрезка по чернилам и вписывание в NORM x NORM с сохранением формы:
    растягивать узкую букву до квадрата нельзя, поэтому дополняем полями."""
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    crop = mask[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    h, w = crop.shape
    side = max(h, w)
    sq = np.zeros((side, side), np.uint8)
    y0, x0 = (side - h) // 2, (side - w) // 2
    sq[y0:y0 + h, x0:x0 + w] = crop
    g = cv2.resize(sq * 255, (NORM, NORM), interpolation=cv2.INTER_AREA)
    g = (g > 127).astype(np.uint8)
    # Обрезка ещё раз уже по нормализованной - чтобы линии сканирования
    # считались от краёв самой буквы, а не от полей квадрата.
    ys, xs = np.nonzero(g)
    if len(xs) == 0:
        return None
    g = g[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    return (cv2.resize(g * 255, (NORM, NORM), interpolation=cv2.INTER_NEAREST) > 127
            ).astype(np.uint8)


def _deskew(mask):
    """Небольшой наклон буквы (наклеена криво, робот стоит не ровно) -
    выпрямляем по описанному прямоугольнику чернил. Правила ножек и середины
    рассчитаны на ровную букву: на 6° ножка H уже "гуляет" больше допуска."""
    pts = cv2.findNonZero(mask)
    if pts is None:
        return mask
    angle = cv2.minAreaRect(pts)[2]
    angle = (angle + 45) % 90 - 45          # к ближайшей оси: -45..45
    if abs(angle) < 1 or abs(angle) > MAX_DESKEW_DEG:
        return mask
    h, w = mask.shape
    side = int(np.hypot(h, w)) + 2
    canvas = np.zeros((side, side), np.uint8)
    y0, x0 = (side - h) // 2, (side - w) // 2
    canvas[y0:y0 + h, x0:x0 + w] = mask
    m = cv2.getRotationMatrix2D((side / 2, side / 2), angle, 1.0)
    return (cv2.warpAffine(canvas * 255, m, (side, side)) > 127).astype(np.uint8)


def _has_hole(mask):
    """Замкнутая полость внутри - у H, S и U их нет, у Φ (и O, A, ...) есть.
    Мелкие дырки - шум бинаризации, их не считаем."""
    contours, hier = cv2.findContours(mask.copy(), cv2.RETR_CCOMP,
                                      cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return False
    min_area = MIN_HOLE_FRAC * mask.size
    return any(h[3] >= 0 and cv2.contourArea(c) > min_area
               for c, h in zip(contours, hier[0]))


def classify_mask(mask):
    """Бинарная маска одной буквы (1 = чернила) в любом из 4 поворотов ->
    'H' / 'S' / 'U' / None. Повороты перебираются; если разные повороты
    дают разные буквы - не уверены, None."""
    if _has_hole(mask):
        return None
    mask = _deskew(mask)
    found = set()
    for k in range(4):
        g = _normalize(np.rot90(mask, k))
        if g is None:
            return None
        letter = classify_glyph(g)
        if letter:
            found.add(letter)
    return found.pop() if len(found) == 1 else None


def find_letter(frame):
    """BGR-кадр камеры -> (буква, (x, y, w, h)) или None.

    Тёмное отделяется от стены по Оцу; из контуров берутся похожие на букву
    по размеру, форме и заполненности, и не касающиеся края кадра (край -
    это обычно угол, пол или соседняя стена). Если подходит несколько -
    побеждает самый крупный."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, ink = cv2.threshold(gray, 0, 1, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    ink = cv2.morphologyEx(ink, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    fh, fw = ink.shape
    contours, _ = cv2.findContours(ink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if x <= 1 or y <= 1 or x + w >= fw - 1 or y + h >= fh - 1:
            continue
        frac = w * h / (fw * fh)
        if not MIN_BOX_FRAC <= frac <= MAX_BOX_FRAC:
            continue
        if not MIN_ASPECT <= w / h <= MAX_ASPECT:
            continue
        mask = np.zeros((h, w), np.uint8)
        cv2.drawContours(mask, [c - [x, y]], -1, 1, thickness=cv2.FILLED)
        # Дырки внутри контура (S, U их не имеют, но O/Φ имеют) - берём
        # реальные чернила, а не залитый контур.
        mask &= ink[y:y + h, x:x + w]
        fill = mask.mean()
        if not MIN_FILL <= fill <= MAX_FILL:
            continue
        letter = classify_mask(mask)
        if letter and (best is None or w * h > best[1][2] * best[1][3]):
            best = (letter, (x, y, w, h))
    return best


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for path in sys.argv[1:]:
        frame = cv2.imread(path)
        if frame is None:
            print(f"{path}: не читается")
            continue
        r = find_letter(frame)
        print(f"{path}: {r[0] if r else '—'}" + (f"  рамка={r[1]}" if r else ""))


if __name__ == "__main__":
    main()
