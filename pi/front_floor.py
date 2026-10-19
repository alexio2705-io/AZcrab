"""Цвет следующей клетки по передней USB-камере.

Камера стоит в носу боком и низко, у нижнего края кадра - бампер. prep()
поворачивает кадр (верх - вперёд) и отрезает бампер; дальше все пиксели -
уже этого кадра.

Пол - плоскость, поэтому пиксель кадра переводится в точку пола одной
матрицей перспективы H (пиксель -> мм в системе робота: x вперёд от центра
робота, y влево). H снимается один раз: плитка известного размера кладётся
на известное место перед роботом (обычно - чёрная или синяя плитка в
соседней клетке, робот в центре своей), calibrate_h() находит её углы на
кадре. Камера сдвинулась - снимать заново.

Зная H и где робот в своей клетке (смещение центра и поворот относительно
сетки), cell_polygon() даёт на кадре четырёхугольник следующей клетки - в
любом положении робота, не только из центра. Внутри него каждый пиксель
относится к ближайшему образцу цвета (как у датчика в tile_classify.py:
доли каналов и логарифм яркости), клетка - цвет большинства. Образцы
снимаются с этой же камеры на поле (sample_cell()): у неё свой баланс
белого, образцы датчика ей не подходят.

Пока калибровки по плитке нет, H берётся из модели камеры (DEFAULT_CAM):
высота, наклон и фокус оценены 18.10.2026 по стене перед роботом - стык
стены со столом на кадре против расстояния до неё по лидару, а точка схода
стыка и верхнего края стены дала наклон и фокус. Пока нет образцов цвета,
пиксели размечаются порогами HSV (hsv_label).

Решение есть всегда (read_cell никогда не говорит «не знаю»): цвет
большинства пикселей и уверенность conf 0..1. Каждый пиксель тоже всегда
получает цвет - всё, что не чёрное, не синее и не красное, считается белым
полом. Когда кадру верить нельзя, вывод делается по гироскопу и тому, что
известно о поле:
  - робот наклонён на горке (|tilt| >= RAMP_TILT_DEG) - пол впереди не та
    плоскость, на которую посчитана H; на горке особых плиток не бывает,
    так что впереди обычная клетка (белая, conf низкий);
  - меньший наклон (нос задрало на неровности) - H поворачивается на него
    (pitched_h), чтение идёт как обычно;
  - клетка почти вся вне кадра (робот сильно повёрнут) - белая с conf 0:
    датчик цвета всё равно проверит её под собой.
"""
from __future__ import annotations

import json
import math
import os
import time

import cv2
import numpy as np

CALIB_PATH = os.path.expanduser("~/maze/front_floor.json")
CELL_MM = 300.0

ROTATE = cv2.ROTATE_90_COUNTERCLOCKWISE   # камера стоит боком: бампер был слева
CROP_BOTTOM = 0.24                        # нижние 24% повёрнутого кадра - бампер
CAM_W = 720                               # ширина кадра после prep()

COLORS = ["white", "black", "blue", "red", "silver"]
NAMES = {"white": "белая", "black": "чёрная", "blue": "синяя", "red": "красная",
         "silver": "зеркальная"}

# Признаки и масштабы - как в tile_classify.py (доли каналов, ln яркости).
FRAC_SCALE = 0.03
LN_SCALE = 0.15
MARGIN_MM = 40.0        # клетку читаем с отступом от краёв: стыки, тени стен, ошибка позы
MAX_PX = 4000           # больше пикселей не берём - хватает и для голосования
MIN_PX = 150            # меньше видимых пикселей - кадру не верим (conf 0)
# Ближнюю половину следующей клетки из центра своей камера не видит (бампер
# обрезан): видно примерно от 200 до 410 мм, это 0.49 клетки с отступами.
MIN_VISIBLE = 0.3       # меньше этой доли клетки в кадре - кадру не верим
FULL_VISIBLE = 0.5      # с этой доли видимости кадру веришь полностью
RAMP_TILT_DEG = 10.0    # наклон не меньше - робот на горке (горки 25-30°)
RAMP_CONF = 0.3


# Модель камеры в повёрнутом кадре 720x973: высота объектива над полом,
# наклон вниз, фокус (px), объектив впереди центра робота, главная точка.
DEFAULT_CAM = {"h_mm": 68.0, "pitch_deg": 14.7, "f_px": 982.0, "x_mm": 95.0,
               "cx": 360.0, "cy": 640.0}

# Пороги HSV (OpenCV: H 0..180), пока нет образцов. Замер 18.10.2026: синий
# лист H101 S185 V243, дерево стола H19 S102 V188 (по порогам - белое).
HSV_BLUE = ((90, 100, 60), (130, 255, 255))
HSV_RED_S, HSV_RED_V = 110, 60            # и H <= 8 или H >= 170
HSV_BLACK_V = 70                          # V не выше - чёрная
# Зеркальную порогами не различить (в ней отражаются стены и лампы) -
# только по образцам, иначе решает датчик цвета.


def model_point(x_mm, y_mm, cam=None):
    """Точка пола (мм, система робота) -> пиксель по модели камеры."""
    c = cam or DEFAULT_CAM
    th = math.radians(c["pitch_deg"])
    x = x_mm - c["x_mm"]
    z = x * math.cos(th) + c["h_mm"] * math.sin(th)
    yi = -x * math.sin(th) + c["h_mm"] * math.cos(th)
    return c["cx"] - c["f_px"] * y_mm / z, c["cy"] + c["f_px"] * yi / z


def model_h(cam=None):
    """H (пиксель -> пол) по модели камеры."""
    floor = [(450.0, 150.0), (450.0, -150.0), (250.0, -150.0), (250.0, 150.0)]
    img = np.float32([model_point(x, y, cam) for x, y in floor])
    return cv2.getPerspectiveTransform(img, np.float32(floor))


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def pitched_h(h_img2floor, nose_up_deg, cam=None):
    """H для робота, у которого нос задран на nose_up_deg (вниз - минус):
    камера поворачивается вместе с корпусом, кадр - на поворот вокруг её
    горизонтальной оси. Высота объектива при этом почти не меняется."""
    if not nose_up_deg:
        return np.asarray(h_img2floor, float)
    c = cam or DEFAULT_CAM
    k = np.array([[c["f_px"], 0.0, c["cx"]], [0.0, c["f_px"], c["cy"]], [0.0, 0.0, 1.0]])
    # нос вверх - картинка уходит вниз (y кадра растёт)
    m = k @ _rot_x(-math.radians(nose_up_deg)) @ np.linalg.inv(k)  # старый пиксель -> новый
    return np.asarray(h_img2floor, float) @ np.linalg.inv(m)


def prep(frame):
    """Кадр с камеры -> повёрнутый, без бампера."""
    r = cv2.rotate(frame, ROTATE)
    return r[:int(round(r.shape[0] * (1.0 - CROP_BOTTOM)))]


# ── Калибровка плоскости пола ──────────────────────────────────────

def find_quad(frame, color):
    """Углы калибровочной плитки на кадре: [дальний левый, дальний правый,
    ближний правый, ближний левый] (x, y) или (None, причина)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if color == "blue":
        mask = cv2.inRange(hsv, (90, 80, 40), (130, 255, 255))
    elif color == "black":
        mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 70))
    else:
        return None, f"калибровка только по чёрной или синей плитке, не {color}"
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, "плитка не найдена"
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 0.01 * frame.shape[0] * frame.shape[1]:
        return None, "плитка не найдена (слишком мелкая)"
    hull = cv2.convexHull(c)
    approx = None
    for eps in (0.01, 0.02, 0.03, 0.05, 0.08):
        a = cv2.approxPolyDP(hull, eps * cv2.arcLength(hull, True), True)
        if len(a) == 4:
            approx = a.reshape(4, 2).astype(float)
            break
    if approx is None:
        return None, "у плитки не 4 угла - заслонена или неровно лежит"
    h, w = frame.shape[:2]
    if any(x < 3 or y < 3 or x > w - 4 or y > h - 4 for x, y in approx):
        return None, "плитка касается края кадра - отодвиньте её дальше и укажите, где она"
    by_y = approx[np.argsort(approx[:, 1])]
    far = by_y[:2][np.argsort(by_y[:2, 0])]
    near = by_y[2:][np.argsort(by_y[2:, 0])]
    return [far[0].tolist(), far[1].tolist(), near[1].tolist(), near[0].tolist()], None


def calibrate_h(quad, near_mm, depth_mm=CELL_MM, width_mm=CELL_MM, left_mm=0.0):
    """H по углам плитки. near_mm - от центра робота до ближнего края,
    left_mm - насколько середина плитки левее оси робота."""
    far = near_mm + depth_mm
    yl, yr = left_mm + width_mm / 2.0, left_mm - width_mm / 2.0
    floor = np.float32([[far, yl], [far, yr], [near_mm, yr], [near_mm, yl]])
    return cv2.getPerspectiveTransform(np.float32(quad), floor)


def to_image(h_img2floor, pts_mm):
    """Точки пола (мм, система робота) -> пиксели кадра."""
    inv = np.linalg.inv(np.asarray(h_img2floor, float))
    p = cv2.perspectiveTransform(np.float32(pts_mm).reshape(-1, 1, 2), inv)
    return p.reshape(-1, 2)


def cell_polygon(pose=(0.0, 0.0, 0.0), ahead=1, margin=MARGIN_MM):
    """Клетка через ahead вперёд по сетке, углы в мм в системе робота.
    pose - (ox, oy, dpsi): центр робота от центра его клетки по сетке (мм,
    x вперёд, y влево) и поворот робота от направления сетки (рад, влево +)."""
    ox, oy, dpsi = pose
    x0, x1 = CELL_MM * (ahead - 0.5) + margin, CELL_MM * (ahead + 0.5) - margin
    y0 = CELL_MM / 2.0 - margin
    cs, sn = math.cos(dpsi), math.sin(dpsi)
    out = []
    for cx, cy in ((x1, y0), (x1, -y0), (x0, -y0), (x0, y0)):
        dx, dy = cx - ox, cy - oy
        out.append((cs * dx + sn * dy, -sn * dx + cs * dy))
    return out


# ── Цвет ──────────────────────────────────────────────────────────

def features(bgr):
    """Пиксели BGR (N, 3) -> признаки (N, 4): доли r, g, b и ln яркости."""
    px = np.asarray(bgr, np.float32).reshape(-1, 3) + 1.0
    s = px.sum(axis=1, keepdims=True)
    return np.hstack([px[:, ::-1] / s, np.log(s)])


def _scaled(f):
    return np.asarray(f, np.float32) / np.float32([FRAC_SCALE] * 3 + [LN_SCALE])


def cell_pixels(frame, h_img2floor, pose=(0.0, 0.0, 0.0), ahead=1, margin=MARGIN_MM):
    """Пиксели клетки (N, 3), четырёхугольник на кадре и видимая доля клетки."""
    poly = to_image(h_img2floor, cell_polygon(pose, ahead, margin))
    h, w = frame.shape[:2]
    full = np.zeros((h, w), np.uint8)
    cv2.fillPoly(full, [np.round(poly).astype(np.int32)], 1)
    n_in = int(full.sum())
    area = abs(cv2.contourArea(poly.astype(np.float32)))
    visible = n_in / area if area > 1 else 0.0
    ys, xs = np.nonzero(full)
    if len(ys) > MAX_PX:
        idx = np.linspace(0, len(ys) - 1, MAX_PX).astype(int)
        ys, xs = ys[idx], xs[idx]
    return frame[ys, xs], poly, min(1.0, visible)


def hsv_label(bgr):
    """Цвет пикселей по порогам HSV (индекс в COLORS). Не чёрное, не синее
    и не красное - белый пол (дерево, серое, пересвет - тоже он)."""
    hsv = cv2.cvtColor(np.asarray(bgr, np.uint8).reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)
    hsv = hsv.reshape(-1, 3).astype(int)
    hh, ss, vv = hsv[:, 0], hsv[:, 1], hsv[:, 2]
    out = np.full(len(hsv), COLORS.index("white"))

    def within(rng):
        (h0, s0, v0), (h1, s1, v1) = rng
        return (hh >= h0) & (hh <= h1) & (ss >= s0) & (ss <= s1) & (vv >= v0) & (vv <= v1)
    out[within(HSV_BLUE)] = COLORS.index("blue")
    out[((hh <= 8) | (hh >= 170)) & (ss >= HSV_RED_S) & (vv >= HSV_RED_V)] = COLORS.index("red")
    out[vv <= HSV_BLACK_V] = COLORS.index("black")
    return out


def classify_pixels(samples, feats):
    """Цвет каждого пикселя по ближайшему образцу (индекс в COLORS)."""
    keys, rows = [], []
    for color in COLORS:
        for s in samples.get(color, []):
            keys.append(COLORS.index(color))
            rows.append(s["f"])
    d = np.linalg.norm(_scaled(feats)[:, None, :] - _scaled(rows)[None, :, :], axis=2)
    return np.asarray(keys)[d.argmin(axis=1)]


def read_cell(frame, cal, pose=(0.0, 0.0, 0.0), ahead=1, tilt_deg=0.0):
    """Цвет клетки - всегда. tilt_deg - наклон робота по гироскопу (нос
    вверх +) от ровного пола. -> {'color', 'conf' 0..1, 'frac', 'visible',
    'n', 'votes', 'poly', 'why'}: why - почему conf не полный."""
    res = {"color": "white", "conf": 0.0, "frac": 0.0, "visible": 0.0, "n": 0, "votes": {},
           "poly": None, "why": ""}
    if abs(tilt_deg) >= RAMP_TILT_DEG:
        res.update(conf=RAMP_CONF, why=f"на горке ({tilt_deg:+.0f}°): впереди обычная клетка")
        return res
    h = (cal or {}).get("H")
    h = pitched_h(model_h() if h is None else h, tilt_deg)
    px, poly, visible = cell_pixels(frame, h, pose, ahead)
    res.update(poly=poly.tolist(), visible=round(visible, 2), n=len(px))
    if visible < MIN_VISIBLE or len(px) < MIN_PX:
        res["why"] = "клетка почти вне кадра: считаю обычной, проверит датчик"
        return res
    samples = (cal or {}).get("samples")
    lab = classify_pixels(samples, features(px)) if samples else hsv_label(px)
    votes = {COLORS[i]: int((lab == i).sum()) for i in range(len(COLORS)) if (lab == i).any()}
    color, k = max(votes.items(), key=lambda kv: kv[1])
    frac = k / len(px)
    conf = frac * min(1.0, visible / FULL_VISIBLE)
    res.update(color=color, votes=votes, frac=round(frac, 2), conf=round(conf, 2))
    if visible < FULL_VISIBLE:
        res["why"] = f"видно {visible:.0%} клетки"
    return res


def sample_cell(frame, cal, pose=(0.0, 0.0, 0.0), ahead=1):
    """Образец цвета с клетки: медиана признаков её пикселей, или None."""
    h = cal.get("H")
    px, _, visible = cell_pixels(frame, model_h() if h is None else h, pose, ahead)
    if visible < MIN_VISIBLE or len(px) < MIN_PX:
        return None
    return {"f": np.median(features(px), axis=0).round(4).tolist(), "n": len(px),
            "ts": time.strftime("%Y-%m-%d %H:%M:%S")}


# ── Файл калибровки ───────────────────────────────────────────────

def load(path=None):
    """{'H': 3x3 | None, 'samples': {цвет: [...]}, ...}; нет файла - пусто."""
    try:
        with open(path or CALIB_PATH) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {"H": None, "samples": {}}
    data.setdefault("H", None)
    data["samples"] = {k: v for k, v in (data.get("samples") or {}).items()
                       if k in COLORS and v}
    return data


def save(cal, path=None):
    """Атомарно, как tile_classify.save: nav.py может читать файл в этот момент."""
    path = path or CALIB_PATH
    os.makedirs(os.path.dirname(path), exist_ok=True)
    out = dict(cal, saved=time.strftime("%Y-%m-%d %H:%M:%S"))
    if out.get("H") is not None:
        out["H"] = np.asarray(out["H"], float).tolist()
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)
