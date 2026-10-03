"""Чтение цветов колец когнитивной цели.

Общий модуль для victim_web.py и victim_detect.py. Пороги лежат в
~/maze/ring_colors.json и настраиваются со страницы /tune.

Как читаем:
0. Баланс белого по бумаге. Бумага вокруг цели заведомо белая: берём её цвет
   (самые светлые во всех трёх каналах пиксели рядом с целью) и выравниваем
   каналы так, чтобы она стала серой. Иначе цветная подсветка/неподходящий
   AWB камеры сдвигают оттенки (жёлтый уезжал в зелёный) и бумага выглядит
   «цветной» — край цели ищется по краю листа.
1. Контур цели. Из центра пускаем лучи и на каждом ищем первый выход с
   «чернил» на бумагу — это внешний край. Лучи, упёршиеся в край кадра
   (цель видна не целиком), выбрасываем. По точкам края подгоняем эллипс
   (камера смотрит под углом), отбрасываем выбросы, уточняем в два прохода:
   второй раз лучи идут уже из центра найденного эллипса — рамка YOLO у
   обрезанной цели смещена.
2. Каждое из 5 колец (равные полосы по радиусу) опрашиваем в N точках по
   окружности, цвет — большинством голосов. Точки вне кадра не считаются.
3. Чёрный отличаем от тёмно-синего по яркости относительно бумаги.
4. Кольцо «надёжно», если в кадре хотя бы половина его точек и согласие не
   ниже min_agree. Если хоть одно кольцо ненадёжно — чексуммы нет («не
   прочитано»), а не случайный «фейк».
"""
import json
import os
from collections import Counter

import cv2
import numpy as np

CONFIG_PATH = os.path.expanduser('~/maze/ring_colors.json')
COLORS = ['black', 'red', 'yellow', 'green', 'blue']
COLOR_SCORE = {'black': -2, 'red': -1, 'yellow': 0, 'green': 1, 'blue': 2}
DRAW_BGR = {'black': (0, 0, 0), 'red': (0, 0, 255), 'yellow': (0, 255, 255),
            'green': (0, 200, 0), 'blue': (255, 0, 0), '?': (160, 160, 160)}

DEFAULTS = {
    # чёрный: V / V_бумаги ниже этого (синий по замерам 0.33–0.44, чёрный 0.08–0.22)
    'black_v_ratio': 0.27,
    # насыщенность меньше (S_бумаги + это) — белый/серый, цвет не определяем -> '?'
    'min_sat': 50,
    # диапазоны оттенка OpenCV (0..180); у красного два куска вокруг 0
    'red_h1': 10, 'red_h2': 160,
    'yellow_lo': 15, 'yellow_hi': 38,
    'green_lo': 40, 'green_hi': 88,
    'blue_lo': 90, 'blue_hi': 140,
    # «чернила» при поиске края цели: S выше S_бумаги на ink_sat или темнее
    # ink_v_ratio от бумаги; остальное — бумага.
    'ink_sat': 60, 'ink_v_ratio': 0.6,
    # тёмный пиксель с зелёным оттенком — это тёмно-зелёный, а не чёрный
    'dark_green': 1,
    # ...но только если не совсем чёрный: V / V_бумаги выше этого
    'dark_green_min': 0.15,
    # найденный эллипс меньше этой доли рамки YOLO — не верим, берём круг по рамке
    'min_fit_ratio': 0.6,
    # точек на кольцо
    'samples': 24,
    # 1 — выравнивать цвета по белой бумаге (см. шапку)
    'paper_wb': 1,
    # кольцо надёжно при согласии точек не ниже этого
    'min_agree': 0.6,
    # целиком видимая цель: центр эллипса дальше этой доли размера рамки от её
    # центра — эллипс ложный, берём вписанный в рамку
    'box_center_tol': 0.12,
    # 1 — центр цели по внутреннему диску (против перспективы при съёмке под углом)
    'find_center': 1,
}


def load_config():
    cfg = dict(DEFAULTS)
    try:
        with open(CONFIG_PATH) as f:
            cfg.update({k: v for k, v in json.load(f).items() if k in DEFAULTS})
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f'Не прочитал {CONFIG_PATH}: {e}', flush=True)
    return cfg


def save_config(cfg):
    with open(CONFIG_PATH, 'w') as f:
        json.dump({k: cfg[k] for k in DEFAULTS}, f, indent=2)


def classify(h, s, v_ratio, cfg, paper_s=0):
    colorful = s >= paper_s + cfg['min_sat']
    if v_ratio < cfg['black_v_ratio']:
        if (cfg['dark_green'] and colorful and v_ratio >= cfg['dark_green_min']
                and cfg['green_lo'] <= h <= cfg['green_hi']):
            return 'green'
        return 'black'
    if not colorful:
        return '?'
    if h <= cfg['red_h1'] or h >= cfg['red_h2']:
        return 'red'
    for c in ('yellow', 'green', 'blue'):
        if cfg[c + '_lo'] <= h <= cfg[c + '_hi']:
            return c
    return '?'


CODE = {'?': 0, 'black': 1, 'red': 2, 'yellow': 3, 'green': 4, 'blue': 5}


def classify_map(hsv, cfg, paper, paper_s):
    """classify() для целого массива HSV — коды цветов (CODE)."""
    h = hsv[..., 0].astype(np.int16)
    s_ = hsv[..., 1].astype(np.int16)
    vr = hsv[..., 2].astype(np.float32) / paper
    colorful = s_ >= paper_s + cfg['min_sat']
    out = np.zeros(h.shape, np.uint8)
    for c in ('yellow', 'green', 'blue'):
        out[colorful & (h >= cfg[c + '_lo']) & (h <= cfg[c + '_hi'])] = CODE[c]
    out[colorful & ((h <= cfg['red_h1']) | (h >= cfg['red_h2']))] = CODE['red']
    dark = vr < cfg['black_v_ratio']
    dg = (dark & colorful & (vr >= cfg['dark_green_min'])
          & (h >= cfg['green_lo']) & (h <= cfg['green_hi'])) if cfg['dark_green'] else 0
    out[dark] = CODE['black']
    if cfg['dark_green']:
        out[dg] = CODE['green']
    return out


def find_center(ctx, ell, cfg):
    """Проекция центра цели — центр самого внутреннего сплошного пятна.

    Под углом кольца на кадре не концентричны: центральный круг смещён к
    ближнему краю относительно центра внешнего эллипса. Ищем внутри эллипса
    одноцветное пятно-диск (центр масс лежит в самом пятне — у кольца он в
    дырке), площадью 1..40% эллипса и не у края; берём самое маленькое.
    None — не нашли.
    """
    (cx, cy), (a, b), ang = ell
    H, W = ctx['wb'].shape[:2]
    r = max(a, b)
    X1, Y1 = max(0, int(cx - r)), max(0, int(cy - r))
    X2, Y2 = min(W, int(cx + r) + 1), min(H, int(cy + r) + 1)
    if X2 - X1 < 8 or Y2 - Y1 < 8:
        return None
    hsv = cv2.cvtColor(ctx['wb'][Y1:Y2, X1:X2], cv2.COLOR_BGR2HSV)
    codes = classify_map(hsv, cfg, ctx['paper'], ctx['paper_s'])
    inside = np.zeros(codes.shape, np.uint8)
    cv2.ellipse(inside, (int(cx - X1), int(cy - Y1)), (int(a * 0.9), int(b * 0.9)), ang, 0, 360, 1, -1)
    area_ell = np.pi * a * b
    best = None
    for code in range(1, 6):
        mask = ((codes == code) & (inside > 0)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for j in range(1, n):
            area = stats[j, cv2.CC_STAT_AREA]
            if not 0.01 * area_ell <= area <= 0.4 * area_ell:
                continue
            mx, my = cents[j]
            if lab[int(round(my)), int(round(mx))] != j:      # кольцо, а не диск
                continue
            if np.hypot(mx + X1 - cx, my + Y1 - cy) > 0.6 * r:
                continue
            if best is None or area < best[0]:
                best = (area, mx + X1, my + Y1)
    return None if best is None else (best[1], best[2])


def _ray_to_ellipse(ell, px, py, th):
    """Расстояние от точки (px, py) внутри эллипса до его края по направлению th."""
    (cx, cy), (a, b), ang = ell
    t = np.deg2rad(ang)
    ct, st = np.cos(t), np.sin(t)
    # в систему осей эллипса
    x0 = ((px - cx) * ct + (py - cy) * st) / a
    y0 = (-(px - cx) * st + (py - cy) * ct) / b
    dx = (np.cos(th) * ct + np.sin(th) * st) / a
    dy = (-np.cos(th) * st + np.sin(th) * ct) / b
    A, B, C = dx * dx + dy * dy, 2 * (x0 * dx + y0 * dy), x0 * x0 + y0 * y0 - 1
    disc = B * B - 4 * A * C
    return (-B + np.sqrt(max(disc, 0))) / (2 * A)


def _roi(frame, xyxy, pad):
    x1, y1, x2, y2 = xyxy
    w, h = x2 - x1, y2 - y1
    return (max(0, int(x1 - w * pad)), max(0, int(y1 - h * pad)),
            min(frame.shape[1], int(x2 + w * pad)), min(frame.shape[0], int(y2 + h * pad)))


def prepare(frame, xyxy, cfg):
    """Баланс белого по бумаге + яркость/насыщенность бумаги.

    Бумага — пиксели рядом с целью, светлые во ВСЕХ каналах (максимум
    min(B,G,R)): жёлтое кольцо светлое, но синего в нём мало. В лабиринте
    стены белые, как и лист, так что «самое белое вокруг» — это бумага/стена.
    Возвращает ctx: wb (кадр после баланса), gains, paper (V), paper_s (S).
    """
    X1, Y1, X2, Y2 = _roi(frame, xyxy, 0.4)
    roi = frame[Y1:Y2, X1:X2].reshape(-1, 3).astype(np.float32)
    mn = roi.min(axis=1)
    ok = roi.max(axis=1) < 250                      # без пересвеченных бликов
    use = ok if ok.sum() > 50 else np.ones(len(roi), bool)
    thr = np.percentile(mn[use], 92)
    sel = (mn >= thr) & use
    ref = np.median(roi[sel], axis=0) if sel.sum() else roi.mean(axis=0)
    if cfg.get('paper_wb', 1):
        gains = np.clip(ref.mean() / np.maximum(ref, 1), 0.4, 2.5)
        wb = np.clip(frame.astype(np.float32) * gains, 0, 255).astype(np.uint8)
    else:
        gains = np.ones(3, np.float32)
        wb = frame
    hsv_ref = cv2.cvtColor(np.clip(roi[sel] * gains, 0, 255).astype(np.uint8)[None],
                           cv2.COLOR_BGR2HSV)[0] if sel.sum() else None
    paper = max(40.0, float(np.median(hsv_ref[:, 2]))) if hsv_ref is not None else 200.0
    paper_s = float(np.median(hsv_ref[:, 1])) if hsv_ref is not None else 0.0
    return {'wb': wb, 'gains': gains, 'paper': paper, 'paper_s': paper_s}


def _edge_points(ink, cx, cy, r_max):
    """Точки внешнего края по лучам из (cx, cy); лучи, дошедшие до края кадра
    без выхода на бумагу, не берём — там цель обрезана."""
    H, W = ink.shape
    pts = []
    for a in np.linspace(0, 2 * np.pi, 64, endpoint=False):
        ca, sa = np.cos(a), np.sin(a)
        paper_run, last_ink, closed = 0, None, False
        for r in np.arange(1, r_max, 0.5):
            px, py = int(cx + r * ca), int(cy + r * sa)
            if not (0 <= px < W and 0 <= py < H):
                break                                   # упёрлись в край кадра
            if ink[py, px]:
                last_ink, paper_run = r, 0
            else:
                paper_run += 1
                if last_ink is not None and paper_run >= 4:   # 2 px бумаги подряд
                    closed = True
                    break
        if closed:
            pts.append((cx + last_ink * ca, cy + last_ink * sa, last_ink))
    return pts


def _fit(pts):
    """Эллипс по точкам края с отбрасыванием выбросов."""
    if len(pts) < 10:
        return None
    r = np.array([p[2] for p in pts])
    med = np.median(r)
    xy = np.array([(p[0], p[1]) for p in pts if 0.75 * med <= p[2] <= 1.25 * med], np.float32)
    if len(xy) < 8:
        return None
    ell = cv2.fitEllipse(xy)
    # второй проход: выкидываем 20% точек, дальше всех от эллипса
    (ex, ey), (d1, d2), ang = ell
    t = np.deg2rad(ang)
    dx, dy = xy[:, 0] - ex, xy[:, 1] - ey
    u = (dx * np.cos(t) + dy * np.sin(t)) / (d1 / 2)
    v = (-dx * np.sin(t) + dy * np.cos(t)) / (d2 / 2)
    res = np.abs(np.hypot(u, v) - 1)
    keep = res <= np.percentile(res, 80)
    if keep.sum() >= 8:
        ell = cv2.fitEllipse(xy[keep])
    return ell


def fit_target(ctx, xyxy, cfg):
    """Эллипс внешнего края цели ((cx, cy), (a, b), angle_deg) на кадре ctx['wb']."""
    frame = ctx['wb']
    x1, y1, x2, y2 = xyxy
    w, h = x2 - x1, y2 - y1
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    ink = ((hsv[..., 1] >= ctx['paper_s'] + cfg['ink_sat'])
           | (hsv[..., 2] < cfg['ink_v_ratio'] * ctx['paper']))
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    r_max = 0.5 * max(w, h) * 1.4
    ell = None
    for _ in range(2):                        # 2-й проход — из центра найденного эллипса
        e = _fit(_edge_points(ink, cx, cy, r_max))
        if e is None:
            break
        (ex, ey), (d1, d2), ang = e
        if (d1 + d2) / 2 < cfg['min_fit_ratio'] * (w + h) / 2 or max(d1, d2) > 3 * max(w, h):
            break
        ell = ((ex, ey), (d1 / 2, d2 / 2), ang)
        cx, cy, r_max = ex, ey, 0.5 * max(d1, d2) * 1.3
    H, W = frame.shape[:2]
    m = 0.02 * max(W, H)
    whole = x1 > m and y1 > m and x2 < W - m and y2 < H - m     # цель видна целиком
    bcx, bcy = (x1 + x2) / 2, (y1 + y2) / 2
    if ell is not None and whole:
        # У целиком видимой цели центр эллипса обязан быть у центра рамки YOLO.
        # Уехал — эллипс нашёлся по постороннему краю; центр берём по рамке,
        # а размер — найденный, если он правдоподобный, иначе по рамке.
        (ex, ey), (a, b), ang = ell
        if np.hypot(ex - bcx, ey - bcy) / max(w, h) > cfg['box_center_tol']:
            if max(a, b) <= 0.6 * max(w, h):
                ell = ((bcx, bcy), (a, b), ang)
            else:
                ell = None
    if ell is None:   # запасной вариант — вписанный в рамку эллипс
        ell = ((bcx, bcy), (w / 2 * 0.85, h / 2 * 0.85), 0.0)
    return ell


def read_rings(frame, xyxy, cfg, draw_on=None):
    """5 колец (от центра наружу), эллипс, ctx (баланс белого, бумага)."""
    ctx = prepare(frame, xyxy, cfg)
    ell = fit_target(ctx, xyxy, cfg)
    (ecx, ecy), (a, b), ang = ell
    c0 = find_center(ctx, ell, cfg) if cfg.get('find_center', 1) else None
    cx, cy = c0 if c0 is not None else (ecx, ecy)
    ctx['center'] = (float(cx), float(cy))
    ctx['center_found'] = c0 is not None
    hsv = cv2.cvtColor(ctx['wb'], cv2.COLOR_BGR2HSV)
    H, W = hsv.shape[:2]
    paper, paper_s = ctx['paper'], ctx['paper_s']
    n_ring = int(cfg['samples'])
    angles = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
    # длина луча от центра до внешнего края по каждому направлению: при наклоне
    # с ближней стороны кольца шире, с дальней — уже, и доли по лучу это учитывают
    reach = [_ray_to_ellipse(ell, cx, cy, th) for th in angles]
    rings = []
    for i in range(5):
        rho = (i + 0.5) / 5
        votes, vals = Counter(), []
        if i == 0:   # центр: сам центр и 4 точки на половине радиуса диска
            pts = [(cx, cy)] + [(cx + 0.5 * rho * reach[k] * np.cos(angles[k]),
                                 cy + 0.5 * rho * reach[k] * np.sin(angles[k]))
                                for k in range(0, n_ring, max(1, n_ring // 4))][:4]
        else:
            pts = [(cx + rho * reach[k] * np.cos(angles[k]), cy + rho * reach[k] * np.sin(angles[k]))
                   for k in range(n_ring)]
        n = len(pts)
        for fx, fy in pts:
            px, py = int(round(fx)), int(round(fy))
            if not (1 <= px < W - 1 and 1 <= py < H - 1):
                continue
            hh, ss, vv = np.median(hsv[py - 1:py + 2, px - 1:px + 2].reshape(-1, 3), axis=0)
            c = classify(hh, ss, vv / paper, cfg, paper_s)
            votes[c] += 1
            vals.append((hh, ss, vv / paper))
            if draw_on is not None:
                cv2.circle(draw_on, (px, py), 2, DRAW_BGR[c], -1)
        color, cnt = votes.most_common(1)[0] if votes else ('?', 0)
        total = sum(votes.values())
        agree = cnt / total if total else 0
        reliable = total >= 0.5 * n and agree >= cfg['min_agree'] and color != '?'
        med = np.median(np.array(vals), axis=0) if vals else (0, 0, 0)
        rings.append({'color': color, 'score': COLOR_SCORE.get(color),
                      'agree': round(agree, 2), 'reliable': bool(reliable),
                      'in_frame': round(total / n, 2),
                      'votes': dict(votes),
                      'h': int(med[0]), 's': int(med[1]), 'v_ratio': round(float(med[2]), 2)})
    if draw_on is not None:
        cv2.ellipse(draw_on, (int(ecx), int(ecy)), (int(a), int(b)), ang, 0, 360, (255, 0, 255), 1)
        cv2.drawMarker(draw_on, (int(cx), int(cy)), (255, 0, 255), cv2.MARKER_CROSS, 12, 2)
    return rings, ell, ctx


def checksum(rings):
    """Сумма баллов или None, если хоть одно кольцо прочитано ненадёжно."""
    if any(r['score'] is None or not r.get('reliable', True) for r in rings):
        return None
    return sum(r['score'] for r in rings)


def probe(ctx, x, y):
    """HSV и V/бумага в точке кадра после баланса белого (для /tune)."""
    frame = ctx['wb']
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    x = min(max(1, int(x)), frame.shape[1] - 2)
    y = min(max(1, int(y)), frame.shape[0] - 2)
    hh, ss, vv = np.median(hsv[y - 1:y + 2, x - 1:x + 2].reshape(-1, 3), axis=0)
    return {'h': int(hh), 's': int(ss), 'v': int(vv), 'v_ratio': round(float(vv / ctx['paper']), 2),
            's_over_paper': int(ss - ctx['paper_s'])}


# ---------------------------------------------------------------- образцы цветов
# Страница /colors: человек кликает по кольцам и говорит, какого они цвета.
# Образец хранит признаки точки в тех же единицах, что и classify(): оттенок H
# и насыщенность S после баланса белого, яркость относительно бумаги, S сверх
# бумаги. По образцам подбираются пороги (fit_thresholds).

LABELS = COLORS + ['white']     # white — бумага/стена (для порога насыщенности)


def sample_features(ctx, x, y, r=2):
    """Признаки точки (медиана окна (2r+1)^2) на кадре после баланса белого."""
    frame = ctx['wb']
    H, W = frame.shape[:2]
    x = min(max(r, int(round(x))), W - r - 1)
    y = min(max(r, int(round(y))), H - r - 1)
    hsv = cv2.cvtColor(frame[y - r:y + r + 1, x - r:x + r + 1], cv2.COLOR_BGR2HSV)
    h, s, v = np.median(hsv.reshape(-1, 3), axis=0)
    return {'h': int(h), 's': int(s), 'v': int(v),
            'v_ratio': round(float(v / ctx['paper']), 3),
            's_over': int(s - ctx['paper_s']), 'paper_s': round(float(ctx['paper_s']), 1)}


def classify_sample(f, cfg):
    c = classify(f['h'], f['s'], f['v_ratio'], cfg, f['paper_s'])
    return 'white' if c == '?' else c


def _split(lo_vals, hi_vals):
    """Порог между двумя выборками: середина между 95-м перцентилем нижней и
    5-м верхней; если перекрываются — точка с наименьшим числом ошибок.
    None — в одной из выборок нет образцов (порог не трогаем)."""
    lo_vals, hi_vals = np.asarray(lo_vals, float), np.asarray(hi_vals, float)
    if not len(lo_vals) or not len(hi_vals):
        return None
    a, b = np.percentile(lo_vals, 95), np.percentile(hi_vals, 5)
    if a < b:
        return (a + b) / 2
    cand = np.unique(np.concatenate([lo_vals, hi_vals]))
    errs = [(lo_vals > t).sum() + (hi_vals <= t).sum() for t in cand]
    return float(cand[int(np.argmin(errs))])


def fit_thresholds(samples, cfg):
    """Пороги classify() по размеченным образцам. Возвращает (новый cfg, отчёт).
    Пороги, для которых нет образцов с обеих сторон, остаются прежними."""
    new = dict(cfg)
    by = {c: [s for s in samples if s['label'] == c] for c in LABELS}
    # чёрный против остальных цветных — по яркости относительно бумаги
    others = [s['v_ratio'] for c in ('red', 'yellow', 'green', 'blue') for s in by[c]]
    t = _split([s['v_ratio'] for s in by['black']], others)
    if t is not None:
        new['black_v_ratio'] = round(t, 3)
    # белое против цветного — по насыщенности сверх бумаги
    colored = [s['s_over'] for c in ('red', 'yellow', 'green', 'blue') for s in by[c]
               if s['v_ratio'] >= new['black_v_ratio']]
    t = _split([s['s_over'] for s in by['white']], colored)
    if t is not None:
        new['min_sat'] = int(round(t))
    # оттенки по кругу: красный(0) < жёлтый < зелёный < синий < красный(180)
    hue = {c: [s['h'] for s in by[c]] for c in ('yellow', 'green', 'blue')}
    red_lo = [s['h'] for s in by['red'] if s['h'] < 90]
    red_hi = [s['h'] for s in by['red'] if s['h'] >= 90]
    for lo_vals, hi_vals, k_lo, k_hi in ((red_lo, hue['yellow'], 'red_h1', 'yellow_lo'),
                                         (hue['yellow'], hue['green'], 'yellow_hi', 'green_lo'),
                                         (hue['green'], hue['blue'], 'green_hi', 'blue_lo'),
                                         (hue['blue'], red_hi, 'blue_hi', 'red_h2')):
        t = _split(lo_vals, hi_vals)
        if t is not None:
            new[k_lo], new[k_hi] = int(np.floor(t)), int(np.floor(t)) + 1
    return new, evaluate(samples, new)


def evaluate(samples, cfg):
    """Сколько образцов пороги cfg распознают правильно + путаница по цветам."""
    conf, ok = {}, 0
    for s in samples:
        got = classify_sample(s, cfg)
        ok += got == s['label']
        if got != s['label']:
            key = f"{s['label']}→{got}"
            conf[key] = conf.get(key, 0) + 1
    return {'n': len(samples), 'ok': ok, 'errors': conf}
