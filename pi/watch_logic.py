"""Чистая логика наблюдателя меток жертв на ходу (victim_watch.py, nav.py).

Без железа и без torch - импортируется где угодно, покрыта
pi/tests/test_watch_logic.py. Дизайн: docs/superpowers/specs/
2026-10-07-victim-watch-on-the-move-design.md.

Координаты. Положение метки вдоль стены считается в мировых мм, а стена
задаётся ключом (направление, линия сетки, этаж): одна и та же стена
соседних клеток - один ключ, и метка на стыке клеток не задваивается.
Вдоль стен E/W ось - на север (+y), вдоль стен N/S - на восток (+x).
"""
from __future__ import annotations

import bisect
import math
import os

import config

CELL_MM = 300.0

# Камеры: OV5647 с рыбьим глазом 130°, кадр 320x320. Шлейф смотрит назад,
# поэтому ось "вдоль стены" - вертикаль кадра, верх кадра - вперёд по ходу.
FRAME_PX = 320

# Какой номер камеры (Picamera2) смотрит на какую сторону. Шлейфы
# переставлены (17.10): у idx 0 (порт i2c@88000) - левый борт, у idx 1
# (i2c@80000) - правый. Проверено на роботе: лидар видел стену справа в
# 118 мм, слева в 480 мм, а кадр idx 1 - стена вплотную, idx 0 - простор.
# Переставят шлейфы снова - поменять здесь, это единственное место.
CAM_IDX = {"right": 1, "left": 0}
CAM_NAMES = {i: ("правая" if s == "right" else "левая")
             for s, i in sorted(CAM_IDX.items(), key=lambda kv: kv[1])}
CAM_BACK_MM = 25.0     # объектив сдвинут назад от центра робота

# Калибровка линейкой (правая камера, 2026-10-07, робот в центре клетки):
# (y на кадре 320, мм вдоль стены от оси объектива, вперёд > 0). Напротив
# объектива - y=165, а не центр кадра 160. В кадр помещается ~125мм стены
# (±62мм), шаг ~30 px/см у оси и 18-22 у краёв. Формула рыбьего глаза
# с "18 см в кадре" ошибалась на 20-40%, поэтому - таблица.
CAM_CALIB_Y_MM = (
    (5.0, 60.0), (26.7, 50.0), (50.7, 40.0), (77.3, 30.0), (106.0, 20.0),
    (135.3, 10.0), (165.3, 0.0), (196.0, -10.0), (224.0, -20.0),
    (250.7, -30.0), (275.3, -40.0), (296.7, -50.0), (315.0, -60.0),
)
_CALIB_Y = [y for y, _ in CAM_CALIB_Y_MM]

# Метка на ходу - клетке, над которой робот: ближе этого к центру клетки -
# она, иначе следующая (см. cell_index).
ATTRIB_MM = 150.0


def prefer_ncnn(pt_path):
    """Модель для наблюдателя: экспорт NCNN рядом с .pt (maze_v26n_B.pt ->
    maze_v26n_B_ncnn_model/), если он есть, иначе сам .pt. На Pi 5 NCNN в один
    поток - ~28 мс на кадр против ~85-100 у PyTorch: метку замечают раньше,
    и робот встаёт ближе к ней. Экспорт: YOLO(pt).export(format="ncnn",
    imgsz=320)."""
    ncnn_dir = os.path.splitext(pt_path)[0] + "_ncnn_model"
    return ncnn_dir if os.path.isdir(ncnn_dir) else pt_path


# Классы-жертвы - config.VICTIM_CLASSES (задание может добавить H/S/U).
DECOY_IOU = 0.5


def _iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def victim_boxes(boxes):
    """Из выдачи YOLO [(класс, уверенность, xyxy)] - только рамки жертв.
    Фильтра по классу мало: голова YOLO26 берёт лучшие пары «якорь × класс»,
    и одна U приходит и как U 0.9, и как omega 0.5. Поэтому рамка жертвы
    выкидывается и тогда, когда на том же месте модель увереннее видит
    приманку."""
    decoys = [b for b in boxes if b[0] not in config.VICTIM_CLASSES]
    return [b for b in boxes if b[0] in config.VICTIM_CLASSES
            and not any(d[1] >= b[1] and _iou(d[2], b[2]) > DECOY_IOU for d in decoys)]


def px_to_mm(cy_frac):
    """Смещение точки кадра вдоль стены от оси объектива, мм (вперёд > 0),
    по таблице CAM_CALIB_Y_MM. cy_frac - y в долях высоты кадра (0 - верх):
    годится для любого разрешения, и 320, и 960. За краями кадра (центр
    обрезанной метки, см. mark_center_y) - продолжение крайних отрезков."""
    y = cy_frac * FRAME_PX
    i = min(max(bisect.bisect_right(_CALIB_Y, y), 1), len(_CALIB_Y) - 1)
    (y0, m0), (y1, m1) = CAM_CALIB_Y_MM[i - 1], CAM_CALIB_Y_MM[i]
    return m0 + (m1 - m0) * (y - y0) / (y1 - y0)


def mark_pos_mm(traveled_mm, offset_mm):
    """Где метка вдоль хода, мм от центра исходной клетки: центр робота
    проехал traveled_mm, объектив на CAM_BACK_MM сзади него, метка смещена
    от оси объектива на offset_mm (см. px_to_mm)."""
    return traveled_mm - CAM_BACK_MM + offset_mm


# Объектив от оси робота вбок и от него до стены, когда робот в центре
# клетки (по ним снималась калибровка CAM_CALIB_Y_MM): стена в 140 мм от
# центра робота (SIDE_CENTER_MM в nav.py).
CAM_SIDE_MM = 40.0
WALL_FACE_MM = 140.0
LAT_REF_MM = WALL_FACE_MM - CAM_SIDE_MM


def turned_mark_pos(phi_deg, side, offset_mm):
    """Где метка вдоль хода (мм от центра клетки, как mark_pos_mm при
    traveled=0), если кадр снят на повороте: робот в центре клетки повёрнут
    на phi_deg по часовой от курса сетки, метка в кадре смещена на offset_mm
    (px_to_mm). Луч из объектива под углом к стене - пересечение с ней.
    При phi=0 - то же, что mark_pos_mm(0, offset_mm)."""
    if side == "left":                 # зеркало: левая камера - правая при -phi
        phi_deg = -phi_deg
    r = math.radians(phi_deg)
    c, s = math.cos(r), math.sin(r)
    # поворот по часовой: (x вперёд, y вправо) -> (x c - y s, x s + y c)
    cx, cy = -CAM_BACK_MM * c - CAM_SIDE_MM * s, -CAM_BACK_MM * s + CAM_SIDE_MM * c
    k = offset_mm / LAT_REF_MM
    dx, dy = -s + c * k, c + s * k                 # ось объектива + смещение в кадре
    if dy <= 1e-6:
        return None                                # луч не к стене
    return cx + (WALL_FACE_MM - cy) / dy * dx


# Кадр с поворота - только когда камера смотрит на стену не круче этого. На
# диагонали (40-50°) неясно, какую из двух стен она видит: в заезде 16:53 Ψ на
# южной стене с кадра под 44° записали на восточную, не узнали как
# прочитанную и выдали второй комплект. И место по такому кадру врёт на 10+ см.
TURN_MARK_PHI_MAX = 35.0
# ...и не положе этого: кадры почти по сетке сняты в начале и в конце
# поворота, там ту же стену смотрит скан стоя (scan_sides).
TURN_MARK_PHI_MIN = 8.0


def wall_of(hdg, side):
    """Абсолютное направление стены, на которую смотрит камера side."""
    return (hdg + (1 if side == "right" else 3)) % 4


def level_of(cy, stride):
    """Этаж клетки по её cy (см. LEVEL_STRIDE в nav.py)."""
    return int(math.floor((cy + stride / 2) / stride))


def world_along(cell, hdg, wall_dir, pos_mm, stride):
    """(ключ стены, мировая координата вдоль неё в мм) для метки на стене
    wall_dir, которая в pos_mm (mark_pos_mm) от центра cell по ходу hdg.
    wall_dir - боковая стена хода, wall_of(hdg, ...)."""
    cx, cy = cell
    sgn = 1 if hdg in (0, 1) else -1          # N и E - по оси, S и W - против
    if wall_dir in (1, 3):                    # стены E/W, ось вдоль - y
        level = level_of(cy, stride)
        key = (wall_dir, cx, level)
        center = (cy - level * stride) * CELL_MM
    else:                                     # стены N/S, ось вдоль - x
        key = (wall_dir, cy)
        center = cx * CELL_MM
    return key, center + sgn * pos_mm


# Метка ближе этого к углу клетки - кандидат в ту же, что на соседней стене
# у того же угла: с поворота камера видит угол с обеих стен, и одну Ψ в
# заезде 18.10 00:34 прочитали на E и на S клетки (-2,-2) - два комплекта.
CORNER_MM = 100.0


def wall_center_along(cell, wall_dir, stride):
    """Координата вдоль стены wall_dir (как в world_along) центра её отрезка
    у клетки cell. Дальше CELL_MM/2 от неё метки на этой стене быть не может."""
    return world_along(cell, 0, wall_dir, 0.0, stride)[1]


def corner_neighbours(cell, wall_dir, along, stride, near_mm=CORNER_MM):
    """Метка на стене wall_dir клетки cell в along (world_along) ближе near_mm
    к углу клетки - [(ключ соседней стены у этого угла, координата угла
    вдоль неё)], иначе []."""
    cx, cy = cell
    level = level_of(cy, stride)
    xc, yc, half = cx * CELL_MM, (cy - level * stride) * CELL_MM, CELL_MM / 2
    out = []
    if wall_dir in (1, 3):                    # E/W: вдоль - y, у углов N и S
        xw = xc + (half if wall_dir == 1 else -half)
        for adj, y in ((0, yc + half), (2, yc - half)):
            if abs(along - y) <= near_mm:
                out.append(((adj, cy), xw))
    else:                                     # N/S: вдоль - x, у углов E и W
        yw = yc + (half if wall_dir == 0 else -half)
        for adj, x in ((1, xc + half), (3, xc - half)):
            if abs(along - x) <= near_mm:
                out.append(((adj, cx, level), yw))
    return out


def cell_index(traveled_mm):
    """Над какой по счёту клеткой поездки робот (0 - исходная): меньше
    ATTRIB_MM от центра клетки - она, дальше - следующая. Поездка без
    остановок проходит много клеток, поэтому - номер, а не "эта или
    следующая"."""
    return max(0, math.floor((traveled_mm + ATTRIB_MM) / CELL_MM))


DEDUP_MM = 150.0       # повтор ближе этого на той же стене - та же метка
# Тот же класс YOLO - та же метка и подальше: положение по кадру с поворота
# (turned_mark_pos при 30-40°) ошибается на 15-20 см, и одну Ψ в домашнем
# заезде 17.10 прочитали дважды на 150-190 мм друг от друга (лишний комплект).
DEDUP_SAME_MM = 250.0
# Подъезд к метке дальше этого - оценке не верим. Обычный случай - назад:
# кадр->решение ~175 мс при двух камерах (замер на Pi) это ~10 см на 55 см/с,
# плюс тормозной путь с упреждением ~8 см.
CREEP_MAX_MM = 250.0
EDGE_MARGIN = 0.10     # "рамка целиком в кадре" - с таким полем от краёв


class MarkRegistry:
    """Где на стенах уже есть метки: 'read' - прочитана (на ходу или стоя),
    'rejected' - наблюдатель сработал, а стоя ничего не подтвердилось. Ни
    то, ни другое больше не останавливает робота (каждая лишняя остановка -
    ~2с), а прочитанная ещё и не репортится второй раз."""

    def __init__(self, dedup_mm=DEDUP_MM, same_mm=DEDUP_SAME_MM):
        self.dedup_mm = dedup_mm
        self.same_mm = same_mm
        self.marks = []          # (ключ стены, мм вдоль неё, вид, класс YOLO или None)

    def add(self, key, along, kind, cls=None):
        self.marks.append((key, along, kind, cls))

    def near(self, key, along, cls=None):
        """Ближайшая запись на той же стене ближе dedup_mm (того же класса
        cls - ближе same_mm) или None."""
        best = None
        for k, a, kind, c in self.marks:
            d = abs(a - along)
            lim = self.same_mm if cls is not None and c == cls else self.dedup_mm
            if k == key and d < lim and (best is None or d < abs(best[1] - along)):
                best = (k, a, kind)
        return best


def decide_stop(conf, side_px, key, along, registry, busy, conf_min, min_box_px, cls=None):
    """Тормозить ли ради рамки наблюдателя: (да/нет, причина для лога).
    min_box_px - тот же фильтр дальних стен, что в VictimDetector.scan();
    cls - класс YOLO рамки (см. MarkRegistry.near)."""
    if conf < conf_min:
        return False, "conf"
    if side_px < min_box_px:
        return False, "мелкая"
    if busy:
        return False, "уже стоим"
    dup = registry.near(key, along, cls)
    if dup is not None:
        return False, "дубль" if dup[2] == "read" else "отбракована раньше"
    return True, "стоп"


def stop_point_cm(traveled_cm, v_cms, decel_cms2, lead_s):
    """Где встанет робот, если тормозить с v_cms с постоянным замедлением.
    lead_s - поправка на запаздывание (STOP_LEAD_S): профиль drive_cells
    вычитает v*lead_s из остатка, и без неё в такт решения скорость
    скачком падала бы ниже текущей - рывок."""
    return traveled_cm + v_cms * lead_s + v_cms * v_cms / (2.0 * decel_cms2)


def mark_center_y(xyxy, frame_px=FRAME_PX):
    """Центр метки по y в долях кадра. Метка, обрезанная верхним или нижним
    краем кадра (въезжает спереди или уезжает назад), выглядит ниже, чем она
    есть, а центр видимой части врёт на полметки. Ширину край не режет, а
    метки почти квадратные: центр - на полширины от необрезанного края."""
    x1, y1, x2, y2 = xyxy
    w, h = x2 - x1, y2 - y1
    if h < w and y1 <= 1.0:
        return (y2 - w / 2.0) / frame_px
    if h < w and y2 >= frame_px - 1.0:
        return (y1 + w / 2.0) / frame_px
    return (y1 + y2) / 2.0 / frame_px


def brake_target_cm(mark_pos_mm, traveled_cm, v_cms, decel_cms2, lead_s):
    """Куда тормозить ради метки: (путь в см, поздно ли). Цель - встать так,
    чтобы метка была в центре кадра (центр робота на CAM_BACK_MM впереди
    объектива), а не "как можно раньше": иначе при раннем кадре робот
    вставал до метки, при позднем - улетал за неё. Если до этой точки уже не
    затормозить, - как можно раньше, дальше подъезд назад."""
    target = (mark_pos_mm + CAM_BACK_MM) / 10.0
    soonest = stop_point_cm(traveled_cm, v_cms, decel_cms2, lead_s)
    if target >= soonest:
        return target, False
    return soonest, True


def creep_to_mark(box_y, est_mm, tol_mm, max_mm=CREEP_MAX_MM):
    """Сколько подъехать, чтобы метка встала в центр кадра: (мм или None,
    если такому подъезду не верим; источник). box_y - (y1, y2) рамки со
    стоячего кадра в долях высоты или None (метки в кадре уже нет - обычно
    робот проехал её на тормозном пути); тогда est_mm - оценка по
    координате, запомненной на ходу."""
    if box_y is not None:
        y1, y2 = box_y
        off = px_to_mm((y1 + y2) / 2.0)
        inside = y1 >= EDGE_MARGIN and y2 <= 1.0 - EDGE_MARGIN
        mm, src = (0.0 if inside and abs(off) < tol_mm else off), "рамка"
    else:
        mm, src = est_mm, "оценка"
    if abs(mm) > max_mm:
        return None, src
    return mm, src


# Сторона включена ещё столько после того, как лидар перестал видеть стену.
# Ноль: камера смотрит только туда, где стена есть сейчас. С 0.2с она
# успевала заглянуть в проём за концом стены и прочитать метку на дальней.
WATCH_SIDE_HOLD_S = 0.0


def watch_allowed(ramp_state, bump_on, assist_mode, pitch_delta, rearm_deg, wall_forced):
    """Наблюдатель - только на ровном ходу: не горка и не кандидат в неё, не
    всплеск наклона, не ассист моментом, не экстренное торможение у стены."""
    return (ramp_state is None and not bump_on and assist_mode is None
            and abs(pitch_delta) < rearm_deg and not wall_forced)


def shift_timers(times, pause_s):
    """Отметки времени, сдвинутые на паузу остановки у метки; 0.0 значит
    "не взведено" (так их и заводит drive_cells) - такие не трогаем."""
    return tuple(t + pause_s if t else t for t in times)


class OdoHistory:
    """(время, путь) за последние keep_s: путь на момент экспозиции кадра,
    а не на момент ответа воркера - на 55см/с разница 2-5см."""

    def __init__(self, keep_s=1.0):
        self.keep_s = keep_s
        self.t = []
        self.d = []

    def add(self, t, d):
        self.t.append(t)
        self.d.append(d)
        cut = 0
        while cut < len(self.t) - 1 and self.t[cut] < t - self.keep_s:
            cut += 1
        if cut:
            del self.t[:cut]
            del self.d[:cut]

    def at(self, t):
        if not self.t:
            return None
        if t <= self.t[0]:
            return self.d[0]
        if t >= self.t[-1]:
            return self.d[-1]
        i = bisect.bisect_right(self.t, t)
        t0, t1 = self.t[i - 1], self.t[i]
        d0, d1 = self.d[i - 1], self.d[i]
        return d0 + (d1 - d0) * (t - t0) / (t1 - t0)


class Stats:
    """Выборка для p50/p95/max: частота, задержки, такт цикла."""

    def __init__(self):
        self.v = []

    def add(self, x):
        self.v.append(x)

    def pct(self, p):
        if not self.v:
            return None
        s = sorted(self.v)
        return s[max(0, math.ceil(p / 100.0 * len(s)) - 1)]

    def fmt(self):
        if not self.v:
            return "—"
        return (f"p50 {self.pct(50):.1f} p95 {self.pct(95):.1f} "
                f"max {max(self.v):.1f} (n={len(self.v)})")


class SideHold:
    """Стороны, где есть стена, с удержанием hold_s (см. WATCH_SIDE_HOLD_S)."""

    def __init__(self, hold_s=WATCH_SIDE_HOLD_S, t0=0.0):
        self.hold_s = hold_s
        self.last = {}
        self.hist = {}           # side -> [(t, есть стена)] по тактам поездки
        self.t0 = t0             # кадры раньше этого - не этой поездки

    def update(self, t, present):
        for side, p in present.items():
            if p:
                self.last[side] = t
            h = self.hist.setdefault(side, [])
            h.append((t, bool(p)))
            if len(h) > 2000:    # ~10с при 200Гц - кадры старше не приходят
                del h[:1000]
        return {s for s, tl in self.last.items() if t - tl <= self.hold_s}

    def reset(self, t0=None):
        self.last.clear()
        self.hist.clear()
        if t0 is not None:
            self.t0 = t0

    def frame_ok(self, side, t_frame):
        """Рамку с кадра t_frame брать: кадр снят в этой поездке (не на
        повороте перед ней - там наблюдатель смотрит в обе стороны), и на
        последнем такте до кадра лидар видел стену с его стороны. Ответ
        воркера приходит через 0.1-0.3 с, когда стена уже могла кончиться
        (или начаться следующая), - поэтому по времени кадра, а не по
        текущему такту."""
        if t_frame < self.t0:
            return False
        h = self.hist.get(side, [])
        i = bisect.bisect_right(h, (t_frame, True)) - 1
        return i >= 0 and h[i][1]


def guess_rings(readings, n=5):
    """Кольца мишени по нескольким неполным чтениям (VictimDetector.unread_hist)
    - решение нужно всегда. Для каждого кольца: цвет из того чтения, где оно
    надёжно и согласие точек выше всех; надёжного нет - голосование по всем
    чтениям с весом согласия; не видно ни разу - жёлтое (0 баллов)."""
    out = []
    for i in range(n):
        best, votes = None, {}
        for r in readings:
            cols = r.get("rings") or []
            if i >= len(cols) or cols[i] == "?":
                continue
            ok = (r.get("ring_ok") or [False] * n)[i]
            agree = (r.get("ring_agree") or [0.0] * n)[i]
            if ok and (best is None or agree > best[1]):
                best = (cols[i], agree)
            votes[cols[i]] = votes.get(cols[i], 0.0) + max(agree, 0.1)
        if best is not None:
            out.append(best[0])
        elif votes:
            out.append(max(votes, key=lambda c: votes[c]))
        else:
            out.append("yellow")
    return out
