"""Поддельные камеры и лента для симулятора: то, чего ms.nav.py ждёт от
victim_detect.VictimDetector, victim_watch.VictimWatcher и leds.

Камеры - по геометрии, а не по подсказке ms.nav.py: метка попадает в кадр,
только если она на стене со стороны камеры, в пределах её поля (~125 мм
стены у центра клетки) и на том же этаже. Где метка в кадре - по обратной
таблице калибровки watch_logic.CAM_CALIB_Y_MM (как её снял бы настоящий
объектив). Клетку ms.nav.py приписывает метке сам - если он ошибся в своём
положении, оценка заезда это увидит (в результате есть "_truth").

Ошибки зрения - из Sim.noise: пропуск метки, путаница типа, ложная метка.
Скан стоя стоит модельного времени (кадр + YOLO), но только в главном
потоке: ms.nav.py сканирует вторую сторону в соседнем потоке одновременно, а
двигать часы модели из двух потоков нельзя.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import maze_sim as ms
import watch_logic

CAM_SIDE_MM = 40.0         # объектив от оси робота вбок
MARK_MM = 45.0             # сторона метки
LAT_REF_MM = 100.0         # от объектива до стены при калибровке (центр клетки)
LAT_RANGE = (30.0, 240.0)  # ближе/дальше - метка не читается
MIN_VISIBLE_PX = 14
STILL_SCAN_S = 0.15        # кадр + YOLO стоя
BEST_FAST_S = 0.10
WATCH_HZ = 12.0            # на ходу упирается в инференс
WATCH_LATENCY_S = 0.09

CLASS_OF = {"damaged": "phi", "stable": "psi", "unharmed": "omega", "fake": "cognetive"}
LABEL = {"damaged": "Φ", "stable": "Ψ", "unharmed": "Ω", "fake": "+3"}
REAL = ("damaged", "stable", "unharmed")

_CAL = watch_logic.CAM_CALIB_Y_MM


def mm_to_y(mm):
    """Обратная к watch_logic.px_to_mm: мм вдоль стены от оси объектива
    (вперёд > 0) -> y на кадре 320. За краями - продолжение крайних отрезков."""
    pts = sorted(((m, y) for y, m in _CAL))           # по мм, по возрастанию
    i = 1
    while i < len(pts) - 1 and pts[i][0] < mm:
        i += 1
    (m0, y0), (m1, y1) = pts[i - 1], pts[i]
    return y0 + (y1 - y0) * (mm - m0) / (m1 - m0)


def _main_thread():
    return threading.current_thread() is threading.main_thread()


def _sleep(s):
    if _main_thread():
        ms.nav.time.sleep(s)


def view(sim, side):
    """Метки в кадре камеры side сейчас: [(индекс, метка, xyxy в кадре 320)]."""
    th = math.radians(sim.theta)
    fwd = (math.sin(th), math.cos(th))
    sgn = 1.0 if side == "right" else -1.0
    sv = (math.cos(th) * sgn, -math.sin(th) * sgn)
    cx = sim.x - fwd[0] * watch_logic.CAM_BACK_MM + sv[0] * CAM_SIDE_MM
    cy = sim.y - fwd[1] * watch_logic.CAM_BACK_MM + sv[1] * CAM_SIDE_MM
    h_robot = sim.w.height(sim.x, sim.y)
    out = []
    for i, v in enumerate(sim.w.victims):
        px, py = v.point()
        nx, ny = ((0, 1), (1, 0), (0, -1), (-1, 0))[v.d]
        if nx * sv[0] + ny * sv[1] < 0.9:          # стена не напротив камеры
            continue
        rx, ry = px - cx, py - cy
        lat = rx * sv[0] + ry * sv[1]
        if not LAT_RANGE[0] <= lat <= LAT_RANGE[1]:
            continue
        if abs(sim.w.height(px - nx * 40, py - ny * 40) - h_robot) > 60.0:
            continue                                 # другой этаж
        k = LAT_REF_MM / lat                         # дальше стена - мельче и шире поле
        off = (rx * fwd[0] + ry * fwd[1]) * k
        half = MARK_MM / 2.0 * k
        ya, yb = mm_to_y(off + half), mm_to_y(off - half)
        y1, y2 = max(0.0, min(ya, yb)), min(320.0, max(ya, yb))
        if y2 - y1 < MIN_VISIBLE_PX:
            continue
        w = abs(yb - ya)
        out.append((i, v, (160.0 - w / 2, y1, 160.0 + w / 2, y2)))
    return out


def detections(sim, side, still):
    """Что "увидела" YOLO на кадре: [(класс, уверенность, xyxy, индекс|None, тип)]
    с ошибками из sim.noise. still - кадр стоя (ложные метки - только там)."""
    n, rng = sim.noise, sim.rng
    out = []
    for i, v, box in view(sim, side):
        if n.vis_miss and rng.random() < n.vis_miss:
            continue
        t = v.type
        if t in REAL and n.vis_wrong and rng.random() < n.vis_wrong:
            t = rng.choice([x for x in REAL if x != t])
        out.append((CLASS_OF[t], round(0.55 + 0.4 * rng.random(), 3), box, i, t))
    if still and not out and n.vis_false and rng.random() < n.vis_false:
        t = rng.choice(REAL)
        out.append((CLASS_OF[t], round(0.46 + 0.2 * rng.random(), 3), (100.0, 110.0, 220.0, 230.0), None, t))
    return out


class FakeDetector:
    """victim_detect.VictimDetector для одной стороны."""

    def __init__(self, sim, side):
        self.sim, self.side = sim, side
        self.scanned = set()
        self.log = []
        self.cam = None

    def scan(self, cell_x, cell_y, wall_dir=None):
        key = (cell_x, cell_y, wall_dir)
        if key in self.scanned:
            return None
        self.scanned.add(key)
        return self.read_wall(cell_x, cell_y, wall_dir)

    def read_wall(self, cell_x, cell_y, wall_dir=None):
        _sleep(STILL_SCAN_S)
        found = detections(self.sim, self.side, still=True)
        found = [f for f in found if f[2][3] - f[2][1] >= watch_logic.FRAME_PX * 0.125]
        if not found:
            return None
        cls, conf, box, idx, t = max(found, key=lambda f: f[1])
        result = {
            "type": t, "label": LABEL[t], "image": None, "cell": (cell_x, cell_y),
            "class": cls, "conf": conf, "blink": t in REAL,
            "rings": None, "checksum": 3 if t == "fake" else None,
            "box_y": (round(box[1] / 320.0, 4), round(box[3] / 320.0, 4)),
            "_truth": idx,
        }
        self.log.append(result)
        print(f"[VD] {'★ ЖЕРТВА' if t in REAL else '○ МЕТКА (без мигания)'} {t.upper()} "
              f"({LABEL[t]})  клетка=({cell_x},{cell_y})  класс={cls}  conf={conf:.2f}  [сим]")
        return result

    def best_fast(self):
        _sleep(BEST_FAST_S)
        found = detections(self.sim, self.side, still=False)
        if not found:
            return None
        cls, conf, box, _, _ = max(found, key=lambda f: f[1])
        if max(box[2] - box[0], box[3] - box[1]) < 40:
            return None
        return cls, conf, (box[1] / 320.0, box[3] / 320.0)

    def close(self):
        pass


@dataclass
class WatchBox:            # как victim_watch.WatchBox
    side: str
    cls: str
    conf: float
    xyxy: tuple
    t_frame: float
    image: str | None


class FakeWatcher:
    """victim_watch.VictimWatcher: кадр каждой включённой стороны WATCH_HZ
    раз в секунду модельного времени, рамки - через WATCH_LATENCY_S."""

    def __init__(self, sim):
        self.sim = sim
        self.ready = threading.Event()
        self.ready.set()
        self.dead = False
        self._armed = set()
        self._next = {}
        self._pending = []
        self._live = {s: {"fid": 0, "t": None, "boxes": [], "bt": None} for s in ("left", "right")}

    def arm(self, sides):
        self._armed = set(sides)
        return bool(self._armed)

    def poll(self):
        now = self.sim.t
        for side in self._armed:
            if now >= self._next.get(side, 0.0):
                self._next[side] = now + 1.0 / WATCH_HZ
                boxes = detections(self.sim, side, still=False)
                self._pending.append((now + WATCH_LATENCY_S, side, now, boxes))
                lv = self._live[side]
                lv["fid"] += 1
                lv["t"] = now
        out, keep = [], []
        for p in self._pending:
            (out if p[0] <= now else keep).append(p)
        self._pending = keep
        res = []
        for _, side, tf, boxes in out:
            if side not in self._armed:
                continue
            lv = self._live[side]
            lv["boxes"] = [[c, conf, [round(v) for v in box]] for c, conf, box, _, _ in boxes]
            lv["bt"] = tf
            res += [WatchBox(side, c, conf, box, tf, None) for c, conf, box, _, _ in boxes]
        return res

    def wait_idle(self, timeout=0.2):
        pass

    def take_stats(self):
        return None

    def live_info(self):
        return {s: {"shm": None, "armed": s in self._armed, "failed": False, "dead": False,
                    "ready": True, **lv} for s, lv in self._live.items()}

    def close(self):
        self.dead = True


class FakeLeds:
    """leds.NeoPixels: fill/clear/show/close - ничего не делают."""

    def __getattr__(self, name):
        return lambda *a, **k: None


def blink_pink(px, seconds=5.3, stop=None):
    """Индикация жертвы: стоим seconds модельного времени (как настоящая),
    событие для оценки - с последней записью ms.nav._victim_log."""
    sim = ms.CURRENT
    last = ms.nav._victim_log[-1] if ms.nav._victim_log else {}
    sim.event("blink", s=seconds, type=last.get("type"), truth=last.get("_truth"),
              cell=list(last.get("cell") or ()), on_move=bool(last.get("on_move")))
    end = ms.nav.time.monotonic() + seconds
    while ms.nav.time.monotonic() < end:
        if stop is not None and stop():
            break
        ms.nav.time.sleep(0.05)


def blink_return(px, seconds=10.5):
    sim = ms.CURRENT
    sim.event("finish_blink", s=seconds)
    ms.nav.time.sleep(seconds)


def install(sim, vision=True):
    """Поставить подделки в nav (после ms.setup)."""
    nav = ms.nav
    ms.nav._leds = FakeLeds()
    ms.nav.blink_pink = blink_pink
    ms.nav.blink_return = blink_return
    if vision:
        ms.nav._victim_r = FakeDetector(sim, "right")
        ms.nav._victim_l = FakeDetector(sim, "left")
        ms.nav._watcher = FakeWatcher(sim)
