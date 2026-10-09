"""Трасса прогона симулятора для просмотрщика sim/viewer: поза, лидар,
события лога, снимки карты nav, мир.

Чистые функции (под тестами в sim/tests/) и Recorder - его подключает
run_scenarios.py --html через Sim.rec. Дизайн:
docs/superpowers/specs/2026-10-09-sim-animation-design.md.
"""
from __future__ import annotations

import io
import re


# ── строки лога со временем модели ─────────────────────────
class TimedLines:
    """Подмена stdout на время прогона: весь текст - в getvalue() как есть
    (это и есть лог), строки по \\n и \\r - в lines как (t, строка) со
    временем clock() на момент начала строки. Прогресс drive_cells идёт
    через \\r - каждая такая строка отдельно."""

    def __init__(self, clock):
        self.clock = clock
        self.lines = []
        self._text = io.StringIO()
        self._cur = []
        self._t = None

    def write(self, s):
        self._text.write(s)
        for piece in re.split(r"([\r\n])", s):
            if piece in ("\r", "\n"):
                self._end()
            elif piece:
                if self._t is None:
                    self._t = self.clock()
                self._cur.append(piece)
        return len(s)

    def _end(self):
        line = "".join(self._cur).rstrip()
        if line.strip():
            self.lines.append((self._t, line))
        self._cur, self._t = [], None

    def finish(self):
        """Хвост без перевода строки (последний прогресс) - тоже строка."""
        self._end()

    def flush(self):
        pass

    def getvalue(self):
        return self._text.getvalue()


# ── разбор строки лога в событие ───────────────────────────
_SIDE = re.compile(r"([RLF])=(есть|нет|\?)\(")


def _decide(m):
    return {"cell": [int(m[1]), int(m[2])], "straight": m[4] == "прямо", "reason": m[5],
            "sides": dict(_SIDE.findall(m[3]))}


# (тип, regex, поля) - первое совпадение; порядок важен там, где строки похожи
_PATTERNS = [
    ("step", r"─── \((-?\d+),(-?\d+)\) ([NESW]) → ([NESW])",
     lambda m: {"cell": [int(m[1]), int(m[2])], "from": m[3], "to": m[4]}),
    ("decide", r"→ кл\.\((-?\d+), (-?\d+)\): (.*) → (прямо|стоп) \((.*)\)\s*$", _decide),
    ("post", r"\[odo\] столбик (left|right) у границы кл\.(\d+): ([+-]?\d+)мм - цель ([+-]?\d+)мм",
     lambda m: {"side": m[1], "cell_no": int(m[2]), "err": int(m[3]), "fix": int(m[4])}),
    ("progress", r"кл\.(\d+) ост\.\s*(-?\d+)мм.*spd=(-?\d+)",
     lambda m: {"cell_no": int(m[1]), "rem": int(m[2]), "spd": int(m[3])}),
    ("brake", r"торможение с (\d+)см/с", lambda m: {"v": int(m[1])}),
    ("at_wall", r"у стены: f=(\d+)мм", lambda m: {"f": int(m[1])}),
    ("turn", r"поворот ([+-]?\d+)°\s+цель=([\d.]+)°",
     lambda m: {"angle": int(m[1]), "target": float(m[2])}),
    ("turn_done", r"итог yaw=([\d.]+)°\s+ошибка=([+-]?[\d.]+)°",
     lambda m: {"yaw": float(m[1]), "err": float(m[2])}),
    ("move", r"движение\s+курс=(\d+)°", lambda m: {"hdg": int(m[1])}),
    ("not_ramp", r"не горка: ([+-]?[\d.]+)°", lambda m: {"pitch": float(m[1])}),
    ("ramp_detect", r"горка: pitch=([+-]?[\d.]+)°, момент ([\d.]+)A",
     lambda m: {"pitch": float(m[1]), "amps": float(m[2])}),
    ("ramp_measure", r"горка: (\d+)мм по уклону.*уклон ([\d.]+)°, высота ([+-]?\d+)мм",
     lambda m: {"length": int(m[1]), "slope": float(m[2]), "rise": int(m[3])}),
    ("ramp_up", r"ГОРКА ВВЕРХ \(([+-]?[\d.]+)°\)", lambda m: {"pitch": float(m[1])}),
    ("ramp_down", r"ГОРКА ВНИЗ \(([+-]?[\d.]+)°\)", lambda m: {"pitch": float(m[1])}),
    ("ramp_done", r"⛰ ГОРКА (вверх|вниз): .*\((-?\d+),(-?\d+)\) → \((-?\d+),(-?\d+)\)",
     lambda m: {"up": m[1] == "вверх", "from": [int(m[2]), int(m[3])],
                "to": [int(m[4]), int(m[5])]}),
    ("obstacle", r"препятствие #(\d+): pitch=([+-]?[\d.]+)°.*момент ([\d.]+)A",
     lambda m: {"n": int(m[1]), "pitch": float(m[2]), "amps": float(m[3])}),
    ("bumper", r"БАМПЕР \((\w+)\) на (\d+)мм", lambda m: {"side": m[1], "mm": int(m[2])}),
    ("backoff", r"откат (\d+)мм", lambda m: {"mm": int(m[1])}),
    ("align", r"\[выр\]", lambda m: {}),
]
_COMPILED = [(t, re.compile(r), f) for t, r, f in _PATTERNS]


def parse_line(text):
    """Строка лога nav -> {"type": ..., поля}; нераспознанная - "other"."""
    for typ, rx, fields in _COMPILED:
        m = rx.search(text)
        if m:
            return {"type": typ, **fields(m)}
    return {"type": "other"}


def parse_log(lines, t0=0.0):
    """[(t, строка)] -> события с временем от t0 (до 1 мс) и текстом."""
    return [{"t": round(t - t0, 3), "text": s.strip(), **parse_line(s)} for t, s in lines]


# ── поза ───────────────────────────────────────────────────
POSE_DT = 0.02          # 50 Гц
_POSE_ND = {"x": 1, "y": 1, "z": 1, "th": 2, "p": 2, "vl": 2, "vr": 2,
            "sl": 2, "sr": 2, "cap": 2, "f": 1}


def thin_poses(samples, keep_times=(), dt=POSE_DT):
    """Сэмплы каждого шага модели (5 мс) -> кадры для анимации: раз в dt,
    плюс первый и последний, смена режима команды или бампера и первый
    кадр не раньше каждого из keep_times (строки лога)."""
    keep = sorted(keep_times)
    ki = 0
    out = []
    last_t = None
    prev = None
    for i, s in enumerate(samples):
        take = (prev is None or i == len(samples) - 1 or s["t"] >= last_t + dt - 1e-9
                or s["m"] != prev["m"] or s["b"] != prev["b"])
        while ki < len(keep) and keep[ki] <= s["t"] + 1e-9:
            take = True
            ki += 1
        if take:
            out.append(s)
            last_t = s["t"]
        prev = s
    return out


def pose_columns(poses, t0):
    """Кадры -> колонки (компактнее в JSON): числа округлены, режим - строка
    букв M/T/B, бампер - 0/1, передняя стена - мм или None."""
    cols = {"t": [round(p["t"] - t0, 3) for p in poses],
            "m": "".join(p["m"] for p in poses),
            "b": [1 if p["b"] else 0 for p in poses]}
    for k, nd in _POSE_ND.items():
        cols[k] = [None if p[k] is None else round(p[k], nd) for p in poses]
    return cols


# ── мир ────────────────────────────────────────────────────
CELL = 300.0
LEVEL_STRIDE = 1000     # как nav.LEVEL_STRIDE
WALL_PROBE_Z = 40.0     # стена ребра должна перекрывать пол клетки + столько
CLIFF_MM = 50.0         # перепад рельефа на ребре больше этого - тоже стена (уступ острова)


def sample_terrain(x0, y0, x1, y1, f, step=10.0):
    """Кусок рельефа (прямоугольник + функция высоты) -> описание для
    отрисовки: плоский ({h}) или наклонный ({dir подъёма, h0 низ, h1 верх})."""
    nx = max(1, int(round((x1 - x0) / step)))
    ny = max(1, int(round((y1 - y0) / step)))
    hs = [f(x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny)
          for i in range(nx + 1) for j in range(ny + 1)]
    rect = [round(v, 1) for v in (x0, y0, x1, y1)]
    lo, hi = min(hs), max(hs)
    if hi - lo < 0.5:
        return {"kind": "flat", "rect": rect, "h": round(hi, 1)}
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    n, e, s, w = f(mx, y1), f(x1, my), f(mx, y0), f(x0, my)
    rise = [n - s, e - w, s - n, w - e]
    return {"kind": "slope", "rect": rect, "dir": "NESW"[rise.index(max(rise))],
            "h0": round(lo, 1), "h1": round(hi, 1)}


def world_json(walls, pieces):
    return {"cell": int(CELL),
            "walls": [[round(v, 1) for v in w] for w in walls],
            "terrain": [sample_terrain(*p) for p in pieces]}


def level_of(cy):
    """Этаж клетки по cy - как nav.level_of."""
    return int((cy + LEVEL_STRIDE / 2) // LEVEL_STRIDE)


def edge_truth(walls, height, cx, cy, d):
    """Есть ли в мире стена на ребре d клетки (cx, cy) карты nav (любой
    этаж): стена должна закрывать середину ребра на высоте пола клетки +
    WALL_PROBE_Z; или рельеф на ребре обрывается больше чем на CLIFF_MM
    (остров сбоку: его стены стоят на нём, снизу стену заменяет уступ)."""
    wy = cy - level_of(cy) * LEVEL_STRIDE
    x, y = cx * CELL, wy * CELL
    h = CELL / 2
    px, py = ((x, y + h), (x + h, y), (x, y - h), (x - h, y))[d]
    z = height(x, y) + WALL_PROBE_Z
    ox, oy = (px - x) / h * 5.0, (py - y) / h * 5.0
    if abs(height(px - ox, py - oy) - height(px + ox, py + oy)) > CLIFF_MM:
        return True
    return any(x0 <= px <= x1 and y0 <= py <= y1 and z0 <= z <= z1
               for x0, y0, x1, y1, z0, z1 in walls)


def truth_rows(walls, height, cells):
    """[[cx, cy, N, E, S, W]] правды мира (1/0) по клеткам карты."""
    return [[cx, cy, *(1 if edge_truth(walls, height, cx, cy, d) else 0 for d in range(4))]
            for cx, cy in sorted(set(cells))]


# ── карта nav ──────────────────────────────────────────────
def _tri(v):
    return None if v is None else (1 if v else 0)


def map_snapshot(maze, visited, ramps, blocked, edge_fixed, phase):
    """Карта, как её видит nav, -> сортированный JSON-снимок."""
    return {
        "phase": phase,
        "maze": sorted([cx, cy, *(_tri(v) for v in w)] for (cx, cy), w in maze.items()),
        "visited": sorted([cx, cy] for cx, cy in visited),
        "ramps": sorted([cx, cy, d, *v[:2], round(v[2], 2), v[3]]
                        for (cx, cy, d), v in ramps.items()),
        "blocked": sorted([cx, cy] for cx, cy in blocked),
        "fixed": [list(e) for e in sorted({e for k, v in edge_fixed.items() if v is True
                                            for e in k})],
    }
