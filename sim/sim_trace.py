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
