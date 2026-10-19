"""Учёт времени по категориям (декоратор @timing._timed) и отчёты по нему."""
from __future__ import annotations

import time

import state


def _timed(cat):
    def wrap(fn):
        def inner(*a, **kw):
            now = time.monotonic()
            if state.tstack:
                state.tacc[state.tstack[-1][0]] = state.tacc.get(state.tstack[-1][0], 0.0) + now - state.tstack[-1][1]
            state.tstack.append([cat, now])
            try:
                return fn(*a, **kw)
            finally:
                now = time.monotonic()
                c, t0 = state.tstack.pop()
                state.tacc[c] = state.tacc.get(c, 0.0) + now - t0
                if state.tstack:
                    state.tstack[-1][1] = now
        inner.__name__, inner.__doc__ = fn.__name__, fn.__doc__
        return inner
    return wrap


_TIME_ORDER = ("ход", "поворот", "доводка", "лидар", "камеры", "жертва", "синяя")


def time_report(since, wall_s, prefix="  ⏱"):
    """Разбивка _tacc относительно снимка since за wall_s секунд."""
    parts = []
    used = 0.0
    for c in _TIME_ORDER:
        dt = state.tacc.get(c, 0.0) - since.get(c, 0.0)
        used += dt
        if dt >= 0.05:
            parts.append(f"{c} {dt:.1f}")
    rest = wall_s - used
    if rest >= 0.05:
        parts.append(f"прочее {rest:.1f}")
    return f"{prefix} {wall_s:.1f}с: " + ", ".join(parts)


def tick_report():
    """Такт drive_cells с наблюдателем меток и без - см. _tick_ms."""
    return "  ⏱ такт drive_cells, мс: " + "; ".join(
        f"наблюдатель {k}: {s.fmt()}" for k, s in state.tick_ms.items())
