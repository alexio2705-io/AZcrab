"""Старт с телефона (phone_web.py) вместо кнопки на роботе.

phone_web.py кладёт файл REQUEST_PATH со стороной "left" или "right",
recovery.wait_start_button() его забирает - как нажатие GPIO27 (левая рука)
или GPIO25 (правая). Запрос, оставшийся с тех пор, когда nav.py кнопку не
ждал, стирается в начале ожидания (clear), чтобы робот не уехал сам.
"""
from __future__ import annotations

import os

REQUEST_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "start_request")
SIDES = ("left", "right")


def request(side, path=None):
    """Телефон нажал «Старт» с этой стороной."""
    path = path or REQUEST_PATH
    if side not in SIDES:
        raise ValueError(f"сторона: {' или '.join(SIDES)}")
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(side)
    os.replace(tmp, path)       # читатель не увидит пустой файл


def clear(path=None):
    try:
        os.remove(path or REQUEST_PATH)
    except FileNotFoundError:
        pass


def take(path=None):
    """Сторона из запроса (и запрос стирается) или None, если его нет."""
    path = path or REQUEST_PATH
    try:
        with open(path) as f:
            side = f.read().strip()
    except FileNotFoundError:
        return None
    clear(path)
    return side if side in SIDES else None
