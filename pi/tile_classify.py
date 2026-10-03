"""Распознавание плитки по APDS9960 по ОБРАЗЦАМ, снятым на поле.

Образцы снимает tile_calib.py (страница + кнопка GPIO27) и кладёт в
CALIB_PATH; nav.py загружает их на старте. Один и тот же classify() работает
и там, и там - страница калибровки показывает ровно то решение, которое
примет робот, а не копию порогов, которую надо держать в согласии руками
(как NAV_* в color.py).

Почему образцы, а не калибровка по стартовой клетке: старт может быть на
чекпоинте (зеркальная плитка), и тогда "белая" запоминалась зеркальной -
порог чёрной и всё, что считается от белой, уезжали вместе с ней.

Признаки - доли каналов r/c, g/c, b/c (цвет: не зависят от освещения и
высоты датчика) и ln(C) (светлота: чёрная, белая и зеркальная по долям
почти одинаковы, различаются только ей). Логарифм, а не C: освещение зала
умножает C всех плиток на один и тот же множитель, в ln(C) это сдвиг,
одинаковый для всех - расстояния между плитками не меняются.

Решение - ближайший образец в нормированном пространстве признаков (у
каждого цвета может быть несколько образцов с разных плиток). Дальше
MAX_DIST от всех - "не знаю": лучше не решить, чем принять неизвестное
за чекпоинт или за запретную клетку.
"""
from __future__ import annotations

import json
import math
import os
import time

CALIB_PATH = os.path.expanduser("~/maze/tile_calib.json")

# Цвета, которые можно откалибровать, в порядке на странице.
#   air    - робот на весу. Плиткой не является, но нужен, чтобы поднятого
#            робота не распознать как ближайшую плитку.
COLORS = ["white", "black", "blue", "red", "silver", "air"]
NAMES = {
    "white": "белая",
    "black": "чёрная (яма)",
    "blue": "синяя (стоянка)",
    "red": "красная (вход в опасную зону)",
    "silver": "зеркальная (чекпоинт)",
    "air": "воздух (робот на весу)",
}

# Масштабы признаков: разница на столько считается "одной единицей".
# Доли каналов у разных плиток расходятся на 0.1..0.4 (см. замеры в nav.py),
# а шум у тусклой чёрной - до ~0.03. ln(C): белая/чёрная - это ln(335/27)=2.5,
# зеркальная от белой по светлоте может отличаться всего в ~1.5 раза (0.4).
FRAC_SCALE = 0.03
LNC_SCALE = 0.15
# Дальше этого (в единицах выше) от ближайшего образца - "не знаю".
MAX_DIST = 6.0
SEP_WARN = 4.0      # цвета ближе этого друг к другу - предупреждение
MIN_C = 8           # ниже - датчик ничего не видит (как TILE_MIN_C в nav.py)


def features(r, g, b, c):
    c = max(c, 1)
    return (r / c, g / c, b / c, math.log(c))


def distance(fa, fb):
    return math.sqrt(sum(((x - y) / FRAC_SCALE) ** 2 for x, y in zip(fa[:3], fb[:3]))
                     + ((fa[3] - fb[3]) / LNC_SCALE) ** 2)


def load(path=None):
    """{цвет: [образец, ...]}, образец - dict r,g,b,c,n,ts. Пусто, если
    файла нет или он битый."""
    try:
        with open(path or CALIB_PATH) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    return {k: v for k, v in data.get("samples", {}).items() if k in COLORS and v}


def save(samples, path=None):
    """Атомарно (tmp + replace): nav.py, запущенный в этот момент, не должен
    прочитать половину файла."""
    path = path or CALIB_PATH
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"saved": time.strftime("%Y-%m-%d %H:%M:%S"), "samples": samples},
                  f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def classify(samples, r, g, b, c):
    """-> (цвет | None, расстояние до ближайшего образца, второй по близости
    цвет, расстояние до него). None - датчик ничего не видит, образцов нет,
    или всё слишком далеко (MAX_DIST)."""
    if c < MIN_C or not samples:
        return None, None, None, None
    f = features(r, g, b, c)
    best = {}
    for color, lst in samples.items():
        best[color] = min(distance(f, features(s["r"], s["g"], s["b"], s["c"]))
                          for s in lst)
    ranked = sorted(best.items(), key=lambda kv: kv[1])
    color, d = ranked[0]
    second, d2 = ranked[1] if len(ranked) > 1 else (None, None)
    if d > MAX_DIST:
        return None, d, color, d2
    return color, d, second, d2


def separation(samples):
    """Попарные расстояния между цветами (по ближайшим образцам), от самой
    близкой пары. Показываются на странице: пару ближе SEP_WARN шум будет
    путать, и знать об этом надо до заезда, а не на поле."""
    out = []
    keys = [k for k in COLORS if k in samples]
    for i, a in enumerate(keys):
        for b_ in keys[i + 1:]:
            d = min(distance(features(s["r"], s["g"], s["b"], s["c"]),
                             features(t["r"], t["g"], t["b"], t["c"]))
                    for s in samples[a] for t in samples[b_])
            out.append((a, b_, d))
    return sorted(out, key=lambda x: x[2])
