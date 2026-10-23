#!/usr/bin/env python3
"""Режим технического испытания - заготовка, чтобы на площадке не писать
логику заново, а заполнить таблицу под условие (см. docs/tech_challenges.md).

Ездит тем же кодом, что и nav.py (drive_cells, turn, карта, BFS, чекпоинт,
сон по кнопке), но вместо обычных правил RoboCup - настраиваемые:
  * tiles  - реакция на цвет плитки: стоять stop_s, мигнуть blinks раз
             (on_s/off_s; если не заданы - stop_s делится поровну), сбросить
             kits комплектов на месте, every_visit - на каждом заезде, а не
             один раз на клетку. Цвета: blue / red / black / silver / white.
  * order  - цвета, которые надо отработать именно в этом порядке. Плитку
             не по порядку только запоминаем и едем дальше; когда до неё
             дойдёт очередь - едем к ней по карте.
  * finish - "start" или цвет плитки; finish_after - какие цвета надо
             посетить до финиша; finish_hold_s - сколько стоять с
             выключенными моторами.
  * explore_all - обойти весь лабиринт до финиша (искать буквы), иначе
             финиш сразу, как только всё нужное сделано.
  * time_limit_s / sec_per_cell - таймер: когда до конца остаётся столько,
             сколько нужно на дорогу до финиша, бросаем всё и едем финишировать.
  * victims - обычные жертвы Φ/Ψ/Ω/когнитивные (как в nav.py, YOLO);
    victim_kits - сколько комплектов на любую букву (None - как в nav.py);
    letters - буквы H/S/U классикой (letters_cv.py): {буква: реакция}.
  * order_from_cognitive - порядок плиток = цвета колец когнитивной метки у
             старта (от центра наружу); сама метка без индикации и комплектов.
  * strict_order - цветная плитка из tiles отрабатывается ТОЛЬКО когда её
             очередь (иначе - когда угодно, пока её цвета нет в order).
  * order_greedy - ехать к следующей по порядку плитке, как только она
             известна (True), или сначала разведать всё, отрабатывая
             попутные, а потом объехать оставшиеся (False).
  * black_passable - чёрная - обычная плитка, по ней можно ездить.
  * maze_size - лабиринт N x N, старт в углу: края берутся по стенам
             стартовой клетки, клетки за краем не разведываются.
  * stairs_line_open - линия (строка/столбец по ходу), где робот проехал
             лестницу или горку, целиком без стен: открываем её в карте
             насовсем, чтобы лидар на наклоне не нарисовал там ложных стен.
  * push_through_small - мелкие препятствия заведомо проезжаемы: не взял с
             ходу - толкать дальше без отката (nav.PUSH_THROUGH_SMALL).
  * single_level - этажей нет: горка - обычные плитки той же плоской сетки
             (nav.py увёл бы клетки за ней на другой этаж и выкинул плитки
             самой горки из карты - линия становилась короче).

Чёрная плитка здесь не стена навсегда: при разведке робот, как и в nav.py,
откатывается от неё (и так узнаёт, где она, не заезжая - это важно для
порядка), а когда до чёрной доходит очередь или она финиш - заезжает в центр.

Запуск на малине:
    ~/maze/venv/bin/python ~/maze/challenge.py tomsk
    ~/maze/venv/bin/python ~/maze/challenge.py colors --time 300
    ~/maze/venv/bin/python ~/maze/challenge.py cog
    ~/maze/venv/bin/python ~/maze/challenge.py tomsk --cfg my.json --show
--show печатает итоговую таблицу и выходит (робот не нужен). my.json - те
же ключи, что в PRESETS; поверх пресета, "tiles" сливаются по цветам.
"""

from __future__ import annotations

import copy
import json
import os
import sys
import threading
import time
from contextlib import contextmanager, nullcontext, redirect_stdout

nav = None   # импортируется в main(): на ноутбуке без RPi.GPIO работает --show

# ══════════════════════════════════════════════════════════════
#  Пресеты - известные задания
# ══════════════════════════════════════════════════════════════
DEFAULTS = {
    "tiles": {},
    "order": [],
    "finish": "start",
    "finish_after": [],
    "finish_hold_s": 11.0,      # правила "не менее 10с" + запас на счёт судьи
    "explore_all": False,
    "time_limit_s": None,
    "sec_per_cell": 5.0,        # ЗАМЕРИТЬ на поле: среднее время на клетку
    "time_margin_s": 10.0,
    "victims": False,
    "victim_kits": None,
    "letters": {},
    "order_from_cognitive": False,
    "strict_order": False,
    "order_greedy": True,
    "black_passable": False,
    "maze_size": None,
    "stairs_line_open": False,
    "single_level": False,
    "push_through_small": False,
    "web": True,
}

_VICTIM_SIGNAL = {"stop_s": 5.0, "blinks": 5, "kits": 1}   # как жертва: 5с, 0.5/0.5

PRESETS = {
    # Томск 16.05.2026: синяя -> красная -> чёрная, мигание 1с/1с,
    # комплекты 1/2/3, возврат на старт, 3 минуты, бонус за остаток времени.
    "tomsk": {
        "tiles": {
            "blue":  {"blinks": 1, "on_s": 1.0, "off_s": 1.0, "kits": 1},
            "red":   {"blinks": 3, "on_s": 1.0, "off_s": 1.0, "kits": 2},
            "black": {"blinks": 3, "on_s": 1.0, "off_s": 1.0, "kits": 3},
        },
        "order": ["blue", "red", "black"],
        "finish": "start",
        "time_limit_s": 180,
    },
    # 2 красных, 2 синих, 1 чёрная: чёрная - стоп 3с + 3 мигания, синяя -
    # не реагировать, красная - стоп 5с + 5 миганий; буквы H/S; финиш на
    # чёрной после синей и красной.
    "colors": {
        "tiles": {
            "black": {"stop_s": 3.0, "blinks": 3},
            "red":   {"stop_s": 5.0, "blinks": 5, "every_visit": True},
        },
        "finish": "black",
        "finish_after": ["blue", "red"],
        "explore_all": True,
        "letters": {
            "H": {"stop_s": 5.0, "blinks": 5},
            "S": {"stop_s": 5.0, "blinks": 5},
        },
    },
    # 5x5, старт в углу. У старта две одинаковые когнитивные метки из
    # красных/чёрных/синих колец: порядок колец от центра наружу - порядок
    # плиток, на каждой стоп 5с + мигание + 1 комплект. Плиток каждого цвета
    # столько же, сколько колец. По любым плиткам ездить можно (и по
    # чёрной). На буквах Φ/Ψ/Ω - по одному комплекту (букв <= 3, всего 8).
    # Финиш - чекпоинт (серебряная).
    "cog": {
        "tiles": {"red": dict(_VICTIM_SIGNAL), "black": dict(_VICTIM_SIGNAL),
                  "blue": dict(_VICTIM_SIGNAL)},
        "order_from_cognitive": True,
        "strict_order": True,
        "order_greedy": False,
        "black_passable": True,
        "maze_size": 5,
        "stairs_line_open": True,
        "single_level": True,
        "push_through_small": True,
        "finish": "silver",
        "explore_all": True,
        "victims": True,
        "victim_kits": 1,
    },
}

RING_COLORS = ("red", "black", "blue")   # из чего состоит метка в "cog"
# Других цветов в метке нет - значит, это ошибка чтения соседнего оттенка:
# жёлтый ближе всех к красному по тону, зелёный - к синему.
RING_FIX = {"yellow": "red", "green": "blue"}

SIGNAL_MARGIN_S = 0.3   # как MARGIN_S в leds.py: судья считает секунды глазами
COLOR_READ_S = 0.4      # сколько ждать внятного цвета под роботом в центре
MAX_STALLS = 5          # как в nav.py: столько шагов подряд без продвижения - конец


def load_config(name, path=None, overrides=None):
    if name not in PRESETS:
        raise SystemExit(f"нет пресета {name!r}, есть: {', '.join(PRESETS)}")
    cfg = copy.deepcopy(DEFAULTS)
    layers = [PRESETS[name]]
    if path:
        with open(path) as f:
            layers.append(json.load(f))
    if overrides:
        layers.append(overrides)
    for layer in layers:
        for k, v in layer.items():
            if k in ("tiles", "letters"):
                cfg[k].update(copy.deepcopy(v))
            else:
                cfg[k] = copy.deepcopy(v)
    return cfg


def signal_timing(spec):
    """Реакция -> (миганий, вкл, выкл, всего стоять). Если on_s/off_s не
    заданы - stop_s делится поровну на blinks циклов ("3 раза за 3с")."""
    n = int(spec.get("blinks", 0))
    stop_s = float(spec.get("stop_s", 0.0))
    on, off = spec.get("on_s"), spec.get("off_s")
    if n and (on is None or off is None):
        period = stop_s / n if stop_s else 1.0
        on = period / 2 if on is None else on
        off = period / 2 if off is None else off
    on, off = on or 0.0, off or 0.0
    total = max(stop_s, n * (on + off))
    return n, on, off, total


def describe(spec):
    n, on, off, total = signal_timing(spec)
    parts = []
    if total:
        parts.append(f"стоп {total:.1f}с")
    if n:
        parts.append(f"мигнуть {n}x ({on:.3g}/{off:.3g}с)")
    if spec.get("kits"):
        parts.append(f"комплектов {spec['kits']}")
    if spec.get("every_visit"):
        parts.append("каждый заезд")
    return ", ".join(parts) or "ничего"


def print_config(cfg):
    print("Реакции на плитки:")
    for color, spec in cfg["tiles"].items():
        print(f"  {color:7s} {describe(spec)}")
    print("  остальные цвета - без реакции (синяя тоже: стоянки nav.py нет)")
    if cfg["order_from_cognitive"]:
        print("Порядок: по кольцам когнитивной метки у старта (от центра наружу)")
    elif cfg["order"]:
        print(f"Порядок: {' -> '.join(cfg['order'])}")
    if cfg["strict_order"]:
        print("  цветные плитки - только в свою очередь")
    print(f"  {'сразу к следующей' if cfg['order_greedy'] else 'сначала весь лабиринт, попутные - по ходу'}")
    if cfg["black_passable"]:
        print("Чёрная - обычная плитка, ездим по ней")
    if cfg["maze_size"]:
        print(f"Лабиринт {cfg['maze_size']}x{cfg['maze_size']}, старт в углу")
    if cfg["stairs_line_open"]:
        print("Линия с лестницей - без стен (открываем после проезда лестницы)")
    if cfg["push_through_small"]:
        print("Мелкие препятствия: без отката, толкать дальше")
    if cfg["letters"]:
        print("Буквы: " + "; ".join(f"{k}: {describe(v)}" for k, v in cfg["letters"].items()))
    kits = "" if cfg["victim_kits"] is None else f", комплектов на любую: {cfg['victim_kits']}"
    print(f"Жертвы Φ/Ψ/Ω: {'да' + kits if cfg['victims'] else 'нет'}")
    after = f" после {', '.join(cfg['finish_after'])}" if cfg["finish_after"] else ""
    print(f"Финиш: {cfg['finish']}{after}, стоять {cfg['finish_hold_s']:.0f}с; "
          f"{'весь лабиринт' if cfg['explore_all'] else 'сразу, как всё сделано'}")
    if cfg["time_limit_s"]:
        print(f"Время: {cfg['time_limit_s']}с, {cfg['sec_per_cell']}с/клетку, "
              f"запас {cfg['time_margin_s']}с")


# ══════════════════════════════════════════════════════════════
#  Решения - без железа (проверяется на ноутбуке)
# ══════════════════════════════════════════════════════════════
class Mission:
    """Что уже сделано и куда ехать дальше. Карта и BFS - снаружи
    (функции bfs/frontier), так что логику можно гонять без робота.

    Маршрут - (kind, path, black_dir): path - список направлений по карте,
    black_dir - последний шаг С заездом на чёрную (её клетка в карте
    закрыта, BFS туда не ведёт - едем до соседней и заезжаем руками)."""

    def __init__(self, cfg):
        self.cfg = cfg
        # None - порядок ещё не известен (прочитаем с когнитивной метки).
        self.pending = None if cfg["order_from_cognitive"] else list(cfg["order"])
        self.order_read = None   # кольца, как их прочитали (pending потом убывает)
        self.order_guess = None  # ненадёжное чтение колец - если лучше не будет
        self.start_bounds = {}   # ось 'x'/'y' -> (мин, макс) по стенам старта
        self.known = {}          # цвет -> {клетки}
        self.cell_color = {}     # клетка -> цвет, однажды прочитанный
        self.black_from = {}     # чёрная клетка -> (откуда её видели x, y, направление)
        self.reacted = set()     # клетки, где реакция уже была
        self.visited_colors = set()
        self.relaxed = False     # порядок уже не спасти - реагируем на что есть
        self.hurry = False       # время кончается - только к финишу

    # ── наблюдения ──
    def saw(self, cell, color):
        if color in (None, "?", "white"):
            return
        self.cell_color[cell] = color
        self.known.setdefault(color, set()).add(cell)

    def saw_cognitive(self, rings, cell, reliable=True):
        """Кольца метки (от центра наружу) -> порядок плиток. Первое верное
        чтение задаёт порядок; вторая метка такая же - расхождение печатаем.
        Ненадёжное (ring_colors не уверен хоть в одном кольце) - только
        запасной вариант, см. accept_guess(); два одинаковых ненадёжных -
        уже достаточно."""
        fixed = [RING_FIX.get(c, c) for c in rings or []]
        if len(fixed) != 5 or any(c not in RING_COLORS for c in fixed):
            print(f"  ✗ когнитивная {cell}: кольца {rings} - не прочитались")
            return
        if fixed != list(rings):
            print(f"  ~ кольца {rings} -> {fixed} (в метке только {'/'.join(RING_COLORS)})")
        if not reliable and self.order_read is None:
            if fixed != self.order_guess:
                self.order_guess = fixed
                print(f"  ? кольца {cell}: {' -> '.join(fixed)} - ненадёжно, запомнил")
                return
            print("  ? то же самое второй раз - принимаю")
        if self.order_read is None:
            self.order_read = fixed
            self.pending = list(fixed)
            print(f"  ★ ПОРЯДОК ПЛИТОК: {' -> '.join(fixed)}  (метка в {cell})")
        elif fixed != self.order_read:
            print(f"  [WARN] вторая метка {cell}: {fixed} - не совпала, оставляю первую")

    def set_start_walls(self, walls):
        """Стены стартовой клетки [N, E, S, W] -> края лабиринта N x N.
        Верим только однозначному углу: по оси ровно одна стена из двух."""
        n = self.cfg["maze_size"]
        if not n or walls is None:
            return
        for axis, lo_wall, hi_wall in (("x", walls[3], walls[1]), ("y", walls[2], walls[0])):
            if lo_wall is True and hi_wall is not True:
                self.start_bounds[axis] = (0, n - 1)
            elif hi_wall is True and lo_wall is not True:
                self.start_bounds[axis] = (-(n - 1), 0)
        print(f"  края по стартовому углу: {self.start_bounds or 'не понятно'}")

    def bounds(self, axis, visited):
        """(мин, макс) по оси или None. Стартовый угол - если посещённые
        клетки ему не противоречат; иначе - размах посещённых, когда он
        уже ровно N."""
        n = self.cfg["maze_size"]
        if not n:
            return None
        i = 0 if axis == "x" else 1
        vals = [c[i] for c in visited if abs(c[1]) < 500]    # только этаж 0
        b = self.start_bounds.get(axis)
        if b and vals and (min(vals) < b[0] or max(vals) > b[1]):
            print(f"  [WARN] клетка за краем {axis}={b} - края по старту неверны")
            del self.start_bounds[axis]
            b = None
        if b:
            return b
        if vals and max(vals) - min(vals) == n - 1:
            return min(vals), max(vals)
        return None

    def in_bounds(self, cell, visited):
        if not self.cfg["maze_size"] or abs(cell[1]) >= 500:
            return True
        for i, axis in enumerate(("x", "y")):
            b = self.bounds(axis, visited)
            if b and not b[0] <= cell[i] <= b[1]:
                return False
        return True

    def stairs_line(self, cx, cy, d, visited):
        """Лестницу проехали из (cx,cy) по направлению d: рёбра линии по ходу
        от края до края -> [(x, y, направление)] или None (края неизвестны)."""
        axis = "y" if d in (0, 2) else "x"
        b = self.bounds(axis, visited)
        if b is None:
            return None
        if axis == "y":
            return [(cx, y, 0) for y in range(b[0], b[1])]
        return [(x, cy, 1) for x in range(b[0], b[1])]

    def accept_guess(self):
        """Надёжного чтения так и не было - берём ненадёжное."""
        if self.order_read is None and self.order_guess is not None:
            print("  ! надёжно метку не прочитали - беру ненадёжное чтение")
            self.saw_cognitive(self.order_guess, "запас")

    @property
    def order_known(self):
        return self.pending is not None or not self.cfg["order_from_cognitive"]

    def saw_black_ahead(self, cx, cy, d, cell):
        self.black_from[cell] = (cx, cy, d)
        self.saw(cell, "black")

    def reaction_for(self, cell):
        """Робот в центре cell: какую реакцию выполнить (spec) или None.
        Вызывающий выполняет её и зовёт done(cell)."""
        color = self.cell_color.get(cell)
        if color is None:
            return None
        self.visited_colors.add(color)
        spec = self.cfg["tiles"].get(color)
        if not spec or self.hurry:
            return None
        if spec.get("every_visit"):
            return spec
        if cell in self.reacted:
            return None
        pending = self.pending or []
        nxt = pending[0] if pending else None
        if self.relaxed and color in pending:
            return spec
        if color in pending or self.cfg["strict_order"]:
            if color != nxt:
                print(f"  ◇ {color} {cell} - не очередь (ждём {nxt or 'порядок'}), запомнил")
                return None
        return spec

    def done(self, cell):
        self.reacted.add(cell)
        color = self.cell_color.get(cell)
        if self.pending and color in self.pending:
            self.pending.remove(color)

    # ── маршруты ──
    def _route_to(self, color, here, bfs, skip_reacted):
        cells = self.known.get(color, set())
        if skip_reacted:
            cells = cells - self.reacted
        if not cells:
            return None
        if here in cells:
            return [], None
        if color != "black" or self.cfg["black_passable"]:
            path = bfs(lambda c: c in cells)
            return None if path is None else (path, None)
        # Чёрная в карте закрыта - BFS до клетки, откуда её видели.
        entries = {(x, y): d for b, (x, y, d) in self.black_from.items() if b in cells}
        path = bfs(lambda c: c in entries)
        if path is None:
            return None
        return (path, None) if path else ([], entries[here])

    def _finish_route(self, here, bfs):
        fin = self.cfg["finish"]
        if fin == "start":
            path = bfs(lambda c: c == (0, 0))
            return None if path is None else (path, None)
        return self._route_to(fin, here, bfs, skip_reacted=False)

    def _finish_ready(self):
        return set(self.cfg["finish_after"]) <= self.visited_colors and not self.pending

    def _out_of_time(self, route, elapsed):
        limit = self.cfg["time_limit_s"]
        if not limit or route is None:
            return False
        cells = len(route[0]) + (1 if route[1] is not None else 0)
        need = cells * self.cfg["sec_per_cell"] + self.cfg["time_margin_s"]
        return elapsed + need >= limit

    def plan(self, here, bfs, frontier, elapsed, visited=()):
        """-> (kind, path, black_dir). kind: 'explore' / 'target' / 'finish'.
        Пустой path и black_dir=None - уже на месте."""
        fin = self._finish_route(here, bfs)
        if not self.hurry and self._out_of_time(fin, elapsed):
            print(f"  ⏰ {elapsed:.0f}с - время на исходе, еду финишировать")
            self.hurry = True
        if self.hurry and fin is not None:
            return ("finish",) + fin

        if not self.order_known:
            # Метки - в клетках у старта: сначала их, иначе плитки по пути
            # пропадают зря, пока порядок неизвестен.
            # Сначала прямо: стартовый курс - N, "следующая после стартовой"
            # клетка - (0,1); правая рука BFS иначе свернула бы направо.
            for near in ({(0, 1)}, {(1, 0), (-1, 0), (0, -1)}):
                near = near - set(visited)
                path = bfs(lambda c: c in near)
                if path:
                    return "explore", path, None
            self.accept_guess()

        if not self.cfg["order_greedy"]:
            f = frontier()
            if f is not None:
                return "explore", f, None

        while self.pending:
            nxt = self.pending[0]
            r = self._route_to(nxt, here, bfs, skip_reacted=True)
            if r is not None:
                return ("target",) + r
            f = frontier()
            if f is not None:
                return "explore", f, None
            print(f"  ✗ {nxt} не найдена во всём доступном лабиринте - пропускаю, "
                  f"дальше без порядка")
            self.pending.pop(0)
            self.relaxed = True
            # Найденные раньше не по порядку - теперь можно.
            for color in self.pending:
                r = self._route_to(color, here, bfs, skip_reacted=True)
                if r is not None:
                    return ("target",) + r

        if not self._finish_ready() or self.cfg["explore_all"]:
            f = frontier()
            if f is not None:
                return "explore", f, None
        if fin is None:
            print(f"  [WARN] финиш '{self.cfg['finish']}' не найден - финиширую здесь")
            return "finish", [], None
        return ("finish",) + fin


# ══════════════════════════════════════════════════════════════
#  Исполнение на роботе (через nav)
# ══════════════════════════════════════════════════════════════
_real_tile_color = None


@contextmanager
def _black_passable():
    """drive_cells() бросает проезд на первом же чёрном показании - на время
    заезда В чёрную клетку подменяем ему датчик: чёрная читается белой."""
    def patched():
        t = _real_tile_color()
        return nav.TILE_WHITE if t == nav.TILE_BLACK else t
    nav.tile_color = patched
    try:
        yield
    finally:
        nav.tile_color = _real_tile_color


def _read_tile():
    """Цвет под роботом, стоящим в центре: tile_color() бывает '?' сразу
    после горки/наклона - ждём недолго внятного ответа."""
    end = time.monotonic() + COLOR_READ_S
    t = _real_tile_color()
    while t == nav.TILE_UNKNOWN and time.monotonic() < end:
        time.sleep(0.05)
        t = _real_tile_color()
    return t


def _blink(n, on_s, off_s, total_s):
    """n миганий вкл/выкл, потом просто стоим до total_s. По часам, а не
    суммой sleep - см. blink_pink в leds.py."""
    from leds import PINK
    px = nav._leds
    t0 = time.monotonic()
    end = t0 + total_s
    for _ in range(n):
        if nav.get_esp_paused():
            break
        if px is not None:
            px.fill(*PINK)
            px.show()
        time.sleep(on_s)
        if px is not None:
            px.clear()
            px.show()
        time.sleep(off_s)
    while time.monotonic() < end and not nav.get_esp_paused():
        time.sleep(min(0.05, end - time.monotonic()))


def _back_to_wall(cx, cy, hdg):
    """Спиной к ближайшей по повороту стене клетки (сзади - без поворота,
    сбоку - 90°, спереди - 180°), чтобы выданный комплект упёрся в неё и
    остался на плитке, а не укатился. Стен в карте нет - не крутимся.
    -> курс."""
    walls = nav._maze.get((cx, cy)) or [None] * 4
    for rel in (2, 1, 3, 0):                  # стена сзади, справа, слева, спереди
        d = (hdg + rel) % 4
        if walls[d] is True:
            back_hdg = (d + 2) % 4
            delta = nav.DIR_DELTA[(back_hdg - hdg) % 4]
            if delta:
                nav.turn(delta)
                if nav.get_esp_paused():
                    return hdg
                hdg = back_hdg
            nav.align_at_wall(nav.snap_target())   # "зад" - к стене, как у жертвы
            return hdg
    return hdg


def react(spec, what, cx=None, cy=None, hdg=None):
    """Остановиться/мигнуть, потом сбросить комплекты. Если передана клетка
    и курс - перед выдачей встаёт спиной к стене (см. _back_to_wall).
    -> (успели без паузы?, курс)."""
    n, on, off, total = signal_timing(spec)
    print(f"  ▓ {what}: {describe(spec)}")
    if total > 0:
        nav.hold_while(_blink, n, on, off, total + SIGNAL_MARGIN_S)
    kits = int(spec.get("kits", 0))
    if kits and hdg is not None and not nav.get_esp_paused():
        hdg = _back_to_wall(cx, cy, hdg)
    for _ in range(kits):
        if nav.get_esp_paused():
            return False, hdg
        nav.dispense_kit()
    return not nav.get_esp_paused(), hdg


_letter_walls = set()


def look_for_letters(m, cx, cy, hdg, walls_rel):
    """Буквы на боковых стенах клетки - камерами детекторов, без YOLO."""
    import cv2
    import letters_cv
    for side, det in (("right", nav._victim_r), ("left", nav._victim_l)):
        wall_dir = (hdg + (1 if side == "right" else 3)) % 4
        key = (cx, cy, wall_dir)
        if det is None or not walls_rel.get(side) or key in _letter_walls:
            continue
        _letter_walls.add(key)
        det.cam.latest()                      # сброс кадра, снятого на ходу
        frame = det.cam.latest()
        found = letters_cv.find_letter(frame)
        if not found:
            continue
        letter, (x, y, w, h) = found
        fname = f"letter_{cx}_{cy}_{letter}_{time.strftime('%H-%M-%S')}.jpg"
        shown = frame.copy()
        cv2.rectangle(shown, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imwrite(os.path.join(os.path.expanduser("~/maze/detections"), fname), shown)
        nav._victim_log.append({"type": letter, "label": letter, "cell": (cx, cy),
                                "side": side, "wall_dir": wall_dir, "image": fname,
                                "ts": time.strftime("%H:%M:%S")})
        spec = m.cfg["letters"].get(letter)
        print(f"  ★ буква {letter} ({side}) в ({cx},{cy})")
        if spec:
            react(spec, f"буква {letter}")    # буквам - без разворота (курс камер)


_cog_seen = 0   # сколько записей _victim_log уже разобрано на когнитивные
# Метки, которые детектор нашёл, но кольца счёл ненадёжными и вернул None:
# [((x, y, стена), [цвета])]. Сам victim_detect их не отдаёт, поэтому
# ловим через подмену ring_colors.checksum - см. _wrap_detector().
_unread_cog = []
_ring_ctx = threading.local()
_cog_walls = set()   # (x, y, стена), где видели когнитивную метку - любое чтение


def _wrap_detector(det):
    """det.scan, который запоминает кольца ненадёжного чтения. Сканы обеих
    камер идут параллельно, поэтому кольца - в threading.local своего потока."""
    orig = det.scan

    def scan(cell_x, cell_y, wall_dir=None):
        _ring_ctx.rings = None
        res = orig(cell_x, cell_y, wall_dir)
        if res is None and _ring_ctx.rings:
            _unread_cog.append(((cell_x, cell_y, wall_dir), _ring_ctx.rings))
        return res
    det.scan = scan


def _patch_checksum():
    import ring_colors as rc
    orig = rc.checksum

    def checksum(rings):
        _ring_ctx.rings = [r["color"] for r in rings]
        return orig(rings)
    rc.checksum = checksum


def _take_cognitive(m):
    """Новые записи сканов -> кольца когнитивных меток в Mission. Читаем лог,
    а не результат report_victim: метка с "фейковой" суммой колец туда не
    попадает, а порядок нам нужен из любой."""
    global _cog_seen
    for r in nav._victim_log[_cog_seen:]:
        if r.get("class") == "cognetive":
            cell = tuple(r.get("cell", ()))
            _cog_walls.add(cell + (r.get("wall_dir"),))
            m.saw_cognitive(r.get("rings"), cell)
    _cog_seen = len(nav._victim_log)
    while _unread_cog:
        key, rings = _unread_cog.pop(0)
        _cog_walls.add(key)
        m.saw_cognitive(rings, key[:2], reliable=False)


def _report_victim(result, side, wall_dir, hdg):
    """report_victim из nav.py, но на когнитивные метки - ничего: ни
    мигания, ни комплектов (по условию они только задают порядок)."""
    if result.get("class") == "cognetive":
        return hdg
    return _nav_report_victim(result, side, wall_dir, hdg)


_nav_report_victim = None


def _rescan_wall(m, cx, cy, hdg, d):
    """Ещё раз снять стену d этой клетки. Если она уже сбоку - без поворота
    (камеры смотрят только вбок), иначе поворот на 90°. -> курс."""
    if d not in ((hdg + 1) % 4, (hdg + 3) % 4):
        nav.turn(90)
        if nav.get_esp_paused():
            return hdg
        hdg = (hdg + 1) % 4
        nav.align_at_wall(nav.snap_target())
    # Детекторы и nav помнят, что стену уже смотрели, и второй раз не
    # сканируют - забываем только её (стены с буквами не трогаем: иначе
    # букву засчитаем и выдадим на неё второй раз).
    nav._scanned_walls.discard((cx, cy, d))
    for det in (nav._victim_r, nav._victim_l):
        if det is not None:
            det.scanned.discard((cx, cy, d))
    hdg = nav.scan_cell(cx, cy, hdg)
    _take_cognitive(m)
    return hdg


def read_order_here(m, cx, cy, hdg):
    """Клетка рядом со стартом, порядок ещё неизвестен (боковые стены уже
    сняты при приезде). Метку видели, но ненадёжно - переснимаем ЕЁ стену
    (она сбоку - без поворота), до двух раз. Не видели вовсе - она на
    передней/задней стене: поворот на 90° и скан. Надёжного так и нет -
    берём ненадёжное. -> курс."""
    def seen_here():
        return [w for w in _cog_walls if w[:2] == (cx, cy) and w[2] is not None]

    if not seen_here():
        nav.turn(90)
        if nav.get_esp_paused():
            return hdg
        hdg = (hdg + 1) % 4
        nav.align_at_wall(nav.snap_target())
        hdg = nav.scan_cell(cx, cy, hdg)
        _take_cognitive(m)
    for _ in range(2):
        walls = seen_here()
        if m.order_known or not walls or nav.get_esp_paused():
            break
        print(f"  … метка в ({cx},{cy}) прочиталась ненадёжно - снимаю её ещё раз")
        hdg = _rescan_wall(m, cx, cy, hdg, walls[0][2])
    m.accept_guess()
    return hdg


def arrive(m, cx, cy, hdg, checkpoint, new_cell):
    """Стоим в центре (cx,cy): стены в карту, цвет, реакция, камеры.
    -> (checkpoint, курс): скан с выдачей комплекта разворачивает робота."""
    nav._write_state(cx, cy, hdg, checkpoint, "challenge")
    walls_rel = nav._measure_cell(hdg)
    nav.set_cell_walls(cx, cy, hdg, walls_rel)
    nav._visited.add((cx, cy))
    if nav.get_esp_paused():
        return checkpoint, hdg
    tile = _read_tile()
    m.saw((cx, cy), tile)
    if tile == nav.CHECKPOINT_TILE and checkpoint[:2] != (cx, cy):
        checkpoint = (cx, cy, hdg)
        print(f"  ✓ ЧЕКПОИНТ ({cx},{cy})")
    spec = m.reaction_for((cx, cy))
    if spec is not None:
        ok, hdg = react(spec, f"{m.cell_color[(cx, cy)]} ({cx},{cy})", cx, cy, hdg)
        if ok:
            m.done((cx, cy))
    if new_cell and not m.hurry:
        if m.cfg["victims"]:
            hdg = nav.scan_cell(cx, cy, hdg, walls_rel)
            _take_cognitive(m)
        if m.cfg["letters"]:
            look_for_letters(m, cx, cy, hdg, walls_rel)
    # Метка - в клетке ПОСЛЕ стартовой; на самом старте не крутимся.
    if (not m.order_known and m.cfg["victims"]
            and abs(cx) + abs(cy) == 1 and not nav.get_esp_paused()):
        hdg = read_order_here(m, cx, cy, hdg)
        # Порядок узнали - плитка под нами может оказаться первой по очереди.
        if m.order_known:
            spec = m.reaction_for((cx, cy))
            if spec is not None:
                ok, hdg = react(spec, f"{m.cell_color[(cx, cy)]} ({cx},{cy})", cx, cy, hdg)
                if ok:
                    m.done((cx, cy))
    nav._write_state(cx, cy, hdg, checkpoint, "challenge")
    return checkpoint, hdg


class _Tee:
    """stdout, который ещё и запоминает напечатанное - чтобы узнать, что
    drive_cells() сам решил про наклон (он только печатает об этом)."""
    def __init__(self, out):
        self.out, self.buf = out, []

    def write(self, text):
        self.buf.append(text)
        return self.out.write(text)

    def flush(self):
        self.out.flush()


STAIRS_MARK = "лестница/препятствие"   # вывод drive_cells: наклон был, но не горка


def _open_stairs_line(m, cx, cy, d):
    """Линия с лестницей по условию без стен: открываем её рёбра по факту
    (_edge_fixed), голоса лидара их больше не закроют."""
    if nav.level_of(cy) != 0:
        return
    edges = m.stairs_line(cx, cy, d, nav._visited)
    if edges is None:
        print("  ≡ лестница, но края лабиринта неизвестны - открываю только это ребро")
        edges = [(cx, cy, d)]
    for x, y, e in edges:
        _force_open(x, y, e)
        nav._blocked.discard((x, y))
    line = f"x={cx}" if d in (0, 2) else f"y={cy}"
    print(f"  ≡ ЛЕСТНИЦА/ГОРКА: линия {line} открыта целиком ({len(edges)} рёбер)")


RAMP_FIX_MAX_MM = 1300.0   # дальше стена в конце линии - по ней не уточняем


def _force_open(x, y, e):
    nav._edge_fixed[nav._edge_key(x, y, e)] = False
    nav._set_wall(x, y, e, False)
    nav._blocked.discard(nav._next_cell(x, y, e))


def _ramp_arrival(m, cx, cy, d):
    """Съехали с горки, проехав её из (cx,cy) по d (single_level). Где мы:
    k клеток вперёд в плане по замеру горки (nav._ramp_measure), а если край
    лабиринта по этой оси известен и стена в конце линии недалеко -
    по расстоянию до неё (линия по условию без стен, значит спереди край).
    Плитки горки - обычные клетки: проехали по ним, рёбра открыты. -> клетка."""
    info = nav._last_ramp
    k = info["cells"] if info else 1
    axis_i = 1 if d in (0, 2) else 0
    sgn = 1 if d in (0, 1) else -1
    b = m.bounds("y" if axis_i else "x", nav._visited)
    nav.wait_scan(2, timeout=1.0)
    f = nav.get_walls().get("front")
    if b is not None and f is not None and f < RAMP_FIX_MAX_MM:
        ahead = int(round((f - nav.FRONT_CENTER_MM) / nav.CELL_MM))
        end = b[1] if sgn > 0 else b[0]
        k_wall = (end - sgn * ahead - (cx, cy)[axis_i]) * sgn
        if k_wall >= 1 and abs(k_wall - k) <= 2:
            if k_wall != k:
                print(f"    горка: по стене спереди ({f:.0f}мм) не {k}, а {k_wall} кл.")
            k = k_wall
    dx, dy = nav._DX[d], nav._DY[d]
    for i in range(k):
        _force_open(cx + dx * i, cy + dy * i, d)
        if i:
            nav._visited.add((cx + dx * i, cy + dy * i))
    dest = (cx + dx * k, cy + dy * k)
    print(f"  ⛰ ГОРКА (один этаж): ({cx},{cy}) → {dest}, {k} кл.")
    if m.cfg["stairs_line_open"]:
        _open_stairs_line(m, cx, cy, d)
    return dest


def _unblock(cx, cy, d):
    """Заехали в чёрную - ребро к ней открыто по факту. Без этого карта
    держит его закрытым (block_direction), и из тупика с чёрной BFS не
    выпустит. Если потом BFS поведёт через неё снова, drive_cells просто
    опять откатится от чёрной и закроет ребро."""
    nav._blocked.discard(nav._next_cell(cx, cy, d))
    nav._edge_fixed.pop(nav._edge_key(cx, cy, d), None)


_stall_fails = {}   # (x, y, курс) -> сколько раз подряд не взяли препятствие/горку


def step(m, cx, cy, hdg, d, into_black=False):
    """Шаг в соседнюю клетку по направлению d -> (cx, cy, hdg, приехали?).
    приехали=None - не ехали, но и не застряли: скан после поворота выдал
    комплект и развернул робота, маршрут надо строить заново.
    Реакции на неудачный проезд - как в return_to_start() nav.py."""
    delta = nav.DIR_DELTA[(d - hdg) % 4]
    if delta:
        # Как в flood_fill_run: 180° - двумя по 90° со сканом посередине,
        # иначе передняя стена тупика остаётся непросмотренной.
        steps = (90, 90) if abs(delta) == 180 else (delta,)
        for i, st in enumerate(steps):
            ok = nav.turn(st)
            if nav.get_esp_paused():
                return cx, cy, hdg, False
            if not ok:
                # Не доворачивал - курс с гироскопа, маршрут заново (как None ниже).
                return cx, cy, nav.gyro_heading(), None
            hdg = (hdg + st // 90) % 4
            if i == len(steps) - 1:
                nav.align_at_wall(nav.snap_target())
            if m.cfg["victims"] and not m.hurry:
                h = nav.scan_cell(cx, cy, hdg)
                _take_cognitive(m)
                if h != hdg or nav.get_esp_paused():
                    return cx, cy, h, None
    tee = _Tee(sys.stdout)
    with _black_passable() if into_black or m.cfg["black_passable"] else nullcontext(), \
            redirect_stdout(tee):
        n, reason = nav.drive_cells(1, expect_ramp=(cx, cy, hdg) in nav._ramps)
    if m.cfg["stairs_line_open"] and STAIRS_MARK in "".join(tee.buf):
        _open_stairs_line(m, cx, cy, hdg)
    if n > 0:
        _stall_fails.pop((cx, cy, hdg), None)
        if into_black:
            _unblock(cx, cy, hdg)
        if reason == "ramp" and m.cfg["single_level"]:
            cx, cy = _ramp_arrival(m, cx, cy, hdg)
            return cx, cy, hdg, True
        if reason != "ramp":
            nav.open_edge(cx, cy, hdg)
        cx, cy = nav._arrived_cell(cx, cy, hdg, reason)
        return cx, cy, hdg, True

    if reason == "black":
        cell = nav._next_cell(cx, cy, hdg)
        print(f"  ■ чёрная {cell} - запомнил, откуда заезжать")
        m.saw_black_ahead(cx, cy, hdg, cell)
        nav.block_direction(cx, cy, hdg)
    elif reason in ("obstacle_stall", "ramp_stall"):
        # Как в flood_fill_run: не взял - откат и ещё OBSTACLE_RETRIES попыток,
        # только потом ребро непроезжее. Раньше здесь закрывали с первого
        # раза, и бугорок, который робот не взял с ходу, отрезал пол-лабиринта.
        if reason == "obstacle_stall":
            nav.creep_mm(-nav.OBSTACLE_BACKOFF_MM, nav.snap_target())
        key = (cx, cy, hdg)
        _stall_fails[key] = _stall_fails.get(key, 0) + 1
        if _stall_fails[key] <= nav.OBSTACLE_RETRIES:
            print(f"    не взял ({_stall_fails[key]}/{nav.OBSTACLE_RETRIES}) - повтор")
            return cx, cy, hdg, None
        print("    не взял и с повторами - ребро непроезжее")
        nav.block_direction(cx, cy, hdg)
    elif reason in ("front_obstacle", "wall_close"):
        nav.close_edge(cx, cy, hdg)
    elif reason == "bumper_both":
        nav.creep_mm(-nav.BUMPER_BACKOFF_MM, nav.snap_target())
        nav.block_direction(cx, cy, hdg)
    elif reason in ("bumper_left", "bumper_right"):
        base_yaw = nav.snap_target()
        turn_deg = -45 if reason == "bumper_left" else 45
        nav.turn(turn_deg)
        if not nav.get_esp_paused():
            nav.creep_mm(-nav.BUMPER_SIDE_BACKOFF_MM, nav.get_yaw())
        if not nav.get_esp_paused():
            nav.turn_to(base_yaw)
        nav.block_direction(cx, cy, hdg)
    return cx, cy, hdg, False


def finish(m, cx, cy, hdg, checkpoint):
    cell = (cx, cy)
    spec = m.cfg["tiles"].get(m.cell_color.get(cell))
    if spec and (spec.get("every_visit") or cell not in m.reacted):
        react(spec, f"финиш {m.cell_color[cell]}", cx, cy, hdg)
        m.done(cell)
    print(f"  ● ФИНИШ ({cx},{cy})")
    nav._write_state(cx, cy, hdg, checkpoint, "blinking")
    if nav._leds is not None:
        # Как в nav.py на старте после возврата: 1с/1с не меньше 10с, на тормозе.
        nav.blink_start_return()
    else:
        nav.hold_while(time.sleep, m.cfg["finish_hold_s"])
    nav.stop()
    nav._write_state(cx, cy, hdg, checkpoint, "done")


def _forget_everything(m):
    """Всё, что робот узнал за попытку: карта, стены, горки, метки, порядок,
    плитки, выданные комплекты (столбики заряжают заново)."""
    global _cog_seen
    for c in (nav._maze, nav._visited, nav._blocked, nav._wall_votes,
              nav._edge_fixed, nav._ramps, nav._scanned_walls, nav._blue_cells,
              _unread_cog, _cog_walls, _letter_walls, _stall_fails):
        c.clear()
    del nav._victim_log[:]
    nav._last_ramp = None
    for side in nav._kits_used:
        nav._kits_used[side] = 0
    for det in (nav._victim_r, nav._victim_l):
        if det is not None:
            det.scanned.clear()
            del det.log[:]
    _cog_seen = 0
    m.__init__(m.cfg)


def restart_attempt(m):
    """Сон кнопкой BTN_1 = попытка заново: не чекпоинт, как в nav.py, а
    старт с чистого листа. Ждём кнопку старта, сетку поля берём от того,
    как робот стоит (как при первом старте)."""
    nav.stop()
    _forget_everything(m)
    print("\n  ✋ ПОПЫТКА ЗАНОВО - всё забыто. Поставьте робота на СТАРТ "
          "тем же курсом, что в начале, зарядите комплекты и нажмите кнопку старта")
    nav._write_state(0, 0, 0, (0, 0, 0), "sleep")
    nav.leds_sleep(True)
    nav.wait_start_button()
    nav.leds_sleep(False)
    time.sleep(nav.RESTART_SETTLE_S)
    nav.YAW_ORIGIN = nav.get_yaw()
    nav.zero_odo()
    time.sleep(0.1)
    nav.resume_esp()
    print(f"  ▶ Новая попытка, YAW_ORIGIN={nav.YAW_ORIGIN:.1f}°")


def run(m):
    while True:
        res = _attempt(m)
        if res is not None:
            return res
        restart_attempt(m)


def _attempt(m):
    """Одна попытка с клетки старта. -> (cx, cy, hdg) по окончании или None,
    если её прервали сном (тогда run() начинает новую с чистого листа)."""
    cx, cy, hdg = 0, 0, 0
    checkpoint = (0, 0, 0)
    t0 = time.monotonic()
    checkpoint, hdg = arrive(m, cx, cy, hdg, checkpoint, True)
    m.set_start_walls(nav._maze.get((0, 0)))
    stalls = 0
    while True:
        if nav.get_esp_paused():
            return None
        if not nav.telem_fresh():
            t_wait = time.time()
            while not nav.telem_fresh():
                if time.time() - t_wait > 5.0:
                    print("  [FATAL] связь с ESP потеряна")
                    return cx, cy, hdg
                time.sleep(0.05)
            continue

        here = (cx, cy)
        kind, path, black_dir = m.plan(
            here,
            bfs=lambda goal: nav.find_path_bfs(here, goal, hdg),
            frontier=lambda: nav.find_path_bfs(
                here, lambda c: c not in nav._visited and m.in_bounds(c, nav._visited), hdg),
            elapsed=time.monotonic() - t0,
            visited=nav._visited)

        if not path and black_dir is None:
            if kind == "finish":
                print(f"  ⏱ {time.monotonic() - t0:.0f}с до финиша")
                finish(m, cx, cy, hdg, checkpoint)
                return cx, cy, hdg
            # Стоим на цели, до которой только что дошла очередь.
            spec = m.reaction_for(here)
            ok = False
            if spec is not None:
                ok, hdg = react(spec, f"{m.cell_color[here]} {here}", cx, cy, hdg)
            if ok:
                m.done(here)
            else:
                m.reacted.add(here)    # больше сюда не звать
            continue

        into_black = not path
        d = black_dir if into_black else path[0]
        print(f"─── ({cx},{cy}) → {nav._DIR_NAME[d]}  [{kind}"
              f"{', в чёрную' if into_black else ''}]  {time.monotonic() - t0:.0f}с")
        new_cx, new_cy, hdg, ok = step(m, cx, cy, hdg, d, into_black)
        if ok is None:
            continue
        if ok:
            stalls = 0
            new_cell = (new_cx, new_cy) not in nav._visited
            cx, cy = new_cx, new_cy
            if into_black:
                m.saw((cx, cy), "black")   # датчик тут подменён не был, но цвет известен
            checkpoint, hdg = arrive(m, cx, cy, hdg, checkpoint, new_cell)
        else:
            stalls += 1
            if stalls >= MAX_STALLS:
                print("  [WARN] застряли - финиширую где стою")
                finish(m, cx, cy, hdg, checkpoint)
                return cx, cy, hdg


def boot(cfg):
    """То же, что начало nav.main(): связь, лидар, лента, камеры, кнопка."""
    import serial
    global _real_tile_color
    _real_tile_color = nav.tile_color
    nav.PER_CELL_STOP = True
    nav.PUSH_THROUGH_SMALL = cfg["push_through_small"]
    try:
        nav._link = serial.Serial(nav.LINK_PORT, nav.LINK_BAUD, timeout=0.1)
    except serial.SerialException as e:
        raise SystemExit(f"[FATAL] Порт {nav.LINK_PORT}: {e}")
    threading.Thread(target=nav._telem_thread, daemon=True).start()
    threading.Thread(target=nav._lidar_thread, daemon=True).start()
    t = time.time()
    while not nav._state["telem_ok"]:
        if time.time() - t > 5:
            raise SystemExit("[FATAL] нет телеметрии с ESP\n        " + nav.link_report())
        time.sleep(0.05)

    # Лента нужна всегда - в nav.py её поднимает только ветка со зрением.
    try:
        nav._leds = nav.NeoPixels()
    except Exception as e:
        print(f"  [WARN] лента недоступна: {e} - мигать нечем!")
    global _nav_report_victim
    _patch_checksum()
    _nav_report_victim = nav.report_victim
    nav.report_victim = _report_victim          # scan_sides зовёт его по имени
    if cfg["victim_kits"] is not None:
        for t in nav.KITS_BY_TYPE:
            nav.KITS_BY_TYPE[t] = cfg["victim_kits"]
    if cfg["victims"] or cfg["letters"]:
        for idx, attr in ((0, "_victim_r"), (1, "_victim_l")):
            try:
                setattr(nav, attr, nav.VictimDetector(cam_idx=idx))
                _wrap_detector(getattr(nav, attr))
            except Exception as e:
                print(f"  [WARN] камера {idx} недоступна: {e}")

    nav.wait_start_button()
    time.sleep(nav.RESTART_SETTLE_S)
    nav.YAW_ORIGIN = nav.get_yaw()
    if nav.get_esp_paused():
        nav.resume_esp()
    nav.wait_scan(3, timeout=5.0)
    if nav._scan_cnt == 0:
        raise SystemExit("[FATAL] лидар молчит")
    nav.load_tile_calib()
    nav.calibrate_white()
    nav.zero_odo()
    time.sleep(0.1)
    nav._lifted.clear()


def shutdown():
    nav.brake()
    time.sleep(0.1)
    nav.stop()
    nav._link.close()
    nav._lidar_stop.set()
    if nav._lidar_proc is not None and nav._lidar_proc.poll() is None:
        nav._lidar_proc.terminate()
    for det in (nav._victim_r, nav._victim_l):
        if det is not None:
            det.close()
    if nav._leds is not None:
        nav._leds.close()
    nav.GPIO.cleanup()


def parse_args(argv):
    name, path, show, over = None, None, False, {}
    it = iter(argv)
    for a in it:
        if a == "--cfg":
            path = next(it)
        elif a == "--time":
            over["time_limit_s"] = float(next(it))
        elif a == "--sec-per-cell":
            over["sec_per_cell"] = float(next(it))
        elif a == "--order":
            over["order"] = [c for c in next(it).split(",") if c]
        elif a == "--finish":
            over["finish"] = next(it)
        elif a == "--victims":
            over["victims"] = True
        elif a == "--no-web":
            over["web"] = False
        elif a == "--show":
            show = True
        elif a.startswith("-"):
            raise SystemExit(f"не знаю ключ {a}\n{__doc__}")
        else:
            name = a
    if name is None:
        raise SystemExit(f"какой пресет? есть: {', '.join(PRESETS)}\n{__doc__}")
    return name, path, show, over


def main():
    global nav
    name, path, show, over = parse_args(sys.argv[1:])
    cfg = load_config(name, path, over)
    print(f"═══ Испытание '{name}' ═══")
    print_config(cfg)
    if show:
        return
    # The old implementation below predates nav-split and accesses globals
    # no longer exported by nav.py. Dispatch to the current actions/boot tasks.
    if path or any(k not in ("time_limit_s", "web") for k in over):
        raise SystemExit("Этот старый JSON-интерфейс не совместим с nav-split. "
                         "Используйте task_template.py или отредактируйте task_*.py; "
                         "--show доступен без железа.")
    import importlib
    import boot as runtime_boot
    task = importlib.import_module({"tomsk": "task_tomsk", "colors": "task_colors", "cog": "task_cog"}[name])
    if "time_limit_s" in over:
        if not hasattr(task, "LIMIT_S"):
            raise SystemExit("Этот сценарий не задаёт LIMIT_S; добавьте Timer в task_colors.py")
        task.LIMIT_S = over["time_limit_s"]
    runtime_boot.run(task.scenario, args=[] if cfg["web"] else ["--no-web"], cameras=name != "tomsk")


if __name__ == "__main__":
    main()
