"""Глаголы для заданий технического испытания. Задание - файл task_*.py,
который читается как лист с условием: boot.start() → глаголы → boot.stop().

Шпаргалка (w - робот на поле, explore.Walk: w.cell, w.hdg, w.checkpoint):

  Стоя (всё время на тормозе):
    blink(n, on_s, off_s, color)    мигнуть n раз (по умолчанию розовым 0,5/0,5 с)
    blink_victim()                  мигание как у жертвы: розовым 0,5/0,5 с, 5,3 с
    blink_finish()                  мигание как на старте: 1/1 с, 10,5 с
    stand(s)                        стоять s секунд на тормозе
    stand_motors_off(s)             стоять s секунд с выключенными моторами (финиш)
    give_kits(w, n, wall_dir=None)  выдать n комплектов: на месте или спиной к стене

  Ехать:
    new_walk()                      робот на старте (0,0), курс 0
    look_around(w)                  осмотр клетки, как в разведке: стены, плитка, камеры
    explore_until(w, done)          разведка, пока done() не вернёт True
    go_to_cell(w, cell)             доехать до клетки и встать в ней
    go_to_color(w, color)           доехать до ближайшей известной клетки цвета
    go_to_nearest(w, cells)         доехать до ближайшей (по пути) из клеток
    nearest(w, cells)               какая из клеток ближе всех по пути
    visit_targets(w, targets, work, done, stop)
                                    по очереди: к ближайшей из targets() и work(клетка);
                                    целей не видно - разведка, пока не появятся
    (arrive=... - что делать по приезде в каждую клетку; по умолчанию
     разведка - explore.arrive_explore, поездки - explore.arrive_home)

  Время:
    Timer(limit_s)                  часы задания от создания
      .elapsed() / .left()          сколько прошло / осталось, с
      .trip_s(w, cell)              оценка пути до клетки, с
      .time_to_go(w, cell, extra_s) пора ехать: осталось только на дорогу (+extra_s)

  Что найдено:
    cells_of(color)                 клетки этого цвета по карте цветов
    marks_found(classes)            прочитанные метки (state.victim_log)

Цвета плиток и реакции на них - tile_rules.RULES, классы меток-жертв и
комплекты за тип - config (см. config.VICTIM_CLASSES, config.KITS_BY_TYPE).
"""
from __future__ import annotations

import time

import leds

import state
import dash_live
import esp_link
import explore
import maze_map
import tile_rules
import victims


# ══════════════════════════════════════════════════════════════
#  Стоя
# ══════════════════════════════════════════════════════════════
def blink(n, on_s=0.5, off_s=0.5, color=None):
    """Мигнуть n раз лентой на тормозе. Ленты нет - простоять то же время."""
    print(f"  ● мигание {n} раз ({on_s:g}/{off_s:g} с)")
    if state.leds is None:
        stand(n * (on_s + off_s))
        return
    esp_link.hold_while(leds.blink_n, state.leds, n, on_s, off_s,
                        leds.PINK if color is None else color, stop=esp_link.get_esp_paused)


def blink_victim():
    """Мигание как при находке жертвы (leds.blink_pink)."""
    if state.leds is None:
        stand(leds.VICTIM_STOP_S + leds.MARGIN_S)
        return
    esp_link.hold_while(leds.blink_pink, state.leds, stop=esp_link.get_esp_paused)


def blink_finish():
    """Мигание как на старте после возврата (dash_live.blink_start_return)."""
    dash_live.blink_start_return()


def stand(seconds):
    """Стоять на тормозе. Удержание на ESP само отпускает через 3 с, поэтому
    тормоз подтверждается каждые полсекунды (как в blue_stop)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end and not esp_link.interrupted():
        esp_link.brake()
        time.sleep(min(0.5, max(0.0, end - time.monotonic())))


def stand_motors_off(seconds):
    """Финиш: моторы выключены, стоим seconds секунд."""
    print(f"  ■ финиш: моторы выключены, стоим {seconds:g} с")
    esp_link.stop()
    time.sleep(seconds)


def give_kits(w, n, wall_dir=None):
    """Выдать n комплектов: на месте (wall_dir=None) или спиной к стене
    wall_dir, как у жертвы. Курс робота после выдачи - в w.hdg."""
    print(f"  ▼ комплектов: {n}")
    w.hdg = victims.give_kits(n, wall_dir, w.hdg)
    w.show()


# ══════════════════════════════════════════════════════════════
#  Ехать
# ══════════════════════════════════════════════════════════════
def new_walk(phase="exploring"):
    """Робот на старте (0,0), курс 0. Дальше - look_around(w): без осмотра
    стартовой клетки в карте нет стен, и разведке некуда ехать."""
    return explore.Walk(phase=phase)


def look_around(w):
    """Осмотр клетки, где стоим, как новой клетки в разведке: стены, плитка
    (реакция по tile_rules), камеры."""
    explore.arrive_explore(w, True)


def explore_until(w, done, arrive=None):
    """Разведка, пока done() не вернёт True. True - условие выполнено,
    False - разведывать больше нечего (или потолок клеток)."""
    arrive = arrive or explore.arrive_explore
    while not done():
        if w.total >= w.max_cells:
            return False
        if explore.step(w, explore.unvisited, arrive=arrive) == "no_path":
            return False
    return True


def go_to_cell(w, cell, arrive=None):
    """Доехать до клетки по карте и встать в ней."""
    print(f"  → едем в клетку {cell}")
    explore.go_to(w, lambda c: c == cell, arrive or explore.arrive_home)


def go_to_color(w, color, arrive=None):
    """Доехать до ближайшей (по пути) известной клетки цвета color.
    Возвращает эту клетку или None, если таких клеток на карте нет."""
    return go_to_nearest(w, cells_of(color), arrive)


def go_to_nearest(w, cells, arrive=None):
    """Доехать до ближайшей по пути из клеток cells. Возвращает её или None,
    если список пуст."""
    if not cells:
        return None
    cell = nearest(w, cells)
    if cell is None:
        return None
    go_to_cell(w, cell, arrive)
    return cell if w.cell == cell else None


def visit_targets(w, targets, work, done, stop):
    """Пока не done() и не stop(): известна цель (targets() не пуст) - ехать
    к ближайшей и work(клетка); не известна - разведка, пока не появится.
    Разведывать больше нечего - выход."""
    while not done() and not stop():
        cells = targets()
        if cells:
            reached = go_to_nearest(w, cells)
            if reached is None:
                return False
            work(reached)
        elif not explore_until(w, lambda: done() or bool(targets()) or stop()):
            return


def nearest(w, cells):
    """Reachable target with the lowest route cost, including blue visits.

    One multi-goal search replaces a separate search for every candidate.
    An unreachable target must not win by its straight-line distance.
    """
    targets = set(cells)
    if not targets:
        return None
    path = maze_map.find_path_bfs(w.cell, targets.__contains__, w.hdg)
    if path is None:
        return None
    cell = w.cell
    for direction in path:
        cell = maze_map._next_cell(*cell, direction)
    return cell


# ══════════════════════════════════════════════════════════════
#  Время
# ══════════════════════════════════════════════════════════════
SEC_PER_CELL = 4.0       # оценка на клетку, пока своих замеров мало
MIN_CELLS_MEASURED = 3   # с этого числа клеток верим своему замеру
TRIP_MARGIN = 1.3        # запас к оценке пути: горки, доводки, повороты
# Из каких статей хронометража (timing._TIME_ORDER) складывается дорога.
TRAVEL_PARTS = ("ход", "поворот", "доводка", "лидар")


def sec_per_cell(total_cells, tacc):
    """Среднее время дороги на клетку по хронометражу заезда: только езда,
    повороты, доводка и лидар - без камер, жертв и стоянок."""
    if total_cells < MIN_CELLS_MEASURED:
        return SEC_PER_CELL
    return sum(tacc.get(p, 0.0) for p in TRAVEL_PARTS) / total_cells


def path_cells(w, cell):
    """Сколько клеток пути до cell по карте. Пути нет - по прямой
    (|dx| + |dy|): оценка всё равно нужна, решать робот должен всегда."""
    if cell == w.cell:
        return 0
    path = maze_map.find_path_bfs(w.cell, lambda c: c == cell, w.hdg)
    if path is not None:
        return len(path)
    return abs(cell[0] - w.cx) + abs(cell[1] - w.cy)


class Timer:
    """Часы задания: отсчёт от создания, лимит limit_s секунд."""

    def __init__(self, limit_s, clock=time.monotonic):
        self.limit_s = limit_s
        self.clock = clock
        self.t0 = clock()

    def elapsed(self):
        return self.clock() - self.t0

    def left(self):
        return self.limit_s - self.elapsed()

    def trip_s(self, w, cell):
        """Оценка дороги до клетки, с: клеток пути × среднее на клетку × запас."""
        import config
        path = maze_map.find_path_bfs(w.cell, lambda c: c == cell, w.hdg)
        # A disconnected route has no credible time estimate. Return immediately
        # rather than pretending the Manhattan distance is a drivable route.
        if path is None:
            return float("inf")
        estimate = len(path) * sec_per_cell(w.total, state.tacc) * TRIP_MARGIN
        if path is not None and tile_rules.role(config.TILE_BLUE) == tile_rules.STOP:
            pos = w.cell
            for d in path:
                pos = maze_map._next_cell(*pos, d)
                if pos in state.blue_cells or state.tile_map.get(pos) == config.TILE_BLUE:
                    estimate += config.BLUE_WAIT_S
        return estimate

    def time_to_go(self, w, cell, extra_s=0.0):
        """Пора ехать в cell: времени осталось только на дорогу и extra_s
        (стоянка на финише, мигание)."""
        return self.left() <= self.trip_s(w, cell) + extra_s


# ══════════════════════════════════════════════════════════════
#  Что найдено
# ══════════════════════════════════════════════════════════════
def cells_of(color):
    """Клетки этого цвета по карте цветов (state.tile_map)."""
    return tile_rules.cells_of(color)


def marks_found(classes=None):
    """Прочитанные метки из state.victim_log; classes - только эти классы
    YOLO (например {"H", "S"}), None - все."""
    return [r for r in state.victim_log if classes is None or r.get("class") in classes]

