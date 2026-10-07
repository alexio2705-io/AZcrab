"""Маршрут по карте клеток - BFS из nav.find_path_bfs, вынесенный сюда, чтобы
решение "прямо или стоп" на ходу (drive_cells с планом) проверялось тестами
без железа: nav.py на Маке не импортируется (RPi.GPIO, serial)."""
from __future__ import annotations

from collections import deque


def bfs_path(start, hdg, open_dirs, next_cell, goal_test, side_order):
    """Кратчайший (по числу клеток) путь от start по открытым рёбрам до
    ближайшей клетки, для которой goal_test() верно.

    Из равных по длине путей выигрывает тот, что раньше сворачивает в
    приоритетную сторону: соседи каждой клетки перебираются по side_order
    относительно курса, которым в неё въехали (для start - hdg).
    open_dirs(cx, cy) - открытые направления клетки, next_cell(cx, cy, d) -
    куда ведёт ребро.

    Возвращает список направлений (0..3, по одному на клетку пути) или None,
    если ни одна подходящая клетка не достижима."""
    if goal_test(start):
        return []
    seen = {start}
    prev = {}
    q = deque([(start, hdg)])
    while q:
        cell, h = q.popleft()
        dirs = open_dirs(cell[0], cell[1])
        for d in ((h + k) % 4 for k in side_order):
            if d not in dirs:
                continue
            nxt = next_cell(cell[0], cell[1], d)
            if nxt in seen:
                continue
            seen.add(nxt)
            prev[nxt] = (cell, d)
            if goal_test(nxt):
                path = [d]
                back = cell
                while back != start:
                    back, dd = prev[back]
                    path.append(dd)
                path.reverse()
                return path
            q.append((nxt, d))
    return None


def go_straight(start, hdg, open_dirs, next_cell, goal_test, side_order):
    """Ехать ли через клетку start дальше прямо, не останавливаясь: первый
    шаг маршрута совпадает с курсом. Цель уже здесь или маршрута нет - стоп."""
    path = bfs_path(start, hdg, open_dirs, next_cell, goal_test, side_order)
    return bool(path) and path[0] == hdg


def black_retreat(traveled_cm, boundary_cm, cells, cell_cm):
    """Чёрная плитка под датчиком на ходу: (сколько клеток считать
    пройденными, на сколько см откатиться назад - в центр клетки перед
    чёрной). boundary_cm - центр следующей клетки; после решения "прямо"
    через клетку она уже клетка дальше, поэтому чёрная бывает и над
    клеткой, засчитанной пройденной: её тогда из пройденных убираем."""
    k_black = cells + 1 if traveled_cm >= boundary_cm - cell_cm / 2.0 else max(cells, 1)
    center_before = boundary_cm - (cells + 2 - k_black) * cell_cm
    return k_black - 1, traveled_cm - center_before
