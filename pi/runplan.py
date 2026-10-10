"""Маршрут по карте клеток - поиск из nav.find_path_bfs, вынесенный сюда, чтобы
решение "прямо или стоп" на ходу (drive_cells с планом) проверялось тестами
без железа: nav.py на Маке не импортируется (RPi.GPIO, serial)."""
from __future__ import annotations

import heapq


def bfs_path(start, hdg, open_dirs, next_cell, goal_test, side_order, step_cost=None):
    """Самый дешёвый путь от start по открытым рёбрам до ближайшей клетки,
    для которой goal_test() верно.

    step_cost(cell, h, d, nxt) - цена шага из cell (въехали курсом h) в nxt
    по направлению d; None - каждый шаг по 1, то есть кратчайший по числу
    клеток (обычный BFS). Поиск - Дейкстра по состояниям (клетка, курс
    въезда): цена может зависеть от поворота.

    Из равных по цене путей выигрывает тот, что раньше сворачивает в
    приоритетную сторону: соседи каждой клетки перебираются по side_order
    относительно курса, которым в неё въехали (для start - hdg), а из
    равных по цене состояний первым раскрывается найденное раньше.
    open_dirs(cx, cy) - открытые направления клетки, next_cell(cx, cy, d) -
    куда ведёт ребро.

    Возвращает список направлений (0..3, по одному на клетку пути) или None,
    если ни одна подходящая клетка не достижима."""
    if goal_test(start):
        return []
    s0 = (start, hdg)
    dist = {s0: 0.0}
    prev = {}
    done = set()
    seq = 1
    heap = [(0.0, 0, start, hdg)]
    while heap:
        g, _, cell, h = heapq.heappop(heap)
        if (cell, h) in done:
            continue
        done.add((cell, h))
        if cell != start and goal_test(cell):
            path, st = [], (cell, h)
            while st != s0:
                st, d = prev[st]
                path.append(d)
            path.reverse()
            return path
        dirs = open_dirs(cell[0], cell[1])
        for d in ((h + k) % 4 for k in side_order):
            if d not in dirs:
                continue
            nxt = next_cell(cell[0], cell[1], d)
            if nxt == start:
                continue
            ng = g + (1.0 if step_cost is None else step_cost(cell, h, d, nxt))
            st = (nxt, d)
            if st in dist and dist[st] <= ng:
                continue
            dist[st] = ng
            prev[st] = ((cell, h), d)
            heapq.heappush(heap, (ng, seq, nxt, d))
            seq += 1
    return None


def go_straight(start, hdg, open_dirs, next_cell, goal_test, side_order, step_cost=None):
    """Ехать ли через клетку start дальше прямо, не останавливаясь: первый
    шаг маршрута совпадает с курсом. Цель уже здесь или маршрута нет - стоп."""
    path = bfs_path(start, hdg, open_dirs, next_cell, goal_test, side_order, step_cost)
    return bool(path) and path[0] == hdg


def sensor_cell(traveled_cm, boundary_cm, cells, cell_cm, sensor_fwd_cm=0.0, margin_cm=1.5):
    """Над какой клеткой поездки датчик цвета: cells (засчитана последней)
    или cells + 1 - уже за её границей с запасом margin_cm. boundary_cm -
    центр следующей клетки. sensor_fwd_cm - датчик впереди центра робота."""
    if traveled_cm + sensor_fwd_cm >= boundary_cm - cell_cm / 2.0 + margin_cm:
        return cells + 1
    return cells


def black_retreat(traveled_cm, boundary_cm, cells, cell_cm, sensor_fwd_cm=0.0):
    """Чёрная плитка под датчиком на ходу: (сколько клеток считать
    пройденными, на сколько см откатиться назад - в центр клетки перед
    чёрной). boundary_cm - центр следующей клетки; после решения "прямо"
    через клетку она уже клетка дальше, поэтому чёрная бывает и над
    клеткой, засчитанной пройденной: её тогда из пройденных убираем.
    sensor_fwd_cm - датчик впереди центра робота: чёрная - клетка под
    датчиком, а не под центром (датчик на бампере видит следующую клетку,
    пока центр ещё в предыдущей)."""
    k_black = (cells + 1 if traveled_cm + sensor_fwd_cm >= boundary_cm - cell_cm / 2.0
               else max(cells, 1))
    center_before = boundary_cm - (cells + 2 - k_black) * cell_cm
    return k_black - 1, traveled_cm - center_before
