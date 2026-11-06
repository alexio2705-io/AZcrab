"""Карта лабиринта: стены, рёбра, голосование, поиск пути."""
from __future__ import annotations

import runplan
from walls import SIDE_THRESH

import config
import state
import bumper
import lidar_walls
import tile_rules


# Пороги стены для КАРТЫ - замер стоя в центре клетки. Своя стена ребра
# тогда спереди на ~FRONT_CENTER_MM (145), сзади на ~BACK_CENTER_MM (135),
# а всё, что дальше 200мм, - уже внутри соседней клетки: кирпич посреди
# неё, а не стена между клетками. С WALL_THRESH=260 такой кирпич шёл в
# карту стеной, и клетка за ним могла остаться неразведанной. Для езды
# (остановка, доводка, аварийный стоп) пороги прежние - кирпич там видеть
# надо. Если стену всё же не досчитали, робот поедет, упрётся (front_obstacle
# / wall_close) и закроет ребро по факту.
FRONT_MAP_WALL_MM = 200.0
BACK_MAP_WALL_MM = 200.0
                    # деле (чёрная плитка, неподнятое препятствие, бампер) -
                    # BFS обходит их так же, как настоящую стену.
# Стены по голосам. Раньше стена писалась в карту один раз, при первом
# приезде в клетку, и BFS верил ей навсегда: одна ошибка лидара - и робот
# либо не видел прохода (часть лабиринта не разведана), либо ехал в стену.
# Теперь каждый замер стоя (первый приезд, каждый повторный проезд, замер из
# соседней клетки - одна и та же стена видна с двух сторон) добавляет голос
# ребру, а в _maze лежит текущий итог.
#   _wall_votes[ребро] - сумма голосов, +1 "стена" / -1 "открыто", в пределах
#                        ±WALL_VOTE_CAP, чтобы давняя серия не перевешивала
#                        свежие замеры вечно;
#   _edge_fixed[ребро] - то, что известно не по лидару, а по факту, и
#                        голосами не перебивается: True - упёрлись (бампер,
#                        стена вплотную, препятствие, чёрная плитка), False -
#                        проехали по нему или это горка.
# Ребро - frozenset двух концов (клетка, направление), см. _edge_key().
WALL_VOTE_CAP = 3


# ══════════════════════════════════════════════════════════════
#  Карта лабиринта и BFS
# ══════════════════════════════════════════════════════════════
def _maze_cell(cx, cy):
    """Запись клетки в _maze, создаёт её (все 4 стороны неизвестны), если
    клетка встретилась впервые - в том числе для клетки, где робот ещё
    не стоял, но её сторону уже подтвердил сосед (см. _set_wall)."""
    return state.maze.setdefault((cx, cy), [None, None, None, None])


def _cell_at(cell, hdg, k):
    """k-я клетка по курсу hdg от cell (0 - сама cell), через известные горки."""
    cx, cy = cell
    for _ in range(k):
        cx, cy = _next_cell(cx, cy, hdg)
    return cx, cy


def _next_cell(cx, cy, d):
    """Куда ведёт ребро (cx,cy)->d: обычно в соседа, а если это известная
    горка - в клетку на другом её конце (другой этаж, см. _ramps)."""
    r = state.ramps.get((cx, cy, d))
    if r is not None:
        return r[0], r[1]
    return cx + config._DX[d], cy + config._DY[d]


def _walls_match(cell, hdg, walls_rel):
    """Свежий замер стоя (относительно курса hdg) не противоречит тому,
    что карта уже знает про cell. Зад не сравниваем - это ребро горки.
    Совпадением считаем хотя бы две известные стороны и ни одного
    расхождения."""
    known = state.maze.get(cell)
    if known is None:
        return False
    rel_to_abs = {'front': hdg, 'right': (hdg + 1) % 4, 'left': (hdg + 3) % 4}
    n = 0
    for rel, d in rel_to_abs.items():
        if known[d] is None:
            continue
        if known[d] != walls_rel[rel]:
            return False
        n += 1
    return n >= 2


def _set_wall(cx, cy, d, present):
    """Стена по абсолютному направлению d из (cx,cy) - одна и та же стена
    сразу выставляется у обеих клеток, которые она разделяет."""
    _maze_cell(cx, cy)[d] = present
    nx, ny = _next_cell(cx, cy, d)
    _maze_cell(nx, ny)[config._OPP[d]] = present


def _edge_key(cx, cy, d):
    nx, ny = _next_cell(cx, cy, d)
    return frozenset(((cx, cy, d), (nx, ny, config._OPP[d])))


def vote_wall(cx, cy, d, present):
    """Голос лидара за стену (present=True) или проход на ребре (cx,cy)->d.
    Итог - знак суммы голосов; при равенстве остаётся прежнее значение.
    Рёбра из _edge_fixed голосами не меняются. Смену уже известного
    значения печатает - по логу видно, где лидар ошибался."""
    if present is None:
        return  # Unknown is not a vote for an open passage.
    key = _edge_key(cx, cy, d)
    if key in state.edge_fixed:
        return
    v = state.wall_votes.get(key, 0) + (1 if present else -1)
    v = max(-WALL_VOTE_CAP, min(WALL_VOTE_CAP, v))
    state.wall_votes[key] = v
    old = _maze_cell(cx, cy)[d]
    new = True if v > 0 else False if v < 0 else (present if old is None else old)
    if old is not None and new != old:
        print(f"    ⟳ ({cx},{cy}) {config._DIR_NAME[d]}: "
              f"{'стена' if old else 'проход'} → {'стена' if new else 'проход'} "
              f"(голоса {v:+d})")
    _set_wall(cx, cy, d, new)


def close_edge(cx, cy, d):
    """Ребро непроезжее по факту (упёрлись) - навсегда, голоса не в счёт."""
    state.edge_fixed[_edge_key(cx, cy, d)] = True
    _set_wall(cx, cy, d, True)


def open_edge(cx, cy, d):
    """Ребро проезжее по факту (проехали) - голоса больше не в счёт. Если
    оно уже было закрыто по факту, это сильнее: туда упирались."""
    key = _edge_key(cx, cy, d)
    if state.edge_fixed.get(key) is True:
        return
    state.edge_fixed[key] = False
    _set_wall(cx, cy, d, False)


def set_cell_walls(cx, cy, hdg, walls_rel):
    """walls_rel: {'front','right','left','back': bool} - относительно
    текущего курса hdg. Переводит в абсолютные направления и голосует ими
    за стены карты (см. vote_wall)."""
    rel_to_abs = {'front': hdg, 'right': (hdg + 1) % 4,
                  'back': (hdg + 2) % 4, 'left': (hdg + 3) % 4}
    for rel, present in walls_rel.items():
        vote_wall(cx, cy, rel_to_abs[rel], present)


def block_direction(cx, cy, d):
    """Считать ребро (cx,cy)->d непроезжим, хотя лидар мог видеть его как
    открытое - для случаев, которые лидар не предсказывает (чёрная плитка,
    непреодолённое препятствие, бампер вплотную). BFS будет обходить его
    так же, как настоящую стену."""
    close_edge(cx, cy, d)
    state.blocked.add(_next_cell(cx, cy, d))


def forget_soft_blocks():
    """Путь к цели пропал - забыть рёбра, закрытые препятствием и бампером
    (state.soft_closed): ребро снова решают голоса лидара, клетка за ним
    больше не непроезжая. Рёбра в чёрные клетки остаются закрытыми.
    Возвращает, сколько рёбер забыто."""
    n = 0
    for cx, cy, d in list(state.soft_closed):
        nxt = _next_cell(cx, cy, d)
        if nxt in state.black_cells:
            continue
        key = _edge_key(cx, cy, d)
        if state.edge_fixed.get(key) is True:
            del state.edge_fixed[key]
        _set_wall(cx, cy, d, state.wall_votes.get(key, 0) > 0)
        state.blocked.discard(nxt)
        state.soft_closed.discard((cx, cy, d))
        n += 1
    return n


def open_dirs(cx, cy):
    """Направления из (cx,cy), про которые точно известно, что там открыто
    И соседняя клетка не помечена непроезжей."""
    cell = state.maze.get((cx, cy))
    if cell is None:
        return []
    return [d for d in range(4)
            if cell[d] is False and _next_cell(cx, cy, d) not in state.blocked]


def find_path_bfs(start, goal_test, hdg=0):
    """Путь с наименьшей стоимостью от start по уже известным открытым
    рёбрам карты до ближайшей клетки, для которой goal_test() верно.

    Стоимость учитывает ход, повороты и синие плитки. Историческое имя
    сохранено для совместимости. Из равных по стоимости путей выигрывает
    тот, что раньше сворачивает в
    приоритетную сторону (_side_pref): соседи каждой клетки перебираются
    по SIDE_ORDER относительно курса, которым в неё въехали (для start -
    hdg, текущий курс робота).

    Возвращает список направлений (0..3, по одному на клетку пути; первый
    шаг - первый элемент) или None, если ни одна подходящая клетка не
    достижима по разведанной части лабиринта."""
    return runplan.weighted_path(start, hdg, open_dirs, _next_cell, goal_test,
                                 config.SIDE_ORDER[state.side_pref], route_step_cost)


def route_step_cost(src, dst, heading, direction):
    turn = abs((direction - heading + 2) % 4 - 2)
    cost = config.ROUTE_CELL_S + config.ROUTE_QUARTER_TURN_S * turn
    # Challenges can explicitly make blue ordinary; then no standard penalty.
    if tile_rules.role(config.TILE_BLUE) == tile_rules.STOP and (
            dst in state.blue_cells or state.tile_map.get(dst) == config.TILE_BLUE):
        cost += config.BLUE_WAIT_S
        if state.blue_visits.get(dst, 0):
            cost += config.ROUTE_BLUE_REVISIT_PENALTY_S
    return cost


def find_frontier_path(cx, cy, hdg=0):
    """Путь до ближайшей ещё не посещённой клетки - см. find_path_bfs().
    None означает, что разведанная часть лабиринта исчерпана: из текущей
    клетки по известным открытым рёбрам больше некуда идти."""
    return find_path_bfs((cx, cy), lambda c: c not in state.visited, hdg)


def _walls_rel_map(w):
    """Замер стоя -> {'front','right','left','back': bool} по порогам карты
    (FRONT_MAP_WALL_MM / BACK_MAP_WALL_MM). Прижатые оба концевика - стена
    впереди, даже если лидар её не видит (вплотную). Один - нет: заезд
    24.10, робот задел правым угол стены, концевик ещё был прижат, и замер
    записал стену поперёк прохода, хотя лидар видел впереди 340-390мм."""
    return {
        "front": lidar_walls.wall_present(w["front"], FRONT_MAP_WALL_MM) or bumper.both_pressed(),
        "right": lidar_walls.wall_present(w["right"], SIDE_THRESH),
        "left":  lidar_walls.wall_present(w["left"], SIDE_THRESH),
        "back":  lidar_walls.wall_present(w["back"], BACK_MAP_WALL_MM),
    }
