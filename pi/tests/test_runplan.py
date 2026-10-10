"""Тесты маршрута по карте и решения «прямо или стоп» (pi/runplan.py)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import runplan as rp  # noqa: E402

N, E, S, W = 0, 1, 2, 3
DX = {N: 0, E: 1, S: 0, W: -1}
DY = {N: 1, E: 0, S: -1, W: 0}
RIGHT_HAND = (1, 0, 3, 2)      # SIDE_ORDER["right"] из nav.py


def nxt(cx, cy, d):
    return cx + DX[d], cy + DY[d]


def maze(*edges):
    """Открытые рёбра (клетка, направление) в обе стороны -> open_dirs()."""
    opn = {}
    for (cx, cy), d in edges:
        opn.setdefault((cx, cy), set()).add(d)
        opn.setdefault(nxt(cx, cy, d), set()).add((d + 2) % 4)
    return lambda cx, cy: sorted(opn.get((cx, cy), ()))


class TestBfsPath(unittest.TestCase):
    def test_l_shaped_path(self):
        od = maze(((0, 0), N), ((0, 1), E), ((1, 1), E))
        self.assertEqual(rp.bfs_path((0, 0), N, od, nxt, lambda c: c == (2, 1), RIGHT_HAND),
                         [N, E, E])

    def test_already_at_goal(self):
        self.assertEqual(rp.bfs_path((0, 0), N, maze(), nxt, lambda c: True, RIGHT_HAND), [])

    def test_unreachable(self):
        self.assertIsNone(rp.bfs_path((0, 0), N, maze(), nxt, lambda c: c == (5, 5), RIGHT_HAND))


class TestStepCost(unittest.TestCase):
    # квадрат 2x2: из (0,0) в (1,1) через (0,1) или через (1,0)
    od = staticmethod(maze(((0, 0), N), ((0, 1), E), ((0, 0), E), ((1, 0), N)))

    def goal(self, c):
        return c == (1, 1)

    def test_tie_keeps_side_priority(self):
        # правая рука: из (0,0) курсом N сначала направо (E)
        self.assertEqual(rp.bfs_path((0, 0), N, self.od, nxt, self.goal, RIGHT_HAND), [E, N])
        unit = lambda c, h, d, n: 1.0        # noqa: E731
        self.assertEqual(rp.bfs_path((0, 0), N, self.od, nxt, self.goal, RIGHT_HAND, unit), [E, N])

    def test_expensive_cell_avoided(self):
        blue = {(1, 0)}
        cost = lambda c, h, d, n: 1.0 + (20.0 if n in blue else 0.0)   # noqa: E731
        self.assertEqual(rp.bfs_path((0, 0), N, self.od, nxt, self.goal, RIGHT_HAND, cost), [N, E])

    def test_expensive_cell_taken_if_only_way(self):
        od = maze(((0, 0), E), ((1, 0), E))
        cost = lambda c, h, d, n: 1.0 + (20.0 if n == (1, 0) else 0.0)   # noqa: E731
        self.assertEqual(rp.bfs_path((0, 0), N, od, nxt, lambda c: c == (2, 0), RIGHT_HAND, cost),
                         [E, E])

    def test_turn_cost_prefers_straight(self):
        # в (2,1) два пути одной длины: N,N,E,E (один поворот) или E,N,E,N (три)
        od = maze(((0, 0), N), ((0, 1), E), ((1, 1), E), ((0, 0), E), ((1, 0), N),
                  ((0, 1), N), ((0, 2), E), ((1, 2), E), ((2, 2), S), ((1, 0), E), ((2, 0), N))
        cost = lambda c, h, d, n: 1.0 + (0.0 if d == h else 2.0)   # noqa: E731
        path = rp.bfs_path((0, 0), N, od, nxt, lambda c: c == (2, 1), RIGHT_HAND, cost)
        turns = sum(1 for a, b in zip([N] + path, path) if a != b)
        self.assertEqual(turns, 1)


class TestGoStraight(unittest.TestCase):
    visited = {(0, 0), (0, 1)}

    def frontier(self, c):
        return c not in self.visited

    def test_corridor_goes_straight(self):
        od = maze(((0, 0), N), ((0, 1), N))
        self.assertTrue(rp.go_straight((0, 1), N, od, nxt, self.frontier, RIGHT_HAND))

    def test_right_opening_stops_with_right_hand(self):
        od = maze(((0, 0), N), ((0, 1), N), ((0, 1), E))
        self.assertFalse(rp.go_straight((0, 1), N, od, nxt, self.frontier, RIGHT_HAND))

    def test_dead_end_stops(self):
        self.visited = {(0, 0), (0, 1), (1, 0)}
        od = maze(((0, 0), N), ((0, 0), E))
        self.assertFalse(rp.go_straight((0, 1), N, od, nxt, self.frontier, RIGHT_HAND))

    def test_home_goal_at_start_stops(self):
        od = maze(((0, 0), N), ((0, 1), N))
        self.assertFalse(rp.go_straight((0, 1), N, od, nxt, lambda c: c == (0, 1), RIGHT_HAND))

    def test_no_route_stops(self):
        self.assertFalse(rp.go_straight((0, 1), N, maze(), nxt, self.frontier, RIGHT_HAND))


class TestBlackRetreat(unittest.TestCase):
    def test_single_cell_drive_backs_to_start(self):
        self.assertEqual(rp.black_retreat(17.0, 30.0, 0, 30.0), (0, 17.0))

    def test_black_after_straight_decision_backs_to_cell_before_it(self):
        # решили "прямо" через клетку 1 (цель уже центр клетки 2 = 60см),
        # чёрная - над клеткой 1: назад в центр клетки 0, пройдено 0
        self.assertEqual(rp.black_retreat(22.0, 60.0, 1, 30.0), (0, 22.0))

    def test_black_in_next_cell_before_decision(self):
        self.assertEqual(rp.black_retreat(48.0, 60.0, 1, 30.0), (1, 18.0))

    def test_sensor_ahead_sees_next_cell_first(self):
        # Датчик на бампере, 9см впереди центра (ночной прогон, поле 22):
        # прямо через клетку 1, центр робота на 38.8см - ещё над клеткой 1,
        # а датчик (47.8см) уже за её границей (45см), над клеткой 2. Чёрная -
        # клетка 2: пройдена одна клетка, откат 8.8см в центр клетки 1, а не
        # на всю поездку назад с пометкой клетки 1 чёрной.
        self.assertEqual(rp.black_retreat(38.8, 60.0, 1, 30.0, sensor_fwd_cm=9.0),
                         (1, 38.8 - 30.0))


class TestSensorCell(unittest.TestCase):
    def test_center_based_by_default(self):
        # без смещения - как было: клетка дальше, только когда центр робота
        # на 1.5см за границей
        self.assertEqual(rp.sensor_cell(38.8, 60.0, 1, 30.0), 1)
        self.assertEqual(rp.sensor_cell(46.6, 60.0, 1, 30.0), 2)

    def test_sensor_ahead_is_over_next_cell(self):
        # поле 6 ночного прогона: синяя (2,0), за ней серебро (3,0). Центр на
        # 38.8см (клетка 1), датчик на бампере +9см = 47.8см - уже над
        # клеткой 2: серебро - её, а не клетки 1
        self.assertEqual(rp.sensor_cell(38.8, 60.0, 1, 30.0, sensor_fwd_cm=9.0), 2)
        self.assertEqual(rp.sensor_cell(36.0, 60.0, 1, 30.0, sensor_fwd_cm=9.0), 1)


if __name__ == "__main__":
    unittest.main()
