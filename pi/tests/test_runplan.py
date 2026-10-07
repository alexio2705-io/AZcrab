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


if __name__ == "__main__":
    unittest.main()
