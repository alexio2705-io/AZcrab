"""Маршрут по времени, синие, уход домой по лимиту (pi/nav.py) - на карте в памяти."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import maze_sim as ms  # noqa: E402
import run_scenarios as rs  # noqa: E402

N, E, S, W = 0, 1, 2, 3


def open_grid(nav, w, h, walls=()):
    """Карта nav: открытая площадка w x h со стенами по краю и walls - (клетка, d)."""
    nav._maze.clear()
    for x in range(w):
        for y in range(h):
            nav._maze[(x, y)] = [y == h - 1, x == w - 1, y == 0, x == 0]
            nav._visited.add((x, y))
    for (x, y), d in walls:
        nav._maze[(x, y)][d] = True
        n = nav._next_cell(x, y, d)
        if n in nav._maze:
            nav._maze[n][(d + 2) % 4] = True


class RoutesTest(unittest.TestCase):
    def setUp(self):
        rs.fresh_nav()
        self.nav = ms.nav
        self.nav._side_pref = "right"

    def test_fewer_turns_preferred(self):
        nav = self.nav
        open_grid(nav, 3, 3)
        path = nav.find_path_bfs((0, 0), lambda c: c == (2, 2), N)
        turns = sum(1 for a, b in zip([N] + path, path) if a != b)
        self.assertEqual(len(path), 4)
        self.assertEqual(turns, 1, "лесенкой дольше - поворот стоит ~1.8 с")

    def test_visited_blue_avoided_when_detour_exists(self):
        nav = self.nav
        open_grid(nav, 3, 2)
        nav._blue_cells.add((1, 0))
        path = nav.find_path_bfs((0, 0), lambda c: c == (2, 0), E)
        cells, c = [], (0, 0)
        for d in path:
            c = nav._next_cell(c[0], c[1], d)
            cells.append(c)
        self.assertNotIn((1, 0), cells)

    def test_path_time(self):
        nav = self.nav
        open_grid(nav, 3, 1)
        t = nav.path_time((0, 0), N, [E, E])
        self.assertAlmostEqual(t, 2 * nav.ROUTE_CELL_S + nav.ROUTE_TURN_S)
        nav._blue_cells.add((1, 0))
        self.assertAlmostEqual(nav.path_time((0, 0), E, [E]), nav.ROUTE_CELL_S + nav.BLUE_STOP_S)

    def test_must_go_home(self):
        nav = self.nav
        open_grid(nav, 4, 1)
        nav.time = ms.FakeTime(type("S", (), {"t": 1000.0})())
        nav._t_run = None
        self.assertFalse(nav.must_go_home(3, 0, W, [W])[0], "заезд не идёт - лимита нет")
        nav._t_run = 1000.0 - 100.0
        self.assertFalse(nav.must_go_home(3, 0, W, [W])[0])
        nav._t_run = 1000.0 - (nav.RUN_BUDGET_S - 20.0)
        late, why = nav.must_go_home(3, 0, W, [W])
        self.assertTrue(late, why)

    def test_home_skips_edge_that_failed_twice(self):
        nav = self.nav
        open_grid(nav, 2, 2)
        # домой из (1,1): мягко закрыты оба ребра к (0,1) и (1,0)
        nav.close_edge(1, 1, W)
        nav.close_edge(1, 1, S)
        p1 = nav.find_home_path(1, 1, N)
        self.assertIsNotNone(p1)
        key = nav._edge_key(1, 1, p1[0])
        nav._home_fails[key] = nav.HOME_EDGE_TRIES
        p2 = nav.find_home_path(1, 1, N)
        self.assertNotEqual(p2[0], p1[0], "не пустившее дважды ребро - в последнюю очередь")


if __name__ == "__main__":
    unittest.main()
