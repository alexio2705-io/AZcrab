"""actions: то, что проверяется без железа - таймер, оценка пути, найденное."""
import unittest

from fakes import fake_module, import_with

import config


class FakeWalk:
    def __init__(self, cx=0, cy=0, hdg=0, total=0):
        self.cx, self.cy, self.hdg, self.total = cx, cy, hdg, total

    @property
    def cell(self):
        return self.cx, self.cy


class ActionsTest(unittest.TestCase):
    def setUp(self):
        self.st = fake_module("state", tile_map={}, react_cells={}, tacc={},
                              victim_log=[], CHECKPOINT_TILE=config.TILE_SILVER)
        self.paths = {}
        mm = fake_module("maze_map", find_path_bfs=self.fake_bfs)
        tr = import_with("tile_rules", state=self.st)
        self.act = import_with("actions", state=self.st, maze_map=mm, tile_rules=tr,
                               leds=fake_module("leds"), dash_live=fake_module("dash_live"),
                               esp_link=fake_module("esp_link"), explore=fake_module("explore"),
                               victims=fake_module("victims"))

    def fake_bfs(self, start, goal_test, hdg=0):
        for cell, path in self.paths.items():
            if goal_test(cell):
                return path
        return None

    def test_sec_per_cell_default_then_measured(self):
        a = self.act
        self.assertEqual(a.sec_per_cell(2, {"ход": 100.0}), a.SEC_PER_CELL)
        tacc = {"ход": 20.0, "поворот": 6.0, "доводка": 2.0, "лидар": 2.0, "камеры": 50.0}
        self.assertAlmostEqual(a.sec_per_cell(10, tacc), 3.0)     # камеры не в счёт

    def test_path_cells(self):
        a = self.act
        self.paths[(2, 1)] = [1, 1, 0]
        self.assertEqual(a.path_cells(FakeWalk(), (2, 1)), 3)
        self.assertEqual(a.path_cells(FakeWalk(), (0, 0)), 0)
        self.assertEqual(a.path_cells(FakeWalk(), (-3, 2)), 5)    # пути нет - по прямой

    def test_timer(self):
        a = self.act
        now = [100.0]
        t = a.Timer(180, clock=lambda: now[0])
        now[0] = 130.0
        self.assertEqual(t.elapsed(), 30.0)
        self.assertEqual(t.left(), 150.0)
        self.paths[(0, 0)] = [2] * 10
        w = FakeWalk(0, 10, total=10)
        self.st.tacc.update({"ход": 30.0})                       # 3 с на клетку
        self.assertAlmostEqual(t.trip_s(w, (0, 0)), 10 * 3.0 * a.TRIP_MARGIN)
        self.assertFalse(t.time_to_go(w, (0, 0), extra_s=10))     # 150 > 39 + 10
        now[0] = 241.0                                            # осталось 39 с
        self.assertTrue(t.time_to_go(w, (0, 0), extra_s=10))

    def test_found(self):
        a = self.act
        self.st.tile_map.update({(1, 0): config.TILE_RED, (2, 0): config.TILE_BLUE,
                                 (3, 0): config.TILE_RED})
        self.assertEqual(sorted(a.cells_of(config.TILE_RED)), [(1, 0), (3, 0)])
        self.st.victim_log.extend([{"class": "H"}, {"class": "phi"}, {"class": "S"}])
        self.assertEqual(len(a.marks_found({"H", "S"})), 2)
        self.assertEqual(len(a.marks_found()), 3)


if __name__ == "__main__":
    unittest.main()
