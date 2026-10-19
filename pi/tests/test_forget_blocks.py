"""maze_map.forget_soft_blocks: путь к цели пропал - забыть рёбра, закрытые
препятствием и бампером, но не чёрные клетки."""
import unittest

from fakes import fake_module, import_with



def fresh_state():
    return fake_module("state", maze={}, blocked=set(), soft_closed=set(),
                       black_cells=set(), wall_votes={}, edge_fixed={}, ramps={},
                       side_pref="right")


class ForgetSoftBlocksTest(unittest.TestCase):
    def setUp(self):
        self.st = fresh_state()
        self.mm = import_with("maze_map", state=self.st,
                              walls=fake_module("walls", SIDE_THRESH=150),
                              bumper=fake_module("bumper"),
                              lidar_walls=fake_module("lidar_walls"))

    def path_east(self):
        return self.mm.find_path_bfs((0, 0), lambda c: c == (1, 0), 0)

    def open_east(self):
        self.mm.open_edge(0, 0, 1)            # (0,0) -> (1,0) проехали
        self.st.edge_fixed.clear()

    def test_obstacle_edge_is_forgotten(self):
        self.open_east()
        self.mm.block_direction(0, 0, 1)
        self.st.soft_closed.add((0, 0, 1))
        self.assertIsNone(self.path_east())
        self.assertEqual(self.mm.forget_soft_blocks(), 1)
        self.assertEqual(self.path_east(), [1])
        self.assertNotIn((1, 0), self.st.blocked)
        self.assertEqual(self.st.soft_closed, set())

    def test_black_cell_stays_closed(self):
        self.open_east()
        self.mm.block_direction(0, 0, 1)
        self.st.soft_closed.add((0, 0, 1))
        self.st.black_cells.add((1, 0))
        self.assertEqual(self.mm.forget_soft_blocks(), 0)
        self.assertIsNone(self.path_east())

    def test_lidar_wall_votes_win_after_forgetting(self):
        self.open_east()
        self.mm.close_edge(0, 0, 1)
        self.st.soft_closed.add((0, 0, 1))
        self.st.wall_votes[self.mm._edge_key(0, 0, 1)] = 2   # лидар видит стену
        self.mm.forget_soft_blocks()
        self.assertTrue(self.st.maze[(0, 0)][1])
        self.assertIsNone(self.path_east())


if __name__ == "__main__":
    unittest.main()
