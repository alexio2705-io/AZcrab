"""Тесты чистой логики наблюдателя меток (pi/watch_logic.py).

Запуск из корня репозитория:
    python3 -m unittest discover -s pi/tests -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import watch_logic as wl  # noqa: E402

STRIDE = 1000


class TestGeometry(unittest.TestCase):
    def test_px_to_mm_center_is_zero(self):
        self.assertAlmostEqual(wl.px_to_mm(0.5), 0.0)

    def test_px_to_mm_edges_are_90mm(self):
        self.assertAlmostEqual(wl.px_to_mm(0.0), 90.0, delta=1.0)
        self.assertAlmostEqual(wl.px_to_mm(1.0), -90.0, delta=1.0)

    def test_px_to_mm_top_is_forward(self):
        # верх кадра - вперёд по ходу (шлейф смотрит назад)
        self.assertGreater(wl.px_to_mm(0.2), 0.0)

    def test_px_to_mm_monotonic_and_fisheye(self):
        vals = [wl.px_to_mm(i / 10) for i in range(11)]
        self.assertTrue(all(a > b for a, b in zip(vals, vals[1:])))
        center_step = vals[5] - vals[6]
        edge_step = vals[0] - vals[1]
        self.assertGreater(edge_step, 1.4 * center_step)   # у края мм/px крупнее

    def test_px_to_mm_quarter(self):
        self.assertAlmostEqual(wl.px_to_mm(0.25), 39.3, delta=0.5)

    def test_mark_pos_accounts_for_camera_back_offset(self):
        self.assertAlmostEqual(wl.mark_pos_mm(200.0, 30.0), 205.0)

    def test_wall_of(self):
        self.assertEqual(wl.wall_of(0, "right"), 1)
        self.assertEqual(wl.wall_of(0, "left"), 3)
        self.assertEqual(wl.wall_of(3, "right"), 0)
        self.assertEqual(wl.wall_of(2, "left"), 1)

    def test_level_of(self):
        self.assertEqual(wl.level_of(0, STRIDE), 0)
        self.assertEqual(wl.level_of(1003, STRIDE), 1)
        self.assertEqual(wl.level_of(-998, STRIDE), -1)

    def test_world_along_heading_north_east_wall(self):
        key, along = wl.world_along((0, 0), 0, 1, 205.0, STRIDE)
        self.assertEqual(key, (1, 0, 0))
        self.assertAlmostEqual(along, 205.0)

    def test_same_mark_from_opposite_directions(self):
        # едем на север из (0,0): метка на восточной стене в 205мм впереди;
        # едем на юг из (0,1): та же метка - левой камерой, в 95мм впереди
        k1, a1 = wl.world_along((0, 0), 0, 1, 205.0, STRIDE)
        k2, a2 = wl.world_along((0, 1), 2, 1, 95.0, STRIDE)
        self.assertEqual(k1, k2)
        self.assertAlmostEqual(a1, a2)

    def test_world_along_east_west_headings_on_north_south_walls(self):
        k1, a1 = wl.world_along((2, 3), 1, 0, 100.0, STRIDE)
        k2, a2 = wl.world_along((3, 3), 3, 0, 200.0, STRIDE)
        self.assertEqual(k1, (0, 3))
        self.assertEqual(k1, k2)
        self.assertAlmostEqual(a1, 700.0)
        self.assertAlmostEqual(a2, 700.0)

    def test_neighbour_cells_share_wall_key(self):
        k1, _ = wl.world_along((0, 0), 0, 1, 0.0, STRIDE)
        k2, _ = wl.world_along((0, 1), 0, 1, 0.0, STRIDE)
        self.assertEqual(k1, k2)

    def test_opposite_faces_differ(self):
        k_east, _ = wl.world_along((0, 0), 0, 1, 0.0, STRIDE)   # восточная (0,0)
        k_west, _ = wl.world_along((1, 0), 0, 3, 0.0, STRIDE)   # западная (1,0)
        self.assertNotEqual(k_east, k_west)

    def test_levels_differ(self):
        k0, a0 = wl.world_along((0, 0), 0, 1, 50.0, STRIDE)
        k1, a1 = wl.world_along((0, 1000), 0, 1, 50.0, STRIDE)
        self.assertNotEqual(k0, k1)
        self.assertAlmostEqual(a0, a1)

    def test_attribute_boundary_150mm(self):
        self.assertEqual(wl.attribute((0, 0), (0, 1), 0, "right", 149.9), ((0, 0), 1))
        self.assertEqual(wl.attribute((0, 0), (0, 1), 0, "right", 150.0), ((0, 1), 1))
        self.assertEqual(wl.attribute((0, 0), (0, 1), 0, "left", 10.0), ((0, 0), 3))


if __name__ == "__main__":
    unittest.main()
