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


class TestRegistry(unittest.TestCase):
    def setUp(self):
        self.reg = wl.MarkRegistry()
        self.key = (1, 0, 0)
        self.reg.add(self.key, 205.0, "read")

    def test_near_within_15cm(self):
        self.assertEqual(self.reg.near(self.key, 354.9), (self.key, 205.0, "read"))

    def test_not_near_at_15cm(self):
        self.assertIsNone(self.reg.near(self.key, 355.0))

    def test_other_wall_is_not_near(self):
        self.assertIsNone(self.reg.near((3, 1, 0), 205.0))

    def test_nearest_wins(self):
        self.reg.add(self.key, 300.0, "rejected")
        self.assertEqual(self.reg.near(self.key, 290.0)[2], "rejected")


class TestDecideStop(unittest.TestCase):
    def setUp(self):
        self.reg = wl.MarkRegistry()
        self.key = (1, 0, 0)

    def decide(self, conf=0.6, side_px=60, along=500.0, busy=False):
        return wl.decide_stop(conf, side_px, self.key, along, self.reg, busy, 0.25, 40)

    def test_stop(self):
        self.assertEqual(self.decide(), (True, "стоп"))

    def test_low_conf(self):
        self.assertEqual(self.decide(conf=0.2), (False, "conf"))

    def test_small_box(self):
        self.assertEqual(self.decide(side_px=39), (False, "мелкая"))

    def test_busy(self):
        self.assertEqual(self.decide(busy=True), (False, "уже стоим"))

    def test_duplicate_of_read(self):
        self.reg.add(self.key, 450.0, "read")
        self.assertEqual(self.decide(), (False, "дубль"))

    def test_duplicate_of_rejected(self):
        self.reg.add(self.key, 560.0, "rejected")
        self.assertEqual(self.decide(), (False, "отбракована раньше"))


class TestStopAndCreep(unittest.TestCase):
    def test_stop_point(self):
        self.assertAlmostEqual(wl.stop_point_cm(10.0, 55.0, 300.0), 10.0 + 3025.0 / 600.0)

    def test_stop_point_standing(self):
        self.assertAlmostEqual(wl.stop_point_cm(12.0, 0.0, 300.0), 12.0)

    def test_creep_box_centered_no_move(self):
        self.assertEqual(wl.creep_to_mark((0.45, 0.55), 999.0, 15.0), (0.0, "рамка"))

    def test_creep_box_above_center_goes_forward(self):
        mm, src = wl.creep_to_mark((0.2, 0.4), 0.0, 15.0)
        self.assertEqual(src, "рамка")
        assert mm is not None
        self.assertAlmostEqual(mm, 31.0, delta=0.5)

    def test_creep_box_cut_by_edge(self):
        mm, src = wl.creep_to_mark((0.0, 0.2), 0.0, 15.0)
        self.assertEqual(src, "рамка")
        assert mm is not None
        self.assertAlmostEqual(mm, wl.px_to_mm(0.1))
        self.assertGreater(mm, 0.0)

    def test_creep_by_estimate_backwards(self):
        self.assertEqual(wl.creep_to_mark(None, -95.0, 15.0), (-95.0, "оценка"))

    def test_creep_too_far_rejected(self):
        self.assertEqual(wl.creep_to_mark(None, 160.0, 15.0), (None, "оценка"))


if __name__ == "__main__":
    unittest.main()
