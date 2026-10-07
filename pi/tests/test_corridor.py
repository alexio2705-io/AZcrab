"""Тесты стен клеток впереди по точкам лидара (pi/corridor.py).

Обороты синтетические: лучи по 1° до прямых боковых стен на ±150 мм, с теми же
масками, что в walls.MASKS (ручки, леска, передние стойки, зад).
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import corridor as cr  # noqa: E402

MASKS = ((90, 12), (270, 12), (105, 7), (255, 7), (25, 10), (335, 10), (180, 55))


def scan(x_robot, walls):
    """Оборот с позиции x_robot (система пути): walls - [(сторона, X от, X до)]
    стен на 150 мм вбок. Возвращает точки в системе робота (x вперёд, y вправо)."""
    pts = []
    for deg in range(360):
        if any(abs((deg - c + 180) % 360 - 180) <= hw for c, hw in MASKS):
            continue
        r = math.radians(deg)
        s, c = math.sin(r), math.cos(r)
        for side, lo, hi in walls:
            sgn = 1 if side == "right" else -1
            if s * sgn <= 0.05:
                continue
            d = 150.0 / abs(s)
            if d <= 600 and lo <= x_robot + d * c <= hi:
                pts.append((d * c, sgn * 150.0))
    return pts


def feed(sides, positions, walls, k=1, c=300.0):
    for x in positions:
        path = cr.to_path_frame(scan(x, walls), 1.0, 0.1, lambda t, x=x: x, 0.0)
        sides.add_scan(path, x, [(k, c)])


class TestFrames(unittest.TestCase):
    def test_point_time_follows_rotation_from_rear_seam(self):
        self.assertAlmostEqual(cr.point_time(1.0, 0.1, -100, 0), 0.9)    # 180° - начало
        self.assertAlmostEqual(cr.point_time(1.0, 0.1, 100, 0), 0.95)    # 0° - середина
        self.assertAlmostEqual(cr.point_time(1.0, 0.1, 0, 100), 0.975)   # 90° - правый борт

    def test_to_path_frame_static(self):
        self.assertEqual(cr.to_path_frame([(100, 150)], 1.0, 0.1, lambda t: 200.0, 0.0),
                         [(300.0, 150.0)])

    def test_to_path_frame_uses_point_time(self):
        (X, _), = cr.to_path_frame([(0, 150)], 1.0, 0.1, lambda t: 1000.0 * t, 0.0)
        self.assertAlmostEqual(X, 975.0)

    def test_to_path_frame_rotates_by_yaw_error(self):
        (X, Y), = cr.to_path_frame([(100, 0)], 1.0, 0.1, lambda t: 200.0, 10.0)
        self.assertAlmostEqual(X, 200.0 + 100.0 * math.cos(math.radians(10)))
        self.assertAlmostEqual(Y, 100.0 * math.sin(math.radians(10)))   # довернул вправо


class TestVisible(unittest.TestCase):
    def test_next_cell_wall_visible_near_decision_point(self):
        span = len(cr.visible_steps(190.0, 200.0, 400.0, "right", MASKS)) * cr.SPAN_STEP_MM
        self.assertGreaterEqual(span, 150.0)

    def test_front_struts_hide_far_wall(self):
        span = len(cr.visible_steps(0.0, 200.0, 400.0, "right", MASKS)) * cr.SPAN_STEP_MM
        self.assertLess(span, 50.0)

    def test_left_mirrors_right(self):
        self.assertEqual(cr.visible_steps(190.0, 200.0, 400.0, "left", MASKS),
                         cr.visible_steps(190.0, 200.0, 400.0, "right", MASKS))


class TestCellSides(unittest.TestCase):
    def test_wall_present(self):
        s = cr.CellSides(MASKS)
        feed(s, (100.0, 150.0, 190.0), [("right", 150.0, 450.0)])
        self.assertIs(s.result(1, "right")[0], True)

    def test_opening(self):
        s = cr.CellSides(MASKS)
        feed(s, (100.0, 150.0, 190.0), [("right", 150.0, 450.0)])
        val, n, span = s.result(1, "left")
        self.assertIs(val, False)
        self.assertEqual(n, 0)
        self.assertGreaterEqual(span, cr.OPEN_MIN_SPAN_MM)

    def test_too_little_seen_is_unsure(self):
        s = cr.CellSides(MASKS)
        feed(s, (0.0,), [])
        self.assertIsNone(s.result(1, "right")[0])

    def test_no_points_is_unsure(self):
        self.assertEqual(cr.CellSides(MASKS).result(1, "right"), (None, 0, 0.0))

    def test_band_x_kept_for_posts_and_pruned(self):
        s = cr.CellSides(MASKS)
        feed(s, (190.0,), [("right", 150.0, 450.0)])
        self.assertTrue(s.band_x["right"])
        s.prune(10_000.0)
        self.assertEqual(s.band_x["right"], [])


class TestPostError(unittest.TestCase):
    @staticmethod
    def wall(lo, hi):
        return [lo + i * 5.0 for i in range(int((hi - lo) / 5.0) + 1)]

    # Стена толщиной 20 мм: на идеальной одометрии она кончается на
    # полтолщины ЗА границей клеток (столбик), а начинается - на полтолщины ДО.
    def test_perfect_post_has_no_error(self):
        self.assertEqual(cr.post_error(self.wall(300, 460), 450.0, True, False), 0.0)
        self.assertEqual(cr.post_error(self.wall(440, 600), 450.0, False, True), 0.0)

    def test_wall_ends_later_than_expected(self):
        self.assertEqual(cr.post_error(self.wall(300, 475), 450.0, True, False), 15.0)

    def test_wall_starts_earlier_than_expected(self):
        self.assertEqual(cr.post_error(self.wall(430, 600), 450.0, False, True), -10.0)

    def test_too_far_is_ignored(self):
        self.assertIsNone(cr.post_error(self.wall(300, 510), 450.0, True, False))

    def test_same_sides_have_no_post(self):
        self.assertIsNone(cr.post_error(self.wall(300, 600), 450.0, True, True))
        self.assertIsNone(cr.post_error(self.wall(300, 465), 450.0, True, None))

    def test_needs_points_at_the_end(self):
        self.assertIsNone(cr.post_error([300.0, 360.0, 465.0], 450.0, True, False))


class TestCorridorPoints(unittest.TestCase):
    def test_keeps_only_side_corridors(self):
        filt = {0: 300.0, 45: 212.13, 90: 140.0, 270: 140.0, 30: 1000.0}
        self.assertEqual(sorted(map(tuple, cr.corridor_points(filt))),
                         [(0, -140), (0, 140), (150, 150)])


if __name__ == "__main__":
    unittest.main()
