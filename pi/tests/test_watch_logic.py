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
    # Калибровка линейкой (правая камера, 2026-10-07): напротив объектива -
    # y=165.3 из 320, деления 6 см вверх/вниз - y=5 и y=315.
    def test_px_to_mm_lens_axis_is_zero(self):
        self.assertAlmostEqual(wl.px_to_mm(165.3 / 320), 0.0)

    def test_px_to_mm_calibration_points(self):
        self.assertAlmostEqual(wl.px_to_mm(5.0 / 320), 60.0)
        self.assertAlmostEqual(wl.px_to_mm(106.0 / 320), 20.0)
        self.assertAlmostEqual(wl.px_to_mm(315.0 / 320), -60.0)

    def test_px_to_mm_interpolates_between_points(self):
        self.assertAlmostEqual(wl.px_to_mm((135.3 + 165.3) / 2 / 320), 5.0)

    def test_px_to_mm_extrapolates_past_frame_edges(self):
        # центр обрезанной метки (mark_center_y) бывает за краем кадра
        self.assertGreater(wl.px_to_mm(-10.0 / 320), 60.0)
        self.assertLess(wl.px_to_mm(330.0 / 320), -60.0)

    def test_px_to_mm_top_is_forward(self):
        # верх кадра - вперёд по ходу (шлейф смотрит назад)
        self.assertGreater(wl.px_to_mm(0.2), 0.0)

    def test_px_to_mm_monotonic_and_fisheye(self):
        vals = [wl.px_to_mm(i / 10) for i in range(11)]
        self.assertTrue(all(a > b for a, b in zip(vals, vals[1:])))
        center_step = vals[5] - vals[6]
        edge_step = vals[0] - vals[1]
        self.assertGreater(edge_step, 1.2 * center_step)   # у края мм/px крупнее

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

    def test_cell_index_along_multi_cell_drive(self):
        for mm, k in ((0, 0), (149.9, 0), (150, 1), (449.9, 1), (450, 2), (-10, 0)):
            self.assertEqual(wl.cell_index(mm), k)


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
        self.assertAlmostEqual(wl.stop_point_cm(10.0, 55.0, 300.0, 0.05),
                               10.0 + 55.0 * 0.05 + 3025.0 / 600.0)

    def test_stop_point_standing(self):
        self.assertAlmostEqual(wl.stop_point_cm(12.0, 0.0, 300.0, 0.05), 12.0)

    def test_stop_point_brakes_from_current_speed(self):
        # профиль drive_cells: v = sqrt(2a*(остаток - v*STOP_LEAD_S)); в такт
        # решения он должен дать текущую скорость, а не скачок вниз (рывок)
        v, a, lead = 55.0, 300.0, 0.05
        rem = wl.stop_point_cm(0.0, v, a, lead)
        self.assertAlmostEqual((2.0 * a * (rem - v * lead)) ** 0.5, v)

    def test_creep_box_centered_no_move(self):
        self.assertEqual(wl.creep_to_mark((0.45, 0.55), 999.0, 15.0), (0.0, "рамка"))

    def test_creep_box_above_center_goes_forward(self):
        mm, src = wl.creep_to_mark((0.2, 0.4), 0.0, 15.0)
        self.assertEqual(src, "рамка")
        assert mm is not None
        self.assertAlmostEqual(mm, 30.0 - 10.0 * (96.0 - 77.3) / (106.0 - 77.3))

    def test_creep_box_cut_by_edge(self):
        mm, src = wl.creep_to_mark((0.0, 0.2), 0.0, 15.0)
        self.assertEqual(src, "рамка")
        assert mm is not None
        self.assertAlmostEqual(mm, wl.px_to_mm(0.1))
        self.assertGreater(mm, 0.0)

    def test_creep_by_estimate_backwards(self):
        self.assertEqual(wl.creep_to_mark(None, -95.0, 15.0), (-95.0, "оценка"))

    def test_creep_back_after_real_latency_accepted(self):
        # замер на Pi: кадр->решение ~175 мс при двух камерах; на 55 см/с
        # это ~10 см, плюс тормозной путь с упреждением ~8 см - назад ~20 см
        self.assertEqual(wl.creep_to_mark(None, -200.0, 15.0), (-200.0, "оценка"))

    def test_creep_too_far_rejected(self):
        self.assertEqual(wl.creep_to_mark(None, 260.0, 15.0), (None, "оценка"))


class TestMarkCenter(unittest.TestCase):
    def test_full_box_center(self):
        self.assertAlmostEqual(wl.mark_center_y((100, 100, 160, 160)), 130 / 320)

    def test_cut_by_top_edge_uses_width(self):
        # метка въезжает в кадр сверху (спереди): видна нижняя часть 20px,
        # ширина не обрезана - центр на полширины выше нижнего края
        self.assertAlmostEqual(wl.mark_center_y((100, 0, 160, 20)), (20 - 30) / 320)

    def test_cut_by_bottom_edge_uses_width(self):
        self.assertAlmostEqual(wl.mark_center_y((100, 300, 160, 320)), (300 + 30) / 320)

    def test_tall_box_at_edge_is_not_cut(self):
        self.assertAlmostEqual(wl.mark_center_y((100, 0, 140, 60)), 30 / 320)


class TestBrakeTarget(unittest.TestCase):
    def test_brakes_to_mark_when_far_enough(self):
        # метка в 300мм от центра клетки: в центре кадра она будет, когда
        # центр робота (объектив на 25мм сзади) пройдёт 325мм
        self.assertEqual(wl.brake_target_cm(300.0, 10.0, 55.0, 300.0, 0.05), (32.5, False))

    def test_too_late_stops_as_soon_as_possible(self):
        target, late = wl.brake_target_cm(100.0, 20.0, 55.0, 300.0, 0.05)
        self.assertTrue(late)
        self.assertAlmostEqual(target, wl.stop_point_cm(20.0, 55.0, 300.0, 0.05))


class TestPreferNcnn(unittest.TestCase):
    def test_pt_when_no_export(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            pt = os.path.join(d, "maze_v26n_B.pt")
            self.assertEqual(wl.prefer_ncnn(pt), pt)

    def test_ncnn_dir_when_exported(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            pt = os.path.join(d, "maze_v26n_B.pt")
            os.mkdir(os.path.join(d, "maze_v26n_B_ncnn_model"))
            self.assertEqual(wl.prefer_ncnn(pt), os.path.join(d, "maze_v26n_B_ncnn_model"))


class TestWatchAllowed(unittest.TestCase):
    def ok(self, **kw):
        a = dict(ramp_state=None, bump_on=False, assist_mode=None, pitch_delta=0.5,
                 rearm_deg=1.2, wall_forced=False)
        a.update(kw)
        return wl.watch_allowed(**a)

    def test_flat(self):
        self.assertTrue(self.ok())

    def test_blocked(self):
        for kw in (dict(ramp_state="cand"), dict(ramp_state="exit"), dict(bump_on=True),
                   dict(assist_mode="kick"), dict(pitch_delta=1.3), dict(pitch_delta=-1.3),
                   dict(wall_forced=True)):
            with self.subTest(**kw):
                self.assertFalse(self.ok(**kw))


class TestShiftTimers(unittest.TestCase):
    def test_shift_skips_unarmed_zero(self):
        self.assertEqual(wl.shift_timers((10.0, 0.0, 5.0), 2.5), (12.5, 0.0, 7.5))


def _at(h, t):
    v = h.at(t)
    assert v is not None
    return v


class TestOdoHistory(unittest.TestCase):
    def test_empty(self):
        self.assertIsNone(wl.OdoHistory().at(1.0))

    def test_interpolate_and_clamp(self):
        h = wl.OdoHistory()
        h.add(0.0, 0.0)
        h.add(0.1, 5.0)
        h.add(0.2, 10.0)
        self.assertAlmostEqual(_at(h, 0.15), 7.5)
        self.assertAlmostEqual(_at(h, -1.0), 0.0)
        self.assertAlmostEqual(_at(h, 5.0), 10.0)

    def test_trims_old(self):
        h = wl.OdoHistory(keep_s=1.0)
        for i in range(21):
            h.add(i * 0.1, float(i))
        self.assertGreaterEqual(h.t[0], 2.0 - 1.0 - 1e-9)
        self.assertAlmostEqual(_at(h, 0.0), h.d[0])


class TestStats(unittest.TestCase):
    def test_percentiles(self):
        s = wl.Stats()
        for i in range(1, 101):
            s.add(float(i))
        self.assertEqual(s.pct(50), 50.0)
        self.assertEqual(s.pct(95), 95.0)
        self.assertIn("max 100.0", s.fmt())
        self.assertIn("n=100", s.fmt())

    def test_empty(self):
        self.assertIsNone(wl.Stats().pct(50))
        self.assertEqual(wl.Stats().fmt(), "—")


class TestSideHold(unittest.TestCase):
    def test_hold(self):
        h = wl.SideHold(0.2)
        self.assertEqual(h.update(0.0, {"right": True, "left": False}), {"right"})
        self.assertEqual(h.update(0.1, {"right": False, "left": False}), {"right"})
        self.assertEqual(h.update(0.25, {"right": False, "left": False}), set())

    def test_reset(self):
        h = wl.SideHold(0.2)
        h.update(0.0, {"left": True})
        h.reset()
        self.assertEqual(h.update(0.05, {"left": False}), set())


if __name__ == "__main__":
    unittest.main()


class TurnedMarkPosTest(unittest.TestCase):
    def test_straight_matches_mark_pos(self):
        for o in (-50.0, 0.0, 37.0):
            for side in ("left", "right"):
                self.assertAlmostEqual(wl.turned_mark_pos(0.0, side, o), wl.mark_pos_mm(0.0, o))

    def test_turned_cameras(self):
        # повёрнут по часовой - правый борт смотрит назад, левый - вперёд
        self.assertLess(wl.turned_mark_pos(20.0, "right", 0.0), wl.mark_pos_mm(0.0, 0.0) - 30)
        self.assertGreater(wl.turned_mark_pos(20.0, "left", 0.0), wl.mark_pos_mm(0.0, 0.0) + 30)
        # сверено с геометрией sim/fake_hw.view: поворот 10°, метка справа в -60 мм
        self.assertAlmostEqual(wl.turned_mark_pos(10.0, "right", wl.px_to_mm(0.62)), -55, delta=15)
