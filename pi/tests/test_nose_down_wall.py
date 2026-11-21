"""Заезд 24.10: нос клюнул на 12° в 20см от стены - робот принял это за
начало горки вниз, не затормозил в центре клетки, въехал в стену, а потом
закрыл бампером единственный выход и закончил разведку."""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from test_reliability import load, TIMING


class FloorOrWallTest(unittest.TestCase):
    def setUp(self):
        self.front = load("drive_front", {**{n: Mock() for n in (
            "bumper", "tile_rules", "turning", "corridor", "esp_link", "lidar_walls")},
            "state": SimpleNamespace(lifted=threading.Event()),
            "walls": SimpleNamespace(SIDE_THRESH=200, WALL_THRESH=260)})

    def test_close_range_with_nose_down_is_a_wall(self):
        self.assertTrue(self.front.wall_not_floor(190, -12.0))    # пол был бы в ~610мм

    def test_floor_distance_or_nose_up_is_not_trusted(self):
        self.assertFalse(self.front.wall_not_floor(600, -12.0))
        self.assertFalse(self.front.wall_not_floor(190, +12.0))   # нос вверх: горка как стена
        self.assertFalse(self.front.wall_not_floor(None, -12.0))

    def test_front_wall_kept_on_ramp_candidate_with_nose_down(self):
        self.front.lidar_walls.get_wall_snapshot.return_value = (5, 10.0, {"front": 190}, .1)
        self.front.lidar_walls.get_timed_back.return_value = (None, None)
        odo = SimpleNamespace(at_checked=lambda t: 20.0)
        trip = SimpleNamespace(on_slope_now=True, p_abs=8.3, pitch_delta=-12.0, ramp_state="cand",
                               last_seq=4, f_anchor=None, traveled_cm=20.0, odo_t=odo)
        with patch.object(self.front.time, "monotonic", return_value=10.05):
            self.front.track_front_wall(trip)
        self.assertAlmostEqual(trip.f_est, 190)


class StopOnCandidateTest(unittest.TestCase):
    def test_wall_ahead_keeps_planned_stop(self):
        import continuous as ramp
        self.assertTrue(ramp.stop_suspended(SimpleNamespace(ramp_state="cand", f_est=None)))
        self.assertFalse(ramp.stop_suspended(SimpleNamespace(ramp_state="cand", f_est=190.0)))
        self.assertFalse(ramp.stop_suspended(SimpleNamespace(ramp_state=None, f_est=None)))


class ExploreSoftBlocksTest(unittest.TestCase):
    def module(self, forgotten):
        deps = {n: Mock() for n in ("bumper", "dash_live", "driving", "esp_link", "floor_tiles",
            "lidar_walls", "marks", "maze_map", "ramps", "recovery", "tile_rules", "turning", "victims")}
        deps.update(state=SimpleNamespace(PER_CELL_STOP=False, ramps={}, lifted=threading.Event()),
                    timing=TIMING)
        m = load("explore", deps)
        m.arrive_explore = Mock()
        m.maze_map.forget_soft_blocks.side_effect = forgotten
        return m

    def test_soft_closed_exit_is_forgotten_before_finishing(self):
        m = self.module([1, 0])
        m.step = Mock(side_effect=["no_path", "step", "no_path"])
        status = m.flood_fill_run(300)[-1]
        self.assertEqual(status, "explored")
        self.assertEqual(m.step.call_count, 3)       # после забывания разведка шла дальше

    def test_forgetting_is_limited(self):
        m = self.module([1] * 10)
        m.step = Mock(return_value="no_path")
        self.assertEqual(m.flood_fill_run(300)[-1], "explored")
        self.assertEqual(m.maze_map.forget_soft_blocks.call_count, m.SOFT_FORGET_MAX)


class SideBumperMapTest(unittest.TestCase):
    """Заезд 24.10: правый концевик ещё прижат после удара об угол, а замер
    стоя записал стену поперёк прохода (лидар видел впереди 338-394мм)."""
    def walls(self, c1, c2, front):
        st = SimpleNamespace(lock=threading.Lock(), telem={"clicker1": c1, "clicker2": c2})
        bumper = load("bumper", {"state": st, "driving": Mock(), "esp_link": Mock(), "turning": Mock()})
        mm = load("maze_map", {"state": SimpleNamespace(), "bumper": bumper, "tile_rules": Mock(),
            "lidar_walls": SimpleNamespace(wall_present=lambda d, t: d is not None and d < t),
            "walls": SimpleNamespace(SIDE_THRESH=200)})
        return mm._walls_rel_map({"front": front, "right": None, "left": 140, "back": 130})

    def test_one_clicker_does_not_make_front_wall(self):
        self.assertFalse(self.walls(False, True, 360)["front"])

    def test_both_clickers_mean_wall_even_if_lidar_misses_it(self):
        self.assertTrue(self.walls(True, True, None)["front"])

    def test_lidar_wall_still_counts(self):
        self.assertTrue(self.walls(False, False, 150)["front"])


class TurnMarkReadWallTest(unittest.TestCase):
    """Заезд 24.10: Ψ, уже прочитанная на ходу, на обратном пути снова
    попала в кадр на повороте - доворот и три пустых подъезда."""
    def place(self, scanned, duplicate=False):
        st = SimpleNamespace(scanned_walls=scanned, maze={(0, 1): [True, False, False, True]},
                             marks=SimpleNamespace(near=lambda *a: ("W", 312.0, "read")
                                                   if duplicate else None))
        deps = {n: Mock() for n in ("driving", "esp_link", "maze_map", "turning", "victims")}
        deps.update(state=st, timing=TIMING, walls=SimpleNamespace(SIDE_THRESH=200),
            lidar_walls=Mock(), victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=10,
                                                              RING_SCORE={}, SUM_TO_TYPE={}))
        m = load("marks", deps)
        m.watch_logic = SimpleNamespace(turned_mark_pos=lambda *a: 100.0, px_to_mm=lambda v: v,
            mark_center_y=lambda b: 0, world_along=lambda *a: ("W", 312.0),
            wall_center_along=lambda *a: 300.0, CELL_MM=300.0)
        m._corner_dup = lambda *a: False
        return m._turn_mark_place(0, 1, "right", 3, 46, (0, 0, 46, 46), 2, 13.0, "psi")

    def test_corner_on_scanned_wall_is_still_tried(self):
        self.assertIsNotNone(self.place({(0, 1, 3)}))

    def test_known_mark_on_scanned_wall_is_skipped(self):
        self.assertIsNone(self.place({(0, 1, 3)}, duplicate=True))


class TurnMarkMarksWallTest(unittest.TestCase):
    def test_wall_read_on_turn_is_not_scanned_again(self):
        st = SimpleNamespace(scanned_walls=set(), victim_log=[],
                             marks=SimpleNamespace(near=lambda *a: None, add=lambda *a: None))
        deps = {n: Mock() for n in ("driving", "esp_link", "maze_map", "turning", "victims", "lidar_walls")}
        deps.update(state=st, timing=TIMING, walls=SimpleNamespace(SIDE_THRESH=200),
            victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=10, RING_SCORE={}, SUM_TO_TYPE={}))
        m = load("marks", deps)
        m._corner_dup = lambda *a: False
        result = {"label": "+2", "class": "cognetive", "blink": False}
        m._record_turn_mark(None, 0, 2, "left", 0, 1, "N", -39.0, result, None)
        self.assertIn((0, 2, 0), st.scanned_walls)


class FarWallMarkTest(unittest.TestCase):
    """Заезд 24.10: подъезды туда-сюда ради меток на дальней стене."""
    def module(self):
        st = SimpleNamespace(lifted=threading.Event(), watcher=None, victim_l=Mock(), victim_r=Mock(),
                             maze={}, marks=SimpleNamespace(add=Mock()))
        deps = {n: Mock() for n in ("driving", "esp_link", "maze_map", "turning", "victims", "lidar_walls")}
        deps.update(state=st, timing=TIMING, walls=SimpleNamespace(SIDE_THRESH=200),
            victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40, RING_SCORE={}, SUM_TO_TYPE={}))
        m = load("marks", deps)
        m._interrupt_reason = Mock(return_value=None)
        return m, st

    def test_no_readable_mark_in_frame_stops_rereads(self):
        m, st = self.module()
        det = SimpleNamespace(best_fast=lambda: None, last_unread=None)   # мельче 40px отсеяна
        m._read_wall = Mock(return_value=None)
        result, unread, moved = m._reread_aligned(det, (1, 3), 1, 0, 0.0, "test", 3)
        self.assertIsNone(result)
        m.driving.creep_soft.assert_not_called()

    def test_mark_in_frame_still_nudges(self):
        m, st = self.module()
        det = SimpleNamespace(best_fast=lambda: ("cognetive", .9, (0.3, 0.7)), last_unread=None)
        m._read_wall = Mock(return_value=None)
        m.driving.creep_soft.return_value = 25.0
        m._reread_aligned(det, (1, 3), 1, 0, 0.0, "test", 1)
        m.driving.creep_soft.assert_called()


class MarkStopCenterTest(unittest.TestCase):
    """Заезд 24.10: после метки колёса - "за центром 122мм", стена - в центре."""
    def left(self, front, traveled=72.2, boundary=60.0):
        lidar = Mock(); lidar.get_walls.return_value = {"front": front}
        m = load("drive_stop", {"state": Mock(), "esp_link": Mock(), "driving": Mock(),
                                "marks": Mock(), "drive_checks": Mock(), "lidar_walls": lidar})
        return m._left_to_center_cm(SimpleNamespace(boundary_cm=boundary, traveled_cm=traveled))

    def test_front_wall_beats_slipping_wheels(self):
        self.assertAlmostEqual(self.left(152.0), 0.7)

    def test_no_close_wall_keeps_odometry(self):
        self.assertAlmostEqual(self.left(None), -12.2)
        self.assertAlmostEqual(self.left(600.0), -12.2)


class StartOffsetTest(unittest.TestCase):
    """Доводка после поворота - поправкой цели поездки, без подъезда."""
    def module(self, off):
        st = SimpleNamespace(start_offset=off)
        turning = SimpleNamespace(angle_err=lambda a, b: (a - b + 180) % 360 - 180)
        m = load("drive_trip", {"esp_link": Mock(), "state": st, "turning": turning,
            "walls": SimpleNamespace(MASKS=[]), "corridor": Mock()})
        return m, st

    def test_same_heading_uses_offset_once(self):
        m, st = self.module((-37.0, 90.0))
        self.assertAlmostEqual(m._start_offset_cm(90.5), -3.7)
        self.assertIsNone(st.start_offset)
        self.assertEqual(m._start_offset_cm(90.0), 0.0)

    def test_other_heading_ignores_offset(self):
        m, st = self.module((52.0, 90.0))
        self.assertEqual(m._start_offset_cm(180.0), 0.0)
        self.assertIsNone(st.start_offset)


class WatchedWallsTest(unittest.TestCase):
    """Скан стоя пропускает стены, которые наблюдатель смотрел у центра."""
    def setUp(self):
        self.st = SimpleNamespace(watched_sides=set(), lifted=threading.Event())
        deps = {n: Mock() for n in ("driving", "esp_link", "maze_map", "turning", "victims", "lidar_walls")}
        deps.update(state=self.st, timing=TIMING, walls=SimpleNamespace(SIDE_THRESH=200),
            victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40, RING_SCORE={}, SUM_TO_TYPE={}))
        self.m = load("marks", deps)

    def trip(self, traveled, armed=True):
        return SimpleNamespace(boundary_cm=30.0, traveled_cm=traveled, watch_boundary=30.0,
                               watch_armed=armed)

    def test_near_center_with_frames_counts(self):
        self.m._note_watched(self.trip(27.0), {"left"}, 0.1)
        self.assertEqual(self.st.watched_sides, {"left"})

    def test_far_from_center_or_slow_frames_or_off_do_not_count(self):
        self.m._note_watched(self.trip(15.0), {"left"}, 0.1)
        self.m._note_watched(self.trip(29.0), {"left"}, 0.6)
        self.m._note_watched(self.trip(29.0, armed=False), {"left"}, 0.1)
        self.assertEqual(self.st.watched_sides, set())

    def test_new_target_forgets_previous_cell(self):
        t = self.trip(29.0)
        self.m._note_watched(t, {"right"}, 0.1)
        t.boundary_cm, t.traveled_cm = 60.0, 40.0
        self.m._note_watched(t, {"right"}, 0.1)
        self.assertEqual(self.st.watched_sides, set())


class TripPitchZeroTest(unittest.TestCase):
    """Заезд 24.10: доводка задом заехала на препятствие (+7.6°), этот
    наклон стал нулём поездки, и съезд на пол приняли за горку вниз."""
    def zero(self, pitch_now, level=-0.1):
        link = Mock()
        link.get_pitch.return_value = pitch_now
        m = load("drive_trip", {"esp_link": link, "state": SimpleNamespace(pitch_level=level),
            "turning": Mock(), "walls": SimpleNamespace(MASKS=[]), "corridor": Mock()})
        return m._pitch_zero()

    def test_tilted_start_uses_level_floor(self):
        self.assertEqual(self.zero(7.6), -0.1)

    def test_level_start_keeps_current_pitch(self):
        self.assertEqual(self.zero(0.9), 0.9)


if __name__ == "__main__":
    unittest.main()
