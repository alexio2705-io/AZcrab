"""Проверка окна камеры на танковом повороте без железа."""
import unittest
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

from test_reliability import load, TIMING


class TurnCornerTest(unittest.TestCase):
    def setUp(self):
        deps: dict[str, object] = {n: Mock() for n in
                                   ("driving", "esp_link", "ramps", "turn_unstick")}
        lidar = Mock()
        deps["lidar_walls"] = lidar
        deps.update(state=SimpleNamespace(YAW_ORIGIN=0.0), timing=TIMING,
                    victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40),
                    walls=SimpleNamespace(BACK_THRESH=260, SIDE_THRESH=210, WALL_THRESH=260))
        lidar.wall_present.side_effect = lambda d, t: d is not None and d < t
        self.turning = load("turning", deps)

    def test_right_and_left_quarter_turns(self):
        side = self.turning.corner_watch_side
        self.assertEqual(side(45, 90, True, True, True, {"left": 150}, None, 10),
                         ("left", True, 10))
        self.assertEqual(side(45, -90, True, True, True, {"right": 150}, None, 10),
                         ("right", True, 10))
        quarter = self.turning.corner_quarter
        self.assertEqual(quarter(45, 180, 0), (45, 0, 1))
        self.assertEqual(quarter(135, 180, 0), (45, 1, 2))
        self.assertEqual(quarter(135, -180, 0), (45, 3, 2))
        self.assertIsNone(quarter(135, 90, 0))

    def test_two_walls_of_corner_get_different_directions(self):
        q = self.turning.corner_quarter
        wall = self.turning.corner_frame_wall
        self.assertEqual(wall(q(35, 90, 0), 1), (3, 0))   # W до 45°
        self.assertEqual(wall(q(55, 90, 0), 1), (0, 1))   # N после 45°
        self.assertEqual(wall(q(35, -90, 0), -1), (1, 0)) # E до левого поворота
        self.assertEqual(wall(q(125, 180, 0), 1), (0, 1))
        self.assertEqual(wall(q(145, 180, 0), 1), (1, 2))

    def test_look_only_at_close_wall_near_corner(self):
        side = self.turning.corner_watch_side
        self.assertEqual(side(0, 90, True, True, True, {"left": 150}, None, 10),
                         ("left", False, 10))
        self.assertEqual(side(30, 90, True, True, True, {"left": 150}, None, 10),
                         ("left", True, 10))
        self.assertEqual(side(30, 90, True, False, True, {"left": 150}, None, 10),
                         ("left", False, 10))
        self.assertEqual(side(45, 90, False, True, True, {"left": 150}, 10, 10.1),
                         (None, False, None))
        self.assertEqual(side(45, 90, True, True, True, {"left": 300}, None, 10),
                         (None, False, None))
        self.assertEqual(side(45, 90, True, True, True, {"right": 150}, None, 10),
                         (None, False, None))
        self.assertEqual(side(75, 90, True, True, True, {"left": 150}, None, 10),
                         ("left", True, 10))
        self.assertEqual(side(76, 90, True, True, True, {"left": 150}, 10, 10.1),
                         (None, False, None))

    def test_brief_lidar_gap_keeps_camera_armed_but_rejects_its_frame(self):
        side = self.turning.corner_watch_side
        first = side(40, 90, True, True, True, {"left": 150}, None, 10.0)
        self.assertEqual(first, ("left", True, 10.0))
        gap = side(50, 90, True, True, False, {"left": 150}, first[2], 10.12)
        self.assertEqual(gap, ("left", False, 10.0))
        resumed = side(55, 90, True, True, True, {"left": 150}, gap[2], 10.18)
        self.assertEqual(resumed, ("left", True, 10.18))
        expired = side(60, 90, True, True, False, {"left": 150}, resumed[2], 10.40)
        self.assertEqual(expired, (None, False, 10.18))


class ContinuousHalfTurnTest(unittest.TestCase):
    def test_half_turn_is_one_motor_turn_with_two_corner_windows(self):
        deps: dict[str, object] = {n: Mock() for n in (
            "bumper", "dash_live", "driving", "esp_link", "floor_tiles", "lidar_walls",
            "marks", "maze_map", "ramps", "recovery", "tile_rules", "turning", "victims")}
        deps.update(state=SimpleNamespace(maze={(0, 0): [True] * 4}), timing=TIMING)
        explore = load("explore", deps)
        turning = deps["turning"]
        assert isinstance(turning, Mock)
        turning.snap_target.return_value = 180.0
        turning.turn_to.return_value = True
        turning.angle_err.return_value = 0.0
        esp = deps["esp_link"]
        assert isinstance(esp, Mock)
        esp.get_esp_paused.return_value = False
        victims = deps["victims"]
        assert isinstance(victims, Mock)
        victims.scan_cell.return_value = 2
        walk = explore.Walk()
        self.assertTrue(explore._turn_toward(walk, 2))
        self.assertEqual(walk.hdg, 2)
        turning.turn_to.assert_called_once()
        self.assertEqual(turning.turn_to.call_args.kwargs["turn_sign"], 1)
        turning.align_at_wall.assert_called_once()
        victims.scan_cell.assert_called_once()

    def test_seen_mark_is_served_before_finishing_turn(self):
        deps: dict[str, object] = {n: Mock() for n in (
            "bumper", "dash_live", "driving", "esp_link", "floor_tiles", "lidar_walls",
            "marks", "maze_map", "ramps", "recovery", "tile_rules", "turning", "victims")}
        deps.update(state=SimpleNamespace(maze={(0, 0): [True] * 4}), timing=TIMING)
        explore = load("explore", deps)
        turning = deps["turning"]
        assert isinstance(turning, Mock)
        turning.snap_target.return_value = 90.0
        turning.turn_to.return_value = "mark"
        marks = deps["marks"]
        assert isinstance(marks, Mock)
        marks._turn_marks.return_value = 3
        esp = deps["esp_link"]
        assert isinstance(esp, Mock)
        esp.get_esp_paused.return_value = False
        walk = explore.Walk()
        self.assertFalse(explore._turn_toward(walk, 1))
        self.assertEqual(walk.hdg, 3)
        turning.turn_to.assert_called_once()
        marks._turn_marks.assert_called_once()
        turning.align_at_wall.assert_not_called()
        victims = deps["victims"]
        assert isinstance(victims, Mock)
        victims.scan_cell.assert_not_called()


class StopOnMarkTest(unittest.TestCase):
    def test_motor_turn_stops_at_first_corner_box(self):
        deps: dict[str, object] = {n: Mock() for n in
                                   ("driving", "esp_link", "ramps", "turn_unstick")}
        watcher = Mock(dead=False)
        watcher.cams = {"left": object()}
        watcher.live_info.return_value = {"left": {"fid": 1}}
        watcher.poll.side_effect = [
            [SimpleNamespace(side="left", t_frame=time.monotonic(),
                             xyxy=(100, 100, 180, 180), conf=.9, cls="phi")], []]
        state = SimpleNamespace(YAW_ORIGIN=0.0, watcher=watcher,
                                telem_seq=0, lifted=Mock(), turn_ff=.15)
        deps["state"] = state
        deps["timing"] = TIMING
        deps["config"] = SimpleNamespace(TURN_TOL_DEG=2.0)
        deps["walls"] = SimpleNamespace(BACK_THRESH=260, SIDE_THRESH=210,
                                         WALL_THRESH=260)
        deps["victim_detect"] = SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40)
        hist = Mock()
        hist.at_checked.return_value = 35.0
        deps["watch_logic"] = SimpleNamespace(
            OdoHistory=Mock(return_value=hist), SideHold=Mock(return_value=Mock(frame_ok=Mock(return_value=True))),
            FRAME_PX=320)
        lidar = Mock()
        lidar.get_wall_snapshot.side_effect = lambda: (1, time.monotonic(), {"left": 150}, .1)
        lidar.wall_present.side_effect = lambda d, t: d is not None and d < t
        deps["lidar_walls"] = lidar
        esp = deps["esp_link"]
        assert isinstance(esp, Mock)
        esp.get_yaw.side_effect = [0.0, 0.0, 35.0, 35.0, 35.0, 35.0]
        esp.interrupted.return_value = False
        turning = load("turning", deps)
        seen = []
        self.assertEqual(turning.turn_to(90.0, seen=seen, walls=[True] * 4), "mark")
        self.assertEqual(len(seen), 1)
        esp.motors.assert_not_called()
        esp.settle_brake.assert_called_once()
        watcher.wait_idle.assert_called_once()
        unstick = deps["turn_unstick"]
        assert isinstance(unstick, Mock)
        unstick.Unstick.return_value.return_after_turn.assert_not_called()

    def test_turn_continues_when_camera_on_corner_side_is_missing(self):
        deps: dict[str, object] = {n: Mock() for n in
                                   ("driving", "esp_link", "ramps", "turn_unstick")}
        watcher = Mock(dead=False)
        watcher.cams = {"left": object()}
        watcher.live_info.return_value = {"left": {"fid": 1}}
        state = SimpleNamespace(YAW_ORIGIN=0.0, watcher=watcher,
                                telem_seq=0, lifted=Mock(), turn_ff=.15)
        deps.update(state=state, timing=TIMING,
                    walls=SimpleNamespace(BACK_THRESH=260, SIDE_THRESH=210,
                                          WALL_THRESH=260),
                    victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40))
        esp = deps["esp_link"]
        assert isinstance(esp, Mock)
        esp.get_yaw.return_value = 90.0
        esp.interrupted.return_value = True
        esp.get_esp_paused.return_value = True
        turning = load("turning", deps)
        self.assertFalse(turning.turn_to(0.0, seen=[], walls=[True] * 4))
        watcher.live_info.assert_not_called()
        watcher.arm.assert_not_called()


class CornerWallAttributionTest(unittest.TestCase):
    def test_frame_at_45_degrees_belongs_to_front_wall(self):
        deps: dict[str, object] = {n: Mock() for n in
                                   ("driving", "esp_link", "lidar_walls", "maze_map", "victims")}
        deps.update(state=SimpleNamespace(YAW_ORIGIN=0.0, turn_mark_tried=set()),
                    timing=TIMING, walls=SimpleNamespace(SIDE_THRESH=210),
                    victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40,
                                                  RING_SCORE={}, SUM_TO_TYPE={}))
        turning = Mock()
        turning.angle_err.side_effect = lambda cur, tgt: (cur - tgt + 180) % 360 - 180
        deps["turning"] = turning
        marks = load("marks", deps)
        seen = [("left", 45.0, (100, 100, 160, 160), .8, "psi", 1, 0)]
        best = marks._best_turn_frames(0, 0, seen)
        self.assertIn(("left", 0), best)
        self.assertEqual(best[("left", 0)][3:5], (1, -45.0))

    def test_side_wall_frame_is_kept_after_approach_scan(self):
        deps: dict[str, object] = {n: Mock() for n in
                                   ("driving", "esp_link", "lidar_walls", "maze_map", "victims")}
        deps.update(state=SimpleNamespace(YAW_ORIGIN=0.0, turn_mark_tried=set(),
                                          scanned_walls={(0, 1, 3)}),
                    timing=TIMING, walls=SimpleNamespace(SIDE_THRESH=210),
                    victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40,
                                                  RING_SCORE={}, SUM_TO_TYPE={}))
        turning = Mock()
        turning.angle_err.side_effect = lambda cur, tgt: (cur - tgt + 180) % 360 - 180
        deps["turning"] = turning
        marks = load("marks", deps)
        seen = [("left", 35.0, (100, 100, 160, 160), .8, "psi", 0, 3)]
        best = marks._best_turn_frames(0, 1, seen)
        self.assertIn(("left", 3), best)
        self.assertEqual(best[("left", 3)][3:5], (0, 35.0))

    def test_confirmed_mark_is_read_without_returning_to_frame_yaw(self):
        deps: dict[str, object] = {n: Mock() for n in
                                   ("driving", "esp_link", "lidar_walls", "maze_map", "victims")}
        deps.update(state=SimpleNamespace(YAW_ORIGIN=0.0), timing=TIMING,
                    walls=SimpleNamespace(SIDE_THRESH=210),
                    victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40,
                                                  RING_SCORE={}, SUM_TO_TYPE={}))
        turning = Mock()
        turning.angle_err.side_effect = lambda cur, tgt: (cur - tgt + 180) % 360 - 180
        deps["turning"] = turning
        esp = deps["esp_link"]
        assert isinstance(esp, Mock)
        esp.get_yaw.return_value = 35.0
        marks = load("marks", deps)
        det = SimpleNamespace(last_unread=None, unread_hist=[])
        with patch.object(marks, "_read_wall", return_value={"box_y": (.4, .6)}) as read_wall:
            result, _, _, _ = marks._read_turn_mark(det, 0, 1, "left", 3, 0,
                                                    60.0, (3, 0, 0), 350.0, 28.0)
        self.assertIsNotNone(result)
        read_wall.assert_called_once()
        turning.turn_to.assert_not_called()


if __name__ == "__main__":
    unittest.main()
