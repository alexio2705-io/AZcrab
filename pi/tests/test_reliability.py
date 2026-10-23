"""Fault injection against real controller code with hardware boundaries replaced."""
import binascii
from collections import deque
import importlib.util
import math
from pathlib import Path
import struct
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))
import config
import runplan
import watch_logic


def load(name, dependencies):
    spec = importlib.util.spec_from_file_location("tested_" + name, PI / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, dependencies):
        spec.loader.exec_module(module)
    return module


TIMING = SimpleNamespace(_timed=lambda name: lambda fn: fn)


class RouteActionTests(unittest.TestCase):
    def setUp(self):
        self.map = Mock()
        self.state = SimpleNamespace(tacc={}, blue_cells=set(), tile_map={})
        self.rules = SimpleNamespace(STOP="stop", role=lambda color: "stop")
        self.actions = load("actions", {**{n: Mock() for n in
            ("leds", "dash_live", "esp_link", "explore", "victims")},
            "state": self.state, "maze_map": self.map, "tile_rules": self.rules})
        self.walk = SimpleNamespace(cell=(0, 0), hdg=1, total=0, cx=0, cy=0)

    def test_nearest_uses_one_weighted_search_and_ignores_unreachable(self):
        cells = {(0, 0), (1, 0), (2, 0), (0, 1), (0, 2)}
        def nxt(x, y, d): return x + config._DX[d], y + config._DY[d]
        def opened(x, y): return [d for d in range(4) if nxt(x, y, d) in cells]
        def find(start, goal, heading):
            return runplan.weighted_path(start, heading, opened, nxt, goal, (0, 1, 3, 2),
                lambda src, dst, h, d: 30 if dst == (1, 0) else 1)
        self.map.find_path_bfs.side_effect = find
        self.map._next_cell.side_effect = nxt
        self.assertEqual(self.actions.nearest(self.walk, [(1, 0), (0, 2), (-1, 0)]), (0, 2))
        self.map.find_path_bfs.assert_called_once()

    def test_no_target_does_not_start_motion_or_work(self):
        self.map.find_path_bfs.return_value = None
        work = Mock()
        result = self.actions.visit_targets(self.walk, lambda: [(8, 8)], work,
                                            lambda: False, lambda: False)
        self.assertFalse(result)
        work.assert_not_called()
        self.actions.explore.go_to.assert_not_called()

    def test_unreachable_return_is_not_a_short_manhattan_estimate(self):
        self.map.find_path_bfs.return_value = None
        timer = self.actions.Timer(420)
        self.assertEqual(timer.trip_s(self.walk, (8, 8)), math.inf)
        self.assertTrue(timer.time_to_go(self.walk, (8, 8)))

    def test_return_estimate_includes_blue_wait_once_per_entry(self):
        self.map.find_path_bfs.return_value = [1, 1]
        self.map._next_cell.side_effect = lambda x, y, d: (x + 1, y)
        self.state.blue_cells.add((1, 0))
        expected = 2 * self.actions.SEC_PER_CELL * self.actions.TRIP_MARGIN + config.BLUE_WAIT_S
        self.assertAlmostEqual(self.actions.Timer(420).trip_s(self.walk, (2, 0)), expected)
        self.map.find_path_bfs.assert_called_once()


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.state = SimpleNamespace(lock=threading.Lock(), telem={}, telem_seq=0, telem_t=0,
            telem_evt=threading.Event(), esp_running=threading.Event(), lifted=threading.Event(),
            link_stop=threading.Event(), magic_gaps={}, bad_ver=0, bad_crc=0, seq_lost=0,
            motion_history=deque(maxlen=4096))
        self.module = load("esp_link", {"state": self.state, "ramps": Mock(),
            "serial": SimpleNamespace(SerialException=OSError)})

    def packet(self, boot, seq, yaw=0.0):
        vals = [4, seq] + [yaw, 0., 0., 0., 0., 0., 0.] + [1] + [11, 12, 13, 14]
        vals += [0., 0., 0, 127, 0, boot, 0]
        vals += [160] * 4 + [25] * 4 + [0] * 4 + [0]
        body = bytearray(struct.pack(config.FMT, *vals))
        body[-2:] = struct.pack("<H", binascii.crc_hqx(body[:-2], 0xffff))
        return b"\xaa\x55" + body

    def receive(self, packets):
        chunks = [b"".join(packets)]
        def read(size):
            if chunks: return chunks.pop()
            self.state.link_stop.set()
            return b""
        self.state.link = SimpleNamespace(in_waiting=4096, read=read)
        self.module._telem_thread()

    def test_reboot_requires_coordinate_recovery(self):
        self.receive([self.packet(42, 1), self.packet(43, 0)])
        self.assertTrue(self.state.lifted.is_set())
        self.assertEqual(self.state.telem_seq, 2)

    def test_valid_crc_nan_does_not_refresh_sensor_data(self):
        self.receive([self.packet(42, 1, 12.0), self.packet(42, 2, float("nan"))])
        self.assertEqual(self.state.telem_seq, 1)
        self.assertEqual(self.state.telem["yaw"], 12.0)

    def test_live_uart_with_frozen_gyro_interrupts_motion(self):
        self.state.esp_running.set()
        self.state.telem_seq = 1
        self.state.telem_t = time.monotonic()
        self.state.telem["gyro_fresh"] = False
        self.assertTrue(self.module.telem_fresh())
        self.assertTrue(self.module.interrupted())
        self.state.telem["gyro_fresh"] = True
        self.assertFalse(self.module.interrupted())

    def test_shutdown_cancels_port_reconnection(self):
        self.state.link_stop.set()
        self.state.link = Mock(is_open=True)
        self.assertFalse(self.module._link_reopen())
        self.state.link.open.assert_not_called()


class CognitiveTests(unittest.TestCase):
    @staticmethod
    def reading(colors, ok=None):
        return {"rings": colors, "ring_ok": ok or [True] * len(colors),
                "ring_agree": [0.9] * len(colors)}

    def test_unseen_ring_is_not_yellow(self):
        self.assertEqual(watch_logic.guess_rings([self.reading(["red"])]), ["red"] + ["?"] * 4)

    def test_reliable_partial_views_combine(self):
        a = self.reading(["red", "blue", "?", "?", "?"])
        b = self.reading(["?", "?", "yellow", "green", "black"])
        self.assertEqual(watch_logic.guess_rings([a, b]), ["red", "blue", "yellow", "green", "black"])

    def test_conflicting_or_weak_readings_remain_unknown(self):
        self.assertEqual(watch_logic.guess_rings([
            self.reading(["red"]), self.reading(["blue"])])[0], "?")
        self.assertEqual(watch_logic.guess_rings([self.reading(["blue"], [False])])[0], "?")
        self.assertEqual(watch_logic.guess_rings([{"rings": ["red"], "ring_ok": []}])[0], "?")

    def test_incomplete_target_cannot_authorize_signalling(self):
        deps = {n: Mock() for n in ("state", "driving", "esp_link", "lidar_walls", "maze_map", "turning", "victims")}
        deps.update({"timing": TIMING, "walls": SimpleNamespace(SIDE_THRESH=200),
            "victim_detect": SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=10,
                RING_SCORE={"black": -2, "red": -1, "yellow": 0, "green": 1, "blue": 2},
                SUM_TO_TYPE=config.COG_SUM_TO_TYPE)})
        module = load("marks", deps)
        unread = self.reading(["blue"])
        self.assertIsNone(module._decide_unread(SimpleNamespace(unread_hist=[unread]), unread))


class LidarAndStartTests(unittest.TestCase):
    def test_live_scans_without_walls_are_not_a_lidar_outage(self):
        state = SimpleNamespace(lifted=threading.Event())
        link = Mock()
        lidar = Mock()
        lidar.get_wall_snapshot.return_value = (12, time.monotonic(),
                                               dict.fromkeys(("front", "back", "left", "right")), .1)
        module = load("drive_front", {**{n: Mock() for n in ("bumper", "tile_rules", "turning", "corridor")},
            "state": state, "esp_link": link, "lidar_walls": lidar,
            "walls": SimpleNamespace(SIDE_THRESH=200, WALL_THRESH=260)})
        trip = SimpleNamespace(on_slope_now=False, p_abs=0, last_seq=11, f_anchor=None,
                               traveled_cm=1, f_est=None, odo_t=watch_logic.OdoHistory())
        self.assertIsNone(module.track_front_wall(trip))
        self.assertFalse(state.lifted.is_set())
        lidar.get_wall_snapshot.return_value = (12, time.monotonic()-1, trip.walls, .1)
        self.assertEqual(module.track_front_wall(trip), "stop")
        self.assertEqual(trip.reason, "lidar_lost")

    def test_lidar_timeout_waits_instead_of_ending_run(self):
        """Лидар молчит - ждём его (не выход) и меряем уже новый скан."""
        state = SimpleNamespace(walls_cache=None)
        module = load("lidar_walls", {**{n: Mock() for n in ("esp_link", "maze_map", "turning")},
            "state": state, "timing": TIMING,
            "ld19": SimpleNamespace(BAUDRATE=230400, DEFAULT_PORT="unused")})
        events = []
        module.wait_scan = Mock(side_effect=lambda n=2, timeout=3.0: events.append("scan") or False)
        module.wait_lidar_back = Mock(side_effect=lambda: events.append("wait"))
        module.get_walls_still = Mock(side_effect=lambda: events.append("measure") or
                                      {"front": 150, "back": None, "left": None, "right": None})
        out = module.measure_walls(1)
        self.assertEqual(events, ["scan", "wait", "measure"])
        self.assertEqual(out["front"], 150)

    def test_unknown_is_not_an_open_wall_vote(self):
        state = SimpleNamespace(edge_fixed={}, wall_votes={}, maze={})
        module = load("maze_map", {**{n: Mock() for n in ("bumper", "lidar_walls", "tile_rules")},
            "state": state, "walls": SimpleNamespace(SIDE_THRESH=200)})
        module.vote_wall(0, 0, 0, None)
        self.assertEqual(state.wall_votes, {})
        self.assertEqual(state.maze, {})

    def test_no_odometry_ack_retries_before_motion(self):
        """ESP не подтвердил Z - тормоз и повтор, а не выход из заезда."""
        link = Mock()
        link.wait_odo_zeroed.side_effect = [False, False, True]
        module = load("drive_trip", {"esp_link": link, "state": Mock(), "turning": Mock(),
            "walls": SimpleNamespace(MASKS=[]), "corridor": Mock()})
        with patch.object(module.time, "sleep"):
            module._zero_odometry()
        self.assertEqual(link.zero_odo.call_count, 3)
        self.assertEqual(link.brake.call_count, 2)
        link.motors.assert_not_called()

    def test_offline_encoder_stops_before_odometry_integration(self):
        link = Mock()
        link.get_esp_paused.return_value = False
        link.telem_fresh.return_value = link.gyro_fresh.return_value = True
        link.encoders_online.return_value = False
        module = load("drive_checks", {"state": SimpleNamespace(lifted=threading.Event()), "esp_link": link})
        trip = SimpleNamespace()
        self.assertEqual(module.check_abort(trip), "stop")
        self.assertEqual(trip.reason, "odo_fault")
        link.get_odo.assert_not_called()


class NavigationTests(unittest.TestCase):
    def setUp(self):
        deps = {n: Mock() for n in ("bumper", "dash_live", "driving", "esp_link", "floor_tiles",
            "lidar_walls", "marks", "maze_map", "ramps", "recovery", "tile_rules", "turning", "victims")}
        deps.update(state=SimpleNamespace(PER_CELL_STOP=True, ramps={}, lifted=threading.Event()), timing=TIMING)
        self.module = load("explore", deps)

    def test_per_cell_stop_flag_reaches_motor_planner(self):
        self.module.driving.drive_cells.return_value = (0, "done")
        walk = SimpleNamespace(cx=0, cy=0, hdg=0, cell=(0, 0), max_cells=300, total=0)
        self.module._drive(walk, lambda c: False, False, None)
        self.assertEqual(self.module.driving.drive_cells.call_args.args[0], 1)

    def test_continuous_mode_does_not_limit_trip_to_one_cell(self):
        self.module.state.PER_CELL_STOP = False
        self.module.driving.drive_cells.return_value = (0, "done")
        walk = SimpleNamespace(cx=0, cy=0, hdg=0, cell=(0, 0), max_cells=300, total=12)
        self.module._drive(walk, lambda c: False, False, None)
        self.assertEqual(self.module.driving.drive_cells.call_args.args[0], 288)

    def test_empty_route_is_goal_not_index_error(self):
        self.module._wait_robot = Mock(return_value=False)
        self.module.maze_map.find_path_bfs.return_value = []
        self.assertEqual(self.module.step(SimpleNamespace(cell=(0, 0), hdg=0), lambda c: True), "goal")

    def test_lost_pose_never_triggers_arrival_scan_or_map_write(self):
        self.module.state.lifted.set()
        self.module.driving.drive_cells.return_value = (1, "pose_lost")
        walk = SimpleNamespace(cx=0, cy=0, hdg=0, cell=(0, 0), max_cells=300, total=0)
        arrive = Mock()
        self.module._drive(walk, lambda c: False, False, arrive)
        arrive.assert_not_called()
        self.module.maze_map.open_edge.assert_not_called()
        self.module._close_soft(walk)
        self.module.maze_map.block_direction.assert_not_called()


class BootCleanupTests(unittest.TestCase):
    def setUp(self):
        self.gpio = Mock()
        self.state = SimpleNamespace(link=None, link_stop=threading.Event(), lidar_stop=threading.Event(),
            lidar_proc=None, lidar_dropped=0, lidar_restarts=0, watcher=Mock(), victim_r=Mock(),
            victim_l=Mock(), victim_f=Mock(), leds=Mock())
        deps = {n: Mock() for n in ("front_victim", "leds", "victim_detect", "victim_watch", "timing",
            "dash_live", "esp_link", "floor_tiles", "lidar_walls", "recovery", "tile_rules")}
        deps.update({"state": self.state, "RPi": SimpleNamespace(GPIO=self.gpio), "RPi.GPIO": self.gpio,
                     "serial": SimpleNamespace(SerialException=OSError)})
        deps["esp_link"].link_errors.return_value = "test link"
        deps["timing"].time_report.return_value = "test timings"
        deps["timing"].tick_report.return_value = "test ticks"
        self.module = load("boot", deps)

    def test_continuous_is_default_and_tile_stops_are_explicit(self):
        self.module._configure_motion([])
        self.assertFalse(self.state.PER_CELL_STOP)
        self.module._configure_motion(['--stop-each-cell'])
        self.assertTrue(self.state.PER_CELL_STOP)
        self.module._configure_motion(['--nonstop'])
        self.assertFalse(self.state.PER_CELL_STOP)
        with self.assertRaises(ValueError):
            self.module._configure_motion(['--nonstop', '--stop-each-cell'])

    def test_partial_initialization_failure_runs_cleanup(self):
        self.module._start = Mock(side_effect=RuntimeError("camera failed"))
        with self.assertRaisesRegex(RuntimeError, "camera failed"):
            self.module.start([])
        self.assertTrue(self.state.link_stop.is_set())
        self.assertTrue(self.state.lidar_stop.is_set())
        self.gpio.cleanup.assert_called_once()

    def test_one_camera_cleanup_failure_does_not_skip_other_resources(self):
        self.state.victim_r.close.side_effect = OSError("camera disconnected")
        self.module.stop(self.module.Run(0, None, time.monotonic()))
        self.state.victim_l.close.assert_called_once()
        self.state.victim_f.close.assert_called_once()
        self.state.leds.close.assert_called_once()
        self.gpio.cleanup.assert_called_once()


if __name__ == "__main__":
    unittest.main()
