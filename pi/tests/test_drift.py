"""Motion traces and fault injection; no motors or physical sensors required."""
from collections import deque
import math
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from test_reliability import load, TIMING
from motion_estimate import MotionSample, ProjectedOdometry, MotionDiscontinuity
from motion_guard import CenterFilter, SlipGuard
from watch_logic import OdoHistory
import corridor


def sample(seq, t, left=0, right=None, yaw=0, zero=0, ready=True):
    return MotionSample(seq, t, left, left if right is None else right, yaw, zero, ready)


class ProjectionTests(unittest.TestCase):
    def test_straight_and_reverse(self):
        p = ProjectedOdometry(sample(0, 10), 0)
        self.assertAlmostEqual(p.update(sample(1, 10.1, 5)), 5)
        self.assertAlmostEqual(p.update(sample(2, 10.2, 2)), -3)

    def test_constant_heading_is_projected_on_trip_axis(self):
        p = ProjectedOdometry(sample(0, 10, yaw=20), 0)
        self.assertAlmostEqual(p.update(sample(1, 10.1, 5, yaw=20)), 5*math.cos(math.radians(20)))

    def test_curve_and_wrap_are_continuous(self):
        p = ProjectedOdometry(sample(0, 10, yaw=359), 0)
        self.assertAlmostEqual(p.update(sample(1, 10.1, 5, yaw=1)),
                               5*math.sin(math.radians(1))/math.radians(1))

    def test_turn_and_sideways_creep_do_not_count_as_forward(self):
        p = ProjectedOdometry(sample(0, 10), 0)
        self.assertEqual(p.update(sample(1, 10.15, -2, 2, 90)), 0)
        self.assertAlmostEqual(p.update(sample(2, 10.25, 3, 7, 90)), 0)
        self.assertEqual(p.update(sample(3, 10.4, 5, 5, 0)), 0)
        self.assertAlmostEqual(p.update(sample(4, 10.5, 8, 8, 0)), 3)

    def test_duplicates_do_not_add_distance_twice(self):
        first = sample(0, 10)
        p = ProjectedOdometry(first, 0)
        self.assertEqual(p.update(first), 0)

    def test_bad_samples_never_change_integrator_state(self):
        first = sample(0, 10)
        invalid = [sample(2, 10.1, 1), sample(1, 10.5, 1), sample(1, 10, 1),
                   sample(1, 10.1, 1, zero=1), sample(1, 10.1, 1, ready=False),
                   sample(1, 10.01, 20), sample(1, 10.01, yaw=90),
                   sample(1, 10.1, float('nan'))]
        for bad in invalid:
            with self.subTest(bad=bad):
                p = ProjectedOdometry(first, 0)
                with self.assertRaises(MotionDiscontinuity): p.update(bad)
                self.assertEqual(p.last, first)

    def test_invalid_initial_sample_rejected(self):
        for s in (sample(0, 10, ready=False), sample(0, 10, yaw=float('nan'))):
            with self.assertRaises(MotionDiscontinuity): ProjectedOdometry(s, 0)


class HistoryTests(unittest.TestCase):
    def test_old_camera_frame_not_attached_to_present_pose(self):
        h = OdoHistory()
        h.add(10, 0)
        h.add(10.1, 5)
        self.assertAlmostEqual(h.at_checked(10.05), 2.5)
        self.assertIsNone(h.at_checked(9))
        self.assertIsNone(h.at_checked(11))

    def test_each_lidar_ray_uses_its_capture_heading(self):
        points = corridor.to_path_frame([(100, 0), (0, 100)], 10, .1,
            lambda t: 50, 0, lambda t: 90 if t < 9.96 else 0)
        self.assertAlmostEqual(points[0][0], 50)
        self.assertAlmostEqual(points[0][1], 100)
        self.assertAlmostEqual(points[1][0], 50)
        self.assertAlmostEqual(points[1][1], 100)
        self.assertEqual(corridor.to_path_frame([(100, 0)], 10, .1, lambda t: None, 0), [])

    def test_history_overflow_is_explicit(self):
        state = SimpleNamespace(lock=threading.Lock(), motion_history=deque([sample(8, 10), sample(9, 10.1)]))
        module = load('esp_link', {'state': state, 'ramps': Mock(),
            'serial': SimpleNamespace(SerialException=OSError)})
        with self.assertRaises(MotionDiscontinuity): module.motion_since(3)
        self.assertEqual(module.motion_since(8), [sample(9, 10.1)])
        self.assertEqual(module.motion_since(9), [])

    def test_blocking_marker_turn_is_integrated_from_all_samples(self):
        state = SimpleNamespace(lifted=threading.Event())
        link = Mock()
        module = load('drive_checks', {'state': state, 'esp_link': link})
        trip = SimpleNamespace(odometry=ProjectedOdometry(sample(0, 10), 0),
            traveled_cm=20, odo_t=OdoHistory(), yaw_t=OdoHistory())
        trip.odo_t.add(10, 20)
        trip.yaw_t.add(10, 0)
        link.motion_since.return_value = [sample(1, 10.15, -2, 2, 90),
            sample(2, 10.25, 3, 7, 90), sample(3, 10.4, 5, 5), sample(4, 10.5, 8)]
        self.assertIsNone(module.integrate_motion(trip))
        self.assertAlmostEqual(trip.traveled_cm, 23)
        self.assertAlmostEqual(trip.yaw_t.at_checked(10.25), 90)
        self.assertFalse(state.lifted.is_set())
        # Сброс одометрии посреди истории: не стоп и не чекпоинт - новая
        # опора с этого пакета, скачок от "Z" в путь не идёт.
        link.motion_since.return_value = [sample(5, 10.6, 0, zero=1)]
        self.assertIsNone(module.integrate_motion(trip))
        self.assertAlmostEqual(trip.traveled_cm, 23)
        self.assertFalse(state.lifted.is_set())
        link.brake.assert_not_called()
        link.motion_since.return_value = [sample(6, 10.7, 4, zero=1)]
        module.integrate_motion(trip)
        self.assertAlmostEqual(trip.traveled_cm, 27)

    def test_delayed_front_scan_uses_capture_pose(self):
        link, lidar = Mock(), Mock()
        module = load('drive_front', {**{n: Mock() for n in ('bumper', 'tile_rules', 'turning')},
            'state': SimpleNamespace(lifted=threading.Event()), 'esp_link': link, 'lidar_walls': lidar,
            'walls': SimpleNamespace(SIDE_THRESH=200, WALL_THRESH=260)})
        h = OdoHistory()
        h.add(9.9, 0)
        h.add(10.1, 10)
        trip = SimpleNamespace(odo_t=h, on_slope_now=False, p_abs=0, last_seq=0,
            f_anchor=None, traveled_cm=10)
        lidar.get_wall_snapshot.return_value = (1, 10., {'front': 300}, .1)
        with patch.object(module.time, 'monotonic', return_value=10.1):
            module.track_front_wall(trip)
        self.assertAlmostEqual(trip.front_scan_cm, 2.5)
        self.assertAlmostEqual(trip.f_est, 225)
        # A newly started trip cannot attach a scan from before its origin.
        trip.odo_t = OdoHistory()
        trip.odo_t.add(10.1, 0)
        trip.f_anchor = None
        lidar.get_wall_snapshot.return_value = (2, 10., {'front': 300}, .1)
        with patch.object(module.time, 'monotonic', return_value=10.1):
            module.track_front_wall(trip)
        self.assertIsNone(trip.f_est)


class CenterGuardTests(unittest.TestCase):
    def test_single_side_outlier_does_not_steer(self):
        f = CenterFilter()
        self.assertEqual(f.update(1, 5, 'both'), 5)
        self.assertEqual(f.update(2, 60, 'both'), 5)
        self.assertEqual(f.update(3, 6, 'both'), 6)

    def test_only_distinct_scans_confirm_jump_or_side_change(self):
        f = CenterFilter()
        f.update(1, 0, 'both')
        self.assertEqual(f.update(2, 40, 'left'), 0)
        self.assertEqual(f.update(2, 40, 'left'), 0)
        self.assertEqual(f.update(3, 42, 'left'), 42)

    def test_missing_wall_clears_correction(self):
        f = CenterFilter()
        f.update(1, 20, 'both')
        self.assertEqual(f.update(2, None, None), 0)
        self.assertEqual(f.update(3, -20, 'right'), -20)


class PartialSlipTests(unittest.TestCase):
    def update(self, guard, idx, t, wheel, actual, back=True):
        walls = {'front': 700-actual, 'back': 500+actual if back else None}
        return guard.update(idx, t, t-.05, wheel, 0, 0, walls, 35)

    def test_partial_slip_requires_two_independent_confirmations(self):
        g = SlipGuard()
        self.assertFalse(self.update(g, 1, 10, 0, 0))
        self.assertFalse(self.update(g, 2, 10.3, 100, 30))
        self.assertFalse(self.update(g, 2, 10.31, 110, 30))
        self.assertTrue(self.update(g, 3, 10.4, 140, 40))
        self.assertEqual(g.evidence, 'partial_slip')

    def test_normal_motion_and_one_wall_partial_evidence_not_stuck(self):
        for ratio, back in ((1, True), (.4, False)):
            g = SlipGuard()
            for i in range(8):
                self.assertFalse(self.update(g, i, 10+i*.1, i*35, i*35*ratio, back))

    def test_stale_duplicate_discards_anchor(self):
        g = SlipGuard()
        self.update(g, 1, 10, 0, 0)
        self.assertFalse(g.update(1, 10.5, 10, 100, 0, 0, {'front': 700}, 35))
        self.assertIsNone(g.anchor)
        self.assertFalse(self.update(g, 2, 10.6, 120, 0))


class CreepTests(unittest.TestCase):
    def controller(self, moving=True, bumper=False):
        state = SimpleNamespace(telem_seq=0, lifted=threading.Event())
        clock = SimpleNamespace(now=10.)
        samples = [sample(0, 10)]
        link = Mock()
        link.interrupted.side_effect = state.lifted.is_set
        link.encoders_online.return_value = True
        link.get_yaw.return_value = link.get_yaw_rate.return_value = 0
        link.motion_snapshot.side_effect = lambda: samples[-1]
        link.motion_since.side_effect = lambda seq: [s for s in samples if s.seq > seq]
        def advance(seconds):
            clock.now += seconds
            last = samples[-1]
            v = link.motors.call_args.args[0] if moving and link.motors.called else 0
            if link.brake.called: v = 0
            samples.append(sample(last.seq+1, clock.now, last.left+v*seconds))
            return samples[-1].seq
        link.wait_telem.side_effect = lambda seq: advance(.02)
        deps = {n: Mock() for n in ('drive_checks', 'drive_front', 'drive_motion',
            'drive_ramp', 'drive_stop', 'drive_trip', 'floor_tiles', 'marks', 'recovery')}
        deps.update(state=state, timing=TIMING, esp_link=link,
            bumper=SimpleNamespace(get_bumper=lambda: bumper),
            turning=SimpleNamespace(angle_err=lambda a,b: (a-b+180)%360-180))
        module = load('driving', deps)
        module.time = SimpleNamespace(monotonic=lambda: clock.now, sleep=advance)
        return module, state, link

    def test_returns_actual_reverse_distance_for_marker_return(self):
        module, state, link = self.controller()
        actual = module.creep_soft(-30, 0)
        self.assertLess(actual, -20)
        self.assertGreater(actual, -35)
        self.assertFalse(state.lifted.is_set())
        link.brake.assert_called_once()

    def test_no_progress_timeout_reports_measured_zero(self):
        module, state, link = self.controller(moving=False)
        self.assertEqual(module.creep_mm(100, 0, timeout_s=.2), 0)
        self.assertFalse(state.lifted.is_set())     # робот решает сам, без чекпоинта
        link.brake.assert_called_once()

    def test_forward_bumper_abort_not_reported_as_completed(self):
        module, state, link = self.controller(bumper=True)
        self.assertEqual(module.creep_mm(100, 0), 0)
        self.assertFalse(state.lifted.is_set())
        link.motors.assert_not_called()

    def test_reverse_releases_pressed_bumper(self):
        module, state, link = self.controller(bumper=True)
        self.assertLess(module.creep_mm(-30, 0), -15)
        self.assertFalse(state.lifted.is_set())

    def test_rollback_includes_motion_during_braking(self):
        module, state, link = self.controller()
        trip = SimpleNamespace(watcher=None, ramp_state=None, reason='front_obstacle',
                               traveled_cm=6, forced_from_cm=5, target_yaw=0)
        def integrate(t): t.traveled_cm = 7
        module.drive_checks.integrate_motion.side_effect = integrate
        module.creep_mm = Mock()
        module.finish_trip(trip)
        self.assertEqual(module.creep_mm.call_args.args[0], -20)

    def test_esp_reboot_during_wait_does_not_retreat_by_odometry(self):
        module, state, link = self.controller()
        state.live = {'cp': (0, 0, 0), 'phase': 'exploring'}
        trip = SimpleNamespace(watcher=None, ramp_state=None, reason='link_lost',
                               cell=(0, 0), hdg=0)
        module.recovery.wait_for_robot.side_effect = lambda *a: state.lifted.set() or True
        module.creep_mm = Mock()
        module.finish_trip(trip)
        module.creep_mm.assert_not_called()

    def test_lidar_outage_waits_then_returns_to_counted_center(self):
        module, state, link = self.controller()
        state.live = {'cp': (0, 0, 0), 'phase': 'exploring'}
        trip = SimpleNamespace(watcher=None, ramp_state=None, reason='lidar_lost',
                               cell=(0, 0), hdg=0, cells=1, boundary_cm=60.0,
                               traveled_cm=40.0, cell_cm=30.0, target_yaw=0,
                               ramp_rejected=False, pitch_start=0)
        module.recovery.wait_for_robot.return_value = False     # лидар уже вернулся
        module.creep_mm = Mock()
        module.turning = Mock()                                  # доводка после возврата
        module.finish_trip(trip)
        self.assertEqual(module.creep_mm.call_args_list[0].args[0], -100)
        self.assertEqual((trip.cells, trip.reason), (1, 'resumed'))
        self.assertFalse(state.lifted.is_set())


class RecoveryTests(unittest.TestCase):
    def test_tile_timeout_stops_without_checkpoint(self):
        state = SimpleNamespace(lifted=threading.Event())
        link = Mock()
        link.get_esp_paused.return_value = False
        link.telem_fresh.return_value = link.gyro_fresh.return_value = link.encoders_online.return_value = True
        module = load('drive_checks', {'state': state, 'esp_link': link})
        with patch.object(module.time, 'monotonic', return_value=10):
            trip = SimpleNamespace(t_cell=0, assist_extra_s=0)
            self.assertEqual(module.check_abort(trip), 'stop')
        self.assertEqual(trip.reason, 'timeout')       # разбирает вызывающий, как застревание
        self.assertFalse(state.lifted.is_set())

    def test_marker_return_tracks_actual_creep_not_requested_creep(self):
        deps = {n: Mock() for n in ('state', 'driving', 'esp_link', 'lidar_walls', 'maze_map', 'turning', 'victims')}
        deps.update(timing=TIMING, walls=SimpleNamespace(SIDE_THRESH=200),
            victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=10, RING_SCORE={}, SUM_TO_TYPE={}))
        module = load('marks', deps)
        module._interrupt_reason = Mock(return_value=None)
        module._read_wall = Mock(return_value={'label': 'test'})
        module.driving.creep_soft.return_value = 4.0
        # мишень видели, кольца не прочитались - сдвиг для другого блика
        det = SimpleNamespace(last_unread={'rings': ['?'] * 5}, best_fast=lambda: None)
        result, unread, moved = module._reread_aligned(det, (0, 0), 0, 0, 0, 'test', 1)
        self.assertEqual(moved, 4)
        self.assertNotEqual(module.driving.creep_soft.call_args.args[0], 4)

    def test_bumper_brakes_before_side_confirmation(self):
        link = Mock()
        module = load('bumper', {'state': Mock(), 'driving': Mock(), 'esp_link': link, 'turning': Mock()})
        module.get_bumper = Mock(return_value=True)
        def classify():
            link.brake.assert_called_once()
            return 'left'
        module.bumper_side = classify
        trip = SimpleNamespace(traveled_cm=1, boundary_cm=30)
        self.assertEqual(module.check_bumper_while_driving(trip), 'stop')
        self.assertEqual(trip.reason, 'bumper_left')


if __name__ == '__main__':
    unittest.main()
