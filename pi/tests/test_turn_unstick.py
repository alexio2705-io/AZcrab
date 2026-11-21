"""Лестница выхода из застревания на повороте - без железа."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from fakes import fake_module, import_with


class Link:
    """esp_link: корпус стоит, пока не пришло free_after команд момента."""

    def __init__(self, free_after=None):
        self.free_after = free_after
        self.torques = []
        self.yaw = 0.0
        self.now = 0.0
        self.motors = Mock()
        self.settle_brake = Mock()
        self.get_esp_paused = Mock(return_value=False)

    def interrupted(self) -> bool: return False
    def brake(self): pass
    def get_yaw(self): return self.yaw
    def wait_telem(self, seq, timeout):
        self.now += 0.02
        if self.free_after is not None and len(self.torques) >= self.free_after:
            l, r = self.torques[-1]
            self.yaw = (self.yaw + (5.0 if l > r else -5.0)) % 360.0
        return seq + 1

    def torque(self, l, r, cap):
        self.torques.append((l, r))

    def get_yaw_rate(self):
        if self.free_after is not None and len(self.torques) >= self.free_after:
            l, r = self.torques[-1]
            return 50.0 if l > r else -50.0
        return 0.0


def load(link, driving):
    config = SimpleNamespace(OBSTACLE_TORQUE_A=1.25, TURN_SIGN=1)
    mod = import_with("turn_unstick", config=fake_module("config", **vars(config)),
                      esp_link=link, driving=driving,
                      state=fake_module("state", telem_seq=0))
    setattr(mod, "time", SimpleNamespace(monotonic=lambda: link.now))
    return mod


def load_turning(link, driving, unstick, watcher=None):
    lidar = Mock()
    lidar.get_wall_snapshot.return_value = (0, None, {}, .1)
    turning = import_with(
        "turning", esp_link=link, driving=driving, turn_unstick=unstick,
        state=SimpleNamespace(YAW_ORIGIN=0.0, watcher=watcher, telem_seq=0,
                              turn_ff=.245, lifted=Mock(is_set=Mock(return_value=False))),
        config=SimpleNamespace(TURN_TOL_DEG=1.5, TURN_SIGN=1),
        ramps=SimpleNamespace(is_tilted=lambda: False),
        timing=SimpleNamespace(_timed=lambda _: lambda fn: fn),
        walls=SimpleNamespace(BACK_THRESH=260, SIDE_THRESH=210, WALL_THRESH=260),
        victim_detect=SimpleNamespace(CONF_FAST=.5, MIN_BOX_PX=40),
        lidar_walls=lidar, watch_logic=Mock())
    setattr(turning, "time", unstick.time)
    return turning


class UnstickTests(unittest.TestCase):
    def test_successful_push_keeps_torque_until_braking_in_both_directions(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                link = Link(free_after=1)
                mod = load(link, Mock())
                u = mod.Unstick()
                self.assertFalse(u.hold_turn(sign * 30, sign * 100, 0, sign * 38, 10))
                self.assertTrue(u.next_stage(sign * 38))
                # Толчок уже кончился, но до цели ещё далеко: сохраняем T:.
                self.assertTrue(u.hold_turn(sign * 30, sign * 100, sign * 50, sign * 33, 10))
                self.assertEqual(link.torques[-1], (sign * 2.0, -sign * 2.0))
                # У полки ток плавно снижается; выше полки и у цели нужен M:.
                self.assertTrue(u.hold_turn(sign * 20, sign * 100, sign * 90, sign * 30, 10))
                self.assertLess(abs(link.torques[-1][0]), mod.UNSTICK_TORQUE_A)
                n = len(link.torques)
                self.assertFalse(u.hold_turn(sign * 20, sign * 100, sign * 110, sign * 30, 10))
                self.assertFalse(u.hold_turn(sign * 5, sign * 30, 0, sign * 10, 10))
                self.assertFalse(u.hold_turn(-sign * 5, -sign * 30, sign * 40, sign * 15, 10))
                self.assertEqual(len(link.torques), n)

    def test_rate_spike_without_heading_progress_does_not_release_torque(self):
        link = Link()
        link.get_yaw_rate = lambda: 50.0
        mod = load(link, Mock())
        self.assertFalse(mod.Unstick().next_stage(76.0))
        self.assertGreaterEqual(link.now, mod.SPIN_S)
        self.assertGreater(len(link.torques), 1)

    def test_real_progress_across_zero_frees_both_turn_directions(self):
        for sign, yaw in ((1, 358.0), (-1, 2.0)):
            with self.subTest(sign=sign):
                link = Link(free_after=2)
                link.yaw = yaw
                self.assertTrue(load(link, Mock()).Unstick().next_stage(sign * 30.0))
                self.assertEqual(len(link.torques), 2)

    def test_pause_prevents_torque(self):
        link = Link()
        link.interrupted = lambda: True
        self.assertFalse(load(link, Mock()).Unstick().next_stage(76.0))
        self.assertEqual(link.torques, [])

    def test_first_stage_torques_sides_apart_toward_target(self):
        link = Link(free_after=1)
        u = load(link, Mock()).Unstick()
        self.assertTrue(u.next_stage(+30.0))
        l, r = link.torques[0]
        self.assertGreater(l, 0)
        self.assertLess(r, 0)

    def test_pivot_pulls_one_side_at_a_time(self):
        link = Link()
        u = load(link, Mock()).Unstick()
        u.stage = 2
        self.assertFalse(u.next_stage(-30.0))
        sides = {(l != 0, r != 0) for l, r in link.torques}
        self.assertEqual(sides, {(True, False), (False, True)})
        self.assertTrue(all(l <= 0 and r >= 0 for l, r in link.torques))

    def test_four_stages_then_exhausted_and_back_off_returned(self):
        link = Link()
        driving = Mock()
        driving.creep_mm.return_value = -40.0
        mod = load(link, driving)
        u = mod.Unstick()
        for _ in range(4):
            self.assertFalse(u.next_stage(-30.0))
        self.assertTrue(u.exhausted())
        driving.creep_mm.assert_called_once_with(-mod.BACK_MM, 0.0)
        u.return_after_turn()          # курс не поменялся - назад вперёд целиком
        self.assertAlmostEqual(driving.creep_mm.call_args[0][0], 40.0)

    def test_no_return_after_quarter_turn(self):
        link = Link()
        driving = Mock()
        driving.creep_mm.return_value = -40.0
        u = load(link, driving).Unstick()
        u.stage = 3
        u.next_stage(30.0)
        link.yaw = 90.0                # смещение теперь поперёк - доберёт центровка
        u.return_after_turn()
        self.assertEqual(driving.creep_mm.call_count, 1)


class TurnProgressTests(unittest.TestCase):
    def setUp(self):
        self.mod = load(Link(), Mock())

    def test_last_log_stall_and_rocking_trigger_recovery(self):
        for sign in (1, -1):
            p = self.mod.TurnProgress(0.5, 8.0)
            # Конец run_2022: остаётся ~76°, дрожание не приближает к цели.
            for i, rem in enumerate((75.8, 74.5, 76.2, 75.0, 76.0, 75.9)):
                self.assertEqual(p.stalled(i * 0.1, sign * rem), i == 5)

    def test_steady_turn_and_reset_do_not_trigger_recovery(self):
        p = self.mod.TurnProgress(0.5, 8.0)
        for i in range(6):
            self.assertFalse(p.stalled(i * 0.1, 90.0 - i * 3.0))
        p.reset()  # после выхода из застревания отсчитываем новое окно
        self.assertFalse(p.stalled(10.0, 60.0))
        self.assertFalse(p.stalled(10.2, 60.0))
        self.assertTrue(p.stalled(10.5, 60.0))

    def test_wrong_direction_is_not_progress(self):
        p = self.mod.TurnProgress(0.5, 8.0)
        self.assertFalse(p.stalled(0.0, 40.0))
        self.assertTrue(p.stalled(0.5, 50.0))


class TurnRecoveryIntegrationTest(unittest.TestCase):
    def test_main_loop_keeps_assist_then_brakes_and_settles(self):
        link = Link()
        driving = Mock()
        mod = load(link, driving)
        turning = load_turning(link, driving, mod)
        commands = []
        # Заданные измерения после толчка: проверяем команды контура,
        # физику движения робота здесь не моделируем.
        headings = iter((5, 8, 12, 18, 28, 42, 60, 75, 83, 89, 90))
        rate = [0.0]

        def torque(l, r, cap):
            link.torques.append((l, r))
            commands.append((link.yaw, "T", cap))

        def wait_telem(seq, timeout):
            link.now += .02
            if link.torques:
                link.yaw = float(next(headings, 90))
                rate[0] = 70.0 if link.yaw < 89 else 0.0
            return seq + 1

        link.torque = torque
        link.motors.side_effect = lambda l, r: commands.append((link.yaw, "M", None))
        link.wait_telem = wait_telem
        link.get_yaw_rate = lambda: rate[0]
        self.assertTrue(turning.turn_to(90.0))
        held = [(mode, cap) for yaw, mode, cap in commands if 5 <= yaw <= 75]
        self.assertGreater(len(held), 3)
        self.assertTrue(all(mode == "T" and 5 <= cap <= 20 for mode, cap in held))
        self.assertTrue(all(mode == "M" for yaw, mode, cap in commands if yaw >= 83))
        self.assertEqual(turning.state.turn_ff, .245)
        link.settle_brake.assert_called_once()
        driving.creep_mm.assert_not_called()

    def test_interruption_after_successful_push_stops_commands(self):
        for paused in (False, True):
            with self.subTest(paused=paused):
                link = Link(free_after=1)
                link.interrupted = lambda: bool(link.torques)
                link.get_esp_paused = Mock(return_value=paused)
                link.brake = Mock()
                driving = Mock()
                mod = load(link, driving)
                turning = load_turning(link, driving, mod)
                self.assertFalse(turning.turn_to(90.0))
                self.assertEqual(len(link.torques), 1)
                self.assertEqual(link.brake.call_count, 0 if paused else 1)
                driving.creep_mm.assert_not_called()

    def test_stationary_heading_with_high_gyro_rate_gets_torque_before_timeout(self):
        self.check_stuck_turn(watch=False)

    def test_recovery_also_runs_with_corner_camera_watching(self):
        self.check_stuck_turn(watch=True)

    def check_stuck_turn(self, watch):
        link = Link()
        link.yaw = 324.0
        # В старом контуре rate > 8 постоянно сбрасывал таймер застревания.
        link.get_yaw_rate = lambda: 12.0
        driving = Mock()
        driving.creep_mm.return_value = -40.0
        unstick = load(link, driving)
        first_torque_at = []
        torque = link.torque

        def record_torque(l, r, cap):
            if not first_torque_at:
                first_torque_at.append(link.now)
            torque(l, r, cap)

        link.torque = record_torque
        watcher = Mock(dead=False, cams={"right": object()}) if watch else None
        if watcher is not None:
            watcher.live_info.return_value = {"right": {"fid": 0}}
            watcher.poll.return_value = []
        turning = load_turning(link, driving, unstick, watcher)
        self.assertFalse(turning.turn_to(248.2, seen=[] if watch else None, walls=[True] * 4))
        self.assertEqual(len(first_torque_at), 1)
        self.assertLess(first_torque_at[0], 1.0)
        self.assertTrue(all(l <= 0 and r >= 0 for l, r in link.torques[:20]))
        link.settle_brake.assert_called_once()
        # Ступень 4 откатилась, но поворот не выполнен: обратного подъезда нет.
        driving.creep_mm.assert_called_once()


if __name__ == "__main__":
    unittest.main()
