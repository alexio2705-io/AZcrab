"""Горка определяется наклоном и временем, клетка за ней - уклоном: колёса на склоне врут."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from test_reliability import load

TICK_S = 0.01
TRANSITION_CM = 12.0


def trip_log(approach_cm, slope_cm, slope_deg=-30.0):
    """(одометрия, pitch) поездки: ровно, горка, снова ровно - шагом 1 см."""
    log = [(float(d), 0.0) for d in range(int(approach_cm))]
    log += [(approach_cm + d, slope_deg) for d in range(int(slope_cm))]
    end = approach_cm + slope_cm
    return log + [(end + d, 0.0) for d in range(6)]


def pitch_at(cm, start, length, slope):
    """Наклон корпуса на настоящем пути cm, с плавным заездом и съездом."""
    if cm < start or cm > start + length + TRANSITION_CM:
        return 0.0
    if cm < start + TRANSITION_CM:
        return slope * (cm - start) / TRANSITION_CM
    if cm <= start + length:
        return slope
    return slope * (1 - (cm - start - length) / TRANSITION_CM)


class RampCellsTest(unittest.TestCase):
    def setUp(self):
        self.ramps = load("ramps", {"state": SimpleNamespace(), "esp_link": Mock(), "maze_map": Mock()})

    def cells(self, wheels_cm, slope_deg):
        return self.ramps._ramp_measure(trip_log(9.0, wheels_cm, slope_deg), -1)["cells"]

    def test_wheel_length_does_not_change_ramp_cells(self):
        """Заезд 09.11: одна горка намерена 529 и 370 мм - вышло 3 и 2 клетки."""
        self.assertEqual(self.cells(53.0, -28.0), 2)
        self.assertEqual(self.cells(37.0, -28.0), 2)
        self.assertEqual(self.cells(22.0, -28.0), 2)

    def test_short_descent_with_long_transitions_keeps_true_slope(self):
        """Заезд 09.11 17:26: переходы занизили медиану до 25,6° - вышло 3 клетки вместо 2."""
        down = [-29.0 * i / 12 for i in range(1, 12)]
        log = [(float(d), 0.0) for d in range(10)]
        for pitch in down + [-29.0] * 12 + down[::-1]:
            log.append((log[-1][0] + 1.0, pitch))
        log += [(log[-1][0] + 1.0 + d, 0.0) for d in range(6)]
        self.assertEqual(self.ramps._ramp_measure(log, -1)["cells"], 2)

    def test_gentler_ramp_of_same_height_reaches_further(self):
        self.assertEqual(self.cells(47.0, -20.0), 3)


class RampAcceptTest(unittest.TestCase):
    def setUp(self):
        self.st = SimpleNamespace(last_ramp=None)
        ramps = load("ramps", {"state": self.st, "esp_link": Mock(), "maze_map": Mock()})
        self.slope = load("drive_ramp", {"state": self.st, "ramps": ramps})

    def drive(self, length_cm, slope_deg, wheels=1.0, speed_cms=60.0, expect_ramp=False):
        """Проезд по настоящему track_slope; wheels - какую долю пути на склоне видят колёса."""
        trip = SimpleNamespace(pitch_f=None, ramp_log=[], ramp_state=None, ramp_flat_cm=None, bump_on=False,
            bump_extra_cm=0.0, boundary_cm=30.0, stopping=False, pending=True, reason=None, cells=0,
            ramp_cells0=0, expect_ramp=expect_ramp, ramp_rejected=False, f_anchor=None, cell_cm=30.0,
            traveled_cm=0.0, t_cell=0.0, last_progress_t=0.0, loop_dt=TICK_S)
        approach, real_cm = 15.0, 0.0
        while real_cm < approach + length_cm + 45 and trip.ramp_state != "exit":
            on_slope = approach <= real_cm <= approach + length_cm + TRANSITION_CM
            real_cm += speed_cms * TICK_S
            trip.traveled_cm += speed_cms * TICK_S * (wheels if on_slope else 1.0)
            trip.pitch_delta = pitch_at(real_cm, approach, length_cm, slope_deg)
            self.slope.track_slope(trip)
        return trip

    def test_descent_is_a_ramp_when_wheels_count_half(self):
        """Заезд 09.11: спуск ~470 мм, колёса - 166..225 мм, «не горка»."""
        trip = self.drive(47.0, -28.0, wheels=0.4)
        self.assertEqual(trip.ramp_state, "exit")
        self.assertFalse(trip.ramp_rejected)

    def test_fast_slide_down_is_a_ramp(self):
        """Заезд 09.11 16:16: робот съехал быстро, круто было 0,33с - спуск отброшен."""
        trip = self.drive(30.0, -27.0, wheels=0.4, speed_cms=150.0)
        self.assertLess(trip.ramp_hold_s, 0.35)
        self.assertEqual(trip.ramp_state, "exit")

    def test_gentle_long_tilt_is_not_a_ramp(self):
        trip = self.drive(100.0, 10.0)
        self.assertTrue(trip.ramp_rejected)

    def test_short_steep_spike_is_not_a_ramp(self):
        trip = self.drive(2.0, 25.0, speed_cms=20.0)
        self.assertNotEqual(trip.ramp_state, "exit")


if __name__ == "__main__":
    unittest.main()
