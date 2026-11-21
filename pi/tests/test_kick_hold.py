"""Толчок моментом - только на нос, задранный от 5° дольше 0,15 с (не на качке шин)."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from test_reliability import load

TICK_S = 0.01


class KickHoldTest(unittest.TestCase):
    def setUp(self):
        st = SimpleNamespace(last_ramp=None)
        ramps = load("ramps", {"state": st, "esp_link": Mock(), "maze_map": Mock()})
        self.slope = load("drive_ramp", {"state": st, "ramps": ramps})

    def kicked(self, pitches):
        """Такты update_assist с наклоном из списка; был ли толчок."""
        trip = SimpleNamespace(braking=False, assist_mode=None, assist_kicks=0, assist_gap_until=0.0,
                               ramp_announced=False, v_meas=0.0, fwd=20.0, traveled_cm=0.0,
                               assist_extra_s=0.0, kick_tilt_t0=None, now_m=0.0)
        for i, pitch in enumerate(pitches):
            trip.now_m, trip.pitch_delta = i * TICK_S, pitch
            self.slope.update_assist(trip)
            if trip.assist_mode == "kick":
                return True
        return False

    def test_tyre_bounce_at_start_is_not_an_obstacle(self):
        """Заезд 10.11 10:52: +3,1° при v=0 на ровной плитке - рывок и занос."""
        self.assertFalse(self.kicked([3.1] * 30 + [0.0] * 5))
        self.assertFalse(self.kicked(([8.0] * 5 + [0.0] * 5) * 4))

    def test_nose_held_up_gets_a_kick(self):
        self.assertTrue(self.kicked([6.0] * 20))


if __name__ == "__main__":
    unittest.main()
