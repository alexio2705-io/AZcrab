"""Заезд кончается только на старте: моторы выключили посреди проезда -
nav.py ждёт, возвращается в центр клетки и доезжает до старта."""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import run_scenarios as rs  # noqa: E402
import maze_sim as ms  # noqa: E402
import fake_hw  # noqa: E402


class MotorsOffTest(unittest.TestCase):
    def test_waits_for_motors_and_returns_home(self):
        rs.fresh_nav()
        sim = ms.Sim(rs.corridor_world())
        sim.limit = 200
        # питание моторов выключено с 3-й по 11-ю секунду - робот на ходу
        sim.hooks.append(lambda s: setattr(s, "motors_on", not 3.0 <= s.t - s.t0 < 11.0))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ms.setup(sim)
            fake_hw.install(sim)
            cx, cy, hdg, cp, status = ms.nav.flood_fill_run(30)
            cx, cy, hdg = ms.nav.return_to_start(cx, cy, hdg, cp)
        log = buf.getvalue()
        self.assertIn("[WAIT] моторы не отвечают", log)
        self.assertNotIn("FATAL", log)
        self.assertEqual((cx, cy), (0, 0))
        self.assertEqual(ms.cell_of(sim.x, sim.y), (0, 0), "nav думает, что дома, - и робот дома")
        self.assertGreaterEqual(len(ms.nav._visited), 6, "весь коридор разведан")


if __name__ == "__main__":
    unittest.main()
