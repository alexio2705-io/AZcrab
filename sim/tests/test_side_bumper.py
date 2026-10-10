"""Удар одним боком бампера о препятствие у стены (3.4.4b): назад в центр
клетки и повтор того же ребра со сдвигом от задетого бока (nav.py,
BUMPER_SIDE_SHIFT_MM) - робот проходит, клетки дальше разведаны."""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fake_hw  # noqa: E402
import maze_sim as ms  # noqa: E402
import run_scenarios as rs  # noqa: E402


class SideBumperTest(unittest.TestCase):
    def test_shift_past_obstacle(self):
        rs.fresh_nav()
        w = ms.World()
        w.box([(0, 0), (0, 1), (0, 2), (0, 3)])
        # у восточной стены (0,2): выступает на 90 мм, правый край носа его задевает
        w.walls.append((50.0, 560.0, 140.0, 640.0, 0.0, 200.0))
        sim = ms.Sim(w, color=False)
        sim.limit = 200
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            ms.setup(sim)
            fake_hw.install(sim)
            cx, cy, hdg, cp, _ = ms.nav.flood_fill_run(30)
            cx, cy, hdg = ms.nav.return_to_start(cx, cy, hdg, cp)
        log = buf.getvalue()
        self.assertIn("БАМПЕР (right)", log)
        self.assertIn("повтор со сдвигом", log)
        self.assertIn((0, 3), ms.nav._visited, "за препятствием разведано")
        self.assertEqual(ms.cell_of(sim.x, sim.y), (0, 0), "вернулся на старт")
        self.assertEqual((cx, cy), (0, 0))


if __name__ == "__main__":
    unittest.main()
