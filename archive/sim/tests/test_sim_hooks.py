"""Хуки регистратора трассы в модели (sim/maze_sim.py)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import maze_sim as ms  # noqa: E402
from run_scenarios import corridor_world, ramp_world  # noqa: E402


class FakeRec:
    def __init__(self):
        self.steps = []
        self.scans = []

    def on_step(self, sim, pitch):
        self.steps.append((sim.t, sim.cmd_raw, pitch))

    def on_scan(self, sim, hits, pts):
        self.scans.append((hits, pts))


class SimHooksTest(unittest.TestCase):
    def test_raw_command_mode_kept(self):
        sim = ms.Sim(corridor_world())
        sim.write(b"T:20,20,30*")
        self.assertEqual(sim.cmd_raw, "T")
        sim.write(b"B*")
        self.assertEqual(sim.cmd_raw, "B")
        self.assertEqual(sim.cmd, ("M", 0.0, 0.0, 0.0))
        sim.write(b"M:10,10*")
        self.assertEqual(sim.cmd_raw, "M")

    def test_no_recorder_by_default(self):
        sim = ms.Sim(corridor_world())
        self.assertIsNone(sim.rec)
        sim.advance(0.2)          # без регистратора модель работает как раньше

    def test_recorder_gets_steps_and_scans(self):
        sim = ms.Sim(ramp_world())       # лучи вперёд упираются в склон горки
        rec = FakeRec()
        sim.rec = rec
        sim.advance(0.2)
        self.assertEqual(len(rec.steps), 40)
        self.assertGreaterEqual(len(rec.scans), 1)
        hits, pts = rec.scans[0]
        self.assertTrue(all(len(h) == 4 for h in hits))
        kinds = {h[3] for h in hits}
        self.assertEqual(kinds, {True, False})   # и стены, и рельеф
        self.assertIsInstance(pts, list)


if __name__ == "__main__":
    unittest.main()
