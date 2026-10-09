"""Трасса настоящего прогона nav.py в симуляторе (run_scenarios.run(trace=True))."""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import run_scenarios as rs  # noqa: E402
from sim_trace import dump_trace  # noqa: E402


def traced(name):
    rs.fresh_nav()
    with contextlib.redirect_stdout(io.StringIO()):
        return rs.run(name, trace=True)


class TraceRunTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ok, cls.tr = traced("flat")

    def test_deterministic(self):
        ok2, tr2 = traced("flat")
        self.assertTrue(self.ok and ok2)
        self.assertEqual(dump_trace(self.tr), dump_trace(tr2))

    def test_contents(self):
        tr = self.tr
        self.assertEqual(sorted(tr), ["events", "maps", "meta", "poses", "scans", "truth", "world"])
        self.assertEqual(tr["meta"]["status"], "explored")
        ts = tr["poses"]["t"]
        self.assertGreater(len(ts), 100)
        self.assertLessEqual(max(b - a for a, b in zip(ts, ts[1:])), 0.0201)
        self.assertEqual(len(tr["poses"]["m"]), len(ts))
        self.assertGreaterEqual(len(tr["scans"]), 10)
        sc = tr["scans"][len(tr["scans"]) // 2]
        self.assertTrue(sc["hits"] and all(len(h) == 4 for h in sc["hits"]))
        self.assertIn("corr", sc)
        self.assertTrue(any(e["type"] == "decide" for e in tr["events"]))
        self.assertGreaterEqual(len(tr["maps"]), 1)
        self.assertTrue(tr["truth"])

    def test_times_start_near_zero(self):
        self.assertLess(self.tr["poses"]["t"][0], 1.0)
        self.assertGreaterEqual(self.tr["events"][0]["t"], 0.0)


if __name__ == "__main__":
    unittest.main()
