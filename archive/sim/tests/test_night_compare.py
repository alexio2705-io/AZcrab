"""Сравнение вариантов nav.py в ночном прогоне (sim/night.py --compare)."""
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import night  # noqa: E402


def run(score, sim_s=100.0, home=True, problems=()):
    return {"score": score, "score_frac": score / 400, "sim_s": sim_s, "coverage": 1.0, "svi": 3,
            "srd": 2, "sbv": 1, "blue_repeat": 0, "lop": 0, "misid": 0, "home": home,
            "n_problems": len(problems), "problems": list(problems), "seed": 1, "noise": 0.0}


class CompareTest(unittest.TestCase):
    def test_boot_ci_deterministic_and_covers_mean(self):
        d = [1.0, 2.0, 3.0, 4.0, 5.0, 2.0, 3.0]
        lo, hi = night.boot_ci(d)
        self.assertEqual((lo, hi), night.boot_ci(d))
        self.assertLess(lo, night.mean(d))
        self.assertGreater(hi, night.mean(d))

    def test_verdicts(self):
        pairs = [(run(200 + i, sim_s=200), run(260 + i, sim_s=150)) for i in range(10)]
        rows = {r[0]: r for r in night.compare_table(pairs)}
        self.assertEqual(rows["score"][7], "лучше")
        self.assertEqual(rows["sim_s"][7], "лучше", "меньше времени - лучше")
        self.assertEqual(rows["coverage"][7], "—", "без разницы - без вердикта")
        worse_pairs = [(b, a) for a, b in pairs]
        self.assertEqual({r[0]: r for r in night.compare_table(worse_pairs)}["score"][7], "хуже")

    def test_worse_reasons(self):
        self.assertEqual(night.worse(run(300), run(300)), [])
        why = night.worse(run(300), run(250, home=False, problems=["не вернулся"]))
        self.assertTrue(any("очки" in w for w in why))
        self.assertTrue(any("перестал возвращаться" in w for w in why))

    def test_arm_working_copy_with_constants(self):
        tmp = tempfile.mkdtemp()
        try:
            a = night.Arm(".+BLUE_STOP_S=5.3,FOO=bar", "B", tmp)
            self.assertEqual(a.pi, os.path.join(night.ROOT, "pi"))
            self.assertEqual(a.sets, {"BLUE_STOP_S": 5.3, "FOO": "bar"})
            g = night.Arm("HEAD", "A", tmp)
            self.assertTrue(os.path.exists(os.path.join(g.pi, "nav.py")))
            self.assertIn("git HEAD", g.desc)
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main()
