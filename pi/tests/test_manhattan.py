"""Курс по стенам (pi/manhattan.py) на синтетических точках."""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import manhattan as mh  # noqa: E402


def scene(beta, noise=0.0, front=True, seed=0):
    """Коридор: стены справа/слева в 140 мм, спереди в 450 мм (система стен);
    робот повёрнут на beta по часовой - точки в его системе."""
    rng = random.Random(seed)
    pts = []
    for x in range(-100, 200, 3):
        pts += [(x, 140.0), (x, -140.0)]
    if front:
        for y in range(-130, 131, 3):
            pts.append((450.0, float(y)))
    r = math.radians(-beta)
    c, s = math.cos(r), math.sin(r)
    out = []
    for x, y in pts:
        u, v = x * c - y * s, x * s + y * c
        out.append((u + rng.gauss(0, noise), v + rng.gauss(0, noise)))
    return out


class WallAngleTest(unittest.TestCase):
    def test_exact(self):
        for beta in (0.0, 1.3, -4.0, 8.0, 30.0):
            self.assertAlmostEqual(mh.wall_angle(scene(beta)), beta, delta=0.05)

    def test_modulo_90(self):
        self.assertAlmostEqual(mh.wall_angle(scene(93.0)), 3.0, delta=0.05)

    def test_noise(self):
        errs = [abs(mh.wall_angle(scene(2.5, noise=6.0, seed=k)) - 2.5) for k in range(20)]
        self.assertLess(sum(errs) / len(errs), 0.6)

    def test_too_few_points(self):
        self.assertIsNone(mh.wall_angle(scene(2.0)[:5]))
        self.assertIsNone(mh.wall_angle([]))

    def test_corner_does_not_bias(self):
        # стены, сходящиеся в угол: точки угла не должны тянуть наклон
        pts = [(float(x), 140.0) for x in range(-100, 141, 3)] + \
              [(140.0, float(y)) for y in range(-140, 141, 3)]
        self.assertAlmostEqual(mh.wall_angle(pts), 0.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
