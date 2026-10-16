"""Нижняя граница визитов на синюю (sim/blue_audit.min_visits): сколько раз на
неё заехать неизбежно, если обойти всё поле и вернуться на старт."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import blue_audit as ba  # noqa: E402


def spec(w, h, blue, walls=()):
    return {"w": w, "h": h, "walls": [list(e) for e in walls],
            "tiles": [[x, y, "blue"] for x, y in blue], "victims": [], "bumps": [],
            "ramps": [], "obstacles": []}


class MinVisitsTest(unittest.TestCase):
    def test_corridor_through_blue_needs_a_way_back(self):
        # (0,0) - [1,0] - (2,0) - (3,0): за синей два клетки, туда и обратно
        self.assertEqual(ba.min_visits(spec(4, 1, [(1, 0)])), {(1, 0): 2})

    def test_dead_end_blue_needs_one(self):
        self.assertEqual(ba.min_visits(spec(4, 1, [(3, 0)])), {(3, 0): 1})

    def test_blue_on_a_loop_needs_one(self):
        # открытый квадрат 2x2: в (1,1) можно попасть мимо синей
        self.assertEqual(ba.min_visits(spec(2, 2, [(1, 0)])), {(1, 0): 1})

    def test_fork_on_blue_needs_one_more_per_branch(self):
        # Т-образная развилка на синей (1,0): ветки (2,0) и (1,1) - только через неё
        walls = [(0, 0, "h"), (2, 0, "h"), (0, 1, "v"), (1, 1, "v")]
        self.assertEqual(ba.min_visits(spec(3, 2, [(1, 0)], walls)), {(1, 0): 3})

    def test_lost_points(self):
        r = {"blues": [{"visits": 3, "need": 2}, {"visits": 2, "need": 2}]}
        self.assertEqual(ba.lost_points(r), 10)
        # 4-й визит гасит синюю целиком: ещё 10 очков и 20 бонуса/надёжности
        r = {"blues": [{"visits": 4, "need": 2}]}
        self.assertEqual(ba.lost_points(r), 40)


if __name__ == "__main__":
    unittest.main()
