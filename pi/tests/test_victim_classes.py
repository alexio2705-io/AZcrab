"""Классы меток-жертв в одном месте (config): H/S/U - приманки в обычном
заезде и жертвы, если задание объявило их через config.add_victim_class."""
import copy
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config  # noqa: E402
import watch_logic as wl  # noqa: E402

BOX_H = ("H", 0.9, (100.0, 100.0, 160.0, 160.0))
BOX_S = ("S", 0.8, (0.0, 0.0, 50.0, 50.0))
BOX_PHI = ("phi", 0.7, (200.0, 0.0, 250.0, 50.0))


class VictimClassesTest(unittest.TestCase):
    def setUp(self):
        self.saved = (set(config.VICTIM_CLASSES), dict(config.CLASS_TO_VICTIM),
                      copy.deepcopy(config.KITS_BY_TYPE))

    def tearDown(self):
        classes, to_victim, kits = self.saved
        config.VICTIM_CLASSES.clear()
        config.VICTIM_CLASSES.update(classes)
        config.CLASS_TO_VICTIM.clear()
        config.CLASS_TO_VICTIM.update(to_victim)
        config.KITS_BY_TYPE.clear()
        config.KITS_BY_TYPE.update(kits)

    def test_letters_are_decoys_in_normal_run(self):
        self.assertEqual(wl.victim_boxes([BOX_H, BOX_S, BOX_PHI]), [BOX_PHI])

    def test_task_makes_letters_victims(self):
        for c in ("H", "S", "U"):
            config.add_victim_class(c, kits=0)
        self.assertEqual(wl.victim_boxes([BOX_H, BOX_S, BOX_PHI]), [BOX_H, BOX_S, BOX_PHI])
        self.assertEqual(config.CLASS_TO_VICTIM["H"], "H")
        self.assertIn("H", config.KITS_BY_TYPE)          # мигание, как у жертвы
        self.assertEqual(config.KITS_BY_TYPE["H"], 0)    # без комплектов

    def test_normal_run_kits_unchanged(self):
        self.assertEqual(config.KITS_BY_TYPE, {"damaged": 2, "stable": 1, "unharmed": 0})


if __name__ == "__main__":
    unittest.main()
