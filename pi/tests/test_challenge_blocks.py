import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from challenge_blocks import Challenge, blink, wait, kits, go_to, home


class BlocksTests(unittest.TestCase):
    def setup_run(self, expired=False):
        actions = Mock()
        actions.Timer.return_value.time_to_go.return_value = expired
        rules = SimpleNamespace(RULES={"blue": object()}, STOP="stop", Rule=Mock())
        return actions, rules, SimpleNamespace(cell=(0, 0))

    def test_order_and_restore(self):
        a, r, w = self.setup_run()
        original = dict(r.RULES)
        task = Challenge(blink(2), wait(1), home()).on("red", kits(1))
        task.execute(a, r, w)
        self.assertEqual(r.RULES, original)
        self.assertEqual([c[0] for c in a.method_calls], [
            "Timer", "look_around", "blink", "stand", "go_to_cell", "stand_motors_off"])

    def test_deadline_returns_home_before_next_block(self):
        a, r, w = self.setup_run(True)
        Challenge(kits(2)).execute(a, r, w)
        a.give_kits.assert_not_called()
        a.go_to_cell.assert_called_once_with(w, (0, 0))

    def test_failed_navigation_restores_rules(self):
        a, r, w = self.setup_run()
        original = dict(r.RULES)
        a.go_to_color.return_value = (1, 0)
        with self.assertRaises(RuntimeError):
            Challenge(go_to("red")).on("red", blink(1)).execute(a, r, w)
        self.assertEqual(r.RULES, original)

    def test_validation(self):
        for value in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError): wait(value)
        with self.assertRaises(ValueError): kits(1.5)
        with self.assertRaises(ValueError): Challenge(home()).on("red", home())
        with self.assertRaises(ValueError): Challenge(limit_s=5, reserve_s=11)

    def test_preview_without_hardware(self):
        self.assertIn("blink: 2", Challenge(blink(2)).describe())


if __name__ == "__main__":
    unittest.main()
