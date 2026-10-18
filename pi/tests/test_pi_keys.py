"""Кнопки программ, запущенных из меню (pi/pi_keys.py): антидребезг."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pi_keys  # noqa: E402


class TestDebounce(unittest.TestCase):
    def test_held_from_start_is_not_a_press(self):
        # Запустили из меню кнопкой 25 и ещё держат - это не нажатие.
        d = pi_keys.Debounce()
        self.assertFalse(any(d.feed(True, t / 100) for t in range(20)))
        self.assertFalse(d.feed(False, 0.3))
        self.assertFalse(d.feed(True, 0.31))
        self.assertTrue(d.feed(True, 0.35))            # 40 мс низко после отпускания
        self.assertFalse(d.feed(True, 0.5))            # одно нажатие на удержание

    def test_short_glitch_ignored(self):
        d = pi_keys.Debounce()
        d.feed(False, 0.0)
        self.assertFalse(d.feed(True, 0.01))
        self.assertFalse(d.feed(False, 0.02))          # 10 мс - дребезг
        self.assertFalse(d.feed(True, 0.03))


if __name__ == "__main__":
    unittest.main()
