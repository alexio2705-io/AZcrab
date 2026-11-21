"""phone_web: флаги nav.py из формы, поиск процесса, разбор лога и state.json."""
import json
import os
import tempfile
import time
import unittest

import fakes  # noqa: F401  (кладёт pi/ в sys.path)
import phone_web as pw


class NavArgvTest(unittest.TestCase):
    def test_defaults_and_flags(self):
        self.assertEqual(pw.nav_argv({}), ("nav.py",))
        self.assertEqual(pw.nav_argv({"cells": " 6 ", "no_color": True, "no_vision": False,
                                      "red_checkpoint": True}),
                         ("nav.py", "6", "--no-color", "--red-checkpoint"))

    def test_bad_cells(self):
        for bad in ("0", "-3", "abc", "5000", "1; rm"):
            with self.assertRaises(ValueError):
                pw.nav_argv({"cells": bad})


class FindScriptTest(unittest.TestCase):
    def test_only_python_running_that_script(self):
        cmds = {10: "/usr/bin/vim nav.py",
                11: "/home/madness/maze/venv/bin/python -u /home/madness/maze/oled.py",
                12: "/home/madness/maze/venv/bin/python -u /home/madness/maze/nav.py 6 --no-color"}
        self.assertEqual(pw.find_script(cmds, "nav.py"), 12)
        self.assertIsNone(pw.find_script(cmds, "telem.py"))


class LogTest(unittest.TestCase):
    def test_tail_keeps_last_overwrite_of_cr_line(self):
        with tempfile.NamedTemporaryFile("w", delete=False) as f:
            f.write("a\nb\n  Старт через 2...\r  Старт!   \nc\n")
        try:
            self.assertEqual(pw.tail(f.name, 3), ["b", "  Старт!   ", "c"])
        finally:
            os.unlink(f.name)
        self.assertEqual(pw.tail("/nonexistent/log"), [])

    def test_waits_button(self):
        wait = "Нажмите GPIO27 (левая рука) или GPIO25 (правая рука) для старта..."
        self.assertTrue(pw.waits_button(["Инициализация ленты", wait]))
        self.assertFalse(pw.waits_button([wait, "  приоритет: ЛЕВАЯ рука (GPIO27)", "Старт!"]))
        self.assertFalse(pw.waits_button(["Движение: прямые участки"]))


class RunInfoTest(unittest.TestCase):
    def test_counts_real_victims_and_ignores_old_file(self):
        st = {"phase": "exploring", "ts": "12:00:00", "robot": {"cx": 1, "cy": 2, "dir": "N"},
              "visited": [[0, 0], [1, 0]],
              "victims": [{"type": "H"}, {"type": "fake"}, {"type": "green"}]}
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "state.json")
            with open(path, "w") as f:
                json.dump(st, f)
            info = pw.run_info(0, path)
            self.assertEqual((info["phase"], info["visited"], info["victims"]), ("exploring", 2, 2))
            self.assertIsNone(pw.run_info(time.time() + 60, path))
            self.assertIsNone(pw.run_info(0, os.path.join(d, "none.json")))


if __name__ == "__main__":
    unittest.main()
