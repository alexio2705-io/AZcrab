"""remote_start: «Старт» с телефона; recovery.wait_start_button его забирает."""
import os
import tempfile
import threading
import types
import unittest
from unittest import mock

import fakes
import remote_start as rs


class RequestTest(unittest.TestCase):
    def test_request_take_once(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "start_request")
            self.assertIsNone(rs.take(path))
            rs.request("left", path)
            self.assertEqual(rs.take(path), "left")
            self.assertIsNone(rs.take(path))     # одно нажатие - один старт

    def test_bad_side(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "start_request")
            with self.assertRaises(ValueError):
                rs.request("up", path)
            with open(path, "w") as f:
                f.write("garbage")
            self.assertIsNone(rs.take(path))
            self.assertFalse(os.path.exists(path))


class FakeGPIO:
    BCM, IN, PUD_UP, LOW, HIGH = "bcm", "in", "pu", 0, 1

    def setmode(self, *_): pass
    def setup(self, *_, **__): pass
    def input(self, _pin): return self.HIGH       # кнопки не нажаты


def import_recovery():
    gpio = FakeGPIO()
    rpi = fakes.fake_module("RPi", GPIO=gpio)
    empty = {n: fakes.fake_module(n) for n in
             ("config", "dash_live", "esp_link", "lidar_walls", "maze_map", "ramps", "turning")}
    state = types.SimpleNamespace(side_pref=None)
    return fakes.import_with("recovery", **{"RPi": rpi, "RPi.GPIO": gpio, "state": state}, **empty)


class WaitStartButtonTest(unittest.TestCase):
    def test_phone_start_sets_side_and_stale_request_is_dropped(self):
        recovery = import_recovery()
        with tempfile.TemporaryDirectory() as d, \
                mock.patch.object(rs, "REQUEST_PATH", os.path.join(d, "start_request")):
            rs.request("left")                   # остался с прошлого раза - не в счёт
            timer = threading.Timer(0.1, rs.request, args=("right",))
            timer.start()
            recovery.wait_start_button()
            timer.join()
            self.assertEqual(recovery.state.side_pref, "right")
            self.assertIsNone(rs.take())


if __name__ == "__main__":
    unittest.main()
