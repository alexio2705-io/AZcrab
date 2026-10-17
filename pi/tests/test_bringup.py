"""Проверка Pi после включения (pi/bringup.py): подсчёт пакетов ESP в сыром
потоке - сколько нужной версии, сколько чужой."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import bringup  # noqa: E402

MAGIC = b"\xAA\x55"


def pkt(ver, size=10):
    return MAGIC + bytes([ver]) + bytes(size - 3)


class CountPacketsTest(unittest.TestCase):
    def test_counts_good_and_foreign_version(self):
        data = b"\x01\x02" + pkt(3) * 4 + pkt(2) + pkt(3)
        self.assertEqual(bringup.count_packets(data, MAGIC, 3, 10), (5, 1))

    def test_partial_tail_is_not_counted(self):
        data = pkt(3) + pkt(3)[:6]
        self.assertEqual(bringup.count_packets(data, MAGIC, 3, 10), (1, 0))

    def test_noise_without_magic(self):
        self.assertEqual(bringup.count_packets(b"\x00" * 100, MAGIC, 3, 10), (0, 0))


class FrameCheckTest(unittest.TestCase):
    def test_frontend_timeout_is_a_failure(self):
        bad = ("ERROR RPI pipeline_base.cpp:1346 Camera frontend has timed out!\n"
               "ERROR: Device timeout detected, attempting a restart!!!")
        self.assertFalse(bringup.frame_ok(bad))
        good = "INFO RPI pisp.cpp:1483 Sensor: ... Selected sensor format: 1296x972-SGBRG10_1X10/RAW"
        self.assertTrue(bringup.frame_ok(good))


if __name__ == "__main__":
    unittest.main()
