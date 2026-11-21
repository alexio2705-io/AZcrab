"""teleop: джойстик -> скорости колёс и стоп по протухшей команде."""
import unittest

import fakes  # noqa: F401  (кладёт pi/ в sys.path)
import teleop


class MixTest(unittest.TestCase):
    def test_straight_and_spin(self):
        self.assertEqual(teleop.mix(0, 1, 30), (30.0, 30.0))
        self.assertEqual(teleop.mix(0, -1, 30), (-30.0, -30.0))
        l, r = teleop.mix(1, 0, 30)
        self.assertAlmostEqual(l, 18.0)
        self.assertAlmostEqual(r, -18.0)

    def test_never_above_speed(self):
        for x, y in ((1, 1), (-1, 1), (5, -9)):
            l, r = teleop.mix(x, y, 40)
            self.assertLessEqual(max(abs(l), abs(r)), 40.0 + 1e-9)

    def test_speed_capped(self):
        self.assertEqual(teleop.mix(0, 1, 999), (teleop.MAX_CMS, teleop.MAX_CMS))


class TargetTest(unittest.TestCase):
    def test_stale_command_means_stop(self):
        t = teleop.Target()
        self.assertEqual(t.get(100.0), (0.0, 0.0))
        t.set(20.0, 20.0, 100.0)
        self.assertEqual(t.get(100.0 + teleop.STALE_S / 2), (20.0, 20.0))
        self.assertEqual(t.get(100.0 + teleop.STALE_S * 2), (0.0, 0.0))


class ReadLoopTest(unittest.TestCase):
    def test_marker_split_between_reads_is_counted(self):
        import threading
        chunks = [b"\xAA", b"\x55" + b"x" * 10 + b"\xAA", b"\x55" + b"x" * 10]
        stop = threading.Event()

        class Link:
            in_waiting = 0

            def read(self, n):
                if not chunks:
                    stop.set()
                    return b""
                return chunks.pop(0)

        esp = teleop.Esp(Link())
        esp.read_loop(stop)
        self.assertEqual(esp.packets, 2)


if __name__ == "__main__":
    unittest.main()
