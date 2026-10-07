"""VictimWatcher с фейковой камерой и фейковым воркером (нужен numpy)."""
import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

try:
    import numpy as np
except ImportError:          # на Маке без окружения из плана - пропуск
    np = None

FAKE = os.path.join(HERE, 'fake_watch_proc.py')


class FakeCam:
    def __init__(self, ts_offset_s=0.0):
        self.ts_offset_s = ts_offset_s
        self.fast = 0
        self.restored = 0

    def fast_exposure(self, max_us):
        self.fast += 1
        return max_us, 2.0

    def restore_exposure(self):
        self.restored += 1

    def try_latest(self):
        assert np is not None
        ts = int((time.monotonic() + self.ts_offset_s) * 1e9)
        return np.zeros((320, 320, 3), np.uint8), {'SensorTimestamp': ts}


def wait_for(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


@unittest.skipIf(np is None, 'нужен numpy')
class TestVictimWatcher(unittest.TestCase):
    def make(self, cam=None):
        from victim_watch import VictimWatcher
        self.cam = cam or FakeCam()
        w = VictimWatcher({'right': self.cam}, 'model.pt', '/tmp', 0.25, script=FAKE)
        self.addCleanup(w.close)
        self.assertTrue(w.ready.wait(5.0))
        return w

    def test_boxes_arrive_with_frame_time(self):
        w = self.make()
        t_arm = time.monotonic()
        self.assertTrue(w.arm({'right'}))
        got = []
        self.assertTrue(wait_for(lambda: got.extend(w.poll()) or got))
        b = got[0]
        self.assertEqual((b.side, b.cls), ('right', 'phi'))
        self.assertGreaterEqual(b.t_frame, t_arm - 0.05)
        self.assertEqual(self.cam.fast, 1)

    def test_unknown_side_and_disarm_restore(self):
        w = self.make()
        self.assertFalse(w.arm({'left'}))          # левой камеры нет
        w.arm({'right'})
        self.assertTrue(wait_for(lambda: self.cam.fast == 1))
        w.arm(set())
        w.wait_idle(1.0)
        self.assertEqual(self.cam.restored, 1)

    def test_stale_results_dropped_after_rearm(self):
        w = self.make()
        w.arm({'right'})
        time.sleep(0.3)
        w.arm(set())
        w.wait_idle(1.0)
        time.sleep(0.1)                             # ответы старого включения пришли
        t_rearm = time.monotonic()
        w.arm({'right'})
        for b in w.poll():
            self.assertGreaterEqual(b.t_frame, t_rearm - 0.05)

    def test_foreign_clock_falls_back_to_now(self):
        w = self.make(FakeCam(ts_offset_s=-1000.0))
        w.arm({'right'})
        got = []
        self.assertTrue(wait_for(lambda: got.extend(w.poll()) or got))
        self.assertLess(abs(got[0].t_frame - time.monotonic()), 1.0)

    def test_silent_worker_marks_dead(self):
        os.environ['FAKE_SILENT'] = '1'
        self.addCleanup(os.environ.pop, 'FAKE_SILENT', None)
        w = self.make()
        w.arm({'right'})
        self.assertTrue(wait_for(lambda: (w.poll(), w.dead)[1], timeout=4.0))
        self.assertFalse(w.arm({'right'}))

    def test_frame_copy_does_not_block_control_loop(self):
        # arm()/poll() зовутся каждый такт drive_cells (5 мс): копия кадра в
        # общую память не должна держать их замок
        import threading

        class SlowSlot:
            def __init__(self):
                self.busy = threading.Event()

            def __setitem__(self, key, value):
                self.busy.set()
                time.sleep(0.3)

        w = self.make()
        slot = SlowSlot()
        w._buf['right'] = slot
        w.arm({'right'})
        self.assertTrue(slot.busy.wait(2.0))
        t = time.monotonic()
        w.arm({'right'})
        w.poll()
        self.assertLess(time.monotonic() - t, 0.1)

    def test_attach_shm_shares_memory_and_leaves_it_to_owner(self):
        # watch_proc.py открывает память nav.py; на Pi venv - Python 3.11,
        # где у SharedMemory ещё нет track=False
        from multiprocessing import shared_memory
        from victim_watch import attach_shm
        own = shared_memory.SharedMemory(create=True, size=16)
        self.addCleanup(own.unlink)
        self.addCleanup(own.close)
        other = attach_shm(own.name)
        other.buf[0] = 42
        self.assertEqual(own.buf[0], 42)
        other.close()
        again = shared_memory.SharedMemory(name=own.name)   # владелец её не потерял
        again.close()

    def test_stats_line(self):
        w = self.make()
        w.arm({'right'})
        time.sleep(0.5)
        w.poll()
        line = w.take_stats()
        self.assertIsNotNone(line)
        assert line is not None
        self.assertIn('right', line)
        self.assertIn('Гц', line)


if __name__ == '__main__':
    unittest.main()
