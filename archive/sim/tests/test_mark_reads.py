"""Чтение уже замеченной метки стоя (nav._read_seen_mark) и спор кадров о типе
(nav._confirm / _vote_type): осечка камеры не должна терять метку, а спор
типов решается большинством, без большинства - в пользу большего числа
комплектов (5.6.14: неверное число комплектов не штрафуется)."""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fake_hw  # noqa: E402
import maze_sim as ms  # noqa: E402
import run_scenarios as rs  # noqa: E402

LABEL = {"damaged": "Φ", "stable": "Ψ", "unharmed": "Ω"}


def mark(t, conf):
    return {"type": t, "label": LABEL[t], "conf": conf, "blink": True}


class Det:
    """read_wall отдаёт кадры по очереди (None - пустой кадр)."""

    def __init__(self, *frames):
        self.frames = list(frames)
        self.calls = 0

    def read_wall(self, cx, cy, wall_dir=None):
        self.calls += 1
        return self.frames.pop(0) if self.frames else None


class MarkReadsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rs.fresh_nav()
        sim = ms.Sim(rs.corridor_world(), color=False)
        with contextlib.redirect_stdout(io.StringIO()):
            ms.setup(sim)
            fake_hw.install(sim)
        cls.nav = ms.nav

    def call(self, fn, *a):
        with contextlib.redirect_stdout(io.StringIO()):
            return fn(*a)

    def test_seen_mark_survives_empty_frames(self):
        det = Det(None, None, mark("stable", 0.9))
        r = self.call(self.nav._read_seen_mark, det, 0, 0, 1, "тест")
        self.assertEqual(r["type"], "stable")
        self.assertEqual(det.calls, 3)

    def test_seen_mark_gives_up_after_tries(self):
        det = Det()
        self.assertIsNone(self.call(self.nav._read_seen_mark, det, 0, 0, 1, "тест"))
        self.assertEqual(det.calls, self.nav.MARK_READ_TRIES)

    def test_confirm_empty_second_frame_third_decides(self):
        det = Det(None, mark("damaged", 0.8))
        r = self.call(self.nav._confirm, det, 0, 0, 1, mark("damaged", 0.5))
        self.assertEqual(r["type"], "damaged")

    def test_confirm_two_empty_frames_no_blink(self):
        det = Det(None, None)
        self.assertIsNone(self.call(self.nav._confirm, det, 0, 0, 1, mark("damaged", 0.5)))

    def test_type_dispute_majority(self):
        det = Det(mark("unharmed", 0.8), mark("damaged", 0.6))
        r = self.call(self.nav._confirm, det, 0, 0, 1, mark("damaged", 0.5))
        self.assertEqual(r["type"], "damaged")
        self.assertEqual(r["conf"], 0.6, "из большинства - самый уверенный кадр")

    def test_type_dispute_no_majority_more_kits(self):
        # Φ и Ψ, третий кадр пуст: большинства нет - Φ (2 комплекта)
        det = Det(mark("stable", 0.9), None)
        r = self.call(self.nav._confirm, det, 0, 0, 1, mark("damaged", 0.5))
        self.assertEqual(r["type"], "damaged")

    def test_confident_read_no_extra_frames(self):
        det = Det(mark("unharmed", 0.9))
        r = self.call(self.nav._confirm, det, 0, 0, 1, mark("damaged", 0.95))
        self.assertEqual(r["type"], "damaged")
        self.assertEqual(det.calls, 0)


if __name__ == "__main__":
    unittest.main()
