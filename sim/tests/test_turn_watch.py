"""Наблюдатель меток на поворотах (nav.turn_to(seen=...), _turn_marks): стоя
камера видит ~125 мм из 280 мм стены, на повороте её проносит вдоль стены."""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fake_hw  # noqa: E402
import maze_sim as ms  # noqa: E402
import run_scenarios as rs  # noqa: E402


def run(world):
    rs.fresh_nav()
    sim = ms.Sim(world, color=False)
    sim.limit = 200
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        ms.setup(sim)
        fake_hw.install(sim)
        cx, cy, hdg, cp, _ = ms.nav.flood_fill_run(30)
        ms.nav.return_to_start(cx, cy, hdg, cp)
    return sim, buf.getvalue()


class TurnWatchTest(unittest.TestCase):
    def test_mark_seen_only_while_turning(self):
        # (0,3): приезд курсом на север, поворот на восток. Метка у северного
        # конца западной стены - стоя в кадре её нет (камера видит -87..+37 мм)
        w = rs.corridor_world()
        w.victims.append(ms.Victim((0, 3), 3, "damaged", 85.0))
        sim, log = run(w)
        blinks = [e for e in sim.events if e[1] == "blink" and e[4].get("truth") == 0]
        self.assertEqual(len(blinks), 1, "метка найдена один раз")
        self.assertIn("на повороте", log)

    def test_no_double_report(self):
        # метка посередине стены - её читает обычный скан стоя; поворот не
        # должен репортить её второй раз
        w = rs.corridor_world()
        w.victims.append(ms.Victim((0, 3), 3, "stable", -10.0))
        sim, _ = run(w)
        self.assertEqual(len([e for e in sim.events if e[1] == "blink"]), 1)


if __name__ == "__main__":
    unittest.main()
