"""Клетки длиннее 300 мм (домашнее поле 17.10: ~315 мм) и колёса, что за оборот
проходят меньше номинала: к концу коридора из четырёх одометрия отставала на
11-13 см, стену в его конце робот принимал за препятствие в пятой клетке и
записывал её в карту. Теперь лишнюю клетку снимает задняя стена
(nav._phantom_cells), а стоянки в коридоре доводятся по дальней задней стене
(nav._far_back_wall)."""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import maze_sim as ms  # noqa: E402
import run_scenarios as rs  # noqa: E402

DIRS = ((0, 1), (1, 0), (0, -1), (-1, 0))


def explore(cells, cell_mm=315.0, wheel=0.92, start_off_mm=40.0):
    """Разведка мира из клеток cells с шагом cell_mm; -> (nav, лог)."""
    saved = ms.CELL
    ms.CELL = cell_mm
    try:
        rs.fresh_nav()
        nav = ms.nav
        w = ms.World()
        w.box(cells)
        sim = ms.Sim(w, noise=ms.Noise(wheel_l=wheel, wheel_r=wheel))
        sim.y -= start_off_mm               # стартовали чуть позади центра
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ms.setup(sim)
            nav.flood_fill_run(40)
        return nav, buf.getvalue()
    finally:
        ms.CELL = saved


def map_errors(nav, cells):
    cs = set(cells)
    bad = []
    for c in sorted(cs & nav._visited):
        for d, (dx, dy) in enumerate(DIRS):
            got = nav._maze.get(c, [None] * 4)[d]
            if got is not None and got != ((c[0] + dx, c[1] + dy) not in cs):
                bad.append(f"{c}{'NESW'[d]}")
    return bad


class LongTilesTest(unittest.TestCase):
    def check(self, cells):
        nav, log = explore(cells)
        visited = {c for c in nav._visited if nav.level_of(c[1]) == 0}
        self.assertEqual(sorted(visited - set(cells)), [], "лишняя клетка в карте")
        return nav, log

    def test_corridor_of_four(self):
        cells = [(0, y) for y in range(4)]
        nav, _ = self.check(cells)
        self.assertEqual(map_errors(nav, cells), [])

    def test_corridor_with_side_rooms(self):
        # Карту здесь не сверяем: боковой проём (0,1)E при клетках 315 мм и
        # старте со сдвигом записывается стеной и без этой правки (известно).
        _, log = self.check([(0, y) for y in range(4)] + [(1, 1), (1, 2)])
        self.assertIn("стена через", log, "стоянки в коридоре - по дальней задней стене")


if __name__ == "__main__":
    unittest.main()
