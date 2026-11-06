"""Путь домой по карте упирается в стену, которой при разведке не было
(заезд 17.10: после ложной горки робот десятки раз ехал в одно и то же ребро
"СТЕНА ВПЛОТНУЮ", чередуя "через мягко закрытые рёбра" / "через все мягкие
рёбра заново"). Мягкое ребро, что не пустило HOME_EDGE_MAX_TRIES раз,
больше не берётся; старт недостижим - робот разведывает то, что ещё не
видел, и находит обход."""
import contextlib
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import fake_hw  # noqa: E402
import maze_sim as ms  # noqa: E402
import run_scenarios as rs  # noqa: E402

N, E, S, W = 0, 1, 2, 3


class FindHomePathTest(unittest.TestCase):
    def setUp(self):
        rs.fresh_nav()
        self.nav = ms.nav

    def test_only_edge_is_given_up_after_max_tries(self):
        nav = self.nav
        # коридор (0,0)-(0,1), ребро между ними закрыто мягко (упёрлись)
        for c, walls in (((0, 0), [False, True, True, True]),
                         ((0, 1), [True, True, False, True])):
            nav._maze[c] = walls
            nav._visited.add(c)
        nav.close_edge(0, 1, S)
        key = nav._edge_key(0, 1, S)
        tries = 0
        with contextlib.redirect_stdout(io.StringIO()):
            while nav.find_home_path(0, 1, S) is not None:
                tries += 1
                self.assertLessEqual(tries, nav.HOME_EDGE_MAX_TRIES, "одно ребро - по кругу")
                nav._home_fails[key] = nav._home_fails.get(key, 0) + 1
        self.assertEqual(tries, nav.HOME_EDGE_MAX_TRIES)


def blocked_ring():
    """Кольцо 2x3: (0,0)-(0,1)-(0,2)-(1,2)-(1,1)-(1,0)-(0,0); между (0,1) и
    (1,1) стена. Плюс стена между (0,1) и (0,2) - её при разведке не было."""
    w = ms.World()
    w.box([(x, y) for x in (0, 1) for y in (0, 1, 2)])
    w.wall(0, 1, E)
    w.wall(0, 1, N)
    return w


def seed_map(nav):
    """Карта, как будто робот проехал (0,0)->(0,1)->(0,2), а восточную
    половину ещё не разведал: (0,2)->E открыто, (1,*) не посещены."""
    for (x, y), walls in (((0, 0), [False, None, True, True]),
                          ((0, 1), [False, True, False, True]),
                          ((0, 2), [True, False, False, True])):
        for d, v in enumerate(walls):
            if v is not None:
                nav._set_wall(x, y, d, v)
        nav._visited.add((x, y))
    nav.open_edge(0, 0, N)      # проехали - голоса лидара ребро не закроют
    nav.open_edge(0, 1, N)


class BlockedHomeEdgeTest(unittest.TestCase):
    def test_gives_up_edge_and_explores_detour(self):
        rs.fresh_nav()
        sim = ms.Sim(blocked_ring(), cell=(0, 2), color=False)
        sim.limit = 240
        nav = ms.nav
        buf = io.StringIO()
        res = None
        with contextlib.redirect_stdout(buf):
            ms.setup(sim)
            fake_hw.install(sim)
            seed_map(nav)
            try:
                res = nav.return_to_start(0, 2, N, (0, 0, N))
            except ms.SimTimeUp:
                pass
        log = buf.getvalue()
        fails = nav._home_fails.get(nav._edge_key(0, 2, S), 0)
        self.assertLessEqual(fails, nav.HOME_EDGE_MAX_TRIES,
                             f"в стену {fails} раз:\n{log[-3000:]}")
        self.assertIsNotNone(res, "не доехал за отведённое время:\n" + log[-3000:])
        self.assertEqual(res[:2], (0, 0))
        self.assertEqual(ms.cell_of(sim.x, sim.y), (0, 0), "nav думает, что дома, - и робот дома")
        self.assertIn((1, 1), nav._visited, "обход разведан")
        self.assertNotIn("FATAL", log)


if __name__ == "__main__":
    unittest.main()
