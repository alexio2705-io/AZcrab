"""Очки по регламенту RCJ Rescue Maze 2026 (sim/scoring.py)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import maze_sim as ms  # noqa: F401,E402 - pi/ в sys.path
import mazegen  # noqa: E402
import scoring  # noqa: E402


def spec_corridor(n=4, tiles=(), victims=(), ramps=(), bumps=()):
    """Коридор (0,0)..(n-1,0) на восток, всё обнесено стенами."""
    cells = [(x, 0) for x in range(n)]
    walls = set()
    for c in cells:
        for d in range(4):
            walls.add(mazegen._edge(c, d))
    for x in range(n - 1):
        walls.discard(mazegen._edge((x, 0), 1))
    return {"w": n, "h": 1, "walls": sorted(map(list, walls)), "tiles": [list(t) for t in tiles],
            "victims": list(victims), "bumps": list(bumps), "ramps": list(ramps), "high": []}


def visit(cell, still=0.0, full=True, t=0.0):
    return {"t": t, "cell": tuple(cell), "tile": "?", "full": full, "still": still}


def blink(t, x, y, th=90.0, typ="damaged", dur=5.3, moved=0.0):
    return (t, "blink", x, y, {"th": th, "type": typ, "dur": dur, "moved": moved})


class LinearTest(unittest.TestCase):
    def test_corridor_all_linear(self):
        self.assertEqual(mazegen.linear_tiles(spec_corridor(4)), {(0, 0), (1, 0), (2, 0), (3, 0)})

    def test_open_3x3_centre_floating(self):
        # открытая площадка 3x3, старт в углу: обход по стенам не заходит в центр
        cells = [(x, y) for x in range(3) for y in range(3)]
        walls = {mazegen._edge(c, d) for c in cells for d in range(4)}
        walls = {e for e in walls if not mazegen._inner_of(e, set(cells))}
        spec = {"w": 3, "h": 3, "walls": sorted(map(list, walls)), "tiles": []}
        lin = mazegen.linear_tiles(spec)
        self.assertNotIn((1, 1), lin)
        self.assertEqual(len(lin), 8)

    def test_black_is_virtual_wall(self):
        cells = [(x, y) for x in range(3) for y in range(3)]
        walls = {mazegen._edge(c, d) for c in cells for d in range(4)}
        walls = {e for e in walls if not mazegen._inner_of(e, set(cells))}
        spec = {"w": 3, "h": 3, "walls": sorted(map(list, walls)), "tiles": [[1, 0, "black"]]}
        # чёрная (1,0) - виртуальная стена: обход прижимается к ней и заходит в центр
        self.assertIn((1, 1), mazegen.linear_tiles(spec))


class ScoreTest(unittest.TestCase):
    def test_blue_revisits(self):
        spec = spec_corridor(4, tiles=[(2, 0, "blue")])
        v = [visit((0, 0)), visit((1, 0)), visit((2, 0), still=5.4), visit((3, 0))]
        s = scoring.score(spec, [], v, (3, 0))
        self.assertEqual((s["parts"]["blue"], s["sbv"], s["lop"]), (30, 1, 0))
        v += [visit((2, 0), still=5.4), visit((1, 0))]
        s = scoring.score(spec, [], v, (1, 0))
        self.assertEqual((s["parts"]["blue"], s["blue_repeat"]), (20, 1))
        v += [visit((2, 0), still=5.4), visit((3, 0)), visit((2, 0), still=5.4), visit((1, 0)),
              visit((2, 0), still=5.4), visit((3, 0))]
        s = scoring.score(spec, [], v, (3, 0))
        self.assertEqual(s["parts"]["blue"], 0, "не ниже нуля")
        self.assertEqual(s["sbv"], 0)

    def test_blue_without_stop_is_lop(self):
        spec = spec_corridor(4, tiles=[(2, 0, "blue")])
        v = [visit((1, 0)), visit((2, 0), still=4.0), visit((3, 0))]
        s = scoring.score(spec, [], v, (3, 0))
        self.assertEqual((s["lop_blue"], s["parts"]["blue"]), (1, 0))
        self.assertEqual(s["parts"]["reliability"], 0, "надёжность не отрицательная")

    def test_black_visit_is_lop(self):
        spec = spec_corridor(4, tiles=[(2, 0, "black")])
        s = scoring.score(spec, [], [visit((1, 0)), visit((2, 0)), visit((1, 0))], (1, 0))
        self.assertEqual(s["lop_black"], 1)

    def test_victim_points_and_kits(self):
        # Φ буквой на южной стене (1,0): линейная - 5; два комплекта - 30
        vic = {"cell": [1, 0], "d": 2, "type": "damaged", "along": 0, "kind": "letter"}
        spec = spec_corridor(4, victims=[vic])
        ev = [blink(10.0, 300, 0),
              (20.0, "kit", 300, 0, {"at": [300, -100]}),
              (22.0, "kit", 300, 0, {"at": [310, -100]}),
              (24.0, "kit", 300, 0, {"at": [310, -100]})]          # третий - лишний
        s = scoring.score(spec, ev, [visit((0, 0)), visit((1, 0))], (1, 0))
        self.assertEqual((s["parts"]["victims"], s["parts"]["kits"], s["srd"]), (5, 30, 2))
        self.assertEqual(s["parts"]["reliability"], 10 * 1 + 10 * 2)

    def test_kit_before_blink_does_not_count(self):
        vic = {"cell": [1, 0], "d": 2, "type": "stable", "along": 0, "kind": "letter"}
        spec = spec_corridor(4, victims=[vic])
        ev = [(5.0, "kit", 300, 0, {"at": [300, -100]}), blink(10.0, 300, 0, typ="stable")]
        self.assertEqual(scoring.score(spec, ev, [visit((1, 0))], (1, 0))["srd"], 0)

    def test_cognitive_floating_30(self):
        cells = [(x, y) for x in range(3) for y in range(3)]
        walls = {mazegen._edge(c, d) for c in cells for d in range(4)}
        walls = {e for e in walls if not mazegen._inner_of(e, set(cells))}
        walls.add(mazegen._edge((1, 1), 0))        # стенка у центра - на ней метка
        vic = {"cell": [1, 1], "d": 0, "type": "unharmed", "along": 0, "kind": "cog"}
        spec = {"w": 3, "h": 3, "walls": sorted(map(list, walls)), "tiles": [], "victims": [vic],
                "bumps": []}
        self.assertNotIn((1, 1), mazegen.linear_tiles(spec))
        s = scoring.score(spec, [blink(3.0, 300, 300, th=90.0, typ="unharmed")], [visit((1, 1))], (1, 1))
        self.assertEqual(s["parts"]["victims"], 30)

    def test_far_blink_is_misidentification(self):
        vic = {"cell": [3, 0], "d": 2, "type": "damaged", "along": 0}
        spec = spec_corridor(4, victims=[vic])
        s = scoring.score(spec, [blink(3.0, 0, 0)], [visit((0, 0))], (0, 0))
        self.assertEqual((s["misid"], s["svi"], s["score"]), (1, 0, 0), "очки не ниже нуля")

    def test_wrong_type_still_identified(self):
        vic = {"cell": [1, 0], "d": 2, "type": "damaged", "along": 0}
        spec = spec_corridor(4, victims=[vic])
        s = scoring.score(spec, [blink(3.0, 300, 0, typ="stable")], [visit((1, 0))], (1, 0))
        self.assertEqual((s["svi"], s["wrong"]), (1, 1))

    def test_two_victims_in_reach_credit_the_one_read(self):
        # робот в (1,0) курсом на восток: метки на южной и северной стенах обе в 15 см
        vics = [{"cell": [1, 0], "d": 2, "type": "damaged", "along": 0},
                {"cell": [1, 0], "d": 0, "type": "unharmed", "along": 0}]
        spec = spec_corridor(4, victims=vics)
        b1 = blink(3.0, 300, 0, typ="unharmed")
        b1[4]["truth"] = 1
        b2 = blink(9.0, 300, 0, typ="damaged")
        b2[4]["truth"] = 0
        s = scoring.score(spec, [b1, b2], [visit((1, 0))], (1, 0))
        self.assertEqual((s["svi"], s["wrong"], s["dup"]), (2, 0, 0))
        b3 = blink(12.0, 300, 0, typ="damaged")               # без truth - ещё не опознанная
        s = scoring.score(spec, [b1, b3], [visit((1, 0))], (1, 0))
        self.assertEqual((s["svi"], s["dup"]), (2, 0))

    def test_blink_while_moving_does_not_count(self):
        vic = {"cell": [1, 0], "d": 2, "type": "damaged", "along": 0}
        spec = spec_corridor(4, victims=[vic])
        s = scoring.score(spec, [blink(3.0, 300, 0, moved=60.0)], [visit((1, 0))], (1, 0))
        self.assertEqual((s["svi"], s["bad_blink"]), (0, 1))

    def test_exit_bonus(self):
        vic = {"cell": [1, 0], "d": 2, "type": "unharmed", "along": 0}
        spec = spec_corridor(4, tiles=[(2, 0, "blue")], victims=[vic])
        v = [visit((0, 0)), visit((1, 0)), visit((2, 0), still=6.0), visit((1, 0)), visit((0, 0))]
        ev = [blink(3.0, 300, 0, typ="unharmed"), (60.0, "finish_blink", 0, 0, {"s": 10.5, "moved": 0.0})]
        s = scoring.score(spec, ev, v, (0, 0))
        self.assertTrue(s["exit"])
        self.assertEqual(s["parts"]["exit"], 10 * 1 + 10 * 1)
        s2 = scoring.score(spec, ev[:1], v, (0, 0))
        self.assertEqual(s2["parts"]["exit"], 0, "без мигания на старте бонуса нет")

    def test_ramp_navigation(self):
        r = {"kind": "ramp", "cells": [[1, 0]], "d": 1, "bottom": [0, 0], "top": [2, 0]}
        spec = spec_corridor(3, ramps=[r])
        up = [visit((0, 0)), visit((1, 0)), visit((2, 0), full=True)]
        self.assertEqual(scoring.score(spec, [], up, (2, 0))["srn"], 1)
        half = [visit((0, 0)), visit((1, 0)), visit((2, 0), full=False)]
        self.assertEqual(scoring.score(spec, [], half, (2, 0))["srn"], 0)
        back = [visit((0, 0)), visit((1, 0)), visit((0, 0))]
        self.assertEqual(scoring.score(spec, [], back, (0, 0))["srn"], 0)
        twice = up + [visit((1, 0)), visit((0, 0), full=True)]
        self.assertEqual(scoring.score(spec, [], twice, (0, 0))["parts"]["ramps"], 10, "одна горка - 10")

    def test_bumps_once_per_tile(self):
        spec = spec_corridor(3, bumps=[{"cell": [1, 0], "axis": "ew", "off": 0, "h": 10}])
        ev = [(1.0, "bump_cross", 300, 0, {"cell": [1, 0]}), (9.0, "bump_cross", 300, 0, {"cell": [1, 0]})]
        self.assertEqual(scoring.score(spec, ev, [visit((1, 0))], (1, 0))["parts"]["bumps"], 5)

    def test_dist_to_robot(self):
        # робот курсом на восток в (0,0): правый борт на y=-62.5
        self.assertAlmostEqual(scoring.dist_to_robot(0, -140, 0, 0, 90.0), 77.5)
        self.assertAlmostEqual(scoring.dist_to_robot(96.5, 0, 0, 0, 90.0), 0.0)


if __name__ == "__main__":
    unittest.main()
