"""Прореживание позы, мир, правда по рёбрам клеток, снимки карты (sim/sim_trace.py)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import run_scenarios as rs  # noqa: E402
from sim_trace import (edge_truth, map_snapshot, pose_columns, sample_terrain,  # noqa: E402
                       thin_poses, truth_rows, world_json)


def pose(t, m="M", b=False, **kw):
    s = {"t": t, "x": 0.0, "y": 0.0, "z": 0.0, "th": 0.0, "p": 0.0, "vl": 0.0, "vr": 0.0,
         "m": m, "sl": 0.0, "sr": 0.0, "cap": 0.0, "b": b, "f": None}
    s.update(kw)
    return s


def ticks(n, **kw):
    return [pose(round(i * 0.005, 3), **kw) for i in range(1, n + 1)]


class ThinPosesTest(unittest.TestCase):
    def test_50hz_plus_last(self):
        out = thin_poses(ticks(200))
        self.assertEqual(len(out), 51)
        self.assertEqual(out[0]["t"], 0.005)
        self.assertEqual(out[1]["t"], 0.025)
        self.assertEqual(out[-1]["t"], 1.0)

    def test_mode_change_kept(self):
        s = ticks(200)
        for p in s[101:]:
            p["m"] = "T"
        self.assertIn(s[101]["t"], [p["t"] for p in thin_poses(s)])

    def test_bumper_change_kept(self):
        s = ticks(200)
        s[50]["b"] = True
        ts = [p["t"] for p in thin_poses(s)]
        self.assertIn(s[50]["t"], ts)
        self.assertIn(s[51]["t"], ts)

    def test_keep_times(self):
        s = ticks(200)
        ts = [p["t"] for p in thin_poses(s, keep_times=[0.0123])]
        self.assertIn(0.015, ts)        # первый кадр не раньше события

    def test_empty(self):
        self.assertEqual(thin_poses([]), [])


class PoseColumnsTest(unittest.TestCase):
    def test_columns_rounded(self):
        cols = pose_columns([pose(1000.00449, x=1.234, th=12.3456, b=True, f=147.04),
                             pose(1000.02, m="T")], t0=1000.0)
        self.assertEqual(cols["t"], [0.004, 0.02])
        self.assertEqual(cols["x"], [1.2, 0.0])
        self.assertEqual(cols["th"], [12.35, 0.0])
        self.assertEqual(cols["m"], "MT")
        self.assertEqual(cols["b"], [1, 0])
        self.assertEqual(cols["f"], [147.0, None])


class TerrainTest(unittest.TestCase):
    def test_flat_island(self):
        self.assertEqual(sample_terrain(750, 750, 1050, 1050, lambda x, y: 140.0),
                         {"kind": "flat", "rect": [750, 750, 1050, 1050], "h": 140.0})

    def test_ramp_north(self):
        r = sample_terrain(-150, 450, 150, 750, lambda x, y: 140.0 * (y - 450.0) / 300.0)
        self.assertEqual(r, {"kind": "slope", "rect": [-150, 450, 150, 750], "dir": "N",
                             "h0": 0.0, "h1": 140.0})

    def test_ramp_down_to_east_rises_west(self):
        r = sample_terrain(0, 0, 300, 300, lambda x, y: 100.0 * (300.0 - x) / 300.0)
        self.assertEqual(r["dir"], "W")

    def test_bump(self):
        self.assertEqual(sample_terrain(-150, 345, 150, 375, lambda x, y: 15.0)["h"], 15.0)

    def test_world_json(self):
        w = rs.ramp_world()
        j = world_json(w.walls, w.pieces)
        self.assertEqual(len(j["walls"]), len(w.walls))
        self.assertEqual(len(j["walls"][0]), 6)
        self.assertEqual(sorted(t["kind"] for t in j["terrain"]), ["flat"] * 3 + ["slope"])
        self.assertEqual(j["cell"], 300)


class EdgeTruthTest(unittest.TestCase):
    def test_ground_floor_matches_old_wall_truth(self):
        for name, (make, _, _) in rs.SCENARIOS.items():
            w = make()
            for cx in range(-1, 4):
                for cy in range(-1, 6):
                    for d in range(4):
                        self.assertEqual(edge_truth(w.walls, w.height, cx, cy, d),
                                         rs.wall_truth(w, cx, cy, d), (name, cx, cy, d))

    def test_upper_floor_of_ramp_world(self):
        w = rs.ramp_world()
        t = lambda cx, cy: [edge_truth(w.walls, w.height, cx, cy, d) for d in range(4)]  # noqa: E731
        self.assertEqual(t(0, 1003), [False, True, False, True])
        self.assertEqual(t(1, 1004), [True, True, True, False])

    def test_island_cliff_is_wall_ramp_foot_is_not(self):
        w = rs.ramp_world()
        self.assertTrue(edge_truth(w.walls, w.height, -1, 3, 1))    # уступ острова
        self.assertFalse(edge_truth(w.walls, w.height, 0, 1, 0))    # подножие горки

    def test_truth_rows(self):
        w = rs.ramp_world()
        self.assertEqual(truth_rows(w.walls, w.height, [(1, 1004), (0, 1003)]),
                         [[0, 1003, 0, 1, 0, 1], [1, 1004, 1, 1, 1, 0]])


class MapSnapshotTest(unittest.TestCase):
    def test_snapshot(self):
        maze = {(1, 0): [True, None, False, True], (0, 0): [False, False, True, True]}
        fixed = {frozenset(((0, 0, 1), (1, 0, 3))): True,
                 frozenset(((1, 0, 0), (1, 1, 2))): False}
        snap = map_snapshot(maze, {(1, 0), (0, 0)}, {(0, 1, 0): (0, 1003, 1, 1)},
                            {(5, 5)}, fixed, "exploring")
        self.assertEqual(snap, {
            "phase": "exploring",
            "maze": [[0, 0, 0, 0, 1, 1], [1, 0, 1, None, 0, 1]],
            "visited": [[0, 0], [1, 0]],
            "ramps": [[0, 1, 0, 0, 1003, 1, 1]],
            "blocked": [[5, 5]],
            "fixed": [[0, 0, 1], [1, 0, 3]],
        })


if __name__ == "__main__":
    unittest.main()
