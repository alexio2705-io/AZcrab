"""Генератор полей (sim/mazegen.py) по регламенту 2026 и учёт посещений в модели."""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import maze_sim as ms  # noqa: E402
import mazegen  # noqa: E402
from run_scenarios import corridor_world  # noqa: E402

SEEDS = range(1, 151)


class GeneratorRulesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.specs = [mazegen.generate(s) for s in SEEDS]

    def test_deterministic(self):
        self.assertEqual(json.dumps(mazegen.generate(42)), json.dumps(mazegen.generate(42)))

    def test_features_present_somewhere(self):
        kinds = {r["kind"] for s in self.specs for r in s["ramps"]}
        self.assertEqual(kinds, {"ramp", "stairs"})
        self.assertTrue(any(s["zone"] for s in self.specs))
        self.assertTrue(any(s["obstacles"] for s in self.specs))
        self.assertTrue(any(s["high"] for s in self.specs))

    def test_no_features_when_disabled(self):
        s = mazegen.generate(7, features=[])
        self.assertEqual((s["ramps"], s["zone"], s["obstacles"], s["high"]), ([], [], [], []))

    def test_everything_reachable(self):
        for s in self.specs:
            tiles = {(x, y): t for x, y, t in s["tiles"]}
            rc = mazegen.ramp_cells(s)
            want = {(x, y) for x in range(s["w"]) for y in range(s["h"])
                    if tiles.get((x, y)) != "black" and (x, y) not in rc} - mazegen.blocked_cells(s)
            self.assertEqual(mazegen.reachable(s), want, f"seed {s['seed']}")

    def test_victims_not_facing_special_tiles(self):
        # 3.6.3: не на стенах у чёрной/серебра/синей/красной, бугорков, препятствий, горок
        for s in self.specs:
            bad = {(x, y) for x, y, _ in s["tiles"]} | mazegen.ramp_cells(s)
            bad |= {tuple(b["cell"]) for b in s["bumps"]} | {tuple(d["cell"]) for d in s["debris"]}
            bad |= {tuple(o["cell"]) for o in s["obstacles"]}
            walls = {tuple(e) for e in s["walls"]}
            for v in s["victims"]:
                self.assertNotIn(tuple(v["cell"]), bad, f"seed {s['seed']}")
                self.assertIn(mazegen._edge(tuple(v["cell"]), v["d"]), walls)
                self.assertIn(v["kind"], ("letter", "cog"))

    def test_zone_has_red_entrance_and_single_door(self):
        for s in self.specs:
            if not s["zone"]:
                continue
            zone = {tuple(c) for c in s["zone"]}
            red = [(x, y) for x, y, t in s["tiles"] if t == "red"]
            self.assertEqual(len(red), 1)
            self.assertIn(red[0], zone)
            walls = {tuple(e) for e in s["walls"]}
            doors = [(c, d) for c in zone for d in range(4)
                     if mazegen._step(c, d) not in zone and mazegen._edge(c, d) not in walls
                     and 0 <= mazegen._step(c, d)[0] < s["w"] and 0 <= mazegen._step(c, d)[1] < s["h"]]
            self.assertEqual(len(doors), 1, f"seed {s['seed']}: зона обнесена стенами, вход один")
            self.assertEqual(doors[0][0], red[0])
            self.assertNotIn((0, 0), zone)

    def test_obstacles_by_rules(self):
        # 3.4.3-4: не ниже 15 см; у стены - вглубь не дальше 10 см от края плитки
        for s in self.specs:
            for o in s["obstacles"]:
                self.assertGreaterEqual(o["h"], 150)
                x0, y0, x1, y1 = o["rect"]
                cx, cy = o["cell"][0] * 300, o["cell"][1] * 300
                if o["kind"] == "wall":
                    depth = min(x1 - x0, y1 - y0)
                    self.assertLessEqual(depth, 100)
                    edge_gap = min(abs(x0 - (cx - 150)), abs(x1 - (cx + 150)),
                                   abs(y0 - (cy - 150)), abs(y1 - (cy + 150)))
                    self.assertLessEqual(edge_gap, 11, "касается стены")

    def test_ramp_walls_and_heights(self):
        s = next(s for s in self.specs if any(r["kind"] == "ramp" for r in s["ramps"]))
        w = mazegen.build_world(s)
        r = next(r for r in s["ramps"] if r["kind"] == "ramp")
        b, top = r["bottom"], r["top"]
        self.assertEqual(w.height(b[0] * 300, b[1] * 300), 0.0)
        self.assertEqual(w.height(top[0] * 300, top[1] * 300), mazegen.RISE)
        mid = r["cells"][len(r["cells"]) // 2]
        h = w.height(mid[0] * 300, mid[1] * 300)
        self.assertTrue(0 < h < mazegen.RISE)
        walls = {tuple(e) for e in s["walls"]}
        for c in r["cells"]:
            for d in ((r["d"] + 1) % 4, (r["d"] + 3) % 4):
                self.assertIn(mazegen._edge(tuple(c), d), walls, "у горки боковые стены")

    def test_stairs_steps_at_most_2cm(self):
        s = next(s for s in self.specs if any(r["kind"] == "stairs" for r in s["ramps"]))
        w = mazegen.build_world(s)
        r = next(r for r in s["ramps"] if r["kind"] == "stairs")
        dx, dy = mazegen.DIRS[r["d"]]
        b = r["bottom"]
        hs = [w.height(b[0] * 300 + dx * k, b[1] * 300 + dy * k) for k in range(0, 901, 5)]
        self.assertEqual(hs[0], 0.0)
        self.assertEqual(hs[-1], mazegen.RISE)
        self.assertLessEqual(max(abs(a - c) for a, c in zip(hs, hs[1:])), 20.0 + 1e-6)

    def test_world_json_nav_coords(self):
        s = next(s for s in self.specs if s["high"])
        wj = mazegen.world_json(s)
        hx, hy = s["high"][0]
        self.assertIn(f"{hx},{hy + 1000}", wj["maze"])


class VisitsTest(unittest.TestCase):
    """Посещения плиток в модели: больше половины корпуса (5.4.4)."""

    def drive(self, sim, cms, seconds):
        sim.cmd = ("M", cms, cms, 0.0)
        sim.advance(seconds)
        sim.cmd = ("M", 0.0, 0.0, 0.0)

    def test_visit_switches_past_half(self):
        sim = ms.Sim(corridor_world())
        sim.advance(0.1)
        self.assertEqual(sim.visits[-1]["cell"], (0, 0))
        self.assertTrue(sim.visits[-1]["full"])
        sim.y = 140.0                        # центр ещё на своей плитке
        sim.advance(0.05)
        self.assertEqual(sim.visits[-1]["cell"], (0, 0))
        sim.y = 160.0                        # больше половины - на следующей
        sim.advance(0.05)
        self.assertEqual(sim.visits[-1]["cell"], (0, 1))
        self.assertFalse(sim.visits[-1]["full"])

    def test_blue_stop_counts_only_standing(self):
        w = corridor_world()
        w.tiles[(0, 1)] = "blue"
        sim = ms.Sim(w)
        sim.y = 300.0
        sim.advance(5.5)                     # стоит 5.5 с
        sim.theta = 10.0
        sim.y = 600.0
        sim.advance(0.1)
        ev = [e for e in sim.events if e[1] == "blue_left"]
        self.assertEqual(len(ev), 1)
        self.assertGreaterEqual(ev[0][4]["still_s"], 5.0)

    def test_bump_cross_event(self):
        w = corridor_world()
        w.bumps.append({"cell": (0, 1), "axis": "ns", "pos": 300.0})
        sim = ms.Sim(w)
        sim.y = 150.0
        sim.advance(0.05)
        sim.y = 320.0                        # колёса ещё на бугорке
        sim.advance(0.05)
        self.assertFalse([e for e in sim.events if e[1] == "bump_cross"])
        sim.y = 450.0
        sim.advance(0.05)
        self.assertEqual(len([e for e in sim.events if e[1] == "bump_cross"]), 1)

    def test_kit_lands_behind(self):
        sim = ms.Sim(corridor_world())
        sim.write(b"K:L*")
        at = [e for e in sim.events if e[1] == "kit"][0][4]["at"]
        self.assertLess(at[1], -100)        # курс на север - комплект южнее центра


if __name__ == "__main__":
    unittest.main()
