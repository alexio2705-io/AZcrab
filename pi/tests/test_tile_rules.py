"""tile_rules: таблица «цвет -> роль и реакция» и карта цветов state.tile_map."""
import unittest

from fakes import fake_module, import_with

import config


class TileRulesTest(unittest.TestCase):
    def setUp(self):
        self.st = fake_module("state", tile_map={}, react_cells={},
                              CHECKPOINT_TILE=config.TILE_SILVER)
        self.tr = import_with("tile_rules", state=self.st)
        self.calls = []

    def react(self, cell):
        self.calls.append(cell)

    def test_normal_run_table(self):
        tr = self.tr
        self.assertEqual(tr.role(config.TILE_BLACK), tr.WALL)
        self.assertEqual(tr.role(config.TILE_BLUE), tr.STOP)
        self.assertEqual(tr.role(config.TILE_SILVER), tr.CHECKPOINT)
        self.assertEqual(tr.role(config.TILE_RED), tr.NORMAL)
        self.assertEqual(tr.role(config.TILE_WHITE), tr.NORMAL)
        self.assertEqual(tr.role(config.TILE_UNKNOWN), tr.NORMAL)
        self.assertTrue(tr.must_stop({config.TILE_WHITE, config.TILE_BLUE}))
        self.assertFalse(tr.must_stop({config.TILE_WHITE, config.TILE_RED}))

    def test_red_checkpoint(self):
        tr = self.tr
        tr.use_checkpoint(config.TILE_RED)
        self.assertEqual(tr.role(config.TILE_RED), tr.CHECKPOINT)
        self.assertEqual(tr.role(config.TILE_SILVER), tr.NORMAL)
        self.assertEqual(self.st.CHECKPOINT_TILE, config.TILE_RED)

    def test_remember_skips_unknown(self):
        self.tr.remember((1, 2), config.TILE_RED)
        self.tr.remember((1, 2), config.TILE_UNKNOWN)
        self.tr.remember((3, 3), None)
        self.assertEqual(self.st.tile_map, {(1, 2): config.TILE_RED})
        self.assertEqual(self.tr.cells_of(config.TILE_RED), [(1, 2)])

    def test_reacts_every_visit_and_on_a_bad_reading(self):
        tr = self.tr
        tr.RULES[config.TILE_BLUE] = tr.Rule(tr.STOP, react=self.react)
        tr.on_arrive((0, 1), config.TILE_BLUE)
        tr.on_arrive((0, 1), config.TILE_WHITE)      # датчик соврал - помним синюю
        tr.on_arrive((0, 2), config.TILE_WHITE)
        self.assertEqual(self.calls, [(0, 1), (0, 1)])
        self.assertEqual(self.st.tile_map[(0, 1)], config.TILE_WHITE)

    def test_once_rule(self):
        tr = self.tr
        tr.RULES[config.TILE_RED] = tr.Rule(tr.NORMAL, react=self.react, every_visit=False)
        tr.on_arrive((2, 0), config.TILE_RED)
        tr.on_arrive((2, 0), config.TILE_RED)
        self.assertEqual(self.calls, [(2, 0)])


if __name__ == "__main__":
    unittest.main()
