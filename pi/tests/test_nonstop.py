"""Continuous corridor decisions and mandatory stop exceptions."""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from test_reliability import load, TIMING
import config


class RouteTests(unittest.TestCase):
    def setUp(self):
        self.state = SimpleNamespace(maze={(1, 0): [True, False, True, False]},
            visited={(1, 0)}, tile_map={}, react_cells={}, ramps={}, blocked=set(), side_pref='right')
        # Use the configured side preference spelling.
        self.state.side_pref = next(iter(config.SIDE_ORDER))
        self.map = Mock()
        self.map._cell_at.side_effect = lambda c,h,k: (c[0]+k, c[1])
        self.map._next_cell.side_effect = lambda x,y,d: (x+(0,1,0,-1)[d], y+(1,0,-1,0)[d])
        self.map.open_dirs.side_effect = lambda x,y: ([1] if x < 3 else []) + ([3] if x > 0 else [])
        self.map.route_step_cost.return_value = 1
        self.rules = load('tile_rules', {'state': self.state})
        deps = {n: Mock() for n in ('bumper', 'dash_live', 'driving', 'esp_link', 'floor_tiles',
            'lidar_walls', 'marks', 'ramps', 'recovery', 'turning', 'victims')}
        deps.update(state=self.state, timing=TIMING, maze_map=self.map, tile_rules=self.rules)
        self.module = load('explore', deps)
        self.sides = {'right': (True, 20, 80), 'left': (True, 20, 80)}

    def plan(self, **kwargs):
        return self.module._DrivePlan((0, 0), 1, lambda c: c == (3, 0), home=True, **kwargs)

    def test_known_straight_continues_without_tile_stop(self):
        self.assertTrue(self.plan().decide(1, self.sides, (False, 0, 3))[0])

    def test_fresh_opening_cannot_be_overridden_by_old_straight_map(self):
        self.sides['right'] = (False, 20, 400)
        self.assertFalse(self.plan().decide(1, self.sides, (False, 0, 3))[0])

    def test_turn_destination_and_unknown_walls_still_stop(self):
        plan = self.module._DrivePlan((0, 0), 1, lambda c: c == (1, 1), home=True)
        self.state.maze[(1, 0)] = [False, False, True, False]
        self.assertFalse(plan.decide(1, self.sides, (False, 0, 3))[0])
        plan = self.module._DrivePlan((0, 0), 1, lambda c: c == (1, 0), home=True)
        self.assertFalse(plan.decide(1, self.sides, (False, 0, 3))[0])
        self.state.visited.clear()
        self.state.maze.clear()
        sides = {'right': (None, 0, 0), 'left': (True, 20, 80)}
        self.assertFalse(self.plan().decide(1, sides, (False, 0, 3))[0])

    def test_known_blue_stops_even_if_live_color_is_missing(self):
        self.state.tile_map[(1, 0)] = config.TILE_BLUE
        self.assertFalse(self.plan().decide(1, self.sides, (False, 0, 3))[0])
        self.state.tile_map[(1, 0)] = config.TILE_UNKNOWN
        self.state.react_cells[(1, 0)] = config.TILE_BLUE
        self.assertFalse(self.plan().decide(1, self.sides, (False, 0, 3))[0])

    def test_return_budget_checked_on_continuous_straight(self):
        predicate = Mock(return_value=True)
        self.assertFalse(self.plan(should_stop=predicate).decide(1, self.sides, (False, 0, 3))[0])
        predicate.assert_called_once_with((0, 0))


class TileTests(unittest.TestCase):
    def setUp(self):
        self.state = SimpleNamespace(tile_map={}, react_cells={}, blue_cells=set(), lifted=threading.Event())
        self.rules = load('tile_rules', {'state': self.state})
        self.floor = load('floor_tiles', {'state': self.state, 'esp_link': Mock(),
            'tile_rules': self.rules, 'dash_live': Mock(), 'timing': TIMING})
        self.floor.tile_color = Mock(return_value=config.TILE_BLUE)

    def test_late_blue_detection_counts_cell_once_and_stops_even_after_center(self):
        for traveled in (24, 31):
            with self.subTest(traveled=traveled):
                trip = SimpleNamespace(plan=SimpleNamespace(cell=lambda k: (k, 0)), cells=1,
                    traveled_cm=traveled, boundary_cm=60, cell_cm=30, tiles={}, stopping=False,
                    pending=True, cp_done=set())
                self.floor.check_floor_while_driving(trip)
                self.assertTrue(trip.stopping)
                self.assertEqual(trip.boundary_cm, 30)
                self.assertEqual(trip.cells, 0)
                # A second sensor poll must not undo the count again.
                self.floor.check_floor_while_driving(trip)
                self.assertEqual(trip.cells, 0)
                trip.cells += 1  # drive_stop arrival
                self.assertEqual(trip.cells, 1)

    def test_blue_seen_on_approach_still_reacts_at_arrival(self):
        react = Mock()
        self.rules.RULES[config.TILE_BLUE].react = react
        self.state.tile_map[(1, 0)] = config.TILE_BLUE
        self.rules.on_arrive((1, 0), config.TILE_UNKNOWN)
        react.assert_called_once_with((1, 0))

    def test_blue_preempts_an_existing_farther_stop(self):
        from continuous import PassageLedger
        ledger = PassageLedger()
        ledger.schedule(1, 30, {})
        trip = SimpleNamespace(plan=SimpleNamespace(cell=lambda k: (k, 0)), cells=1,
            traveled_cm=24, boundary_cm=60, cell_cm=30, tiles={}, stopping=True,
            pending=False, cp_done=set(), passages=ledger)
        self.floor.check_floor_while_driving(trip)
        self.assertEqual((trip.boundary_cm, trip.cells), (30, 0))
        self.assertEqual(ledger.pending, [])

    def test_checkpoint_waits_for_center_after_early_continue_decision(self):
        self.floor.tile_color.return_value = config.TILE_SILVER
        trip = SimpleNamespace(plan=Mock(), cells=1, traveled_cm=24, boundary_cm=60,
            cell_cm=30, tiles={}, stopping=False, pending=True, cp_done=set())
        trip.plan.cell.side_effect = lambda k: (k, 0)
        self.floor.check_floor_while_driving(trip)
        trip.plan.checkpoint.assert_not_called()
        trip.traveled_cm = 30.1
        self.floor.check_floor_while_driving(trip)
        trip.plan.checkpoint.assert_called_once_with(1)


if __name__ == '__main__':
    unittest.main()
