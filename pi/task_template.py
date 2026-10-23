"""Training challenge example: change TABLE and FINISH_COLOR before the run.

Not the normal RoboCup 2026 rule set. Run: python task_template.py
"""
import actions
import boot
import config
import state
import tile_rules

# color: (blinks, kits); add another task in a few lines.
TABLE = {config.TILE_BLUE: (3, 1), config.TILE_RED: (2, 0)}
FINISH_COLOR = config.TILE_RED
LIMIT_S = 180
w = None


def react(cell):
    color = state.tile_map.get(cell)
    if color in TABLE:
        blinks, kits = TABLE[color]
        actions.blink(blinks)
        actions.give_kits(w, kits)


for color in TABLE:
    tile_rules.RULES[color] = tile_rules.Rule(tile_rules.STOP, react, every_visit=False)


def scenario(run):
    global w
    w = actions.new_walk()
    timer = actions.Timer(LIMIT_S)
    actions.look_around(w)
    actions.explore_until(w, lambda: bool(actions.cells_of(FINISH_COLOR)) or
                          timer.time_to_go(w, (0, 0), extra_s=11))
    if actions.go_to_color(w, FINISH_COLOR) is None:
        actions.go_to_cell(w, (0, 0))
    actions.stand_motors_off(11)


if __name__ == "__main__":
    boot.run(scenario, cameras=False)
