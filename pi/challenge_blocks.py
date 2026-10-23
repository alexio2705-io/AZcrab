"""Small, hardware-free challenge description. Hardware imports happen only in run()."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Block:
    command: str
    value: object = None


def wait(seconds):
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("Wait must be a finite nonnegative duration")
    return Block("wait", seconds)


def blink(count):
    if type(count) is not int or count < 0:
        raise ValueError("Blink count must be a nonnegative integer")
    return Block("blink", count)


def kits(count):
    if type(count) is not int or count < 0:
        raise ValueError("Kit count must be a nonnegative integer")
    return Block("kits", count)


def explore_until(color):
    return Block("explore", color)


def go_to(color):
    return Block("go_to", color)


def home():
    return Block("home")


class Challenge:
    def __init__(self, *steps, limit_s=180, reserve_s=11):
        if not all(math.isfinite(v) and v > 0 for v in (limit_s, reserve_s)):
            raise ValueError("Time limits must be finite and positive")
        if reserve_s >= limit_s:
            raise ValueError("Finish reserve must be shorter than the run")
        self.limit_s, self.reserve_s = limit_s, reserve_s
        self.steps = tuple(steps)
        self.reactions = {}
        self._validate(self.steps)

    @staticmethod
    def _validate(steps, reaction=False):
        allowed = {"wait", "blink", "kits"} if reaction else {
            "wait", "blink", "kits", "explore", "go_to", "home"}
        for step in steps:
            if not isinstance(step, Block) or step.command not in allowed:
                raise ValueError(f"Unsupported {'reaction' if reaction else 'step'}: {step!r}")

    def on(self, color, *steps):
        """Once per tile; reactions cannot recursively start navigation."""
        self._validate(steps, reaction=True)
        self.reactions[color] = tuple(steps)
        return self

    def describe(self):
        lines = [f"Limit {self.limit_s}s; finish reserve {self.reserve_s}s"]
        for color, steps in self.reactions.items():
            lines.append(f"On {color!r}, once per tile: {steps}")
        lines.extend(f"{i}. {s.command}: {s.value}" for i, s in enumerate(self.steps, 1))
        return "\n".join(lines)

    def execute(self, actions, rules, walk):
        """Dependency injection also permits simulation without robot libraries."""
        timer = actions.Timer(self.limit_s)
        previous = dict(rules.RULES)

        def deadline():
            return timer.time_to_go(walk, (0, 0), extra_s=self.reserve_s)

        def perform(step):
            cmd, value = step.command, step.value
            if cmd == "wait": actions.stand(value)
            elif cmd == "blink": actions.blink(value)
            elif cmd == "kits": actions.give_kits(walk, value)
            elif cmd == "home": actions.go_to_cell(walk, (0, 0))
            elif cmd == "explore":
                actions.explore_until(walk, lambda: bool(actions.cells_of(value)) or deadline())
            elif cmd == "go_to":
                target = actions.go_to_color(walk, value)
                if target is None or walk.cell != target:
                    raise RuntimeError(f"No reachable target of colour {value!r}")

        def reaction(steps):
            def react(cell):
                for step in steps:
                    perform(step)
            return react

        try:
            for color, steps in self.reactions.items():
                rules.RULES[color] = rules.Rule(rules.STOP, reaction(steps), every_visit=False)
            actions.look_around(walk)
            for step in self.steps:
                if deadline():
                    actions.go_to_cell(walk, (0, 0))
                    break
                perform(step)
            actions.stand_motors_off(self.reserve_s)
        finally:
            rules.RULES.clear()
            rules.RULES.update(previous)

    def run(self, cameras=False):
        import actions
        import boot
        import tile_rules
        boot.run(lambda run: self.execute(actions, tile_rules, actions.new_walk()), cameras=cameras)
