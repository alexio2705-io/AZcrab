"""Training example. Preview: python task_blocks.py --show; run without --show."""
import sys
import config
from challenge_blocks import Challenge, blink, kits, explore_until, go_to, home

# Change only these blocks to describe the exercise.
task = Challenge(
    explore_until(config.TILE_RED),
    go_to(config.TILE_RED),
    home(),
    limit_s=180,
)
task.on(config.TILE_RED, blink(2), kits(1))

if __name__ == "__main__":
    if sys.argv[1:] == ["--show"]:
        print(task.describe())
    elif sys.argv[1:]:
        raise SystemExit("Usage: python task_blocks.py [--show]")
    else:
        task.run()
