#!/usr/bin/env python3
"""PostToolUse: после правки .py в pi/ гонит юнит-тесты, при падении сообщает Claude (exit 2)."""
import json
import os
import subprocess
import sys

data = json.load(sys.stdin)
path = (data.get("tool_input") or {}).get("file_path") or ""
root = os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()
rel = os.path.relpath(path, root) if path else ""

if not (rel.startswith("pi" + os.sep) and rel.endswith(".py")):
    sys.exit(0)

try:
    r = subprocess.run(
        [sys.executable, "-m", "unittest", "discover", "-s", "pi/tests"],
        cwd=root, capture_output=True, text=True, timeout=60,
    )
except subprocess.TimeoutExpired:
    print("pi/tests: не уложились в 60 с", file=sys.stderr)
    sys.exit(2)

if r.returncode != 0:
    tail = "\n".join((r.stderr or r.stdout).strip().splitlines()[-25:])
    print(f"pi/tests упали после правки {rel}:\n{tail}", file=sys.stderr)
    sys.exit(2)
