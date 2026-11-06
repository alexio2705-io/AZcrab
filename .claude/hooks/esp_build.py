#!/usr/bin/env python3
"""PostToolUse: после правки .c/.cpp/.h в src/ и include/ собирает прошивку (pio run), при ошибке сообщает Claude (exit 2)."""
import json
import os
import re
import subprocess
import sys

data = json.load(sys.stdin)
path = (data.get("tool_input") or {}).get("file_path") or ""
root = os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()
rel = os.path.relpath(path, root) if path else ""

if not (rel.startswith(("src" + os.sep, "include" + os.sep)) and rel.endswith((".c", ".cpp", ".h", ".hpp"))):
    sys.exit(0)

pio = os.path.expanduser("~/.platformio/penv/bin/pio")
if not os.path.exists(pio):
    sys.exit(0)

try:
    r = subprocess.run([pio, "run"], cwd=root, capture_output=True, text=True, timeout=110)
except subprocess.TimeoutExpired:
    print(f"pio run: не уложились в 110 с после правки {rel}, соберите вручную", file=sys.stderr)
    sys.exit(2)

if r.returncode != 0:
    lines = (r.stdout + "\n" + r.stderr).strip().splitlines()
    errors = [l for l in lines if re.search(r"error|undefined reference|\*\*\*", l, re.I)]
    tail = "\n".join((errors or lines)[-25:])
    print(f"pio run упал после правки {rel}:\n{tail}", file=sys.stderr)
    sys.exit(2)
