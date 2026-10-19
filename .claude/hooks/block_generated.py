#!/usr/bin/env python3
"""PreToolUse: не даёт править сгенерированные файлы вручную (exit 2)."""
import json
import os
import sys

data = json.load(sys.stdin)
path = (data.get("tool_input") or {}).get("file_path") or ""
name = os.path.basename(path)

if name == "compile_commands.json":
    print("compile_commands.json генерируется: pio run -t compiledb", file=sys.stderr)
    sys.exit(2)
if name.startswith("sdkconfig."):
    print(f"{name} генерируется ESP-IDF (pio run -t menuconfig); править руками нельзя", file=sys.stderr)
    sys.exit(2)
