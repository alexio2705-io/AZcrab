#!/usr/bin/env python3
"""PreToolUse: не даёт править сгенерированные файлы вручную (exit 2)."""
import json
import os
import sys

data = json.load(sys.stdin)
path = (data.get("tool_input") or {}).get("file_path") or ""
name = os.path.basename(path)

if name == "compile_commands.json":
    print("compile_commands.json генерируется: pio run -d firmware -t compiledb", file=sys.stderr)
    sys.exit(2)
if name == "CODEMAP.md":
    print("docs/CODEMAP.md генерируется: python3 .claude/hooks/codemap.py (описание модуля — первая строка его docstring)", file=sys.stderr)
    sys.exit(2)
if name.startswith("sdkconfig."):
    print(f"{name} генерируется ESP-IDF (pio run -d firmware -t menuconfig); править руками нельзя", file=sys.stderr)
    sys.exit(2)
