#!/usr/bin/env python3
"""PostToolUse: после правки pi/nav.py и src/main.c проверяет правила проекта, при нарушении сообщает Claude (exit 2).

pi/nav.py: нет аварийных выходов (FATAL, sys.exit, exit()).
src/main.c: для заезда BUTTON_TEST 0, DRIVE_TEST 0, RUN_MAZE 1.
"""
import json
import os
import re
import sys

data = json.load(sys.stdin)
path = (data.get("tool_input") or {}).get("file_path") or ""
root = os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()
rel = os.path.relpath(path, root) if path else ""

NAV = os.path.join("pi", "nav.py")
MAIN = os.path.join("src", "main.c")

EXIT_RE = re.compile(r"FATAL|\bsys\.exit\b|\bos\._exit\b|\bexit\s*\(|\bquit\s*\(")
SWITCH_RE = re.compile(r"^\s*#\s*define\s+(BUTTON_TEST|DRIVE_TEST|RUN_MAZE)\s+(\S+)")
EXPECTED = {"BUTTON_TEST": "0", "DRIVE_TEST": "0", "RUN_MAZE": "1"}


def read_lines(p):
    with open(p, encoding="utf-8", errors="replace") as f:
        return f.read().splitlines()


def check_nav(p):
    found = []
    for n, line in enumerate(read_lines(p), 1):
        code = line.split("#", 1)[0]
        if EXIT_RE.search(code):
            found.append(f"  {rel}:{n}: {line.strip()}")
    if not found:
        return None
    return (
        "В nav.py нет аварийных выходов: заезд кончается только на старте. "
        "«Старт недостижим» — баг навигации; если пропали моторы или связь с ESP, "
        "ждать и показывать это на дашборде, а не выходить. Нашлось:\n" + "\n".join(found)
    )


def check_main(p):
    bad = []
    for n, line in enumerate(read_lines(p), 1):
        m = SWITCH_RE.match(line)
        if m and m.group(2) != EXPECTED[m.group(1)]:
            bad.append(f"  {rel}:{n}: {line.strip()} (для заезда {EXPECTED[m.group(1)]})")
    if not bad:
        return None
    return (
        "Тестовые переключатели в src/main.c не в режиме заезда. Включать тест можно только "
        "по просьбе пользователя, после теста вернуть и сказать об этом:\n" + "\n".join(bad)
    )


if rel == NAV:
    problem = check_nav(path)
elif rel == MAIN:
    problem = check_main(path)
else:
    sys.exit(0)

if problem:
    print(problem, file=sys.stderr)
    sys.exit(2)
