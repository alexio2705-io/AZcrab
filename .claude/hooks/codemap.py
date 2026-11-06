#!/usr/bin/env python3
"""Карта кода docs/CODEMAP.md: модуль → первая строка описания → публичные функции.

Без LLM, только из кода, поэтому не устаревает. Запуск руками: `python3 .claude/hooks/codemap.py`.
Как PostToolUse-хук пересобирает карту после правки .py/.c/.h и молчит.
"""
import ast
import json
import os
import re
import subprocess
import sys

OUT = "docs/CODEMAP.md"
GROUPS = [  # (заголовок, шаблоны git ls-files в glob-режиме: * не проходит через /)
    ("Корень", ["*.py"]),
    ("pi/ — Raspberry Pi", ["pi/**/*.py"]),
    ("camera_web/", ["camera_web/*.py"]),
    ("src/ + include/ — ESP32-S3", ["include/*.h", "src/*.c", "src/*.cpp"]),
]
CODE_EXT = (".py", ".c", ".cpp", ".h")
SKIP_DIRS = ("/tests/", "components/")
MAX_FUNCS = 4
MAX_DESC = 90
C_FUNC = re.compile(r"^[A-Za-z_][\w \*]*?\b([a-z_]\w*)\s*\([^;]*\)\s*;", re.M)


def git_files(root, patterns):
    """Отслеживаемые и новые (не в .gitignore) файлы, которые есть на диске."""
    out = subprocess.run(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard",
                          *(":(glob)" + p for p in patterns)],
                         cwd=root, capture_output=True, text=True).stdout.split("\0")
    return [f for f in out if f and os.path.isfile(os.path.join(root, f))
            and not any(s in "/" + f for s in SKIP_DIRS)]


def python_entry(path):
    try:
        tree = ast.parse(open(path, encoding="utf-8").read())
    except (SyntaxError, UnicodeDecodeError):
        return "(не разбирается)", []
    doc = (ast.get_docstring(tree) or "").strip().splitlines()
    names = [n.name for n in tree.body
             if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
             and not n.name.startswith("_")]
    return (doc[0] if doc else "—"), names


def c_comment(text):
    for line in text.splitlines():
        s = line.strip().lstrip("/").strip(" *=-")
        if line.strip().startswith(("//", "/*", "*")) and len(s) > 3:
            return s
    return "—"


def c_entry(path):
    """Заголовок .h: первый комментарий + объявленные функции. Исходник: ссылка на парный .h."""
    text = open(path, encoding="utf-8", errors="replace").read()
    if path.endswith(".h"):
        return c_comment(text), C_FUNC.findall(text)
    base = os.path.splitext(os.path.basename(path))[0]
    header = os.path.join(os.path.dirname(path), "..", "include", base + ".h")
    if os.path.exists(header):
        return f"реализация `include/{base}.h`", []
    return ("точка входа прошивки (`app_main`)" if "app_main" in text else "—"), []


def line_for(path):
    desc, names = (python_entry if path.endswith(".py") else c_entry)(path)
    extra = f" … ещё {len(names) - MAX_FUNCS}" if len(names) > MAX_FUNCS else ""
    funcs = f" · `{'`, `'.join(names[:MAX_FUNCS])}`{extra}" if names else ""
    return f"- `{path}` — {desc[:MAX_DESC]}{funcs}"


def build(root):
    lines = ["# Карта кода", "",
             "Генерируется `.claude/hooks/codemap.py` после каждой правки кода. Руками не править.",
             "Где что менять по задачам — ручной указатель `docs/robot_code_map.md`.",
             "Тесты Pi — `pi/tests/` (unittest).", ""]
    for title, patterns in GROUPS:
        files = sorted(set(git_files(root, patterns)))
        if not files:
            continue
        lines += [f"## {title} ({len(files)})", ""]
        lines += [line_for(os.path.join(root, f)).replace(root + os.sep, "") for f in files]
        lines.append("")
    return "\n".join(lines)


def write_if_changed(root):
    text = build(root)
    path = os.path.join(root, OUT)
    old = open(path, encoding="utf-8").read() if os.path.exists(path) else ""
    if text != old:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)


def should_rebuild(root):
    """Ручной запуск (stdin пуст) или хук после правки кода внутри проекта."""
    raw = "" if sys.stdin.isatty() else sys.stdin.read().strip()
    if not raw:
        return True
    try:
        path = (json.loads(raw).get("tool_input") or {}).get("file_path") or ""
    except ValueError:
        return False
    inside = os.path.abspath(path).startswith(os.path.abspath(root) + os.sep)
    return inside and path.endswith(CODE_EXT) and "/tests/" not in path


if __name__ == "__main__":
    project = os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()
    if should_rebuild(project):
        write_if_changed(project)
