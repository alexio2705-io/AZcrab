#!/usr/bin/env python3
"""Заливка файлов репозитория на Pi в ~/maze с проверками.

    python3 .claude/skills/pi_connect/deploy.py HOST ФАЙЛ [ФАЙЛ ...] [--dry-run] [--force]

ФАЙЛ - путь в репозитории: pi/nav.py, pi/victim_detect.py, pi/dash/dash.js ...
Куда ложится: pi/X -> ~/maze/X, X в корне -> ~/maze/X (вложенные папки
сохраняются: pi/dash/dash.js -> ~/maze/dash/dash.js).

Для каждого файла:
  * одинаков с Pi (md5) - пропуск;
  * копия на Pi не совпадает ни с рабочей копией, ни с одной версией из git -
    на Pi правка, которой нет в репозитории: СТОП (её надо сначала забрать
    и посмотреть; --force - перезаписать всё равно, старое уйдёт в .bak);
  * иначе: .bak старой копии, scp, сверка md5, py_compile для .py в venv.
Перед заливкой печатает, запущен ли nav.py (заливать под живым заездом нельзя
- но решает человек, скрипт только предупреждает).

Только ключ (BatchMode): пароль не спрашивается и не передаётся.
Код выхода: 0 - всё залито или уже было, 1 - ошибка, 2 - стоп из-за правок на Pi.
"""
from __future__ import annotations

import hashlib
import os
import shlex
import subprocess
import sys

REPO = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True,
                      text=True, check=True).stdout.strip()
REMOTE_DIR = "maze"            # относительно домашней папки на Pi
VENV_PY = "~/maze/venv/bin/python"
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=6"]

# Для проверки самого скрипта без Pi: "удалённая" сторона - локальная папка.
FAKE_HOME = os.environ.get("PI_DEPLOY_FAKE_HOME")


def remote(host: str, cmd: str) -> subprocess.CompletedProcess:
    if FAKE_HOME:
        cmd = cmd.replace(VENV_PY, sys.executable).replace("~/", FAKE_HOME + "/")
        return subprocess.run(["bash", "-c", f"cd {shlex.quote(FAKE_HOME)} && {cmd}"],
                              capture_output=True, text=True)
    return subprocess.run(["ssh", *SSH_OPTS, host, cmd], capture_output=True, text=True)


def upload(host: str, src: str, dst_rel: str) -> subprocess.CompletedProcess:
    if FAKE_HOME:
        return subprocess.run(["cp", src, os.path.join(FAKE_HOME, dst_rel)],
                              capture_output=True, text=True)
    return subprocess.run(["scp", "-q", *SSH_OPTS, src, f"{host}:{dst_rel}"],
                          capture_output=True, text=True)


def md5(data: bytes) -> str:
    return hashlib.md5(data).hexdigest()


def remote_rel(path: str) -> str:
    rel = path[len("pi/"):] if path.startswith("pi/") else path
    return f"{REMOTE_DIR}/{rel}"


def git_versions(path: str) -> set[str]:
    """md5 всех версий файла в истории HEAD (до 300 коммитов)."""
    out = subprocess.run(["git", "-C", REPO, "rev-list", "-n", "300", "HEAD", "--", path],
                         capture_output=True, text=True).stdout.split()
    seen = set()
    for sha in out:
        blob = subprocess.run(["git", "-C", REPO, "show", f"{sha}:{path}"], capture_output=True)
        if blob.returncode == 0:
            seen.add(md5(blob.stdout))
    return seen


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if len(args) < 2:
        print(__doc__)
        return 1
    host, files = args[0], args[1:]
    dry, force = "--dry-run" in flags, "--force" in flags

    for f in files:
        if not os.path.isfile(os.path.join(REPO, f)):
            print(f"[ОШИБКА] нет такого файла в репозитории: {f}")
            return 1
        if f.endswith(".py"):
            r = subprocess.run([sys.executable, "-c", "import ast,sys; ast.parse(open(sys.argv[1]).read())",
                                os.path.join(REPO, f)], capture_output=True, text=True)
            if r.returncode != 0:
                print(f"[ОШИБКА] {f} не разбирается локально - не заливаю ничего:")
                print("\n".join(r.stderr.strip().splitlines()[-3:]))
                return 1

    ping = remote(host, "hostname")
    if ping.returncode != 0:
        print(f"[ОШИБКА] нет связи с {host}: {(ping.stderr or ping.stdout).strip()}")
        return 1
    running = "" if FAKE_HOME else remote(host, "pgrep -af '[n]av.py' || true").stdout.strip()
    if running:
        print(f"[ВНИМАНИЕ] на Pi сейчас запущен nav.py:\n  {running}")

    rels = [remote_rel(f) for f in files]
    quoted = " ".join(shlex.quote(r) for r in rels)
    sums = remote(host, f"for f in {quoted}; do if [ -f \"$f\" ]; then md5sum \"$f\"; "
                        f"else echo \"- $f\"; fi; done").stdout.split("\n")
    on_pi = {}
    for line in sums:
        parts = line.split()
        if len(parts) == 2:
            on_pi[parts[1]] = None if parts[0] == "-" else parts[0]

    todo, blocked = [], []
    for f, rel in zip(files, rels):
        local = md5(open(os.path.join(REPO, f), "rb").read())
        pi = on_pi.get(rel)
        if pi == local:
            print(f"  = {f}  (на Pi то же самое)")
            continue
        if pi is not None and pi not in git_versions(f):
            blocked.append((f, rel))
            print(f"  ! {f}  на Pi правка, которой нет ни в рабочей копии, ни в git")
            if not force:
                continue
        print(f"  → {f}  ->  ~/{rel}" + ("  (новый)" if pi is None else ""))
        todo.append((f, rel, local, pi is not None))

    if blocked and not force:
        print("\nСТОП: сначала забрать эти файлы с Pi и посмотреть разницу:")
        for f, rel in blocked:
            print(f"  scp {host}:{rel} <scratchpad>/  &&  diff <scratchpad>/{os.path.basename(rel)} {f}")
        print("Или --force: перезаписать, старая копия останется в .bak.")
        return 2
    if dry or not todo:
        print("\n(ничего не залито)" if dry else "\nВсё уже на месте.")
        return 0

    for f, rel, local, existed in todo:
        d = os.path.dirname(rel)
        prep = f"mkdir -p {shlex.quote(d)}" + (f" && cp {shlex.quote(rel)} {shlex.quote(rel)}.bak" if existed else "")
        r = remote(host, prep)
        if r.returncode != 0:
            print(f"[ОШИБКА] {f}: {r.stderr.strip()}")
            return 1
        r = upload(host, os.path.join(REPO, f), rel)
        if r.returncode != 0:
            print(f"[ОШИБКА] scp {f}: {r.stderr.strip()}")
            return 1
        got = remote(host, f"md5sum {shlex.quote(rel)}").stdout.split()
        if not got or got[0] != local:
            print(f"[ОШИБКА] {f}: md5 на Pi не совпал после копирования")
            return 1
        if f.endswith(".py"):
            r = remote(host, f"{VENV_PY} -m py_compile {shlex.quote(rel)}")
            if r.returncode != 0:
                # Чаще всего синтаксис новее Python 3.11 из venv на Pi.
                back = f"mv {shlex.quote(rel)}.bak {shlex.quote(rel)}" if existed else f"rm {shlex.quote(rel)}"
                remote(host, back)
                print(f"[ОШИБКА] {f} не компилируется на Pi (венв 3.11), вернул как было:\n{r.stderr.strip()}")
                return 1
        print(f"  ✓ {f}" + ("  (старая копия: .bak)" if existed else ""))
    print("\nЗалито и проверено.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
