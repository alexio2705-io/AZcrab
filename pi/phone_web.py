#!/usr/bin/env python3
"""Запуск заезда с телефона: страница с кнопками «Запустить» / «Стоп».

Поднимается сам после загрузки Pi (maze-phone.service) и живёт всё время.
Открыть с телефона в той же Wi-Fi сети:  http://<IP робота>/  (IP - на
дисплее сзади робота). Порт 80, чтобы не набирать номер порта.

«Запустить» делает то же, что команда из SSH или пункт меню на дисплее:
nav.py с выбранными флагами, вывод в ~/maze/run_ЧЧММ.log (oled.Launcher).
Дальше nav.py как обычно ждёт кнопку старта на роботе (GPIO27 - левая рука,
GPIO25 - правая) - или «Старт ← левая» / «Старт правая →» на странице
(remote_start.py), то же и при перезапуске с чекпоинта. Курс поля берётся в
момент нажатия, поэтому жать, когда робот уже стоит на старте. «Стоп» - как
Ctrl+C (nav.py гасит моторы и ленту), не помогло - SIGTERM, потом SIGKILL.

Страница показывает, идёт ли заезд (в том числе запущенный из SSH или с
дисплея), хвост его лога, фазу и счёт из state.json и ссылки на пульт
(:8888) и карту (:8001).

Запуск вручную:  ~/maze/venv/bin/python ~/maze/phone_web.py [--port 8090]
Только стандартная библиотека.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oled  # noqa: E402
import remote_start  # noqa: E402

HTTP_PORT = 80
MAZE_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_PATH = os.path.join(MAZE_DIR, "state.json")     # пишет dash_live._write_state
LOG_LINES = 40
# Тоже держат порт ESP или кнопки - при них nav.py не запускаем.
OTHER_PROGRAMS = ("tile_calib.py", "telem.py", "servo_test.py", "bringup.py",
                  "neopixel_test.py", "lift.py", "color.py", "teleop.py")
MAX_CELLS = 1000


# ── процессы и логи ────────────────────────────────────────────

def proc_cmdlines(proc="/proc"):
    """{pid: cmdline} всех процессов, кроме этого."""
    me = os.getpid()
    out = {}
    try:
        pids = [int(p) for p in os.listdir(proc) if p.isdigit() and int(p) != me]
    except OSError:
        return out
    for pid in pids:
        try:
            with open(os.path.join(proc, str(pid), "cmdline"), "rb") as f:
                out[pid] = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
    return out


def find_script(cmdlines, script):
    """pid процесса python, запущенного со скриптом script, или None."""
    for pid, cmd in sorted(cmdlines.items()):
        words = cmd.split()
        if words and "python" in os.path.basename(words[0]) and \
                any(os.path.basename(w) == script for w in words[1:]):
            return pid
    return None


def proc_start(pid, proc="/proc"):
    """Когда запущен процесс (time.time()) - по /proc/<pid>/stat и btime."""
    try:
        with open(os.path.join(proc, str(pid), "stat")) as f:
            ticks = int(f.read().rsplit(")", 1)[1].split()[19])
        with open(os.path.join(proc, "stat")) as f:
            btime = next(int(ln.split()[1]) for ln in f if ln.startswith("btime"))
        return btime + ticks / os.sysconf("SC_CLK_TCK")
    except (OSError, ValueError, IndexError, StopIteration):
        return time.time()


def stdout_file(pid, proc="/proc"):
    """Файл, куда пишет stdout процесса (лог заезда), или None - если это
    терминал или труба (`| tee` в SSH)."""
    path = os.path.realpath(os.path.join(proc, str(pid), "fd", "1"))
    return path if os.path.isfile(path) else None


def tail(path, n=LOG_LINES):
    """Последние n строк файла (кусок с конца, без чтения целиком)."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 16384))
            text = f.read().decode(errors="replace")
    except OSError:
        return []
    # \r - отсчёт «Старт через N...» переписывает строку на месте
    return [ln.rsplit("\r", 1)[-1] for ln in text.rstrip("\n").split("\n")][-n:]


def waits_button(lines):
    """По логу: nav.py сейчас ждёт кнопку старта на роботе."""
    for ln in reversed(lines):
        if "приоритет:" in ln:
            return False
        if "для старта" in ln:
            return True
    return False


def nav_argv(form):
    """Аргументы nav.py из формы страницы: {"cells": "", "no_color": bool, ...}."""
    argv = ["nav.py"]
    cells = str(form.get("cells") or "").strip()
    if cells:
        if not cells.isdigit() or not 0 < int(cells) <= MAX_CELLS:
            raise ValueError(f"клеток: целое от 1 до {MAX_CELLS}")
        argv.append(cells)
    for key, flag in (("no_color", "--no-color"), ("no_vision", "--no-vision"),
                      ("red_checkpoint", "--red-checkpoint")):
        if form.get(key):
            argv.append(flag)
    return tuple(argv)


def run_info(since, path=STATE_PATH):
    """Фаза и счёт из state.json - если он записан этим заездом."""
    try:
        if os.path.getmtime(path) < since:
            return None
        with open(path) as f:
            st = json.load(f)
    except (OSError, ValueError):
        return None
    victims = [v for v in st.get("victims") or []
               if isinstance(v, dict) and v.get("type") != "fake"]
    return {"phase": st.get("phase"), "ts": st.get("ts"), "robot": st.get("robot"),
            "visited": len(st.get("visited") or []), "victims": len(victims)}


# ── запуск и остановка ─────────────────────────────────────────

class Control:
    """Один nav.py за раз: свой (oled.Launcher) или чужой - из SSH или меню
    дисплея. Чужой тоже видно и можно остановить."""

    def __init__(self, launcher=None):
        self.launcher = launcher or oled.Launcher(base=MAZE_DIR)
        self.lock = threading.Lock()   # два нажатия подряд - один запуск
        self.log = None                # лог своего запуска
        self.stopping = False
        self.last = None               # чем кончился свой прошлый запуск

    def _reap(self):
        proc = self.launcher.proc
        if proc is not None and proc.poll() is not None:
            self.last = {"code": proc.returncode, "log": _name(self.log)}
            self.launcher.poll()

    def _nav_pid(self, cmdlines):
        if self.launcher.running():
            return self.launcher.proc.pid
        return find_script(cmdlines, "nav.py")

    def status(self):
        with self.lock:
            self._reap()
            cmdlines = proc_cmdlines()
            pid = self._nav_pid(cmdlines)
            out = {"nav": None, "last": self.last, "time": time.strftime("%H:%M:%S"),
                   "other": next((s for s in OTHER_PROGRAMS if find_script(cmdlines, s)), None)}
            if pid is None:
                return out
            since = proc_start(pid)
            log = stdout_file(pid)
            lines = tail(log) if log else []
            out["nav"] = {"pid": pid, "own": self.launcher.running(),
                          "since": time.strftime("%H:%M:%S", time.localtime(since)),
                          "cmd": cmdlines.get(pid, ""), "log": _name(log), "lines": lines,
                          "button": waits_button(lines), "info": run_info(since),
                          "stopping": self.stopping}
            return out

    def start(self, form):
        argv = nav_argv(form)
        with self.lock:
            self._reap()
            cmdlines = proc_cmdlines()
            if self._nav_pid(cmdlines) is not None:
                raise RuntimeError("заезд уже идёт")
            other = next((s for s in OTHER_PROGRAMS if find_script(cmdlines, s)), None)
            if other:
                raise RuntimeError(f"запущен {other}: он держит порт ESP или кнопки, сначала останови его")
            if not self.launcher.start(("Проезд лабиринта", "nav", argv)):
                raise RuntimeError("nav.py не запустился, см. journalctl -u maze-phone")
            self.log = stdout_file(self.launcher.proc.pid)
            self.stopping = False
            self.last = None
            print(f"запущен {' '.join(argv)} -> {self.log}", flush=True)

    def go(self, form):
        """«Старт» с телефона вместо кнопки на роботе: nav.py заберёт запрос
        в wait_start_button (remote_start). Пока он кнопку не ждёт - отказ."""
        with self.lock:
            pid = self._nav_pid(proc_cmdlines())
            if pid is None:
                raise RuntimeError("заезд не идёт - сначала «Запустить»")
            log = stdout_file(pid)
            if log and not waits_button(tail(log)):
                raise RuntimeError("nav.py сейчас не ждёт старта")
            remote_start.request(form.get("side"))
            print(f"старт с телефона: {form.get('side')}", flush=True)

    def stop(self):
        with self.lock:
            if self.launcher.running():
                self.stopping = True
                self.launcher.stop()
                return
            pid = find_script(proc_cmdlines(), "nav.py")
            if pid is None:
                raise RuntimeError("заезд не идёт")
            print(f"стоп чужому nav.py (pid {pid})", flush=True)
            threading.Thread(target=stop_pid, args=(pid,), daemon=True, name="phone-stop").start()


def alive(pid, proc="/proc"):
    """Процесс жив и не зомби (у зомби пустой cmdline)."""
    try:
        with open(os.path.join(proc, str(pid), "cmdline"), "rb") as f:
            return bool(f.read())
    except OSError:
        return False


def stop_pid(pid, wait_s=oled.STOP_WAIT_S):
    """Чужой nav.py (SSH, nohup): как oled.Launcher.stop, но только этому
    процессу - Ctrl+C, не вышел - SIGTERM, потом SIGKILL."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.kill(pid, sig)
        except OSError:
            return
        t_end = time.monotonic() + wait_s
        while time.monotonic() < t_end:
            if not alive(pid):
                return
            time.sleep(0.1)


def _name(path):
    return os.path.basename(path) if path else None


# ── HTTP ───────────────────────────────────────────────────────

PAGE_PATH = os.path.join(MAZE_DIR, "phone", "index.html")


class Handler(BaseHTTPRequestHandler):
    control: "Control | None" = None

    def log_message(self, fmt, *args):     # без строки на каждый опрос /status
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            try:
                with open(PAGE_PATH, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError as e:
                self._send(500, {"error": f"нет страницы {PAGE_PATH}: {e}"})
        elif self.path == "/status":
            self._send(200, self.control.status())
        else:
            self._send(404, {"error": "нет такой страницы"})

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
            form = json.loads(self.rfile.read(n) or b"{}") if n else {}
            if self.path == "/start":
                self.control.start(form)
            elif self.path == "/go":
                self.control.go(form)
            elif self.path == "/stop":
                self.control.stop()
            else:
                return self._send(404, {"error": "нет такой команды"})
        except (ValueError, RuntimeError, OSError) as e:
            return self._send(409, {"error": str(e)})
        self._send(200, {"ok": True})


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=HTTP_PORT)
    port = ap.parse_args().port
    Handler.control = Control()
    server = ThreadingHTTPServer(("", port), Handler)
    print(f"Пульт запуска: http://{local_ip()}:{port}/", flush=True)
    server.serve_forever()


def local_ip():
    import socket
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))     # ничего не шлёт, только выбирает интерфейс
            return s.getsockname()[0]
    except OSError:
        return "<IP робота>"


if __name__ == "__main__":
    main()
