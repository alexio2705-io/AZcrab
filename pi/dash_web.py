#!/usr/bin/env python3
"""Пульт робота: живой дашборд заезда в браузере + запись и повтор заездов.

Идея - как у отладочного экрана Overengineering² (RCJ Line): на одной
странице камеры с рамками, лидар, карта, все датчики, 3D-модель робота,
повёрнутая по гироскопу, статус, лог и большие таймеры. Только это сайт:
открывается с ноутбука или телефона, на роботе ничего не рисуется.

Откуда данные. nav.py (и он же в симуляторе, sim/live_sim.py) раздаёт
снимок состояния 10 раз в секунду по TCP на 127.0.0.1:8766 (см. live.py).
Этот процесс к нему подключается, держит последний снимок, лог и минуту
истории для графиков и раздаёт браузерам через Server-Sent Events. nav.py
перезапустили - переподключится сам; сам пульт переживает заезды.

Камеры. Кадр (320x320 BGR) берётся прямо из общей памяти наблюдателя меток
(victim_watch.py) по имени из снимка и жмётся в JPEG здесь - nav.py за это
ничего не платит. Кадры есть, только пока наблюдатель включён (на ходу).

Запись. Каждый сеанс связи с nav.py пишется целиком в
~/maze/runs/<дата_время>.jsonl.gz (строка = снимок, ~2 МБ за 8 минут), и
любой заезд потом открывается на той же странице с полосой времени - как
повтор: видно, что робот видел и делал в каждый момент.

Запуск (nav.py поднимает его сам; вручную - например, для повтора):
    ~/maze/venv/bin/python ~/maze/dash_web.py
Страница:  http://<IP робота>:8888/
На Маке с симулятором:  python3 pi/dash_web.py  ->  http://localhost:8888/
Только стандартная библиотека; numpy/cv2 нужны лишь для картинок с камер.
"""
from __future__ import annotations

import argparse
import base64
import gzip
import json
import mimetypes
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live  # noqa: E402
import oled  # noqa: E402

HTTP_PORT = 8888
HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(HERE, "dash")
# На роботе - ~/maze/runs; на Маке (симулятор) ~/maze нет - тогда sim/runs.
RUNS_DIR = (os.path.expanduser("~/maze/runs") if os.path.isdir(os.path.expanduser("~/maze"))
            else os.path.join(os.path.dirname(HERE), "sim", "runs"))
DETECTIONS_DIR = os.path.expanduser("~/maze/detections")   # как victim_detect.SAVE_DIR
HIST_S = 60.0                 # секунд истории для графиков при открытии страницы
LOG_KEEP = 3000
SYS_PERIOD_S = 1.0
THROTTLE_PERIOD_S = 5.0
RECONNECT_S = 0.5
REC_FLUSH_S = 2.0
SSE_PING_S = 5.0              # пустое событие - чтобы прокси/браузер не рвали поток
NAV_STALE_S = 1.0             # снимков нет дольше - nav.py считается молчащим



def hist_row(s):
    """Короткая строка истории из снимка: время, скорость-уставка, наклоны,
    поворот, стены, руление, средний путь колёс. Порядок - как H в dash.js."""
    imu = s.get("imu") or {}
    w = s.get("walls") or {}
    dr = s.get("drive") or {}
    odo = s.get("odo") or [None, None]
    om = None if None in odo else round((odo[0] + odo[1]) / 2.0, 2)
    return [s.get("t"), dr.get("spd"), imu.get("pitch"), imu.get("roll"), imu.get("rate"),
            w.get("front"), w.get("left"), w.get("right"), dr.get("steer"), om]


class SysStats:
    """Загрузка CPU, температура, память и флаги питания Pi (vcgencmd)."""

    def __init__(self):
        self._prev = None
        self.data = {}
        self._t_thr = 0.0

    def sample(self):
        d = {}
        try:
            with open("/proc/stat") as f:
                v = [int(x) for x in f.readline().split()[1:]]
            idle, total = v[3] + v[4], sum(v)
            if self._prev is not None:
                di, dt = idle - self._prev[0], total - self._prev[1]
                d["cpu"] = round(100.0 * (1.0 - di / dt), 1) if dt > 0 else None
            self._prev = (idle, total)
        except (OSError, ValueError, IndexError):
            try:
                d["load"] = round(os.getloadavg()[0], 2)
            except (OSError, AttributeError):
                pass
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                d["temp"] = round(int(f.read()) / 1000.0, 1)
        except (OSError, ValueError):
            pass
        try:
            mem = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, v = line.split(":", 1)
                    mem[k] = int(v.split()[0])
            d["mem"] = round(100.0 * (1 - mem["MemAvailable"] / mem["MemTotal"]), 1)
        except (OSError, ValueError, KeyError):
            pass
        now = time.monotonic()
        if now - self._t_thr >= THROTTLE_PERIOD_S:
            self._t_thr = now
            self.data["throttled"] = self._throttled()
        d["throttled"] = self.data.get("throttled")
        self.data = d
        return d

    @staticmethod
    def _throttled():
        """Биты get_throttled: 0 - просадка питания сейчас, 2 - троттлинг
        сейчас, 16/18 - было с загрузки. None - не Pi."""
        try:
            out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                                 text=True, timeout=1.0).stdout
            return int(out.strip().split("=")[1], 16)
        except (OSError, ValueError, IndexError, subprocess.SubprocessError):
            return None


class Hub:
    """Последнее состояние + лог + история; будит SSE-клиентов на каждый снимок."""

    def __init__(self, record=True):
        self.cond = threading.Condition()
        self.snap = None
        self.map = None
        self.map_ver = 0              # растёт при каждой новой карте (своя нумерация)
        self.log = deque(maxlen=LOG_KEEP)
        self.log_seq = 0              # своя нумерация: nav.py перезапускается со своей с нуля
        self.hist = deque()
        self.n = 0                    # номер снимка
        self.t_rx = None              # monotonic последнего снимка
        self.connected = False
        self.session = 0
        self.record = record
        self.rec_path = None
        self._rec = None
        self._rec_n = 0
        self._rec_flush_t = 0.0
        self.sys = {}
        # Дисплей сзади робота: тот же снимок -> тот же oled.compose(), кадр
        # 128x64 (1024 байта, base64) - браузеру. См. oled_loop().
        self.oled_feed = oled.Feed(start=False)
        self.oled = None
        self.oled_ver = 0

    # ── из потока связи с nav.py ───────────────────────────────
    def on_connect(self):
        with self.cond:
            self.connected = True
            self.oled_feed.connected = True
            self.session += 1
            self.hist.clear()
            self._open_rec()
            self._add_log_local("── пульт: связь с nav.py ──")
            self.cond.notify_all()

    def on_disconnect(self):
        with self.cond:
            if self.connected:
                self._add_log_local("── пульт: nav.py отключился ──")
            self.connected = False
            self.oled_feed.connected = False
            self._close_rec()
            self.cond.notify_all()

    def _add_log_local(self, text):
        self.log_seq += 1
        self.log.append((self.log_seq, round(time.time(), 3), text))

    def on_snapshot(self, s):
        lines = s.pop("log", None) or []
        with self.cond:
            for _, ts, text in lines:
                self.log_seq += 1
                self.log.append((self.log_seq, ts, text))
            m = s.pop("map", None)
            self.oled_feed.take(dict(s, log=lines, map=m), time.monotonic())
            if m is not None and m != self.map:
                self.map = m
                self.map_ver += 1
            self.snap = s
            self.n += 1
            self.t_rx = time.monotonic()
            t = s.get("t")
            if t is not None:
                self.hist.append(hist_row(s))
                while self.hist and t - self.hist[0][0] > HIST_S:
                    self.hist.popleft()
            self.cond.notify_all()
        if self._rec is not None:
            try:
                extra = {}
                if m is not None:
                    extra["map"] = m
                if lines:
                    extra["log"] = lines
                self._rec.write(json.dumps(dict(s, **extra) if extra else s,
                                           separators=(",", ":")) + "\n")
                self._rec_n += 1
                # Сброс на диск раз в REC_FLUSH_S: пульт могут убить (kill,
                # питание) - потеряется не больше пары секунд, а не весь заезд.
                now = time.monotonic()
                if now - self._rec_flush_t >= REC_FLUSH_S:
                    self._rec_flush_t = now
                    self._rec.flush()
            except (OSError, ValueError):
                self._close_rec()

    def _open_rec(self):
        if not self.record:
            return
        try:
            os.makedirs(RUNS_DIR, exist_ok=True)
            base = os.path.join(RUNS_DIR, time.strftime("%Y-%m-%d_%H-%M-%S"))
            path, k = base + ".jsonl.gz", 1
            while os.path.exists(path):          # два сеанса в одну секунду
                k += 1
                path = f"{base}-{k}.jsonl.gz"
            self.rec_path = path
            self._rec = gzip.open(self.rec_path, "wt", compresslevel=5)
            self._rec_n = 0
            self._rec_flush_t = time.monotonic()
        except OSError as e:
            print(f"[WARN] запись заезда не открылась: {e}", flush=True)
            self._rec = None
            self.rec_path = None

    def _close_rec(self):
        if self._rec is not None:
            try:
                self._rec.close()
            except OSError:
                pass
            # Сеанс без единого снимка (nav.py подключился и сразу вышел) -
            # пустой файл в списке записей не нужен.
            if self._rec_n == 0 and self.rec_path:
                try:
                    os.remove(self.rec_path)
                except OSError:
                    pass
        self._rec = None

    # ── для страниц ────────────────────────────────────────────
    def link(self):
        age = None if self.t_rx is None else time.monotonic() - self.t_rx
        return {"nav": self.connected and age is not None and age < NAV_STALE_S,
                "connected": self.connected, "age": None if age is None else round(age, 2),
                "rec": os.path.basename(self.rec_path) if self.rec_path and self._rec else None,
                "sys": self.sys, "session": self.session}

    def message(self, client):
        """Сообщение клиенту: снимок, новые строки лога, карта, если у него
        старая. client - его dict: log_seq, map_ver, n."""
        msg = {"s": self.snap, "link": self.link()}
        if self.log and self.log[-1][0] > client["log_seq"]:
            lines = [ln for ln in self.log if ln[0] > client["log_seq"]][-500:]
            msg["log"] = lines
            client["log_seq"] = lines[-1][0]
        if self.map_ver != client["map_ver"]:
            msg["map"] = self.map
            client["map_ver"] = self.map_ver
        client["n"] = self.n
        self.oled_message(client, msg)
        return msg

    def oled_message(self, client, msg):
        if self.oled is not None and self.oled_ver != client.get("oled_ver"):
            msg["oled"] = self.oled
            client["oled_ver"] = self.oled_ver
        return msg


def feed_loop(hub, host, port):
    """Связь с публикатором nav.py: подключиться, читать, при обрыве - снова."""
    while True:
        try:
            for i, s in enumerate(live.read_lines(host, port)):
                if i == 0:
                    hub.on_connect()
                hub.on_snapshot(s)
        except (OSError, ConnectionError):
            pass
        hub.on_disconnect()
        time.sleep(RECONNECT_S)


def render_oled(hub, cv, sysinfo, now):
    """Один кадр дисплея в hub.oled. True - кадр поменялся (клиентам уйдёт)."""
    try:
        oled.compose(cv, oled.build_state(now, hub.oled_feed, sysinfo))
    except Exception as e:              # noqa: BLE001 - пульт не должен падать из-за экрана
        cv.clear()
        oled.header(cv, "ОШИБКА ЭКРАНА")
        oled.block(cv, 0, 5, f"{type(e).__name__}: {e}", now)
    frame = base64.b64encode(bytes(cv.buf)).decode()
    if frame == hub.oled:
        return False
    with hub.cond:
        hub.oled = frame
        hub.oled_ver += 1
        hub.cond.notify_all()
    return True


def oled_loop(hub):
    """Кадр дисплея с частотой самого демона (oled.FPS). Без снимков он тоже
    живой: часы, мигание шапки, «nav.py ВЫКЛ». Шлётся только изменившийся."""
    sysinfo = oled.SysInfo()
    cv = oled.Canvas()
    while True:
        t0 = time.monotonic()
        render_oled(hub, cv, sysinfo, t0)
        time.sleep(max(0.0, 1.0 / oled.FPS - (time.monotonic() - t0)))


def sys_loop(hub):
    st = SysStats()
    while True:
        hub.sys = st.sample()
        time.sleep(SYS_PERIOD_S)


# ── картинки с камер ───────────────────────────────────────────
_shm_cache = {}
_shm_lock = threading.Lock()


def camera_jpeg(info):
    with _shm_lock:
        return _camera_jpeg_locked(info)


def _camera_jpeg_locked(info):
    """JPEG последнего кадра стороны из общей памяти наблюдателя или None."""
    try:
        import numpy as np
        import cv2
        from multiprocessing import shared_memory
    except ImportError:
        return None
    name = info.get("shm")
    if not name:
        return None
    shm = _shm_cache.get(name)
    if shm is None:
        try:
            shm = (shared_memory.SharedMemory(name=name, track=False) if sys.version_info >= (3, 13)
                   else shared_memory.SharedMemory(name=name))
            if sys.version_info < (3, 13):
                # не отдавать чужой сегмент своему resource_tracker (см. victim_watch.attach_shm)
                from multiprocessing import resource_tracker
                resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
        except (OSError, ValueError):
            return None
        _shm_cache[name] = shm
        while len(_shm_cache) > 6:
            old = next(iter(_shm_cache))
            _shm_cache.pop(old).close()
    offset = 8 if info.get("preview") else 0
    before = int.from_bytes(shm.buf[:8], "little") if offset else 0
    if offset and (not before or before & 1):
        return None
    frame = np.ndarray((320, 320, 3), np.uint8, buffer=shm.buf, offset=offset).copy()
    if offset and before != int.from_bytes(shm.buf[:8], "little"):
        return None
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return buf.tobytes() if ok else None


# ── повтор ─────────────────────────────────────────────────────
# Запись - файл в RUNS_DIR или в папке на уровень ниже (ночной прогон sim/night.py).
_RUN_RE = re.compile(r"^(?:[\w-][\w.-]*/)?[\w-][\w.-]*\.jsonl(\.gz)?$")


def list_runs():
    out = []
    try:
        names = sorted(os.listdir(RUNS_DIR), reverse=True)
    except OSError:
        return out
    for n in names:
        p = os.path.join(RUNS_DIR, n)
        sub = [f"{n}/{f}" for f in sorted(os.listdir(p))] if os.path.isdir(p) else [n]
        for name in sub:
            if _RUN_RE.match(name):
                q = os.path.join(RUNS_DIR, name)
                try:
                    out.append({"name": name, "size": os.path.getsize(q), "mtime": os.path.getmtime(q)})
                except OSError:
                    pass
    return out


# ── HTTP ───────────────────────────────────────────────────────
class QuietServer(ThreadingHTTPServer):
    """Браузер закрыл соединение (вкладка, keep-alive, перезагрузка повтора) -
    это не ошибка: без трассировки в консоль. Остальное - как обычно."""

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], (ConnectionResetError, BrokenPipeError,
                                          ConnectionAbortedError, TimeoutError)):
            return
        super().handle_error(request, client_address)


class Handler(BaseHTTPRequestHandler):
    hub: Hub = None  # type: ignore[assignment]
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):     # без строки в лог на каждый запрос
        pass

    def _send(self, code, body, ctype, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(200, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        path = unquote(urlparse(self.path).path)
        try:
            if path in ("/", "/index.html"):
                return self._static("index.html")
            if path.startswith("/dash/"):
                return self._static(path[len("/dash/"):])
            if path == "/events":
                return self._events()
            if path == "/state":
                h = self.hub
                with h.cond:
                    return self._json({"s": h.snap, "map": h.map, "link": h.link(),
                                       "log": list(h.log)[-500:], "hist": list(h.hist),
                                   "oled": h.oled})
            if path == "/cam/front.jpg":
                preview = "/dev/shm/maze_front.jpg"
                try:
                    if not self.hub.link()["nav"] or time.time() - os.stat(preview).st_mtime > 1:
                        return self._send(404, b"stale frame", "text/plain")
                    return self._file(preview)
                except OSError:
                    return self._send(404, b"no front frame", "text/plain")
            m = re.match(r"^/cam/(left|right)\.jpg$", path)
            if m:
                return self._camera(m.group(1))
            if path == "/runs":
                cur = self.hub.link()["rec"]
                return self._json([dict(r, active=r["name"] == cur) for r in list_runs()])
            m = re.match(r"^/runs/(.+)$", path)
            if m and _RUN_RE.match(m.group(1)) and ".." not in m.group(1):
                return self._run_file(m.group(1))
            m = re.match(r"^/detections/([\w.-]+)$", path)
            if m:
                return self._file(os.path.join(DETECTIONS_DIR, m.group(1)))
            self._send(404, b"not found", "text/plain")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _static(self, rel):
        p = os.path.normpath(os.path.join(STATIC_DIR, rel))
        if not p.startswith(STATIC_DIR + os.sep) and p != STATIC_DIR:
            return self._send(403, b"forbidden", "text/plain")
        self._file(p, cache=rel.startswith("vendor/"))

    def _file(self, p, cache=False):
        try:
            with open(p, "rb") as f:
                body = f.read()
        except OSError:
            return self._send(404, b"not found", "text/plain")
        ctype = mimetypes.guess_type(p)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype.endswith("javascript"):
            ctype += "; charset=utf-8"
        self._send(200, body, ctype, {"Cache-Control": "max-age=86400"} if cache else None)

    def _camera(self, side):
        snap = self.hub.snap or {}
        info = (snap.get("cams") or {}).get(side)
        if (not self.hub.link()["nav"] or not info or info.get("dead") or
                info.get("preview_t") is None or snap.get("t", 0) - info["preview_t"] > 1):
            return self._send(404, b"stale frame", "text/plain")
        jpg = camera_jpeg(info) if info else None
        if jpg is None:
            return self._send(404, b"no frame", "text/plain")
        self._send(200, jpg, "image/jpeg")

    def _run_file(self, name):
        p = os.path.join(RUNS_DIR, name)
        try:
            with open(p, "rb") as f:
                body = f.read()
        except OSError:
            return self._send(404, b"not found", "text/plain")
        if name.endswith(".gz"):
            # Последний заезд мог ещё писаться - распаковываем здесь сами:
            # незакрытый gzip браузер не разжал бы.
            try:
                body = gzip.decompress(body)
            except (OSError, EOFError):
                import zlib
                body = zlib.decompressobj(16 + zlib.MAX_WBITS).decompress(body)
        self._send(200, body, "application/x-ndjson; charset=utf-8")

    def _events(self):
        self.connection.settimeout(2.0)  # disconnect slow clients, do not queue old states
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        h = self.hub
        client = {"log_seq": 0, "map_ver": -1, "n": -1, "oled_ver": -1}
        with h.cond:
            first = h.message(client)
            first["hist"] = list(h.hist)
            first["init"] = True
        self._sse(first)
        last_ping = time.monotonic()
        while True:
            with h.cond:
                h.cond.wait_for(lambda: h.n != client["n"] or h.oled_ver != client["oled_ver"],
                                timeout=SSE_PING_S)
                if h.n != client["n"]:
                    msg = h.message(client)
                elif h.oled_ver != client["oled_ver"]:
                    # только экран: снимок не повторяем - он ушёл бы в историю дважды
                    msg = h.oled_message(client, {})
                else:
                    msg = None
                if msg is None and time.monotonic() - last_ping >= SSE_PING_S:
                    msg = {"link": h.link()}
            if msg is not None:
                self._sse(msg)
                last_ping = time.monotonic()

    def _sse(self, obj):
        data = json.dumps(obj, separators=(",", ":"))
        self.wfile.write(f"data: {data}\n\n".encode())
        self.wfile.flush()


def _on_term(_signum, _frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=HTTP_PORT)
    ap.add_argument("--live-host", default="127.0.0.1")
    ap.add_argument("--live-port", type=int, default=live.LIVE_PORT)
    ap.add_argument("--no-record", action="store_true", help="не писать заезды в ~/maze/runs")
    a = ap.parse_args()

    hub = Hub(record=not a.no_record)
    Handler.hub = hub
    try:
        srv = QuietServer(("0.0.0.0", a.port), Handler)
    except OSError as e:
        # Пульт уже работает (с прошлого заезда) - он и подхватит этот nav.py.
        print(f"Порт {a.port} занят ({e}) - пульт уже запущен, выхожу.", flush=True)
        return
    srv.daemon_threads = True
    # SIGTERM (nav.py/systemd гасит пульт) - закрыть запись как следует.
    signal.signal(signal.SIGTERM, _on_term)
    threading.Thread(target=feed_loop, args=(hub, a.live_host, a.live_port), daemon=True).start()
    threading.Thread(target=sys_loop, args=(hub,), daemon=True).start()
    threading.Thread(target=oled_loop, args=(hub,), daemon=True).start()
    print(f"Пульт: http://localhost:{a.port}/  (nav.py: {a.live_host}:{a.live_port}, "
          f"записи: {RUNS_DIR if not a.no_record else 'выкл'})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        hub.on_disconnect()


if __name__ == "__main__":
    main()
