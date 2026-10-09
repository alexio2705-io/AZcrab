"""Живой поток состояния nav.py для пульта (dash_web.py).

nav.py отдаёт снимок своего состояния (телеметрия, лидар, поза, карта,
камеры, лог) строкой JSON LIVE_HZ раз в секунду по TCP на 127.0.0.1:LIVE_PORT.
Клиент - обычно один dash_web.py - подключается сам и переподключается,
если nav.py перезапустили. Нет клиента - снимок даже не собирается.

Почему так, а не Flask внутри nav.py: у пульта свой процесс и свой GIL, а
здесь остаётся один поток, который 10 раз в секунду на доли миллисекунды
берёт GIL (json.dumps на C). Медленный клиент не тормозит nav.py: отправка
идёт из этого же потока с таймаутом, зависший клиент просто отключается.

Тот же модуль работает в симуляторе (sim/live_sim.py): там nav.time - часы
модели, а здесь своё настоящее время, поэтому поток тикает по-настоящему.

LogTee - обёртка над sys.stdout: всё, что nav.py печатает, идёт в терминал
как раньше и заодно в кольцевой буфер строк для консоли пульта. Строка
статуса drive_cells ("\\r кл.2 ост.120мм ...") в лог не попадает - она
держится отдельно как текущий статус.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque

LIVE_PORT = 8766
LIVE_HZ = 10.0
SEND_TIMEOUT_S = 0.5        # клиент не принял строку за это время - отключаем
LOG_KEEP = 2000             # строк лога в памяти (новый клиент получает все)


class LogTee:
    """sys.stdout, который ещё и запоминает строки (seq, time, text)."""

    def __init__(self, stream, keep=LOG_KEEP, clock=time.time):
        self.stream = stream
        self.lines = deque(maxlen=keep)
        self.seq = 0
        self.status = ""            # текущая строка "\r..." без перевода строки
        self._part = ""
        self._clock = clock
        self._lock = threading.Lock()

    def write(self, s):
        n = self.stream.write(s)
        try:
            self._feed(s)
        except Exception:           # лог пульта не должен ронять print
            pass
        return n

    def _feed(self, s):
        with self._lock:
            buf = self._part + s
            *done, self._part = buf.split("\n")
            for line in done:
                # "\r" перезаписывает строку в терминале - остаётся последний
                # непустой кусок, как его и видно глазами.
                text = next((p for p in reversed(line.split("\r")) if p.strip()), "")
                if text.strip():
                    self.seq += 1
                    self.lines.append((self.seq, round(self._clock(), 3), text.rstrip()))
                    self.status = ""
            if "\r" in self._part:
                self._part = self._part.rsplit("\r", 1)[1]
                self.status = self._part.strip()

    def since(self, seq):
        with self._lock:
            if not self.lines or self.lines[-1][0] <= seq:
                return []
            return [ln for ln in self.lines if ln[0] > seq]

    def flush(self):
        self.stream.flush()

    def __getattr__(self, name):          # fileno, isatty, encoding ...
        return getattr(self.stream, name)


class LivePublisher:
    """TCP-сервер, который раздаёт snapshot() всем подключённым клиентам.

    snapshot() вызывается из потока публикации - он должен только читать
    состояние nav.py, ничего не ждать и не менять. Исключение в нём не
    роняет поток: снимок пропускается, ошибка печатается один раз.
    """

    def __init__(self, snapshot, tee=None, port=LIVE_PORT, hz=LIVE_HZ, host="127.0.0.1"):
        self.snapshot = snapshot
        self.tee = tee
        self.period = 1.0 / hz
        self._clients = []          # [socket, log_seq]
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._err_shown = False
        self.sent = 0
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(4)
        self.port = self._srv.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True, name="live-accept").start()
        threading.Thread(target=self._loop, daemon=True, name="live-send").start()

    def close(self):
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass
        with self._lock:
            for c, _ in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()

    def _accept(self):
        while not self._stop.is_set():
            try:
                c, _ = self._srv.accept()
            except OSError:
                return
            c.settimeout(SEND_TIMEOUT_S)
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._clients.append([c, 0])

    def _loop(self):
        nxt = time.monotonic()
        n = 0
        while not self._stop.is_set():
            nxt += self.period
            with self._lock:
                clients = list(self._clients)
            if clients:
                self._publish(clients, n)
                n += 1
            dt = nxt - time.monotonic()
            if dt > 0:
                self._stop.wait(dt)
            else:
                nxt = time.monotonic()     # отстали (машина занята) - не догоняем пачкой

    def _publish(self, clients, n):
        try:
            snap = self.snapshot(n)
        except Exception as e:
            if not self._err_shown:
                self._err_shown = True
                print(f"  [WARN] пульт: снимок не собрался: {type(e).__name__}: {e}")
            return
        if self.tee is not None:
            snap["status"] = self.tee.status
        for cl in clients:
            c, seq = cl
            if self.tee is not None:
                lines = self.tee.since(seq)
                snap["log"] = lines
                if lines:
                    cl[1] = lines[-1][0]
            try:
                data = json.dumps(snap, separators=(",", ":"), default=_json_default)
                c.sendall(data.encode() + b"\n")
                self.sent += 1
            except (OSError, ValueError, TypeError):
                with self._lock:
                    if cl in self._clients:
                        self._clients.remove(cl)
                try:
                    c.close()
                except OSError:
                    pass


def _json_default(o):
    # numpy-числа, множества, кортежи-ключи - всё, что json сам не умеет
    if isinstance(o, (set, frozenset)):
        return list(o)
    for attr in ("item", "tolist"):
        f = getattr(o, attr, None)
        if callable(f):
            return f()
    return str(o)


def read_lines(host="127.0.0.1", port=LIVE_PORT, timeout=2.0):
    """Генератор снимков с публикатора (для dash_web.py и тестов). Бросает
    OSError, когда соединение рвётся - переподключение на стороне вызывающего."""
    s = socket.create_connection((host, port), timeout=timeout)
    s.settimeout(timeout)
    buf = b""
    try:
        while True:
            chunk = s.recv(65536)
            if not chunk:
                raise ConnectionError("публикатор закрыл соединение")
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    yield json.loads(line)
                except ValueError:
                    continue
    finally:
        s.close()
