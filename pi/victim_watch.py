"""Наблюдатель меток жертв на ходу - сторона nav.py.

Кадры снимают потоки здесь (picamera2 не открыть из двух процессов, а скан
стоя - VictimDetector в nav.py - пользуется теми же камерами), а YOLO крутится
в отдельном процессе watch_proc.py: со своим GIL, на одном ядре. Цена для
nav.py - копия 300 КБ в общую память и строка в pipe на кадр.

nav.py на каждом такте drive_cells говорит arm(стороны) и забирает рамки
poll(). Кадр с камеры - не чаще WATCH_HZ и только когда воркер закончил
предыдущий кадр этой камеры: очереди нет, всегда свежий кадр. На время
включения камера переходит на короткую выдержку (Camera.fast_exposure), при
выключении - обратно, до любого снимка стоя.

Метка на 55см/с целиком в кадре ~0,24с: 10 Гц - это 2-3 кадра на метку.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from multiprocessing import shared_memory

import numpy as np

from watch_logic import Stats

WATCH_HZ = 10.0
FAST_EXPOSURE_US = 3000        # смаз на 55см/с ~1,7мм
RESPONSE_TIMEOUT_S = 2.0       # воркер молчит дольше - считаем мёртвым
IDLE_WAIT_S = 0.2
TS_SKEW_MAX_S = 0.5            # SensorTimestamp дальше от monotonic - чужие часы
FRAME_SHAPE = (320, 320, 3)
SIDE_IDX = {"right": 0, "left": 1}   # как cam_idx в victim_detect.py


def attach_shm(name):
    """Открыть общую память, которую создал nav.py, не отдавая её своему
    resource_tracker: иначе при выходе воркера он удалил бы чужой сегмент и
    ругался на "утечку". track=False есть только с Python 3.13, а venv на Pi
    - 3.11: там то же самое делает unregister."""
    if sys.version_info >= (3, 13):
        return shared_memory.SharedMemory(name=name, track=False)
    from multiprocessing import resource_tracker
    shm = shared_memory.SharedMemory(name=name)
    resource_tracker.unregister(shm._name, "shared_memory")   # type: ignore[attr-defined]
    return shm


@dataclass
class WatchBox:
    side: str
    cls: str
    conf: float
    xyxy: tuple
    t_frame: float          # time.monotonic() кадра
    image: str | None       # картинка в папке детекций (сохранил воркер)


class VictimWatcher:
    def __init__(self, cams, model_path, save_dir, conf, script=None):
        self.cams = dict(cams)           # {"right"/"left": Camera}
        self.dead = False
        self.ready = threading.Event()
        self._closing = False
        self._lock = threading.Lock()
        self._armed = {s: False for s in self.cams}
        self._wake = {s: threading.Event() for s in self.cams}
        self._idle = {s: threading.Event() for s in self.cams}
        self._free = {s: threading.Event() for s in self.cams}   # воркер свободен для камеры
        self._pending = {}               # side -> (fid, t_frame, arm_id, t_sent)
        self._out = []                   # (arm_id, t_frame, [WatchBox])
        self._arm_id = 0
        self._any_armed = False
        self._fid = 0
        self._ts_warned = False
        self._failed = set()
        self._frames_n = {s: 0 for s in self.cams}
        self._armed_s = {s: 0.0 for s in self.cams}
        self._armed_t: dict[str, float | None] = {s: None for s in self.cams}
        self.latency = Stats()
        self.infer = Stats()
        self._t_start = time.monotonic()

        nbytes = int(np.prod(FRAME_SHAPE))
        self._shm = {s: shared_memory.SharedMemory(create=True, size=nbytes) for s in self.cams}
        self._buf = {s: np.ndarray(FRAME_SHAPE, np.uint8, buffer=self._shm[s].buf)
                     for s in self.cams}
        script = script or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "watch_proc.py")
        slots = [f"{SIDE_IDX[s]}:{self._shm[s].name}" for s in self.cams]
        self._proc = subprocess.Popen(
            [sys.executable, "-u", script, model_path, save_dir, str(conf), *slots],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        for s in self.cams:
            self._idle[s].set()
            self._free[s].set()
        threading.Thread(target=self._reader, daemon=True).start()
        for s in self.cams:
            threading.Thread(target=self._cam_loop, args=(s,), daemon=True).start()

    # ── управление из петли drive_cells ─────────────────────────
    def arm(self, sides):
        """Какие стороны смотреть (каждый такт). Возвращает, включено ли хоть
        что-то. Переход "ничего" -> "что-то" - новый номер включения: ответы
        воркера на кадры прошлого включения poll() отбросит."""
        if self.dead or not self.ready.is_set():
            sides = set()
        sides = set(sides) & (set(self.cams) - self._failed)
        now = time.monotonic()
        with self._lock:
            any_now = bool(sides)
            if any_now and not self._any_armed:
                self._arm_id += 1
            self._any_armed = any_now
            for s in self.cams:
                on = s in sides
                if on == self._armed[s]:
                    continue
                self._armed[s] = on
                if on:
                    self._armed_t[s] = now
                else:
                    t = self._armed_t[s]
                    if t is not None:
                        self._armed_s[s] += now - t
                    self._armed_t[s] = None
                self._wake[s].set()
        return any_now

    def poll(self):
        """Рамки, пришедшие с прошлого вызова (только текущего включения).
        Здесь же - сторож воркера: процесс умер или молчит дольше
        RESPONSE_TIMEOUT_S - наблюдатель выключается до конца заезда."""
        now = time.monotonic()
        with self._lock:
            out, self._out = self._out, []
            arm_id = self._arm_id
            stale = any(self._armed[s] and now - p[3] > RESPONSE_TIMEOUT_S
                        for s, p in self._pending.items())
        if self._proc.poll() is not None:
            self._die(f"воркер завершился (код {self._proc.returncode})")
        elif stale:
            self._die(f"воркер не ответил за {RESPONSE_TIMEOUT_S:.0f}с")
        boxes = []
        for a, t_frame, bxs in out:
            if a != arm_id:
                continue
            self.latency.add((now - t_frame) * 1000.0)
            boxes.extend(bxs)
        return boxes

    def wait_idle(self, timeout=IDLE_WAIT_S):
        """Ждёт, пока выключенные камеры отдадут кадр и вернут выдержку."""
        end = time.monotonic() + timeout
        for s in self.cams:
            self._idle[s].wait(max(0.0, end - time.monotonic()))

    def take_stats(self):
        """Строка статистики с прошлого вызова или None, если не включался."""
        now = time.monotonic()
        with self._lock:
            parts = []
            for s in self.cams:
                t = self._armed_s[s]
                armed_t = self._armed_t[s]
                if armed_t is not None:
                    t += now - armed_t
                    self._armed_t[s] = now
                if t > 0.05:
                    parts.append(f"{s} {self._frames_n[s] / t:.1f}Гц ({self._frames_n[s]} кадров)")
                self._armed_s[s] = 0.0
                self._frames_n[s] = 0
            lat, inf = self.latency, self.infer
            self.latency, self.infer = Stats(), Stats()
        if not parts:
            return None
        return (f"    [VW] {', '.join(parts)}; кадр→рамки {lat.fmt()} мс; "
                f"инференс {inf.fmt()} мс")

    def close(self):
        self._closing = True
        self.arm(set())
        self.dead = True
        for s in self.cams:
            self._wake[s].set()
        self.wait_idle(1.0)
        try:
            if self._proc.stdin is not None:
                self._proc.stdin.close()
            self._proc.terminate()
            self._proc.wait(timeout=1.0)
        except (OSError, subprocess.TimeoutExpired):
            self._proc.kill()
        if self._proc.stdout is not None:
            self._proc.stdout.close()
        self._buf.clear()               # иначе shm.close(): "exported pointers exist"
        for shm in self._shm.values():
            try:
                shm.close()
                shm.unlink()
            except (OSError, BufferError):
                pass

    # ── потоки ──────────────────────────────────────────────────
    def _die(self, why):
        if self.dead:
            return
        self.dead = True
        with self._lock:
            for s in self.cams:
                self._armed[s] = False
                self._wake[s].set()
        if not self._closing:
            print(f"\n  [WARN] наблюдатель меток: {why} - выключен до конца заезда", flush=True)

    def _fail(self, side, e):
        self._failed.add(side)
        with self._lock:
            self._armed[side] = False
        print(f"\n  [WARN] наблюдатель меток: камера {side} - {e}; сторона выключена",
              flush=True)

    def _frame_time(self, md):
        now = time.monotonic()
        ts = md.get("SensorTimestamp")
        if ts:
            t = ts / 1e9
            if abs(now - t) <= TS_SKEW_MAX_S:
                return t
        if not self._ts_warned:
            self._ts_warned = True
            print(f"\n  [WARN] наблюдатель меток: SensorTimestamp не в часах monotonic "
                  f"({ts}) - время кадра = момент его получения", flush=True)
        return now

    def _reader(self):
        idx_side = {v: k for k, v in SIDE_IDX.items()}
        stdout = self._proc.stdout
        assert stdout is not None
        try:
            for line in stdout:
                self._on_line(line, idx_side)
        except (OSError, ValueError):       # close() закрыл pipe на ходу
            pass
        self._die("воркер закрыл вывод")

    def _on_line(self, line, idx_side):
        try:
            msg = json.loads(line)
        except ValueError:
            return
        if msg.get("ready"):
            print(f"[VW] воркер готов за {time.monotonic() - self._t_start:.1f}с", flush=True)
            self.ready.set()
            return
        side = idx_side.get(msg.get("cam"))
        if side is None or side not in self.cams:
            return
        with self._lock:
            p = self._pending.get(side)
            if p is None or p[0] != msg.get("id"):
                return
            del self._pending[side]
            _, t_frame, arm_id, _ = p
            self.infer.add(float(msg.get("ms", 0.0)))
            boxes = [WatchBox(side, str(b["cls"]), float(b["conf"]),
                              tuple(b["xyxy"]), t_frame, msg.get("img"))
                     for b in msg.get("boxes", [])]
            self._out.append((arm_id, t_frame, boxes))
        self._free[side].set()

    def _cam_loop(self, side):
        cam = self.cams[side]
        fast = False
        log_exposure = None
        next_t = 0.0
        while not self.dead and side not in self._failed:
            if not self._armed[side]:
                if fast:
                    try:
                        cam.restore_exposure()
                    except Exception as e:
                        self._fail(side, e)
                        break
                    fast = False
                self._idle[side].set()
                self._wake[side].wait(0.5)
                self._wake[side].clear()
                continue
            self._idle[side].clear()
            try:
                if not fast:
                    got = cam.fast_exposure(FAST_EXPOSURE_US)
                    if got is None:
                        time.sleep(0.05)
                        continue
                    fast, log_exposure = True, got
                now = time.monotonic()
                if now < next_t:
                    time.sleep(next_t - now)
                    continue
                if not self._free[side].wait(0.1):
                    continue
                frame, md = cam.try_latest()
            except Exception as e:
                self._fail(side, e)
                break
            if frame is None:
                time.sleep(0.02)
                continue
            next_t = time.monotonic() + 1.0 / WATCH_HZ
            t_frame = self._frame_time(md)
            if log_exposure is not None:
                exp, gain = log_exposure
                log_exposure = None
                print(f"\n    [VW] {side}: выдержка {exp}мкс x{gain:.1f}, "
                      f"яркость {float(frame.mean()):.0f}", flush=True)
            if not self._armed[side] or self.dead:
                continue
            # Копия - вне замка: слот этой камеры свободен (_free), а замок
            # каждый такт берут arm()/poll() из петли drive_cells.
            self._buf[side][...] = frame
            with self._lock:
                if not self._armed[side] or self.dead:
                    continue
                self._fid += 1
                fid, arm_id = self._fid, self._arm_id
                self._pending[side] = (fid, t_frame, arm_id, time.monotonic())
                self._free[side].clear()
                self._frames_n[side] += 1
            try:
                stdin = self._proc.stdin
                assert stdin is not None
                stdin.write(f"{SIDE_IDX[side]} {fid} {arm_id}\n")
                stdin.flush()
            except (OSError, ValueError):
                self._die("pipe воркера закрыт")
                break
        if fast:
            try:
                cam.restore_exposure()
            except Exception:
                pass
        self._idle[side].set()
