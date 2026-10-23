"""Метки на передней стене - передней USB-камерой, стоя (для nav.py).

Тот же VictimDetector, что у боковых камер (victim_detect.py: модель,
пороги, кольца, снимок в ~/maze/detections), только камера другая:
USB-камера в носу. Кадр повёрнут и без бампера (front_floor.prep) и
дополнен до квадрата чёрными полями: детектор ждёт квадратный кадр и
масштабирует рамку по ширине. Снимка с длинной выдержкой у неё нет -
метка читается по обычному кадру.

Камера открыта весь заезд. Фоновый поток всё время забирает кадры из
буфера V4L (без декодирования - это дёшево), поэтому latest() всегда
отдаёт свежий. Раз в PREVIEW_S он же декодирует кадр, уменьшает и пишет в
PREVIEW_PATH - его показывает страница карты (map_web.py, :8001).
"""
from __future__ import annotations

import atexit
import math
import os
import threading
import time

import cv2
import numpy as np

import front_floor as ff
import ring_colors as rc
import victim_detect as vd

CAM_DEV = '/dev/v4l/by-id/usb-SYX-240327-J_HD_Camera-video-index0'
# 1920x1080: обзор тот же, что у 1280x720 и 640x480 (проверено 19.10 -
# камера во всех режимах видит одно и то же), а пикселей на метку в 1.5
# раза больше - кольца чётче.
CAM_SIZE = (1920, 1080)
FLUSH_FRAMES = 2       # буфер держит свежим фоновый поток - хватает двух
PREVIEW_PATH = '/dev/shm/maze_front.jpg'   # в памяти, не на SD; читает map_web.py
PREVIEW_S = 0.12       # как часто обновлять картинку для страницы (~8 кадр/с)
PREVIEW_W = 540        # её ширина, px (страница карты и окно /front)
EDGE_PX = 4            # рамка ближе этого к краю кадра - метка обрезана


def square(frame):
    """Кадр -> квадрат: чёрные поля по бокам (кадр стоит вертикально)."""
    h, w = frame.shape[:2]
    if h == w:
        return frame
    n = max(h, w)
    out = np.zeros((n, n, 3), frame.dtype)
    y0, x0 = (n - h) // 2, (n - w) // 2
    out[y0:y0 + h, x0:x0 + w] = frame
    return out


class FrontCamera:
    """latest() и capture_long() - как у dataset_web.Camera, которую ждёт
    VictimDetector."""

    def __init__(self, dev=CAM_DEV, size=CAM_SIZE):
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise OSError(f'камера {dev} не открылась')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = size[1]        # ширина кадра после prep() (камера стоит боком)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._floor_cal = ff.load()
        if self.latest() is None:
            raise OSError(f'камера {dev} не отдаёт кадры')
        self._thread = threading.Thread(target=self._run, daemon=True, name='front-cam')
        self._thread.start()
        # Поток, застигнутый внутри OpenCV при выходе интерпретатора, роняет
        # процесс ("terminate called without an active exception") - гасим его
        # раньше.
        atexit.register(self.close)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        with self.lock:
            self.cap.release()

    def _run(self):
        """Забирать кадры из буфера, раз в PREVIEW_S - картинка для страницы."""
        t_prev = 0.0
        while not self._stop.is_set():
            with self.lock:
                ok = self.cap.grab()
                f = None
                if ok and time.monotonic() - t_prev >= PREVIEW_S:
                    t_prev = time.monotonic()
                    ok, f = self.cap.retrieve()
            if not ok:
                time.sleep(0.1)
            elif f is not None:
                self._preview(ff.prep(f))

    def _preview(self, frame):
        try:
            # Share the camera already open for victim recognition.
            import state
            import config
            import floor_pose
            with state.lock:
                st = dict(state.telem)
            snap = {"t": state.telem_t, "pose": dict(state.live), "drive": state.drive_live,
                    "imu": {"yaw": (st["yaw"] - state.YAW_ORIGIN) % 360,
                            "pitch": st["pitch"] - state.pitch_level}, "walls": st["wall"]}
            pose = floor_pose.from_snapshot(snap, time.monotonic())
            if pose is not None:
                small = cv2.resize(frame, (ff.CAM_W, round(frame.shape[0] * ff.CAM_W / frame.shape[1])))
                result = ff.read_cell(small, self._floor_cal, pose[0], tilt_deg=pose[1])
                state.front_floor_live = dict(result, t=time.monotonic(), pose=list(pose[0]))
            h = int(frame.shape[0] * PREVIEW_W / frame.shape[1])
            ok, buf = cv2.imencode('.jpg', cv2.resize(frame, (PREVIEW_W, h),
                                                      interpolation=cv2.INTER_AREA),
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                tmp = PREVIEW_PATH + '.tmp'
                with open(tmp, 'wb') as fh:
                    fh.write(buf.tobytes())
                os.replace(tmp, PREVIEW_PATH)    # страница не увидит полфайла
        except OSError:
            pass                                 # картинка для страницы - не повод падать

    def latest(self):
        with self.lock:
            for _ in range(FLUSH_FRAMES):
                self.cap.grab()
            ok, f = self.cap.retrieve()
        return square(ff.prep(f)) if ok else None

    def capture_long(self, *args, **kwargs):
        return None, {}             # длинной выдержки нет - читаем обычный кадр


class FrontVictimDetector(vd.VictimDetector):
    def __init__(self):
        from ultralytics import YOLO

        os.makedirs(vd.SAVE_DIR, exist_ok=True)
        self.cam_idx = 'front'
        print('[VD] Загрузка YOLO (передняя)...')
        self.model = YOLO(vd.YOLO_MODEL_PATH)
        print('[VD] Инициализация камеры (перёд, USB)...')
        self.cam = FrontCamera()
        self.ring_cfg = rc.load_config()
        self.scanned = set()
        self.log = []
        self.last_unread = None
        self.unread_hist = []
        self.last_seen = None
        # прогрев: первый predict ~1.6с дольше остальных - пусть он будет
        # здесь, до кнопки, а не у первой стены
        self._best(self.cam.latest(), vd.CONF_FAST)
        print('[VD] Готов (перёд)')

    def locate(self):
        """На сколько градусов вправо (+) от оси камеры центр метки на свежем
        кадре, или None - метки нет. Метка, обрезанная краем кадра: её рамка
        сдвинута внутрь, центр берём по высоте рамки (метка круглая)."""
        frame = self.cam.latest()
        found = None if frame is None else self._best(frame, vd.CONF_FAST)
        if found is None:
            return None
        x1, y1, x2, y2 = found[2]
        w = self.cam.width
        x0 = (frame.shape[1] - w) / 2.0                 # слева чёрное поле square()
        x1, x2 = x1 - x0, x2 - x0
        h = y2 - y1
        if x2 >= w - EDGE_PX and x2 - x1 < h:
            u = x1 + h / 2.0                            # обрезана справа
        elif x1 <= EDGE_PX and x2 - x1 < h:
            u = x2 - h / 2.0                            # обрезана слева
        else:
            u = (x1 + x2) / 2.0
        c, k = ff.DEFAULT_CAM, w / ff.CAM_W             # модель камеры - для ширины 720
        return math.degrees(math.atan((u - c["cx"] * k) / (c["f_px"] * k)))
