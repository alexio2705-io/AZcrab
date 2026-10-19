"""Метки на передней стене - передней USB-камерой, стоя (для nav.py).

Тот же VictimDetector, что у боковых камер (victim_detect.py: модель,
пороги, кольца, снимок в ~/maze/detections), только камера другая:
USB-камера в носу. Кадр повёрнут и без бампера (front_floor.prep) и
дополнен до квадрата чёрными полями: детектор ждёт квадратный кадр и
масштабирует рамку по ширине. Снимка с длинной выдержкой у неё нет -
метка читается по обычному кадру.

Камера открыта весь заезд, но кадры не читаются, пока их не попросят:
latest() пропускает накопившиеся в буфере и отдаёт свежий. Фоновой
декодировки MJPG нет - процессор занят, только когда стена впереди и её
читают (см. _scan_front в nav.py).
"""
from __future__ import annotations

import math
import os

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
FLUSH_FRAMES = 4       # столько кадров в буфере V4L может быть старыми
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
        if self.latest() is None:
            raise OSError(f'камера {dev} не отдаёт кадры')

    def latest(self):
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
