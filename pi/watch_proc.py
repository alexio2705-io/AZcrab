#!/usr/bin/env python3
"""YOLO наблюдателя меток на ходу - отдельным процессом (см. victim_watch.py).

Свой GIL: петля drive_cells в nav.py (200 Гц) и поток телеметрии не делят
интерпретатор с инференсом. Раньше после YOLO в том же процессе лидар отдавал
пустой перед - именно из-за этого. Один поток torch и nice +5: инференс
занимает одно ядро из четырёх.

МОДЕЛЬ - .pt (PyTorch) или папка экспорта NCNN (*_ncnn_model, см.
watch_logic.prefer_ncnn). Замер на Pi 5 (YOLO26n, 320): NCNN в 1 поток - 28 мс
на кадр, в 4 - 15; PyTorch в 1 поток - ~96 мс, в 3 - 80. Привязка к ядрам
(sched_setaffinity) torch здесь только тормозит. Потоки PyTorch ставятся
ПОСЛЕ первого predict (ultralytics в select_device() сам выставляет
cpu_count-1 = 3), а NCNN - до загрузки: менять opt.num_threads у загруженной
сети поздно, свёртки остаются на значении при загрузке.

Запуск (это делает victim_watch.py):
    watch_proc.py МОДЕЛЬ ПАПКА CONF idx:shm [idx:shm]
stdin:  "idx номер_кадра номер_включения" - кадр 320x320 BGR уже лежит в
        общей памяти камеры idx (0 - правая, 1 - левая, как cam_idx);
stdout: {"ready": true} после прогрева, дальше по строке JSON на запрос:
        {"cam", "id", "arm", "boxes": [{"cls", "conf", "xyxy"}], "ms", "img"}.
Каждый кадр с рамкой сохраняется в ПАПКУ с нарисованными рамками - по ним
подбирают порог (каждая лишняя остановка стоит ~2с).
"""
import json
import os
import sys
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from victim_watch import attach_shm  # noqa: E402

IMGSZ = 320
FRAME_SHAPE = (IMGSZ, IMGSZ, 3)
TORCH_THREADS = 1
NCNN_THREADS = 1
NICE = 5
CAM_TAG = {0: 'r', 1: 'l'}


def save(frame, boxes, save_dir, idx):
    """Кадр с рамками в папку детекций; имя - время, камера, лучший класс."""
    shown = frame.copy()
    for b in boxes:
        x1, y1, x2, y2 = (int(v) for v in b['xyxy'])
        cv2.rectangle(shown, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.putText(shown, f"{b['cls']} {b['conf']:.2f}", (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    best = max(boxes, key=lambda b: b['conf'])
    t = time.time()
    name = (f"w_{time.strftime('%H-%M-%S', time.localtime(t))}.{int(t * 1000) % 1000:03d}"
            f"_{CAM_TAG.get(idx, idx)}_{best['cls']}_{best['conf']:.2f}.jpg")
    cv2.imwrite(os.path.join(save_dir, name), shown)
    return name


def main():
    model_path, save_dir, conf = sys.argv[1], sys.argv[2], float(sys.argv[3])
    slots = {}
    for arg in sys.argv[4:]:
        idx, name = arg.split(':', 1)
        slots[int(idx)] = attach_shm(name)     # создал и освободит nav.py
    frames = {i: np.ndarray(FRAME_SHAPE, np.uint8, buffer=s.buf) for i, s in slots.items()}

    os.nice(NICE)
    is_ncnn = model_path.rstrip('/').endswith('_ncnn_model')
    if is_ncnn:
        import ncnn

        class _Net(ncnn.Net):
            """ultralytics создаёт ncnn.Net() сам - число потоков задаём ему
            здесь, до load_param (см. шапку)."""
            def __init__(self, *a, **kw):
                super().__init__(*a, **kw)
                self.opt.num_threads = NCNN_THREADS

        ncnn.Net = _Net
    import torch
    from ultralytics import YOLO
    model = YOLO(model_path, task='detect')
    os.makedirs(save_dir, exist_ok=True)
    model.predict(np.zeros(FRAME_SHAPE, np.uint8), imgsz=IMGSZ, conf=conf, verbose=False)
    if not is_ncnn:
        torch.set_num_threads(TORCH_THREADS)    # после predict - см. шапку
    print(f'[VW] модель {os.path.basename(model_path.rstrip("/"))}', file=sys.stderr, flush=True)
    print(json.dumps({'ready': True}), flush=True)

    for line in sys.stdin:
        try:
            idx, fid, arm = (int(v) for v in line.split())
        except ValueError:
            continue
        if idx not in frames:
            continue
        frame = frames[idx].copy()
        t0 = time.monotonic()
        res = model.predict(frame, imgsz=IMGSZ, conf=conf, verbose=False)[0]
        ms = (time.monotonic() - t0) * 1000.0
        boxes = [{'cls': str(model.names[int(b.cls[0])]),
                  'conf': round(float(b.conf[0]), 3),
                  'xyxy': [round(float(v), 1) for v in b.xyxy[0].tolist()]}
                 for b in res.boxes]
        img = save(frame, boxes, save_dir, idx) if boxes else None
        print(json.dumps({'cam': idx, 'id': fid, 'arm': arm, 'boxes': boxes,
                          'ms': round(ms, 1), 'img': img}), flush=True)


if __name__ == '__main__':
    main()
