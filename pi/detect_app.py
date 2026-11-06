#!/usr/bin/env python3
"""Веб-страница live-детекции жертв.

Слева — сырое видео с камеры, справа — тот же кадр с обведённой меткой
и подписанным типом (damaged/stable/unharmed), плюс текстовая панель
со статусом. Работает без езды, робот может стоять на месте.
"""
import os
import sys
import time
import threading
from threading import Condition

from flask import Flask, Response, jsonify, render_template

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from victim_detect import (
    YOLO_MODEL_PATH, CLASS_TO_VICTIM, CLASS_LABEL, _process_cognitive, cv2,
)

FRAME_SIZE = (320, 320)
HTTP_PORT = 8000
DETECT_CONF = 0.4


class FrameStore:
    def __init__(self):
        self.condition = Condition()
        self.raw_jpg = None
        self.annotated_jpg = None
        self.status = {"type": None, "label": None, "class": None,
                        "conf": None, "visible": False, "ts": None}

    def update(self, raw_jpg, annotated_jpg, status):
        with self.condition:
            self.raw_jpg = raw_jpg
            self.annotated_jpg = annotated_jpg
            self.status = status
            self.condition.notify_all()


store = FrameStore()


def detection_loop():
    from ultralytics import YOLO
    from picamera2 import Picamera2

    n_cams = len(Picamera2.global_camera_info())
    # По контрольным кадрам 21.09.2026: idx=0 смотрит вправо.
    cam_idx = 0
    print(f'[APP] Камер обнаружено: {n_cams}, использую idx={cam_idx}')

    picam2 = Picamera2(cam_idx)
    cfg = picam2.create_preview_configuration(main={"size": FRAME_SIZE, "format": "RGB888"})
    picam2.configure(cfg)
    picam2.start()
    try:
        from libcamera import controls as _lcc
        picam2.set_controls({"AeExposureMode": _lcc.AeExposureModeEnum.Short})
    except Exception:
        pass
    time.sleep(0.8)

    print('[APP] Загрузка YOLO...')
    model = YOLO(YOLO_MODEL_PATH)
    print('[APP] Готово, запускаю цикл детекции')

    last_status = {"type": None, "label": None, "class": None,
                    "conf": None, "visible": False, "ts": None}

    while True:
        frame = picam2.capture_array()
        ok, raw_jpg = cv2.imencode('.jpg', frame)
        raw_bytes = raw_jpg.tobytes() if ok else None

        annotated = frame.copy()
        results = model.predict(source=frame, imgsz=FRAME_SIZE[0], conf=DETECT_CONF,
                                 save=False, verbose=False)
        boxes = results[0].boxes

        if len(boxes) > 0:
            best = max(boxes, key=lambda b: float(b.conf[0]))
            cls_id = int(best.cls[0])
            cls_name = str(model.names[cls_id])
            conf = float(best.conf[0])

            victim_type = None
            if cls_name == 'shneyne':
                victim_type, _chk, _rings = _process_cognitive(annotated, best)
            elif cls_name in CLASS_TO_VICTIM:
                victim_type = CLASS_TO_VICTIM[cls_name]

            label = CLASS_LABEL.get(victim_type, cls_name)
            color = (0, 255, 0) if victim_type else (0, 165, 255)

            # cv2.putText не умеет кириллицу/греческие буквы (только ASCII),
            # поэтому на кадре пишем victim_type латиницей, а не label.
            x1, y1, x2, y2 = [int(v) for v in best.xyxy[0].tolist()]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            ascii_tag = f'{cls_name} -> {victim_type or "?"} ({conf:.2f})'
            cv2.putText(annotated, ascii_tag,
                        (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            last_status = {
                "type": victim_type, "label": label, "class": cls_name,
                "conf": round(conf, 3), "visible": True,
                "ts": time.strftime("%H:%M:%S"),
            }
        else:
            cv2.putText(annotated, 'no label', (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            last_status = {**last_status, "visible": False, "ts": time.strftime("%H:%M:%S")}

        ok2, ann_jpg = cv2.imencode('.jpg', annotated)
        annotated_bytes = ann_jpg.tobytes() if ok2 else raw_bytes

        store.update(raw_bytes, annotated_bytes, last_status)


app = Flask(__name__, template_folder='templates_detect')


def mjpeg_generator(getter):
    while True:
        with store.condition:
            store.condition.wait()
            data = getter(store)
        if data:
            yield (b"--FRAME\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + data + b"\r\n")


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/stream/raw.mjpg')
def stream_raw():
    return Response(mjpeg_generator(lambda s: s.raw_jpg),
                     mimetype='multipart/x-mixed-replace; boundary=FRAME')


@app.route('/stream/detected.mjpg')
def stream_detected():
    return Response(mjpeg_generator(lambda s: s.annotated_jpg),
                     mimetype='multipart/x-mixed-replace; boundary=FRAME')


@app.route('/status.json')
def status():
    return jsonify(store.status)


if __name__ == '__main__':
    threading.Thread(target=detection_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
