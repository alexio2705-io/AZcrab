#!/usr/bin/env python3
"""Сбор датасета для дообучения YOLO с двух камер робота.

Веб-страница: сначала вводишь имя папки (создаётся в ~/maze/datasets/),
дальше видишь обе камеры и жмёшь «Сохранить» под нужной. Справа — лог.
Горячие клавиши: 1 — сохранить с камеры 0, 2 — с камеры 1.
Ползунки внизу страницы меняют цветокоррекцию обеих камер на лету и
сохраняют её в ~/maze/camera_settings.json (подхватывается при запуске).

Кадры снимаются в тех же настройках, что и в victim_detect.py
(320x320, RGB888 → BGR-массив, AE Short), чтобы обучающие картинки
совпадали с тем, что модель видит в бою.
"""
import json
import os
import re
import time
import threading
from collections import deque
from datetime import datetime
from threading import Condition, Lock

import cv2
from flask import Flask, Response, jsonify, request

FRAME_SIZE = (320, 320)
# Снимок для чтения метки: тот же квадрат обзора, но втрое детальнее. Режим
# сенсора 1296x972 — полный угол с биннингом 2x2 (меньше шума, чем 2592x1944).
STILL_SIZE = (960, 960)
STILL_SENSOR = (1296, 972)
HTTP_PORT = 8000
DATASETS_ROOT = os.path.expanduser('~/maze/datasets')
CAM_NAMES = {0: 'правая', 1: 'левая'}
STREAM_JPEG_QUALITY = 70
SAVE_JPEG_QUALITY = 95
SETTINGS_PATH = os.path.expanduser('~/maze/camera_settings.json')
# Имя контрола libcamera -> (по умолчанию, мин, макс, шаг). AwbMode — список.
SETTINGS_SPEC = {
    'Saturation':    (1.0, 0.0, 4.0, 0.05),
    'Contrast':      (1.0, 0.0, 4.0, 0.05),
    'Brightness':    (0.0, -1.0, 1.0, 0.02),
    'ExposureValue': (0.0, -4.0, 4.0, 0.1),
    'Sharpness':     (1.0, 0.0, 16.0, 0.1),
    # 0 = автоэкспозиция. >0 — ручная выдержка в мкс, тогда действует AnalogueGain.
    'ExposureTime':  (0, 0, 1000000, 1000),
    'AnalogueGain':  (2.0, 1.0, 16.0, 0.1),
}
# Режим автоэкспозиции; Short — как в victim_detect.py (меньше смаза).
AE_MODES = ['Normal', 'Short', 'Long']
AWB_MODES = ['Auto', 'Incandescent', 'Tungsten', 'Fluorescent', 'Indoor', 'Daylight', 'Cloudy']


def load_settings():
    s = {k: v[0] for k, v in SETTINGS_SPEC.items()}
    s['AwbMode'] = 0
    s['AeExposureMode'] = 1
    try:
        with open(SETTINGS_PATH) as f:
            s.update({k: v for k, v in json.load(f).items() if k in s})
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f'Не прочитал {SETTINGS_PATH}: {e}', flush=True)
    return s


cam_settings = load_settings()

# ---------------------------------------------------------------- лог

_log = deque(maxlen=500)
_log_seq = 0
_log_lock = Lock()


def log(msg):
    global _log_seq
    line = f'{datetime.now():%H:%M:%S} {msg}'
    print(line, flush=True)
    with _log_lock:
        _log_seq += 1
        _log.append((_log_seq, line))


# ---------------------------------------------------------------- камеры

class Camera:
    """stream=True — фоновый поток постоянно снимает (для веб-страниц);
    stream=False — камера только открыта, latest() снимает кадр по запросу
    (для nav.py: не грузить процессор, пока робот едет)."""

    def __init__(self, idx, stream=True):
        self.idx = idx
        self.stream = stream
        self.cond = Condition()
        self.frame = None      # последний BGR-кадр (numpy)
        self.jpg = None        # он же в JPEG для стрима
        self.ok = False
        self.cam = None
        self.default_fdl = None   # FrameDurationLimits из конфигурации по умолчанию
        self.cap_lock = Lock()    # на время capture_long поток _run не снимает
        self.preview_cfg = None
        self.scaler_crop = None   # область сенсора, которую видит обычный кадр
        if stream:
            threading.Thread(target=self._run, daemon=True).start()
        elif not self._open():
            raise RuntimeError(f'камера {idx} недоступна')

    def _open(self):
        from picamera2 import Picamera2
        try:
            cam = Picamera2(self.idx)
            cfg = cam.create_preview_configuration(
                main={"size": FRAME_SIZE, "format": "RGB888"})
            cam.configure(cfg)
            self.preview_cfg = cfg
            self.default_fdl = cfg['controls'].get('FrameDurationLimits')
            cam.start()
            self.cam = cam
            self.apply(cam_settings)
            self.scaler_crop = cam.capture_metadata().get('ScalerCrop')
        except Exception as e:
            log(f'Камера {self.idx} ({CAM_NAMES[self.idx]}) недоступна: {e}')
            return False
        self.ok = True
        log(f'Камера {self.idx} ({CAM_NAMES[self.idx]}) запущена')
        return True

    def _run(self):
        if not self._open():
            return
        cam = self.cam
        while True:
            with self.cap_lock:
                frame = cam.capture_array()
            ok, jpg = cv2.imencode('.jpg', frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
            with self.cond:
                self.frame = frame
                if ok:
                    self.jpg = jpg.tobytes()
                self.cond.notify_all()

    def apply(self, settings):
        if self.cam is None:
            return
        ctrl = {k: v for k, v in settings.items()
                if k not in ('ExposureTime', 'AnalogueGain')}
        ctrl['AwbEnable'] = True
        exp = int(settings['ExposureTime'])
        if exp > 0:
            # Ручная выдержка: выключаем AE и раздвигаем длительность кадра,
            # иначе выдержка упрётся в ~33 мс (30 fps).
            ctrl.update(AeEnable=False, ExposureTime=exp,
                        AnalogueGain=float(settings['AnalogueGain']),
                        FrameDurationLimits=(100, max(33333, exp + 1000)))
        else:
            # Автоэкспозиция: возвращаем штатный предел длительности кадра,
            # как в victim_detect.py — не режем выдержку сверху.
            ctrl['AeEnable'] = True
            if self.default_fdl:
                ctrl['FrameDurationLimits'] = self.default_fdl
        try:
            self.cam.set_controls(ctrl)
        except Exception as e:
            log(f'Камера {self.idx}: настройки не применились: {e}')

    def auto_exposure(self, max_exposure_us):
        """Выдержка/усиление для снимка по текущей автоэкспозиции: та же яркость,
        что выбрала AE для обычного кадра, но за счёт выдержки при усилении 1
        (меньше шума). Выдержка не длиннее max_exposure_us — остаток добирается
        усилением. С подсветкой получится короткая выдержка, в темноте — длинная."""
        md = self.cam.capture_metadata()
        total = (md.get('ExposureTime', 10000) * md.get('AnalogueGain', 1.0)
                 * md.get('DigitalGain', 1.0))
        exp = int(min(max_exposure_us, max(1000, total)))
        return exp, max(1.0, total / exp)

    def fast_exposure(self, max_us):
        """Кадры на ходу: выдержка не длиннее max_us (смаз), остаток яркости,
        выбранной автоэкспозицией, - усилением. Наоборот к auto_exposure():
        там шум важнее смаза. Сумма - с текущего кадра; зовётся из потока
        наблюдателя, не из петли управления (capture_metadata ждёт кадр).
        Возвращает (выдержка, усиление) или None - камера занята снимком."""
        if self.cam is None or not self.cap_lock.acquire(timeout=1.0):
            return None
        try:
            md = self.cam.capture_metadata()
            total = (md.get('ExposureTime', 10000) * md.get('AnalogueGain', 1.0)
                     * md.get('DigitalGain', 1.0))
            exp = int(max(100, min(max_us, total)))
            gain = max(1.0, total / exp)
            self.cam.set_controls({'AeEnable': False, 'ExposureTime': exp,
                                   'AnalogueGain': float(gain)})
            return exp, gain
        finally:
            self.cap_lock.release()

    def restore_exposure(self):
        """Назад к camera_settings.json после fast_exposure()."""
        self.apply(cam_settings)

    def capture_long(self, exposure_us, gain, size=None, timeout=6.0, settle_async=False,
                     auto_max_us=None):
        """Один кадр с ручной выдержкой (и, если задан size, в большом разрешении).

        auto_max_us — вместо exposure_us/gain взять их из текущей автоэкспозиции
        (см. auto_exposure), с этим потолком выдержки.

        Для большого разрешения камера на время перенастраивается в режим сенсора
        STILL_SENSOR, причём с той же ScalerCrop, что у обычного кадра, — чтобы
        снимок показывал ровно ту же область, только детальнее. Ждём кадр, у
        которого в метаданных уже стоит нужная выдержка. Потом всё возвращаем.
        Возвращает (кадр или None, метаданные).

        settle_async=True — возврат в обычный режим (и сброс первых кадров, пока
        сходятся AE/AWB) идёт в фоновом потоке, а кадр возвращается сразу.
        cap_lock держится до конца возврата, так что latest() просто подождёт.
        """
        if self.cam is None:
            return None, {}
        frame, md = None, {}
        self.cap_lock.acquire()
        switched = False
        try:
            if auto_max_us:
                exposure_us, gain = self.auto_exposure(auto_max_us)
            ctrl = {'AeEnable': False, 'ExposureTime': int(exposure_us),
                    'AnalogueGain': float(gain),
                    'FrameDurationLimits': (100, int(exposure_us) + 20000)}
            if size:
                self.cam.stop()
                switched = True
                still = self.cam.create_still_configuration(
                    main={'size': tuple(size), 'format': 'RGB888'},
                    sensor={'output_size': STILL_SENSOR}, buffer_count=2)
                self.cam.configure(still)
                extra = {'NoiseReductionMode': 2}          # HighQuality
                if self.scaler_crop:
                    extra['ScalerCrop'] = tuple(self.scaler_crop)
                self.cam.set_controls({**cam_settings_controls(), **ctrl, **extra})
                self.cam.start()
            else:
                self.cam.set_controls(ctrl)
            t0 = time.time()
            while time.time() - t0 < timeout:
                req = self.cam.capture_request()
                try:
                    md = req.get_metadata()
                    if abs(md.get('ExposureTime', 0) - exposure_us) <= 0.1 * exposure_us:
                        frame = req.make_array('main')
                        break
                finally:
                    req.release()
        except Exception as e:
            log(f'Камера {self.idx}: снимок с длинной выдержкой не удался: {e}')
        finally:
            if settle_async:
                threading.Thread(target=self._restore_preview, args=(switched,),
                                 daemon=True).start()
            else:
                self._restore_preview(switched)
        return frame, md

    def _restore_preview(self, switched):
        """Возврат из снимка в обычный режим; отпускает cap_lock, взятый в
        capture_long()."""
        try:
            if switched:
                self.cam.stop()
                self.cam.configure(self.preview_cfg)
                self.cam.start()
            self.apply(cam_settings)
            # После перенастройки AE/AWB сходятся заново — первые кадры
            # тёмные/цветные. Выбрасываем их, пока держим камеру.
            for _ in range(8):
                self.cam.capture_array()
        except Exception as e:
            log(f'Камера {self.idx}: не вернулась в обычный режим: {e}')
        finally:
            self.cap_lock.release()

    def close(self):
        if self.cam is not None:
            with self.cap_lock:
                self.cam.stop()
                self.cam.close()

    def latest(self):
        if not self.stream:
            with self.cap_lock:
                return self.cam.capture_array()
        with self.cond:
            return None if self.frame is None else self.frame.copy()

    def try_latest(self):
        """(кадр, метаданные) без ожидания cap_lock: камера занята снимком
        или его фоновым возвратом (settle_async) - (None, {}), наблюдатель
        просто пропустит кадр. Метаданные нужны ради SensorTimestamp."""
        if self.cam is None or not self.cap_lock.acquire(blocking=False):
            return None, {}
        try:
            req = self.cam.capture_request()
            try:
                return req.make_array('main'), req.get_metadata()
            finally:
                req.release()
        finally:
            self.cap_lock.release()


def cam_settings_controls():
    """Цветокоррекция из cam_settings (без экспозиции) — для снимка в другом режиме."""
    skip = ('ExposureTime', 'AnalogueGain', 'AeExposureMode')
    return {k: v for k, v in cam_settings.items() if k not in skip}


# ---------------------------------------------------------------- состояние

state_lock = Lock()
session_dir = None


def count_images(path):
    return sum(1 for f in os.listdir(path) if f.lower().endswith('.jpg'))


app = Flask(__name__)
cameras = {}


@app.route('/')
def index():
    return PAGE


@app.route('/stream/<int:cam_id>.mjpg')
def stream(cam_id):
    cam = cameras.get(cam_id)
    if cam is None:
        return 'нет камеры', 404

    def gen():
        while True:
            with cam.cond:
                cam.cond.wait(timeout=2)
                jpg = cam.jpg
            if jpg is None:
                continue
            yield (b'--FRAME\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=FRAME')


@app.route('/api/session', methods=['GET', 'POST'])
def api_session():
    global session_dir
    if request.method == 'POST':
        name = (request.get_json(silent=True) or {}).get('name', '').strip()
        if not re.fullmatch(r'[A-Za-z0-9_\-]{1,64}', name):
            return jsonify(error='Имя: латиница, цифры, _ и -, до 64 символов'), 400
        path = os.path.join(DATASETS_ROOT, name)
        existed = os.path.isdir(path)
        os.makedirs(path, exist_ok=True)
        with state_lock:
            session_dir = path
        n = count_images(path)
        if existed:
            log(f'Продолжаю в существующей папке {path} (уже {n} кадров)')
        else:
            log(f'Создана папка {path}')
    with state_lock:
        path = session_dir
    if path is None:
        return jsonify(folder=None, count=0,
                       existing=sorted(os.listdir(DATASETS_ROOT)))
    return jsonify(folder=os.path.basename(path), path=path, count=count_images(path))


@app.route('/api/save/<int:cam_id>', methods=['POST'])
def api_save(cam_id):
    with state_lock:
        path = session_dir
    if path is None:
        return jsonify(error='Сначала создай папку'), 400
    cam = cameras.get(cam_id)
    frame = cam.latest() if cam else None
    if frame is None:
        log(f'Камера {cam_id}: кадра нет, не сохранено')
        return jsonify(error=f'Камера {cam_id} не даёт кадр'), 503
    folder = os.path.basename(path)
    with state_lock:
        # Номер = максимальный существующий + 1, чтобы не затереть кадр,
        # даже если какие-то файлы из середины удалили.
        pat = re.compile(rf'{re.escape(folder)}_(\d+)\.jpg$')
        nums = [int(m.group(1)) for f in os.listdir(path) if (m := pat.match(f))]
        fname = f'{folder}_{max(nums, default=0) + 1:04d}.jpg'
        full = os.path.join(path, fname)
        if not cv2.imwrite(full, frame, [cv2.IMWRITE_JPEG_QUALITY, SAVE_JPEG_QUALITY]):
            log(f'Ошибка записи {full}')
            return jsonify(error='Не удалось записать файл'), 500
    n = count_images(path)
    log(f'cam{cam_id} → {fname}  (всего {n})')
    return jsonify(file=fname, count=n)


@app.route('/api/camera_settings', methods=['GET', 'POST'])
def api_camera_settings():
    if request.method == 'POST':
        data = request.get_json(silent=True) or {}
        changed = []
        for k, v in data.items():
            if k in SETTINGS_SPEC:
                _, lo, hi, _ = SETTINGS_SPEC[k]
                cam_settings[k] = min(hi, max(lo, float(v)))
            elif k == 'AwbMode':
                cam_settings[k] = min(len(AWB_MODES) - 1, max(0, int(v)))
            elif k == 'AeExposureMode':
                cam_settings[k] = min(len(AE_MODES) - 1, max(0, int(v)))
            else:
                continue
            changed.append(f'{k}={cam_settings[k]}')
        for cam in cameras.values():
            cam.apply(cam_settings)
        with open(SETTINGS_PATH, 'w') as f:
            json.dump(cam_settings, f, indent=2)
        if changed:
            log('Настройки: ' + ', '.join(changed))
    return jsonify(values=cam_settings, spec=SETTINGS_SPEC, awb_modes=AWB_MODES,
                   ae_modes=AE_MODES)


@app.route('/api/logs')
def api_logs():
    since = request.args.get('since', 0, type=int)
    with _log_lock:
        lines = [(s, l) for s, l in _log if s > since]
    return jsonify(lines=[l for _, l in lines], last=lines[-1][0] if lines else since)


PAGE = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Сбор датасета - MAZE2027</title>
<style>
  /* Системный моноширинный стек: страницы отдаёт Пи без интернета, веб-шрифт не загрузится. */
  :root {
    color-scheme:dark;
    --bg:#14161A; --panel:#1B1E23; --fg:#D7DADF; --mut:#7E838C;
    --line:rgba(255,255,255,.08); --line2:rgba(255,255,255,.28); --hov:rgba(255,255,255,.04);
    --acc:#F2A900;
    --damaged:#E5534B; --stable:#C7D84B; --unharmed:#3FB27F;
    --mono:ui-monospace,"SF Mono",Menlo,Consolas,"Liberation Mono",monospace;
  }
  * { box-sizing:border-box; }
  html { background:var(--bg); }
  body { margin:0; background:var(--bg); color:var(--fg); font:13px/20px var(--mono);
         font-variant-numeric:tabular-nums; -webkit-font-smoothing:antialiased; -webkit-text-size-adjust:100%; }
  .wrap { max-width:1120px; margin:0 auto; padding:16px; }
  @media (min-width:900px) { .wrap { padding:24px; } }
  b { font-weight:600; }
  .mut { color:var(--mut); }
  .bad { color:var(--acc); }

  .bar { display:flex; align-items:center; justify-content:space-between; gap:8px 16px; flex-wrap:wrap;
         padding:0 0 12px; border-bottom:1px solid var(--line); margin-bottom:16px; }
  .bar h1 { margin:0; font:600 13px/20px var(--mono); letter-spacing:.1em; text-transform:uppercase; white-space:nowrap; }
  .bar h1 span { color:var(--mut); font-weight:400; }
  .nav { display:flex; gap:16px; font-size:12px; letter-spacing:.06em; }
  .nav a { color:var(--mut); text-decoration:none; }
  .nav a:hover, .nav a[aria-current] { color:var(--fg); }
  .st { display:flex; align-items:center; gap:8px; color:var(--mut); font-size:12px; letter-spacing:.06em; white-space:nowrap; }
  .led { width:8px; height:8px; border-radius:50%; background:var(--fg); flex:none; }
  .led.off { background:none; box-shadow:inset 0 0 0 1.5px var(--mut); }
  .led.err { background:var(--acc); }

  .grid2 { display:grid; grid-template-columns:minmax(0,1fr); gap:16px; align-items:start; }
  @media (min-width:900px) { .grid2 { grid-template-columns:repeat(2,minmax(0,1fr)); } }
  .stack > * + * { margin-top:16px; }
  .panel { border:1px solid var(--line); border-radius:4px; background:var(--panel); }
  .sec { display:flex; justify-content:space-between; align-items:center; gap:12px; padding:12px 16px 8px;
         color:var(--mut); font-size:11px; letter-spacing:.1em; text-transform:uppercase; }
  .sec .n { color:var(--fg); letter-spacing:0; text-transform:none; font-size:12px; }
  .pad { padding:12px 16px; }
  .tools { display:flex; flex-wrap:wrap; align-items:center; gap:8px; padding:12px 16px; }
  .tools label { display:flex; align-items:center; gap:8px; color:var(--mut); font-size:12px; }
  .hint { color:var(--mut); font-size:12px; line-height:18px; padding:8px 16px 12px; }

  .r { display:flex; justify-content:space-between; align-items:baseline; gap:12px; padding:8px 16px;
       border-top:1px solid var(--line); }
  .r:hover { background:var(--hov); }
  .r > span:first-child { color:var(--mut); font-size:12px; }
  .r .v { font-size:14px; text-align:right; }
  .r .v small { font-size:11px; color:var(--mut); margin-left:4px; }
  .r.err { box-shadow:inset 3px 0 0 var(--acc); }
  .r.err .v { color:var(--acc); }

  .note { margin:8px 16px 12px; padding:12px; border:1px dashed var(--line2); border-radius:2px;
          color:var(--mut); font-size:12px; line-height:18px; }
  .note b { display:block; color:var(--fg); margin-bottom:2px; }
  .note.err { border-color:var(--acc); border-style:solid; }
  .note.err b, .note.err { color:var(--acc); }
  .note.err { color:var(--mut); }

  .vid { display:block; width:100%; background:#000; border-top:1px solid var(--line); }
  .vidbox { position:relative; background:#000; border-top:1px solid var(--line); }
  .vidbox .vid { border-top:0; }
  .ph { position:absolute; inset:0; display:flex; align-items:center; justify-content:center; text-align:center;
        color:var(--mut); font-size:12px; letter-spacing:.06em; padding:16px; pointer-events:none; }
  .ph.err { color:var(--acc); }

  .btn { font:inherit; font-size:13px; color:var(--fg); background:none; border:1px solid var(--line2);
         border-radius:4px; padding:6px 12px; min-height:32px; cursor:pointer; text-decoration:none; display:inline-flex;
         align-items:center; justify-content:center; gap:8px; }
  .btn:hover { background:var(--hov); }
  .btn.p { background:var(--fg); color:var(--bg); border-color:var(--fg); }
  .btn.p:hover { opacity:.88; }
  .btn.on { border-color:var(--fg); background:rgba(255,255,255,.08); }
  .btn:disabled { opacity:.4; cursor:default; }
  .btn.block { width:100%; }
  .btn:focus-visible, select:focus-visible, input:focus-visible, a:focus-visible { outline:2px solid var(--acc); outline-offset:2px; }
  select, input[type=text], input:not([type]) { font:inherit; color:var(--fg); background:var(--bg);
         border:1px solid var(--line2); border-radius:4px; padding:5px 8px; min-height:32px; }
  input[type=text], input:not([type]) { width:100%; }
  input[type=range] { -webkit-appearance:none; appearance:none; width:100%; height:20px; background:transparent; margin:0; }
  input[type=range]::-webkit-slider-runnable-track { height:2px; background:var(--line2); }
  input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; width:12px; height:12px; margin-top:-5px; border-radius:50%; background:var(--fg); border:0; }
  input[type=range]::-moz-range-track { height:2px; background:var(--line2); }
  input[type=range]::-moz-range-thumb { width:12px; height:12px; border-radius:50%; background:var(--fg); border:0; }
  kbd { font:11px var(--mono); color:var(--mut); border:1px solid var(--line2); border-radius:2px; padding:0 5px; }
  .btn.p kbd { color:var(--bg); border-color:rgba(20,22,26,.4); }
  a { color:var(--fg); }

  table { border-collapse:collapse; width:100%; font-size:12px; }
  th { color:var(--mut); font-weight:400; font-size:11px; letter-spacing:.06em; text-transform:uppercase;
       text-align:right; padding:6px 8px; border-top:1px solid var(--line); border-bottom:1px solid var(--line); white-space:nowrap; }
  td { padding:6px 8px; text-align:right; border-bottom:1px solid var(--line); }
  th:first-child, td:first-child { text-align:left; padding-left:16px; }
  th:last-child, td:last-child { padding-right:16px; }
  tbody tr:hover td, tr:hover > td { background:var(--hov); }
  .tscroll { overflow-x:auto; }
  .sw { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:6px; vertical-align:-1px;
        box-shadow:inset 0 0 0 1px var(--line2); }
  .sw.sq { border-radius:2px; }

  .sl { display:grid; grid-template-columns:170px minmax(0,1fr) 48px; gap:2px 12px; align-items:center; padding:4px 16px 12px; }
  @media (max-width:520px) { .sl { grid-template-columns:96px minmax(0,1fr) 44px; } }
  .sl .k { font-size:12px; color:var(--mut); }
  .sl .o { text-align:right; font-size:13px; }
  .sl .grp { grid-column:1/-1; margin-top:12px; padding-top:8px; border-top:1px solid var(--line);
             color:var(--mut); font-size:11px; letter-spacing:.1em; text-transform:uppercase; }
  .sl .grp:first-child { margin-top:0; border-top:0; }
  .sl .hn { grid-column:1/-1; color:var(--mut); font-size:11px; line-height:16px; margin:-2px 0 4px; opacity:.85; }

  #setup { position:fixed; inset:0; background:rgba(20,22,26,.94); display:flex; align-items:center; justify-content:center; padding:16px; z-index:5; }
  #setup form { width:100%; max-width:380px; }
  #setup h2 { margin:0; padding:12px 16px 8px; color:var(--mut); font:400 11px/20px var(--mono); letter-spacing:.1em; text-transform:uppercase; }
  #setup .f { padding:0 16px 12px; }
  #err { color:var(--acc); min-height:20px; font-size:12px; padding:4px 0; }
  .strip { display:flex; flex-wrap:wrap; gap:4px 24px; align-items:baseline; padding:12px 16px; }
  .strip span.k { color:var(--mut); font-size:12px; margin-right:8px; }
  .strip .grow { flex:1; }
  .sq1 { aspect-ratio:1/1; }
  .sq1 .vid { height:100%; object-fit:contain; image-rendering:pixelated; }
  .flash { outline:3px solid var(--acc); outline-offset:-3px; }
  #logs { height:260px; overflow-y:auto; font-size:12px; line-height:18px; padding:8px 16px 12px; white-space:pre-wrap;
          border-top:1px solid var(--line); color:var(--mut); }
</style></head>
<body>

<div id="setup"><form id="setupForm" class="panel">
  <h2>Папка датасета</h2>
  <div class="f">
    <input id="name" placeholder="например run1" autofocus autocomplete="off">
    <div id="err"></div>
    <div class="mut" style="font-size:12px;margin-bottom:12px">Уже есть: <span id="existing">-</span></div>
    <button class="btn p block" type="submit">Создать / продолжить</button>
  </div>
</form></div>
<div class="wrap"><div class="bar"><h1>MAZE2027 <span>/ датасет</span></h1><div class="st"><i class="led off" id="conn-led"></i><span id="conn">подключение</span></div></div>
  <div class="panel" style="margin-bottom:16px">
    <div class="strip">
      <span><span class="k">Папка</span><b id="folder">-</b></span>
      <span><span class="k">Кадров</span><b id="count">0</b></span>
      <a href="#" id="change" class="mut">сменить папку</a>
      <span class="grow"></span>
      <span class="mut">клавиши <kbd>1</kbd> <kbd>2</kbd></span>
    </div>
  </div>
  <div class="grid2">
    <div class="panel">
      <div class="sec"><span>Камера 0 · правая</span></div>
      <div class="vidbox sq1" id="box0"><img class="vid" id="img0" src="/stream/0.mjpg" alt=""></div>
      <div class="pad"><button class="btn p block" onclick="save(0)">Сохранить <kbd>1</kbd></button></div>
    </div>
    <div class="panel">
      <div class="sec"><span>Камера 1 · левая</span></div>
      <div class="vidbox sq1" id="box1"><img class="vid" id="img1" src="/stream/1.mjpg" alt=""></div>
      <div class="pad"><button class="btn p block" onclick="save(1)">Сохранить <kbd>2</kbd></button></div>
    </div>
  </div>
  <div class="grid2" style="margin-top:16px">
    <div class="panel"><div class="sec"><span>Журнал</span></div><div id="logs"></div></div>
    <div class="panel">
      <div class="sec"><span>Настройки камер · обе сразу</span></div>
      <div class="sl" id="settings"></div>
    </div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
function esc(s) { return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function setConn(kind, text) { $('conn-led').className = 'led' + (kind === 'on' ? '' : ' ' + kind); $('conn').textContent = text; }

let lastLog = 0, busy = false;

function showSession(s) {
  $('folder').textContent = s.folder || '-';
  $('count').textContent = s.count || 0;
  $('setup').style.display = s.folder ? 'none' : 'flex';
  if (s.existing) $('existing').textContent = s.existing.join(', ') || '-';
}
async function loadSession() {
  showSession(await (await fetch('/api/session')).json());
}
$('setupForm').onsubmit = async e => {
  e.preventDefault();
  const r = await fetch('/api/session', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                         body: JSON.stringify({name: $('name').value})});
  const j = await r.json();
  if (!r.ok) { $('err').textContent = j.error; return; }
  $('err').textContent = '';
  showSession(j);
};
$('change').onclick = async e => {
  e.preventDefault();
  const j = await (await fetch('/api/session')).json();
  $('setup').style.display = 'flex';
  $('name').value = j.folder || '';
  $('name').focus();
};
async function save(cam) {
  if (busy || $('setup').style.display !== 'none') return;
  busy = true;
  try {
    const r = await fetch('/api/save/' + cam, {method: 'POST'});
    const j = await r.json();
    if (r.ok) {
      $('count').textContent = j.count;
      const b = $('box' + cam);
      b.classList.add('flash'); setTimeout(() => b.classList.remove('flash'), 150);
    }
  } finally { busy = false; pollLogs(); }
}
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === '1') save(0);
  if (e.key === '2') save(1);
});
async function pollLogs() {
  try {
    const j = await (await fetch('/api/logs?since=' + lastLog)).json();
    if (j.lines.length) {
      const box = $('logs');
      const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 5;
      box.textContent += j.lines.join('\\n') + '\\n';
      if (atBottom) box.scrollTop = box.scrollHeight;
    }
    lastLog = j.last;
    setConn('on', 'онлайн');
  } catch (e) { setConn('off', 'нет связи'); }
}
let camSpec = {}, sendTimer = null, pending = {};
function sendSetting(k, v) {
  pending[k] = v;
  clearTimeout(sendTimer);
  sendTimer = setTimeout(async () => {
    const body = pending; pending = {};
    await fetch('/api/camera_settings', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                         body: JSON.stringify(body)});
    pollLogs();
  }, 250);
}
function cell(cls, text) { const s = document.createElement('span'); s.className = cls; s.textContent = text; return s; }
async function loadCamSettings() {
  const j = await (await fetch('/api/camera_settings')).json();
  camSpec = j.spec;
  const box = $('settings');
  box.innerHTML = '';
  for (const [k, [def, lo, hi, step]] of Object.entries(j.spec)) {
    const r = document.createElement('input');
    Object.assign(r, {type: 'range', min: lo, max: hi, step: step, value: j.values[k], id: 'set_' + k});
    const out = cell('o', j.values[k]);
    r.oninput = () => { out.textContent = r.value; sendSetting(k, r.value); };
    box.append(cell('k', k), r, out);
  }
  const sel = document.createElement('select');
  j.awb_modes.forEach((m, i) => sel.add(new Option(m, i, false, i === j.values.AwbMode)));
  sel.onchange = () => sendSetting('AwbMode', sel.value);
  const aeSel = document.createElement('select');
  j.ae_modes.forEach((m, i) => aeSel.add(new Option(m, i, false, i === j.values.AeExposureMode)));
  aeSel.onchange = () => sendSetting('AeExposureMode', aeSel.value);
  box.append(cell('k', 'AeExposureMode'), aeSel, document.createElement('span'));
  const reset = document.createElement('button');
  reset.className = 'btn'; reset.textContent = 'сброс';
  reset.onclick = async () => {
    const d = {AwbMode: 0, AeExposureMode: 1};
    for (const [k, v] of Object.entries(camSpec)) d[k] = v[0];
    await fetch('/api/camera_settings', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                         body: JSON.stringify(d)});
    loadCamSettings(); pollLogs();
  };
  box.append(cell('k', 'AwbMode'), sel, reset);
  box.append(cell('hn', 'ExposureTime = 0 — автоэкспозиция; > 0 — ручная выдержка (мкс) с AnalogueGain. Длинная выдержка смазывает кадр на ходу.'));
}
loadCamSettings();
setInterval(pollLogs, 1000);
loadSession(); pollLogs();
</script>
</body></html>
"""


if __name__ == '__main__':
    os.makedirs(DATASETS_ROOT, exist_ok=True)
    cameras = {i: Camera(i) for i in CAM_NAMES}
    log(f'Сервер на порту {HTTP_PORT}, датасеты в {DATASETS_ROOT}')
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
