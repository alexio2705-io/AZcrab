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
<title>Сбор датасета</title>
<style>
  body { margin: 0; font-family: sans-serif; background: #1b1b1b; color: #eee; }
  header { padding: 10px 16px; background: #262626; display: flex; gap: 16px; align-items: center; flex-wrap: wrap; }
  main { display: flex; gap: 16px; padding: 16px; flex-wrap: wrap; }
  .cam { background: #262626; padding: 10px; border-radius: 6px; }
  .cam img { width: 480px; height: 480px; max-width: 90vw; max-height: 90vw; background: #000; display: block; image-rendering: pixelated; }
  button { font-size: 18px; padding: 10px 16px; margin-top: 8px; width: 100%; cursor: pointer;
           background: #2e7d32; color: #fff; border: 0; border-radius: 4px; }
  button:disabled { background: #555; cursor: default; }
  #logs { flex: 1; min-width: 300px; height: 540px; overflow-y: auto; background: #111;
          font: 13px monospace; padding: 8px; border-radius: 6px; white-space: pre-wrap; }
  #setup { position: fixed; inset: 0; background: rgba(0,0,0,.85); display: flex;
           align-items: center; justify-content: center; }
  #setup form { background: #262626; padding: 24px; border-radius: 8px; min-width: 320px; }
  #setup input { font-size: 18px; padding: 8px; width: 100%; box-sizing: border-box; }
  #err { color: #ff6b6b; min-height: 1.2em; }
  .flash { outline: 4px solid #fff; }
  #settings { margin: 0 16px 16px; background: #262626; padding: 10px 16px; border-radius: 6px;
              display: grid; grid-template-columns: 130px 1fr 60px; gap: 6px 12px; align-items: center; max-width: 700px; }
  #settings input[type=range] { width: 100%; }
  #settings .small { width: auto; font-size: 14px; padding: 6px 10px; margin: 0; }
</style></head><body>
<div id="setup"><form id="setupForm">
  <h2>Новая папка датасета</h2>
  <input id="name" placeholder="например run1" autofocus>
  <div id="err"></div>
  <div style="font-size:13px;color:#aaa">Уже есть: <span id="existing">—</span></div>
  <button type="submit">Создать / продолжить</button>
</form></div>
<header>
  <b>Папка:</b> <span id="folder">—</span>
  <b>Кадров:</b> <span id="count">0</span>
  <a href="#" id="change" style="color:#8ab4f8">сменить папку</a>
  <span style="color:#aaa">клавиши: 1 / 2</span>
</header>
<main>
  <div class="cam"><div>Камера 0 (правая)</div><img id="img0" src="/stream/0.mjpg">
    <button onclick="save(0)">Сохранить с камеры 0</button></div>
  <div class="cam"><div>Камера 1 (левая)</div><img id="img1" src="/stream/1.mjpg">
    <button onclick="save(1)">Сохранить с камеры 1</button></div>
  <div id="logs"></div>
</main>
<div id="settings"></div>
<script>
const $ = id => document.getElementById(id);
let lastLog = 0, busy = false;

function showSession(s) {
  $('folder').textContent = s.folder || '—';
  $('count').textContent = s.count || 0;
  $('setup').style.display = s.folder ? 'none' : 'flex';
  if (s.existing) $('existing').textContent = s.existing.join(', ') || '—';
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
      const img = $('img' + cam);
      img.classList.add('flash'); setTimeout(() => img.classList.remove('flash'), 150);
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
  } catch (e) {}
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
async function loadCamSettings() {
  const j = await (await fetch('/api/camera_settings')).json();
  camSpec = j.spec;
  const box = $('settings');
  box.innerHTML = '<b style="grid-column:1/-1">Настройки камер (обе сразу)</b>';
  for (const [k, [def, lo, hi, step]] of Object.entries(j.spec)) {
    const lab = document.createElement('span'); lab.textContent = k;
    const r = document.createElement('input');
    Object.assign(r, {type: 'range', min: lo, max: hi, step: step, value: j.values[k], id: 'set_' + k});
    const out = document.createElement('span'); out.textContent = j.values[k];
    r.oninput = () => { out.textContent = r.value; sendSetting(k, r.value); };
    box.append(lab, r, out);
  }
  const lab = document.createElement('span'); lab.textContent = 'AwbMode';
  const sel = document.createElement('select');
  j.awb_modes.forEach((m, i) => sel.add(new Option(m, i, false, i === j.values.AwbMode)));
  sel.onchange = () => sendSetting('AwbMode', sel.value);
  const aeLab = document.createElement('span'); aeLab.textContent = 'AeExposureMode';
  const aeSel = document.createElement('select');
  j.ae_modes.forEach((m, i) => aeSel.add(new Option(m, i, false, i === j.values.AeExposureMode)));
  aeSel.onchange = () => sendSetting('AeExposureMode', aeSel.value);
  box.append(aeLab, aeSel, document.createElement('span'));
  const hint = document.createElement('span');
  hint.style.cssText = 'grid-column:1/-1;font-size:13px;color:#aaa';
  hint.textContent = 'ExposureTime = 0 — автоэкспозиция; > 0 — ручная выдержка (мкс) с AnalogueGain. Длинная выдержка смазывает кадр на ходу.';
  box.append(hint);
  const reset = document.createElement('button');
  reset.className = 'small'; reset.textContent = 'сброс';
  reset.onclick = async () => {
    const d = {AwbMode: 0, AeExposureMode: 1};
    for (const [k, v] of Object.entries(camSpec)) d[k] = v[0];
    await fetch('/api/camera_settings', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                         body: JSON.stringify(d)});
    loadCamSettings(); pollLogs();
  };
  box.append(lab, sel, reset);
}
loadCamSettings();
setInterval(pollLogs, 1000);
loadSession(); pollLogs();
</script></body></html>
"""


if __name__ == '__main__':
    os.makedirs(DATASETS_ROOT, exist_ok=True)
    cameras = {i: Camera(i) for i in CAM_NAMES}
    log(f'Сервер на порту {HTTP_PORT}, датасеты в {DATASETS_ROOT}')
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
