#!/usr/bin/env python3
"""Live-детекция меток на обеих камерах с веб-страницей.

/      — для каждой камеры видео с рамками и «окошко» с последней найденной
         меткой: вырезка, класс, уверенность, тип жертвы; для когнитивной цели —
         цвета пяти колец (от центра к краю), их баллы и чексумма.
/tune  — настройка порогов цвета колец на последней увиденной цели:
         ползунки применяются сразу, «Сохранить» пишет ~/maze/ring_colors.json,
         клик по картинке показывает HSV точки.

Камеры берутся из dataset_web.Camera — с теми же настройками из
~/maze/camera_settings.json, на которых снимался датасет.
"""
import base64
import os
import sys
import threading
import time
from threading import Lock

import cv2
from flask import Flask, Response, jsonify, request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import colors_page  # noqa: E402
import ring_colors as rc  # noqa: E402
from dataset_web import CAM_NAMES, STILL_SIZE, Camera, log  # noqa: E402

MODEL_PATH = os.path.expanduser(f"~/maze/models/maze_v26n_{os.environ.get('MAZE_MODEL', 'B')}.pt")
HTTP_PORT = 8000
IMGSZ = 320
CONF = 0.45
TUNE_PX = 900           # размер увеличенной вырезки на странице настройки
CARD_PX = 560           # размер вырезки метки в карточке на главной странице
# Когда метка появилась в кадре, снимаем отдельный кадр с длинной выдержкой —
# по нему и читаем метку/цвета (меньше шума и синевы в тёмном).
LONG_MAX_EXPOSURE_US = 500_000   # выдержка — по автоэкспозиции, не длиннее этого
LONG_REARM_S = 1.0      # метка должна пропасть хотя бы на столько, чтобы снять снова

LETTERS = {  # класс модели -> (буква, тип жертвы)
    'phi':   ('Φ', 'damaged'),
    'psi':   ('Ψ', 'stable'),
    'omega': ('Ω', 'unharmed'),
}
COG_CLASS = 'cognetive'
SUM_TO_TYPE = {2: 'damaged', 1: 'stable', 0: 'unharmed'}   # иначе — фейк

cfg_lock = Lock()
ring_cfg = rc.load_config()


def get_cfg():
    with cfg_lock:
        return dict(ring_cfg)


def set_cfg(values, save=False):
    with cfg_lock:
        for k, v in values.items():
            if k in rc.DEFAULTS:
                ring_cfg[k] = type(rc.DEFAULTS[k])(v)
        if save:
            rc.save_config(ring_cfg)


def cog_verdict(rings):
    cs = rc.checksum(rings)
    return cs, (None if cs is None else SUM_TO_TYPE.get(cs, 'fake'))


def crop_box(frame, xyxy, pad=0.15):
    x1, y1, x2, y2 = xyxy
    w, h = x2 - x1, y2 - y1
    return (max(0, int(x1 - w * pad)), max(0, int(y1 - h * pad)),
            min(frame.shape[1], int(x2 + w * pad)), min(frame.shape[0], int(y2 + h * pad)))


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0


def tune_scale(w, h):
    return TUNE_PX / max(1, w, h)


def jpg_b64(img, quality=85):
    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(jpg.tobytes()).decode() if ok else None


def crop_b64(frame, xyxy, size=CARD_PX):
    X1, Y1, X2, Y2 = crop_box(frame, xyxy)
    crop = frame[Y1:Y2, X1:X2]
    if crop.size == 0:
        return None
    scale = size / max(crop.shape[:2])
    # Уменьшать - усреднением (NEAREST при уменьшении выбрасывает пиксели и
    # даёт кашу), увеличивать - NEAREST: видны настоящие пиксели снимка.
    interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_NEAREST
    return jpg_b64(cv2.resize(crop, None, fx=scale, fy=scale, interpolation=interp), 92)


class Detector:
    def __init__(self, cam):
        self.cam = cam
        self.lock = Lock()
        self.jpg = None
        self.fps = 0.0
        self.last = None          # последняя найденная метка (держим на экране)
        self.visible = False
        self.cog = None           # (кадр, рамка) последней когнитивной цели — для /tune
        self.frozen = False       # /tune: не обновлять self.cog
        self.last_seen = 0.0      # когда метка последний раз была в кадре
        self.last_cls = None      # класс метки на обычном кадре (не на снимке)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        from ultralytics import YOLO
        model = YOLO(MODEL_PATH)   # своя копия на поток — predict не потокобезопасен
        log(f'Камера {self.cam.idx}: модель загружена')
        t_prev = time.time()
        while True:
            frame = self.cam.latest()
            if frame is None:
                time.sleep(0.1)
                continue
            res = model.predict(frame, imgsz=IMGSZ, conf=CONF, verbose=False)[0]
            shown = frame.copy()
            info = None
            if len(res.boxes):
                b = max(res.boxes, key=lambda b: float(b.conf[0]))
                cls = model.names[int(b.cls[0])]
                now = time.time()
                # «появилась» = не было видно LONG_REARM_S или сменился класс
                # на обычном кадре (со снимком не сравниваем — иначе при их
                # расхождении снимок делался бы на каждом кадре)
                appeared = now - self.last_seen > LONG_REARM_S or cls != self.last_cls
                self.last_seen = now
                self.last_cls = cls
                conf = float(b.conf[0])
                xyxy = [float(v) for v in b.xyxy[0].tolist()]
                x1, y1, x2, y2 = map(int, xyxy)
                cv2.rectangle(shown, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(shown, f'{cls} {conf:.2f}', (x1, max(12, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                if appeared:
                    info = self._read_long(model, cls, conf, xyxy)
                else:
                    with self.lock:   # та же метка висит в кадре — держим длинный снимок
                        info = self.last
            ok, jpg = cv2.imencode('.jpg', shown, [cv2.IMWRITE_JPEG_QUALITY, 75])
            now = time.time()
            with self.lock:
                if ok:
                    self.jpg = jpg.tobytes()
                self.visible = info is not None
                if info is not None:
                    self.last = info
                self.fps = 0.8 * self.fps + 0.2 / max(1e-3, now - t_prev)
            t_prev = now

    def _read_long(self, model, cls, conf, xyxy):
        """Класс — по обычному кадру (на таких модель обучена); снимок с длинной
        выдержкой — только для цветов колец и вырезки.

        Снимок той же области, поэтому рамка переносится умножением. Если на
        снимке модель нашла ту же метку — берём её рамку (точнее), класс снимка
        только показываем.
        """
        t0 = time.time()
        frame, md = self.cam.capture_long(None, None, size=STILL_SIZE,
                                          auto_max_us=LONG_MAX_EXPOSURE_US)
        still_cls = None
        if frame is not None:
            k = frame.shape[1] / IMGSZ
            box = [v * k for v in xyxy]
            small = cv2.resize(frame, (IMGSZ, IMGSZ), interpolation=cv2.INTER_AREA)
            r = model.predict(small, imgsz=IMGSZ, conf=CONF, verbose=False)[0]
            if len(r.boxes):
                b = max(r.boxes, key=lambda b: float(b.conf[0]))
                still_cls = model.names[int(b.cls[0])]
                sb = [float(v) * k for v in b.xyxy[0].tolist()]
                if iou(sb, box) > 0.3:
                    box = sb
            xyxy = box
            exp_ms = round(md.get('ExposureTime', 0) / 1000)
        else:
            frame = self.cam.latest()
            exp_ms = None
        log(f'Камера {self.cam.idx}: {cls} {conf:.2f}'
            + (f', снимок {frame.shape[1]}x{frame.shape[0]} {exp_ms} мс'
               + (f' (на снимке модель видит {still_cls})' if still_cls and still_cls != cls else '')
               if exp_ms else ', снимок не удался — всё по обычному кадру')
            + f' ({time.time() - t0:.1f} с)')
        info = {'class': cls, 'conf': round(conf, 2), 'ts': time.strftime('%H:%M:%S'),
                'crop': crop_b64(frame, xyxy), 'exposure_ms': exp_ms,
                'res': f'{frame.shape[1]}x{frame.shape[0]}', 'still_class': still_cls}
        if cls == COG_CLASS:
            rings, _, _ = rc.read_rings(frame, xyxy, get_cfg())
            cs, vtype = cog_verdict(rings)
            info.update(rings=rings, checksum=cs, type=vtype)
            with self.lock:
                if not self.frozen:
                    self.cog = (frame, xyxy)
        else:
            letter, vtype = LETTERS.get(cls, (cls, None))
            info.update(letter=letter, type=vtype)
        return info

    def status(self):
        with self.lock:
            return {'visible': self.visible, 'fps': round(self.fps, 1), 'last': self.last}

    def tune_view(self):
        """Последняя когнитивная цель, перечитанная с текущими порогами."""
        with self.lock:
            cog, frozen = self.cog, self.frozen
        if cog is None:
            return {'has': False, 'frozen': frozen}
        frame, xyxy = cog
        cfg = get_cfg()
        rings, ell, ctx = rc.read_rings(frame, xyxy, cfg)
        # показываем кадр ПОСЛЕ баланса белого — такие цвета и видит код
        shown = ctx['wb'].copy()
        rc.read_rings(frame, xyxy, cfg, draw_on=shown)
        cs, vtype = cog_verdict(rings)
        X1, Y1, X2, Y2 = crop_box(frame, xyxy, pad=0.3)
        k = tune_scale(X2 - X1, Y2 - Y1)
        big = cv2.resize(shown[Y1:Y2, X1:X2], None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
        g = ctx['gains']
        return {'has': True, 'frozen': frozen, 'img': jpg_b64(big, 90), 'rings': rings,
                'checksum': cs, 'type': vtype,
                'paper_v': int(ctx['paper']), 'paper_s': int(ctx['paper_s']),
                'gains': [round(float(g[2]), 2), round(float(g[1]), 2), round(float(g[0]), 2)],
                'center_found': bool(ctx.get('center_found'))}

    def probe(self, x, y):
        """x, y — координаты клика на увеличенной вырезке."""
        with self.lock:
            cog = self.cog
        if cog is None:
            return None
        frame, xyxy = cog
        ctx = rc.prepare(frame, xyxy, get_cfg())
        X1, Y1, X2, Y2 = crop_box(frame, xyxy, pad=0.3)
        k = tune_scale(X2 - X1, Y2 - Y1)
        return rc.probe(ctx, X1 + x / k, Y1 + y / k)


app = Flask(__name__)
detectors = {}


@app.route('/')
def index():
    return PAGE


@app.route('/tune')
def tune():
    return TUNE_PAGE


@app.route('/stream/<int:cam_id>.mjpg')
def stream(cam_id):
    det = detectors.get(cam_id)
    if det is None:
        return 'нет камеры', 404

    def gen():
        last = None
        while True:
            with det.lock:
                jpg = det.jpg
            if jpg is None or jpg is last:
                time.sleep(0.03)
                continue
            last = jpg
            yield b'--FRAME\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=FRAME')


@app.route('/api/status')
def api_status():
    return jsonify({str(i): d.status() for i, d in detectors.items()})


@app.route('/api/tune/<int:cam_id>')
def api_tune(cam_id):
    det = detectors.get(cam_id)
    return jsonify(det.tune_view() if det else {'has': False})


TUNE_SAVE_DIR = os.path.expanduser('~/maze/tune_frames')


@app.route('/api/freeze/<int:cam_id>', methods=['POST'])
def api_freeze(cam_id):
    det = detectors.get(cam_id)
    if det:
        with det.lock:
            det.frozen = bool((request.get_json(silent=True) or {}).get('freeze'))
            cog = det.cog
        msg = f'Камера {cam_id}: кадр для настройки {"заморожен" if det.frozen else "обновляется"}'
        if det.frozen and cog is not None:
            # исходный кадр + рамка — чтобы разбирать трудные цели на компьютере
            os.makedirs(TUNE_SAVE_DIR, exist_ok=True)
            name = time.strftime('%Y%m%d_%H%M%S') + f'_cam{cam_id}'
            cv2.imwrite(os.path.join(TUNE_SAVE_DIR, name + '.png'), cog[0])
            with open(os.path.join(TUNE_SAVE_DIR, name + '.txt'), 'w') as f:
                f.write(' '.join(f'{v:.1f}' for v in cog[1]))
            msg += f', сохранён {name}.png'
        log(msg)
    return jsonify(ok=True)


@app.route('/api/probe/<int:cam_id>', methods=['POST'])
def api_probe(cam_id):
    det = detectors.get(cam_id)
    d = request.get_json(silent=True) or {}
    return jsonify(det.probe(float(d.get('x', 0)), float(d.get('y', 0))) if det else None)


@app.route('/api/ring_config', methods=['GET', 'POST'])
def api_ring_config():
    if request.method == 'POST':
        d = request.get_json(silent=True) or {}
        with cfg_lock:
            if d.get('reset'):
                ring_cfg.clear()
                ring_cfg.update(rc.DEFAULTS)
            for k, v in (d.get('values') or {}).items():
                if k in rc.DEFAULTS:
                    ring_cfg[k] = type(rc.DEFAULTS[k])(v)
            if d.get('save'):
                rc.save_config(ring_cfg)
                log(f'Пороги колец сохранены в {rc.CONFIG_PATH}')
    with cfg_lock:
        return jsonify(values=dict(ring_cfg), defaults=rc.DEFAULTS)


PAGE = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Детекция меток</title>
<style>
  body { margin: 0; font-family: sans-serif; background: #1b1b1b; color: #eee; }
  header { padding: 10px 16px; background: #262626; }
  header a { color: #8ab4f8; }
  main { display: flex; gap: 16px; padding: 16px; flex-wrap: wrap; }
  .cam { background: #262626; padding: 12px; border-radius: 8px; display: flex; gap: 12px; flex-wrap: wrap; }
  .cam h3 { margin: 0 0 8px; font-size: 16px; width: 100%; }
  .cam img.video { width: 560px; height: 560px; max-width: 90vw; max-height: 90vw; background: #000; }
  .card { width: 580px; max-width: 90vw; background: #111; border-radius: 8px; padding: 12px; box-sizing: border-box; }
  .card.stale { opacity: .45; }
  .crop { width: 100%; aspect-ratio: 1; background: #000; object-fit: contain; border-radius: 4px; }
  .big { font-size: 56px; text-align: center; line-height: 1.1; margin: 6px 0; }
  .row { display: flex; justify-content: space-between; margin: 3px 0; font-size: 17px; }
  .type { font-size: 20px; font-weight: bold; text-align: center; margin: 8px 0; padding: 6px; border-radius: 4px; }
  .damaged { background: #b71c1c; } .stable { background: #f9a825; color: #000; }
  .unharmed { background: #2e7d32; } .fake, .none { background: #555; }
  .ring { display: flex; align-items: center; gap: 8px; font-size: 18px; margin: 3px 0; }
  .sw { width: 24px; height: 24px; border-radius: 50%; border: 1px solid #888; flex: none; }
  .muted { color: #999; font-size: 13px; }
</style></head><body>
<header><a href="/tune">настройка цветов колец →</a> &nbsp; <a href="/colors">разметка цветов →</a></header>
<main>
  <div class="cam"><h3>Камера 0 (правая) <span class="muted" id="fps0"></span></h3>
    <img class="video" src="/stream/0.mjpg"><div class="card" id="card0">нет данных</div></div>
  <div class="cam"><h3>Камера 1 (левая) <span class="muted" id="fps1"></span></h3>
    <img class="video" src="/stream/1.mjpg"><div class="card" id="card1">нет данных</div></div>
</main>
<script>
const TYPE_RU = {damaged: 'Повреждённая (2 аптечки)', stable: 'Стабильная (1 аптечка)',
                 unharmed: 'Невредимая (0 аптечек)', fake: 'Фейк — не жертва'};
const COLOR_RU = {black: 'чёрный', red: 'красный', yellow: 'жёлтый', green: 'зелёный', blue: 'синий', '?': 'не понял'};
const COLOR_CSS = {black: '#000', red: '#e53935', yellow: '#fdd835', green: '#43a047', blue: '#1e40ff', '?': '#777'};
const RING_RU = ['центр', 'кольцо 2', 'кольцо 3', 'кольцо 4', 'внешнее'];
const sgn = v => v === null || v === undefined ? '?' : (v > 0 ? '+' + v : '' + v);

function render(id, s) {
  const el = document.getElementById('card' + id);
  document.getElementById('fps' + id).textContent = s.fps ? s.fps + ' к/с' : '';
  const L = s.last;
  if (!L) { el.innerHTML = '<div class="muted">меток пока не было</div>'; return; }
  el.classList.toggle('stale', !s.visible);
  let h = L.crop ? `<img class="crop" src="data:image/jpeg;base64,${L.crop}">` : '';
  if (L.class === 'cognetive') {
    h += `<div class="big" style="font-size:28px">Когнитивная цель</div>`;
    h += L.rings.map((r, i) => `<div class="ring"><span class="sw" style="background:${COLOR_CSS[r.color]}"></span>
      <span style="flex:1${r.reliable === false ? ';color:#f99' : ''}">${RING_RU[i]}: ${COLOR_RU[r.color]}${r.reliable === false ? ' (?)' : ''}</span><b>${sgn(r.score)}</b>
      <span class="muted">${Math.round(r.agree * 100)}%</span></div>`).join('');
    h += `<div class="row" style="font-size:18px;margin-top:8px"><span>Чексумма</span><b>${sgn(L.checksum)}</b></div>`;
  } else {
    h += `<div class="big">${L.letter}</div>`;
  }
  const t = L.type || 'none';
  h += `<div class="type ${t}">${TYPE_RU[L.type] || (L.class === 'cognetive' ? 'не прочитано — видно не всё/не уверен' : 'не определить')}</div>`;
  h += `<div class="row"><span>класс</span><span>${L.class}</span></div>`;
  if (L.still_class && L.still_class !== L.class)
    h += `<div class="row muted"><span>на снимке модель видит</span><span>${L.still_class}</span></div>`;
  h += `<div class="row"><span>уверенность</span><span>${Math.round(L.conf * 100)}%</span></div>`;
  h += `<div class="row"><span>кадр</span><span>${L.exposure_ms ? L.res + ', ' + L.exposure_ms + ' мс' : 'обычный ' + (L.res || '')}</span></div>`;
  h += `<div class="row muted"><span>${s.visible ? 'видна сейчас' : 'последняя, сейчас не видна'}</span><span>${L.ts}</span></div>`;
  el.innerHTML = h;
}
async function poll() {
  try {
    const j = await (await fetch('/api/status')).json();
    for (const id of ['0', '1']) if (j[id]) render(id, j[id]);
  } catch (e) {}
}
setInterval(poll, 300); poll();
</script></body></html>
"""


TUNE_PAGE = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Настройка колец</title>
<style>
  body { margin: 0; font-family: sans-serif; background: #1b1b1b; color: #eee; }
  header { padding: 10px 16px; background: #262626; display: flex; gap: 12px; align-items: center; flex-wrap: wrap; }
  header a { color: #8ab4f8; }
  button, select { font-size: 15px; padding: 6px 12px; border-radius: 4px; border: 0; cursor: pointer; }
  button.primary { background: #2e7d32; color: #fff; }
  button.on { background: #1565c0; color: #fff; }
  main { display: flex; gap: 16px; padding: 16px; flex-wrap: wrap; align-items: flex-start; }
  .panel { background: #262626; border-radius: 8px; padding: 12px; }
  #img { display: block; max-width: 95vw; cursor: crosshair; background: #000; min-width: 300px; min-height: 300px; }
  table { border-collapse: collapse; font-size: 14px; }
  td, th { padding: 4px 8px; text-align: right; }
  th { color: #aaa; font-weight: normal; }
  td:first-child, th:first-child { text-align: left; }
  .sw { display: inline-block; width: 14px; height: 14px; border-radius: 50%; border: 1px solid #888; vertical-align: middle; margin-right: 6px; }
  .sl { display: grid; grid-template-columns: 150px 1fr 52px; gap: 4px 10px; align-items: center; font-size: 14px; }
  .sl input[type=range] { width: 100%; }
  .grp { grid-column: 1 / -1; color: #8ab4f8; margin-top: 8px; font-size: 13px; }
  .hint { color: #999; font-size: 12px; grid-column: 1 / -1; margin: -2px 0 4px; }
  #probe { font: 13px monospace; white-space: pre; color: #ddd; }
  .big { font-size: 20px; margin: 8px 0; }
</style></head><body>
<header>
  <a href="/">← детекция</a> <a href="/colors">разметка цветов →</a>
  <label>Камера <select id="cam"><option value="0">0 (правая)</option><option value="1">1 (левая)</option></select></label>
  <button id="freeze">Заморозить кадр</button>
  <button class="primary" id="save">Сохранить пороги</button>
  <button id="reset">Сбросить к умолчаниям</button>
  <span id="msg" style="color:#8f8"></span>
</header>
<main>
  <div class="panel">
    <img id="img" alt="покажи когнитивную цель камере">
    <div id="state" style="color:#aaa;font-size:13px;margin-top:6px"></div>
    <div id="probe">клик по картинке — HSV точки</div>
  </div>
  <div class="panel">
    <div class="big" id="sum"></div>
    <table id="rings"></table>
    <div style="color:#aaa;font-size:12px;margin-top:6px">картинка — после баланса белого по бумаге; V/бум — яркость относительно бумаги;<br>красное согласие — кольцо ненадёжно (мало точек в кадре или разнобой)</div>
  </div>
  <div class="panel" style="min-width:380px"><div class="sl" id="sliders"></div></div>
</main>
<script>
const COLOR_RU = {black: 'чёрный', red: 'красный', yellow: 'жёлтый', green: 'зелёный', blue: 'синий', '?': 'не понял'};
const COLOR_CSS = {black: '#000', red: '#e53935', yellow: '#fdd835', green: '#43a047', blue: '#1e40ff', '?': '#777'};
const RING_RU = ['центр', 'кольцо 2', 'кольцо 3', 'кольцо 4', 'внешнее'];
const TYPE_RU = {damaged: 'повреждённая', stable: 'стабильная', unharmed: 'невредимая', fake: 'фейк'};
// ключ, подпись, мин, макс, шаг, подсказка
const SLIDERS = [
  ['#Баланс белого и надёжность'],
  ['paper_wb', 'баланс по бумаге', 0, 1, 1, '1 — выравнивать цвета по белой бумаге вокруг цели (против цветной подсветки)'],
  ['min_agree', 'мин. согласие кольца', 0.3, 1, 0.05, 'меньше — кольцо ненадёжно, чексумма «не прочитано»'],
  ['#Чёрный / синий'],
  ['black_v_ratio', 'чёрный: V/бум <', 0.05, 0.6, 0.01, 'темнее этого — чёрный. Синий обычно 0.33–0.45, чёрный 0.08–0.22'],
  ['dark_green', 'тёмно-зелёный ≠ чёрный', 0, 1, 1, '1 — тёмное с зелёным оттенком считать зелёным'],
  ['dark_green_min', '...если V/бум ≥', 0, 0.4, 0.01, 'совсем тёмное всё равно чёрное'],
  ['#Цветное или белое'],
  ['min_sat', 'насыщенность > бум +', 0, 150, 1, 'ниже — «не понял» (бумага, блик)'],
  ['#Оттенки (0..180)'],
  ['red_h1', 'красный: H ≤', 0, 30, 1, 'красный — от 0 до этого'],
  ['red_h2', 'красный: H ≥', 140, 180, 1, '...и от этого до 180'],
  ['yellow_lo', 'жёлтый от', 5, 40, 1], ['yellow_hi', 'жёлтый до', 15, 60, 1],
  ['green_lo', 'зелёный от', 25, 80, 1], ['green_hi', 'зелёный до', 60, 110, 1],
  ['blue_lo', 'синий от', 70, 120, 1], ['blue_hi', 'синий до', 110, 170, 1],
  ['#Поиск края цели'],
  ['ink_sat', 'чернила: S > бум +', 10, 150, 1, 'что считать цветом цели, а не бумагой'],
  ['ink_v_ratio', 'чернила: V/бум <', 0.2, 0.95, 0.01, '...или темнее этого'],
  ['min_fit_ratio', 'мин. размер эллипса', 0.3, 1, 0.01, 'если найденный край меньше этой доли рамки — берём круг по рамке'],
  ['find_center', 'центр по внутреннему диску', 0, 1, 1, '1 — искать центр по самому внутреннему кругу (помогает, когда цель снята под углом); крестик на картинке'],
  ['box_center_tol', 'допуск центра от рамки', 0.02, 0.4, 0.01, 'цель видна целиком, а центр эллипса дальше этого от центра рамки — берём эллипс по рамке'],
  ['samples', 'точек на кольцо', 6, 48, 1],
];
const $ = id => document.getElementById(id);
let cfg = {}, frozen = false, sendT = null, pending = {};
const cam = () => $('cam').value;

function buildSliders() {
  const box = $('sliders'); box.innerHTML = '';
  for (const s of SLIDERS) {
    if (s[0][0] === '#') { const g = document.createElement('div'); g.className = 'grp'; g.textContent = s[0].slice(1); box.append(g); continue; }
    const [k, label, lo, hi, step, hint] = s;
    const lab = document.createElement('span'); lab.textContent = label;
    const r = document.createElement('input');
    Object.assign(r, {type: 'range', min: lo, max: hi, step: step, value: cfg[k]});
    const out = document.createElement('span'); out.textContent = cfg[k];
    r.oninput = () => { out.textContent = r.value; pending[k] = r.value; clearTimeout(sendT); sendT = setTimeout(send, 120); };
    box.append(lab, r, out);
    if (hint) { const h = document.createElement('div'); h.className = 'hint'; h.textContent = hint; box.append(h); }
  }
}
async function post(url, body) {
  const r = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  return r.json();
}
async function send() { const v = pending; pending = {}; cfg = (await post('/api/ring_config', {values: v})).values; refresh(); }
async function loadCfg() { cfg = (await (await fetch('/api/ring_config')).json()).values; buildSliders(); }

async function refresh() {
  let j;
  try { j = await (await fetch('/api/tune/' + cam())).json(); } catch (e) { return; }
  frozen = j.frozen; $('freeze').classList.toggle('on', frozen);
  $('freeze').textContent = frozen ? 'Кадр заморожен — разморозить' : 'Заморозить кадр';
  if (!j.has) { $('state').textContent = 'когнитивных целей эта камера ещё не видела — покажи цель'; return; }
  $('img').src = 'data:image/jpeg;base64,' + j.img;
  $('state').textContent = `бумага: V=${j.paper_v}, S=${j.paper_s} · баланс R/G/B ×${j.gains.join('/')}`
    + (j.center_found ? ' · центр по диску' : ' · центр по эллипсу')
    + (frozen ? ' · кадр заморожен' : ' · обновляется');
  const cs = j.checksum === null ? '?' : (j.checksum > 0 ? '+' : '') + j.checksum;
  $('sum').innerHTML = `Чексумма <b>${cs}</b> — ${TYPE_RU[j.type] || 'не прочитано'}`;
  let h = '<tr><th>кольцо</th><th>цвет</th><th>балл</th><th>согл.</th><th>в кадре</th><th>H</th><th>S</th><th>V/бум</th><th>голоса</th></tr>';
  j.rings.forEach((r, i) => {
    const votes = Object.entries(r.votes).sort((a, b) => b[1] - a[1]).map(([c, n]) => `${COLOR_RU[c]} ${n}`).join(', ');
    h += `<tr><td>${RING_RU[i]}</td><td style="text-align:left"><span class="sw" style="background:${COLOR_CSS[r.color]}"></span>${COLOR_RU[r.color]}</td>
      <td>${r.score === null ? '?' : r.score}</td><td style="${r.reliable ? '' : 'color:#f66'}">${Math.round(r.agree * 100)}%</td>
      <td>${Math.round(r.in_frame * 100)}%</td><td>${r.h}</td><td>${r.s}</td><td>${r.v_ratio}</td>
      <td style="text-align:left;color:#aaa">${votes}</td></tr>`;
  });
  $('rings').innerHTML = h;
}
$('img').onclick = async e => {
  const im = $('img'), k = im.naturalWidth / im.clientWidth;
  const p = await post('/api/probe/' + cam(), {x: e.offsetX * k, y: e.offsetY * k});
  if (p) $('probe').textContent = `H=${p.h}  S=${p.s} (бум+${p.s_over_paper})  V=${p.v}  V/бум=${p.v_ratio}`;
};
$('freeze').onclick = async () => { await post('/api/freeze/' + cam(), {freeze: !frozen}); refresh(); };
$('save').onclick = async () => { await post('/api/ring_config', {values: {}, save: true});
  $('msg').textContent = 'сохранено'; setTimeout(() => $('msg').textContent = '', 2000); };
$('reset').onclick = async () => { cfg = (await post('/api/ring_config', {reset: true})).values; buildSliders(); refresh(); };
$('cam').onchange = refresh;
loadCfg().then(refresh);
setInterval(() => { if (!frozen) refresh(); }, 700);
</script></body></html>
"""


colors_page.register(app, detectors, get_cfg, set_cfg, MODEL_PATH, log)


if __name__ == '__main__':
    cameras = {i: Camera(i) for i in CAM_NAMES}
    detectors.update({i: Detector(c) for i, c in cameras.items()})
    log(f'Сервер на порту {HTTP_PORT}, модель {MODEL_PATH}; настройка колец — /tune')
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
