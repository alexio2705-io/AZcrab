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
<title>Детекция меток - MAZE2027</title>
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

  .sq .vid { aspect-ratio:1/1; object-fit:contain; }
  .vhead { display:flex; gap:12px; padding:12px 16px; border-top:1px solid var(--line); align-items:stretch; }
  .crop { width:96px; height:96px; object-fit:contain; background:#000; border:1px solid var(--line); border-radius:2px; flex:none; }
  .vmain { flex:1; min-width:0; display:flex; flex-direction:column; justify-content:space-between; gap:8px; }
  .vname { font-size:28px; line-height:32px; font-weight:600; }
  .vname.s { font-size:20px; line-height:28px; }
  .vtype { display:flex; gap:8px; align-items:stretch; font-size:13px; }
  .vtype i { width:3px; background:var(--mut); flex:none; }
  .vtype.damaged i { background:var(--damaged); } .vtype.stable i { background:var(--stable); } .vtype.unharmed i { background:var(--unharmed); }
  .vtype.none { color:var(--mut); }
  .card.stale { opacity:.7; }
  .r.ring .v b { font-weight:600; margin-right:8px; }
</style></head>
<body>
<div class="wrap"><div class="bar"><h1>MAZE2027 <span>/ детекция меток</span></h1><nav class="nav"><a href="/" aria-current="page">детекция</a><a href="/tune">кольца</a><a href="/colors">цвета</a></nav><div class="st"><i class="led off" id="conn-led"></i><span id="conn">подключение</span></div></div>
  <div class="grid2">
    <div class="panel sq"><div class="sec"><span>Камера 0 · правая</span><span class="n" id="fps0"></span></div>
      <img class="vid" src="/stream/0.mjpg" alt=""><div class="card" id="card0"><div class="note">ожидание данных</div></div></div>
    <div class="panel sq"><div class="sec"><span>Камера 1 · левая</span><span class="n" id="fps1"></span></div>
      <img class="vid" src="/stream/1.mjpg" alt=""><div class="card" id="card1"><div class="note">ожидание данных</div></div></div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
function esc(s) { return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function setConn(kind, text) { $('conn-led').className = 'led' + (kind === 'on' ? '' : ' ' + kind); $('conn').textContent = text; }

const TYPE_RU = {damaged: 'Повреждённая · 2 аптечки', stable: 'Стабильная · 1 аптечка',
                 unharmed: 'Невредимая · 0 аптечек', fake: 'Фейк, не жертва'};
const COLOR_RU = {black: 'чёрный', red: 'красный', yellow: 'жёлтый', green: 'зелёный', blue: 'синий', '?': 'не понял'};
const COLOR_CSS = {black: '#000', red: '#e53935', yellow: '#fdd835', green: '#43a047', blue: '#1e40ff', '?': '#777'};
const RING_RU = ['центр', 'кольцо 2', 'кольцо 3', 'кольцо 4', 'внешнее'];
const sgn = v => v === null || v === undefined ? '?' : (v > 0 ? '+' + v : '' + v);
const row = (k, v, cls) => `<div class="r ${cls || ''}"><span>${k}</span><span class="v">${v}</span></div>`;

function render(id, s) {
  const el = $('card' + id);
  $('fps' + id).textContent = s.fps ? s.fps + ' к/с' : '';
  const L = s.last;
  if (!L) { el.classList.remove('stale'); el.innerHTML = '<div class="note"><b>Меток пока не было</b>Покажи метку камере.</div>'; return; }
  el.classList.toggle('stale', !s.visible);
  const t = L.type || 'none', cog = L.class === 'cognetive';
  let h = '<div class="vhead">' + (L.crop ? `<img class="crop" src="data:image/jpeg;base64,${L.crop}" alt="">` : '')
    + `<div class="vmain"><div class="vname${cog ? ' s' : ''}">${cog ? 'Когнитивная цель' : esc(L.letter)}</div>`
    + `<div class="vtype ${t}"><i></i><span>${esc(TYPE_RU[L.type] || (cog ? 'не прочитано: видно не всё или не уверен' : 'не определить'))}</span></div></div></div>`;
  if (cog) {
    h += L.rings.map((r, i) => {
      const bad = r.reliable === false;
      return `<div class="r ring${bad ? ' err' : ''}"><span><i class="sw" style="background:${COLOR_CSS[r.color]}"></i>${RING_RU[i]}: ${COLOR_RU[r.color]}${bad ? ' (?)' : ''}</span>`
        + `<span class="v"><b>${sgn(r.score)}</b><small>${Math.round(r.agree * 100)}%</small></span></div>`;
    }).join('');
    h += row('Чексумма', `<b>${sgn(L.checksum)}</b>`);
  }
  h += row('Класс', esc(L.class));
  if (L.still_class && L.still_class !== L.class) h += row('На снимке модель видит', esc(L.still_class));
  h += row('Уверенность', Math.round(L.conf * 100) + '<small>%</small>');
  h += row('Кадр', L.exposure_ms ? esc(L.res) + '<small>' + L.exposure_ms + ' мс</small>' : 'обычный' + (L.res ? '<small>' + esc(L.res) + '</small>' : ''));
  h += row(s.visible ? 'Видна сейчас' : 'Последняя, сейчас не видна', esc(L.ts));
  el.innerHTML = h;
}
let fails = 0;
async function poll() {
  try {
    const r = await fetch('/api/status');
    if (!r.ok) throw new Error(r.status);
    const j = await r.json(); fails = 0;
    for (const id of ['0', '1']) if (j[id]) render(id, j[id]);
    setConn('on', 'онлайн');
  } catch (e) { if (++fails >= 3) setConn('off', 'нет связи'); }
}
setInterval(poll, 300); poll();
</script>
</body></html>
"""


TUNE_PAGE = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Настройка колец - MAZE2027</title>
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

  .layout { display:grid; grid-template-columns:minmax(0,1fr); gap:16px; align-items:start; }
  @media (min-width:1000px) { .layout { grid-template-columns:minmax(0,1fr) 420px; } }
  #img { display:block; width:100%; max-width:560px; margin:0 auto; cursor:crosshair; background:#000; min-height:200px; }
  .imgbox { background:#000; border-top:1px solid var(--line); }
  #probe { padding:8px 16px 12px; font-size:12px; color:var(--mut); white-space:pre-wrap; }
  #state { padding:8px 16px 0; font-size:12px; color:var(--mut); }
  #sum { padding:4px 16px 12px; font-size:14px; }
  #sum b { font-size:20px; margin:0 4px; }
  #msg { color:var(--mut); font-size:12px; }
</style></head>
<body>
<div class="wrap"><div class="bar"><h1>MAZE2027 <span>/ кольца</span></h1><nav class="nav"><a href="/">детекция</a><a href="/tune" aria-current="page">кольца</a><a href="/colors">цвета</a></nav><div class="st"><i class="led off" id="conn-led"></i><span id="conn">подключение</span></div></div>
  <div class="panel" style="margin-bottom:16px"><div class="tools">
    <label>Камера <select id="cam"><option value="0">0 · правая</option><option value="1">1 · левая</option></select></label>
    <button class="btn" id="freeze">Заморозить кадр</button>
    <button class="btn p" id="save">Сохранить пороги</button>
    <button class="btn" id="reset">Сбросить к умолчаниям</button>
    <span id="msg"></span>
  </div></div>
  <div class="layout">
    <div class="stack">
      <div class="panel">
        <div class="sec"><span>Кадр после баланса белого</span></div>
        <div class="imgbox"><img id="img" alt="покажи когнитивную цель камере"></div>
        <div id="state"></div>
        <div id="probe">Клик по картинке покажет HSV точки.</div>
      </div>
      <div class="panel">
        <div class="sec"><span>Кольца</span></div>
        <div id="sum"></div>
        <div class="tscroll"><table id="rings"></table></div>
        <div class="hint">V/бум — яркость относительно бумаги. Согласие янтарным — кольцо ненадёжно: мало точек в кадре или разнобой.</div>
      </div>
    </div>
    <div class="panel"><div class="sec"><span>Пороги</span></div><div class="sl" id="sliders"></div></div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
function esc(s) { return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function setConn(kind, text) { $('conn-led').className = 'led' + (kind === 'on' ? '' : ' ' + kind); $('conn').textContent = text; }

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
let cfg = {}, frozen = false, sendT = null, pending = {};
const cam = () => $('cam').value;
const mk = (tag, cls, text) => { const e = document.createElement(tag); if (cls) e.className = cls; if (text !== undefined) e.textContent = text; return e; };

function buildSliders() {
  const box = $('sliders'); box.innerHTML = '';
  for (const s of SLIDERS) {
    if (s[0][0] === '#') { box.append(mk('div', 'grp', s[0].slice(1))); continue; }
    const [k, label, lo, hi, step, hint] = s;
    const r = document.createElement('input');
    Object.assign(r, {type: 'range', min: lo, max: hi, step: step, value: cfg[k]});
    r.setAttribute('aria-label', label);
    const out = mk('span', 'o', cfg[k]);
    r.oninput = () => { out.textContent = r.value; pending[k] = r.value; clearTimeout(sendT); sendT = setTimeout(send, 120); };
    box.append(mk('span', 'k', label), r, out);
    if (hint) box.append(mk('div', 'hn', hint));
  }
}
async function post(url, body) {
  const r = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  return r.json();
}
async function send() { const v = pending; pending = {}; cfg = (await post('/api/ring_config', {values: v})).values; refresh(); }
async function loadCfg() { cfg = (await (await fetch('/api/ring_config')).json()).values; buildSliders(); }

let fails = 0;
async function refresh() {
  let j;
  try { j = await (await fetch('/api/tune/' + cam())).json(); fails = 0; setConn('on', 'онлайн'); }
  catch (e) { if (++fails >= 3) setConn('off', 'нет связи'); return; }
  frozen = j.frozen; $('freeze').classList.toggle('on', frozen);
  $('freeze').textContent = frozen ? 'Кадр заморожен, разморозить' : 'Заморозить кадр';
  if (!j.has) { $('state').textContent = 'Когнитивных целей эта камера ещё не видела. Покажи цель.'; return; }
  $('img').src = 'data:image/jpeg;base64,' + j.img;
  $('state').textContent = `бумага: V=${j.paper_v}, S=${j.paper_s} · баланс R/G/B ×${j.gains.join('/')}`
    + (j.center_found ? ' · центр по диску' : ' · центр по эллипсу')
    + (frozen ? ' · кадр заморожен' : ' · обновляется');
  const cs = j.checksum === null ? '?' : (j.checksum > 0 ? '+' : '') + j.checksum;
  $('sum').innerHTML = `Чексумма<b>${cs}</b>${TYPE_RU[j.type] || 'не прочитано'}`;
  let h = '<thead><tr><th>кольцо</th><th>цвет</th><th>балл</th><th>согл.</th><th>в кадре</th><th>H</th><th>S</th><th>V/бум</th><th>голоса</th></tr></thead><tbody>';
  j.rings.forEach((r, i) => {
    const votes = Object.entries(r.votes).sort((a, b) => b[1] - a[1]).map(([c, n]) => `${COLOR_RU[c]} ${n}`).join(', ');
    h += `<tr><td>${RING_RU[i]}</td><td style="text-align:left"><i class="sw" style="background:${COLOR_CSS[r.color]}"></i>${COLOR_RU[r.color]}</td>
      <td>${r.score === null ? '?' : r.score}</td><td class="${r.reliable ? '' : 'bad'}">${Math.round(r.agree * 100)}%</td>
      <td>${Math.round(r.in_frame * 100)}%</td><td>${r.h}</td><td>${r.s}</td><td>${r.v_ratio}</td>
      <td style="text-align:left;color:var(--mut)">${votes}</td></tr>`;
  });
  $('rings').innerHTML = h + '</tbody>';
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
</script>
</body></html>
"""


colors_page.register(app, detectors, get_cfg, set_cfg, MODEL_PATH, log)


if __name__ == '__main__':
    cameras = {i: Camera(i) for i in CAM_NAMES}
    detectors.update({i: Detector(c) for i, c in cameras.items()})
    log(f'Сервер на порту {HTTP_PORT}, модель {MODEL_PATH}; настройка колец — /tune')
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
