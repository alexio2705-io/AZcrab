#!/usr/bin/env python3
"""Передняя USB-камера: метки вживую, реакция кота на дисплее и на странице.

Отдельная проверка без nav.py: живой поток на 127.0.0.1:8766 у них общий,
поэтому nav.py должен быть остановлен (и лидар у них один).

Кадр - либо в метки, либо в цвет пола, никогда в оба: лидар (walls.measure)
говорит, есть ли впереди стена ближе FRONT_WALL_MM. Стена - кадр
идёт в YOLO (та же модель и пороги, что в victim_detect.py), у когнитивной
цели читаются кольца
(ring_colors.py). Если метка прочиталась одинаково на CONFIRM_N кадрах подряд,
она засчитывается и уходит демону дисплея (oled.py) тем же живым потоком,
что шлёт nav.py. Кот на экране реагирует так же, как в заезде: буква - кот с
настроением, мишень - кольца.

Стены нет - YOLO не крутится, читается цвет следующей клетки
(front_floor.read_cell); решение есть всегда. Где робот в своей клетке, - по
лидару: вдоль хода по задней стене или по дальней стене следующей клетки,
вбок по боковой (lidar_pose). Поворот от сетки и наклон без nav.py не знаем
(гироскоп - на ESP), считаем нулевыми. Кадр повёрнут и без бампера
(front_floor.prep). Без калибровки - модель
камеры и пороги HSV (front_floor.DEFAULT_CAM, HSV_*); калибровка пола и
образцы цветов снимаются на странице и пишутся в ~/maze/front_floor.json.

Страница http://<IP>:8005 - видео с рамками и тот же экран дисплея.

    ~/maze/venv/bin/python -u ~/maze/front_cam.py
"""
import os
import sys
import threading
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from flask import Flask, Response, jsonify, request  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import front_floor as ff  # noqa: E402
import live  # noqa: E402
import oled  # noqa: E402
import ring_colors as rc  # noqa: E402
import walls  # noqa: E402
import watch_logic as wl  # noqa: E402
from ld19 import BAUDRATE, DEFAULT_PORT, LD19, ZERO_DIR  # noqa: E402
from victim_detect import (CLASS_LABEL, CLASS_TO_VICTIM, COG_CLASS, CONF, IMGSZ,  # noqa: E402
                           MIN_BOX_PX, SUM_TO_TYPE, YOLO_MODEL_PATH)

CAM_DEV = '/dev/v4l/by-id/usb-SYX-240327-J_HD_Camera-video-index0'
CAM_SIZE = (1280, 720)
HTTP_PORT = 8005
CONFIRM_N = 3          # кадров подряд с одним и тем же прочтением - метка засчитана
LOST_S = 1.0           # метки нет дольше - та же метка засчитается снова
OLED_ZOOM = 5
OLED_HZ = 10.0
STREAM_W = 540         # ширина видео на странице (кадр стоит вертикально)
WALL_MM = 145.0        # от центра робота до стены своей клетки (FRONT_CENTER_MM в nav.py)
FRONT_WALL_MM = 200.0  # стена впереди ближе этого - кадр в метки (FRONT_CAM_WALL_MM в nav.py)
LIDAR_STALE_S = 1.0    # замер старше - стены впереди не знаем, читаем пол


class UsbCamera:
    """Последний кадр с камеры: фоновый поток читает без остановки, иначе
    в буфере V4L копятся старые кадры."""

    def __init__(self, dev=CAM_DEV, size=CAM_SIZE):
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise OSError(f'камера {dev} не открылась')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.frame, self.n = None, 0
        self.cond = threading.Condition()
        threading.Thread(target=self._run, daemon=True, name='usb-cam').start()

    def _run(self):
        while True:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self.cond:
                self.frame, self.n = f, self.n + 1
                self.cond.notify_all()

    def next(self, after, timeout=2.0):
        """Кадр новее номера after: (номер, кадр) или (after, None)."""
        with self.cond:
            self.cond.wait_for(lambda: self.n > after, timeout)
            return (self.n, self.frame) if self.n > after else (after, None)


class Confirm:
    """Метка засчитывается, когда одно и то же прочтение (класс, тип) пришло
    CONFIRM_N раз подряд. Пока она в кадре, второй раз её не засчитываем;
    пропала дольше LOST_S - следующая засчитается снова."""

    def __init__(self, n=CONFIRM_N, lost_s=LOST_S):
        self.n, self.lost_s = n, lost_s
        self.key, self.count = None, 0
        self.reported = None
        self.seen_t = -1e9

    def in_view(self, now):
        return now - self.seen_t < self.lost_s

    def push(self, key, now):
        """key - (класс, тип) или None (метки нет). True - засчитать сейчас."""
        if key is None:
            if not self.in_view(now):
                self.key, self.count, self.reported = None, 0, None
            return False
        self.seen_t = now
        self.count = self.count + 1 if key == self.key else 1
        self.key = key
        if self.count >= self.n and key != self.reported and key[1] is not None:
            self.reported = key
            return True
        return False


def read_frame(model, cfg, frame):
    """Лучшая метка кадра: dict как у VictimDetector.read_wall (type None -
    кольца не прочитались) или None. Рисует рамку и кольца на копии кадра;
    возвращает (метка, картинка)."""
    shown = frame.copy()
    boxes = model.predict(frame, imgsz=IMGSZ, conf=CONF, save=False, verbose=False)[0].boxes
    found = wl.victim_boxes([(str(model.names[int(b.cls[0])]), float(b.conf[0]),
                              b.xyxy[0].tolist()) for b in boxes])
    if not found:
        return None, shown
    cls_name, conf, xyxy = max(found, key=lambda b: b[1])
    x1, y1, x2, y2 = map(int, xyxy)
    side_320 = max(x2 - x1, y2 - y1) * IMGSZ / max(frame.shape[:2])
    rings = checksum = None
    if side_320 < MIN_BOX_PX:
        victim_type, label = None, 'далеко'
    elif cls_name == COG_CLASS:
        ring_info, _, _ = rc.read_rings(frame, xyxy, cfg, draw_on=shown)
        rings = [r['color'] for r in ring_info]
        checksum = rc.checksum(ring_info)
        victim_type = None if checksum is None else SUM_TO_TYPE.get(checksum, 'fake')
        label = '?' if checksum is None else f'{checksum:+d}'
    elif cls_name in CLASS_TO_VICTIM:
        victim_type = CLASS_TO_VICTIM[cls_name]
        label = CLASS_LABEL[victim_type]
    else:
        return None, shown
    color = (0, 255, 0) if victim_type else (0, 200, 255)
    cv2.rectangle(shown, (x1, y1), (x2, y2), color, 3)
    cv2.putText(shown, f'{cls_name}={victim_type} {label} ({conf:.2f})', (x1, max(30, y1 - 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    v = {'type': victim_type, 'label': label, 'class': cls_name, 'conf': round(conf, 3),
         'rings': rings, 'checksum': checksum, 'cell': (0, 0),
         'blink': victim_type in ('damaged', 'stable', 'unharmed')}
    return v, shown


TILE_BGR = {'white': (255, 255, 255), 'black': (40, 40, 40), 'blue': (255, 120, 0),
            'red': (0, 0, 255), 'silver': (200, 200, 200)}


def draw_tile(shown, tile):
    """Четырёхугольник следующей клетки и что в нём прочиталось."""
    color = TILE_BGR[tile['color']]
    if tile.get('poly') is not None:
        pts = np.round(np.float32(tile['poly'])).astype(np.int32)
        cv2.polylines(shown, [pts], True, (0, 0, 0), 5)
        cv2.polylines(shown, [pts], True, color, 2)
    text = f"next: {tile['color']} conf {tile['conf']:.2f}"     # кириллицу cv2 не рисует
    cv2.putText(shown, text, (10, shown.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)
    cv2.putText(shown, text, (10, shown.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)


def canvas_image(cv, zoom=OLED_ZOOM):
    """Экран дисплея картинкой BGR: белое на чёрном, увеличено в zoom раз."""
    pages = np.frombuffer(bytes(cv.buf), np.uint8).reshape(oled.H // 8, 1, oled.W)
    bits = np.unpackbits(pages, axis=1, bitorder='little').reshape(oled.H, oled.W)
    img = np.where(bits[..., None] > 0, np.uint8(230), np.uint8(12)).repeat(3, axis=2)
    return cv2.resize(img, (oled.W * zoom, oled.H * zoom), interpolation=cv2.INTER_NEAREST)


class State:
    """Что видно сейчас и что засчитано - для живого потока и страницы."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.victims = []
        self.current = None          # последнее прочтение (или None)
        self.in_view = False
        self.fps = 0.0
        self.lock = threading.Lock()
        self.jpg = None              # кадр с рамками
        self.raw = None              # последний кадр без рамок (для калибровки)
        self.cal = ff.load()
        self.tile = None             # ff.read_cell следующей клетки (None - впереди стена)
        self.walls, self.walls_t = {}, -1e9   # walls.measure и когда снят
        self.wall_ahead, self.pose, self.front = False, None, None
        self.oled_jpg = None
        self.jpg_cond = threading.Condition()

    def snapshot(self, n=0):
        with self.lock:
            return {'phase': 'exploring', 'act': 'жертва' if self.in_view else '',
                    'run_s': time.monotonic() - self.t0, 'victims_n': len(self.victims),
                    'map': {'victims': list(self.victims)}}


def lidar_pose(w):
    """(ox, oy, 0) по замеру walls.measure: где центр робота от центра его
    клетки. Вдоль хода - по задней стене своей клетки или по дальней стене
    следующей; вбок - по ближней боковой. Стены нет - 0 по этой оси."""
    ox = oy = 0.0
    front, back = w.get('front'), w.get('back')
    if back is not None and back < walls.BACK_THRESH:
        ox = back - WALL_MM
    elif front is not None and FRONT_WALL_MM <= front < FRONT_WALL_MM + ff.CELL_MM:
        ox = WALL_MM + ff.CELL_MM - front
    left, right = w.get('left'), w.get('right')
    if left is not None and left < walls.SIDE_THRESH:
        oy = WALL_MM - left
    elif right is not None and right < walls.SIDE_THRESH:
        oy = right - WALL_MM
    lim = ff.CELL_MM / 2
    return max(-lim, min(lim, ox)), max(-lim, min(lim, oy)), 0.0


def lidar_loop(state):
    """Замер стен раз в оборот. Лидар не открылся или упал - пробуем снова;
    пока замера нет, стены впереди не видно и кадр идёт в цвет пола."""
    warned = False
    while True:
        try:
            with LD19(DEFAULT_PORT, BAUDRATE, zero_dir=ZERO_DIR) as lidar:
                for rev in lidar.revolutions():
                    w = walls.measure(rev)
                    with state.lock:
                        state.walls, state.walls_t = w, time.monotonic()
        except Exception as e:
            if not warned:
                print(f'  [WARN] лидар: {type(e).__name__}: {e} - стен не вижу, читаю пол')
                warned = True
        time.sleep(1.0)


def detect_loop(state, cam, model, cfg):
    conf = Confirm()
    n, t_last = 0, time.monotonic()
    while True:
        n, frame = cam.next(n)
        if frame is None:
            print('  [WARN] камера молчит')
            continue
        frame = ff.prep(frame)
        now = time.monotonic()
        with state.lock:
            cal, w = state.cal, state.walls
            fresh = now - state.walls_t < LIDAR_STALE_S
        front = w.get('front') if fresh else None
        wall_ahead = front is not None and front < FRONT_WALL_MM
        if wall_ahead:
            # стена впереди - читаем метки, пол не смотрим
            tile, pose = None, None
            v, shown = read_frame(model, cfg, frame)
        else:
            # проём - только цвет следующей клетки, YOLO не крутим
            pose = lidar_pose(w) if fresh else (0.0, 0.0, 0.0)
            tile, v, shown = ff.read_cell(frame, cal, pose), None, frame.copy()
            draw_tile(shown, tile)
        mode = 'WALL: labels' if wall_ahead else 'OPEN: floor'
        cv2.putText(shown, mode, (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5)
        cv2.putText(shown, mode, (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        key = None if v is None else (v['class'], v['type'])
        hit = conf.push(key, now)
        with state.lock:
            state.current, state.in_view = v, conf.in_view(now)
            state.raw, state.tile = frame, tile
            state.wall_ahead, state.pose, state.front = wall_ahead, pose, front
            state.fps = 0.8 * state.fps + 0.2 / max(1e-3, now - t_last)
            if hit:
                v['time'] = time.strftime('%H:%M:%S')
                state.victims.append(v)
        t_last = now
        if hit:
            rings = (' кольца=[' + ', '.join(v['rings']) + ']') if v['rings'] else ''
            print(f"[FRONT] {v['time']} {v['type'].upper()} ({v['label']})"
                  f"  класс={v['class']} conf={v['conf']:.2f}{rings}")
        k = STREAM_W / shown.shape[1]
        ok, buf = cv2.imencode('.jpg', cv2.resize(shown, (STREAM_W, int(shown.shape[0] * k))),
                               [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with state.jpg_cond:
                state.jpg = buf.tobytes()
                state.jpg_cond.notify_all()


def oled_loop(state):
    """Тот же экран, что на дисплее: свой Feed из тех же снимков и compose()."""
    feed = oled.Feed(start=False)
    feed.connected = True
    sysinfo = oled.SysInfo(start=False)
    cv = oled.Canvas()
    while True:
        now = time.monotonic()
        feed.take(state.snapshot(), now)
        oled.compose(cv, oled.build_state(now, feed, sysinfo))
        ok, buf = cv2.imencode('.png', canvas_image(cv))
        if ok:
            with state.jpg_cond:
                state.oled_jpg = buf.tobytes()
                state.jpg_cond.notify_all()
        time.sleep(1.0 / OLED_HZ)


PAGE = """<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Передняя камера</title><style>
body{margin:0;padding:16px;background:#111;color:#ddd;font:15px system-ui,sans-serif}
h1{font-size:18px;margin:0 0 12px}h2{font-size:15px;margin:18px 0 6px}
.row{display:flex;flex-wrap:wrap;gap:16px;align-items:flex-start}
img{max-width:100%;border-radius:6px;display:block}#cam{width:540px}
#oled{image-rendering:pixelated;width:640px;max-width:100%}
.st{margin-top:10px;white-space:pre-line;line-height:1.5}ol{margin:6px 0 0;padding-left:20px}
button{background:#2a2a2a;color:#eee;border:1px solid #555;border-radius:5px;padding:6px 10px;margin:2px;font:inherit;cursor:pointer}
button:hover{background:#3a3a3a}input,select{background:#222;color:#eee;border:1px solid #555;border-radius:4px;padding:4px;width:70px;font:inherit}
label{margin-right:10px}.note{color:#999;font-size:13px;max-width:640px}
</style></head><body><h1>Передняя USB-камера</h1><div class="row">
<img id="cam" src="/stream.mjpg" alt="камера"><div><img id="oled" src="/oled.mjpg" alt="дисплей">
<div id="st" class="st"></div>
<h2>Следующая клетка</h2><div id="tile" class="st"></div>
<h2>1. Калибровка пола</h2>
<div class="note">Робот в центре клетки, ровно по сетке. Чёрная или синяя плитка лежит целиком
в кадре; укажите, где она относительно центра робота. Обычно - плитка в соседней клетке:
150 / 300 / 300 / 0.</div>
<div><label>плитка <select id="qc"><option value="black">чёрная</option><option value="blue">синяя</option></select></label>
<label>до ближнего края, мм <input id="near" value="150"></label>
<label>глубина <input id="depth" value="300"></label><label>ширина <input id="width" value="300"></label>
<label>левее оси <input id="left" value="0"></label>
<button onclick="calib()">Снять калибровку</button></div><div id="cmsg" class="st"></div>
<h2>2. Образцы цветов</h2>
<div class="note">Робот в центре клетки, в соседней - плитка этого цвета. Каждый цвет - хотя бы раз;
лучше с нескольких плиток и мест.</div>
<div id="samp"></div><div id="smsg" class="st"></div>
</div></div><script>
const NAMES={white:'белая',black:'чёрная',blue:'синяя',red:'красная',silver:'зеркальная'};
async function post(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});return r.json()}
async function calib(){const v=id=>parseFloat(document.getElementById(id).value);
const r=await post('/api/calib_floor',{color:document.getElementById('qc').value,near_mm:v('near'),depth_mm:v('depth'),width_mm:v('width'),left_mm:v('left')});
document.getElementById('cmsg').textContent=r.ok?'калибровка сохранена':('не вышло: '+r.error)}
async function sample(c){const r=await post('/api/sample/'+c);document.getElementById('smsg').textContent=r.ok?`образец «${NAMES[c]}» добавлен`:('не вышло: '+r.error)}
async function clearS(c){if(!confirm(`Удалить образцы «${NAMES[c]}»?`))return;await post('/api/sample_clear/'+c)}
let sampDrawn='';
async function tick(){try{const s=await (await fetch('/api/state')).json();
let t=s.current?`в кадре: ${s.current.class} → ${s.current.type??'не прочиталась'} ${s.current.label} (${s.current.conf})`
+(s.current.rings?`\\nкольца: ${s.current.rings.join(', ')}`:''):'в кадре меток нет';
t+=`\\n${s.fps.toFixed(1)} кадр/с\\n\\nзасчитано: ${s.victims.length}`;
const d=document.getElementById('st');d.textContent=t;
if(s.victims.length){const ol=document.createElement('ol');for(const v of s.victims.slice().reverse()){
const li=document.createElement('li');li.textContent=`${v.time} ${v.type} ${v.label}`;ol.appendChild(li)}d.appendChild(ol)}
const tl=s.tile;let tt;
if(!tl){tt=`впереди стена (${s.front} мм): читаю метки, пол не смотрю`}else{
tt=`${NAMES[tl.color]}, уверенность ${tl.conf.toFixed(2)}`+(tl.why?` (${tl.why})`:'');
if(s.pose)tt+=`\\nробот от центра клетки: вперёд ${Math.round(s.pose[0])} мм, влево ${Math.round(s.pose[1])} мм`;
if(tl.n)tt+=`\\nвидно ${Math.round(tl.visible*100)}% клетки, ${tl.n} пикс.`;}
if(tl&&tl.votes&&Object.keys(tl.votes).length)tt+='\\nголоса: '+Object.entries(tl.votes).map(([k,v])=>`${NAMES[k]} ${v}`).join(', ');
tt+=`\\nпол: ${s.cal.H?'калибровка по плитке':'модель камеры (оценка по лидару)'}, цвет: ${Object.keys(s.cal.samples).length?'по образцам':'пороги HSV'}`;document.getElementById('tile').textContent=tt;
const key=JSON.stringify(s.cal.samples);if(key!==sampDrawn){sampDrawn=key;
document.getElementById('samp').innerHTML=Object.keys(NAMES).map(c=>`<div><button onclick="sample('${c}')">это ${NAMES[c]}</button> образцов: ${s.cal.samples[c]||0}`+(s.cal.samples[c]?` <button onclick="clearS('${c}')">удалить</button>`:'')+'</div>').join('')}
}catch(e){}setTimeout(tick,300)}tick()</script></body></html>"""


def make_app(state):
    app = Flask(__name__)

    def mjpeg(attr, mime):
        def gen():
            last = None
            while True:
                with state.jpg_cond:
                    state.jpg_cond.wait_for(lambda: getattr(state, attr) is not last, 2.0)
                    data = last = getattr(state, attr)
                if data:
                    yield (b'--FRAME\r\nContent-Type: ' + mime + b'\r\n\r\n' + data + b'\r\n')
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=FRAME')

    @app.route('/')
    def index():
        return PAGE

    @app.route('/stream.mjpg')
    def stream():
        return mjpeg('jpg', b'image/jpeg')

    @app.route('/oled.mjpg')
    def oled_stream():
        return mjpeg('oled_jpg', b'image/png')

    @app.route('/api/state')
    def api_state():
        with state.lock:
            tile = {k: v for k, v in (state.tile or {}).items() if k != 'poly'}
            cal = {'H': state.cal.get('H') is not None,
                   'samples': {c: len(v) for c, v in state.cal['samples'].items()}}
            return jsonify({'current': state.current, 'fps': state.fps,
                            'victims': state.victims, 'tile': tile, 'cal': cal,
                            'wall_ahead': state.wall_ahead, 'front': state.front,
                            'pose': state.pose})

    @app.route('/api/calib_floor', methods=['POST'])
    def api_calib_floor():
        a = request.get_json(force=True)
        with state.lock:
            frame = state.raw
        if frame is None:
            return jsonify({'ok': False, 'error': 'нет кадра'})
        quad, err = ff.find_quad(frame, a.get('color', 'black'))
        if quad is None:
            return jsonify({'ok': False, 'error': err})
        h = ff.calibrate_h(quad, float(a['near_mm']), float(a['depth_mm']),
                           float(a['width_mm']), float(a['left_mm']))
        with state.lock:
            state.cal = dict(state.cal, H=h.tolist(), H_quad=quad, H_size=list(frame.shape[:2]),
                             H_tile={k: a.get(k) for k in ('color', 'near_mm', 'depth_mm',
                                                           'width_mm', 'left_mm')})
            ff.save(state.cal)
        print(f'[FRONT] калибровка пола: углы {[[round(x) for x in p] for p in quad]}')
        return jsonify({'ok': True, 'quad': quad})

    @app.route('/api/sample/<color>', methods=['POST'])
    def api_sample(color):
        if color not in ff.COLORS:
            return jsonify({'ok': False, 'error': f'нет цвета {color}'})
        with state.lock:
            frame, cal = state.raw, state.cal
        if frame is None:
            return jsonify({'ok': False, 'error': 'нет кадра'})
        smp = ff.sample_cell(frame, cal)
        if smp is None:
            return jsonify({'ok': False, 'error': 'клетка почти вне кадра'})
        with state.lock:
            samples = dict(state.cal['samples'])
            samples[color] = samples.get(color, []) + [smp]
            state.cal = dict(state.cal, samples=samples)
            ff.save(state.cal)
        print(f'[FRONT] образец {color}: {smp["f"]}')
        return jsonify({'ok': True})

    @app.route('/api/sample_clear/<color>', methods=['POST'])
    def api_sample_clear(color):
        with state.lock:
            samples = {c: v for c, v in state.cal['samples'].items() if c != color}
            state.cal = dict(state.cal, samples=samples)
            ff.save(state.cal)
        return jsonify({'ok': True})

    return app


def main():
    from ultralytics import YOLO
    state = State()
    try:
        pub = live.LivePublisher(state.snapshot)
    except OSError as e:
        print(f'[FATAL] порт живого потока {live.LIVE_PORT} занят ({e}) - остановите nav.py')
        sys.exit(1)
    if not oled.daemon_running():
        print('  [WARN] демон дисплея не запущен - кот будет только на странице')
    print(f'Модель {YOLO_MODEL_PATH}, камера {CAM_DEV}')
    model = YOLO(YOLO_MODEL_PATH)
    cam = UsbCamera()
    cfg = rc.load_config()
    threading.Thread(target=lidar_loop, args=(state,), daemon=True, name='lidar').start()
    threading.Thread(target=detect_loop, args=(state, cam, model, cfg), daemon=True,
                     name='detect').start()
    threading.Thread(target=oled_loop, args=(state,), daemon=True, name='oled-page').start()
    print(f'Страница: http://{oled._local_ip() or "<IP>"}:{HTTP_PORT}  (Ctrl+C - выход)')
    try:
        make_app(state).run(host='0.0.0.0', port=HTTP_PORT, threaded=True)
    finally:
        pub.close()


if __name__ == '__main__':
    main()
