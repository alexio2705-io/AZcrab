"""Страница /colors: разметка цветов колец вручную и подбор порогов по ней.

Сфоткать цель камерой → выбрать цвет → кликать по кольцам этого цвета. Каждый
клик — образец в ~/maze/color_samples/samples.jsonl: признаки точки после
баланса белого по бумаге (как их видит ring_colors.classify) + метка человека.
«Подобрать пороги» считает пороги по всем образцам и показывает, сколько
образцов они распознают правильно (против текущих); «Применить» ставит их в
работу, «Применить и сохранить» — ещё и в ~/maze/ring_colors.json.

Подключается из victim_web.py: colors_page.register(app, ...).
"""
import base64
import json
import os
import threading
import time

import cv2
from flask import jsonify, request

import ring_colors as rc

SAMPLES_DIR = os.path.expanduser('~/maze/color_samples')
SAMPLES_PATH = os.path.join(SAMPLES_DIR, 'samples.jsonl')
SNAPS_DIR = os.path.join(SAMPLES_DIR, 'snaps')
MAX_EXPOSURE_US = 500_000
IMGSZ = 320

_lock = threading.Lock()
_snaps = {}          # id -> {'frame', 'ctx', 'cam', 'box'}; держим последние несколько
_model = None


def _load_samples():
    try:
        with open(SAMPLES_PATH) as f:
            return [json.loads(line) for line in f if line.strip()]
    except FileNotFoundError:
        return []


def _write_samples(samples):
    os.makedirs(SAMPLES_DIR, exist_ok=True)
    with open(SAMPLES_PATH, 'w') as f:
        for s in samples:
            f.write(json.dumps(s, ensure_ascii=False) + '\n')


def _find_target(frame, model_path):
    """Рамка когнитивной цели на снимке (по ней считается баланс белого) или None."""
    global _model
    from ultralytics import YOLO
    if _model is None:
        _model = YOLO(model_path)
    small = cv2.resize(frame, (IMGSZ, IMGSZ), interpolation=cv2.INTER_AREA)
    boxes = _model.predict(small, imgsz=IMGSZ, conf=0.3, verbose=False)[0].boxes
    k = frame.shape[1] / IMGSZ
    best = None
    for b in boxes:
        if _model.names[int(b.cls[0])] != 'cognetive':
            continue
        if best is None or float(b.conf[0]) > float(best.conf[0]):
            best = b
    return None if best is None else [float(v) * k for v in best.xyxy[0].tolist()]


def register(app, detectors, get_cfg, set_cfg, model_path, log):
    """get_cfg() -> текущие пороги; set_cfg(values: dict, save: bool)."""

    @app.route('/colors')
    def colors_page():
        return PAGE

    @app.route('/api/colors/snap/<int:cam_id>', methods=['POST'])
    def api_colors_snap(cam_id):
        det = detectors.get(cam_id)
        if det is None:
            return jsonify(error='нет такой камеры'), 404
        from dataset_web import STILL_SIZE
        frame, md = det.cam.capture_long(None, None, size=STILL_SIZE, auto_max_us=MAX_EXPOSURE_US)
        shot = f'{md.get("ExposureTime", 0) / 1000:.0f} мс ×{md.get("AnalogueGain", 1):.1f}'
        if frame is None:
            frame, shot = det.cam.latest(), 'обычный кадр'
        with _lock:
            box = _find_target(frame, model_path)
        H, W = frame.shape[:2]
        ctx = rc.prepare(frame, box or [0, 0, W, H], get_cfg())
        sid = time.strftime('%Y%m%d_%H%M%S') + f'_cam{cam_id}'
        os.makedirs(SNAPS_DIR, exist_ok=True)
        cv2.imwrite(os.path.join(SNAPS_DIR, sid + '.png'), frame)
        with _lock:
            _snaps[sid] = {'frame': frame, 'ctx': ctx, 'cam': cam_id, 'box': box}
            for old in sorted(_snaps)[:-6]:
                del _snaps[old]
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        log(f'/colors: снимок {sid} ({shot}), цель {"найдена" if box else "не найдена"}')
        return jsonify(id=sid, w=W, h=H, box=box, shot=shot,
                       img=base64.b64encode(jpg.tobytes()).decode())

    @app.route('/api/colors/sample', methods=['POST'])
    def api_colors_sample():
        d = request.get_json(silent=True) or {}
        label = d.get('label')
        if label not in rc.LABELS:
            return jsonify(error='неизвестный цвет'), 400
        with _lock:
            snap = _snaps.get(d.get('id'))
        if snap is None:
            return jsonify(error='снимок устарел — сфоткай заново'), 400
        feat = rc.sample_features(snap['ctx'], float(d['x']), float(d['y']))
        rec = {**feat, 'label': label, 'snap': d['id'], 'cam': snap['cam'],
               'x': int(d['x']), 'y': int(d['y']), 'ts': time.strftime('%H:%M:%S')}
        with _lock:
            os.makedirs(SAMPLES_DIR, exist_ok=True)
            with open(SAMPLES_PATH, 'a') as f:
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        return jsonify(sample=rec, code_says=rc.classify_sample(feat, get_cfg()))

    @app.route('/api/colors/undo', methods=['POST'])
    def api_colors_undo():
        with _lock:
            samples = _load_samples()
            removed = samples.pop() if samples else None
            _write_samples(samples)
        return jsonify(removed=removed)

    @app.route('/api/colors/clear', methods=['POST'])
    def api_colors_clear():
        with _lock:
            samples = _load_samples()
            if samples:
                # не стираем насовсем — откладываем в сторону
                os.replace(SAMPLES_PATH, SAMPLES_PATH + time.strftime('.%Y%m%d_%H%M%S.bak'))
        log(f'/colors: образцы ({len(samples)}) отложены в .bak')
        return jsonify(ok=True)

    @app.route('/api/colors/samples')
    def api_colors_samples():
        sid = request.args.get('id')
        cfg = get_cfg()
        samples = _load_samples()
        stats = {}
        for c in rc.LABELS:
            ss = [s for s in samples if s['label'] == c]
            if ss:
                med = lambda k: sorted(s[k] for s in ss)[len(ss) // 2]  # noqa: E731
                stats[c] = {'n': len(ss), 'h': med('h'), 's_over': med('s_over'),
                            'v_ratio': med('v_ratio'),
                            'ok': sum(rc.classify_sample(s, cfg) == c for s in ss)}
        points = [{**s, 'code_says': rc.classify_sample(s, cfg)}
                  for s in samples if s.get('snap') == sid]
        return jsonify(stats=stats, points=points, total=len(samples),
                       current=rc.evaluate(samples, cfg))

    @app.route('/api/colors/fit', methods=['POST'])
    def api_colors_fit():
        samples = _load_samples()
        cfg = get_cfg()
        new, rep = rc.fit_thresholds(samples, cfg)
        changed = {k: [cfg[k], new[k]] for k in new if new[k] != cfg[k]}
        return jsonify(values=new, changed=changed, new=rep, current=rc.evaluate(samples, cfg))

    @app.route('/api/colors/apply', methods=['POST'])
    def api_colors_apply():
        d = request.get_json(silent=True) or {}
        set_cfg(d.get('values') or {}, bool(d.get('save')))
        log(f'/colors: пороги применены{" и сохранены" if d.get("save") else ""}')
        return jsonify(ok=True)


PAGE = """<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Разметка цветов - MAZE2027</title>
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
  @media (min-width:1000px) { .layout { grid-template-columns:minmax(0,1fr) 440px; } }
  .colors { display:flex; gap:8px; flex-wrap:wrap; padding:4px 16px 8px; }
  #wrap { position:relative; display:block; line-height:0; background:#000; border-top:1px solid var(--line); text-align:center; }
  #img { display:inline-block; max-width:100%; cursor:crosshair; min-height:200px; vertical-align:top; }
  .pt { position:absolute; width:12px; height:12px; margin:-6px 0 0 -6px; border-radius:50%;
        border:2px solid #fff; box-shadow:0 0 0 1px #000; pointer-events:none; }
  .pt.bad { border-color:var(--acc); }
  #msg { padding:8px 16px 12px; min-height:36px; font-size:12px; color:var(--mut); }
  #shot { color:var(--mut); font-size:12px; }
</style></head>
<body>
<div class="wrap"><div class="bar"><h1>MAZE2027 <span>/ цвета колец</span></h1><nav class="nav"><a href="/">детекция</a><a href="/tune">кольца</a><a href="/colors" aria-current="page">цвета</a></nav><div class="st"><i class="led off" id="conn-led"></i><span id="conn">подключение</span></div></div>
  <div class="panel" style="margin-bottom:16px"><div class="tools">
    <label>Камера <select id="cam"><option value="0">0 · правая</option><option value="1">1 · левая</option></select></label>
    <button class="btn p" id="snap">Сфоткать</button>
    <button class="btn" id="undo">Отменить клик <kbd>Z</kbd></button>
    <span id="shot"></span>
  </div></div>
  <div class="layout">
    <div class="panel">
      <div class="sec"><span>Разметка</span></div>
      <div class="colors" id="colors"></div>
      <div class="hint" style="padding-top:0">Выбери цвет (или клавиши <kbd>1</kbd>–<kbd>6</kbd>), потом кликай по кольцам этого цвета. «Белый» — бумага или стена рядом.</div>
      <div id="wrap"><img id="img" alt="нажми «Сфоткать»"></div>
      <div id="msg"></div>
    </div>
    <div class="stack">
      <div class="panel">
        <div class="sec"><span>Образцы</span><span class="n" id="total"></span></div>
        <div class="tscroll"><table id="stats"></table></div>
        <div class="hint">H — оттенок, S+ — насыщенность сверх бумаги, V/бум — яркость относительно бумаги (медианы). «Распознаётся» — сколько образцов текущие пороги относят к этому цвету.</div>
      </div>
      <div class="panel">
        <div class="sec"><span>Подбор порогов</span></div>
        <div class="pad" style="padding-top:4px"><button class="btn p" id="fit">Подобрать пороги по образцам</button></div>
        <div id="fitres"></div>
        <div class="pad" style="border-top:1px solid var(--line)"><button class="btn" id="clear">Отложить все образцы и начать заново</button></div>
      </div>
    </div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
function esc(s) { return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function setConn(kind, text) { $('conn-led').className = 'led' + (kind === 'on' ? '' : ' ' + kind); $('conn').textContent = text; }

const LABELS = [['black','чёрный','#000'], ['red','красный','#e53935'], ['yellow','жёлтый','#fdd835'],
                ['green','зелёный','#43a047'], ['blue','синий','#1e40ff'], ['white','белый (бумага)','#fff']];
const RU = Object.fromEntries(LABELS.map(l => [l[0], l[1]])), CSS = Object.fromEntries(LABELS.map(l => [l[0], l[2]]));
RU['?'] = 'не понял'; CSS['?'] = '#777';
let label = 'black', snap = null, lastFit = null, fails = 0;

function drawColors() {
  $('colors').innerHTML = '';
  LABELS.forEach(([k, name, css], i) => {
    const b = document.createElement('button');
    b.className = 'btn' + (k === label ? ' on' : '');
    b.innerHTML = `<i class="sw" style="background:${css};margin:0"></i>${i + 1}. ${name}`;
    b.onclick = () => { label = k; drawColors(); };
    $('colors').append(b);
  });
}
async function post(url, body) {
  const r = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body || {})});
  return r.json();
}
function msg(html) { $('msg').innerHTML = html; }

$('snap').onclick = async () => {
  msg('снимаю…'); $('snap').disabled = true;
  try {
    const j = await post('/api/colors/snap/' + $('cam').value);
    if (j.error) { msg(`<span class="bad">${j.error}</span>`); return; }
    snap = j; $('img').src = 'data:image/jpeg;base64,' + j.img;
    $('shot').textContent = `снимок ${j.w}×${j.h}, ${j.shot}` + (j.box ? '' : ' · цель не найдена, баланс белого по всему кадру');
    msg('Кликай по кольцам.'); refresh();
  } finally { $('snap').disabled = false; }
};
$('img').onclick = async e => {
  if (!snap) return;
  const im = $('img'), k = im.naturalWidth / im.clientWidth;
  const j = await post('/api/colors/sample', {id: snap.id, x: e.offsetX * k, y: e.offsetY * k, label});
  if (j.error) { msg(`<span class="bad">${j.error}</span>`); return; }
  const s = j.sample, same = j.code_says === label;
  msg(`${RU[label]}: H=${s.h} S+=${s.s_over} V/бум=${s.v_ratio} · код сейчас видит <b class="${same ? '' : 'bad'}">${RU[j.code_says]}</b>`);
  refresh();
};
$('undo').onclick = async () => { const j = await post('/api/colors/undo'); if (j.removed) msg(`отменён клик: ${RU[j.removed.label]}`); refresh(); };
$('clear').onclick = async () => {
  if (!confirm('Отложить все образцы (файл сохранится как .bak) и начать заново?')) return;
  await post('/api/colors/clear'); refresh();
};
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'SELECT') return;
  const i = parseInt(e.key); if (i >= 1 && i <= 6) { label = LABELS[i - 1][0]; drawColors(); }
  if (e.key === 'z' || e.key === 'я') $('undo').click();
});

async function refresh() {
  let j;
  try { j = await (await fetch('/api/colors/samples?id=' + (snap ? snap.id : ''))).json(); fails = 0; setConn('on', 'онлайн'); }
  catch (e) { if (++fails >= 2) setConn('off', 'нет связи'); return; }
  $('total').textContent = `всего ${j.total} · пороги верны на ${j.current.ok} из ${j.current.n}`;
  let h = '<thead><tr><th>цвет</th><th>образцов</th><th>H</th><th>S+</th><th>V/бум</th><th>распознаётся</th></tr></thead><tbody>';
  for (const [k] of LABELS) {
    const s = j.stats[k], sw = `<i class="sw" style="background:${CSS[k]}"></i>`;
    h += s ? `<tr><td>${sw}${RU[k]}</td><td>${s.n}</td><td>${s.h}</td><td>${s.s_over}</td><td>${s.v_ratio}</td>
      <td class="${s.ok === s.n ? '' : 'bad'}">${s.ok}/${s.n}</td></tr>`
           : `<tr><td>${sw}${RU[k]}</td><td class="mut">0</td><td></td><td></td><td></td><td></td></tr>`;
  }
  $('stats').innerHTML = h + '</tbody>';
  document.querySelectorAll('.pt').forEach(d => d.remove());
  const im = $('img');
  if (snap && im.naturalWidth) {
    const k = im.clientWidth / im.naturalWidth, off = im.offsetLeft;
    for (const p of j.points) {
      const d = document.createElement('div');
      d.className = 'pt' + (p.code_says === p.label ? '' : ' bad');
      d.style.left = (off + p.x * k) + 'px'; d.style.top = p.y * k + 'px'; d.style.background = CSS[p.label];
      d.title = `${RU[p.label]} (код: ${RU[p.code_says]})`;
      $('wrap').append(d);
    }
  }
}
$('img').onload = refresh;

$('fit').onclick = async () => {
  const j = await post('/api/colors/fit'); lastFit = j;
  const err = e => Object.entries(e).map(([k, n]) => k.split('→').map(c => RU[c] || c).join(' → ') + ' ×' + n).join(', ') || 'нет';
  let h = `<div class="r"><span>Сейчас</span><span class="v">${j.current.ok}/${j.current.n}<small>${err(j.current.errors)}</small></span></div>`;
  h += `<div class="r${j.new.ok >= j.current.ok ? '' : ' err'}"><span>С новыми порогами</span><span class="v">${j.new.ok}/${j.new.n}<small>${err(j.new.errors)}</small></span></div>`;
  const ch = Object.entries(j.changed);
  h += ch.length ? '<div class="tscroll"><table><thead><tr><th>порог</th><th>было</th><th>станет</th></tr></thead><tbody>'
      + ch.map(([k, [a, b]]) => `<tr><td>${k}</td><td>${a}</td><td>${b}</td></tr>`).join('') + '</tbody></table></div>'
      : '<div class="hint" style="border-top:1px solid var(--line)">Пороги не меняются.</div>';
  if (ch.length) h += `<div class="pad"><button class="btn p" id="apply">Применить</button> <button class="btn" id="applysave">Применить и сохранить</button></div>
    <div class="hint" style="padding-top:0">«Применить» — сразу в работу (детекция, кольца). «Сохранить» — ещё и в ~/maze/ring_colors.json для заездов.</div>`;
  $('fitres').innerHTML = h;
  if (ch.length) {
    $('apply').onclick = async () => { await post('/api/colors/apply', {values: lastFit.values}); msg('пороги применены'); refresh(); };
    $('applysave').onclick = async () => { await post('/api/colors/apply', {values: lastFit.values, save: true}); msg('пороги применены и сохранены'); refresh(); };
  }
};
window.addEventListener('resize', refresh);
drawColors(); refresh();
</script>
</body></html>
"""
