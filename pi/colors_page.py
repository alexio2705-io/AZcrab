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
<title>Разметка цветов</title>
<style>
  body { margin: 0; font-family: sans-serif; background: #1b1b1b; color: #eee; }
  header { padding: 10px 16px; background: #262626; display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
  header a { color: #8ab4f8; }
  button, select { font-size: 15px; padding: 7px 12px; border-radius: 4px; border: 0; cursor: pointer; }
  .primary { background: #2e7d32; color: #fff; }
  main { display: flex; gap: 16px; padding: 16px; flex-wrap: wrap; align-items: flex-start; }
  .panel { background: #262626; border-radius: 8px; padding: 12px; }
  #wrap { position: relative; display: inline-block; line-height: 0; }
  #img { max-width: min(90vw, 760px); cursor: crosshair; background: #000; min-width: 300px; min-height: 300px; }
  .dot { position: absolute; width: 12px; height: 12px; margin: -6px 0 0 -6px; border-radius: 50%;
         border: 2px solid #fff; box-shadow: 0 0 0 1px #000; pointer-events: none; }
  .dot.bad { border-color: #f33; }
  .colors { display: flex; gap: 6px; flex-wrap: wrap; margin-bottom: 10px; }
  .cbtn { display: flex; align-items: center; gap: 6px; background: #333; color: #eee; }
  .cbtn.on { outline: 3px solid #8ab4f8; }
  .sw { width: 16px; height: 16px; border-radius: 50%; border: 1px solid #888; display: inline-block; }
  table { border-collapse: collapse; font-size: 14px; }
  td, th { padding: 4px 8px; text-align: right; } th { color: #aaa; font-weight: normal; }
  td:first-child, th:first-child { text-align: left; }
  #msg { min-height: 1.3em; margin: 8px 0; }
  .bad { color: #f66; } .good { color: #6f6; }
  .muted { color: #999; font-size: 13px; }
  kbd { background: #444; padding: 1px 5px; border-radius: 3px; font-size: 12px; }
</style></head><body>
<header>
  <a href="/">← детекция</a> <a href="/tune">настройка →</a>
  <label>Камера <select id="cam"><option value="0">0 (правая)</option><option value="1">1 (левая)</option></select></label>
  <button class="primary" id="snap">Сфоткать</button>
  <button id="undo">Отменить последний клик <kbd>Z</kbd></button>
  <span id="shot" class="muted"></span>
</header>
<main>
  <div class="panel">
    <div class="colors" id="colors"></div>
    <div class="muted" style="margin-bottom:8px">выбери цвет (или клавиши <kbd>1</kbd>–<kbd>6</kbd>), потом кликай по кольцам этого цвета; «белый» — бумага/стена рядом</div>
    <div id="wrap"><img id="img" alt="нажми «Сфоткать»"></div>
    <div id="msg"></div>
  </div>
  <div class="panel" style="min-width:420px">
    <b>Образцы</b> <span class="muted" id="total"></span>
    <table id="stats" style="margin-top:6px"></table>
    <div class="muted" style="margin:6px 0 12px">H — оттенок, S+ — насыщенность сверх бумаги, V/бум — яркость относительно бумаги (медианы); «распознаётся» — сколько образцов текущие пороги относят к этому цвету</div>
    <button class="primary" id="fit">Подобрать пороги по образцам</button>
    <div id="fitres" style="margin-top:10px"></div>
    <div style="margin-top:16px"><button id="clear">Отложить все образцы и начать заново</button></div>
  </div>
</main>
<script>
const LABELS = [['black','чёрный','#000'], ['red','красный','#e53935'], ['yellow','жёлтый','#fdd835'],
                ['green','зелёный','#43a047'], ['blue','синий','#1e40ff'], ['white','белый (бумага)','#fff']];
const RU = Object.fromEntries(LABELS.map(l => [l[0], l[1]])), CSS = Object.fromEntries(LABELS.map(l => [l[0], l[2]]));
RU['?'] = 'не понял'; CSS['?'] = '#777';
const $ = id => document.getElementById(id);
let label = 'black', snap = null, lastFit = null;

function drawColors() {
  $('colors').innerHTML = '';
  LABELS.forEach(([k, name, css], i) => {
    const b = document.createElement('button');
    b.className = 'cbtn' + (k === label ? ' on' : '');
    b.innerHTML = `<span class="sw" style="background:${css}"></span>${i + 1}. ${name}`;
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
    $('shot').textContent = `снимок ${j.w}×${j.h}, ${j.shot}` + (j.box ? '' : ' · цель не найдена — баланс белого по всему кадру');
    msg('кликай по кольцам'); refresh();
  } finally { $('snap').disabled = false; }
};
$('img').onclick = async e => {
  if (!snap) return;
  const im = $('img'), k = im.naturalWidth / im.clientWidth;
  const j = await post('/api/colors/sample', {id: snap.id, x: e.offsetX * k, y: e.offsetY * k, label});
  if (j.error) { msg(`<span class="bad">${j.error}</span>`); return; }
  const s = j.sample, same = j.code_says === label;
  msg(`${RU[label]}: H=${s.h} S+=${s.s_over} V/бум=${s.v_ratio} — код сейчас видит <b class="${same ? 'good' : 'bad'}">${RU[j.code_says]}</b>`);
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
  const j = await (await fetch('/api/colors/samples?id=' + (snap ? snap.id : ''))).json();
  $('total').textContent = `всего ${j.total}; текущие пороги верны на ${j.current.ok} из ${j.current.n}`;
  let h = '<tr><th>цвет</th><th>образцов</th><th>H</th><th>S+</th><th>V/бум</th><th>распознаётся</th></tr>';
  for (const [k] of LABELS) {
    const s = j.stats[k];
    h += s ? `<tr><td><span class="sw" style="background:${CSS[k]}"></span> ${RU[k]}</td><td>${s.n}</td><td>${s.h}</td><td>${s.s_over}</td><td>${s.v_ratio}</td>
      <td class="${s.ok === s.n ? 'good' : 'bad'}">${s.ok}/${s.n}</td></tr>`
           : `<tr><td><span class="sw" style="background:${CSS[k]}"></span> ${RU[k]}</td><td class="muted">0</td><td></td><td></td><td></td><td></td></tr>`;
  }
  $('stats').innerHTML = h;
  // точки этого снимка
  document.querySelectorAll('.dot').forEach(d => d.remove());
  const im = $('img');
  if (snap && im.naturalWidth) {
    const k = im.clientWidth / im.naturalWidth;
    for (const p of j.points) {
      const d = document.createElement('div');
      d.className = 'dot' + (p.code_says === p.label ? '' : ' bad');
      d.style.left = p.x * k + 'px'; d.style.top = p.y * k + 'px'; d.style.background = CSS[p.label];
      d.title = `${RU[p.label]} (код: ${RU[p.code_says]})`;
      $('wrap').append(d);
    }
  }
}
$('img').onload = refresh;

$('fit').onclick = async () => {
  const j = await post('/api/colors/fit'); lastFit = j;
  const err = e => Object.entries(e).map(([k, n]) => k.split('→').map(c => RU[c] || c).join(' → ') + ' ×' + n).join(', ') || 'нет';
  let h = `<div>сейчас: <b>${j.current.ok}/${j.current.n}</b> верно; ошибки: ${err(j.current.errors)}</div>`;
  h += `<div>с новыми порогами: <b class="${j.new.ok >= j.current.ok ? 'good' : 'bad'}">${j.new.ok}/${j.new.n}</b> верно; ошибки: ${err(j.new.errors)}</div>`;
  const ch = Object.entries(j.changed);
  h += ch.length ? '<table style="margin:8px 0"><tr><th>порог</th><th>было</th><th>станет</th></tr>'
      + ch.map(([k, [a, b]]) => `<tr><td>${k}</td><td>${a}</td><td>${b}</td></tr>`).join('') + '</table>'
      : '<div class="muted">пороги не меняются</div>';
  if (ch.length) h += `<button class="primary" id="apply">Применить</button> <button id="applysave">Применить и сохранить</button>
    <div class="muted" style="margin-top:6px">«Применить» — сразу в работу (детекция, /tune); «сохранить» — ещё и в ~/maze/ring_colors.json для заездов</div>`;
  $('fitres').innerHTML = h;
  if (ch.length) {
    $('apply').onclick = async () => { await post('/api/colors/apply', {values: lastFit.values}); msg('<span class="good">пороги применены</span>'); refresh(); };
    $('applysave').onclick = async () => { await post('/api/colors/apply', {values: lastFit.values, save: true}); msg('<span class="good">пороги применены и сохранены</span>'); refresh(); };
  }
};
window.addEventListener('resize', refresh);
drawColors(); refresh();
</script></body></html>
"""
