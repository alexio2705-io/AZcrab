#!/usr/bin/env python3
"""Веб-страница карты лабиринта и найденных меток.

Слева - карта (стены, посещённые/непроезжие клетки, робот, чекпоинт),
справа - список считанных меток: тип с цветной обводкой (как цвет LED из
victim_detect.py: damaged=красный, stable=жёлтый, unharmed=зелёный), а для
когнитивной цели (cognetive) ещё и цвета колец с чексуммой.

Данные не читаются напрямую из nav.py (это отдельный процесс с мотор-петлёй
и лидаром на 200Гц - лишний Flask-сервер внутри него только мешал бы) - nav.py
сам пишет маленький JSON-снимок в STATE_PATH при каждом значимом событии
(приезд в клетку, находка метки), а эта страница его просто читает и
раздаёт браузеру поллингом. Тот же приём, что у camera_web/app.py и
detect_app.py, только без потока с камерой.

Запуск (одновременно с nav.py, в отдельном терминале):
    ~/maze/venv/bin/python ~/maze/map_web.py
Страница:  http://<IP робота>:8001/
"""
from __future__ import annotations

import os

from flask import Flask, Response, jsonify, send_from_directory

from victim_detect import SAVE_DIR as DETECTIONS_DIR

HTTP_PORT = 8001
STATE_PATH = os.path.expanduser("~/maze/state.json")

EMPTY_STATE = {
    "ts": None, "phase": "waiting",
    "robot": {"cx": 0, "cy": 0, "hdg": 0, "dir": "N"},
    "checkpoint": {"cx": 0, "cy": 0, "hdg": 0},
    "maze": {}, "visited": [], "blocked": [], "blue_done": [], "victims": [],
}

app = Flask(__name__)


@app.route("/")
def index():
    return Response(INDEX_HTML, mimetype="text/html")


@app.route("/state.json")
def state():
    try:
        with open(STATE_PATH, "rb") as f:
            # Файл уже валидный JSON (пишется атомарно в nav.py._write_state) -
            # отдаём как есть, не гоняя через json.loads/dumps лишний раз.
            return Response(f.read(), mimetype="application/json")
    except (OSError, ValueError):
        return jsonify(EMPTY_STATE)


@app.route("/detections/<path:fname>")
def detection_image(fname):
    return send_from_directory(DETECTIONS_DIR, fname)


INDEX_HTML = """<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Карта лабиринта — MAZE2027</title>
<style>
  :root {
    --bg:#0c0e12; --panel:#181b21; --panel-2:#20242c; --border:#2a2f38;
    --text:#eef1f6; --dim:#8892a0; --accent:#5b9dff;
    --damaged:#ff5c5c; --stable:#ffcf4d; --unharmed:#4ee27a; --fake:#6b7280;
  }
  * { box-sizing: border-box; }
  @keyframes pulse {
    0%   { transform:scale(.7); opacity:.9; }
    70%  { transform:scale(1.8); opacity:0; }
    100% { transform:scale(1.8); opacity:0; }
  }
  body {
    background:
      radial-gradient(1200px 600px at 15% -10%, #1b2a4499 0%, transparent 60%),
      radial-gradient(900px 500px at 100% 0%, #1a1f3355 0%, transparent 55%),
      var(--bg);
    background-attachment:fixed;
    color:var(--text); margin:0; padding:28px;
    font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
    -webkit-font-smoothing:antialiased;
  }
  h1 {
    font-size:21px; margin:0 0 4px; font-weight:800; letter-spacing:-.01em;
    display:flex; align-items:center; gap:10px;
  }
  h1 .live {
    position:relative; width:9px; height:9px; border-radius:50%;
    background:#4ee27a; box-shadow:0 0 8px #4ee27a99; flex:none;
  }
  h1 .live::after {
    content:""; position:absolute; inset:0; border-radius:50%; background:#4ee27a;
    animation:pulse 1.8s ease-out infinite;
  }
  .subtitle { color:var(--dim); font-size:12.5px; margin:0 0 18px; }

  #status-bar { display:flex; flex-wrap:wrap; gap:8px; margin-bottom:22px; }
  .chip {
    display:inline-flex; align-items:center; gap:6px;
    background:#20242cb3; backdrop-filter:blur(10px); -webkit-backdrop-filter:blur(10px);
    border:1px solid var(--border);
    border-radius:999px; padding:6px 13px; font-size:12.5px; color:var(--dim);
    box-shadow:inset 0 1px 0 #ffffff0a;
  }
  .chip b { color:var(--text); font-weight:600; }
  .chip.phase {
    background:linear-gradient(135deg, var(--accent), #7b6bff); color:#fff; border:none;
    box-shadow:0 4px 14px -4px #5b9dff66;
  }
  .chip.phase b { color:#fff; text-transform:uppercase; letter-spacing:.04em; font-weight:700; }
  .chip .mono { font-variant-numeric:tabular-nums; font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }

  .layout { display:flex; gap:20px; align-items:flex-start; flex-wrap:wrap; }
  #map-col { flex: 1 1 480px; min-width:320px; }
  #victims-col { flex: 1 1 560px; min-width:300px; }

  #map-wrap {
    background:linear-gradient(180deg, #191d24ee, #14171cee);
    backdrop-filter:blur(14px); -webkit-backdrop-filter:blur(14px);
    border:1px solid var(--border); border-radius:16px;
    padding:18px; overflow:auto;
    box-shadow:0 12px 32px -12px #000000aa, inset 0 1px 0 #ffffff0d;
  }
  svg#map { display:block; }

  .legend { display:flex; flex-wrap:wrap; gap:8px 10px; margin-top:16px; }
  .legend span {
    display:inline-flex; align-items:center; gap:6px; font-size:12px; color:var(--dim);
    background:#20242cb3; border:1px solid var(--border); border-radius:999px; padding:4px 10px;
    transition:border-color .15s ease, color .15s ease;
  }
  .legend span:hover { border-color:#3a4150; color:var(--text); }
  .swatch { width:11px; height:11px; border-radius:3px; display:inline-block; flex:none; }
  .swatch.dot { border-radius:50%; }

  h2 {
    font-size:13px; margin:0 0 12px; color:var(--dim); font-weight:600;
    text-transform:uppercase; letter-spacing:.08em;
  }
  h2 .count {
    display:inline-block; margin-left:6px; background:var(--panel-2); color:var(--text);
    border-radius:999px; padding:1px 8px; font-size:11px; font-weight:700;
  }

  #victims { display:flex; flex-direction:column; gap:10px; max-height:80vh; overflow-y:auto; padding-right:2px; }
  .vcard {
    background:linear-gradient(180deg, #191d24ee, #14171cee);
    backdrop-filter:blur(14px); -webkit-backdrop-filter:blur(14px);
    border:1px solid var(--border); border-radius:12px;
    padding:12px; display:flex; flex-direction:column; gap:10px; position:relative; overflow:hidden;
    flex:none;   /* список ограничен по высоте - без этого карточки сплющивались */
    box-shadow:inset 0 1px 0 #ffffff0a;
    transition:transform .15s ease, border-color .15s ease, box-shadow .15s ease;
  }
  .vcard::before {
    content:""; position:absolute; inset:0 auto 0 0; width:4px; background:var(--fake);
  }
  .vcard.damaged::before  { background:var(--damaged); box-shadow:0 0 10px 0 var(--damaged); }
  .vcard.stable::before   { background:var(--stable); box-shadow:0 0 10px 0 var(--stable); }
  .vcard.unharmed::before { background:var(--unharmed); box-shadow:0 0 10px 0 var(--unharmed); }
  .vcard.fake { opacity:.55; }
  .vcard:hover { transform:translateX(3px); border-color:#3a4150; box-shadow:0 8px 20px -10px #000000cc, inset 0 1px 0 #ffffff0a; }
  /* Снимок целиком и крупно: метка на кадре бывает в углу, и миниатюра с
     обрезкой (cover) её просто отрезала. Клик - оригинал в новой вкладке. */
  .vcard a.shot { display:block; }
  .vcard img { display:block; width:100%; max-height:70vh; object-fit:contain; border-radius:8px; background:#000; border:1px solid var(--border); cursor:zoom-in; }
  .vcard .body { flex:1; min-width:0; }
  .vcard .title { font-weight:700; font-size:18px; display:flex; align-items:center; gap:6px; }
  .vcard .title .glyph {
    display:inline-flex; align-items:center; justify-content:center;
    width:22px; height:22px; border-radius:50%; font-size:13px; font-weight:800; flex:none;
    background:var(--panel-2);
  }
  .vcard.damaged  .glyph { color:var(--damaged); }
  .vcard.stable   .glyph { color:var(--stable); }
  .vcard.unharmed .glyph { color:var(--unharmed); }
  .vcard .meta { color:var(--dim); font-size:13.5px; margin-top:5px; line-height:1.5; }
  .rings { display:flex; gap:5px; margin-top:8px; }
  .ring { width:22px; height:22px; border-radius:50%; border:1.5px solid #00000066; box-shadow:0 0 0 1px #ffffff1a inset; }
  .ring[data-c="black"]  { background:#000; }
  .ring[data-c="red"]    { background:var(--damaged); }
  .ring[data-c="yellow"] { background:var(--stable); }
  .ring[data-c="green"]  { background:var(--unharmed); }
  .ring[data-c="blue"]   { background:#3f8bff; }
  .ring[data-c="?"]      { background:#555; }

  #empty {
    color:var(--dim); font-size:13px; text-align:center; padding:32px 16px;
    border:1px dashed var(--border); border-radius:12px;
  }
  #empty svg { display:block; margin:0 auto 10px; opacity:.5; }

  #victims::-webkit-scrollbar { width:8px; }
  #victims::-webkit-scrollbar-thumb { background:var(--border); border-radius:99px; }
</style>
</head>
<body>
  <h1><span class="live"></span>Карта лабиринта — MAZE2027</h1>
  <p class="subtitle">живой снимок state.json · опрос каждые 0.25 с</p>
  <div id="status-bar">
    <span class="chip phase">фаза <b id="phase">—</b></span>
    <span class="chip">позиция <b class="mono" id="pos">?,?</b></span>
    <span class="chip">курс <b id="dir">?</b></span>
    <span class="chip">обновлено <b class="mono" id="ts">—</b></span>
  </div>

  <div class="layout">
    <div id="map-col">
      <div id="map-wrap"><svg id="map" xmlns="http://www.w3.org/2000/svg"></svg></div>
      <div class="legend">
        <span><i class="swatch" style="background:#2a3a2a"></i> посещено</span>
        <span><i class="swatch" style="background:#5b8fff33;border:1px solid #5b8fff"></i> синяя (стоянка)</span>
        <span><i class="swatch" style="background:#ff555533;border:1px solid #ff5555"></i> чекпоинт</span>
        <span><i class="swatch" style="background:repeating-linear-gradient(45deg,#772222,#772222 3px,#331111 3px,#331111 6px)"></i> непроезжая клетка</span>
        <span><i class="swatch dot" style="background:#ffd23f"></i> робот</span>
        <span><i class="swatch dot" style="background:var(--damaged)"></i> Φ повреждён</span>
        <span><i class="swatch dot" style="background:var(--stable)"></i> Ψ стабилен</span>
        <span><i class="swatch dot" style="background:var(--unharmed)"></i> Ω не повреждён</span>
      </div>
    </div>

    <div id="victims-col">
      <h2>Найденные метки<span class="count" id="vcount">0</span></h2>
      <div id="victims"></div>
      <div id="empty">
        <svg width="40" height="40" viewBox="0 0 40 40" fill="none">
          <circle cx="20" cy="20" r="18" stroke="#8892a0" stroke-width="1.5" stroke-dasharray="3 4"/>
          <circle cx="20" cy="20" r="11" stroke="#8892a0" stroke-width="1.5" stroke-dasharray="3 4"/>
          <circle cx="20" cy="20" r="2.5" fill="#8892a0"/>
        </svg>
        пока ничего не найдено
      </div>
    </div>
  </div>

<script>
const CELL = 56;   // px на клетку карты
const NAMES = {damaged: 'ПОВРЕЖДЁН', stable: 'СТАБИЛЕН', unharmed: 'НЕ ПОВРЕЖДЁН', fake: 'ФЕЙК'};
const GLYPH = {damaged: 'Φ', stable: 'Ψ', unharmed: 'Ω'};
const DIR_DX = [0, 1, 0, -1], DIR_DY = [1, 0, -1, 0];   // N,E,S,W - как в nav.py

function svgEl(tag, attrs) {
  const el = document.createElementNS('http://www.w3.org/2000/svg', tag);
  for (const k in attrs) el.setAttribute(k, attrs[k]);
  return el;
}

function drawMap(s) {
  const svg = document.getElementById('map');
  svg.innerHTML = '';
  const keys = Object.keys(s.maze || {});
  const cells = keys.map(k => { const [x, y] = k.split(',').map(Number); return {x, y, w: s.maze[k]}; });
  const robot = s.robot || {cx: 0, cy: 0, hdg: 0};
  const known = cells.length ? cells : [{x: robot.cx, y: robot.cy, w: [null, null, null, null]}];

  // Этажи (горки): nav.py хранит этаж в y со сдвигом level_stride*z. На
  // экране кладём этажи рядом слева направо, с промежутком в 2 клетки -
  // иначе этаж 1 рисовался бы в тысяче клеток от этажа 0.
  const LS = s.level_stride || 1000;
  const lvl = y => Math.floor((y + LS / 2) / LS);
  const xs = known.map(c => c.x).concat([robot.cx]);
  const spanX = Math.max(...xs) - Math.min(...xs) + 3;
  const vx = (x, y) => x + lvl(y) * spanX;
  const vy = y => y - lvl(y) * LS;

  let minX = Math.min(...known.map(c => vx(c.x, c.y)), vx(robot.cx, robot.cy));
  let maxX = Math.max(...known.map(c => vx(c.x, c.y)), vx(robot.cx, robot.cy));
  let minY = Math.min(...known.map(c => vy(c.y)), vy(robot.cy));
  let maxY = Math.max(...known.map(c => vy(c.y)), vy(robot.cy));

  const W = (maxX - minX + 1) * CELL, H = (maxY - minY + 1) * CELL;
  svg.setAttribute('width', W + 4);
  svg.setAttribute('height', H + 4);
  svg.setAttribute('viewBox', `-2 -2 ${W + 4} ${H + 4}`);

  const sx = x => (x - minX) * CELL;
  const sy = y => (maxY - y) * CELL;   // север (y растёт) - вверх экрана

  const visited = new Set((s.visited || []).map(p => p[0] + ',' + p[1]));
  const blocked = new Set((s.blocked || []).map(p => p[0] + ',' + p[1]));
  const blueDone = new Set((s.blue_done || []).map(p => p[0] + ',' + p[1]));
  const cp = s.checkpoint || {cx: 0, cy: 0};

  // Общие defs: точечная сетка фона и мягкое свечение для меток/робота.
  const defs = svgEl('defs', {});
  const dotgrid = svgEl('pattern', {id: 'dotgrid', width: CELL, height: CELL, patternUnits: 'userSpaceOnUse'});
  dotgrid.appendChild(svgEl('circle', {cx: 1, cy: 1, r: 1, fill: '#242933'}));
  defs.appendChild(dotgrid);
  const glow = svgEl('filter', {id: 'glow', x: '-80%', y: '-80%', width: '260%', height: '260%'});
  glow.appendChild(svgEl('feGaussianBlur', {stdDeviation: 3.5, result: 'blur'}));
  const merge = svgEl('feMerge', {});
  merge.appendChild(svgEl('feMergeNode', {in: 'blur'}));
  merge.appendChild(svgEl('feMergeNode', {in: 'SourceGraphic'}));
  glow.appendChild(merge);
  defs.appendChild(glow);
  svg.appendChild(defs);
  svg.appendChild(svgEl('rect', {x: -2, y: -2, width: W + 4, height: H + 4, fill: 'url(#dotgrid)'}));

  for (const c of cells) {
    const key = c.x + ',' + c.y;
    const x = sx(vx(c.x, c.y)), y = sy(vy(c.y));
    let fill = 'transparent';
    if (blocked.has(key)) fill = 'url(#hatch)';
    else if (blueDone.has(key)) fill = '#5b8fff33';
    else if (cp.cx === c.x && cp.cy === c.y) fill = '#ff555533';
    else if (visited.has(key)) fill = '#2a3a2a';
    svg.appendChild(svgEl('rect', {x, y, width: CELL, height: CELL, fill,
                                    stroke: '#2a2a2a', 'stroke-width': 1}));
    // Стены: [N,E,S,W]. true - рисуем толстую линию, null (неизвестно) -
    // не рисуем вовсе (это чужая, ещё не подтверждённая сторона соседа).
    const edges = [
      c.w[0] === true ? [x, y, x + CELL, y] : null,               // N
      c.w[1] === true ? [x + CELL, y, x + CELL, y + CELL] : null, // E
      c.w[2] === true ? [x, y + CELL, x + CELL, y + CELL] : null, // S
      c.w[3] === true ? [x, y, x, y + CELL] : null,                // W
    ];
    for (const e of edges) {
      if (!e) continue;
      svg.appendChild(svgEl('line', {x1: e[0], y1: e[1], x2: e[2], y2: e[3],
                                      stroke: '#eee', 'stroke-width': 3, 'stroke-linecap': 'round'}));
    }
  }

  // Диагональная штриховка для непроезжих клеток. svg.innerHTML='' выше
  // чистит всё на каждый опрос, поэтому паттерн приходится пересоздавать
  // тут же - но rect'ы, ссылающиеся на url(#hatch), это переживают: SVG
  // резолвит ссылку по живому DOM, а не по порядку добавления узлов.
  const pat = svgEl('pattern', {id: 'hatch', width: 8, height: 8, patternUnits: 'userSpaceOnUse',
                                 patternTransform: 'rotate(45)'});
  pat.appendChild(svgEl('rect', {width: 8, height: 8, fill: '#331111'}));
  pat.appendChild(svgEl('rect', {width: 3, height: 8, fill: '#772222'}));
  defs.appendChild(pat);

  // Метки на карте - точка со свечением и глифом у той стены, где их нашли.
  for (const v of (s.victims || [])) {
    const [vcx, vcy] = v.cell;
    const d = v.wall_dir ?? 0;
    const cx0 = sx(vx(vcx, vcy)) + CELL / 2, cy0 = sy(vy(vcy)) + CELL / 2;
    const px = cx0 + DIR_DX[d] * CELL * 0.32, py = cy0 - DIR_DY[d] * CELL * 0.32;
    const color = {damaged: '#ff5c5c', stable: '#ffcf4d', unharmed: '#4ee27a'}[v.type] || '#888';
    const glyph = {damaged: 'Φ', stable: 'Ψ', unharmed: 'Ω'}[v.type] || '?';
    // мягкое свечение под меткой, чтобы она не терялась на фоне стен
    svg.appendChild(svgEl('circle', {cx: px, cy: py, r: 9, fill: color, opacity: 0.55, filter: 'url(#glow)'}));
    svg.appendChild(svgEl('circle', {cx: px, cy: py, r: 8.5, fill: color, stroke: '#0c0e12', 'stroke-width': 2}));
    const label = svgEl('text', {x: px, y: py, 'text-anchor': 'middle', 'dominant-baseline': 'central',
                                  'font-size': 10, 'font-weight': 800, fill: '#0c0e12',
                                  'font-family': 'ui-monospace,SFMono-Regular,Menlo,monospace'});
    label.textContent = glyph;
    svg.appendChild(label);
  }

  // Горки - пунктир от клетки у подножия до клетки за гребнем (на другом
  // этаже) с числом плиток.
  for (const [ax, ay, , bx, by, n] of (s.ramps || [])) {
    const x1 = sx(vx(ax, ay)) + CELL / 2, y1 = sy(vy(ay)) + CELL / 2;
    const x2 = sx(vx(bx, by)) + CELL / 2, y2 = sy(vy(by)) + CELL / 2;
    svg.appendChild(svgEl('line', {x1, y1, x2, y2, stroke: '#ffcf4d', 'stroke-width': 2,
                                    'stroke-dasharray': '6 5', opacity: 0.8}));
    const t = svgEl('text', {x: (x1 + x2) / 2, y: (y1 + y2) / 2 - 6, 'text-anchor': 'middle',
                              'font-size': 11, fill: '#ffcf4d'});
    t.textContent = `⛰ ${n}`;
    svg.appendChild(t);
  }

  // Робот - треугольник со свечением, острие по курсу (0=N вверх, 1=E вправо, ...).
  const rcx = sx(vx(robot.cx, robot.cy)) + CELL / 2, rcy = sy(vy(robot.cy)) + CELL / 2, r = CELL * 0.3;
  const points = `${rcx},${rcy - r} ${rcx - r * 0.7},${rcy + r * 0.7} ${rcx + r * 0.7},${rcy + r * 0.7}`;
  const transform = `rotate(${robot.hdg * 90} ${rcx} ${rcy})`;
  svg.appendChild(svgEl('polygon', {points, fill: '#ffd23f', opacity: 0.5, filter: 'url(#glow)', transform}));
  svg.appendChild(svgEl('polygon', {points, fill: '#ffd23f', stroke: '#0c0e12', 'stroke-width': 1.5, transform}));
}

function drawVictims(list) {
  const box = document.getElementById('victims');
  const empty = document.getElementById('empty');
  document.getElementById('vcount').textContent = list.length;
  box.innerHTML = '';
  empty.style.display = list.length ? 'none' : 'block';
  // Свежие сверху.
  for (const v of [...list].reverse()) {
    const card = document.createElement('div');
    card.className = 'vcard ' + (v.type || '');
    const src = v.image ? `/detections/${encodeURIComponent(v.image)}` : '';
    const img = src ? `<a class="shot" href="${src}" target="_blank"><img src="${src}" loading="lazy"></a>` : '';
    const rings = (v.rings || []).map(c => `<span class="ring" data-c="${c}"></span>`).join('');
    card.innerHTML = `
      ${img}
      <div class="body">
        <div class="title"><span class="glyph">${GLYPH[v.type] || '?'}</span>${NAMES[v.type] || v.type || '?'}</div>
        <div class="meta">клетка (${v.cell[0]},${v.cell[1]}) · сторона ${v.side === 'right' ? 'право' : 'лево'}
          · класс ${v.class} · conf ${v.conf} · ${v.ts || ''}</div>
        ${v.class === 'cognetive' ? `<div class="rings">${rings}<span style="margin-left:6px">сумма ${v.checksum ?? '?'}</span></div>` : ''}
      </div>`;
    box.appendChild(card);
  }
}

async function poll() {
  try {
    const r = await fetch('/state.json', {cache: 'no-store'});
    const s = await r.json();
    document.getElementById('phase').textContent = s.phase || '—';
    document.getElementById('pos').textContent = `${s.robot.cx},${s.robot.cy}`;
    document.getElementById('dir').textContent = s.robot.dir || '?';
    document.getElementById('ts').textContent = s.ts || '—';
    drawMap(s);
    drawVictims(s.victims || []);
  } catch (e) { /* сеть/JSON моргнули - подождём следующего опроса */ }
  setTimeout(poll, 250);
}
poll();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
