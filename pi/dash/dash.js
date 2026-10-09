/* Пульт AZcrab - живой экран заезда и повтор записей.
   Данные: /events (Server-Sent Events, 10 раз в секунду) от dash_web.py;
   повтор - /runs/<файл> (строка JSON на снимок). */
'use strict';

const $ = id => document.getElementById(id);
const el = (tag, cls, html) => { const e = document.createElement(tag); if (cls) e.className = cls; if (html != null) e.innerHTML = html; return e; };
const svgEl = (tag, attrs) => { const e = document.createElementNS('http://www.w3.org/2000/svg', tag); for (const k in attrs) e.setAttribute(k, attrs[k]); return e; };
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt = (v, d = 0, unit = '') => v == null || Number.isNaN(v) ? '—' : (+v).toFixed(d) + unit;

const RUN_LIMIT_S = 480;            // заезд RCJ Maze - 8 минут
const WINDOW_S = 30;                // окно графиков
const HIST_KEEP_S = 60;
const LOG_MAX = 3000;
// Порядок как в dash_web.hist_row
const H = {t: 0, spd: 1, pitch: 2, roll: 3, rate: 4, front: 5, left: 6, right: 7, steer: 8, odo: 9, vel: 10};   // vel - считается здесь (withVel)

const PHASE = {
  boot: ['запуск', 'off'], waiting: ['ожидание', 'off'], exploring: ['разведка', 'on'],
  sleep: ['сон', 'att'], explored: ['разведка завершена', 'on'], cap: ['лимит клеток', 'att'],
  returning: ['возврат на старт', 'on'], at_start: ['на старте', 'on'], blinking: ['финиш', 'att'],
  done: ['готово', 'att'], fatal: ['авария', 'err'],
  wait_motors: ['моторы выключены — жду', 'err'], wait_link: ['нет связи с ESP — жду', 'err'],
  wait_lidar: ['лидар молчит — жду', 'err'],
};
// Что делать человеку, пока робот ждёт - крупно в строке статуса.
const WAIT_HINT = {wait_motors: 'Включите питание моторов - заезд продолжится сам',
                   wait_link: 'Проверьте ESP и провод к Pi - заезд продолжится сам',
                   wait_lidar: 'Проверьте кабель и питание лидара'};
const MODE = {ramp: 'горка', kick: 'толчок через препятствие'};
const VNAME = {damaged: 'Повреждён', stable: 'Стабилен', unharmed: 'Не повреждён', fake: 'Фейк', cognetive: 'Когнитивная цель'};
const GLYPH = {damaged: 'Φ', stable: 'Ψ', unharmed: 'Ω', cognetive: '◎'};
const VCOLOR = {damaged: '--damaged', stable: '--stable', unharmed: '--unharmed', cognetive: '--cog'};
const TILE = {white: ['белая', '#e9e9e4'], black: ['чёрная', '#050505'], blue: ['синяя', '#3987e5'],
              red: ['красная', '#e5534b'], silver: ['серебро', '#b9c2cc'], '?': ['не ясно', 'transparent']};
const DIRS = 'NESW';

// ── состояние ────────────────────────────────────────────
let S = null, MAP = null, LINK = null;
let HIST = [];
const LOG = [];
let mode = 'live';                  // 'live' | 'replay'
let es = null;
let dirty = false;
let mapKey = null;
let WORLD = null;                   // правда модели (записи ночного прогона)

class Rate {                        // частота счётчика по времени снимков
  constructor() { this.q = []; }
  push(t, n) {
    if (t == null || n == null) return;
    const q = this.q;
    if (q.length && (n < q[q.length - 1][1] || t < q[q.length - 1][0])) q.length = 0;   // перезапуск
    q.push([t, n]);
    while (q.length > 2 && t - q[0][0] > 2) q.shift();
  }
  hz() { const q = this.q; if (q.length < 2) return null; const dt = q[q.length - 1][0] - q[0][0]; return dt > 0 ? (q[q.length - 1][1] - q[0][1]) / dt : null; }
  clear() { this.q.length = 0; }
}
const rates = {telem: new Rate(), scan: new Rate(), camL: new Rate(), camR: new Rate()};

function histRow(s) {
  const imu = s.imu || {}, w = s.walls || {}, d = s.drive || {}, o = s.odo || [];
  const om = o[0] == null || o[1] == null ? null : (o[0] + o[1]) / 2;
  return [s.t, d.spd ?? null, imu.pitch ?? null, imu.roll ?? null, imu.rate ?? null,
          w.front ?? null, w.left ?? null, w.right ?? null, d.steer ?? null, om];
}

function pushSnap(s) {
  S = s;
  if (s.t != null) {
    if (HIST.length && s.t < HIST[HIST.length - 1][H.t]) HIST = [];      // новый заезд
    HIST.push(histRow(s));
    while (HIST.length && s.t - HIST[0][H.t] > HIST_KEEP_S) HIST.shift();
  }
  rates.telem.push(s.t, s.telem_n);
  rates.scan.push(s.t, s.scan_n);
  const c = s.cams || {};
  rates.camL.push(s.t, c.left?.fid);
  rates.camR.push(s.t, c.right?.fid);
}

function pushLog(lines) {
  for (const ln of lines) LOG.push(ln);
  if (LOG.length > LOG_MAX) LOG.splice(0, LOG.length - LOG_MAX);
  appendLog(lines);
}

// ── поток ───────────────────────────────────────────────
function connect() {
  if (es) es.close();
  es = new EventSource('/events');
  es.onmessage = ev => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.link) LINK = m.link;
    if (mode !== 'live') { dirty = true; return; }
    if (m.init) {
      HIST = (m.hist || []).slice();
      LOG.length = 0; $('log').innerHTML = '';
      Object.values(rates).forEach(r => r.clear());
    }
    if (m.map !== undefined) MAP = m.map;
    if (m.s) pushSnap(m.s);
    if (m.log) pushLog(m.log);
    dirty = true;
  };
  es.onerror = () => { LINK = null; dirty = true; };
}

// ── отрисовка по кадру ──────────────────────────────────
function frame() {
  if (dirty) {
    dirty = false;
    try { render(); } catch (e) { console.error(e); }
  }
  model.tick();
  requestAnimationFrame(frame);
}

function render() {
  renderTop();
  renderHero();
  renderCams();
  renderRadar();
  renderMap();
  renderSensors();
  renderVictims();
  renderCharts();
  model.set(S);
}

// ── шапка ───────────────────────────────────────────────
function chip(label, val, cls = '') { return `<div class="chip ${cls}"><span>${label}</span><b>${val}</b></div>`; }

// Система: [ключ, подпись в шапке, значение, класс ('' - норма)]. Значения
// фиксированной ширины - ячейки не прыгают от 9.8 к 10.0.
function sysRows() {
  const sys = (LINK && LINK.sys) || {}, rows = [];
  const pad = (v, w) => String(v).padStart(w, '\u2007');      // цифровой пробел
  if (sys.cpu != null) rows.push(['cpu', 'CPU', pad(fmt(sys.cpu, 0), 3) + ' %', sys.cpu > 90 ? 'warn' : '']);
  else if (sys.load != null) rows.push(['cpu', 'нагрузка', pad(fmt(sys.load, 1), 5), '']);
  if (sys.temp != null) rows.push(['temp', 'температура Pi', pad(fmt(sys.temp, 0), 3) + ' °C', sys.temp >= 80 ? 'err' : sys.temp >= 70 ? 'warn' : '']);
  if (sys.mem != null) rows.push(['mem', 'ОЗУ', pad(fmt(sys.mem, 0), 3) + ' %', sys.mem > 90 ? 'warn' : '']);
  if (sys.throttled != null) {
    const now = sys.throttled & 0x5, ever = sys.throttled & 0x50000;
    rows.push(['power', 'питание', now ? 'ПРОСАДКА' : ever ? 'было' : 'норма', now ? 'err' : ever ? 'warn' : '']);
  }
  if (S) {
    const th = rates.telem.hz(), sh = rates.scan.hz();
    rows.push(['hzEsp', 'ESP', th == null ? '—' : pad(fmt(th, 0), 3) + ' Гц', th != null && th < 150 ? 'warn' : '']);
    rows.push(['hzLidar', 'лидар', sh == null ? '—' : pad(fmt(sh, 1), 4) + ' Гц', sh != null && sh < 8 ? 'warn' : '']);
    const tk = S.tick_ms && (S.tick_ms['вкл'] || S.tick_ms['выкл']);
    rows.push(['tick', 'такт', tk ? `${pad(fmt(tk[0], 1), 4)}/${pad(fmt(tk[1], 0), 3)} мс` : '—', tk && tk[1] > 30 ? 'warn' : '']);
    const l = S.link || {};
    const errs = (l.crc || 0) + (l.lost || 0) + (l.cmd_err || 0);
    rows.push(['link', 'ошибки линка', errs ? `${l.crc}/${l.lost}/${l.cmd_err}` : 'нет', errs ? 'warn' : '']);
  }
  return rows;
}

function renderTop() {
  const src = $('src');
  let cls = 'off', text = 'нет связи с пультом';
  if (mode === 'replay') { cls = 'rp'; text = 'повтор записи'; }
  else if (LINK && LINK.nav) {
    if (S && S.sim) { cls = 'sim'; text = 'симулятор · ' + S.sim; } else { cls = 'on'; text = 'робот онлайн'; }
  } else if (LINK) text = LINK.connected ? 'nav.py молчит' : 'nav.py не запущен';
  src.className = 'src ' + cls;
  $('srcText').textContent = text;

  // В шапке - только то, что требует внимания; сами числа - в «Датчиках».
  const out = sysRows().filter(r => r[3]).map(([, label, val, cls]) => chip(label, val, cls));
  if (S && S.lidar && (S.lidar.restarts || 0) > 0) out.push(chip('лидар перезапускался', S.lidar.restarts + '×', 'err'));
  $('chips').innerHTML = out.join('');
  const rec = LINK && LINK.rec && mode === 'live';
  $('rec').hidden = !rec;
  if (rec) $('recText').textContent = 'REC';
  $('rec').title = rec ? 'пишется ' + LINK.rec : '';
}

// ── полоса статуса ──────────────────────────────────────
function mmss(s, cs = true) {
  if (s == null) return '--:--' + (cs ? '.--' : '');
  const neg = s < 0; s = Math.abs(s);
  const m = Math.floor(s / 60), r = s - m * 60;
  return (neg ? '-' : '') + String(m).padStart(2, '0') + ':' + (cs ? r.toFixed(2).padStart(5, '0') : String(Math.floor(r)).padStart(2, '0'));
}

function statusText(s) {
  if (!s) return '… ожидание nav.py …';
  if (WAIT_HINT[s.phase]) return WAIT_HINT[s.phase];
  const [pname] = PHASE[s.phase] || [s.phase || '—'];
  const parts = [pname.charAt(0).toUpperCase() + pname.slice(1)];
  if (s.act) parts.push(s.act);
  const d = s.drive;
  if (d) {
    parts.push(`кл. ${d.cell}, осталось ${fmt(d.rem_mm, 0)} мм, ${fmt(d.spd, 0)} см/с`);
    if (d.mode) parts.push(MODE[d.mode] || d.mode);
  }
  return '… ' + parts.join(' · ') + ' …';
}

function renderHero() {
  const s = S;
  const run = s ? s.run_s : null;
  $('tRun').textContent = mmss(run);
  const bar = $('tBar');
  const frac = run == null ? 0 : Math.min(1, run / RUN_LIMIT_S);
  bar.style.width = (frac * 100).toFixed(1) + '%';
  bar.className = run > RUN_LIMIT_S ? 'over' : run > RUN_LIMIT_S - 60 ? 'late' : '';
  $('tSub').textContent = run == null ? 'ожидание старта' : run > RUN_LIMIT_S
    ? `сверх 8:00 на ${mmss(run - RUN_LIMIT_S, false)}` : `осталось ${mmss(RUN_LIMIT_S - run, false)} из 8:00`;

  const [, kind] = (s && PHASE[s.phase]) || ['', 'off'];
  const ph = $('phase');
  ph.className = 'phase ' + kind;
  ph.querySelector('span').textContent = s ? (PHASE[s.phase] || [s.phase])[0] : '—';
  $('statusLine').textContent = statusText(s);
  $('rawStatus').textContent = (s && s.status) || '';

  const m = MAP || {};
  const kits = s && s.kits ? Object.values(s.kits).reduce((a, b) => a + b, 0) : null;
  const pose = s && s.pose;
  const items = [
    [(m.visited || []).length, 'клеток'],
    [(m.victims || []).length, 'меток'],
    [kits ?? '—', 'комплектов'],
    [pose ? pose.level : '—', 'этаж'],
    [pose ? `${pose.cx}:${pose.cy - pose.level * ((m.level_stride) || 1000)}` : '—', 'клетка'],
    [pose ? DIRS[pose.hdg] : '—', 'курс'],
  ];
  $('counts').innerHTML = items.map(([v, l]) => `<div class="count"><b>${esc(v)}</b><span>${l}</span></div>`).join('');
}

// ── камеры ──────────────────────────────────────────────
const camState = {left: {fid: -1, loading: false}, right: {fid: -1, loading: false}};

function boxColor(cls) {
  const k = Object.keys(VCOLOR).find(k => String(cls).toLowerCase().startsWith(k.slice(0, 4)));
  return k ? css(VCOLOR[k]) : css('--blue');
}

function renderCams() {
  for (const side of ['left', 'right']) {
    const card = $('cam-' + side);
    const fr = card.querySelector('.frame'), img = fr.querySelector('img'), svg = fr.querySelector('svg'), ph = fr.querySelector('.ph');
    const info = S && S.cams ? S.cams[side] : null;
    const st = camState[side];
    const hz = (side === 'left' ? rates.camL : rates.camR).hz();
    let meta = '', live = false, note = 'наблюдателя меток нет (запуск без зрения)';
    if (info) {
      if (info.dead) note = 'наблюдатель выключен (ошибка воркера)';
      else if (info.failed) note = 'камера отказала';
      else if (!info.ready) note = 'YOLO загружается…';
      else if (!info.armed) note = 'камера смотрит только на ходу вдоль стены';
      live = info.armed && mode === 'live' && info.fid > 0;
      meta = info.armed ? `${fmt(hz, 0)} к/с` : 'выкл';
      if (live && info.fid !== st.fid && !st.loading) {
        st.loading = true; st.fid = info.fid;
        img.onload = img.onerror = () => { st.loading = false; };
        img.src = `/cam/${side}.jpg?f=${info.fid}`;
      }
    }
    if (mode === 'replay' && info) { note = 'кадры в запись не попадают - только рамки'; }
    fr.classList.toggle('live', live);
    card.querySelector('.meta').textContent = meta;
    // рамки последнего ответа воркера - свежие (≤0.6 с)
    svg.innerHTML = '';
    if (info && info.boxes && info.bt != null && S && S.t - info.bt < 0.6) {
      for (const [cls, conf, [x1, y1, x2, y2]] of info.boxes) {
        const c = boxColor(cls);
        svg.appendChild(svgEl('rect', {x: x1, y: y1, width: x2 - x1, height: y2 - y1, fill: 'none', stroke: c, 'stroke-width': 3, rx: 3}));
        const t = svgEl('text', {x: x1 + 3, y: Math.max(14, y1 - 5), fill: c, class: 'box-lbl'});
        t.textContent = `${GLYPH[cls] || cls} ${(+conf).toFixed(2)}`;
        svg.appendChild(t);
      }
    }
    if (!live) {
      const last = ((MAP && MAP.victims) || []).filter(v => v.side === side && v.image).slice(-1)[0];
      const key = note + (last ? last.image : '');
      if (ph.dataset.k !== key) {
        ph.dataset.k = key;
        ph.innerHTML = (last ? `<div><img src="/detections/${encodeURIComponent(last.image)}" alt=""><small>последняя метка: ${esc(VNAME[last.type] || last.type)}</small></div>` : '')
          + `<div>${esc(note)}</div>`;
        if (last) ph.style.display = 'flex', ph.style.flexDirection = 'column', ph.style.justifyContent = 'center';
      }
    }
  }
}

// ── холст с учётом DPR ──────────────────────────────────
function fitCanvas(cv) {
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return [ctx, r.width, r.height];
}

// ── лидар: точки коридора и дистанции до стен ───────────
function renderRadar() {
  const cv = $('radar');
  const [ctx, W, Hh] = fitCanvas(cv);
  ctx.clearRect(0, 0, W, Hh);
  const R = 620;                                  // мм от центра до края
  const k = Math.min(W, Hh) / 2 / R, cx = W / 2, cy = Hh / 2;
  const P = (x, y) => [cx + y * k, cy - x * k];   // x вперёд (вверх), y вправо
  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(255,255,255,.06)';
  ctx.fillStyle = css('--mut'); ctx.font = '10px ' + css('--mono');
  for (const r of [150, 300, 450, 600]) {
    ctx.beginPath(); ctx.arc(cx, cy, r * k, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText(r, cx + 3, cy - r * k + 11);
  }
  // робот
  const L = 193, Wr = 125;
  const [rx, ry] = P(L / 2, -Wr / 2);
  ctx.fillStyle = 'rgba(242,169,0,.14)'; ctx.strokeStyle = css('--acc'); ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.rect(rx, ry, Wr * k, L * k); ctx.fill(); ctx.stroke();
  ctx.beginPath(); const [nx, ny] = P(L / 2 + 16, 0); ctx.moveTo(nx, ny); ctx.lineTo(...P(L / 2 - 4, -18)); ctx.lineTo(...P(L / 2 - 4, 18)); ctx.closePath();
  ctx.fillStyle = css('--acc'); ctx.fill();
  if (!S) return;
  // точки коридора (lidar_proc: x вперёд, y вправо)
  ctx.fillStyle = css('--s1');
  for (const [x, y] of S.scan || []) {
    const [px, py] = P(x, y);
    ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
  }
  // стены
  const thr = S.thr || [260, 210, 260];
  const w = S.walls || {};
  const sides = [['front', 1, 0, thr[0], 'П'], ['back', -1, 0, thr[2], 'З'], ['right', 0, 1, thr[1], 'Пр'], ['left', 0, -1, thr[1], 'Л']];
  ctx.font = '600 11px ' + css('--mono');
  for (const [key, fx, fy, t, lbl] of sides) {
    const d = w[key];
    if (d == null) continue;
    const wall = d < t;
    const dd = Math.min(d, R - 10), half = 110;
    // отрезок поперёк направления на найденной дистанции
    const [x1, y1] = fx ? P(fx * dd, -half) : P(-half, fy * dd);
    const [x2, y2] = fx ? P(fx * dd, half) : P(half, fy * dd);
    ctx.strokeStyle = wall ? css('--fg') : 'rgba(255,255,255,.25)';
    ctx.lineWidth = wall ? 3 : 1.5;
    ctx.setLineDash(wall ? [] : [5, 4]);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = wall ? css('--fg') : css('--mut');
    const txt = `${lbl} ${Math.round(d)}`;
    const tw = ctx.measureText(txt).width;
    const [ax, ay] = P(fx * dd, fy * dd);
    // подпись снаружи отрезка: спереди - выше, сзади - ниже, сбоку - вбок
    const lx = fy ? ax + fy * 8 - (fy < 0 ? tw : 0) : ax - tw / 2;
    const ly = fx ? ay - fx * 8 + (fx < 0 ? 10 : 0) : ay + 4;
    ctx.fillText(txt, Math.max(2, Math.min(W - tw - 2, lx)), Math.max(10, Math.min(Hh - 3, ly)));
  }
  $('radarMeta').textContent = `${(S.scan || []).length} точек · ${fmt(rates.scan.hz(), 1)} Гц`;
}

// ── карта ───────────────────────────────────────────────
const CELL = 56;
const DX = [0, 1, 0, -1], DY = [1, 0, -1, 0];
let mapGeo = null;

function buildMap(m) {
  const svg = $('map');
  svg.innerHTML = '';
  const cells = Object.entries(m.maze || {}).map(([k, w]) => { const [x, y] = k.split(',').map(Number); return {x, y, w}; });
  const robot = m.robot || {cx: 0, cy: 0};
  const known = cells.length ? cells : [{x: robot.cx, y: robot.cy}];
  const LS = m.level_stride || 1000;
  const lvl = y => Math.floor((y + LS / 2) / LS);
  const xs = known.map(c => c.x).concat([robot.cx]);
  const spanX = Math.max(...xs) - Math.min(...xs) + 2;
  const vx = (x, y) => x + lvl(y) * spanX, vy = y => y - lvl(y) * LS;
  const levels = [...new Set(known.map(c => lvl(c.y)).concat([lvl(robot.cy)]))].sort((a, b) => a - b);
  const multi = levels.length > 1;
  const allX = known.map(c => vx(c.x, c.y)).concat([vx(robot.cx, robot.cy)]);
  const allY = known.map(c => vy(c.y)).concat([vy(robot.cy)]);
  const minX = Math.min(...allX), maxX = Math.max(...allX), minY = Math.min(...allY), maxY = Math.max(...allY);
  const PAD = 8, TOP = multi ? 22 : 0;
  const W = (maxX - minX + 1) * CELL, Hh = (maxY - minY + 1) * CELL;
  svg.setAttribute('viewBox', `${-PAD} ${-PAD - TOP} ${W + 2 * PAD} ${Hh + 2 * PAD + TOP}`);
  svg.style.maxWidth = Math.max(260, (W + 2 * PAD) * 1.6) + 'px';
  const sx = x => (x - minX) * CELL, sy = y => (maxY - y) * CELL;
  const center = (x, y) => [sx(vx(x, y)) + CELL / 2, sy(vy(y)) + CELL / 2];

  const set = a => new Set((a || []).map(p => p[0] + ',' + p[1]));
  const visited = set(m.visited), blocked = set(m.blocked), blue = set(m.blue_done);
  const cp = m.checkpoint || {cx: 0, cy: 0};
  const FG = css('--fg'), ACC = css('--acc');

  const defs = svgEl('defs', {});
  const pat = svgEl('pattern', {id: 'hatch', width: 7, height: 7, patternUnits: 'userSpaceOnUse', patternTransform: 'rotate(45)'});
  pat.appendChild(svgEl('rect', {width: 3, height: 7, fill: 'rgba(229,83,75,.6)'}));
  defs.appendChild(pat); svg.appendChild(defs);

  if (multi) for (const z of levels) {
    const xz = known.filter(c => lvl(c.y) === z).map(c => vx(c.x, c.y));
    const t = svgEl('text', {x: sx(Math.min(...xz)), y: -10, fill: css('--mut'), 'font-size': 12, 'letter-spacing': 1, 'font-family': css('--mono')});
    t.textContent = 'ЭТАЖ ' + z; svg.appendChild(t);
  }
  for (const c of cells) {
    const key = c.x + ',' + c.y, x = sx(vx(c.x, c.y)), y = sy(vy(c.y));
    let fill = 'rgba(255,255,255,.025)';
    if (blocked.has(key)) fill = 'url(#hatch)';
    else if (blue.has(key)) fill = 'rgba(57,135,229,.32)';
    else if (visited.has(key)) fill = '#2a2a28';
    svg.appendChild(svgEl('rect', {x, y, width: CELL, height: CELL, fill, stroke: 'rgba(255,255,255,.06)', 'stroke-width': 1}));
    if (cp.cx === c.x && cp.cy === c.y)
      svg.appendChild(svgEl('rect', {x: x + 5, y: y + 5, width: CELL - 10, height: CELL - 10, fill: 'none', stroke: ACC, 'stroke-width': 1.5, 'stroke-dasharray': '4 3', rx: 4}));
    const e = [[x, y, x + CELL, y], [x + CELL, y, x + CELL, y + CELL], [x, y + CELL, x + CELL, y + CELL], [x, y, x, y + CELL]];
    for (let d = 0; d < 4; d++) {
      if (c.w[d] === true) svg.appendChild(svgEl('line', {x1: e[d][0], y1: e[d][1], x2: e[d][2], y2: e[d][3], stroke: FG, 'stroke-width': 3.5, 'stroke-linecap': 'round'}));
      else if (c.w[d] == null) svg.appendChild(svgEl('line', {x1: e[d][0], y1: e[d][1], x2: e[d][2], y2: e[d][3], stroke: 'rgba(255,255,255,.18)', 'stroke-width': 1, 'stroke-dasharray': '2 4'}));
    }
  }
  for (const [ax, ay, , bx, by, n] of m.ramps || []) {
    const [x1, y1] = center(ax, ay), [x2, y2] = center(bx, by);
    svg.appendChild(svgEl('line', {x1, y1, x2, y2, stroke: ACC, 'stroke-width': 1.5, 'stroke-dasharray': '5 4'}));
    const t = svgEl('text', {x: (x1 + x2) / 2, y: (y1 + y2) / 2 - 6, 'text-anchor': 'middle', 'font-size': 11, fill: ACC, 'font-family': css('--mono')});
    t.textContent = 'горка ' + n; svg.appendChild(t);
  }
  for (const v of m.victims || []) {
    const [vcx, vcy] = v.cell, d = v.wall_dir ?? 0;
    const [x0, y0] = center(vcx, vcy);
    const px = x0 + DX[d] * CELL * 0.32, py = y0 - DY[d] * CELL * 0.32;
    svg.appendChild(svgEl('circle', {cx: px, cy: py, r: 10, fill: css(VCOLOR[v.type] || '--mut'), stroke: css('--card'), 'stroke-width': 2}));
    const t = svgEl('text', {x: px, y: py, 'text-anchor': 'middle', 'dominant-baseline': 'central', 'font-size': 12, 'font-weight': 700, fill: '#111', 'font-family': css('--mono')});
    t.textContent = GLYPH[v.type] || '?'; svg.appendChild(t);
  }
  // правда модели: настоящие плитки (рамкой) и метки (кольцом) и робот-призрак
  if (WORLD) {
    const TC = {black: '#e5534b', blue: '#3987e5', silver: '#c9d1d9', red: '#e5534b'};
    for (const [tx, ty, t] of WORLD.tiles || []) {
      const [x0, y0] = center(tx, ty);
      svg.appendChild(svgEl('rect', {x: x0 - CELL / 2 + 3, y: y0 - CELL / 2 + 3, width: CELL - 6, height: CELL - 6, rx: 4,
                                      fill: 'none', stroke: TC[t] || '#888', 'stroke-width': 1.5, 'stroke-dasharray': '2 3', opacity: .9}));
    }
    for (const v of WORLD.victims || []) {
      const [x0, y0] = center(v.cell[0], v.cell[1]);
      const k = CELL / 300;
      const off = (CELL / 2 - 4), along = v.along * k;
      const px = x0 + DX[v.d] * off + (DY[v.d] ? along : 0), py = y0 - DY[v.d] * off - (DX[v.d] ? along : 0);
      svg.appendChild(svgEl('circle', {cx: px, cy: py, r: 7, fill: 'none', stroke: css(VCOLOR[v.type] || '--mut'), 'stroke-width': 2}));
    }
    const ghost = svgEl('g', {id: 'mapTruth', opacity: .75});
    ghost.appendChild(svgEl('circle', {r: CELL * 0.2, fill: 'none', stroke: '#8fbcf3', 'stroke-width': 2, 'stroke-dasharray': '3 2'}));
    ghost.appendChild(svgEl('line', {x1: 0, y1: 0, x2: 0, y2: -CELL * 0.32, stroke: '#8fbcf3', 'stroke-width': 2}));
    svg.appendChild(ghost);
  }
  const rob = svgEl('g', {id: 'mapRobot'});
  const r = CELL * 0.27;
  rob.appendChild(svgEl('circle', {r: r * 1.25, fill: 'rgba(242,169,0,.14)'}));
  rob.appendChild(svgEl('polygon', {points: `0,${-r} ${-r * .75},${r * .7} 0,${r * .35} ${r * .75},${r * .7}`, fill: ACC, stroke: '#111', 'stroke-width': 1.5, 'stroke-linejoin': 'round'}));
  svg.appendChild(rob);
  mapGeo = {center, levels};
}

function renderMap() {
  const m = MAP;
  $('mapEmpty').style.display = m ? 'none' : 'block';
  $('map').style.display = m ? 'block' : 'none';
  if (!m) return;
  const key = JSON.stringify([m.maze, m.visited, m.blocked, m.blue_done, m.ramps, m.victims, m.checkpoint, !!WORLD]);
  if (key !== mapKey) { mapKey = key; buildMap(m); }
  // робот: клетка по позе nav.py + путь по одометрии текущей поездки, курс - гироскоп
  const s = S, pose = s ? s.pose : m.robot;
  if (!pose || !mapGeo) return;
  let [x, y] = mapGeo.center(pose.cx, pose.cy);
  const d = s && s.drive;
  if (d && d.cell != null && d.rem_mm != null) {
    const along = Math.max(0, d.cell * 300 - d.rem_mm) / 300 * CELL;
    x += DX[pose.hdg] * along; y -= DY[pose.hdg] * along;
  }
  const yaw = s && s.imu && s.imu.yaw != null ? s.imu.yaw : pose.hdg * 90;
  $('mapRobot').setAttribute('transform', `translate(${x.toFixed(1)} ${y.toFixed(1)}) rotate(${yaw.toFixed(1)})`);
  const g = $('mapTruth'), tr = s && s.truth;
  if (g && tr) {
    const [gx, gy] = mapGeo.center(tr.x / 300, tr.y / 300);
    g.setAttribute('transform', `translate(${gx.toFixed(1)} ${gy.toFixed(1)}) rotate(${tr.th})`);
  }
  $('mapMeta').textContent = `${(m.visited || []).length} клеток${mapGeo.levels.length > 1 ? ` · ${mapGeo.levels.length} этажа` : ''}`
    + (WORLD ? ' · синий пунктир - где робот на самом деле' : '');
}

function renderLegend() {
  const items = [['#2a2a28', 'посещена'], ['rgba(57,135,229,.5)', 'синяя'], ['repeating-linear-gradient(45deg,rgba(229,83,75,.7) 0 3px,transparent 3px 6px)', 'чёрная / непроезжая'],
                 ['transparent;border:1.5px dashed var(--acc)', 'чекпоинт']];
  $('mapLegend').innerHTML = items.map(([c, t]) => `<span><i style="background:${c}"></i>${t}</span>`).join('')
    + ['damaged', 'stable', 'unharmed'].map(k => `<span><i style="background:var(${VCOLOR[k]});border-radius:50%"></i>${GLYPH[k]} ${VNAME[k].toLowerCase()}</span>`).join('');
}

// ── датчики ─────────────────────────────────────────────
const SENS = [
  ['Лидар', [['front', 'Спереди'], ['back', 'Сзади'], ['left', 'Слева'], ['right', 'Справа']]],
  ['Гироскоп', [['yaw', 'Курс'], ['pitch', 'Тангаж'], ['roll', 'Крен'], ['rate', 'Скор. поворота']]],
  ['Колёса', [['odoL', 'Путь L'], ['odoR', 'Путь R'], ['spd', 'Скорость'], ['steer', 'Руление']]],
  ['Плитка', [['tile', 'Цвет'], ['rgbc', 'R G B C'], ['bump', 'Бампер L/R'], ['kits', 'Комплекты']]],
  ['Связь с ESP', [['esp', 'Состояние'], ['age', 'Возраст пакета'], ['enc', 'Энкодеры'], ['fresh', 'Гиро / цвет']]],
  ['Курс по стенам', [['wall_e', 'Ошибка стен'], ['head_off', 'Поправка'], ['level', 'Этаж'], ['cp', 'Чекпоинт']]],
  ['Raspberry Pi', [['cpu', 'Загрузка'], ['temp', 'Температура'], ['mem', 'Память'], ['power', 'Питание']]],
  ['Частоты', [['hzEsp', 'Телеметрия ESP'], ['hzLidar', 'Лидар'], ['tick', 'Такт p50/max'], ['link', 'Ошибки линка']]],
];
const sv = {};
function buildSensors() {
  const box = $('sensors');
  for (const [title, rows] of SENS) {
    const g = el('div', 'sg', `<h3>${title}</h3>`);
    for (const [k, label] of rows) {
      const r = el('div', 'row', `<span>${label}</span>`);
      sv[k] = el('div', 'val', '—'); r.appendChild(sv[k]); g.appendChild(r);
    }
    box.appendChild(g);
  }
}
function setv(k, html, cls = '') { const e = sv[k]; if (!e) return; if (e._h !== html) { e.innerHTML = html; e._h = html; } e.className = 'val ' + cls; }

function renderSensors() {
  for (const [k, , v, cls] of sysRows()) setv(k, esc(v), cls);
  const s = S;
  if (!s) return;
  const w = s.walls || {}, thr = s.thr || [260, 210, 260];
  const wv = (d, t) => [d == null ? '—' : fmt(d, 0, ' мм'), d == null ? 'mut' : d < t ? 'ok' : ''];
  setv('front', ...wv(w.front, thr[0])); setv('back', ...wv(w.back, thr[2]));
  setv('left', ...wv(w.left, thr[1])); setv('right', ...wv(w.right, thr[1]));
  const i = s.imu || {};
  setv('yaw', fmt(i.yaw, 1, '°')); setv('pitch', fmt(i.pitch, 1, '°'), Math.abs(i.pitch) > 10 ? 'warn' : '');
  setv('roll', fmt(i.roll, 1, '°'), Math.abs(i.roll) > 10 ? 'warn' : ''); setv('rate', fmt(i.rate, 0, '°/с'));
  const o = s.odo || [], d = s.drive || {};
  setv('odoL', fmt(o[0], 1, ' см')); setv('odoR', fmt(o[1], 1, ' см'));
  const vv = withVel(HIST.slice(-4)).map(r => r[H.vel]).filter(v => v != null);
  const vel = vv.length ? vv.reduce((a, b) => a + b, 0) / vv.length : null;
  setv('spd', vel == null ? '—' : Math.abs(vel) < 0.5 ? 'стоит' : fmt(vel, 0, ' см/с'), vel != null && Math.abs(vel) < 0.5 ? 'mut' : '');
  setv('steer', s.drive ? fmt(d.steer, 1) : '—');
  const [tn, tc] = TILE[s.tile] || [s.tile || '—', 'transparent'];
  setv('tile', s.color_on === false ? 'выкл' : `<span class="swatch" style="background:${tc}"></span>${esc(tn)}`,
       s.tile === 'black' ? 'err' : s.tile === 'silver' || s.tile === 'blue' ? 'ok' : '');
  setv('rgbc', (s.rgbc || []).join(' ') || '—');
  const b = s.bump || [];
  setv('bump', (b[0] ? '●' : '○') + ' ' + (b[1] ? '●' : '○'), b[0] || b[1] ? 'warn' : '');
  const kits = s.kits || {};
  setv('kits', ['L', 'R'].map(k => `${k} <span class="pips">${Array.from({length: 4}, (_, j) => `<i class="${j < (kits[k] ?? 0) ? '' : 'used'}"></i>`).join('')}</span>`).join(' '));
  setv('esp', s.esp ? 'ПАУЗА' : 'работает', s.esp ? 'warn' : 'ok');
  setv('age', fmt(s.telem_age_ms, 0, ' мс'), s.telem_age_ms > 50 ? 'err' : '');
  const e = s.enc || [];
  setv('enc', `${e[0] ? 'L ок' : 'L нет'} · ${e[1] ? 'R ок' : 'R нет'}`, e[0] && e[1] ? 'ok' : 'err');
  const f = s.fresh || [];
  setv('fresh', `${f[0] ? 'ок' : 'НЕТ'} / ${f[1] ? 'ок' : 'нет'}`, f[0] ? '' : 'err');
  setv('wall_e', s.drive ? fmt(d.wall_e, 0, ' мм') : '—'); setv('head_off', s.drive ? fmt(d.head_off, 1, '°') : '—');
  setv('level', s.pose ? s.pose.level : '—');
  setv('cp', s.pose ? `${s.pose.cp[0]}:${s.pose.cp[1]} ${DIRS[s.pose.cp[2]] || ''}` : '—');
}

// ── метки ───────────────────────────────────────────────
let victimsKey = null;
function renderVictims() {
  const list = (MAP && MAP.victims) || [];
  $('vCount').textContent = list.length;
  const key = JSON.stringify(list);
  if (key === victimsKey) return;
  victimsKey = key;
  const box = $('victims');
  box.innerHTML = list.length ? '' : '<div class="empty">пока ни одной</div>';
  for (const v of [...list].reverse()) {
    const c = el('div', 'v');
    c.innerHTML = `<div class="g" style="background:var(${VCOLOR[v.type] || '--mut'})">${GLYPH[v.type] || '?'}</div>
      <div><div class="t">${esc(VNAME[v.type] || v.type || '?')}</div>
      <div class="m">клетка ${v.cell ? v.cell.join(':') : '?'} · ${v.side === 'right' ? 'справа' : 'слева'}${v.on_move ? ' · на ходу' : ''}${v.checksum != null ? ' · сумма ' + v.checksum : ''}</div></div>
      <div class="r">${v.conf != null ? (+v.conf).toFixed(2) : ''}<br><small>${esc(v.ts || '')}</small></div>
      ${v.image ? `<a href="/detections/${encodeURIComponent(v.image)}" target="_blank" rel="noopener"><img src="/detections/${encodeURIComponent(v.image)}" alt="" loading="lazy"></a>` : ''}`;
    box.appendChild(c);
  }
}

// ── графики: малые кратные, общее окно 30 с ──────────────
const CHARTS = [
  {title: 'Скорость и руление', unit: 'см/с', series: [[H.vel, 'по колёсам', '--s1'], [H.spd, 'уставка', '--s2'], [H.steer, 'руление', '--s3']], zero: true},
  {title: 'Наклон корпуса', unit: '°', series: [[H.pitch, 'тангаж', '--s1'], [H.roll, 'крен', '--s2']], zero: true, minSpan: 10},
  {title: 'Скорость поворота', unit: '°/с', series: [[H.rate, 'поворот', '--s1']], zero: true, minSpan: 40},
  {title: 'Дистанции лидара', unit: 'мм', series: [[H.front, 'спереди', '--s1'], [H.left, 'слева', '--s2'], [H.right, 'справа', '--s3']], zero: true, max: 1200},
];
const charts = [];
function buildCharts() {
  const box = $('charts');
  for (const spec of CHARTS) {
    const w = el('div', 'ch');
    const lg = spec.series.length > 1 ? `<span class="lg">${spec.series.map(([, n, c], j) => `<span><i style="background:var(${c})"></i>${n}<em data-j="${j}"></em></span>`).join('')}</span>`
      : `<span class="lg"><span><em data-j="0"></em></span></span>`;
    w.innerHTML = `<div class="hd"><b>${spec.title}</b><small>${spec.unit}</small>${lg}</div><canvas></canvas><div class="tip"></div>`;
    box.appendChild(w);
    const ch = {spec, w, cv: w.querySelector('canvas'), tip: w.querySelector('.tip'), hover: null};
    ch.cv.addEventListener('mousemove', e => { const r = ch.cv.getBoundingClientRect(); ch.hover = (e.clientX - r.left) / r.width; drawChart(ch); });
    ch.cv.addEventListener('mouseleave', () => { ch.hover = null; ch.tip.style.display = 'none'; drawChart(ch); });
    charts.push(ch);
  }
}

function renderCharts() {
  for (const ch of charts) drawChart(ch);
  const n = HIST.length;
  $('chartsMeta').textContent = n ? `${n} точек` : '';
}

function drawChart(ch) {
  const {spec, cv} = ch;
  const [ctx, W, Hh] = fitCanvas(cv);
  ctx.clearRect(0, 0, W, Hh);
  const padL = 34, padR = 6, padT = 6, padB = 16;
  const tEnd = HIST.length ? HIST[HIST.length - 1][H.t] : 0, t0 = tEnd - WINDOW_S;
  const rows = withVel(HIST.filter(r => r[H.t] >= t0 - 1)).filter(r => r[H.t] >= t0);
  let lo = Infinity, hi = -Infinity;
  for (const r of rows) for (const [j] of spec.series) { const v = r[j]; if (v != null) { lo = Math.min(lo, v); hi = Math.max(hi, v); } }
  if (spec.max != null) hi = Math.min(hi, spec.max);
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (spec.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
  const span = Math.max(hi - lo, spec.minSpan || 1);
  const mid = (hi + lo) / 2; lo = Math.min(lo, mid - span / 2); hi = Math.max(hi, mid + span / 2);
  const pad = (hi - lo) * 0.08; lo -= spec.zero && lo === 0 ? 0 : pad; hi += pad;
  const X = t => padL + (t - t0) / WINDOW_S * (W - padL - padR);
  const Y = v => padT + (1 - (v - lo) / (hi - lo)) * (Hh - padT - padB);
  // сетка: 3 деления по y, метки времени по 10 с
  ctx.font = '10px ' + css('--mono'); ctx.fillStyle = css('--mut'); ctx.strokeStyle = 'rgba(255,255,255,.06)'; ctx.lineWidth = 1;
  for (const v of niceTicks(lo, hi, 3)) {
    const y = Math.round(Y(v)) + .5;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
    ctx.fillText(String(+v.toFixed(1)), 2, y + 3);
  }
  if (lo < 0 && hi > 0) { ctx.strokeStyle = 'rgba(255,255,255,.2)'; const y = Math.round(Y(0)) + .5; ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke(); }
  for (let s = 0; s <= WINDOW_S; s += 10) ctx.fillText(s === WINDOW_S ? 'сейчас' : `-${WINDOW_S - s}с`, X(t0 + s) - (s === WINDOW_S ? 34 : 8), Hh - 3);
  // ряды
  ctx.lineWidth = 2; ctx.lineJoin = 'round';
  for (const [j, , c] of spec.series) {
    ctx.strokeStyle = css(c); ctx.beginPath();
    let pen = false, last = null;
    for (const r of rows) {
      const v = r[j];
      if (v == null || (last != null && r[H.t] - last > 0.6)) { pen = false; }
      if (v == null) continue;
      const x = X(r[H.t]), y = Y(Math.min(v, hi));
      pen ? ctx.lineTo(x, y) : ctx.moveTo(x, y); pen = true; last = r[H.t];
    }
    ctx.stroke();
  }
  // последние значения в легенде
  const lastRow = rows[rows.length - 1];
  ch.w.querySelectorAll('.lg em').forEach(e => { const [j] = spec.series[+e.dataset.j]; const v = lastRow ? lastRow[j] : null; e.textContent = v == null ? '—' : fmt(v, Math.abs(v) < 10 ? 1 : 0); });
  // наведение: перекрестие + подсказка
  if (ch.hover != null && rows.length) {
    const t = t0 + (ch.hover * W - padL) / (W - padL - padR) * WINDOW_S;
    let best = rows[0];
    for (const r of rows) if (Math.abs(r[H.t] - t) < Math.abs(best[H.t] - t)) best = r;
    const x = X(best[H.t]);
    ctx.strokeStyle = 'rgba(255,255,255,.35)'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, Hh - padB); ctx.stroke();
    for (const [j, , c] of spec.series) { const v = best[j]; if (v == null) continue; ctx.fillStyle = css(c); ctx.strokeStyle = css('--card'); ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, Y(Math.min(v, hi)), 4, 0, 7); ctx.fill(); ctx.stroke(); }
    ch.tip.innerHTML = `<div style="color:var(--mut)">${(best[H.t] - tEnd).toFixed(1)} с</div>` + spec.series.map(([j, n, c]) => `<div><i style="background:var(${c})"></i>${n}: <b>${fmt(best[j], 1)}</b> ${spec.unit}</div>`).join('');
    ch.tip.style.display = 'block';
    const tw = ch.tip.offsetWidth;
    ch.tip.style.left = Math.min(Math.max(0, x + 12), W - tw) + 'px';
    ch.tip.style.top = '24px';
  }
}

// Скорость по колёсам - производная среднего пути. "Z" (обнуление
// одометрии) даёт скачок назад - такие точки выкидываем.
function withVel(rows) {
  return rows.map((r, i) => {
    const p = rows[i - 1];
    let v = null;
    if (p && r[H.odo] != null && p[H.odo] != null && r[H.t] > p[H.t]) {
      v = (r[H.odo] - p[H.odo]) / (r[H.t] - p[H.t]);
      if (Math.abs(v) > 150) v = null;
    }
    const o = r.slice(); o[H.vel] = v; return o;
  });
}

function niceTicks(lo, hi, n) {
  const raw = (hi - lo) / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 5, 10].map(k => k * mag).find(s => s >= raw) || raw;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(v);
  return out;
}

// ── лог ─────────────────────────────────────────────────
function logClass(t) {
  if (/FATAL|авария/i.test(t)) return 'fatal';
  if (/WARN|\[!\]/.test(t)) return 'warn';
  if (t.includes('[VW]')) return 'vw';
  if (t.startsWith('── пульт')) return 'sys';
  if (/→|←|↑|поворот/.test(t)) return 'dec';
  return '';
}
function logTime(ts) { const d = new Date(ts * 1000); return d.toTimeString().slice(0, 8) + '.' + String(d.getMilliseconds()).padStart(3, '0').slice(0, 1); }
function appendLog(lines) {
  const box = $('log');
  const follow = $('logFollow').checked;
  const f = $('logFilter').value.trim().toLowerCase();
  const frag = document.createDocumentFragment();
  for (const [, ts, text] of lines) {
    const d = el('div', 'll ' + logClass(text));
    d.innerHTML = `<span class="ts">${logTime(ts)}</span>${esc(text)}`;
    d._t = text.toLowerCase();
    if (f && !d._t.includes(f)) d.classList.add('hide');
    frag.appendChild(d);
  }
  box.appendChild(frag);
  while (box.childElementCount > LOG_MAX) box.firstChild.remove();
  if (follow) box.scrollTop = box.scrollHeight;
}
function refilterLog() {
  const f = $('logFilter').value.trim().toLowerCase();
  for (const d of $('log').children) d.classList.toggle('hide', !!f && !d._t.includes(f));
  $('log').classList.toggle('only-warn', $('logWarn').checked);
}

// ── 3D-модель робота (three.js), повёрнута по гироскопу ──
const model = (() => {
  const host = $('model');
  if (!window.THREE) return {set() {}, tick() {}};
  const T = THREE;
  const renderer = new T.WebGLRenderer({antialias: true, alpha: true});
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  host.appendChild(renderer.domElement);
  const scene = new T.Scene();
  const cam = new T.PerspectiveCamera(32, 1, 10, 5000);
  scene.add(new T.HemisphereLight(0xffffff, 0x505050, 0.75));
  const sun = new T.DirectionalLight(0xffffff, 0.9); sun.position.set(-250, 500, 300); scene.add(sun);
  const mat = c => new T.MeshLambertMaterial({color: c});

  // пол: клетки 300 мм и стрелка на север
  const floor = new T.Group(); scene.add(floor);
  const grid = new T.GridHelper(1200, 4, 0x555550, 0x33332f); grid.position.y = 0; floor.add(grid);
  const fine = new T.GridHelper(1200, 24, 0x262624, 0x262624); fine.position.y = -0.5; floor.add(fine);
  const north = new T.Mesh(new T.ConeGeometry(18, 40, 3), mat(0xf2a900)); north.rotation.x = -Math.PI / 2; north.position.set(0, 1, -420); floor.add(north);

  // робот: мультяшная модель AZcrab (robot3d.js); вперёд = -z, вверх = +y, мм
  const robot = new T.Group(); scene.add(robot);
  const az3d = window.makeRobot3D ? window.makeRobot3D(T) : null;
  if (az3d) robot.add(az3d.group);
  // мягкая тень-пятно под роботом
  const sh = document.createElement('canvas'); sh.width = sh.height = 64;
  const sg = sh.getContext('2d'), rg = sg.createRadialGradient(32, 32, 4, 32, 32, 32);
  rg.addColorStop(0, 'rgba(0,0,0,.55)'); rg.addColorStop(1, 'rgba(0,0,0,0)');
  sg.fillStyle = rg; sg.fillRect(0, 0, 64, 64);
  const shadow = new T.Mesh(new T.PlaneGeometry(260, 260), new T.MeshBasicMaterial({map: new T.CanvasTexture(sh), transparent: true, depthWrite: false}));
  shadow.rotation.x = -Math.PI / 2; shadow.position.y = 0.6; scene.add(shadow);

  let az = 0.75, el = 0.33, dist = 560;     // вид: азимут, высота, расстояние
  const cur = {yaw: 0, pitch: 0, roll: 0}, tgt = {yaw: 0, pitch: 0, roll: 0};
  function place() { cam.position.set(Math.sin(az) * Math.cos(el) * dist, Math.sin(el) * dist + 40, Math.cos(az) * Math.cos(el) * dist); cam.lookAt(0, 70, 0); }
  place();
  let drag = null;
  host.addEventListener('pointerdown', e => { drag = [e.clientX, e.clientY, az, el]; host.setPointerCapture(e.pointerId); });
  host.addEventListener('pointermove', e => { if (!drag) return; az = drag[2] - (e.clientX - drag[0]) * 0.01; el = Math.max(0.05, Math.min(1.4, drag[3] + (e.clientY - drag[1]) * 0.01)); place(); });
  host.addEventListener('pointerup', () => { drag = null; });
  host.addEventListener('wheel', e => { e.preventDefault(); dist = Math.max(400, Math.min(1800, dist * (1 + Math.sign(e.deltaY) * 0.08))); place(); }, {passive: false});

  function resize() {
    const r = host.getBoundingClientRect();
    const w = Math.max(1, r.width), h = Math.max(1, r.height);
    const c = renderer.domElement;
    if (c._w !== w || c._h !== h) { renderer.setSize(w, h); cam.aspect = w / h; cam.updateProjectionMatrix(); c._w = w; c._h = h; }
  }
  const angDiff = (a, b) => ((a - b + 540) % 360) - 180;
  return {
    set(s) {
      if (!s || !s.imu) return;
      tgt.yaw = s.imu.yaw ?? 0; tgt.pitch = s.imu.pitch ?? 0; tgt.roll = s.imu.roll ?? 0;
      const o = s.odo || [];
      if (az3d && o[0] != null && o[1] != null) az3d.spin((o[0] + o[1]) / 2 * 10);
      $('modelMeta').textContent = `курс ${fmt(tgt.yaw, 1)}° · тангаж ${fmt(tgt.pitch, 1)}° · крен ${fmt(tgt.roll, 1)}°`;
    },
    tick() {
      resize();
      cur.yaw += angDiff(tgt.yaw, cur.yaw) * 0.25;
      cur.pitch += (tgt.pitch - cur.pitch) * 0.25;
      cur.roll += (tgt.roll - cur.roll) * 0.25;
      robot.rotation.set(cur.pitch * Math.PI / 180, -cur.yaw * Math.PI / 180, -cur.roll * Math.PI / 180, 'YXZ');
      renderer.render(scene, cam);
    },
  };
})();

// ── повтор записей ──────────────────────────────────────
const RP = {frames: [], maps: [], logs: [], i: 0, playing: false, speed: 1, tPlay: 0, wall0: 0, name: ''};

async function openRuns() {
  const dlg = $('runsDlg'), list = $('runsList');
  list.innerHTML = '<div class="empty">загрузка…</div>';
  dlg.showModal();
  try {
    const runs = await (await fetch('/runs', {cache: 'no-store'})).json();
    list.innerHTML = runs.length ? '' : '<div class="empty">записей пока нет - они появятся после первого заезда</div>';
    for (const r of runs) {
      const b = el('button', '', `<span>${esc(r.name.replace(/\.jsonl(\.gz)?$/, '').replace(/_/g, ' '))}${r.active ? ' <small>· пишется сейчас</small>' : ''}</span><small>${(r.size / 1024).toFixed(0)} КБ</small>`);
      b.type = 'button';
      b.onclick = () => { dlg.close(); loadRun(r.name); };
      list.appendChild(b);
    }
  } catch (e) { list.innerHTML = '<div class="empty">не удалось получить список</div>'; }
}

async function loadRun(name) {
  const txt = await (await fetch('/runs/' + encodeURIComponent(name), {cache: 'no-store'})).text();
  const frames = [], maps = [], logs = [];
  let map = null, seq = 0;
  for (const line of txt.split('\n')) {
    if (!line) continue;
    let s; try { s = JSON.parse(line); } catch { continue; }
    if (s.map) map = s.map;
    const lg = (s.log || []).map(([, ts, t]) => [++seq, ts, t]);
    delete s.map; delete s.log;
    frames.push(s); maps.push(map); logs.push(lg);
  }
  if (!frames.length) { alert('В записи нет снимков'); return; }
  WORLD = (frames.find(f => f.truth && f.truth.world) || {truth: {}}).truth.world || null;
  Object.assign(RP, {frames, maps, logs, i: 0, playing: false, name});
  mode = 'replay';
  $('replayBar').hidden = false;
  $('rpName').textContent = name;
  $('rpRange').max = frames.length - 1;
  $('rpEnd').textContent = mmss(relT(frames.length - 1), false) + ' ';
  seek(0, true);
}

const relT = i => RP.frames[i].t - RP.frames[0].t;

function seek(i, rebuildLog) {
  i = Math.max(0, Math.min(RP.frames.length - 1, i));
  const prev = RP.i;
  RP.i = i;
  const s = RP.frames[i];
  // история графиков - снимки в окне перед этим моментом
  HIST = [];
  for (let j = i; j >= 0 && s.t - RP.frames[j].t <= HIST_KEEP_S; j--) HIST.unshift(histRow(RP.frames[j]));
  Object.values(rates).forEach(r => r.clear());
  for (let j = Math.max(0, i - 25); j <= i; j++) { const f = RP.frames[j]; rates.telem.push(f.t, f.telem_n); rates.scan.push(f.t, f.scan_n); }
  S = s; MAP = RP.maps[i];
  if (rebuildLog || i < prev) {
    LOG.length = 0; $('log').innerHTML = '';
    const all = [];
    for (let j = 0; j <= i; j++) all.push(...RP.logs[j]);
    const tail = all.slice(-LOG_MAX);
    LOG.push(...tail); appendLog(tail);
  } else {
    const add = [];
    for (let j = prev + 1; j <= i; j++) add.push(...RP.logs[j]);
    if (add.length) pushLog(add);
  }
  $('rpRange').value = i;
  $('rpTime').textContent = mmss(relT(i), true).slice(0, -1);
  dirty = true;
}

function replayTick() {
  if (mode === 'replay' && RP.playing) {
    const t = RP.tPlay + (performance.now() - RP.wall0) / 1000 * RP.speed;
    let i = RP.i;
    while (i < RP.frames.length - 1 && relT(i + 1) <= t) i++;
    if (i !== RP.i) seek(i, false);
    if (i >= RP.frames.length - 1) setPlaying(false);
  }
  setTimeout(replayTick, 50);
}
function setPlaying(on) {
  RP.playing = on;
  RP.tPlay = relT(RP.i); RP.wall0 = performance.now();
  $('rpPlay').textContent = on ? '❚❚' : '▶';
}
function exitReplay() {
  mode = 'live'; setPlaying(false); WORLD = null; mapKey = null;
  $('replayBar').hidden = true;
  S = null; MAP = null; HIST = [];
  connect();
}

// ── запуск ──────────────────────────────────────────────
buildSensors();
buildCharts();
renderLegend();
$('logFilter').addEventListener('input', refilterLog);
$('logWarn').addEventListener('change', refilterLog);
$('btnRuns').addEventListener('click', openRuns);
$('btnFull').addEventListener('click', () => document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen?.());
$('rpPlay').addEventListener('click', () => { if (RP.i >= RP.frames.length - 1) seek(0, true); setPlaying(!RP.playing); });
$('rpRange').addEventListener('input', e => { seek(+e.target.value, false); if (RP.playing) setPlaying(true); });
$('rpSpeed').addEventListener('change', e => { RP.speed = +e.target.value; if (RP.playing) setPlaying(true); });
$('rpLive').addEventListener('click', exitReplay);
document.addEventListener('keydown', e => {
  if (mode !== 'replay' || e.target.matches('input[type=search]')) return;
  if (e.code === 'Space') { e.preventDefault(); $('rpPlay').click(); }
  if (e.code === 'ArrowRight') seek(RP.i + (e.shiftKey ? 50 : 5), false);
  if (e.code === 'ArrowLeft') seek(RP.i - (e.shiftKey ? 50 : 5), false);
});
window.addEventListener('resize', () => { dirty = true; });
connect();
replayTick();
// ?run=<запись> - сразу открыть повтор (ссылки из отчёта ночного прогона)
{ const r = new URLSearchParams(location.search).get('run'); if (r) loadRun(r); }
requestAnimationFrame(frame);
setInterval(() => { dirty = true; }, 1000);      // связь/возраст обновлять и без снимков
