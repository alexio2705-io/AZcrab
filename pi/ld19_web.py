#!/usr/bin/env python3
"""Живая визуализация LD19: сервер на Пи, радар в браузере.

Запуск на малине:   ~/maze/venv/bin/python ~/maze/ld19_web.py
Открыть с мака:     http://<ip-малины>:8080

Фоновый поток крутит лидар и держит последний оборот, HTTP отдаёт его
как JSON по /scan; страница опрашивает и рисует. Зависимостей сверх
pyserial нет - только стандартная библиотека.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ld19 import DEFAULT_PORT, BAUDRATE, ZERO_DIR, LD19

PAGE = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Лидар LD19 - MAZE2027</title>
<style>
  /* Системный моноширинный стек: страница отдаётся с Пи без интернета. */
  :root {
    --bg:#14161A; --panel:#1B1E23; --fg:#D7DADF; --mut:#7E838C;
    --line:rgba(255,255,255,.08); --line2:rgba(255,255,255,.28); --hov:rgba(255,255,255,.04);
    --acc:#F2A900;
    --mono:ui-monospace,"SF Mono",Menlo,Consolas,"Liberation Mono",monospace;
  }
  * { box-sizing:border-box; }
  html { background:var(--bg); }
  body { margin:0; background:var(--bg); color:var(--fg); font:13px/20px var(--mono);
         font-variant-numeric:tabular-nums; -webkit-font-smoothing:antialiased; -webkit-text-size-adjust:100%; }
  .wrap { max-width:1120px; margin:0 auto; padding:16px; }
  .bar { display:flex; align-items:center; justify-content:space-between; gap:16px;
         padding:0 0 12px; border-bottom:1px solid var(--line); margin-bottom:16px; }
  .bar h1 { white-space:nowrap; margin:0; font:600 13px/20px var(--mono); letter-spacing:.1em; text-transform:uppercase; }
  .bar h1 span { color:var(--mut); font-weight:400; }
  .st { display:flex; align-items:center; gap:8px; color:var(--mut); font-size:12px; letter-spacing:.06em; white-space:nowrap; }
  .dot { width:8px; height:8px; border-radius:50%; background:var(--fg); flex:none; }
  .dot.off { background:none; box-shadow:inset 0 0 0 1.5px var(--mut); }
  .dot.err { background:var(--acc); }

  .layout { display:grid; grid-template-columns:minmax(0,1fr); gap:16px; align-items:start; }
  @media (min-width:900px) { .layout { grid-template-columns:minmax(0,1fr) 320px; } .wrap { padding:24px; } }
  .panel { border:1px solid var(--line); border-radius:4px; background:var(--panel); }
  .sec { display:flex; justify-content:space-between; align-items:center; gap:12px; padding:12px 16px 8px;
         color:var(--mut); font-size:11px; letter-spacing:.1em; text-transform:uppercase; }

  .seg { display:flex; flex-wrap:wrap; }
  .seg button { font:inherit; font-size:12px; letter-spacing:0; text-transform:none; color:var(--mut);
                background:none; border:1px solid var(--line2); margin-left:-1px; padding:2px 8px; cursor:pointer; }
  .seg button:first-child { margin-left:0; border-radius:4px 0 0 4px; }
  .seg button:last-child { border-radius:0 4px 4px 0; }
  .seg button:hover { background:var(--hov); color:var(--fg); }
  .seg button.on { color:var(--bg); background:var(--fg); border-color:var(--fg); position:relative; }
  .seg button:focus-visible { outline:2px solid var(--acc); outline-offset:2px; position:relative; z-index:1; }

  #radar { position:relative; width:100%; aspect-ratio:1/1; max-height:80vh; margin:0 auto; border-top:1px solid var(--line); }
  canvas { position:absolute; inset:0; width:100%; height:100%; display:block; }

  .r { display:flex; justify-content:space-between; align-items:baseline; gap:12px; padding:8px 16px;
       border-top:1px solid var(--line); }
  .r:hover { background:var(--hov); }
  .r > span:first-child { color:var(--mut); font-size:12px; }
  .r .v { font-size:14px; text-align:right; }
  .r .v small { font-size:11px; color:var(--mut); margin-left:4px; }
  .r.err { box-shadow:inset 3px 0 0 var(--acc); }
  .r.err .v { color:var(--acc); }
  .gap { height:16px; }

  .note { margin:8px 16px 12px; padding:12px; border:1px dashed var(--line2); border-radius:2px;
          color:var(--mut); font-size:12px; line-height:18px; }
  .note b { display:block; color:var(--fg); font-weight:600; margin-bottom:2px; }
  .note.err { border-color:var(--acc); border-style:solid; }
  .note.err b { color:var(--acc); }
</style>
</head>
<body>
<div class="wrap">
  <div class="bar">
    <h1>MAZE2027 <span>/ лидар LD19</span></h1>
    <div class="st"><i class="dot off" id="conn-dot"></i><span id="conn">подключение</span></div>
  </div>
  <div class="layout">
    <div class="panel">
      <div class="sec"><span>Радар</span><div class="seg" id="ranges" role="group" aria-label="Масштаб"></div></div>
      <div id="note"></div>
      <div id="radar"><canvas id="c"></canvas></div>
    </div>
    <div>
      <div class="panel">
        <div class="sec"><span>Скан</span></div>
        <div class="r"><span>Оборотов</span><span class="v" id="hz">-</span></div>
        <div class="r"><span>Точек</span><span class="v" id="np">-</span></div>
        <div class="r"><span>Масштаб</span><span class="v" id="rng">-</span></div>
      </div>
      <div class="gap"></div>
      <div class="panel">
        <div class="sec"><span>Расстояния</span></div>
        <div class="r" id="r-near"><span>Ближайшее</span><span class="v" id="near">-</span></div>
        <div class="r"><span>Перёд</span><span class="v" id="s0">-</span></div>
        <div class="r"><span>Право</span><span class="v" id="s90">-</span></div>
        <div class="r"><span>Зад</span><span class="v" id="s180">-</span></div>
        <div class="r"><span>Лево</span><span class="v" id="s270">-</span></div>
      </div>
    </div>
  </div>
</div>
<script>
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
const MONO = 'ui-monospace,Menlo,Consolas,monospace';
const ACC = '#F2A900', FG = '#D7DADF', MUT = '#7E838C';
const NEAR_ALERT = 100;   // мм: ближе - считаем тревогой (почти касание)
const SECTOR = 20;        // полуширина сектора для Перёд/Право/Зад/Лево, град
let ranges = [500, 1000, 2000, 4000, 8000], range = 2000, auto = true;
let pts = [], near = null;
const $ = id => document.getElementById(id);

const rbox = $('ranges');
function mkbtn(label, fn, key) {
  const b = document.createElement('button');
  b.type = 'button'; b.textContent = label; b.dataset.k = key;
  b.onclick = () => { fn(); paintBtns(); draw(); };
  rbox.appendChild(b); return b;
}
const btns = [mkbtn('авто', () => { auto = true; }, 'auto')];
ranges.forEach(r => btns.push(mkbtn(r / 1000, () => { auto = false; range = r; }, r)));
function paintBtns() {
  btns.forEach(b => {
    const on = b.dataset.k === 'auto' ? auto : (!auto && +b.dataset.k === range);
    b.classList.toggle('on', on); b.setAttribute('aria-pressed', on);
  });
}
paintBtns();

function resize() {
  const d = devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
  cv.width = w * d; cv.height = h * d; ctx.setTransform(d, 0, 0, d, 0, 0); draw();
}
addEventListener('resize', resize);

function fmtM(mm) { return mm >= 1000 ? (mm / 1000).toFixed(1) + ' м' : Math.round(mm) + ' мм'; }

function draw() {
  const w = cv.clientWidth, h = cv.clientHeight;
  const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 40;   // запас под подписи сторон
  ctx.clearRect(0, 0, w, h);
  if (R < 20) return;
  const k = R / range;

  ctx.lineWidth = 1; ctx.font = '11px ' + MONO;
  ctx.strokeStyle = 'rgba(255,255,255,.08)'; ctx.fillStyle = MUT;
  const step = range / 4;
  for (let i = 1; i <= 4; i++) {
    const r = step * i * k;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, 6.2832); ctx.stroke();
    ctx.fillText(fmtM(step * i), cx + r * 0.7071 + 4, cy - r * 0.7071 - 3);   // по диагонали, не мешает подписи ПЕРЁД
  }
  for (let a = 0; a < 360; a += 30) {
    const rad = a * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.sin(rad) * R, cy - Math.cos(rad) * R); ctx.stroke();
  }
  ctx.textAlign = 'center';
  [['ПЕРЁД', 0], ['ПРАВО', 90], ['ЗАД', 180], ['ЛЕВО', 270]].forEach(([t, a]) => {
    const rad = a * Math.PI / 180;
    ctx.fillText(t, cx + Math.sin(rad) * (R + 16), cy - Math.cos(rad) * (R + 16) + 4);
  });
  ctx.textAlign = 'left';

  // контур препятствий; линия рвётся на скачке дальности или пропуске по углу - это проём
  ctx.strokeStyle = 'rgba(215,218,223,.3)'; ctx.lineWidth = 1.5;
  ctx.beginPath();
  let prev = null;
  pts.forEach(([a, d]) => {
    const rad = a * Math.PI / 180, r = Math.min(d * k, R);
    const x = cx + Math.sin(rad) * r, y = cy - Math.cos(rad) * r;
    const jump = !prev || Math.abs(d - prev[1]) > Math.max(250, d * 0.25)
                 || ((a - prev[0] + 360) % 360) > 5;
    jump ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    prev = [a, d];
  });
  ctx.stroke(); ctx.lineWidth = 1;

  // точки: яркость по интенсивности отражения (один цвет, без радуги)
  pts.forEach(([a, d, q]) => {
    const r = d * k;
    if (r > R) return;
    const rad = a * Math.PI / 180;
    ctx.fillStyle = 'rgba(215,218,223,' + (0.35 + Math.min(q, 255) / 255 * 0.65).toFixed(2) + ')';
    ctx.fillRect(cx + Math.sin(rad) * r - 1.5, cy - Math.cos(rad) * r - 1.5, 3, 3);
  });

  // ближайшая точка - единственное, что выделено акцентом
  if (near) {
    const rad = near[0] * Math.PI / 180, r = Math.min(near[1] * k, R);
    const x = cx + Math.sin(rad) * r, y = cy - Math.cos(rad) * r;
    ctx.strokeStyle = ACC; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke(); ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(x, y, 5, 0, 6.2832); ctx.lineWidth = 1.5; ctx.stroke();
  }

  // робот
  ctx.fillStyle = FG;
  ctx.beginPath(); ctx.moveTo(cx, cy - 9);
  ctx.lineTo(cx + 6, cy + 6); ctx.lineTo(cx - 6, cy + 6); ctx.closePath(); ctx.fill();
}

function sectorMin(center) {
  let best = null;
  pts.forEach(([a, d]) => {
    if (!d) return;
    const diff = Math.abs(((a - center + 540) % 360) - 180);
    if (diff <= SECTOR && (best === null || d < best)) best = d;
  });
  return best;
}
function setVal(id, mm) {
  $(id).innerHTML = mm === null ? '-' : Math.round(mm) + '<small>мм</small>';
}
function setConn(kind, text) {
  $('conn-dot').className = 'dot' + (kind === 'on' ? '' : ' ' + kind);
  $('conn').textContent = text;
}

function render(j) {
  pts = j.points || [];
  near = null;
  pts.forEach(p => { if (p[1] && (!near || p[1] < near[1])) near = p; });
  $('hz').innerHTML = j.hz ? j.hz.toFixed(1) + '<small>Гц</small>' : '-';
  $('np').textContent = pts.length || '-';
  $('near').innerHTML = near ? Math.round(near[1]) + '<small>мм @ ' + near[0].toFixed(0) + '°</small>' : '-';
  $('r-near').className = 'r' + (near && near[1] < NEAR_ALERT ? ' err' : '');
  setVal('s0', sectorMin(0)); setVal('s90', sectorMin(90));
  setVal('s180', sectorMin(180)); setVal('s270', sectorMin(270));
  if (auto && pts.length) {
    const ds = pts.map(p => p[1]).filter(d => d).sort((x, y) => x - y);
    const p90 = ds[Math.floor(ds.length * 0.9)] || 0;
    range = ranges.find(r => r >= p90 * 1.1) || ranges[ranges.length - 1];
  }
  $('rng').innerHTML = (range / 1000) + '<small>м</small>' + (auto ? '<small>авто</small>' : '');
  paintBtns();
  if (j.error) {
    setConn('err', 'ошибка');
    $('note').innerHTML = '<div class="note err"><b>Лидар не отвечает</b>' + String(j.error).replace(/[<>&]/g, '') + '. Пробуем снова каждую секунду.</div>';
  } else if (!pts.length) {
    setConn('off', 'ожидание данных');
    $('note').innerHTML = '<div class="note"><b>Нет оборотов</b>Порт открыт, но лидар ещё не прислал полный оборот.</div>';
  } else {
    setConn('on', 'онлайн');
    $('note').innerHTML = '';
  }
}

let lost = 0;
async function poll() {
  try {
    const r = await fetch('/scan', { cache: 'no-store' });
    if (!r.ok) throw new Error(r.status);
    render(await r.json()); lost = 0;
  } catch (e) {
    // 1-3 потери при поллинге 16 Гц - не повод мигать офлайном
    if (++lost > 3) {
      setConn('off', 'нет связи');
      $('note').innerHTML = '<div class="note err"><b>Нет связи с ld19_web.py</b>Последний скан остаётся на экране. Пробуем снова.</div>';
    }
  }
  draw();
  setTimeout(poll, 60);
}
resize();
poll();
</script>
</body>
</html>
"""


class Lidar(threading.Thread):
    """Фоновое чтение лидара: держит последний полный оборот."""

    daemon = True

    def __init__(self, port: str, baud: int, zero_dir: float):
        super().__init__()
        self.port, self.baud, self.zero_dir = port, baud, zero_dir
        self.lock = threading.Lock()
        self.points: list = []
        self.hz = 0.0
        self.error: str | None = None

    def run(self) -> None:
        while True:
            try:
                with LD19(self.port, self.baud, zero_dir=self.zero_dir) as lidar:
                    self.error = None
                    prev = time.monotonic()
                    for rev in lidar.revolutions():
                        now = time.monotonic()
                        dt, prev = now - prev, now
                        with self.lock:
                            self.points = [(round(p.angle, 2), p.distance,
                                            p.intensity) for p in rev]
                            if dt > 0:  # сглаживаем, иначе цифра дрожит
                                self.hz = 0.8 * self.hz + 0.2 / dt if self.hz else 1 / dt
            except Exception as exc:  # порт пропал или лидар отключён - ждём и пробуем снова
                self.error = f"{type(exc).__name__}: {exc}"
                time.sleep(1.0)

    def snapshot(self) -> dict:
        with self.lock:
            return {"points": self.points, "hz": self.hz, "error": self.error}


class Handler(BaseHTTPRequestHandler):
    lidar: Lidar

    def _send(self, body: bytes, ctype: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path.startswith("/scan"):
            self._send(json.dumps(self.lidar.snapshot()).encode(),
                       "application/json")
        elif self.path in ("/", "/index.html"):
            self._send(PAGE.encode(), "text/html; charset=utf-8")
        else:
            self.send_error(404)

    def log_message(self, *args) -> None:
        pass  # не засорять консоль запросами опроса


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="LD19 live radar")
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("-b", "--baud", type=int, default=BAUDRATE)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--zero-dir", type=float, default=ZERO_DIR,
                    help="куда смотрит нулевой угол лидара, град")
    args = ap.parse_args()

    lidar = Lidar(args.port, args.baud, args.zero_dir)
    lidar.start()
    Handler.lidar = lidar

    srv = ThreadingHTTPServer((args.host, args.http_port), Handler)
    print(f"радар на http://{args.host}:{args.http_port}  (лидар {args.port})",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
