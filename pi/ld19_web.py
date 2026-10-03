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
<meta charset="utf-8">
<title>LD19</title>
<style>
  :root { --bg:#0b0f14; --fg:#e6edf3; --dim:#7d8796; --grid:#1e2732;
          --accent:#4cc2ff; --warn:#ff5d5d; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg); height:100vh;
         font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;
         display:flex; flex-direction:column; overflow:hidden; }
  header { display:flex; gap:18px; align-items:baseline; flex-wrap:wrap;
           padding:10px 16px; border-bottom:1px solid var(--grid); }
  h1 { font-size:13px; margin:0; font-weight:600; letter-spacing:.08em; }
  .stat b { color:var(--accent); font-weight:600; }
  .stat.bad b { color:var(--warn); }
  .spacer { flex:1; }
  button { font:inherit; color:var(--dim); background:transparent; cursor:pointer;
           border:1px solid var(--grid); border-radius:5px; padding:3px 9px; }
  button.on { color:var(--bg); background:var(--accent); border-color:var(--accent); }
  #wrap { flex:1; position:relative; min-height:0; }
  canvas { position:absolute; inset:0; width:100%; height:100%; }
</style>
<header>
  <h1>LD19</h1>
  <span class="stat">оборотов <b id="hz">-</b> Гц</span>
  <span class="stat">точек <b id="np">-</b></span>
  <span class="stat">ближайшее <b id="near">-</b></span>
  <span class="stat" id="linkstat">связь <b id="link">ждём</b></span>
  <span class="spacer"></span>
  <span id="ranges"></span>
</header>
<div id="wrap"><canvas id="c"></canvas></div>
<script>
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
let ranges = [500, 1000, 2000, 4000, 8000], range = 2000, auto = true;
let pts = [], hz = 0, lost = 0;

const rbox = document.getElementById('ranges');
function mkbtn(label, fn, on) {
  const b = document.createElement('button');
  b.textContent = label; b.onclick = () => { fn(); paintBtns(); };
  b.dataset.on = on; rbox.appendChild(b); return b;
}
const btns = [mkbtn('авто', () => auto = true, 'auto')];
ranges.forEach(r => btns.push(mkbtn(r / 1000 + ' м', () => { auto = false; range = r; }, r)));
function paintBtns() {
  btns.forEach(b => b.classList.toggle('on',
    b.dataset.on === 'auto' ? auto : (!auto && +b.dataset.on === range)));
}
paintBtns();

function resize() {
  const d = devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
  cv.width = w * d; cv.height = h * d; ctx.setTransform(d, 0, 0, d, 0, 0);
}
addEventListener('resize', resize); resize();

function draw() {
  const w = cv.clientWidth, h = cv.clientHeight;
  const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 52;
  ctx.clearRect(0, 0, w, h);
  const k = R / range;

  // кольца дальности
  ctx.strokeStyle = '#1e2732'; ctx.fillStyle = '#7d8796';
  ctx.lineWidth = 1; ctx.font = '11px ui-monospace,monospace';
  const step = range / 4;
  for (let i = 1; i <= 4; i++) {
    const r = step * i * k;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, 6.284); ctx.stroke();
    const lab = step * i >= 1000 ? (step * i / 1000).toFixed(1) + ' м'
                                 : Math.round(step * i) + ' мм';
    ctx.fillText(lab, cx + 4, cy - r - 3);
  }
  // лучи через 30°, подписи по сторонам
  for (let a = 0; a < 360; a += 30) {
    const rad = a * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.sin(rad) * R, cy - Math.cos(rad) * R); ctx.stroke();
  }
  ctx.fillStyle = '#7d8796'; ctx.textAlign = 'center';
  [['перёд', 0], ['право', 90], ['зад', 180], ['лево', 270]].forEach(([t, a]) => {
    const rad = a * Math.PI / 180;
    ctx.fillText(t, cx + Math.sin(rad) * (R + 18), cy - Math.cos(rad) * (R + 18) + 4);
  });
  ctx.textAlign = 'left';

  // контур препятствий
  ctx.strokeStyle = 'rgba(76,194,255,.45)'; ctx.lineWidth = 1.5;
  ctx.beginPath();
  let prev = null;
  pts.forEach(([a, d]) => {
    const rad = a * Math.PI / 180, r = Math.min(d * k, R);
    const x = cx + Math.sin(rad) * r, y = cy - Math.cos(rad) * r;
    // рвём линию на скачке дальности или пропуске по углу: это проём, а не стена
    const jump = !prev || Math.abs(d - prev[1]) > Math.max(250, d * 0.25)
                 || ((a - prev[0] + 360) % 360) > 5;
    jump ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    prev = [a, d];
  });
  ctx.stroke(); ctx.lineWidth = 1;
  // точки, яркость по интенсивности отражения
  pts.forEach(([a, d, q]) => {
    const rad = a * Math.PI / 180, r = d * k;
    if (r > R) return;
    const hue = 200 - Math.min(q, 255) / 255 * 160;
    ctx.fillStyle = 'hsl(' + hue + ' 90% 60%)';
    ctx.fillRect(cx + Math.sin(rad) * r - 1.5, cy - Math.cos(rad) * r - 1.5, 3, 3);
  });
  // робот
  ctx.fillStyle = '#e6edf3';
  ctx.beginPath(); ctx.moveTo(cx, cy - 9);
  ctx.lineTo(cx + 6, cy + 6); ctx.lineTo(cx - 6, cy + 6); ctx.closePath(); ctx.fill();
}

async function poll() {
  try {
    const r = await fetch('/scan', { cache: 'no-store' });
    const j = await r.json();
    pts = j.points; hz = j.hz; lost = 0;
    document.getElementById('hz').textContent = hz.toFixed(1);
    document.getElementById('np').textContent = pts.length;
    let best = null;
    pts.forEach(([a, d]) => { if (d && (!best || d < best[1])) best = [a, d]; });
    document.getElementById('near').textContent =
      best ? best[1] + ' мм @ ' + best[0].toFixed(0) + '°' : '-';
    if (auto && pts.length) {
      const ds = pts.map(p => p[1]).filter(d => d).sort((x, y) => x - y);
      const p90 = ds[Math.floor(ds.length * 0.9)] || 0;
      range = ranges.find(r => r >= p90 * 1.1) || ranges[ranges.length - 1];
    }
    document.getElementById('link').textContent = 'есть';
    document.getElementById('linkstat').classList.remove('bad');
  } catch (e) {
    if (++lost > 3) {
      document.getElementById('link').textContent = 'нет';
      document.getElementById('linkstat').classList.add('bad');
    }
  }
  draw();
  setTimeout(poll, 60);
}
poll();
</script>
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
