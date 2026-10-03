#!/usr/bin/env python3
"""Калибровка датчика цвета по плиткам на поле: страница + кнопка GPIO27.

На странице - живые каналы APDS9960 и решение, которое по ним примет робот
(тот же tile_classify.classify(), что и в nav.py). Выбираешь цвет, ставишь
робота на плитку этого цвета, жмёшь кнопку GPIO27 - замер усредняется
MEASURE_S секунд и сразу пишется в ~/maze/tile_calib.json. nav.py читает
этот файл на старте.

Каждый цвет можно снять несколько раз на разных плитках (и в разных местах
зала) - в памяти хранятся все образцы, и распознавание идёт по ближайшему.
Зеркальную (чекпоинт) стоит снять на двух-трёх плитках: отражение зависит
от того, что над ней.

Порт /dev/serial0 один, поэтому одновременно с nav.py не запускать.

Запуск на малине:
    ~/maze/venv/bin/python ~/maze/tile_calib.py
и открыть http://<ip малины>:8002
"""
from __future__ import annotations

import math
import threading
import time

from flask import Flask, jsonify, request

import color as link            # разбор телеметрии ESP (тот же, что в color.py)
import tile_classify as tc

HTTP_PORT = 8002
BTN_GPIO = 27                   # к GND при нажатии, подтяжка - внутренняя
MEASURE_S = 1.5
# Замер отбрасывается, если C за время усреднения гуляет сильнее этого
# (доля от среднего) - робота двигали или под ним стык плиток.
STABLE_FRAC = 0.12

app = Flask(__name__)
_lock = threading.Lock()
_samples = tc.load()
_st = {"selected": None, "busy": False, "msg": "", "msg_ok": True, "presses": 0}


def _read():
    with link._lock:
        r, g, b, c, _color = link._last
    return r, g, b, c


def _say(msg, ok=True):
    print(("  " if ok else "  [!] ") + msg, flush=True)
    with _lock:
        _st["msg"], _st["msg_ok"] = msg, ok


def measure():
    """Замер выбранного цвета: среднее за MEASURE_S, проверка, что робот
    стоял, запись в файл. Вызывается и кнопкой, и со страницы."""
    with _lock:
        color = _st["selected"]
        if _st["busy"]:
            return
        if color is None:
            _st["msg"], _st["msg_ok"] = "сначала выбери цвет на странице", False
            return
        _st["busy"] = True
    try:
        _say(f"замер: {tc.NAMES[color]}, не трогай робота {MEASURE_S:.1f}с...")
        acc = [0.0] * 4
        cs = []
        end = time.time() + MEASURE_S
        while time.time() < end:
            v = _read()
            for i in range(4):
                acc[i] += v[i]
            cs.append(v[3])
            time.sleep(0.02)
        n = len(cs)
        r, g, b, c = (x / n for x in acc)
        spread = (max(cs) - min(cs)) / max(c, 1.0)
        if c < tc.MIN_C and color != "air":
            _say(f"C={c:.0f} - датчик ничего не видит, замер не сохранён", False)
            return
        if spread > STABLE_FRAC:
            _say(f"C гулял на {spread*100:.0f}% (робот двигался?) - не сохранено, "
                 f"нажми ещё раз", False)
            return
        smp = {"r": round(r, 1), "g": round(g, 1), "b": round(b, 1), "c": round(c, 1),
               "n": n, "ts": time.strftime("%H:%M:%S")}
        with _lock:
            _samples.setdefault(color, []).append(smp)
            snapshot = {k: list(v) for k, v in _samples.items()}
        tc.save(snapshot)
        _say(f"сохранено: {tc.NAMES[color]} #{len(snapshot[color])}  "
             f"R={r:.0f} G={g:.0f} B={b:.0f} C={c:.0f}")
    finally:
        with _lock:
            _st["busy"] = False


def _button_thread():
    """Опрос GPIO27 раз в 10мс, 30мс антидребезга, одно срабатывание на
    нажатие. Без RPi.GPIO (прав нет, не малина) - только кнопка на странице."""
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BTN_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    except Exception as e:
        _say(f"GPIO{BTN_GPIO} недоступен ({e}) - замер только с кнопки на странице", False)
        return
    pressed = False
    low_since = None
    while True:
        low = GPIO.input(BTN_GPIO) == GPIO.LOW
        now = time.monotonic()
        if low:
            low_since = low_since or now
            if not pressed and now - low_since >= 0.03:
                pressed = True
                with _lock:
                    _st["presses"] += 1
                threading.Thread(target=measure, daemon=True).start()
        else:
            low_since = None
            pressed = False
        time.sleep(0.01)


@app.route("/api/state")
def api_state():
    r, g, b, c = _read()
    with _lock:
        samples = {k: list(v) for k, v in _samples.items()}
        st = dict(_st)
    verdict, d, second, d2 = tc.classify(samples, r, g, b, c)
    cf = max(c, 1)
    return jsonify({
        "raw": [r, g, b, c],
        "frac": [r / cf, g / cf, b / cf],
        "lnc": math.log(cf),
        "verdict": verdict, "dist": d, "second": second, "dist2": d2,
        "max_dist": tc.MAX_DIST, "sep_warn": tc.SEP_WARN,
        "state": st,
        "colors": tc.COLORS, "names": tc.NAMES,
        "samples": samples,
        "separation": tc.separation(samples),
        "path": tc.CALIB_PATH,
    })


@app.route("/api/select", methods=["POST"])
def api_select():
    color = (request.get_json(silent=True) or {}).get("color")
    if color not in tc.COLORS:
        return jsonify({"error": "нет такого цвета"}), 400
    with _lock:
        _st["selected"] = color
    _say(f"выбран цвет: {tc.NAMES[color]} - поставь робота и нажми GPIO{BTN_GPIO}")
    return jsonify({"ok": True})


@app.route("/api/measure", methods=["POST"])
def api_measure():
    threading.Thread(target=measure, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/delete", methods=["POST"])
def api_delete():
    """Удалить один образец (color, idx) или все образцы цвета (idx=null)."""
    body = request.get_json(silent=True) or {}
    color, idx = body.get("color"), body.get("idx")
    with _lock:
        lst = _samples.get(color)
        if not lst:
            return jsonify({"error": "нечего удалять"}), 400
        if idx is None:
            _samples.pop(color)
        elif 0 <= idx < len(lst):
            lst.pop(idx)
            if not lst:
                _samples.pop(color)
        snapshot = {k: list(v) for k, v in _samples.items()}
    tc.save(snapshot)
    _say(f"удалено: {tc.NAMES.get(color, color)}" + ("" if idx is None else f" #{idx + 1}"))
    return jsonify({"ok": True})


@app.route("/")
def index():
    return PAGE.replace("__BTN__", str(BTN_GPIO))


PAGE = r"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Калибровка цвета</title>
<style>
:root{--bg:#0f1115;--card:#171a21;--line:#2a2f3a;--txt:#e8eaee;--dim:#8a93a3;--ok:#4ee27a;--bad:#ff5c5c;--acc:#ffd23f}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);font:15px/1.4 system-ui,sans-serif;padding:16px;max-width:900px;margin:auto}
h1{font-size:20px;margin:0 0 12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:12px}
.big{font-size:34px;font-weight:800;letter-spacing:.5px}
.dim{color:var(--dim)}
.mono{font-family:ui-monospace,Menlo,monospace}
.row{display:flex;gap:16px;flex-wrap:wrap;align-items:center}
.btns{display:flex;gap:8px;flex-wrap:wrap}
button{background:#232833;color:var(--txt);border:1px solid var(--line);border-radius:8px;padding:10px 12px;font-size:14px;cursor:pointer}
button.sel{border-color:var(--acc);box-shadow:0 0 0 2px var(--acc) inset}
button.go{background:var(--acc);color:#111;font-weight:700;border:none}
button.x{padding:2px 8px;font-size:12px}
.sw{display:inline-block;width:14px;height:14px;border-radius:3px;vertical-align:-2px;margin-right:6px;border:1px solid #0006}
table{border-collapse:collapse;width:100%}
td,th{padding:4px 6px;border-bottom:1px solid var(--line);text-align:left;font-size:13px}
.msg{padding:8px 10px;border-radius:8px;background:#1d2a1f}
.msg.bad{background:#2d1a1a;color:#ffb3b3}
.warn{color:var(--bad)}
</style></head><body>
<h1>Калибровка датчика цвета</h1>

<div class="card">
  <div class="row">
    <div><span class="sw" id="sw"></span><span class="big" id="verdict">—</span></div>
    <div class="dim" id="vdist"></div>
  </div>
  <div class="mono dim" id="raw" style="margin-top:6px"></div>
</div>

<div class="card">
  <div style="margin-bottom:8px">1. Выбери цвет · 2. поставь робота на плитку · 3. нажми кнопку <b>GPIO__BTN__</b></div>
  <div class="btns" id="colors"></div>
  <div style="margin-top:10px" class="row">
    <button class="go" onclick="post('/api/measure')">Замерить без кнопки</button>
    <span class="dim" id="presses"></span>
  </div>
  <div class="msg" id="msg" style="margin-top:10px">…</div>
</div>

<div class="card">
  <div style="margin-bottom:6px"><b>Сохранённые образцы</b> <span class="dim mono" id="path"></span></div>
  <table><thead><tr><th>цвет</th><th>#</th><th>R</th><th>G</th><th>B</th><th>C</th><th>время</th><th></th></tr></thead>
  <tbody id="samples"></tbody></table>
</div>

<div class="card">
  <div style="margin-bottom:6px"><b>Насколько цвета различимы</b> <span class="dim">(ближе порога — шум будет путать)</span></div>
  <div id="sep" class="mono"></div>
</div>

<script>
const SW={white:'#f4f4f4',black:'#111',blue:'#3a6cff',red:'#e03b3b',silver:'linear-gradient(135deg,#aaa,#fff,#999)',air:'transparent'};
let last=null;
async function post(url,body){await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});poll(true);}
function esc(s){return String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}
function render(s){
  const v=s.verdict;
  document.getElementById('verdict').textContent=v?s.names[v]:'не знаю';
  document.getElementById('sw').style.background=v?SW[v]:'transparent';
  document.getElementById('vdist').textContent=s.dist==null?(Object.keys(s.samples).length?'датчик ничего не видит':'образцов ещё нет'):
    `расстояние ${s.dist.toFixed(1)} (порог ${s.max_dist})`+
    // Вердикта нет - second это ближайший цвет, и dist уже до него; при
    // одном откалиброванном цвете dist2 нет вовсе (null) - раньше на нём
    // падала вся отрисовка и страница оставалась пустой.
    (!s.second?'':!v?` · ближайший: ${s.names[s.second]}`:
      ` · следующий: ${s.names[s.second]}`+(s.dist2==null?'':` ${s.dist2.toFixed(1)}`));
  const [r,g,b,c]=s.raw, f=s.frac;
  document.getElementById('raw').textContent=`R=${r} G=${g} B=${b} C=${c}   r/c=${f[0].toFixed(3)} g/c=${f[1].toFixed(3)} b/c=${f[2].toFixed(3)} lnC=${s.lnc.toFixed(2)}`;
  const box=document.getElementById('colors');
  box.innerHTML='';
  for(const k of s.colors){
    const n=(s.samples[k]||[]).length;
    const bt=document.createElement('button');
    bt.className=s.state.selected===k?'sel':'';
    bt.innerHTML=`<span class="sw" style="background:${SW[k]}"></span>${esc(s.names[k])} <span class="dim">(${n})</span>`;
    bt.onclick=()=>post('/api/select',{color:k});
    box.appendChild(bt);
  }
  const m=document.getElementById('msg');
  m.textContent=(s.state.busy?'⏳ ':'')+(s.state.msg||'выбери цвет');
  m.className='msg'+(s.state.msg_ok?'':' bad');
  document.getElementById('presses').textContent=`нажатий GPIO: ${s.state.presses}`;
  document.getElementById('path').textContent=s.path;
  const tb=document.getElementById('samples');
  tb.innerHTML='';
  for(const k of s.colors){
    (s.samples[k]||[]).forEach((x,i)=>{
      const tr=document.createElement('tr');
      tr.innerHTML=`<td><span class="sw" style="background:${SW[k]}"></span>${esc(s.names[k])}</td><td>${i+1}</td><td>${Math.round(x.r)}</td><td>${Math.round(x.g)}</td><td>${Math.round(x.b)}</td><td>${Math.round(x.c)}</td><td class="dim">${esc(x.ts||'')}</td>`;
      const td=document.createElement('td');
      const del=document.createElement('button');del.className='x';del.textContent='удалить';
      del.onclick=()=>{if(confirm('Удалить образец?'))post('/api/delete',{color:k,idx:i});};
      td.appendChild(del);tr.appendChild(td);tb.appendChild(tr);
    });
  }
  const need=['white','black'].filter(k=>!s.samples[k]);
  const sep=document.getElementById('sep');
  sep.innerHTML=(need.length?`<div class="warn">без образцов «${need.map(k=>s.names[k]).join('», «')}» nav.py калибровку не возьмёт</div>`:'')+
    s.separation.map(([a,b,d])=>`<div class="${d<s.sep_warn?'warn':''}">${esc(s.names[a])} — ${esc(s.names[b])}: ${d.toFixed(1)}${d<s.sep_warn?'  ← близко!':''}</div>`).join('');
}
async function poll(once){
  try{const r=await fetch('/api/state',{cache:'no-store'});render(await r.json());}catch(e){}
  if(!once)setTimeout(poll,300);
}
poll();
</script></body></html>
"""


def main():
    link._connect()
    print(f"Телеметрия идёт. Образцов в {tc.CALIB_PATH}: "
          f"{sum(len(v) for v in _samples.values())}")
    threading.Thread(target=_button_thread, daemon=True).start()
    print(f"Страница: http://<ip малины>:{HTTP_PORT}   кнопка замера: GPIO{BTN_GPIO}")
    try:
        app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
    finally:
        link._link.close()


if __name__ == "__main__":
    main()
