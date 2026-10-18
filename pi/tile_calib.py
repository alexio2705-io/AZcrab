#!/usr/bin/env python3
"""Калибровка датчика цвета по плиткам на поле: с робота (экран, кнопки,
лента) или со страницы.

С робота (её запускают и из меню на экране, oled.py): на экране список
цветов и сколько образцов каждого уже снято, в шапке - что датчик видит
сейчас. Кнопки у малины (внизу экрана нарисованы, как стоят): ↑/↓ - цвет,
ЗАМЕР (крайняя правая, GPIO25; GPIO27 тоже) - замер. Лента светится
выбранным цветом, на время замера гаснет (не засвечивать датчик), потом
вспыхивает зелёным - сохранено или красным - нет. Последний пункт -
«выход в меню».

На странице - живые каналы APDS9960 и решение, которое по ним примет робот
(тот же tile_classify.classify(), что и в nav.py). Выбираешь цвет, ставишь
робота на плитку этого цвета, жмёшь ЗАМЕР - замер усредняется MEASURE_S
секунд и сразу пишется в ~/maze/tile_calib.json. nav.py читает этот файл
на старте.

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
import os
import threading
import time
from typing import Any

from flask import Flask, jsonify, request

import color as link            # разбор телеметрии ESP (тот же, что в color.py)
import tile_classify as tc

from oled import BTN_MODE, BTN_NEXT, BTN_OK, BTN_PREV  # noqa: E402

HTTP_PORT = 8002
BTN_GPIO = BTN_OK               # ЗАМЕР; BTN_MODE (GPIO27, как раньше) - тоже замер
MEASURE_S = 1.5
# Замер отбрасывается, если C за время усреднения гуляет сильнее этого
# (доля от среднего) - робота двигали или под ним стык плиток.
STABLE_FRAC = 0.12
RESULT_S = 4.0                  # столько на экране «сохранено/не сохранено»
FLASH_S = 1.0                   # столько лента горит зелёным/красным после замера
EXIT = "exit"                   # последний пункт списка - выход в меню

# Лента, пока выбран цвет: тем же цветом, неярко (как цвет плитки, а не
# сигнал). Чёрная и воздух - еле заметно, чтобы было видно, что лента жива.
LED_COLORS = {"white": (40, 40, 40), "black": (4, 4, 4), "blue": (0, 0, 70),
              "red": (70, 0, 0), "silver": (20, 28, 45), "air": (25, 12, 0)}
LED_OK, LED_BAD = (0, 90, 0), (90, 0, 0)

app = Flask(__name__)
_lock = threading.Lock()
_samples = tc.load()
_st = {"selected": tc.COLORS[0], "sel": 0, "busy": False, "msg": "", "msg_ok": True,
       "presses": 0, "progress": None, "result": None}


def _read():
    with link._lock:
        r, g, b, c, _color = link._last
    return r, g, b, c


def _say(msg, ok=True):
    print(("  " if ok else "  [!] ") + msg, flush=True)
    with _lock:
        _st["msg"], _st["msg_ok"] = msg, ok


class Lamp:
    """Лента: ровный цвет, вспышка на время, погасить. Ленты нет (нет
    spidev или /dev/spidev0.0) - всё молча ничего не делает."""

    def __init__(self):
        self.px = None
        self._lock = threading.Lock()
        self._base = (0, 0, 0)
        self._flash_until = 0.0
        try:
            import leds                 # без spidev leds.py делает sys.exit
            self.px = leds.NeoPixels()
        except (ImportError, SystemExit, OSError) as e:
            print(f"  [!] лента недоступна ({e}) - калибровка без неё", flush=True)

    def _show(self, rgb):
        if self.px is not None:
            self.px.fill(*rgb)
            self.px.show()

    def set(self, rgb):
        """Ровный цвет (после вспышки лента вернётся к нему)."""
        with self._lock:
            self._base = rgb
            if time.monotonic() >= self._flash_until:
                self._show(rgb)

    def flash(self, rgb, seconds=FLASH_S):
        with self._lock:
            self._flash_until = time.monotonic() + seconds
            self._show(rgb)

        def back():
            time.sleep(seconds)
            with self._lock:
                if time.monotonic() >= self._flash_until - 0.01:
                    self._show(self._base)
        threading.Thread(target=back, daemon=True).start()

    def off(self):
        with self._lock:
            self._flash_until = 0.0
            self._show((0, 0, 0))

    def close(self):
        if self.px is not None:
            try:
                self.px.close()
            except OSError:
                pass


_lamp = None


def _lamp_selected():
    """Лента - цветом выбранного пункта (выход - погашена)."""
    if _lamp is None:
        return
    with _lock:
        color = _st["selected"] if _st["sel"] < len(tc.COLORS) else None
    _lamp.set(LED_COLORS.get(color or "", (0, 0, 0)))


def _result(ok, text):
    with _lock:
        _st["result"] = (ok, text, time.monotonic() + RESULT_S)
    if _lamp is not None:
        _lamp.flash(LED_OK if ok else LED_BAD)


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
        _st["busy"], _st["progress"], _st["result"] = True, 0.0, None
    try:
        if _lamp is not None:
            _lamp.off()                 # лента не должна подсвечивать плитку
            time.sleep(0.1)
        _say(f"замер: {tc.NAMES[color]}, не трогай робота {MEASURE_S:.1f}с...")
        acc = [0.0] * 4
        cs = []
        t0 = time.time()
        end = t0 + MEASURE_S
        while time.time() < end:
            v = _read()
            for i in range(4):
                acc[i] += v[i]
            cs.append(v[3])
            with _lock:
                _st["progress"] = (time.time() - t0) / MEASURE_S
            time.sleep(0.02)
        n = len(cs)
        r, g, b, c = (x / n for x in acc)
        spread = (max(cs) - min(cs)) / max(c, 1.0)
        if c < tc.MIN_C and color != "air":
            _say(f"C={c:.0f} - датчик ничего не видит, замер не сохранён", False)
            _result(False, f"C={c:.0f} - датчик ничего не видит")
            return
        if spread > STABLE_FRAC:
            _say(f"C гулял на {spread*100:.0f}% (робот двигался?) - не сохранено, "
                 f"нажми ещё раз", False)
            _result(False, f"C гулял на {spread*100:.0f}% - робот двигался? "
                           f"Поставь ровно и нажми ЕЩЁ")
            return
        smp = {"r": round(r, 1), "g": round(g, 1), "b": round(b, 1), "c": round(c, 1),
               "n": n, "ts": time.strftime("%H:%M:%S")}
        with _lock:
            _samples.setdefault(color, []).append(smp)
            snapshot = {k: list(v) for k, v in _samples.items()}
        tc.save(snapshot)
        _say(f"сохранено: {tc.NAMES[color]} #{len(snapshot[color])}  "
             f"R={r:.0f} G={g:.0f} B={b:.0f} C={c:.0f}")
        _result(True, f"{_MID[color]} #{len(snapshot[color])}  C={c:.0f}")
    finally:
        with _lock:
            _st["busy"], _st["progress"] = False, None
        if _lamp is not None:
            _lamp.set(LED_COLORS.get(color, (0, 0, 0)))


def _exit_to_menu(keys):
    """«Выход в меню»: погасить ленту, отпустить кнопки, выйти. Меню на
    экране (oled.py) увидит, что калибровка кончилась, и покажется снова."""
    print("Выход в меню.", flush=True)
    if _lamp is not None:
        _lamp.close()
    if keys is not None:
        keys.close()
    try:
        link._link.close()
    except Exception:                   # noqa: BLE001
        pass
    os._exit(0)                         # flask в главном потоке - иначе не выйти


def on_key(pin, keys=None):
    """Кнопка у малины: ↑/↓ - пункт списка, ЗАМЕР - замер или выход."""
    n = len(tc.COLORS) + 1              # цвета + выход
    with _lock:
        _st["presses"] += 1
        if _st["busy"]:
            return                      # во время замера - ничего
        sel = _st["sel"]
        if pin in (BTN_PREV, BTN_NEXT):
            sel = (sel + (1 if pin == BTN_NEXT else -1)) % n
            _st["sel"], _st["result"] = sel, None
            if sel < len(tc.COLORS):
                _st["selected"] = tc.COLORS[sel]
    if pin in (BTN_PREV, BTN_NEXT):
        _lamp_selected()
        return
    if pin in (BTN_OK, BTN_MODE):
        if sel >= len(tc.COLORS):
            _exit_to_menu(keys)
        threading.Thread(target=measure, daemon=True).start()


# Коротко для дисплея сзади (oled.py): мелкий шрифт, 25 символов в строке.
# Двумя буквами, а не одной: строчная «з» рядом с цифрой читается как «3».
_SHORT = {"white": "бе", "black": "чё", "blue": "си", "red": "кр",
          "silver": "зе", "air": "во"}
_MID = {"white": "бел", "black": "чёрн", "blue": "син", "red": "крас",
        "silver": "зерк", "air": "возд"}


def calib_screen(st, samples, verdict, c, now) -> dict[str, Any]:
    """Что показать на экране робота -> аргументы oled.say(). Чистая
    функция: по ней же тест."""
    if st["busy"]:
        name = tc.NAMES[st["selected"]].split(" (")[0]
        return dict(title="ЗАМЕР - НЕ ТРОГАЙ", alert=True, progress=st["progress"] or 0.0,
                    lines=[f"цвет: {name}", f"C={c:.0f}  лента погашена"])
    res = st.get("result")
    if res and now < res[2]:
        return dict(title="СОХРАНЕНО" if res[0] else "НЕ СОХРАНЕНО", lines=[res[1]],
                    bar={BTN_PREV: "↑", BTN_NEXT: "↓", BTN_OK: "ЕЩЁ"})
    # «зеркальная (чекпоинт)» -> «зеркальная»: пояснение есть на странице
    items = [f"{tc.NAMES[k].split(' (')[0]:<15}{len(samples.get(k, [])):>2} шт" for k in tc.COLORS]
    items.append("← выход в меню")
    sel = st["sel"]
    return dict(title="ПЛИТКИ", right=f"видит {_MID.get(verdict, '?') if verdict else '?'}",
                items=items, sel=sel,
                bar={BTN_PREV: "↑", BTN_NEXT: "↓", BTN_OK: "ВЫХОД" if sel >= len(tc.COLORS) else "ЗАМЕР"})


def _oled_thread():
    """Экран калибровки на дисплее робота: стоишь у робота с кнопками -
    видно, какой цвет выбран, сколько образцов снято, что датчик видит
    сейчас и чем кончился замер, без телефона в руке."""
    import oled
    oled.ensure_daemon()
    while True:
        try:
            r, g, b, c = _read()
            with _lock:
                samples = {k: list(v) for k, v in _samples.items()}
                st = dict(_st)
            verdict = tc.classify(samples, r, g, b, c)[0]
            oled.say(ttl=1.5, **calib_screen(st, samples, verdict, c, time.monotonic()))
        except Exception as e:          # экран - не повод ронять калибровку
            print(f"  [!] дисплей: {type(e).__name__}: {e}", flush=True)
            time.sleep(5.0)
        time.sleep(0.1 if _st["busy"] else 0.25)


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
        _st["selected"], _st["sel"], _st["result"] = color, tc.COLORS.index(color), None
    _lamp_selected()
    _say(f"выбран цвет: {tc.NAMES[color]} - поставь робота и нажми ЗАМЕР на роботе")
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
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Калибровка цвета - MAZE2027</title>
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

  .wrap { max-width:760px; }
  .verdict { display:flex; align-items:center; gap:12px; padding:4px 16px 8px; }
  .verdict .sw { width:20px; height:20px; border-radius:2px; margin:0; flex:none; }
  .verdict .big { font-size:28px; line-height:32px; font-weight:600; }
  .verdict .big.none { color:var(--mut); font-weight:400; }
  #vdist, #raw { padding:0 16px; font-size:12px; color:var(--mut); }
  #raw { padding-bottom:12px; }
  .steps { padding:4px 16px 8px; color:var(--mut); font-size:12px; }
  .steps b { color:var(--fg); }
  .btns { display:flex; gap:8px; flex-wrap:wrap; padding:0 16px 8px; }
  .actions { display:flex; gap:12px; align-items:center; flex-wrap:wrap; padding:4px 16px 8px; }
  #presses { color:var(--mut); font-size:12px; }
  #path { color:var(--mut); letter-spacing:0; text-transform:none; font-size:11px; max-width:55%; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  @media (max-width:520px) { .t { display:none; } }
  #sep .r.err span:first-child { color:var(--acc); }
  .btn.x { min-height:24px; padding:0 8px; font-size:12px; }
</style></head>
<body>
<div class="wrap"><div class="bar"><h1>MAZE2027 <span>/ калибровка цвета</span></h1><div class="st"><i class="led off" id="conn-led"></i><span id="conn">подключение</span></div></div>
  <div class="stack">
    <div class="panel">
      <div class="sec"><span>Датчик сейчас</span></div>
      <div class="verdict"><i class="sw sq" id="sw"></i><span class="big none" id="verdict">-</span></div>
      <div id="vdist"></div>
      <div id="raw"></div>
    </div>
    <div class="panel">
      <div class="sec"><span>Замер</span></div>
      <div class="steps">1. Выбери цвет · 2. поставь робота на плитку · 3. нажми <b>ЗАМЕР</b> на роботе (крайняя правая кнопка у малины, GPIO__BTN__)</div>
      <div class="btns" id="colors"></div>
      <div class="actions"><button class="btn p" onclick="post('/api/measure')">Замерить без кнопки</button><span id="presses"></span></div>
      <div class="note" id="msg">-</div>
    </div>
    <div class="panel">
      <div class="sec"><span>Сохранённые образцы</span><span class="n" id="path"></span></div>
      <div class="tscroll"><table><thead><tr><th>цвет</th><th>#</th><th>R</th><th>G</th><th>B</th><th>C</th><th class="t">время</th><th></th></tr></thead>
      <tbody id="samples"></tbody></table></div>
    </div>
    <div class="panel">
      <div class="sec"><span>Насколько цвета различимы</span></div>
      <div id="sep"></div>
      <div class="hint">Ближе порога: шум будет путать цвета.</div>
    </div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
function esc(s) { return String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }
function setConn(kind, text) { $('conn-led').className = 'led' + (kind === 'on' ? '' : ' ' + kind); $('conn').textContent = text; }

const SW = {white:'#f4f4f4', black:'#111', blue:'#3a6cff', red:'#e03b3b', silver:'linear-gradient(135deg,#aaa,#fff,#999)', air:'transparent'};
let fails = 0;
async function post(url, body) { await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body || {})}); poll(true); }
function render(s) {
  const v = s.verdict;
  $('verdict').textContent = v ? s.names[v] : 'не знаю';
  $('verdict').className = 'big' + (v ? '' : ' none');
  $('sw').style.background = v ? SW[v] : 'transparent';
  $('vdist').textContent = s.dist == null ? (Object.keys(s.samples).length ? 'датчик ничего не видит' : 'образцов ещё нет') :
    `расстояние ${s.dist.toFixed(1)} (порог ${s.max_dist})` +
    // Вердикта нет - second это ближайший цвет, и dist уже до него; при
    // одном откалиброванном цвете dist2 нет вовсе (null) - раньше на нём
    // падала вся отрисовка и страница оставалась пустой.
    (!s.second ? '' : !v ? ` · ближайший: ${s.names[s.second]}` :
      ` · следующий: ${s.names[s.second]}` + (s.dist2 == null ? '' : ` ${s.dist2.toFixed(1)}`));
  const [r, g, b, c] = s.raw, f = s.frac;
  $('raw').textContent = `R=${r} G=${g} B=${b} C=${c}   r/c=${f[0].toFixed(3)} g/c=${f[1].toFixed(3)} b/c=${f[2].toFixed(3)} lnC=${s.lnc.toFixed(2)}`;
  const box = $('colors');
  box.innerHTML = '';
  for (const k of s.colors) {
    const n = (s.samples[k] || []).length;
    const bt = document.createElement('button');
    bt.className = 'btn' + (s.state.selected === k ? ' on' : '');
    bt.innerHTML = `<i class="sw sq" style="background:${SW[k]};margin:0"></i>${esc(s.names[k])} <span class="mut">${n}</span>`;
    bt.onclick = () => post('/api/select', {color:k});
    box.appendChild(bt);
  }
  const m = $('msg');
  m.textContent = (s.state.busy ? 'замер… ' : '') + (s.state.msg || 'выбери цвет');
  m.className = 'note' + (s.state.msg_ok ? '' : ' err');
  $('presses').textContent = `нажатий GPIO: ${s.state.presses}`;
  $('path').textContent = s.path;
  const tb = $('samples');
  tb.innerHTML = '';
  for (const k of s.colors) {
    (s.samples[k] || []).forEach((x, i) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td><i class="sw sq" style="background:${SW[k]}"></i>${esc(s.names[k])}</td><td>${i+1}</td><td>${Math.round(x.r)}</td><td>${Math.round(x.g)}</td><td>${Math.round(x.b)}</td><td>${Math.round(x.c)}</td><td class="mut t">${esc(x.ts || '')}</td>`;
      const td = document.createElement('td');
      const del = document.createElement('button'); del.className = 'btn x'; del.textContent = 'удалить';
      del.onclick = () => { if (confirm('Удалить образец?')) post('/api/delete', {color:k, idx:i}); };
      td.appendChild(del); tr.appendChild(td); tb.appendChild(tr);
    });
  }
  if (!tb.children.length) tb.innerHTML = '<tr><td colspan="7" class="mut" style="text-align:left;padding:12px 16px">Образцов пока нет.</td></tr>';
  const need = ['white', 'black'].filter(k => !s.samples[k]);
  $('sep').innerHTML = (need.length ? `<div class="r err"><span>без образцов «${need.map(k => esc(s.names[k])).join('», «')}» nav.py калибровку не возьмёт</span></div>` : '') +
    s.separation.map(([a, b, d]) => `<div class="r${d < s.sep_warn ? ' err' : ''}"><span>${esc(s.names[a])} — ${esc(s.names[b])}</span><span class="v">${d.toFixed(1)}${d < s.sep_warn ? '<small>близко</small>' : ''}</span></div>`).join('');
}
async function poll(once) {
  try { const r = await fetch('/api/state', {cache:'no-store'}); render(await r.json()); fails = 0; setConn('on', 'онлайн'); }
  catch (e) { if (++fails >= 3) setConn('off', 'нет связи'); }
  if (!once) setTimeout(poll, 300);
}
poll();
</script>
</body></html>
"""


def main():
    link._connect()
    print(f"Телеметрия идёт. Образцов в {tc.CALIB_PATH}: "
          f"{sum(len(v) for v in _samples.values())}")
    global _lamp
    _lamp = Lamp()
    _lamp_selected()
    import pi_keys
    keys = None

    def press(pin):
        on_key(pin, keys)
    keys = pi_keys.Keys(press)
    if not keys.ok:
        _say(f"кнопки недоступны ({keys.error}) - замер только с кнопки на странице", False)
    threading.Thread(target=_oled_thread, daemon=True).start()
    print(f"Страница: http://<ip малины>:{HTTP_PORT}   на роботе: ↑ GPIO{BTN_PREV}, "
          f"↓ GPIO{BTN_NEXT}, ЗАМЕР GPIO{BTN_GPIO} (и GPIO{BTN_MODE})")
    try:
        app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
    finally:
        _lamp.close()
        keys.close()
        link._link.close()


if __name__ == "__main__":
    main()
