"""Кнопки у малины для программ, которые управляются с робота (меню на
экране запускает их, см. oled.py): tile_calib.py, neopixel_test.py.

Пины и где они стоят - в oled.py (BTN_PREV/BTN_NEXT/BTN_MODE/BTN_OK). У
каждой кнопки на плате своя подтяжка 10к и RC-фильтр, поэтому хватает
опроса раз в 10 мс и 30 мс низкого уровня.

Нажатие засчитывается только после того, как кнопку видели отпущенной:
программу запускают из меню кнопкой 25, и пока палец ещё на ней, она уже
читается этой программой - без этого запуск калибровки сразу делал замер.
"""
from __future__ import annotations

import threading
import time

from oled import BTN_MODE, BTN_NEXT, BTN_OK, BTN_PREV

PINS = (BTN_PREV, BTN_NEXT, BTN_MODE, BTN_OK)
POLL_S = 0.01
HOLD_S = 0.03


class Debounce:
    """Одна кнопка: уровень -> нажатие (True один раз на удержание)."""

    def __init__(self):
        self.armed = False
        self.low_since = None

    def feed(self, low, now):
        if not low:
            self.armed, self.low_since = True, None
            return False
        if self.low_since is None:
            self.low_since = now
        if self.armed and now - self.low_since >= HOLD_S:
            self.armed = False
            return True
        return False


class Keys:
    """Опрос кнопок в своём потоке; on_press(пин) зовётся из него же - долгое
    (замер) запускать в отдельном потоке. ok = False - кнопок нет (не Pi,
    нет прав): программа работает со страницы/терминала."""

    def __init__(self, on_press, pins=PINS):
        self.on_press = on_press
        self.pins = tuple(pins)
        self.ok = False
        self.error = None
        try:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for p in self.pins:
                GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self._gpio = GPIO
            self.ok = True
        except Exception as e:          # noqa: BLE001 - не на Pi, пин занят
            self.error = f"{type(e).__name__}: {e}"
            return
        threading.Thread(target=self._run, daemon=True, name="keys").start()

    def _run(self):
        deb = {p: Debounce() for p in self.pins}
        while True:
            now = time.monotonic()
            for p in self.pins:
                try:
                    low = self._gpio.input(p) == self._gpio.LOW
                except RuntimeError:
                    continue
                if deb[p].feed(low, now):
                    try:
                        self.on_press(p)
                    except Exception as e:      # noqa: BLE001 - кнопка не должна ронять опрос
                        print(f"  [!] кнопка GPIO{p}: {type(e).__name__}: {e}", flush=True)
            time.sleep(POLL_S)

    def close(self):
        if self.ok:
            try:
                self._gpio.cleanup(list(self.pins))
            except Exception:           # noqa: BLE001
                pass
