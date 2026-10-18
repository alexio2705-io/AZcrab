#!/usr/bin/env python3
"""Тест ленты с робота: экран и кнопки у малины (его же запускает меню на
экране, oled.py). Сам драйвер и мигания живут в leds.py - здесь только
перебор режимов, чтобы не держать две копии кода: тот же модуль
импортирует nav.py.

Кнопки (внизу экрана нарисованы, как стоят): ←/→ - режим, ВЫХОД - выход в
меню. Режимы: чистые красный, зелёный, синий, белый (видно, что живы все
три канала каждого светодиода), «бегущий огонь» по одному (видно, какой
по счёту не горит), мигание жертвы и мигание возврата на старт - ровно те,
что в заезде.

Без кнопок (не Pi, нет прав) режимы листаются сами раз в AUTO_S секунд.

Запуск:  ~/maze/venv/bin/python ~/maze/neopixel_test.py
"""
from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any

import leds
import oled
import pi_keys
from oled import BTN_MODE, BTN_NEXT, BTN_OK, BTN_PREV

AUTO_S = 4.0
LEVEL = 120                    # чистые цвета - не на полную: 4 x 60 мА на белом


def _solid(rgb):
    def run(px, t):
        px.fill(*rgb)
    return run


def _chase(px, t):
    k = int(t / 0.5) % px.count
    px.buf = [(LEVEL, LEVEL, LEVEL) if i == k else (0, 0, 0) for i in range(px.count)]


def _blink(on_s, off_s):
    def run(px, t):
        px.fill(*(leds.PINK if t % (on_s + off_s) < on_s else (0, 0, 0)))
    return run


# (название, что видно, функция кадра)
MODES = [
    ("красный", "горят все {n}", _solid((LEVEL, 0, 0))),
    ("зелёный", "горят все {n}", _solid((0, LEVEL, 0))),
    ("синий", "горят все {n}", _solid((0, 0, LEVEL))),
    ("белый", "горят все {n}", _solid((LEVEL, LEVEL, LEVEL))),
    ("по одному", "горит №{k} из {n}", _chase),
    ("жертва", "розовый 0.5/0.5 с", _blink(leds.BLINK_ON_S, leds.BLINK_OFF_S)),
    ("возврат на старт", "розовый 1/1 с", _blink(leds.RETURN_BLINK_ON_S, leds.RETURN_BLINK_OFF_S)),
]


class Test:
    def __init__(self):
        self.mode = 0
        self.t0 = time.monotonic()
        self.quit = False
        self._lock = threading.Lock()

    def on_key(self, pin):
        with self._lock:
            if pin in (BTN_PREV, BTN_NEXT):
                self.mode = (self.mode + (1 if pin == BTN_NEXT else -1)) % len(MODES)
                self.t0 = time.monotonic()
            elif pin == BTN_OK:
                self.quit = True

    def screen(self, n, now) -> dict[str, Any]:
        """Аргументы oled.say() для текущего режима."""
        name, what, _ = MODES[self.mode]
        k = int((now - self.t0) / 0.5) % n + 1
        return dict(title="ТЕСТ ЛЕНТЫ", right=f"{self.mode + 1}/{len(MODES)}",
                    lines=[name, what.format(n=n, k=k)],
                    bar={BTN_PREV: "←", BTN_NEXT: "→", BTN_OK: "ВЫХОД"})


def main():
    try:
        px = leds.NeoPixels()
    except FileNotFoundError:
        msg = "нет /dev/spidev0.0 - нужен dtoverlay=spi0-0cs,no_miso"
        print(f"[FATAL] {msg}")
        oled.say("ТЕСТ ЛЕНТЫ", ["лента недоступна", msg], ttl=8.0)
        time.sleep(3.0)
        sys.exit(1)
    oled.ensure_daemon()
    test = Test()
    keys = pi_keys.Keys(test.on_key)
    if not keys.ok:
        print(f"кнопки недоступны ({keys.error}) - режимы листаются сами; Ctrl+C - выход")
    print("Тест ленты: ← GPIO%d, → GPIO%d, выход GPIO%d (GPIO%d не занята)"
          % (BTN_PREV, BTN_NEXT, BTN_OK, BTN_MODE))
    shown = None
    try:
        while not test.quit:
            now = time.monotonic()
            with test._lock:
                if not keys.ok and now - test.t0 >= AUTO_S:
                    test.mode, test.t0 = (test.mode + 1) % len(MODES), now
                mode, t = test.mode, now - test.t0
            if mode != shown:
                print(f"  режим {mode + 1}/{len(MODES)}: {MODES[mode][0]}", flush=True)
                shown = mode
            MODES[mode][2](px, t)
            px.show()
            oled.say(ttl=1.0, **test.screen(px.count, now))
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nОстановлено.")
    finally:
        px.close()
        keys.close()
        oled.say("ТЕСТ ЛЕНТЫ", ["выход"], ttl=0.3)
    print("Выход в меню.")
    os._exit(0)


if __name__ == "__main__":
    main()
