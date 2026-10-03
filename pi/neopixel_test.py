#!/usr/bin/env python3
"""Тест ленты. Сам драйвер и индикация живут в leds.py - здесь только запуск,
чтобы не держать две копии кода: тот же модуль импортирует nav.py.

Запуск:  ~/maze/venv/bin/python ~/maze/neopixel_test.py
"""

from leds import main

if __name__ == "__main__":
    main()
