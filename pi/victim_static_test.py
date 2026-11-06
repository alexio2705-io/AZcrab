#!/usr/bin/env python3
"""Статический тест детектора жертв — робот стоит на месте, без езды.

Каждая итерация вызывает scan() с новым номером клетки, иначе дедупликация
внутри VictimDetector заглушит повторные проверки одной и той же сцены.

Запуск:  python3 victim_static_test.py [число_итераций]
Без аргумента — бесконечно, до Ctrl+C.
"""

import os
import sys
import time

import cv2

from victim_detect import VictimDetector, SAVE_DIR

PERIOD = 1.0


def main():
    limit = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    det = VictimDetector()
    print(f'[TEST] Старт (итераций: {limit or "∞"}), Ctrl+C для выхода')

    i = 0
    try:
        while not limit or i < limit:
            i += 1
            if det.scan(i, 0, None) is None:
                print(f'[TEST] {i:>4}  ничего не найдено')
            cv2.imwrite(os.path.join(SAVE_DIR, 'last_frame.jpg'),
                        det.cam.capture_array())
            time.sleep(PERIOD)
    except KeyboardInterrupt:
        print('\n[TEST] Остановлено')
    finally:
        det.close()
        print(f'[TEST] Всего распознано: {len(det.log)}')


if __name__ == '__main__':
    main()
