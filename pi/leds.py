#!/usr/bin/env python3
"""Лента WS2812 на Raspberry Pi: драйвер поверх SPI + мигание розовым.

Железо: DATA_LED_IN сидит на физическом пине 19 (BCM GPIO10 = SPI0 MOSI).
Включается оверлеем `dtoverlay=spi0-0cs,no_miso` - он занимает только GPIO10
и GPIO11, оставляя GPIO8/9 лидарному UART3. Обычный `dtparam=spi=on` так не
годится: он забирает GPIO7-11 целиком и лидар замолкает.

Кодирование: SPI на 2.4 МГц, три тика на бит ленты ("0" = 100, "1" = 110).
Три тика на бит дают ровно 3 байта SPI на байт цвета, без выравнивания.

Правила RoboCupJunior Rescue Maze: чтобы жертву засчитали, робот должен стоять
в пределах 15 см от неё 5 секунд и всё это время мигать индикатором на видном
месте. Цвет и частоту правила не задают - поэтому розовый законен.

Запуск как проверки:  ~/maze/venv/bin/python ~/maze/leds.py
"""

from __future__ import annotations

import sys
import time

try:
    import spidev
except ImportError:
    print("[FATAL] нет модуля spidev:  ~/maze/venv/bin/pip install spidev")
    sys.exit(1)

LED_COUNT = 4
SPI_BUS, SPI_DEV = 0, 0        # SPI0, данные идут по MOSI = пин 19
SPI_HZ = 2_400_000             # 3 тика на бит ленты

# Пауза сброса: лента защёлкивает кадр после >50мкс низкого уровня. Один байт
# на этой частоте - 3.33мкс, поэтому 40 нулевых байт дают ~133мкс с запасом.
RESET_BYTES = 40

# Авторский цвет #FFB3C6. Напрямую в ленту его отдавать нельзя - выйдет почти
# белый, и вот почему: на экране цвет задан в sRGB, где число и яркость связаны
# гаммой ~2.2, а светодиод светит линейно. Зелёный 179 на экране - это 46%
# света, а у WS2812 те же 179 дают 70%. Зелёный с синим светят почти вдвое
# ярче задуманного и подтягивают цвет к белому.
PINK_HEX = (0xFF, 0xB3, 0xC6)
GAMMA = 2.2

# Насыщенность: множитель для всех каналов, кроме самого сильного. Меньше 1 -
# розовее (зелёный и синий уводятся вниз, красный остаётся), 1.0 - ровно
# исходный оттенок после гамма-коррекции.
SATURATION = 0.6

# Общая яркость, 0..255. Делит все три канала поровну, поэтому оттенок не
# уезжает - меняется только светлота. На полной это ~50мА на пиксель, 200мА на
# четыре: если просадит питание или будет слепить, снижать здесь.
BRIGHTNESS = 255


def _led_color(hex_rgb=PINK_HEX, gamma=GAMMA, sat=SATURATION, bright=BRIGHTNESS):
    """Экранный sRGB -> то, что надо послать в WS2812."""
    lin = [((v / 255.0) ** gamma) * 255.0 for v in hex_rgb]
    top = max(lin)
    lin = [v if v >= top else v * sat for v in lin]
    return tuple(int(v * bright / 255.0) for v in lin)


PINK = _led_color()

# Красный для сигнала "робота сняли с поля" - горит ровно, пока его держат.
# Насыщенность тут ни при чём: у чистого красного два других канала и так
# нули, поэтому sat=1.0, а гамма всё равно применяется и на полном канале
# ничего не меняет. Отдельный цвет, а не PINK, чтобы издали было видно, что
# робот не в штатном режиме, а ждёт, когда его поставят.
RED = _led_color((0xFF, 0x00, 0x00), sat=1.0)

# Синий - "сон": BTN_1 на ESP, робот ждёт, пока его перенесут на чекпоинт и
# нажмут кнопку на малине. Тоже ровно, тоже не PINK - чтобы не спутать ни с
# жертвой, ни с подъёмом.
SLEEP = _led_color((0x00, 0x40, 0xFF), sat=1.0)

BLINK_ON_S = 0.5               # частоту правила не задают, полсекунды хорошо видно
BLINK_OFF_S = 0.5
VICTIM_STOP_S = 5.0            # правила: 5 секунд неподвижно
MARGIN_S = 0.3                 # запас: судья считает секунды глазами

# Индикация возврата на стартовую клетку - ДРУГОЙ рисунок мигания, чтобы не
# путать с находкой жертвы: ВКЛ 1с/ВЫКЛ 1с, не менее 10с по правилам.
RETURN_BLINK_ON_S = 1.0
RETURN_BLINK_OFF_S = 1.0
RETURN_STOP_S = 10.0
RETURN_MARGIN_S = 0.5          # тот же смысл, что MARGIN_S у жертвы

# Таблица на байт: 8 бит ленты -> 24 тика SPI -> 3 байта.
_LUT = []
for _v in range(256):
    _bits = 0
    for _i in range(7, -1, -1):          # WS2812 ждёт старший бит первым
        _bits = (_bits << 3) | (0b110 if (_v >> _i) & 1 else 0b100)
    _LUT.append(_bits.to_bytes(3, "big"))


class NeoPixels:
    def __init__(self, count: int = LED_COUNT):
        self.count = count
        self.buf = [(0, 0, 0)] * count
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEV)
        self.spi.max_speed_hz = SPI_HZ
        self.spi.mode = 0

    def fill(self, r: int, g: int, b: int) -> None:
        self.buf = [(r & 0xFF, g & 0xFF, b & 0xFF)] * self.count

    def clear(self) -> None:
        self.fill(0, 0, 0)

    def show(self) -> None:
        out = bytearray()
        for r, g, b in self.buf:
            out += _LUT[g] + _LUT[r] + _LUT[b]     # лента ждёт GRB
        out += bytes(RESET_BYTES)
        self.spi.writebytes2(out)

    def close(self) -> None:
        self.clear()
        self.show()
        self.spi.close()


def blink_pink(px: NeoPixels, seconds: float = VICTIM_STOP_S + MARGIN_S,
               stop=None) -> None:
    """Мигает розовым заданное время и гасит ленту.

    Вызывать, когда робот УЖЕ остановился: отсчёт пяти секунд по правилам
    идёт от остановки, и если робот ещё докатывается, показ не засчитают.

    Длительность считается по часам, а не набором циклов: иначе задержки SPI
    и сна накапливаются и показ выходит короче заявленного - ровно то, за что
    могут не засчитать.

    stop - необязательная функция без аргументов: вернула True - показ
    обрывается (робота усыпили кнопкой посреди мигания).
    """
    r, g, b = PINK        # яркость и гамма уже учтены в _led_color()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if stop is not None and stop():
            break
        px.fill(r, g, b)
        px.show()
        time.sleep(min(BLINK_ON_S, max(0.0, end - time.monotonic())))
        px.clear()
        px.show()
        if time.monotonic() >= end:
            break
        time.sleep(min(BLINK_OFF_S, max(0.0, end - time.monotonic())))
    px.clear()
    px.show()


def blink_return(px: NeoPixels, seconds: float = RETURN_STOP_S + RETURN_MARGIN_S) -> None:
    """Мигает тем же (розовым) индикатором, что и находка жертвы, но своим
    рисунком - ВКЛ 1с/ВЫКЛ 1с, не менее 10с: сигнал распознанного возврата
    на стартовую клетку (правила RoboCup Rescue Maze).

    Вызывать, когда робот уже остановился в центре стартовой клетки - как и
    у blink_pink(), отсчёт идёт от остановки, а не от начала подъезда.
    """
    r, g, b = PINK
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        px.fill(r, g, b)
        px.show()
        time.sleep(min(RETURN_BLINK_ON_S, max(0.0, end - time.monotonic())))
        px.clear()
        px.show()
        if time.monotonic() >= end:
            break
        time.sleep(min(RETURN_BLINK_OFF_S, max(0.0, end - time.monotonic())))
    px.clear()
    px.show()


def main() -> None:
    try:
        px = NeoPixels()
    except FileNotFoundError:
        print("[FATAL] нет /dev/spidev0.0 - нужен оверлей в /boot/firmware/config.txt:")
        print("        dtoverlay=spi0-0cs,no_miso")
        sys.exit(1)

    try:
        while True:
            print(f"мигание розовым {BLINK_ON_S*1000:.0f}/{BLINK_OFF_S*1000:.0f}мс...")
            t0 = time.monotonic()
            blink_pink(px)
            print(f"  заняло {time.monotonic() - t0:.2f}с "
                  f"(правила требуют >= {VICTIM_STOP_S:.2f})")
            time.sleep(1.5)
    except KeyboardInterrupt:
        print("\nОстановлено.")
    finally:
        px.close()


if __name__ == "__main__":
    main()
