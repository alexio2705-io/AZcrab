#!/usr/bin/env python3
"""Отладочный OLED 128x64 (SSD1315, I2C) сзади робота.

Железо: J14 «display» на материнке -> GPIO2/GPIO3 (пины 3/5 гребёнки Pi) =
/dev/i2c-1, адрес 0x3C (у части модулей 0x3D - ищется сам). SSD1315 по
командам совместим с SSD1306. Кроме дисплея на этой шине никого нет.

Ориентация. Модуль стоит сзади, гребёнкой пинов к человеку за роботом.
Обычно у такого модуля пины сверху, а тут они ближе к человеку - то есть
для него «верх» модуля оказался внизу. Поэтому картинка повёрнута на 180° -
самим контроллером (ремап сегментов и обратный обход COM), без перерисовки.
Если читается вверх ногами - ROTATE_180 = False (проверка: --test).

Как устроено. Дисплей ведёт отдельный процесс-демон (этот файл без
аргументов), как map_web/dash_web: свой GIL, а I2C-запись (до ~90 мс на
весь кадр на 100 кГц) не задевает моторную петлю nav.py. Шлются только
изменившиеся страницы по 128 байт. Откуда данные:

  - заезд: тот же живой снимок nav.py, что получает пульт (live.py, TCP 8766);
  - калибровки и другие скрипты: say() - UDP-датаграмма на 127.0.0.1:8767,
    «отправил и забыл»; демона нет - сообщение просто никуда не уходит.
    Свежее сообщение (моложе своего ttl) важнее заезда: их шлют, пока
    человек что-то делает с роботом руками.

Демон один на машину (держит UDP-порт), переживает nav.py и калибровки -
после заезда на экране остаётся итог. При загрузке Pi его поднимает
systemd (maze-oled.service рядом, установка - в нём): на экране сразу IP
для SSH, Wi-Fi, температура и питание. Если сервис не поставлен - демон
поднимут сами nav.py и tile_calib.py (ensure_daemon()).

Только стандартная библиотека: I2C - ioctl на /dev/i2c-1, шрифты -
растровые misc-fixed 5x8 и 6x10 с кириллицей, вшиты в oled_font.py.

    ~/maze/venv/bin/python ~/maze/oled.py            # демон (обычно поднимается сам)
    ~/maze/venv/bin/python ~/maze/oled.py --test     # проверить ориентацию
    ~/maze/venv/bin/python ~/maze/oled.py --say "ШАПКА" "строка" ...   # из shell
    ~/maze/venv/bin/python ~/maze/oled.py --play ~/maze/cat.oledv  # видео по кругу (--stop)
    python3 oled.py --png /tmp/oled                  # образцы экранов в PNG, без железа
"""
from __future__ import annotations

try:
    import fcntl
except ImportError:  # web dashboard and screen renderer also run on Windows
    fcntl = None
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib
from collections import deque

from battery_probe import BatteryProbe
from oled_font import FONTS

W, H = 128, 64
I2C_DEV = "/dev/i2c-1"
ADDRS = (0x3C, 0x3D)
ROTATE_180 = True            # см. шапку: гребёнка к человеку за роботом
CONTRAST = 0xCF
DIM_CONTRAST = 0x08          # простой: экран тусклый, чтобы OLED не выгорал
IDLE_DIM_S = 600.0           # столько на экране простоя - и затемняем
REINIT_S = 30.0              # переинициализация: модуль мог моргнуть питанием на просадке
RETRY_S = 3.0                # дисплея нет на шине - пробовать снова через столько
FPS = 5.0

MSG_PORT = 8767
MSG_TTL_S = 3.0
LOG_PATH = os.path.expanduser("~/maze/oled.log")
SYSINFO_S = 10.0             # как часто обновлять IP/температуру/питание
ALERT_SHOW_S = 15.0          # сколько держать последнее [WARN] внизу экрана
STALE_S = 2.0                # снимков нет дольше - nav.py «молчит»
# Прочитанная метка на весь экран - пока nav.py ею занят (снимок act ==
# "жертва": мигание 5 с и выдача комплектов), но не меньше VICTIM_MIN_S и не
# больше VICTIM_SHOW_S. Раньше - ровно 8 с: ложная метка (робот у неё не
# стоит) и Ω без комплектов висели на экране, пока робот уже ехал дальше
# (домашний заезд 17.10).
VICTIM_MIN_S = 2.0
VICTIM_SHOW_S = 15.0
KITS_PER_COLUMN = 4          # как в nav.py, если снимок старый и kits_max не прислал
LEVEL_STRIDE = 1000          # как в nav.py: cy = y + этаж * LEVEL_STRIDE
RUN_LIMIT_S = 480.0


# ══════════════════════════════════════════════════════════════
#  Кадр 128x64 в формате контроллера: 8 страниц по 128 столбцов,
#  в байте столбец из 8 пикселей, младший бит - верхний.
# ══════════════════════════════════════════════════════════════
_glyphs = {}


def _font(name):
    f = _glyphs.get(name)
    if f is None:
        fw, fh, raw = FONTS[name]
        f = _glyphs[name] = (fw, fh, {c: bytes.fromhex(h) for c, h in raw.items()})
    return f


class Canvas:
    def __init__(self):
        self.buf = bytearray(W * H // 8)

    def clear(self):
        self.buf[:] = bytes(len(self.buf))

    def pixel(self, x, y, on=True):
        if 0 <= x < W and 0 <= y < H:
            i = (y >> 3) * W + x
            if on:
                self.buf[i] |= 1 << (y & 7)
            else:
                self.buf[i] &= ~(1 << (y & 7)) & 0xFF

    def get(self, x, y):
        return bool(self.buf[(y >> 3) * W + x] >> (y & 7) & 1)

    def fill(self, x, y, w, h, on=True):
        for yy in range(max(0, y), min(H, y + h)):
            for xx in range(max(0, x), min(W, x + w)):
                self.pixel(xx, yy, on)

    def text(self, x, y, s, font="6x10", on=True, scale=1):
        """Строка от (x, y) - левый верх ячейки. -> x после строки."""
        fw, _, glyphs = _font(font)
        for ch in s:
            if x >= W:
                break
            rows = glyphs.get(ch) or glyphs["?"]
            for ry, bits in enumerate(rows):
                if not bits:
                    continue
                for rx in range(fw):
                    if bits >> (fw - 1 - rx) & 1:
                        if scale == 1:
                            self.pixel(x + rx, y + ry, on)
                        else:
                            self.fill(x + rx * scale, y + ry * scale, scale, scale, on)
            x += fw * scale
        return x

    def rect(self, x, y, w, h, on=True):
        """Контур прямоугольника в 1 пиксель."""
        self.fill(x, y, w, 1, on)
        self.fill(x, y + h - 1, w, 1, on)
        self.fill(x, y, 1, h, on)
        self.fill(x + w - 1, y, 1, h, on)

    def disc(self, cx, cy, r, on=True):
        ri = int(r) + 1
        for y in range(int(cy) - ri, int(cy) + ri + 2):
            for x in range(int(cx) - ri, int(cx) + ri + 2):
                if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                    self.pixel(x, y, on)

    def stroke(self, x0, y0, x1, y1, w=5.0):
        """Отрезок толщиной w (круглая кисть) - для крупных значков."""
        n = max(1, int(max(abs(x1 - x0), abs(y1 - y0))))
        for k in range(n + 1):
            self.disc(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, w / 2)

    def arc(self, cx, cy, rx, ry, a0, a1, w=5.0):
        """Дуга эллипса толщиной w; углы в градусах, 0 - вправо, 90 - вниз."""
        import math
        n = max(8, int(abs(a1 - a0) / 360.0 * 2 * math.pi * max(rx, ry)))
        for k in range(n + 1):
            a = math.radians(a0 + (a1 - a0) * k / n)
            self.disc(cx + rx * math.cos(a), cy + ry * math.sin(a), w / 2)

    def pages(self):
        return [bytes(self.buf[p * W:(p + 1) * W]) for p in range(H // 8)]

    def ascii(self):
        return "\n".join("".join("#" if self.get(x, y) else "." for x in range(W))
                         for y in range(H))

    def png(self, path, zoom=4):
        """Как видно на экране (белое на чёрном), увеличено в zoom раз."""
        rows = []
        for y in range(H):
            row = bytes(230 if self.get(x // zoom, y) else 12 for x in range(W * zoom))
            rows += [b"\x00" + row] * zoom

        def chunk(tag, data):
            return (struct.pack(">I", len(data)) + tag + data
                    + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
        hdr = struct.pack(">IIBBBBB", W * zoom, H * zoom, 8, 0, 0, 0, 0)
        with open(path, "wb") as f:
            f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", hdr)
                    + chunk(b"IDAT", zlib.compress(b"".join(rows), 9)) + chunk(b"IEND", b""))


# ══════════════════════════════════════════════════════════════
#  SSD1306/SSD1315 по I2C
# ══════════════════════════════════════════════════════════════
I2C_SLAVE = 0x0703


class Ssd1306:
    """Пишет Canvas на дисплей. Ни одна ошибка шины наружу не выходит:
    модуль отвалился - show() возвращает False и раз в RETRY_S пробует снова."""

    def __init__(self, dev=I2C_DEV, rot180=ROTATE_180, log=print):
        self.dev = dev
        self.rot180 = rot180
        self.log = log
        self.fd = -1                 # -1 - не открыт
        self.contrast = CONTRAST     # меняется на ходу (show шлёт 0x81)
        self._sent_contrast = None
        self.addr = None
        self.shown = [None] * (H // 8)
        self._t_init = 0.0
        self._retry_t = 0.0
        self._err = None

    def _cmd(self, *c):
        os.write(self.fd, bytes((0x00,) + c))

    def _open(self):
        if fcntl is None:
            raise OSError("I2C display requires Linux")
        fd = os.open(self.dev, os.O_RDWR)
        try:
            for a in ADDRS:
                fcntl.ioctl(fd, I2C_SLAVE, a)
                try:
                    os.write(fd, b"\x00\xe3")          # NOP: кто ответит ACK
                except OSError:
                    continue
                self.fd, self.addr = fd, a
                self._init()
                return
        except OSError:
            self.fd = -1              # чтобы _fail не закрыл его второй раз
            os.close(fd)
            raise
        os.close(fd)
        raise OSError(f"на {self.dev} никто не отвечает по 0x3C/0x3D")

    def _init(self):
        # Без 0xAE (выкл): та же последовательность служит и периодической
        # переинициализацией, и экран не должен моргать.
        seg, com = (0xA0, 0xC0) if self.rot180 else (0xA1, 0xC8)
        self._cmd(0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
                  seg, com, 0xDA, 0x12, 0x81, self.contrast, 0xD9, 0xF1, 0xDB, 0x40,
                  0x2E, 0xA4, 0xA6, 0xAF)
        self.shown = [None] * (H // 8)
        self._sent_contrast = self.contrast
        self._t_init = time.monotonic()

    def _fail(self, e):
        if self.fd >= 0:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = -1
        self._retry_t = time.monotonic() + RETRY_S
        msg = f"{type(e).__name__}: {e}"
        if msg != self._err:
            self._err = msg
            self.log(f"[WARN] дисплей: {msg} - пробую раз в {RETRY_S:.0f}с")

    def show(self, cv):
        now = time.monotonic()
        try:
            if self.fd < 0:
                if now < self._retry_t:
                    return False
                self._open()
                if self._err is not None:
                    self.log(f"дисплей снова на связи (0x{self.addr:02X})")
                    self._err = None
            elif now - self._t_init > REINIT_S:
                self._init()
            if self.contrast != self._sent_contrast:
                self._cmd(0x81, self.contrast)
                self._sent_contrast = self.contrast
            for p, data in enumerate(cv.pages()):
                old = self.shown[p]
                if data == old:
                    continue
                # Только поменявшийся кусок страницы: видео (oled_video) на
                # 100 кГц иначе упиралось бы в ~10 полных кадров в секунду.
                a, b = 0, W - 1
                if old is not None:
                    while data[a] == old[a]:
                        a += 1
                    while data[b] == old[b]:
                        b -= 1
                self._cmd(0x21, a, b, 0x22, p, p)
                os.write(self.fd, b"\x40" + data[a:b + 1])
                self.shown[p] = data
            return True
        except OSError as e:
            self._fail(e)
            return False


# ══════════════════════════════════════════════════════════════
#  Сообщения от скриптов (калибровки и т.п.)
# ══════════════════════════════════════════════════════════════
_tx = None


def say(title, lines=(), big=None, alert=False, small=False, ttl=MSG_TTL_S, port=MSG_PORT,
        video=None, right=None, items=None, sel=0, bar=None, progress=None):
    """Показать экран на дисплее, пока его шлют (и ещё ttl секунд).

    title - шапка, lines - строки под ней (6x10: 5 строк по 21 символу,
    small=True - 5x8: 6 строк по 25), длинные прокручиваются сами. big -
    крупная строка (до 10 символов) вместо первых двух строк. alert -
    шапка мигает. video - путь к файлу oled_video.py на Pi: вместо текста
    этот ролик по кругу. right - короткое справа в шапке. items + sel -
    список с выбранным пунктом вместо строк. progress (0..1) - полоска
    замера под первой строкой. bar - кнопки внизу экрана {пин: подпись}
    (см. button_bar), тогда под текст остаётся 4 строки. Никогда не бросает
    и не ждёт."""
    global _tx
    msg = {"title": str(title), "lines": [str(s) for s in lines][:6], "big": big,
           "alert": bool(alert), "small": bool(small), "ttl": float(ttl)}
    if video:
        msg["video"] = os.path.abspath(os.path.expanduser(video))
    if right:
        msg["right"] = str(right)
    if items:
        msg["items"], msg["sel"] = [str(s) for s in items], int(sel)
    if bar:
        msg["bar"] = {str(k): str(v) for k, v in bar.items()}
    if progress is not None:
        msg["progress"] = float(progress)
    try:
        if _tx is None:
            _tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _tx.setblocking(False)
        _tx.sendto(json.dumps(msg, ensure_ascii=False).encode(), ("127.0.0.1", port))
    except OSError:
        pass


def daemon_running():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", MSG_PORT))
        return False
    except OSError:
        return True
    finally:
        s.close()


def ensure_daemon():
    """Поднять демон, если его ещё нет. Отдельный сеанс (start_new_session):
    nav.py кончится или оборвётся SSH - экран живёт дальше. Не на Pi (нет
    /dev/i2c-1: симулятор, мак) - ничего не делает."""
    if not os.path.exists(I2C_DEV):
        if sys.platform.startswith("linux") and os.path.exists("/proc/device-tree/model"):
            print(f"  [WARN] дисплей: нет {I2C_DEV} - I2C выключен "
                  f"(sudo raspi-config nonint do_i2c 0 и перезагрузка)")
        return None
    if daemon_running():
        return None
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        log_f = open(LOG_PATH, "a")
        proc = subprocess.Popen([sys.executable, os.path.abspath(__file__)],
                                stdin=subprocess.DEVNULL, stdout=log_f,
                                stderr=subprocess.STDOUT, start_new_session=True)
        log_f.close()
        return proc
    except OSError as e:
        print(f"  [WARN] дисплей не запущен: {e}")
        return None


# ══════════════════════════════════════════════════════════════
#  Экраны. Всё ниже - чистые функции от состояния st (см. Daemon.state()),
#  поэтому рисуются и в тестах, и в --png без железа и без nav.py.
# ══════════════════════════════════════════════════════════════
COLS = 21                    # 6x10 по ширине
COLS_SMALL = 25              # 5x8
ARROW = {0: "↑", 1: "→", 2: "↓", 3: "←"}
DIRS = "NESW"
TILE = {"white": "бел", "black": "ЧЁРН", "blue": "син", "red": "крас",
        "silver": "ЗЕРК", "air": "возд", "?": "?"}
PHASE = {
    "boot": "ЗАПУСК", "wait_button": "ЖМИ СТАРТ", "calib": "КАЛИБРОВКА",
    "exploring": "РАЗВЕДКА", "explored": "РАЗВЕДАНО", "cap": "ЛИМИТ КЛЕТОК",
    "time": "ВРЕМЯ: ДОМОЙ", "returning": "ДОМОЙ", "at_start": "НА СТАРТЕ",
    "sleep": "ПАУЗА ESP", "blinking": "ФИНИШ", "done": "ГОТОВО", "fatal": "АВАРИЯ",
    "wait_link": "НЕТ СВЯЗИ С ESP", "wait_motors": "МОТОРЫ ВЫКЛ",
    "wait_lidar": "ЛИДАР МОЛЧИТ", "wait_gyro": "НЕТ КУРСА BNO",
}
# Ждём человека: шапка мигает, ниже - что сделать (переносится по словам).
WAIT_HINT = {
    "wait_link": "проверь ESP и провод к Pi - заезд продолжится сам",
    "wait_motors": "включи питание моторов - заезд продолжится сам",
    "wait_lidar": "проверь кабель и питание лидара",
    "wait_gyro": "проверь BNO085 и его провод к ESP",
    "fatal": "смотри лог nav.py",
}
ALERT_TAGS = ("[WARN]", "[FATAL]", "[WAIT]", "[!]")
PAGE_S = 2.5                 # длинный текст листается страницами раз в столько


# Бегущей строки нет: в стоп-кадре конец текста оказывался перед началом,
# а на ходу её не прочитать. Вместо неё - перенос по словам; не влезло в
# отведённые строки - текст листается целыми страницами (block), одиночная
# строка обрезается с «…» (clip).
def clip(s, n=COLS):
    s = str(s)
    return s if len(s) <= n else s[:n - 1] + "…"


def wrap(s, n=COLS):
    """Перенос по словам в строки не длиннее n. Слово длиннее строки (путь к
    файлу) режется после последнего «/», нет его - после «_», «.» и т.д.,
    а если ничего такого нет - просто по n."""
    rows, cur = [], ""
    for w in str(s).split():
        while len(w) > n:
            if cur:
                rows.append(cur)
                cur = ""
            cut = n
            for c in "/_.,-":
                k = w.rfind(c, 0, n)
                if k > 0:
                    cut = k + 1
                    break
            rows.append(w[:cut])
            w = w[cut:]
        if not w:
            continue
        if not cur:
            cur = w
        elif len(cur) + 1 + len(w) <= n:
            cur += " " + w
        else:
            rows.append(cur)
            cur = w
    if cur:
        rows.append(cur)
    return rows or [""]


def paged(rows, n, t):
    """Строки -> страница по n строк; страниц несколько - листаются."""
    pages = [rows[i:i + n] for i in range(0, len(rows), n)] or [[]]
    return pages[int(t / PAGE_S) % len(pages)]


def mmss(s):
    if s is None:
        return "--:--"
    s = max(0, int(s))
    return f"{s // 60}:{s % 60:02d}"


def _cm(mm):
    return "--" if mm is None else str(int(round(mm / 10)))


def header(cv, left, right="", t=0.0, blink=False):
    inv = not blink or int(t * 2) % 2 == 0
    if inv:
        cv.fill(0, 0, W, 11)
    room = COLS - (len(right) + 1 if right else 0)
    cv.text(1, 1, clip(left, room), on=not inv)
    if right:
        cv.text(W - 1 - 6 * len(right), 1, right, on=not inv)


def line(cv, i, s, inv=False):
    """i-я строка 6x10 под шапкой, i = 0..4."""
    y = 12 + 10 * i
    if inv:
        cv.fill(0, y, W, 10)
    cv.text(1, y, clip(s), on=not inv)


def line2(cv, i, left, right):
    """Строка в две колонки: right прижат к правому краю."""
    right = clip(right, COLS - 1)
    room = COLS - len(right) - 1
    y = 12 + 10 * i
    cv.text(1, y, clip(left, room))
    cv.text(W - 1 - 6 * len(right), y, right)


def block(cv, row, nrows, s, t, inv=False):
    """Текст с переносом в строки row..row+nrows-1 (листается, если длиннее)."""
    page = paged(wrap(s), nrows, t)
    for i in range(nrows):
        line(cv, row + i, page[i] if i < len(page) else "", inv=inv)


def small_line(cv, i, s):
    """i-я строка 5x8 под шапкой, i = 0..5."""
    cv.text(1, 12 + 8 * i, clip(s, COLS_SMALL), font="5x8")


def big(cv, y, s):
    """Крупно (6x10 x2), по центру."""
    s = clip(s, 10)
    cv.text((W - 12 * len(s)) // 2, y, s, scale=2)


# ── кнопки у малины ───────────────────────────────────────────────
# Материнка: SW2 -> GPIO22, SW3 -> GPIO24, SW4 -> GPIO27, SW1 -> GPIO25; у
# каждой подтяжка 10к к 3.3 В и RC 750 Ом + 0.1 мкФ. Если стоять за роботом,
# слева направо: 22, 24, (промежуток), 27, 25 - старт левой рукой (27) левее
# правой (25). Номера пинов человеку ничего не скажут, поэтому внизу экрана
# рисуются сами кнопки, как стоят, и на каждой - что она сейчас делает.
BTN_PREV, BTN_NEXT, BTN_MODE, BTN_OK = 22, 24, 27, 25
BAR_SLOTS = ((BTN_PREV, 0), (BTN_NEXT, 29), (BTN_MODE, 72), (BTN_OK, 101))
BAR_W, BAR_Y = 27, 54
BAR_CHARS = BAR_W // 5          # подпись на кнопке, 5x8


def button_bar(cv, labels):
    """Полоска кнопок по низу экрана (y 54..63). labels: {пин: подпись};
    у кнопки есть подпись - она залита, нет - пустая рамка (не делает ничего)."""
    labels = {int(k): v for k, v in (labels or {}).items()}
    cv.fill(0, BAR_Y - 1, W, H - BAR_Y + 1, False)
    for pin, x in BAR_SLOTS:
        s = labels.get(pin)
        if s:
            s = str(s)[:BAR_CHARS]
            cv.fill(x, BAR_Y, BAR_W, H - BAR_Y)
            cv.text(x + (BAR_W - 5 * len(s)) // 2 + 1, BAR_Y + 1, s, font="5x8", on=False)
        else:
            cv.rect(x, BAR_Y, BAR_W, H - BAR_Y)


def item_list(cv, items, sel, rows):
    """Список под шапкой, выбранный пункт - инверсией; длинный список едет
    так, чтобы выбранный был виден."""
    top = min(max(0, sel - rows + 1), max(0, len(items) - rows))
    if sel < top:
        top = sel
    for i in range(rows):
        k = top + i
        if k < len(items):
            line(cv, i, items[k], inv=k == sel)


def progress_box(cv, y, frac):
    cv.rect(4, y, W - 8, 10)
    cv.fill(6, y + 2, int((W - 12) * max(0.0, min(1.0, frac))), 6)


def cams_str(cams):
    """Камеры как «Л● П+»: ● смотрит, + готова, … грузится, × нет.
    Не «○»: в этом шрифте это буква «о», и «П○» читается как «По»."""
    if not cams:
        return "нет"
    out = []
    for side, name in (("left", "Л"), ("right", "П")):
        c = cams.get(side)
        if c is None:
            mark = "×"
        elif c.get("dead") or c.get("failed"):
            mark = "×"
        elif not c.get("ready"):
            mark = "…"
        else:
            mark = "●" if c.get("armed") else "+"
        out.append(name + mark)
    return " ".join(out)


def _pos(snap):
    p = snap.get("pose") or {}
    lvl = p.get("level") or 0
    hdg = (p.get("hdg") or 0) % 4
    y = p.get("cy", 0) - lvl * LEVEL_STRIDE
    s = f"{p.get('cx', 0)},{y} {ARROW[hdg]}{DIRS[hdg]}"
    if lvl:
        s += f" эт{lvl}"
    cp = p.get("cp")
    if cp and (cp[0], cp[1]) != (0, 0):
        cpl = (cp[1] // LEVEL_STRIDE) if abs(cp[1]) >= LEVEL_STRIDE // 2 else 0
        s += f" чп{cp[0]},{cp[1] - cpl * LEVEL_STRIDE}"
    return s


def _victims(snap):
    """Найдено жертв - без фейковых мишеней (старый nav.py: все метки)."""
    v = snap.get("victims_real")
    return snap.get("victims_n", 0) if v is None else v


def _kits(snap):
    k = snap.get("kits") or {}
    return f"кит{k.get('L', '-')}/{k.get('R', '-')}"


def _alert(st):
    """Свежее предупреждение из лога nav.py - без тега, с «!» впереди."""
    a = st.get("alert")
    if not a or st["wall"] - a[1] >= ALERT_SHOW_S:
        return None
    text = a[0]
    for tag in ALERT_TAGS:
        if text.startswith(tag):
            text = ("" if tag == "[WAIT]" else "! ") + text[len(tag):].strip()
            break
    return text


def scr_msg(cv, st):
    """Экран из say(): шапка и строки; ещё бывают список с выбором (items,
    sel), полоска замера (progress) и кнопки внизу (bar) - так рисуют меню,
    калибровку плиток и тест ленты."""
    m, t = st["msg"], st["now"]
    header(cv, m.get("title", ""), str(m.get("right") or ""), t=t, blink=m.get("alert"))
    bar = m.get("bar")
    nrows = 4 if bar else 5
    small = bool(m.get("small")) and not m.get("big")
    n = COLS_SMALL if small else COLS
    rows = [r for s in (m.get("lines") or []) for r in wrap(s, n)]
    if m.get("items"):
        item_list(cv, [str(s) for s in m["items"]], int(m.get("sel") or 0), nrows)
    elif m.get("progress") is not None:
        if rows:
            line(cv, 0, rows[0])
        progress_box(cv, 30, float(m["progress"]))
        if len(rows) > 1:
            small_line(cv, 4, rows[1])
    elif m.get("big"):
        big(cv, 13, str(m["big"]))
        for i, s in enumerate(paged(rows, nrows - 2, t)):
            line(cv, i + 2, s)
    elif small:
        for i, s in enumerate(paged(rows, 5 if bar else 6, t)):
            small_line(cv, i, s)
    else:
        for i, s in enumerate(paged(rows, nrows, t)):
            line(cv, i, s)
    if bar:
        button_bar(cv, bar)


def _uptime(sec):
    if sec is None:
        return "--"
    m = int(sec) // 60
    return f"{m // 60}ч{m % 60:02d}" if m >= 60 else f"{m}мин"


def _power(thr):
    """vcgencmd get_throttled: бит 0 - напряжение низкое сейчас, бит 16 -
    было с момента загрузки. Для робота на аккумуляторе это важнее всего."""
    if thr is None:
        return "питание ?"
    if thr & 0x1:
        return "! НИЗКОЕ ПИТАНИЕ"
    if thr & 0x10000:
        return "! были просадки"
    return "питание ок"


def scr_idle(cv, st):
    """Pi загрузилась, nav.py ещё не запускали: как подключиться и как
    чувствует себя малина. Сюда же - если nav.py идёт, но снимков не шлёт."""
    t = st["now"]
    header(cv, "MAZE2027", time.strftime("%H:%M", time.localtime(st["wall"])), t)
    ip = st.get("ip")
    line(cv, 0, f"IP {ip}" if ip else "IP: ждём сеть…")
    ssid = st.get("ssid")
    line(cv, 1, f"Wi-Fi {ssid}" if ssid else ("Wi-Fi: нет" if not ip else "сеть: провод"))
    temp, mem = st.get("temp"), st.get("mem")
    line2(cv, 2, f"CPU {'--' if temp is None else f'{temp:.0f}'}°C",
          f"ОЗУ {'--' if mem is None else f'{mem:.0f}'}%")
    pw = _power(st.get("throttled"))
    ui = st.get("ui") or {}
    if not pw.startswith("!") and "volts" in ui:
        # Вместо "питание ок" - батарея моторов (battery_probe); пульт без
        # демона замера не видит и показывает по-старому.
        pw = f"{ui['volts']:.1f}V" if ui["volts"] is not None else "--.-V"
    if pw.startswith("!"):
        line(cv, 3, pw, inv=True)
    else:
        line2(cv, 3, pw, f"вкл {_uptime(st.get('uptime'))}")
    if (st.get("ui") or {}).get("buttons"):
        button_bar(cv, {BTN_MODE: "МЕНЮ"})
        return
    # Нижняя строка отделена чертой: это уже не о малине, а что дальше.
    cv.fill(0, 52, W, 1)
    cv.text(1, 54, clip("ждёт запуска nav.py"))


def scr_gone(cv, st):
    """nav.py был, но больше не шлёт (кончился или упал) - итог остаётся."""
    t, snap = st["now"], st["snap"]
    header(cv, "nav.py ВЫКЛ" if not st["connected"] else "nav.py МОЛЧИТ",
           time.strftime("%H:%M", time.localtime(st["wall"])), t,
           blink=st["connected"])
    ph = snap.get("phase")
    line(cv, 0, f"было: {PHASE.get(ph, ph or '—')}")
    line(cv, 1, f"время {mmss(snap.get('run_s'))} жертв{_victims(snap)}")
    line(cv, 2, f"клеток {snap.get('visited_n', '—')} {_kits(snap)}")
    a = _alert(st)
    if a:
        block(cv, 3, 2, a, t, inv=True)
    else:
        line(cv, 3, st.get("lastline") or "")
        line(cv, 4, f"IP {st.get('ip') or '—'}")


def scr_preflight(cv, st):
    """Ждём кнопку старта: всё, что стоит проверить, пока робот на старте."""
    t, snap = st["now"], st["snap"]
    header(cv, PHASE["wait_button"], "", t, blink=True)
    r = st.get("rates") or {}
    esp = r.get("esp")
    lk = snap.get("link") or {}
    errs = (lk.get("crc") or 0) + (lk.get("lost") or 0) + (lk.get("cmd_err") or 0)
    s = f"ESP {'--' if esp is None else f'{esp:.0f}'}Гц"
    s += f" ошибок {errs}" if errs else " ок"
    line(cv, 0, s)
    w = snap.get("walls") or {}
    line(cv, 1, f"лидар ↑{_cm(w.get('front'))} →{_cm(w.get('right'))} "
                f"←{_cm(w.get('left'))}")
    line(cv, 2, f"кам {cams_str(snap.get('cams'))} {_kits(snap)}")
    tile = st.get("tile_guess")
    if snap.get("color_on") is False:
        tl = "датчик цвета выкл"
    elif tile is None:
        tl = "плитка: ? (калибр.?)"
    else:
        tl = f"плитка: {TILE.get(tile, tile)}"
    line(cv, 3, tl)
    # Внизу - кнопки старта, как стоят на плате: какой рукой ехать. Заезд
    # запущен из меню - ещё и выход назад в меню, пока не поехали.
    ui = st.get("ui") or {}
    button_bar(cv, {BTN_PREV: "ОТМЕН" if ui.get("child") == "nav" else None,
                    BTN_MODE: "ЛЕВ", BTN_OK: "ПРАВ"})


def scr_boot(cv, st):
    """Запуск nav.py до кнопки и калибровка после неё: что он сейчас делает."""
    t, snap = st["now"], st["snap"]
    ph = snap.get("phase")
    header(cv, "КАЛИБР. НЕ ТРОГАЙ" if ph == "calib" else PHASE.get(ph, ph or "?"),
           "", t, blink=ph == "calib")
    block(cv, 0, 2, st.get("lastline") or "nav.py стартует", t)
    a = _alert(st)
    if a:
        block(cv, 2, 2, a, t, inv=True)
    line(cv, 4, f"IP {st.get('ip') or '—'}")


def scr_wait(cv, st):
    t, snap = st["now"], st["snap"]
    ph = snap.get("phase")
    header(cv, PHASE.get(ph, ph), mmss(snap.get("run_s")) if snap.get("run_s") else "",
           t, blink=True)
    block(cv, 0, 3, WAIT_HINT.get(ph, ""), t)
    if ph == "wait_link":
        lk = snap.get("link") or {}
        line(cv, 4, f"crc {lk.get('crc', 0)}  потеряно {lk.get('lost', 0)}")
    else:
        line(cv, 4, _pos(snap))


# ── заезд: только то, что успеваешь прочесть на ходу ─────────────
# Слева - комплекты в столбиках (как стоят в роботе: Л слева, если смотреть
# из-за робота), справа крупно - плитка под роботом, ниже - время и что
# необычного (едет домой, пауза), внизу на всю ширину - что сломалось.
# Фазы, расстояния до стен и частоты на ходу не прочесть - их здесь нет.
TILE_BIG = {"white": "БЕЛАЯ", "black": "ЧЁРНАЯ", "blue": "СИНЯЯ", "red": "КРАСНАЯ",
            "silver": "ЗЕРКАЛО", "air": "ВОЗДУХ", "?": "?"}
RUN_TAG = {"returning": "ДОМОЙ", "time": "ДОМОЙ", "explored": "ДОМОЙ", "cap": "ДОМОЙ",
           "at_start": "СТАРТ", "sleep": "ПАУЗА"}


def kits_columns(cv, snap, x0=0):
    """Два столбика по KITS_PER_COLUMN ячеек: залитая - комплект на месте.
    Выдаются снизу, поэтому остаток - нижние ячейки."""
    k = snap.get("kits") or {}
    kmax = snap.get("kits_max") or KITS_PER_COLUMN
    top, bottom, gap = 10, 51, 2
    h = max(3, (bottom - top + 1 - (kmax - 1) * gap) // kmax)
    for i, (side, name) in enumerate((("L", "Л"), ("R", "П"))):
        x = x0 + 1 + i * 15
        cv.text(x + 4, 0, name, font="5x8")
        left = k.get(side)
        for j in range(kmax):
            y = top + j * (h + gap)
            if left is not None and j >= kmax - left:
                cv.fill(x, y, 12, h)
            else:
                cv.rect(x, y, 12, h)


def tile_box(cv, tile, x, y, w, h):
    """Плитка под роботом крупно. Чёрная - залитая рамка (как и выглядит)."""
    word = TILE_BIG.get(tile, "—" if tile is None else str(tile).upper())
    word = clip(word, (w - 4) // 12)
    black = tile == "black"
    if black:
        cv.fill(x, y, w, h)
    else:
        cv.rect(x, y, w, h)
    cv.text(x + (w - 12 * len(word)) // 2, y + (h - 20) // 2, word, scale=2, on=not black)


def _problems(st):
    """Что не так с железом прямо сейчас - коротко, для нижней строки."""
    snap = st["snap"]
    r = st.get("rates") or {}
    out = []
    esp, lid = r.get("esp"), r.get("lidar")
    if esp is not None and esp < 100:
        out.append(f"ESP {esp:.0f}Гц")
    if lid is not None and lid < 5:
        out.append(f"лидар {lid:.0f}Гц")
    cams = snap.get("cams") or {}
    for side, name in (("left", "Л"), ("right", "П")):
        c = cams.get(side)
        if c and (c.get("dead") or c.get("failed")):
            out.append(f"камера {name}")
    return ("! " + ", ".join(out)) if out else None


def bottom_bar(cv, text, t):
    """Нижняя строка на всю ширину, инверсией; длинное листается по строке."""
    page = paged(wrap(text), 1, t)
    line(cv, 4, page[0] if page else "", inv=True)


def scr_run(cv, st):
    t, snap = st["now"], st["snap"]
    ph = snap.get("phase")
    kits_columns(cv, snap)
    cv.fill(31, 0, 1, 52)
    x0, w = 34, W - 34
    tile_box(cv, snap.get("tile"), x0, 0, w, 24)
    # время заезда; меньше минуты до конца - мигает
    tl = snap.get("t_left")
    late = tl is not None and tl < 60 and ph in ("exploring", "time", "returning")
    tm = mmss(snap.get("run_s"))
    if late and int(t * 2) % 2 == 0:
        cv.fill(x0, 27, 6 * len(tm) + 2, 11)
        cv.text(x0 + 1, 28, tm, on=False)
    else:
        cv.text(x0 + 1, 28, tm)
    tag = RUN_TAG.get(ph)
    if tag:
        tw = 6 * len(tag) + 2
        cv.fill(W - tw, 27, tw, 11)
        cv.text(W - tw + 1, 28, tag, on=False)
    cv.text(x0 + 1, 40, f"жертв {_victims(snap)}")
    a = _alert(st) or _problems(st)
    if ph == "sleep":
        bottom_bar(cv, "пауза ESP: рестарт с чекпоинта", t)
    elif a:
        bottom_bar(cv, a, t)
    else:
        volts = [v for v in (snap.get("motors") or {}).get("voltage", []) if v is not None]
        cv.text(35, 53, f"4S {min(volts):.1f}V" if volts else "4S --.-V")


# ── прочитанная метка - на весь экран ────────────────────────────
GLYPH = {"damaged": "Φ", "stable": "Ψ", "unharmed": "Ω"}
RING_NAME = {"black": "чёрный", "red": "красный", "yellow": "жёлтый",
             "green": "зелёный", "blue": "синий"}
RING_SCORE = {"black": -2, "red": -1, "yellow": 0, "green": 1, "blue": 2}


def draw_glyph(cv, ch, cx=63.5, w=5.0):
    """Φ/Ψ/Ω во весь экран - нарисованы линиями (растровый шрифт в 6 раз
    крупнее был бы лесенкой)."""
    if ch == "Φ":
        cv.stroke(cx, 2, cx, 61, w)
        cv.arc(cx, 31.5, 24, 17, 0, 360, w)
        cv.stroke(cx - 8, 2, cx + 8, 2, w)
        cv.stroke(cx - 8, 61, cx + 8, 61, w)
    elif ch == "Ψ":
        cv.stroke(cx, 4, cx, 60, w)
        cv.arc(cx, 4, 22, 34, 0, 180, w)
        cv.stroke(cx - 9, 60, cx + 9, 60, w)
    elif ch == "Ω":
        cv.arc(cx, 27, 22, 22, 125, 415, w)
        cv.stroke(cx - 12.6, 45, cx - 16.5, 58, w)
        cv.stroke(cx + 12.6, 45, cx + 16.5, 58, w)
        cv.stroke(cx - 27.5, 58, cx - 15, 58, w)
        cv.stroke(cx + 15, 58, cx + 27.5, 58, w)
    else:
        big(cv, 22, ch)


def _ring_on(color, x, y):
    """Цвет кольца - плотностью заливки: чёрное сплошное, жёлтое пустое."""
    if color == "black":
        return True
    if color == "blue":
        return not (x % 2 == 0 and y % 2 == 0)
    if color == "green":
        return (x + y) % 2 == 0
    if color == "red":
        return x % 2 == 0 and y % 2 == 0
    if color == "yellow":
        return False
    return x % 4 == 0 and y % 4 == 0        # не прочиталось


def draw_target(cv, rings, cx=31.5, cy=31.5, r_out=31.0):
    """Мишень: 5 колец от центра наружу, каждое залито по своему цвету,
    между кольцами - тонкая граница."""
    import math
    step = r_out / 5
    for y in range(int(cy - r_out) - 1, int(cy + r_out) + 2):
        for x in range(int(cx - r_out) - 1, int(cx + r_out) + 2):
            r = math.hypot(x - cx, y - cy)
            if r > r_out + 0.5:
                continue
            k = min(4, int(r / step))
            edge = abs(r - step * round(r / step)) < 0.5 and r > 0.5
            if edge or _ring_on(rings[k] if k < len(rings) else None, x, y):
                cv.pixel(x, y)


def scr_cog(cv, v):
    rings = list(v.get("rings") or [])
    draw_target(cv, rings)
    x = 66
    for i, c in enumerate(rings[:5]):
        y = 1 + 9 * i
        for yy in range(y + 1, y + 7):            # образец заливки
            for xx in range(x, x + 6):
                if _ring_on(c, xx, yy):
                    cv.pixel(xx, yy)
        cv.rect(x - 1, y, 8, 8)
        sc = RING_SCORE.get(c)
        sc = "" if sc is None else f"{sc:+d}" if sc else "0"
        cv.text(x + 9, y, clip(f"{RING_NAME.get(c, c)} {sc}", 10), font="5x8")
    cv.fill(x - 1, 48, W - x + 1, 1)
    cs, typ = v.get("checksum"), v.get("type")
    if cs is None:
        res = "кольца ?"
    elif typ in GLYPH:
        res = f"{cs:+d} → {GLYPH[typ]}"
    else:
        res = f"ФЕЙК {cs:+d}"
    cv.text(x, 52, clip(res, (W - x) // 6))


_cat_cache = {}


def cat_frames(path=None):
    """Кадры кота (cat.oledv) для экрана метки; нет файла - None. Читается
    один раз, файл пересобрали - перечитывается."""
    path = path or CAT_PATH
    try:
        key = (path, os.path.getmtime(path))
    except OSError:
        return None
    if key not in _cat_cache:
        _cat_cache.clear()
        try:
            import oled_video
            _cat_cache[key] = oled_video.load(path)[1] or None
        except (OSError, ValueError):
            _cat_cache[key] = None
    return _cat_cache[key]


def scr_victim(cv, st):
    """Метка-мишень - мишень с кольцами. Буква - кот слева со своим
    настроением (oled_moods.py), буква справа во всю высоту; кота нет (нет
    cat.oledv) - одна буква во весь экран."""
    v = st["victim"]
    if v.get("class") == "cognetive" or v.get("rings"):
        return scr_cog(cv, v)
    ch = GLYPH.get(v.get("type")) or str(v.get("label") or "?")
    import oled_moods
    if oled_moods.draw(cv, ch, st.get("cat") or cat_frames(), st.get("victim_age") or 0.0):
        draw_glyph(cv, ch, cx=oled_moods.LETTER_CX, w=oled_moods.LETTER_W)
    else:
        draw_glyph(cv, ch)


def scr_finish(cv, st):
    t, snap = st["now"], st["snap"]
    ph = snap.get("phase")
    header(cv, PHASE.get(ph, ph), "", t, blink=ph == "blinking")
    big(cv, 13, mmss(snap.get("run_s")))
    line(cv, 2, f"жертв {_victims(snap)}  {_kits(snap)}")
    line(cv, 3, f"клеток {snap.get('visited_n', '—')}")
    line(cv, 4, f"IP {st.get('ip') or '—'}")


def scr_menu(cv, st):
    """Меню на экране: что запустить с самого робота, без ноута."""
    ui = st["ui"]
    sel = ui.get("sel", 0)
    names = [p[0] for p in ui.get("programs") or PROGRAMS]
    scr_msg(cv, dict(st, msg={
        "title": "ВЫБОР ПРОГРАММЫ", "right": f"{sel + 1}/{len(names)}",
        "items": [("→" if i == sel else " ") + n for i, n in enumerate(names)], "sel": sel,
        "bar": {BTN_PREV: "↑", BTN_NEXT: "↓", BTN_MODE: "КОТ", BTN_OK: "ПУСК"}}))


def scr_run_opts(cv, st):
    """Настройки проезда перед запуском: ПОЕХАЛИ, датчики ВКЛ/ВЫКЛ, назад."""
    ui = st["ui"]
    items = ui.get("run_items") or []
    osel = ui.get("osel", 0)
    what, text = items[osel] if items else ("go", "")
    if what == "go":
        ok = "ПУСК"
    elif what == "back":
        ok = "НАЗАД"
    else:
        ok = "ВКЛ" if text.endswith("ВЫКЛ") else "ВЫКЛ"
    scr_msg(cv, dict(st, msg={
        "title": "ПРОЕЗД", "items": [t for _, t in items], "sel": osel,
        "bar": {BTN_PREV: "↑", BTN_NEXT: "↓", BTN_MODE: "НАЗАД", BTN_OK: ok}}))


def scr_launch(cv, st):
    """Программу из меню запустили, а своего экрана она ещё не прислала."""
    ui = st["ui"]
    header(cv, "ЗАПУСК", "", st["now"], blink=True)
    block(cv, 0, 2, ui.get("label") or ui.get("child") or "", st["now"])
    line(cv, 3, "подожди…")


def compose(cv, st):
    """Выбрать экран по состоянию и нарисовать."""
    cv.clear()
    if st.get("msg"):
        return scr_msg(cv, st)
    snap = st.get("snap")
    # Меню и запуск из него - только у демона (st["ui"]); пульт их не видит.
    ui = st.get("ui") or {}
    live = snap is not None and st.get("connected") and st.get("stale", 0) <= STALE_S
    if ui.get("child") and not live:
        return scr_launch(cv, st)
    if ui.get("mode") == "menu" and not ui.get("busy") and not live:
        return scr_menu(cv, st)
    if ui.get("mode") == "run" and not ui.get("busy") and not live:
        return scr_run_opts(cv, st)
    if snap is None:
        return scr_idle(cv, st)
    if not st["connected"] or st.get("stale", 0) > STALE_S:
        return scr_gone(cv, st)
    ph = snap.get("phase")
    if ph == "wait_button":
        return scr_preflight(cv, st)
    if ph in ("boot", "calib"):
        return scr_boot(cv, st)
    if ph in WAIT_HINT:
        return scr_wait(cv, st)
    if ph in ("blinking", "done"):
        return scr_finish(cv, st)
    if st.get("victim"):
        return scr_victim(cv, st)
    return scr_run(cv, st)


# ══════════════════════════════════════════════════════════════
#  Демон
# ══════════════════════════════════════════════════════════════
def _log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


def _run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=2.0).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def _local_ip():
    """IP для SSH. hostname -I - даже без интернета и шлюза; первым - Wi-Fi."""
    ips = [a for a in _run(["hostname", "-I"]).split() if "." in a and not a.startswith("127.")]
    if ips:
        return ips[0]
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))       # ничего не шлёт, только выбор интерфейса
            return s.getsockname()[0]
    except OSError:
        return None


def _ssid():
    out = _run(["iwgetid", "-r"])
    if out:
        return out
    # Bookworm: NetworkManager, iwgetid может не стоять
    out = _run(["nmcli", "-t", "-f", "GENERAL.CONNECTION", "dev", "show", "wlan0"])
    name = out.partition(":")[2].strip()
    return name if name and name != "--" else None


def _read_num(path, scale=1.0):
    try:
        with open(path) as f:
            return float(f.read().split()[0]) / scale
    except (OSError, ValueError, IndexError):
        return None


def _cpu_temp():
    return _read_num("/sys/class/thermal/thermal_zone0/temp", 1000.0)


def _mem_used_pct():
    try:
        with open("/proc/meminfo") as f:
            kv = {ln.split(":")[0]: float(ln.split()[1]) for ln in f if ":" in ln}
        return 100.0 * (1.0 - kv["MemAvailable"] / kv["MemTotal"])
    except (OSError, ValueError, KeyError, ZeroDivisionError):
        return None


def _throttled():
    out = _run(["vcgencmd", "get_throttled"])      # "throttled=0x50000"
    try:
        return int(out.partition("=")[2], 16)
    except ValueError:
        return None


class SysInfo:
    """IP, Wi-Fi, температура, память, питание - раз в SYSINFO_S в своём
    потоке: vcgencmd/nmcli бывают по сотне миллисекунд, кадр ждать не должен."""

    def __init__(self, start=True):
        self.info = {"ip": None, "ssid": None, "temp": None, "mem": None,
                     "uptime": None, "throttled": None}
        if start:
            threading.Thread(target=self._run, daemon=True, name="oled-sys").start()

    def poll(self):
        ip = _local_ip()
        self.info = {"ip": ip, "ssid": _ssid() if ip else None, "temp": _cpu_temp(),
                     "mem": _mem_used_pct(), "uptime": _read_num("/proc/uptime"),
                     "throttled": _throttled()}

    def _run(self):
        while True:
            try:
                self.poll()
            except Exception as e:
                _log(f"[WARN] сведения о Pi: {type(e).__name__}: {e}")
            # Пока сети нет (сразу после загрузки) - чаще: IP нужен как можно раньше.
            time.sleep(SYSINFO_S if self.info["ip"] else 2.0)


class Feed:
    """Живой снимок nav.py (live.py). Переподключается сам: nav.py
    перезапускают, а демон живёт."""

    def __init__(self, start=True):
        self.snap = None
        self.t = 0.0                 # monotonic последнего снимка
        self.connected = False
        self.alert = None            # (текст, time.time() строки)
        self.lastline = None
        self.victim = None           # (метка из _victim_log, monotonic, когда пришла)
        self._vic_n = None           # сколько меток уже видели (None - ещё ни одной карты)
        self._hist = deque(maxlen=40)
        self.rates: dict[str, float | None] = {}
        self._lock = threading.Lock()
        if start:
            threading.Thread(target=self._run, daemon=True, name="oled-feed").start()

    def _run(self):
        try:
            import live
        except ImportError:
            # На Pi может стоять старый nav.py без живого потока - тогда
            # экраны заезда недоступны, остальное работает.
            _log("[WARN] нет live.py - экранов заезда не будет (nav.py без живого потока)")
            return
        while True:
            try:
                for s in live.read_lines(timeout=2.0):
                    self.connected = True
                    self.take(s, time.monotonic())
            except OSError:
                pass
            self.connected = False
            time.sleep(1.0)

    def take(self, s, now):
        for _, ts, text in s.pop("log", None) or []:
            text = text.strip()
            if not text:
                continue
            self.lastline = text
            if any(tag in text for tag in ALERT_TAGS):
                self.alert = (text, ts)
        s.pop("scan", None)
        m = s.pop("map", None)
        if m is not None:
            vics = m.get("victims") or []
            # Первая карта (подключились посреди заезда) - старые метки не
            # показываем; стало меньше - nav.py перезапустили.
            if self._vic_n is not None and len(vics) > self._vic_n:
                self.victim = (vics[-1], now)
            self._vic_n = len(vics)
        self._hist.append((now, s.get("telem_n"), s.get("scan_n")))
        old = next((h for h in self._hist if now - h[0] <= 1.5), None)
        if old is not None and now - old[0] > 0.5:
            dt = now - old[0]
            self.rates = {
                "esp": None if s.get("telem_n") is None or old[1] is None
                else (s["telem_n"] - old[1]) / dt,
                "lidar": None if s.get("scan_n") is None or old[2] is None
                else (s["scan_n"] - old[2]) / dt,
            }
        with self._lock:
            self.snap, self.t = s, now


class Inbox:
    """UDP-сообщения say(). Держит порт - он же замок «демон уже есть»."""

    def __init__(self, port=MSG_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))          # занят - OSError наверх
        self.port = self.sock.getsockname()[1]
        self.msg = None
        self.until = 0.0
        self.since = 0.0             # когда пришло сообщение с другим содержимым
        threading.Thread(target=self._run, daemon=True, name="oled-inbox").start()

    def _run(self):
        while True:
            try:
                data = self.sock.recv(8192)
                m = json.loads(data.decode())
            except (OSError, ValueError):
                continue
            if isinstance(m, dict):
                now = time.monotonic()
                if m != self.msg or now >= self.until:
                    self.since = now
                self.msg, self.until = m, now + float(m.get("ttl", MSG_TTL_S))

    def current(self, now):
        return self.msg if now < self.until else None


class TileGuess:
    """Плитка под роботом до старта - по калибровке tile_calib.py, тем же
    classify(), что в nav.py (nav.py сам до кнопки плитку не смотрит)."""

    def __init__(self):
        self._samples, self._t = {}, -1e9
        try:
            import tile_classify
            self._tc = tile_classify
        except ImportError:
            self._tc = None

    def __call__(self, rgbc, now):
        if self._tc is None or not rgbc or len(rgbc) < 4:
            return None
        if now - self._t > 5.0:                 # калибровку могли переснять
            self._samples, self._t = self._tc.load(), now
        return self._tc.classify(self._samples, *rgbc[:4])[0]


def _has_pinctrl():
    return any(os.access(os.path.join(d, "pinctrl"), os.X_OK)
               for d in os.environ.get("PATH", "/usr/bin").split(os.pathsep))


def build_state(now, feed, sysinfo, msg=None, tile=None):
    """Состояние для compose(): живой снимок (Feed), сведения о Pi (SysInfo),
    сообщение say() и плитка до старта. Им же рисует экран пульт
    (dash_web.py) - на странице ровно то, что на дисплее."""
    with feed._lock:
        snap, t_snap = feed.snap, feed.t
    st = {"now": now, "wall": time.time(), "msg": msg,
          "snap": snap, "connected": feed.connected, "stale": now - t_snap,
          "rates": feed.rates, "alert": feed.alert, "lastline": feed.lastline,
          "tile_guess": None, "victim": None, **sysinfo.info}
    if feed.victim is not None:
        v, age = feed.victim[0], now - feed.victim[1]
        busy = (snap or {}).get("act") == "жертва"
        if age < VICTIM_MIN_S or (busy and age < VICTIM_SHOW_S):
            st["victim"], st["victim_age"] = v, age
    if tile is not None and snap is not None and snap.get("phase") == "wait_button":
        st["tile_guess"] = tile(snap.get("rgbc"), now)
    return st


CAT_PATH = os.path.expanduser("~/maze/cat.oledv")   # oled_cat.py; нет файла - кота нет
BUTTON_GPIO = BTN_MODE       # та же кнопка, что старт nav.py (левая рука) и замер tile_calib.py
PANEL_PINS = (BTN_PREV, BTN_NEXT, BTN_MODE, BTN_OK)
BUTTON_POLL_S = 0.05
GPIO_USERS = ("nav.py", "tile_calib.py", "bringup.py", "neopixel_test.py")   # им кнопки нужны самим
MAZE_DIR = os.path.dirname(os.path.abspath(__file__))
# Что запускает меню на экране. Проезд - как с ноута, только без ноута.
PROGRAMS = (
    ("Проезд лабиринта", "nav", ("nav.py",)),
    ("Калибровка плиток", "calib", ("tile_calib.py",)),
    ("Тест неопикселей", "leds", ("neopixel_test.py",)),
)
STOP_WAIT_S = 3.0            # ОТМЕН: столько ждём после Ctrl+C, потом SIGTERM
# Настройки проезда - экран между меню и запуском: (что, флаг nav.py, когда
# ВЫКЛ). Выбор запоминается до следующего раза.
RUN_OPTIONS = (("датчик цвета", "--no-color"), ("камеры", "--no-vision"))
RUN_OPTS_PATH = os.path.expanduser("~/maze/menu_run.json")


def load_run_opts(path=RUN_OPTS_PATH):
    """{флаг: включено ли} - всё включено, если файла нет или он кривой."""
    opts = {flag: True for _, flag in RUN_OPTIONS}
    try:
        with open(path) as f:
            saved = json.load(f)
        opts.update({k: bool(v) for k, v in saved.items() if k in opts})
    except (OSError, ValueError, AttributeError):
        pass
    return opts


def save_run_opts(opts, path=RUN_OPTS_PATH):
    try:
        with open(path, "w") as f:
            json.dump(opts, f)
    except OSError as e:
        _log(f"[WARN] меню: настройки проезда не сохранились: {e}")


def parse_pinctrl(out):
    """Строка pinctrl get («27: ip    pu | hi // GPIO27 = input») ->
    (отпущена, подтяжка вверх стоит)."""
    head = out.split("//")[0]
    return "| lo" not in head, " pu " in head + " "


def parse_pinctrl_many(out):
    """Вывод pinctrl get для нескольких пинов -> {пин: (отпущена, подтяжка)}."""
    res = {}
    for ln in out.splitlines():
        num = ln.split(":", 1)[0].strip()
        if num.isdigit() and "|" in ln:
            res[int(num)] = parse_pinctrl(ln)
    return res


class Button:
    """Кнопка, которую демон читает, НЕ занимая пин.

    Кнопки нужны и nav.py (старт), и tile_calib.py/neopixel_test.py (их
    собственное управление). Если демон займёт пин (RPi.GPIO / lgpio), их
    GPIO.setup упадёт с «GPIO busy» - проверено на Pi. Поэтому уровень
    читается утилитой pinctrl (регистры, без захвата линии, ~1 мс), а
    подтяжка вверх ставится ей же - та же, что ставят они. Нажатие - LOW два
    опроса подряд после HIGH."""

    def __init__(self, pin=BUTTON_GPIO, read=None, start=True):
        self.pin = pin
        self.presses = 0
        self._read = read or self._pinctrl
        self._low = 0
        # Нажатие - только после того, как кнопку видели отпущенной: сразу
        # после загрузки пин читается низким, пока его не настроят, и кот
        # переключался на экран с IP сам.
        self._armed = False
        if read is None:
            prime_pins([pin])
        if start:
            threading.Thread(target=self._run, daemon=True, name="oled-button").start()

    def _pinctrl(self):
        return read_pins([self.pin]).get(self.pin, True)

    def poll(self, released=None):
        if released is None:
            released = self._read()
        if released:
            self._low = 0
            self._armed = True
            return
        self._low += 1
        if self._armed and self._low >= 2:
            self.presses += 1
            self._armed = False              # следующее - только после отпускания

    def _run(self):
        while True:
            try:
                self.poll()
            except (OSError, subprocess.SubprocessError):
                time.sleep(1.0)
            time.sleep(BUTTON_POLL_S)


def prime_pins(pins):
    """Настроить пины по-настоящему: занять с подтяжкой и сразу отпустить
    (одного pinctrl set после загрузки мало - пин всё равно читается
    низким). Пока их держит nav.py - не трогать."""
    if not gpio_users_running():
        try:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for p in pins:
                GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.cleanup(list(pins))
        except Exception:           # noqa: BLE001 - не на Pi или пин занят
            pass
    for p in pins:
        subprocess.run(["pinctrl", "set", str(p), "ip", "pu"], capture_output=True)


def read_pins(pins, run=subprocess.run):
    """{пин: отпущена} одним вызовом pinctrl на все пины. Подтяжку сбросили
    (GPIO.cleanup() в nav.py/bringup.py) - вернуть; этот отсчёт - «отпущена»
    (без подтяжки «низко» - не нажатие)."""
    out = run(["pinctrl", "get", ",".join(str(p) for p in pins)],
              capture_output=True, text=True).stdout
    got = parse_pinctrl_many(out)
    res = {}
    for p in pins:
        if p not in got:              # старый pinctrl без списков - по одному
            got[p] = parse_pinctrl(run(["pinctrl", "get", str(p)],
                                       capture_output=True, text=True).stdout)
        released, pull_ok = got[p]
        if not pull_ok:
            run(["pinctrl", "set", str(p), "ip", "pu"], capture_output=True)
            released = True
        res[p] = released
    return res


class ButtonPanel:
    """Четыре кнопки у малины: один опрос pinctrl на все раз в BUTTON_POLL_S."""

    def __init__(self, pins=PANEL_PINS, read=None, start=True):
        self.pins = tuple(pins)
        self._read = read or (lambda: read_pins(self.pins))
        self.buttons = {p: Button(p, read=lambda: True, start=False) for p in self.pins}
        if read is None:
            prime_pins(self.pins)
        if start:
            threading.Thread(target=self._run, daemon=True, name="oled-buttons").start()

    def poll(self):
        lv = self._read()
        for p, b in self.buttons.items():
            b.poll(lv.get(p, True))

    def presses(self):
        return {p: b.presses for p, b in self.buttons.items()}

    def _run(self):
        while True:
            try:
                self.poll()
            except (OSError, subprocess.SubprocessError):
                time.sleep(1.0)
            time.sleep(BUTTON_POLL_S)


def gpio_users_running(proc="/proc"):
    """Запущен ли кто-то, кому кнопки нужны самому (по cmdline)."""
    me = os.getpid()
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit() and int(p) != me]
    except OSError:
        return False
    for pid in pids:
        try:
            with open(os.path.join(proc, pid, "cmdline"), "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "oled.py" not in cmd and any(name in cmd for name in GPIO_USERS):
            return True
    return False


def restart_clock(t_start, showing, now):
    """Начало отсчёта анимации: пропала с экрана - None, снова появилась -
    с этого момента (то есть с первого кадра), иначе - прежнее."""
    if not showing:
        return None
    return now if t_start is None else t_start


class Ui:
    """Экран, когда его никто не занял: кот -> IP -> меню -> кот, по кругу
    кнопкой 27. В меню 22/24 листают, 25 - запуск. Проезд - через экран
    настроек («run»): курсор сразу на ПОЕХАЛИ (быстрый старт - 25 дважды),
    ниже - датчик цвета и камеры ВКЛ/ВЫКЛ (25 переключает), «назад»; 27 там
    - тоже назад в меню. Пока работает nav.py,
    tile_calib.py и т.п., кнопки - их (не переключают), а кота не видно;
    исключение - 22 на экране «жми старт» у заезда, запущенного из меню:
    это ОТМЕН. nav.py закончил - его итог (экран «IP»), не кот; калибровка
    или тест ленты закончились - снова меню."""

    MODES = ("cat", "ip", "menu")

    def __init__(self, n_items=len(PROGRAMS), opts_path=RUN_OPTS_PATH):
        self.mode = "cat"
        self.sel = 0
        self.n = n_items
        self.osel = 0                # пункт на экране настроек проезда
        self._opts_path = opts_path
        self.opts = load_run_opts(opts_path)
        self._seen = None
        self._was_connected = False

    def run_items(self):
        """Пункты экрана настроек: ("go"|"back"|флаг, подпись)."""
        return ([("go", "ПОЕХАЛИ!")]
                + [(flag, f"{name:<16}{'ВКЛ' if self.opts[flag] else 'ВЫКЛ':>4}")
                   for name, flag in RUN_OPTIONS]
                + [("back", "← назад в меню")])

    def launch_prog(self):
        """Что запустить: программа из меню, у проезда - с флагами настроек."""
        label, kind, argv = PROGRAMS[self.sel]
        if kind != "nav":
            return PROGRAMS[self.sel]
        offs = [(name, flag) for name, flag in RUN_OPTIONS if not self.opts[flag]]
        if offs:
            label = "Проезд без: " + ", ".join(name for name, _ in offs)
        return label, kind, (*argv, *(flag for _, flag in offs))

    def step(self, presses, busy, connected, can_cancel=False):
        """presses - {пин: сколько всего нажатий}. -> "launch", "cancel" или None."""
        seen = self._seen if self._seen is not None else presses
        new = {p: presses[p] - seen.get(p, 0) for p in presses}
        self._seen = dict(presses)
        if self._was_connected and not connected:
            self.mode = "ip"                 # заезд кончился - сначала его итог
        self._was_connected = connected
        if busy:
            return "cancel" if can_cancel and new.get(BTN_PREV, 0) > 0 else None
        k = new.get(BTN_MODE, 0)
        if k and self.mode == "run":
            self.mode = "menu"               # с настроек проезда 27 - назад
            return None
        if k:
            self.mode = self.MODES[(self.MODES.index(self.mode) + k) % len(self.MODES)]
            return None
        step = new.get(BTN_NEXT, 0) - new.get(BTN_PREV, 0)
        ok = new.get(BTN_OK, 0) > 0
        if self.mode == "run":
            items = self.run_items()
            self.osel = (self.osel + step) % len(items)
            if not ok:
                return None
            what = items[self.osel][0]
            if what == "go":
                return "launch"
            if what == "back":
                self.mode = "menu"
            else:
                self.opts[what] = not self.opts[what]
                save_run_opts(self.opts, self._opts_path)
            return None
        if self.mode != "menu":
            return None
        self.sel = (self.sel + step) % self.n
        if not ok:
            return None
        if PROGRAMS[self.sel][1] == "nav":
            self.mode, self.osel = "run", 0
            return None
        return "launch"

    def child_done(self, kind):
        self.mode = "ip" if kind == "nav" else "menu"

    def show_cat(self, busy, msg, snap, connected):
        return self.mode == "cat" and not busy and msg is None and (snap is None or not connected)


class Launcher:
    """Программа, запущенная из меню: свой сеанс (как ensure_daemon), вывод -
    в лог в ~/maze (заезд - run_ЧЧММ.log, как с ноута)."""

    def __init__(self, base=MAZE_DIR, python=sys.executable, popen=subprocess.Popen):
        self.base, self.python, self._popen = base, python, popen
        self.proc = None
        self.kind = None
        self.label = None

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, prog):
        label, kind, argv = prog
        if self.running():
            return False
        name = time.strftime("run_%H%M.log") if kind == "nav" else f"menu_{kind}.log"
        log_path = os.path.join(self.base, name)
        cmd = [self.python, "-u", os.path.join(self.base, argv[0]), *argv[1:]]
        try:
            with open(log_path, "a") as log_f:
                self.proc = self._popen(cmd, cwd=self.base, stdin=subprocess.DEVNULL, stdout=log_f,
                                        stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as e:
            _log(f"[WARN] меню: не запустился {argv[0]}: {e}")
            return False
        self.kind, self.label = kind, label
        _log(f"меню: запущен {' '.join(argv)} -> {log_path}")
        return True

    def poll(self):
        """Программа закончилась - её вид ("nav", "calib", ...), иначе None."""
        if self.proc is None or self.proc.poll() is None:
            return None
        kind = self.kind
        _log(f"меню: {kind} закончился, код {self.proc.returncode}")
        self.proc, self.kind, self.label = None, None, None
        return kind

    def stop(self):
        """Как Ctrl+C в терминале (nav.py по нему гасит моторы и ленту), не
        вышло - SIGTERM, потом SIGKILL. Не ждёт: всё в отдельном потоке."""
        proc = self.proc
        if proc is None or proc.poll() is not None:
            return

        def _kill():
            import signal
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(proc.pid, sig)
                except OSError:
                    return
                try:
                    proc.wait(STOP_WAIT_S)
                    return
                except subprocess.TimeoutExpired:
                    pass
        threading.Thread(target=_kill, daemon=True, name="oled-stop").start()


class Daemon:
    def __init__(self, rot180=ROTATE_180):
        self.inbox = Inbox()
        self.feed = Feed()
        self.oled = Ssd1306(rot180=rot180, log=_log)
        self.tile = TileGuess()
        self.sys = SysInfo()
        self._idle_since = None
        self._videos = {}            # путь -> (к/с, кадры)
        self.buttons = ButtonPanel() if _has_pinctrl() else None
        self.ui = Ui()
        self.launcher = Launcher()
        self.battery = BatteryProbe()
        self._cat_t0 = None          # когда кот появился на экране: с этого кадра - с начала
        self._busy, self._busy_t = False, -1e9
        self._presses_n = 0

    def state(self, now):
        return build_state(now, self.feed, self.sys, self.inbox.current(now), self.tile)

    def video(self, path):
        """(к/с, кадры) файла oled_video.py; не читается - None (и в лог)."""
        try:
            key = (path, os.path.getmtime(path))       # файл пересобрали - перечитать
        except OSError:
            key = (path, None)
        v = self._videos.get(key)
        if v is None:
            try:
                import oled_video
                v = oled_video.load(path)
            except (OSError, ValueError) as e:
                _log(f"[WARN] видео {path}: {e}")
                v = (0, [])
            self._videos[key] = v
        return v if v[1] else None

    def buttons_step(self, st, t0):
        """Кнопки -> режим экрана, запуск и остановка программ из меню."""
        if t0 - self._busy_t > 0.5:
            self._busy, self._busy_t = gpio_users_running(), t0
        done = self.launcher.poll()
        if done:
            self.ui.child_done(done)
        busy = self._busy or self.launcher.running()
        presses = self.buttons.presses() if self.buttons else {}
        snap = st["snap"] or {}
        can_cancel = (self.launcher.kind == "nav" and st["connected"]
                      and snap.get("phase") == "wait_button")
        act = self.ui.step(presses, busy, st["connected"], can_cancel)
        if act == "launch":
            if self.launcher.start(self.ui.launch_prog()):
                busy, self._busy_t = True, -1e9
        elif act == "cancel":
            self.launcher.stop()
        n = sum(presses.values())
        if n != self._presses_n:             # нажали - экран снова яркий
            self._presses_n, self._idle_since = n, None
        # Порт ESP читаем сами, только пока никто из программ не запущен.
        self.battery.wanted = not busy
        st["ui"] = {"mode": self.ui.mode, "sel": self.ui.sel, "buttons": self.buttons is not None,
                    "osel": self.ui.osel, "run_items": self.ui.run_items(),
                    "busy": busy, "child": self.launcher.kind, "label": self.launcher.label,
                    "volts": self.battery.volts(t0)}
        return busy

    def run(self):
        _log(f"дисплей: {I2C_DEV}, поворот {'180°' if self.oled.rot180 else '0°'}")
        cv = Canvas()
        while True:
            t0 = time.monotonic()
            period = 1.0 / FPS
            try:
                st = self.state(t0)
                busy = self.buttons_step(st, t0)
                m = st["msg"]
                v = self.video(m["video"]) if m and m.get("video") else None
                t_from = self.inbox.since
                cat = None
                if v is None and self.ui.show_cat(busy, m, st["snap"], st["connected"]):
                    cat = v = self.video(CAT_PATH)
                self._cat_t0 = restart_clock(self._cat_t0, cat is not None, t0)
                if self._cat_t0 is not None:
                    t_from = self._cat_t0
                if v is not None:
                    fps, frames = v
                    period = 1.0 / max(1, fps)
                    cv.buf[:] = frames[int((t0 - t_from) * fps) % len(frames)]
                else:
                    compose(cv, st)
                    if st.get("victim") and not m:
                        import oled_moods
                        period = 1.0 / oled_moods.FPS     # кот у метки двигается
                # Простой (никто ничего не шлёт) дольше IDLE_DIM_S - тускло.
                idle = st["msg"] is None and st["snap"] is None
                dim = False
                if not idle:
                    self._idle_since = None
                elif self._idle_since is None:
                    self._idle_since = t0
                else:
                    dim = t0 - self._idle_since > IDLE_DIM_S
                self.oled.contrast = DIM_CONTRAST if dim else CONTRAST
            except Exception as e:              # кривой снимок не должен гасить экран
                _log(f"[WARN] экран не нарисовался: {type(e).__name__}: {e}")
                cv.clear()
                header(cv, "ОШИБКА ЭКРАНА")
                block(cv, 0, 5, f"{type(e).__name__}: {e}", t0)
            self.oled.show(cv)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))


# ══════════════════════════════════════════════════════════════
#  Образцы экранов (--png): видно, как всё ляжет, без робота
# ══════════════════════════════════════════════════════════════
def demo_states():
    now, wall = 100.0, time.time()
    snap = {
        "phase": "exploring", "run_s": 168.4, "t_left": 301.6, "act": "поворот",
        "pose": {"cx": 3, "cy": -1, "hdg": 1, "cp": [2, 0, 0], "level": 0},
        "drive": {"cell": 2, "rem_mm": 120, "spd": 18.0, "mode": None},
        "walls": {"front": 263.0, "right": 95.0, "left": None, "back": 81.0},
        "tile": "white", "victims_n": 2, "kits": {"L": 4, "R": 3}, "visited_n": 34,
        "cams": {"left": {"ready": True, "armed": True}, "right": {"ready": True, "armed": False}},
        "link": {"crc": 0, "lost": 0, "cmd_err": 0}, "color_on": True,
        "rgbc": [120, 140, 150, 410],
    }
    base = {"now": now, "wall": wall, "msg": None, "connected": True, "stale": 0.1,
            "rates": {"esp": 198.0, "lidar": 10.0}, "alert": None,
            "lastline": "Инициализация ленты индикации...", "ip": "192.168.78.26",
            "temp": 61.0, "tile_guess": "white", "ssid": "robocup-maze", "mem": 18.0,
            "uptime": 754.0, "throttled": 0}

    def mk(**kw):
        s = dict(base)
        s["snap"] = dict(snap, **kw.pop("snap", {}))
        s.update(kw)
        return s
    return {
        "1_idle": dict(base, snap=None),
        "1a_idle_hours": dict(base, snap=None, uptime=3 * 3600 + 25 * 60, temp=None, mem=None),
        "1b_idle_no_net": dict(base, snap=None, ip=None, ssid=None, uptime=21.0),
        "1c_idle_undervolt": dict(base, snap=None, throttled=0x50005),
        "2_boot": mk(snap={"phase": "boot", "run_s": None}),
        "3_preflight": mk(snap={"phase": "wait_button", "run_s": None, "drive": None,
                                "cams": {"left": {"ready": False}, "right": {"ready": True}}}),
        "4_run": mk(),
        "5_run_alert": mk(alert=("[WARN] камера справа: кадр не пришёл за 1.0с", wall - 2)),
        "6_wait_motors": mk(snap={"phase": "wait_motors", "drive": None}),
        "7_returning": mk(snap={"phase": "returning", "drive": None, "act": "лидар",
                                "pose": {"cx": 1, "cy": 1001, "hdg": 3, "cp": [2, 1001, 0],
                                         "level": 1}}),
        "8_finish": mk(snap={"phase": "done", "run_s": 401.0, "drive": None}),
        "9_gone": mk(connected=False),
        "10_tile_calib": dict(base, snap=None, msg={
            "title": "КАЛИБР. ПЛИТОК", "small": True, "lines": [
                "цвет: зеркальная", "видит: зерк d=1.4 C=412",
                "бе3 чё2 си1 кр0 зе2 во1",
                "! C гулял на 15% (робот двигался?) - не сохранено, нажми ещё раз"]}),
        "12_run_long_alert": mk(alert=(
            "[WARN] нет калибровки цвета (~/maze/tile_calib.json, нужны хотя бы белая "
            "и чёрная) - старые пороги, ЧЕКПОИНТЫ НЕ РАСПОЗНАЮТСЯ", wall - 2)),
        "11_test": dict(base, snap=None, msg=_test_msg()),
        "13_run_black_home": mk(snap={"tile": "black", "phase": "returning", "kits": {"L": 0, "R": 2}}),
        "14_victim_phi": mk(victim={"type": "damaged", "class": "phi", "label": "Φ"}),
        "15_victim_psi": mk(victim={"type": "stable", "class": "psi", "label": "Ψ"}),
        "16_victim_omega": mk(victim={"type": "unharmed", "class": "omega", "label": "Ω"}),
        "17_victim_cog": mk(victim={"type": "stable", "class": "cognetive", "label": "+1",
                                    "rings": ["green", "red", "yellow", "blue", "red"],
                                    "checksum": 1}),
        "18_victim_cog_fake": mk(victim={"type": "fake", "class": "cognetive", "label": "+3",
                                         "rings": ["blue", "green", "blue", "yellow", "black"],
                                         "checksum": 3}),
        # Меню и программы из него (кнопки у малины)
        "19_ip_buttons": dict(base, snap=None, ui={"mode": "ip", "buttons": True}),
        "19a_ip_volts": dict(base, snap=None, ui={"mode": "ip", "buttons": True, "volts": 14.8}),
        "19b_ip_no_volts": dict(base, snap=None, ui={"mode": "ip", "buttons": True, "volts": None}),
        "20_menu": dict(base, snap=None, ui={"mode": "menu", "sel": 1, "buttons": True}),
        "20a_run_opts": dict(base, snap=None, ui={
            "mode": "run", "osel": 1, "buttons": True,
            "run_items": [("go", "ПОЕХАЛИ!"), ("--no-color", "датчик цвета    ВЫКЛ"),
                          ("--no-vision", "камеры           ВКЛ"), ("back", "← назад в меню")]}),
        "21_launch": dict(base, snap=None, ui={"mode": "menu", "busy": True, "child": "nav",
                                               "label": "Проезд лабиринта"}),
        "22_preflight_menu": mk(snap={"phase": "wait_button", "run_s": None, "drive": None},
                                ui={"mode": "menu", "busy": True, "child": "nav"}),
        "23_calib_pick": dict(base, snap=None, msg={
            "title": "ПЛИТКИ", "right": "видит бел", "sel": 2,
            "items": ["белая            3 шт", "чёрная           2 шт", "синяя            1 шт",
                      "красная          0 шт", "зеркальная       2 шт", "воздух           1 шт",
                      "← выход в меню"],
            "bar": {"22": "↑", "24": "↓", "25": "ЗАМЕР"}}),
        "24_calib_measure": dict(base, snap=None, msg={
            "title": "ЗАМЕР - НЕ ТРОГАЙ", "alert": True, "progress": 0.6,
            "lines": ["цвет: синяя", "C=118  лента погашена"]}),
        "25_calib_bad": dict(base, snap=None, msg={
            "title": "НЕ СОХРАНЕНО", "lines": ["C гулял на 18% - робот двигался? Поставь ровно и нажми ЕЩЁ"],
            "bar": {"22": "↑", "24": "↓", "25": "ЕЩЁ"}}),
        "26_leds": dict(base, snap=None, msg={
            "title": "ТЕСТ ЛЕНТЫ", "right": "2/7", "lines": ["красный", "горят все 4"],
            "bar": {"22": "←", "24": "→", "25": "ВЫХОД"}}),
        "27_victim_phi_late": mk(victim={"type": "damaged", "class": "phi", "label": "Φ"},
                                 victim_age=1.3),
    }


def _test_msg():
    return {"title": "ТЕСТ ДИСПЛЕЯ", "big": "↑ ВЕРХ ↑",
            "lines": ["эта строка - низ", f"поворот {'180' if ROTATE_180 else '0'}°",
                      "АБВГД абвгд 0123"], "ttl": 15.0}


def main():
    args = sys.argv[1:]
    if "--png" in args:
        out = args[args.index("--png") + 1] if len(args) > args.index("--png") + 1 else "."
        os.makedirs(out, exist_ok=True)
        cv = Canvas()
        for name, st in demo_states().items():
            compose(cv, st)
            cv.png(os.path.join(out, f"{name}.png"))
        print(f"образцы экранов: {out}")
        return
    if "--say" in args:
        # Из shell: oled.py --say "ШАПКА" "строка 1" "строка 2" (держится 10 с)
        i = args.index("--say")
        rest = args[i + 1:]
        if ensure_daemon() is not None:          # только что поднят - ждём порт
            for _ in range(30):
                if daemon_running():
                    break
                time.sleep(0.1)
        if rest:
            say(rest[0], rest[1:], ttl=10.0)
        return
    if "--play" in args or "--stop" in args:
        # Видео по кругу (oled_video.py), пока не --stop или не перезапуск демона
        if ensure_daemon() is not None:
            for _ in range(30):
                if daemon_running():
                    break
                time.sleep(0.1)
        if "--stop" in args:
            say("", ttl=0.0)
            print("видео остановлено")
        else:
            path = args[args.index("--play") + 1]
            say("", video=path, ttl=10 * 365 * 86400.0)
            print(f"играет {path} по кругу; остановить: oled.py --stop")
        return
    if "--test" in args:
        # Через демон - заодно проверяется весь путь say() -> экран.
        ensure_daemon()
        m = _test_msg()
        end = time.monotonic() + 12.0
        while time.monotonic() < end:
            say(m["title"], m["lines"], big=m["big"], ttl=1.0)
            time.sleep(0.5)
        print("Надпись «↑ ВЕРХ ↑» должна быть сверху и читаться стоя за роботом.\n"
              "Вверх ногами - поменяй ROTATE_180 в oled.py и перезапусти демон "
              "(pkill -f oled.py).  Пусто - смотри ~/maze/oled.log")
        return
    rot = ROTATE_180
    if "--rot0" in args:
        rot = False
    try:
        d = Daemon(rot180=rot)
    except OSError:
        return                       # порт занят - демон уже работает
    d.run()


if __name__ == "__main__":
    main()
