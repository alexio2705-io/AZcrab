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
    python3 oled.py --png /tmp/oled                  # образцы экранов в PNG, без железа
"""
from __future__ import annotations

import fcntl
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
                if data != self.shown[p]:
                    self._cmd(0x21, 0, W - 1, 0x22, p, p)
                    os.write(self.fd, b"\x40" + data)
                    self.shown[p] = data
            return True
        except OSError as e:
            self._fail(e)
            return False


# ══════════════════════════════════════════════════════════════
#  Сообщения от скриптов (калибровки и т.п.)
# ══════════════════════════════════════════════════════════════
_tx = None


def say(title, lines=(), big=None, alert=False, small=False, ttl=MSG_TTL_S, port=MSG_PORT):
    """Показать экран на дисплее, пока его шлют (и ещё ttl секунд).

    title - шапка, lines - строки под ней (6x10: 5 строк по 21 символу,
    small=True - 5x8: 6 строк по 25), длинные прокручиваются сами. big -
    крупная строка (до 10 символов) вместо первых двух строк. alert -
    шапка мигает. Никогда не бросает и не ждёт."""
    global _tx
    msg = {"title": str(title), "lines": [str(s) for s in lines][:6], "big": big,
           "alert": bool(alert), "small": bool(small), "ttl": float(ttl)}
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
    "wait_lidar": "ЛИДАР МОЛЧИТ",
}
# Ждём человека: шапка мигает, ниже - что сделать (переносится по словам).
WAIT_HINT = {
    "wait_link": "проверь ESP и провод к Pi - заезд продолжится сам",
    "wait_motors": "включи питание моторов - заезд продолжится сам",
    "wait_lidar": "проверь кабель и питание лидара",
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


def first_fit(*options, n=COLS):
    """Первый вариант строки, что влезает целиком (от подробного к краткому);
    не влез ни один - последний, обрезанный."""
    for s in options:
        if len(s) <= n:
            return s
    return clip(options[-1], n)


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


def _walls(snap):
    # Стрелки - относительно робота (человек стоит сзади, ↑ - его перед).
    # Буквами не вышло: «З» в 6x10 от «3» не отличить.
    w = snap.get("walls") or {}
    return (f"↑{_cm(w.get('front'))} →{_cm(w.get('right'))} "
            f"←{_cm(w.get('left'))} ↓{_cm(w.get('back'))} см")


def _kits(snap):
    k = snap.get("kits") or {}
    return f"кит{k.get('L', '-')}/{k.get('R', '-')}"


def _health(st):
    snap = st["snap"]
    r = st.get("rates") or {}
    esp, lid = r.get("esp"), r.get("lidar")
    s = f"ESP{'--' if esp is None else f'{esp:.0f}'} лид{'--' if lid is None else f'{lid:.0f}'}"
    lk = snap.get("link") or {}
    errs = (lk.get("crc") or 0) + (lk.get("lost") or 0) + (lk.get("cmd_err") or 0)
    if errs:
        s += f" ош{errs}"
    return s + " " + cams_str(snap.get("cams")).replace(" ", "")


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
    m, t = st["msg"], st["now"]
    header(cv, m.get("title", ""), t=t, blink=m.get("alert"))
    small = bool(m.get("small")) and not m.get("big")
    n = COLS_SMALL if small else COLS
    rows = [r for s in (m.get("lines") or []) for r in wrap(s, n)]
    if m.get("big"):
        big(cv, 13, str(m["big"]))
        for i, s in enumerate(paged(rows, 3, t)):
            line(cv, i + 2, s)
    elif small:
        for i, s in enumerate(paged(rows, 6, t)):
            small_line(cv, i, s)
    else:
        for i, s in enumerate(paged(rows, 5, t)):
            line(cv, i, s)


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
    if pw.startswith("!"):
        line(cv, 3, pw, inv=True)
    else:
        line2(cv, 3, pw, f"вкл {_uptime(st.get('uptime'))}")
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
    line(cv, 1, f"время {mmss(snap.get('run_s'))} жертв{snap.get('victims_n', 0)}")
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
    line(cv, 4, f"IP {st.get('ip') or '—'}")


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


def scr_run(cv, st):
    t, snap = st["now"], st["snap"]
    ph = snap.get("phase")
    tl = snap.get("t_left")
    late = tl is not None and tl < 60 and ph in ("exploring", "time", "returning")
    header(cv, PHASE.get(ph, ph or "?"), mmss(snap.get("run_s")), t, blink=late)
    line(cv, 0, _pos(snap))
    d = snap.get("drive")
    if d:
        what = {"ramp": "горка", "kick": "толчок"}.get(d.get("mode"), "ход")
        spd = d.get("spd")
        spd = "-" if spd is None else round(spd)
        cell, rem = d.get("cell"), d.get("rem_mm")
        # «толчок кл1 285мм 19см/с» - 23 символа: сначала без номера клетки,
        # потом без «/с»
        s = first_fit(f"{what} кл{cell} {rem}мм {spd}см/с", f"{what} {rem}мм {spd}см/с",
                      f"{what} {rem}мм {spd}см")
    else:
        s = snap.get("act") or snap.get("status") or "стоит"
    line(cv, 1, s)
    line(cv, 2, _walls(snap))
    a = _alert(st)
    if ph == "sleep":
        line(cv, 3, f"{TILE.get(snap.get('tile'), '—')} жертв{snap.get('victims_n', 0)} "
                    f"{_kits(snap)}")
        line(cv, 4, "рестарт с чекпоинта", inv=True)
    elif a:
        # Предупреждение - на две нижние строки, инверсией, пока свежее.
        block(cv, 3, 2, a, t, inv=True)
    else:
        tile = snap.get("tile")
        line(cv, 3, f"{TILE.get(tile, tile or '—')} жертв{snap.get('victims_n', 0)} "
                    f"{_kits(snap)}")
        line(cv, 4, _health(st))


def scr_finish(cv, st):
    t, snap = st["now"], st["snap"]
    ph = snap.get("phase")
    header(cv, PHASE.get(ph, ph), "", t, blink=ph == "blinking")
    big(cv, 13, mmss(snap.get("run_s")))
    line(cv, 2, f"жертв {snap.get('victims_n', 0)}  {_kits(snap)}")
    line(cv, 3, f"клеток {snap.get('visited_n', '—')}")
    line(cv, 4, f"IP {st.get('ip') or '—'}")


def compose(cv, st):
    """Выбрать экран по состоянию и нарисовать."""
    cv.clear()
    if st.get("msg"):
        return scr_msg(cv, st)
    snap = st.get("snap")
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
        self._hist = deque(maxlen=40)
        self.rates: dict[str, float | None] = {}
        self._lock = threading.Lock()
        if start:
            threading.Thread(target=self._run, daemon=True, name="oled-feed").start()

    def _run(self):
        import live
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
        s.pop("map", None)
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
        threading.Thread(target=self._run, daemon=True, name="oled-inbox").start()

    def _run(self):
        while True:
            try:
                data = self.sock.recv(4096)
                m = json.loads(data.decode())
            except (OSError, ValueError):
                continue
            if isinstance(m, dict):
                self.msg, self.until = m, time.monotonic() + float(m.get("ttl", MSG_TTL_S))

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


def build_state(now, feed, sysinfo, msg=None, tile=None):
    """Состояние для compose(): живой снимок (Feed), сведения о Pi (SysInfo),
    сообщение say() и плитка до старта. Им же рисует экран пульт
    (dash_web.py) - на странице ровно то, что на дисплее."""
    with feed._lock:
        snap, t_snap = feed.snap, feed.t
    st = {"now": now, "wall": time.time(), "msg": msg,
          "snap": snap, "connected": feed.connected, "stale": now - t_snap,
          "rates": feed.rates, "alert": feed.alert, "lastline": feed.lastline,
          "tile_guess": None, **sysinfo.info}
    if tile is not None and snap is not None and snap.get("phase") == "wait_button":
        st["tile_guess"] = tile(snap.get("rgbc"), now)
    return st


class Daemon:
    def __init__(self, rot180=ROTATE_180):
        self.inbox = Inbox()
        self.feed = Feed()
        self.oled = Ssd1306(rot180=rot180, log=_log)
        self.tile = TileGuess()
        self.sys = SysInfo()
        self._idle_since = None

    def state(self, now):
        return build_state(now, self.feed, self.sys, self.inbox.current(now), self.tile)

    def run(self):
        _log(f"дисплей: {I2C_DEV}, поворот {'180°' if self.oled.rot180 else '0°'}")
        cv = Canvas()
        period = 1.0 / FPS
        while True:
            t0 = time.monotonic()
            try:
                st = self.state(t0)
                compose(cv, st)
                # Простой (никто ничего не шлёт) дольше IDLE_DIM_S - тускло.
                idle = st["msg"] is None and st["snap"] is None
                if not idle:
                    self._idle_since = None
                elif self._idle_since is None:
                    self._idle_since = t0
                dim = idle and t0 - self._idle_since > IDLE_DIM_S
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
        "4b_run_kick": mk(snap={"drive": {"cell": 1, "rem_mm": 285, "spd": 19.4, "mode": "kick"}}),
        "12_run_long_alert": mk(alert=(
            "[WARN] нет калибровки цвета (~/maze/tile_calib.json, нужны хотя бы белая "
            "и чёрная) - старые пороги, ЧЕКПОИНТЫ НЕ РАСПОЗНАЮТСЯ", wall - 2)),
        "11_test": dict(base, snap=None, msg=_test_msg()),
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
