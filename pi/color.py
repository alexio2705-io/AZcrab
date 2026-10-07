#!/usr/bin/env python3
"""Сырые каналы APDS9960 - живой вывод и калибровка порогов распознавания плиток.

Датчик настроен на усиление 16x и ATIME=0xF6 (~28мс), поэтому потолок АЦП у
всех четырёх каналов - 10250. Значение вплотную к нему означает НАСЫЩЕНИЕ:
плитка ярче, чем датчик способен измерить, и различать по таким числам уже
нечего - тогда надо снижать усиление в apds9960.c.

ВАЖНО про "прошивка: MIRROR". Байт цвета из телеметрии - это результат
apds9960_detect_color() на ESP, и nav.py им НЕ ПОЛЬЗУЕТСЯ вовсе: tile_color()
классифицирует плитку сам, по сырым каналам. То есть MIRROR в консоли ESP на
проезд лабиринта не влияет никак, это только индикатор того, что пороги в
apds9960.c устарели. Исправить их всё равно стоит - см. --calib.

И ещё: RGBC у APDS9960 - ПАССИВНЫЙ замер окружающего света. Своей подсветки у
цветовых каналов нет (светодиод на плате работает только на датчик
приближения и жесты). Поэтому "робот в воздухе" - это НЕ темнота: датчик
видит свет комнаты, и C там может оказаться и больше, чем над белой плиткой.
Различать плитки датчик может лишь потому, что прижат к полу и меряет
отражённый от плитки свет комнаты.

Читает ту же телеметрию, что и nav.py (ESP шлёт её всегда), поэтому робот
трогать не нужно - просто поставьте его на плитку.

Запуск на малине:
    ~/maze/venv/bin/python ~/maze/color.py            живой вывод
    ~/maze/venv/bin/python ~/maze/color.py 3          среднее за 3 секунды
    ~/maze/venv/bin/python ~/maze/color.py --calib    калибровка по плиткам
"""

from __future__ import annotations

import binascii
import struct
import sys
import threading
import time

import serial

LINK_PORT = "/dev/serial0"
LINK_BAUD = 921600

MAGIC = b"\xAA\x55"
# ДОЛЖНО СОВПАДАТЬ с telem_t и TELEM_VER в robot_link.c и с FMT в nav.py.
# ver seq yaw yaw_rate pitch roll ax ay az color cr cg cb cc left_cm right_cm
# status flags z_cnt boot_id cmd_err crc16
TELEM_VER = 3
FMT = "<BBfffffffBHHHHffBBBHHH"
BODY_SIZE = struct.calcsize(FMT)          # 56
PKT_SIZE = len(MAGIC) + BODY_SIZE         # 58
FLAG_COLOR_FRESH = 64                     # см. flags в robot_link.h

# Разборщик чужой версии НЕ замолкает: магия 0xAA55 попадается и внутри
# данных, а XOR-сумма пропускает ~1 мусорный кадр из 300. При 200 Гц это
# примерно строка в секунду - и именно так выглядел "сошедший с ума датчик
# цвета": R=43102 B=57344 при C=316 и невозможном байте цвета 0. Настоящий
# APDS9960 при ATIME=0xF6 выше 10250 не выдаёт, а apds9960_detect_color()
# нуля не возвращает никогда. Отсюда байт версии и счётчик ниже.
_magic_gaps = {}
_bad_ver = 0


def link_report():
    if not _magic_gaps:
        return ("маркеров 0xAA55 в потоке нет вовсе - ESP молчит, "
                "не та скорость или не тот порт")
    gap, hits = max(_magic_gaps.items(), key=lambda kv: kv[1])
    if gap == PKT_SIZE and not _bad_ver:
        return f"пакет {gap} байт, версия {TELEM_VER} - сходится"
    tail = f"; пакетов с чужой версией: {_bad_ver}" if _bad_ver else ""
    return (f"ESP шлёт пакеты по {gap} байт ({hits} раз), а этот color.py ждёт "
            f"{PKT_SIZE} и версию {TELEM_VER}{tail} -> "
            f"ПРОШИВКА И .PY РАЗНЫХ ВЕРСИЙ, перезалей обе стороны")

ADC_MAX = 10250          # потолок при ATIME=0xF6
COLORS = {0: "?", 1: "WHITE", 2: "BLUE", 3: "BLACK", 4: "MIRROR", 5: "RED"}

_lock = threading.Lock()
_last = None             # (r, g, b, c, color)
_link = None


def _reader():
    buf = bytearray()
    global _last, _bad_ver
    since = 0
    have_prev = False
    color_was = True     # о пропаже свежести цвета - один раз
    while True:
        try:
            chunk = _link.read(max(1, min(_link.in_waiting, 4096)))
        except (serial.SerialException, OSError, TypeError):
            if _link is not None and _link.is_open:
                print("\n  [WARN] ошибка порта ESP - переподключаюсь")
                while True:
                    time.sleep(0.5)
                    try:
                        _link.close()
                        _link.open()
                        break
                    except (serial.SerialException, OSError):
                        pass
                buf.clear()
                have_prev = False
                continue
            return
        if not chunk:
            continue
        buf.extend(chunk)
        while len(buf) >= PKT_SIZE:
            if buf[0] != MAGIC[0] or buf[1] != MAGIC[1]:
                del buf[0]
                since += 1
                continue
            if have_prev and len(_magic_gaps) < 64:
                _magic_gaps[since] = _magic_gaps.get(since, 0) + 1
            have_prev = True
            since = 0
            body = bytes(buf[2:PKT_SIZE])
            # Случайный маркер в мусоре: при ошибке съедаем 1 байт, а не весь
            # кадр. Чужая версия - кадр, за которым сразу следующий маркер.
            if body[0] != TELEM_VER:
                if (len(buf) >= PKT_SIZE + 2 and buf[PKT_SIZE] == MAGIC[0]
                        and buf[PKT_SIZE + 1] == MAGIC[1]):
                    _bad_ver += 1
                del buf[0]
                since = 1
                continue
            # CRC-16/CCITT-FALSE, как в robot_link.c
            if binascii.crc_hqx(body[:-2], 0xFFFF) != body[-2] | (body[-1] << 8):
                del buf[0]
                since = 1
                continue
            since = PKT_SIZE
            del buf[:PKT_SIZE]
            vals = struct.unpack(FMT, body)
            # ver, seq + семь float'ов -> color на индексе 9, каналы 10..13
            color, cr, cg, cb, cc = vals[9], vals[10], vals[11], vals[12], vals[13]
            fresh = bool(vals[17] & FLAG_COLOR_FRESH)
            if fresh != color_was:
                print("\n[WARN] ESP: датчик цвета не читается - числа ниже старые"
                      if not fresh else "\nESP: датчик цвета снова читается")
                color_was = fresh
            with _lock:
                _last = (cr, cg, cb, cc, color)


# ── ЗЕРКАЛО порогов из nav.py ────────────────────────────────
# Держать в согласии с nav.py вручную. Нужны, чтобы в живом выводе сразу было
# видно не только сырые числа, но и КАКОЕ РЕШЕНИЕ по ним примет робот - иначе
# подбор порогов превращается в перенос цифр туда-сюда на глаз.
NAV_RED_RG_THRESH = 0.10
NAV_RED_MIN_C_FRACTION = 0.15
NAV_BLUE_RATIO_THRESH = 0.28
NAV_BLACK_FRACTION = 0.15
NAV_BLACK_ABS_MAX_C = 180
NAV_TILE_MIN_C = 8
NAV_AIR_RB = 0.16        # (r-b)/c выше этого - это не плитка, а воздух
# Калибровка по белой снимается в nav.py на стартовой клетке. Здесь её нет,
# поэтому берём замеренное значение как опору - поменять, если менялся зал.
NAV_C_WHITE = 335


def nav_verdict(r, g, b, c):
    """Что решил бы tile_color() из nav.py на этих каналах.

    Одно отличие: nav.py берёт C белой из calibrate_white() на стартовой
    клетке, а здесь калибровки нет - подставляется NAV_C_WHITE.
    """
    if c < NAV_TILE_MIN_C:
        return "?      (C ниже TILE_MIN_C)"
    if (r - b) / c > NAV_AIR_RB:
        return "?      (не плитка - воздух)"
    if ((r - g) / c > NAV_RED_RG_THRESH
            and c > NAV_C_WHITE * NAV_RED_MIN_C_FRACTION):
        return "RED"
    if (b - r) / c > NAV_BLUE_RATIO_THRESH:
        return "BLUE"
    if c < min(NAV_BLACK_ABS_MAX_C, NAV_C_WHITE * NAV_BLACK_FRACTION):
        return "BLACK"
    return "WHITE"


def _fmt(r, g, b, c, color, n=None):
    # Доли каналов в прозрачном - именно по ним плитки различаются устойчивее,
    # чем по абсолютным значениям: они почти не зависят от высоты датчика над
    # полом и от общей освещённости.
    if c > 0:
        frac = (f"r/c={r/c:.3f} g/c={g/c:.3f} b/c={b/c:.3f}  "
                f"(b-r)/c={(b-r)/c:+.3f} (r-b)/c={(r-b)/c:+.3f}")
    else:
        frac = "доли не считаются: C=0"
    sat = "  ← НАСЫЩЕНИЕ" if max(r, g, b, c) >= ADC_MAX * 0.98 else ""
    avg = f" (среднее по {n} замерам)" if n else ""
    return (f"R={r:5d} G={g:5d} B={b:5d} C={c:5d}   {frac}\n"
            f"      nav.py: {nav_verdict(r, g, b, c):<12s} "
            f"прошивка: {COLORS.get(color, '?')}{sat}{avg}")


def _connect():
    global _link
    try:
        _link = serial.Serial(LINK_PORT, LINK_BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"[FATAL] Порт {LINK_PORT}: {e}")
        sys.exit(1)
    threading.Thread(target=_reader, daemon=True).start()
    t0 = time.time()
    while _last is None:
        if time.time() - t0 > 5:
            print("[FATAL] Нет телеметрии с ESP.")
            print("        " + link_report())
            sys.exit(1)
        time.sleep(0.05)
    # Пакеты пошли - но, может быть, чужие. Сказать об этом сразу, а не дать
    # человеку полчаса подбирать пороги по мусорным числам.
    time.sleep(0.5)
    if _bad_ver or (_magic_gaps and
                    max(_magic_gaps.items(), key=lambda kv: kv[1])[0] != PKT_SIZE):
        print("[FATAL] " + link_report())
        sys.exit(1)


def _average(secs):
    """Среднее по каналам за secs секунд -> (r, g, b, c, последний байт цвета)."""
    acc = [0, 0, 0, 0]
    n = 0
    last_color = 0
    end = time.time() + secs
    while time.time() < end:
        with _lock:
            r, g, b, c, last_color = _last
        acc[0] += r; acc[1] += g; acc[2] += b; acc[3] += c
        n += 1
        time.sleep(0.02)
    r, g, b, c = (v // max(n, 1) for v in acc)
    return r, g, b, c, last_color, n


# ── Калибровка ───────────────────────────────────────────────
# Что снимаем и в каком порядке. "воздух" нужен не для плиток, а для
# детектора подъёма в nav.py: он должен знать, чем показание на весу
# отличается от показания на полу.
CALIB_STEPS = [
    ("white", "БЕЛУЮ плитку"),
    ("black", "ЧЁРНУЮ плитку"),
    ("blue",  "СИНЮЮ плитку"),
    ("red",   "КРАСНУЮ плитку (чекпоинт)"),
    ("air",   "поднимите робота в воздух и ДЕРЖИТЕ"),
]
CALIB_SECS = 3.0


def _mid(lo, hi):
    """Порог посередине между двумя наблюдёнными значениями."""
    return (lo + hi) / 2.0


def calibrate():
    print("Калибровка датчика цвета. На каждом шаге ставьте робота и жмите Enter.\n"
          f"Каждый замер усредняется {CALIB_SECS:.0f}с - робота в это время не двигать.\n")
    data = {}
    for key, prompt in CALIB_STEPS:
        try:
            ans = input(f"  → Поставьте робота на {prompt}, "
                        f"затем Enter (s = пропустить): ")
        except EOFError:
            break
        if ans.strip().lower() in ("s", "п"):
            print("    пропущено\n")
            continue
        r, g, b, c, color, n = _average(CALIB_SECS)
        data[key] = (r, g, b, c)
        print("    " + _fmt(r, g, b, c, color, n) + "\n")

    if not {"white", "black"} <= data.keys():
        print("Без белой и чёрной пороги считать не из чего - калибровка прервана.")
        return

    def frac(key, num):
        r, g, b, c = data[key]
        if c <= 0:
            return 0.0
        return {"r": r / c, "rb": (r - b) / c, "br": (b - r) / c}[num]

    print("\n" + "=" * 70)
    print("РЕЗУЛЬТАТ. Наблюдённые величины (по ним и стоят пороги в nav.py):\n")
    print(f"  {'плитка':8s} {'C':>6s}  {'r/c':>6s} {'(b-r)/c':>8s} {'(r-b)/c':>8s}")
    for key, _ in CALIB_STEPS:
        if key not in data:
            continue
        r, g, b, c = data[key]
        print(f"  {key:8s} {c:6d}  {frac(key,'r'):6.3f} "
              f"{frac(key,'br'):+8.3f} {frac(key,'rb'):+8.3f}")

    others = [k for k in data if k != "blue"]
    blue_thr = (_mid(max(frac(k, "br") for k in others), frac("blue", "br"))
                if "blue" in data else None)
    reds = [k for k in data if k != "red"]
    red_thr = (_mid(max(frac(k, "rb") for k in reds), frac("red", "rb"))
               if "red" in data else None)
    red_rfrac = (_mid(max(frac(k, "r") for k in reds), frac("red", "r"))
                 if "red" in data else None)

    # Порог чёрной - по яркости, и между чёрной и САМОЙ ТЁМНОЙ из остальных
    # плиток, а не только белой: синяя и красная заметно тусклее белой.
    c_black = data["black"][3]
    c_dimmest = min(data[k][3] for k in data if k not in ("black", "air"))
    black_thr = _mid(c_black, c_dimmest)

    print("\nПодставить в nav.py:\n")
    if blue_thr is not None:
        print(f"    BLUE_RATIO_THRESH = {blue_thr:.2f}")
    if red_thr is not None:
        print(f"    RED_RATIO_THRESH = {red_thr:.2f}")
        print(f"    RED_MIN_R_FRACTION = {red_rfrac:.2f}")
    print(f"    BLACK_ABS_MAX_C = {black_thr:.0f}"
          f"      # чёрная {c_black}, самая тусклая из прочих {c_dimmest}")
    if c_dimmest <= c_black * 1.5:
        print("    [WARN] чёрная и ближайшая к ней плитка почти не отличаются по\n"
              "           яркости - порог ненадёжен, датчик стоит опустить ниже\n"
              "           к полу или прикрыть от бокового света")

    if "air" in data:
        c_air = data["air"][3]
        c_w = data["white"][3]
        print(f"\n  Воздух: C={c_air} против C={c_w} над белой.")
        if c_air > c_black * 3:
            print("  Поднятый робот НЕ даёт темноты - значит признак «датчик\n"
                  "  погас» для детектора подъёма в этом зале не работает, и\n"
                  "  LIFT_C_FACTOR надо брать по этому числу (см. nav.py).")

    print("\nПодставить в src/apds9960.c, apds9960_detect_color()"
          " - байт цвета nav.py не использует,\nно пусть консоль ESP не врёт:\n")
    print(f"    C < {black_thr:.0f}                 -> BLACK")
    if red_thr is not None:
        print(f"    (r-b)*100 > c*{red_thr*100:.0f}       -> RED")
    if blue_thr is not None:
        print(f"    (b-r)*100 > c*{blue_thr*100:.0f}       -> BLUE")
    print("    иначе                  -> WHITE")
    print("=" * 70)


def main():
    args = sys.argv[1:]
    _connect()

    if "--calib" in args:
        try:
            calibrate()
        except KeyboardInterrupt:
            print("\nПрервано.")
        finally:
            _link.close()
        return

    secs = float(args[0]) if args else None

    if secs is None:
        print("Поставьте робота на плитку. Ctrl+C для выхода.\n"
              "  nav.py   - решение tile_color() на текущих порогах (это то,\n"
              "             по чему робот действительно ездит)\n"
              "  прошивка - байт цвета из apds9960_detect_color(), nav.py его\n"
              "             НЕ использует\n")
        try:
            while True:
                with _lock:
                    r, g, b, c, color = _last
                print(_fmt(r, g, b, c, color), flush=True)
                time.sleep(0.4)
        except KeyboardInterrupt:
            pass
        finally:
            _link.close()
        return

    print(f"Усредняю {secs:.0f}с, не двигайте робота...")
    r, g, b, c, color, n = _average(secs)
    _link.close()
    print(_fmt(r, g, b, c, color, n))


if __name__ == "__main__":
    main()
