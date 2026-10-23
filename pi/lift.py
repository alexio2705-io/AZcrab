#!/usr/bin/env python3
"""Признаки «робота сняли с поля» - замер и подбор порогов для LIFT_* в nav.py.

Показывает то, по чему детектор в nav.py принимает решение, и главное - то,
по чему его ещё предстоит научить: ВЕРТИКАЛЬНУЮ составляющую ускорения.

Зачем вертикаль. Сначала детектор смотрел только на модуль |a|: пик как повод
заподозрить, устойчивая дрожь как подтверждение. На неподвижном роботе это
работало (на полу 0.03..0.06, в руках не ниже 0.25), но базовая линия была
снята с НЕПОДВИЖНОГО робота, и этого мало:

  - робота катят по полу - трясёт на неровностях не слабее руки;
  - робот лезет на лесенку или горку - трясёт ещё сильнее, и не секунду.

По модулю эти случаи от подъёма не отличаются. Отличает их вертикаль: подъём -
это УСТОЙЧИВЫЙ НАБОР ВЫСОТЫ, а тряска - колебания с нулевым итогом. Поэтому
здесь считается скользящий двойной интеграл az за RISE_WINDOW_S: Δv и Δh.
Прокат колёс по одометрии - второй разделитель, для случая «просто катят».

Запуск на малине:
    ~/maze/venv/bin/python ~/maze/lift.py

Снимите ДВЕ записи и сравните развёртки:
  1) стоит 3с -> поднять -> подержать 5с -> поставить -> стоять 3с;
  2) стоит 3с -> прокатить рукой по полу и через препятствие -> стоять 3с.
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
# ДОЛЖНО СОВПАДАТЬ с telem_t и TELEM_VER в robot_link.c.
# ver seq yaw yaw_rate pitch roll ax ay az color cr cg cb cc left_cm right_cm
# status flags z_cnt boot_id cmd_err crc16
from protocol import TELEM_VER, FMT, BODY_SIZE, PKT_SIZE

# ── ЗЕРКАЛО порогов из nav.py ────────────────────────────────
# Держать в согласии с nav.py вручную.
LIFT_AZ_MS2 = 1.2
LIFT_AZ_WINDOW_S = 1.0
LIFT_HELD_MS2 = 0.30
LIFT_HELD_WINDOW_S = 0.6
LIFT_CONFIRM_S = 1.5
LIFT_ARM_TIMEOUT_S = 5.0
LIFT_ROLL_DEG = 35.0
LIFT_MAX_ROLLED_CM = 2.0
# В nav.py есть ещё одно условие - не судить, пока сами крутим моторы
# (LIFT_IDLE_S). Здесь его нет: nav.py не запущен, команд нет по определению.

# Окно скользящего двойного интеграла az. 2с - столько занимает подъём со
# стола вместе с рывком. Дрейф нуля акселерометра за такое окно даёт единицы
# сантиметров, а подъём - десятки.
RISE_WINDOW_S = 2.0

FLOOR_CALIB_S = 2.0
BUCKET_S = 0.25

_lock = threading.Lock()
_last = None             # (roll, pitch, |a|, az, C, left_cm, right_cm)
_link = None
_bad_ver = 0
_magic_gaps = {}


def _reader():
    global _last, _bad_ver
    buf = bytearray()
    since = 0
    have_prev = False
    while True:
        try:
            chunk = _link.read(max(1, min(_link.in_waiting, 4096)))
        except (serial.SerialException, OSError, TypeError):
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
            since = PKT_SIZE
            pkt = bytes(buf[:PKT_SIZE])
            del buf[:PKT_SIZE]
            body = pkt[2:]
            if body[0] != TELEM_VER:
                _bad_ver += 1
                continue
            # CRC-16/CCITT-FALSE, как в robot_link.c
            if binascii.crc_hqx(body[:-2], 0xFFFF) != body[-2] | (body[-1] << 8):
                continue
            v = struct.unpack(FMT, body)
            # ver, seq, yaw, yaw_rate, pitch(4), roll(5), ax..az(6-8), color,
            # cr..cc(10-13), lcm(14), rcm(15)
            ax, ay, az = v[6], v[7], v[8]
            mag = (ax * ax + ay * ay + az * az) ** 0.5
            with _lock:
                _last = (v[5], v[4], mag, az, v[13], v[14], v[15])


def link_report():
    if not _magic_gaps:
        return "маркеров 0xAA55 нет - ESP молчит или не тот порт"
    gap, hits = max(_magic_gaps.items(), key=lambda kv: kv[1])
    if gap == PKT_SIZE and not _bad_ver:
        return f"пакет {gap} байт, версия {TELEM_VER} - сходится"
    return (f"ESP шлёт по {gap} байт ({hits} раз), а lift.py ждёт {PKT_SIZE} "
            f"и версию {TELEM_VER} -> ПРОШИВКА И .PY РАЗНЫХ ВЕРСИЙ")


class Detector:
    """Тот же автомат, что _lift_thread() в nav.py, плюс замер вертикали."""

    def __init__(self):
        self.hist = []          # (t, |a|, az, l, r)
        self.armed_at = None
        self.held_since = None
        self.held_odo = None
        self.fired_at = None

    def step(self, t, roll, a, az, l, r):
        self.hist.append((t, a, az, l, r))
        keep = max(LIFT_AZ_WINDOW_S, LIFT_HELD_WINDOW_S, RISE_WINDOW_S)
        while self.hist and t - self.hist[0][0] > keep:
            self.hist.pop(0)

        vert = [x[2] for x in self.hist if t - x[0] <= LIFT_AZ_WINDOW_S]
        az_mean = sum(vert) / len(vert) if vert else 0.0
        rec = [x[1] for x in self.hist if t - x[0] <= LIFT_HELD_WINDOW_S]
        mean = sum(rec) / len(rec) if rec else 0.0

        # Скользящий двойной интеграл вертикали: от начала окна, v0=h0=0.
        rise = [x for x in self.hist if t - x[0] <= RISE_WINDOW_S]
        vz = hz = 0.0
        for i in range(1, len(rise)):
            dt = rise[i][0] - rise[i - 1][0]
            vz += rise[i][2] * dt
            hz += vz * dt
        rolled = (max(abs(rise[-1][3] - rise[0][3]),
                      abs(rise[-1][4] - rise[0][4])) if len(rise) > 1 else 0.0)

        return (self._fsm(t, roll, az_mean, mean, l, r),
                az_mean, mean, vz, hz * 1000.0, rolled)

    def _fsm(self, t, roll, az_mean, mean, l, r):
        if self.armed_at is None:
            if abs(az_mean) > LIFT_AZ_MS2 or abs(roll) > LIFT_ROLL_DEG:
                self.armed_at, self.held_since = t, None
                return "подозрение"
            return "покой"
        if t - self.armed_at > LIFT_ARM_TIMEOUT_S:
            self.armed_at = self.held_since = self.held_odo = None
            return "снято"
        if mean <= LIFT_HELD_MS2:
            self.held_since = self.held_odo = None
            return "подозрение"
        if self.held_since is None:
            self.held_since, self.held_odo = t, (l, r)
            return "дрожь 0.0с"
        if max(abs(l - self.held_odo[0]),
               abs(r - self.held_odo[1])) > LIFT_MAX_ROLLED_CM:
            self.held_since, self.held_odo = t, (l, r)   # катится, не висит
            return "катится"
        if t - self.held_since >= LIFT_CONFIRM_S:
            if self.fired_at is None:
                self.fired_at = t
            return "ПОДЪЁМ"
        return f"дрожь {t - self.held_since:.1f}с"


def _send(cmd):
    b = cmd.encode()
    _link.write(b + b"*%04X\n" % binascii.crc_hqx(b, 0xFFFF))   # CRC - robot_link.h
    _link.flush()


def _record(secs):
    """|a| и вертикаль за secs -> (|a|ср, |a|макс, макс |среднее az| за окно).

    Третье число - главное: им взводится подозрение, и если самотряска
    (гул моторов, серво) его не поднимает, порог LIFT_AZ_MS2 можно ронять
    сколько угодно, не рискуя ложным срабатыванием.
    """
    vals = []
    azs = []
    end = time.time() + secs
    while time.time() < end:
        with _lock:
            vals.append(_last[2])
            azs.append((time.time(), _last[3]))
        time.sleep(0.02)
    if not vals:
        return 0.0, 0.0, 0.0
    worst = 0.0
    for i, (t, _) in enumerate(azs):
        win = [z for tt, z in azs[max(0, i - 50):i + 1] if t - tt <= LIFT_AZ_WINDOW_S]
        if win:
            worst = max(worst, abs(sum(win) / len(win)))
    return sum(vals) / len(vals), max(vals), worst


def selftest():
    """Сколько робот трясёт САМ СЕБЯ - тормозом и серво дозатора.

    Это те два источника, на которых детектор подъёма дал ложное
    срабатывание: робот стоял, мигал лентой и выдавал комплект, а дрожь
    была неотличима от рук. Если самотряска заметно ниже, чем руки
    (замерено 0.25..1.77), порог LIFT_HELD_MS2 можно просто поднять и
    никаких исключений не понадобится.

    ВНИМАНИЕ: 'дозатор' реально высыпает комплект, а 'тормоз' требует
    включённой силовой - без неё моторы промолчат и замер будет пустым.
    """
    print("Замер собственной тряски робота. Робот должен СТОЯТЬ НА ПОЛУ.\n")
    rows = []

    input("  → Enter: тишина (ничего не шлём), 3с ")
    rows.append(("тишина", _record(3.0)))
    print(f"    ср={rows[-1][1][0]:.3f}  макс={rows[-1][1][1]:.2f}  "
          f"выше порога {rows[-1][1][2]*100:.0f}% времени\n")

    input("  → Enter: активный тормоз (B каждые 0.5с), 4с ")
    end = time.time() + 4.0
    vals = []
    last_b = 0.0
    while time.time() < end:
        if time.time() - last_b > 0.5:
            _send("B")
            last_b = time.time()
        with _lock:
            vals.append(_last[2])
        time.sleep(0.02)
    over = sum(1 for v in vals if v > LIFT_HELD_MS2) / max(len(vals), 1)
    rows.append(("тормоз", (sum(vals) / max(len(vals), 1), max(vals), over)))
    _send("M:0.0,0.0")
    print(f"    ср={rows[-1][1][0]:.3f}  макс={rows[-1][1][1]:.2f}  "
          f"выше порога {rows[-1][1][2]*100:.0f}% времени\n")

    ans = input("  → Enter: серво дозатора (ВЫСЫПЕТ КОМПЛЕКТ), 3с. s = пропустить ")
    if ans.strip().lower() not in ("s", "п"):
        _send("K:L")
        rows.append(("дозатор", _record(3.0)))
        print(f"    ср={rows[-1][1][0]:.3f}  макс={rows[-1][1][1]:.2f}  "
              f"выше порога {rows[-1][1][2]*100:.0f}% времени\n")

    print("=" * 66)
    print(f"  {'источник':10s} {'|a| ср':>8s} {'|a| макс':>9s} {'макс az|1с|':>12s}")
    for name, (m, mx, az) in rows:
        print(f"  {name:10s} {m:8.3f} {mx:9.2f} {az:12.2f}")
    print(f"\n  для сравнения, робот В РУКАХ: |a| ср 0.25..1.77")
    worst = max((az for _, (_, _, az) in rows[1:]), default=0.0)
    print(f"\n  Подозрение взводится при |az за {LIFT_AZ_WINDOW_S:.0f}с| > "
          f"{LIFT_AZ_MS2:.1f}; у подъёма на руки замерено 2.72.")
    if worst:
        print(f"  Самотряска даёт по вертикали {worst:.2f} - "
              + ("запас есть, порог можно ронять дальше."
                 if worst < LIFT_AZ_MS2 * 0.5 else
                 "ВПЛОТНУЮ к порогу, ронять нельзя."))
    print("=" * 66)


def main():
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
            print("[FATAL] Нет телеметрии с ESP.\n        " + link_report())
            sys.exit(1)
        time.sleep(0.05)
    time.sleep(0.3)
    if _bad_ver or (_magic_gaps and
                    max(_magic_gaps.items(), key=lambda kv: kv[1])[0] != PKT_SIZE):
        print("[FATAL] " + link_report())
        sys.exit(1)

    if "--selftest" in sys.argv[1:]:
        try:
            selftest()
        except KeyboardInterrupt:
            print("\nПрервано.")
        finally:
            _link.close()
        return

    print(f"Опора: робот стоит на плитке, замер {FLOOR_CALIB_S:.0f}с...")
    acc_a = acc_az = acc_c = 0.0
    n = 0
    end = time.time() + FLOOR_CALIB_S
    while time.time() < end:
        with _lock:
            acc_a += _last[2]
            acc_az += _last[3]
            acc_c += _last[4]
        n += 1
        time.sleep(0.02)
    n = max(n, 1)
    print(f"  на полу: |a| = {acc_a/n:.3f}   az = {acc_az/n:+.3f} м/с²   "
          f"C = {acc_c/n:.0f}")
    print(f"  (порог 'держат в руках' = {LIFT_HELD_MS2:.2f}; "
          f"az на полу должен быть около нуля)\n")

    print(f"Детектор: |среднее az| за {LIFT_AZ_WINDOW_S:.0f}с > {LIFT_AZ_MS2:.1f} - подозрение; "
          f"среднее |a|>{LIFT_HELD_MS2:.2f} в течение {LIFT_CONFIRM_S:.1f}с "
          f"при прокате <{LIFT_MAX_ROLLED_CM:.0f}см - подъём")
    print("Ctrl+C - развёртка и итог.\n")

    det = Detector()
    tty = sys.stdout.isatty()
    last_dump = 0.0
    trace = []          # (t, roll, |a|, az, dh_мм, прокат_см, C, состояние)
    max_roll = max_acc = max_azmean = 0.0
    c_min = c_max = None
    fired = 0
    try:
        while True:
            with _lock:
                roll, pitch, a, az, c, l, r = _last
            now = time.monotonic()
            state, az_mean, mean, vz, dh, rolled = det.step(now, roll, a, az, l, r)
            trace.append((now, roll, a, az, az_mean, rolled, c, state))

            max_roll = max(max_roll, abs(roll))
            max_acc = max(max_acc, a)
            max_azmean = max(max_azmean, abs(az_mean))
            c_min = c if c_min is None else min(c_min, c)
            c_max = c if c_max is None else max(c_max, c)
            if state == "ПОДЪЁМ":
                fired += 1

            line = (f"крен={roll:+5.1f}°  |a|={a:5.2f} ср={mean:5.2f}  "
                    f"az={az:+5.2f} срза{LIFT_AZ_WINDOW_S:.0f}с={az_mean:+5.2f}  "
                    f"прокат={rolled:5.1f}см  "
                    f"C={c:5d}  {state}")
            if tty:
                sys.stdout.write("\r" + line + "          ")
                sys.stdout.flush()
            elif state != "покой" or now - last_dump > 1.0:
                print(line, flush=True)
                last_dump = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        _link.close()

    if trace:
        t_start = trace[0][0]
        buckets = {}
        for row in trace:
            buckets.setdefault(int((row[0] - t_start) / BUCKET_S), []).append(row)
        print("\n\n" + "=" * 78)
        print("Развёртка. az|1с| - скользящее среднее вертикального "
              f"ускорения за {LIFT_AZ_WINDOW_S:.0f}с,\n"
              f"по нему и взводится подозрение (порог {LIFT_AZ_MS2:.1f}).\n")
        print("   t,с   крен°  |a|макс |a|ср   az ср  az|1с|  прокат,см   C   состояние")
        for k in sorted(buckets):
            rows = buckets[k]
            rr = max(abs(x[1]) for x in rows)
            am = max(x[2] for x in rows)
            aa = sum(x[2] for x in rows) / len(rows)
            zz = sum(x[3] for x in rows) / len(rows)
            azm = max((x[4] for x in rows), key=abs)
            ro = max(x[5] for x in rows)
            cc = sum(x[6] for x in rows) / len(rows)
            st = rows[-1][7]
            st = "" if st == "покой" else st
            print(f"  {k*BUCKET_S:5.2f}  {rr:5.1f}  {am:6.2f} {aa:6.2f}  "
                  f"{zz:+6.2f}  {azm:+6.2f}  {ro:7.1f}  {cc:5.0f}  {st}")

    print("\n" + "=" * 78)
    print(f"Максимумы: |крен| = {max_roll:.1f}°   |a| = {max_acc:.2f} м/с²")
    print(f"ГЛАВНОЕ ЧИСЛО: максимум |среднее az за {LIFT_AZ_WINDOW_S:.0f}с| = "
          f"{max_azmean:.2f}   (порог взвода {LIFT_AZ_MS2:.1f})")
    if max_azmean < LIFT_AZ_MS2:
        print("  -> подозрение не взводилось: подъём слишком плавный для этого\n"
              "     признака. Порог опускать не глядя нельзя - ниже ~0.5 в него\n"
              "     начнёт попадать обычная езда. Нужен другой признак.")
    print(f"Датчик цвета: C = {c_min}..{c_max}   "
          f"(на полу он держится почти постоянным - если при подъёме ушёл\n"
          f"  заметно, по нему можно ловить и медленный подъём)")
    if det.fired_at is not None:
        print(f"Подъём подтверждён, замеров в состоянии ПОДЪЁМ: {fired}.")
    else:
        print("Подъём НЕ подтверждён ни разу.")
    print("=" * 78)


if __name__ == "__main__":
    main()
