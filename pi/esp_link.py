"""Связь с ESP: телеметрия, команды моторам, гироскоп, энкодеры, кнопки."""
from __future__ import annotations

import binascii
import math
import struct
import threading
import time

import serial

import config
import state
import ramps
from motion_estimate import MotionSample, MotionDiscontinuity


# ДОЛЖНО СОВПАДАТЬ с telem_t и TELEM_VER в robot_link.c.
# ver seq yaw yaw_rate pitch roll ax ay az color cr cg cb cc left_cm right_cm
# status flags z_cnt boot_id cmd_err crc16
from protocol import TELEM_VER


def crc16(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) - как crc16_ccitt() в
    robot_link.c. Ей закрыт и пакет телеметрии, и каждая команда к ESP."""
    return binascii.crc_hqx(data, 0xFFFF)


def link_report():
    """Одна строка: что реально идёт по линку и что мы ждём."""
    if not state.magic_gaps:
        return ("маркеров 0xAA55 в потоке нет вовсе - ESP молчит, "
                "не та скорость или не тот порт")
    gap, hits = max(state.magic_gaps.items(), key=lambda kv: kv[1])
    if gap == config.PKT_SIZE and not state.bad_ver:
        return f"пакет {gap} байт, версия {TELEM_VER} - сходится"
    tail = f"; пакетов с чужой версией: {state.bad_ver}" if state.bad_ver else ""
    return (f"ESP шлёт пакеты по {gap} байт ({hits} раз), а этот nav.py ждёт "
            f"{config.PKT_SIZE} и версию {TELEM_VER}{tail} -> "
            f"ПРОШИВКА И .PY РАЗНЫХ ВЕРСИЙ, перезалей обе стороны")


def link_errors():
    """Счётчики ошибок линка с обеих сторон - одной строкой."""
    with state.lock:
        cmd_err = state.telem["esp_cmd_err"]
    return (f"битых CRC {state.bad_crc}, пропущено пакетов {state.seq_lost}, "
            f"ESP отверг команд {cmd_err}")


# ── здоровье линка ───────────────────────────────────────────
# Пакет старше этого - управлять по нему нельзя: yaw, одометрия и флаг
# "энкодеры живы" просто замирают на последнем значении, и робот продолжает
# рулить по мёртвым данным. Раньше telem_ok после старта не сбрасывался
# никогда, то есть обрыв связи с ESP посреди заезда не замечался вообще.
#
# 0.3с - это 60 подряд пропущенных пакетов при TELEM_HZ=200, то есть порог
# заведомо щедрый. Так и задумано: гораздо хуже ложно оборвать заезд из-за
# того, что поток телеметрии на полсекунды не получил GIL (YOLO, лидар), чем
# на треть секунды позже заметить настоящий обрыв. Если этот порог начнёт
# срабатывать - смотреть надо на частоту, которую main() печатает на старте,
# а не поднимать порог дальше.
TELEM_STALE_S = 0.3
# Сколько ждать очередного пакета в шаге управляющей петли. Петля теперь
# ТАКТИРУЕТСЯ ПАКЕТОМ, а не sleep(0.010): раньше два независимых периода по
# 10мс давали биение до целого периода сверху, и это был самый дешёвый
# источник запаздывания во всём контуре (см. STOP_LEAD_S). Таймаут - только
# страховка, чтобы петля не встала намертво при обрыве.
TELEM_WAIT_S = 0.05


# ══════════════════════════════════════════════════════════════
#  Потоки
# ══════════════════════════════════════════════════════════════
def _link_reopen():
    """Ошибка порта на ходу (не закрытие на выходе): переоткрываем, пока не
    получится. False - порт закрыт нарочно, поток должен выйти."""
    if state.link_stop.is_set() or not state.link.is_open:
        return False
    print("\n  [WARN] ошибка порта ESP - переподключаюсь")
    while not state.link_stop.wait(0.5):
        try:
            state.link.close()
            state.link.open()
            return True
        except (serial.SerialException, OSError):
            pass
    return False


def _telem_thread():
    buf = bytearray()
    since = 0            # байт от предыдущего маркера 0xAA55 до текущего
    have_prev = False
    last_seq = None      # seq прошлого пакета - для счёта пропусков
    boot_id = None       # boot_id ESP - его смена значит ребут
    gyro_was = color_was = True   # чтобы сказать о пропаже свежести один раз
    while not state.link_stop.is_set():
        try:
            # 4096, а не 256: при TELEM_HZ=200 в 256 байт влезает всего
            # 6 пакетов, и если поток на мгновение не получил процессор
            # (YOLO, лидар - тот же GIL), разбор начинал отставать пачками.
            chunk = state.link.read(max(1, min(state.link.in_waiting, 4096)))
        except (serial.SerialException, OSError, TypeError):
            if state.link is not None and _link_reopen():
                buf.clear()
                have_prev = False
                continue
            return          # порт закрыт на выходе из main()
        if not chunk:
            continue
        buf.extend(chunk)
        while len(buf) >= config.PKT_SIZE:
            if buf[0] != config.MAGIC[0] or buf[1] != config.MAGIC[1]:
                del buf[0]
                since += 1
                continue
            if have_prev and len(state.magic_gaps) < 64:
                state.magic_gaps[since] = state.magic_gaps.get(since, 0) + 1
            have_prev = True
            since = 0
            body = bytes(buf[2:config.PKT_SIZE])
            # Версия - раньше CRC: она отсекает чужую раскладку сразу и
            # называет причину, а CRC только молча выбросит кадр. Маркер мог
            # быть случайным (внутри мусора) - тогда настоящий пакет лежит
            # внутри этих 51 байт, поэтому при любой ошибке съедаем 1 байт,
            # а не весь кадр. "Чужой версией" считаем кадр, за которым сразу
            # идёт следующий маркер: у случайного такого нет.
            if body[0] != TELEM_VER:
                if (len(buf) >= config.PKT_SIZE + 2 and buf[config.PKT_SIZE] == config.MAGIC[0]
                        and buf[config.PKT_SIZE + 1] == config.MAGIC[1]):
                    state.bad_ver += 1
                del buf[0]
                since = 1
                continue
            if crc16(body[:-2]) != body[-2] | (body[-1] << 8):
                state.bad_crc += 1
                del buf[0]
                since = 1
                continue
            since = config.PKT_SIZE          # столько сейчас будет съедено
            del buf[:config.PKT_SIZE]
            (_ver, seq, yaw, yaw_rate, pitch, roll, ax, ay, az, color,
             cr, cg, cb, cc, lcm, rcm, status, flags,
             z_cnt, esp_boot, cmd_err, *health_crc) = struct.unpack(config.FMT, body)
            if not all(math.isfinite(v) for v in (yaw, yaw_rate, pitch, roll, ax, ay, az, lcm, rcm)):
                # CRC proves transport integrity, not valid sensor arithmetic.
                # Do not refresh the age of an unusable control observation.
                continue
            if boot_id is not None and esp_boot != boot_id:
                print(f"\n  [WARN] ESP ПЕРЕЗАГРУЗИЛСЯ (boot_id {boot_id:04X} -> "
                      f"{esp_boot:04X}): одометрия и пауза на ESP сброшены")
                last_seq = None       # seq тоже начат заново - не пропуск
                state.lifted.set()    # yaw/odometry origin is no longer valid
            boot_id = esp_boot
            if last_seq is not None:
                state.seq_lost += (seq - last_seq - 1) & 0xFF
            last_seq = seq
            gyro_fresh, color_fresh = bool(flags & 32), bool(flags & 64)
            if gyro_fresh != gyro_was:
                print("\n  [WARN] гироскоп ESP замолчал - yaw замер"
                      if not gyro_fresh else "\n  гироскоп ESP снова идёт")
                gyro_was = gyro_fresh
            if color_fresh != color_was:
                print("\n  [WARN] датчик цвета ESP не читается - цвет старый"
                      if not color_fresh else "\n  датчик цвета ESP снова читается")
                color_was = color_fresh
            with state.lock:
                state.telem["motor_voltage"] = [v / 10 if v else None for v in health_crc[:4]]
                state.telem["motor_temp"] = [v if v != -128 else None for v in health_crc[4:8]]
                state.telem["motor_error"] = health_crc[8:12]
                state.telem["yaw"] = float(yaw)
                state.telem["yaw_rate"] = float(yaw_rate)
                # У BNO085 плюс - нос вниз (заезд 09.11: лидар и порядок горок),
                # в коде везде плюс - нос вверх.
                state.telem["pitch"] = -float(pitch)
                state.telem["roll"] = float(roll)
                # Модуль считаем здесь, чтобы у всех читателей он был один и
                # тот же; вертикаль хранится отдельно - именно она отличает
                # подъём от тряски (см. детектор подъёма).
                state.telem["accel"] = math.sqrt(ax * ax + ay * ay + az * az)
                state.telem["az"] = float(az)
                state.telem["rgbc"] = (cr, cg, cb, cc)
                state.telem["lcm"] = float(lcm)
                state.telem["rcm"] = float(rcm)
                state.telem["color"] = int(color)
                state.telem["clicker1"] = bool(flags & 1)
                state.telem["clicker2"] = bool(flags & 2)
                state.telem["enc_left_online"] = bool(flags & 4)
                state.telem["enc_right_online"] = bool(flags & 8)
                state.telem["btn1"] = bool(flags & 16)
                state.telem["esp_status"] = int(status)
                state.telem["z_cnt"] = int(z_cnt)
                state.telem["gyro_fresh"] = gyro_fresh
                state.telem["color_fresh"] = color_fresh
                state.telem["esp_cmd_err"] = int(cmd_err)
                state.telem["telem_ok"] = True
                state.telem_t = time.monotonic()
                state.telem_seq += 1
                state.motion_history.append(MotionSample(state.telem_seq, state.telem_t,
                    float(lcm), float(rcm), float(yaw), int(z_cnt), gyro_fresh and bool(flags & 4) and bool(flags & 8)))
            state.telem_evt.set()
            if status == 0:
                state.esp_running.set()
            else:
                state.esp_running.clear()


# ══════════════════════════════════════════════════════════════
#  Читалки / команды
# ══════════════════════════════════════════════════════════════
def get_yaw():
    with state.lock: return state.telem["yaw"]


def get_yaw_rate():
    with state.lock: return state.telem["yaw_rate"]


def get_yaw_and_rate():
    """Курс и угловая скорость из одного пакета (одним захватом _lock)."""
    with state.lock: return state.telem["yaw"], state.telem["yaw_rate"]


def get_pitch():
    with state.lock: return state.telem["pitch"]


def get_roll():
    with state.lock: return state.telem["roll"]


def get_accel():
    """|линейное ускорение| без g, м/с². 0.0, если BNO085 не отдал репорт."""
    with state.lock: return state.telem["accel"]


def get_az():
    """Вертикальная составляющая линейного ускорения, м/с² (ось рыскания).

    Знак: положительное - вверх, пока робот примерно горизонтален. На
    наклоне ось уезжает вместе с корпусом, но при наклонах поля (до 25°)
    косинус ошибки больше 0.9, так что для интегрирования это несущественно.
    """
    with state.lock: return state.telem["az"]


def telem_age():
    """Сколько секунд прошло с последнего ПРИНЯТОГО пакета."""
    return time.monotonic() - state.telem_t if state.telem_seq else float('inf')


def telem_fresh():
    return telem_age() < TELEM_STALE_S


def gyro_fresh():
    """A live UART packet does not imply a live BNO085 report."""
    with state.lock:
        return telem_fresh() and state.telem.get("gyro_fresh", False)


def wait_telem(last_seq, timeout=TELEM_WAIT_S):
    """Ждёт пакет новее last_seq, возвращает новый номер.

    Это такт управляющей петли вместо sleep(0.010). Гонки нет: событие
    только будит, а решает всегда сравнение номеров - потерянный фронт
    ловится следующей же проверкой условия.
    """
    end = time.monotonic() + timeout
    while state.telem_seq == last_seq:
        left = end - time.monotonic()
        if left <= 0:
            break
        state.telem_evt.wait(left)
        state.telem_evt.clear()
    return state.telem_seq


def get_odo():
    with state.lock: return state.telem["lcm"], state.telem["rcm"]


def motion_snapshot():
    with state.lock:
        if not state.motion_history:
            raise MotionDiscontinuity("no motion observations")
        return state.motion_history[-1]


def motion_since(seq):
    """Copy only unconsumed samples, normally one or two per controller tick."""
    with state.lock:
        result = []
        for sample in reversed(state.motion_history):
            if sample.seq <= seq:
                break
            result.append(sample)
    result.reverse()
    if result and result[0].seq != seq + 1:
        raise MotionDiscontinuity("motion history overflowed during a pause")
    return result


def get_z_cnt():
    with state.lock: return state.telem["z_cnt"]


def wait_odo_zeroed(z0, timeout=0.2):
    """Ждёт пакет, где ESP уже применил "Z" (z_cnt сменился с z0). Одометрия в
    _state с этого пакета и дальше - после обнуления: ESP растит z_cnt строго
    после него, а _state обновляется целым пакетом под _lock. False - не
    дождались за timeout."""
    end = time.monotonic() + timeout
    seq = state.telem_seq
    while get_z_cnt() == z0:
        left = end - time.monotonic()
        if left <= 0:
            return False
        seq = wait_telem(seq, left)
    return True


def encoders_online():
    with state.lock:
        return state.telem["enc_left_online"] and state.telem["enc_right_online"]


def encoder_sides():
    """(левый_жив, правый_жив) - для внятного сообщения об отказе."""
    with state.lock:
        return state.telem["enc_left_online"], state.telem["enc_right_online"]


def get_color():
    with state.lock: return state.telem["color"]


def get_rgbc():
    with state.lock: return state.telem["rgbc"]


def get_esp_paused():
    """BTN_1 (GPIO47) на ESP нажат - это перезапуск с чекпоинта, см.
    restart_from_checkpoint(). ESP в этом состоянии сам игнорирует "M:"/"T:"."""
    return not state.esp_running.is_set()


def resume_esp(timeout=1.0):
    """Снять паузу BTN_1 на ESP командой "R" и дождаться status=running.

    Прошивка, где BTN_1 ещё переключал пауза/ход, "R" не знает - тогда
    просим нажать BTN_1 ещё раз и ждём его."""
    t0 = time.time()
    while get_esp_paused():
        if time.time() - t0 > timeout:
            print("  [WARN] ESP не снял паузу по 'R' (старая прошивка?) - "
                  "нажмите BTN_1 на ESP")
            state.esp_running.wait()
            break
        _cmd("R")
        state.esp_running.wait(0.1)


def interrupted():
    """Причина немедленно бросить текущий манёвр: пауза кнопкой на ESP,
    подъём робота с поля или пропавшая телеметрия."""
    return get_esp_paused() or state.lifted.is_set() or not gyro_fresh()


def _cmd(s):
    # Без flush(): это tcdrain, и на Pi он стоил 8.0мс на КАЖДУЮ команду
    # (замер: p50 7.97, max 8.02) против 0.03мс у самого write() - больше
    # такта петли (5мс). Ядро отправляет байты и так, 16 байт на 921600 -
    # это ~0.2мс. Дожимается только при выходе, перед закрытием порта.
    b = s.encode()
    with state.write_lock:
        state.link.write(b + b"*%04X\n" % crc16(b))    # CRC - см. robot_link.h


# Каждая команда хода превращается на ESP в ЧЕТЫРЕ кадра CAN. Управляющая
# петля теперь тактируется телеметрией (200 Гц), и без ограничителя поток
# кадров к моторам удвоился бы просто потому, что петля стала чаще считать -
# а шина CAN как раз то место, где это уже один раз вышло боком (см.
# ENC_POLL_HZ в motor.c). Свежие данные регулятору нужны, а подавать мотору
# новую уставку чаще, чем он её отрабатывает, - нет: закрытый контур скорости
# на LK живёт своим темпом. 100 Гц - ровно столько, сколько уходило на шину
# до перехода на такт по пакету.
MOTOR_CMD_HZ = 100


def _drive_cmd(s):
    """Команда хода (M:/T:) с ограничением частоты - см. MOTOR_CMD_HZ.

    Пропуск безвреден: следующий тик пошлёт более свежее значение той же
    уставки. Через этот ограничитель НЕ идут ни тормоз, ни остановка, ни
    обнуление одометрии - они обязаны уходить немедленно и всегда.
    """
    now = time.monotonic()
    if now - state.last_drive_cmd_t < 1.0 / MOTOR_CMD_HZ:
        return
    state.last_drive_cmd_t = now
    _cmd(s)


def _drive_cmd_now(s):
    """То же, но мимо ограничителя, и с перевзводом его таймера: после
    тормоза/остановки следующая команда хода не должна ждать окна."""
    state.last_drive_cmd_t = 0.0
    _cmd(s)


def motors(l, r): _drive_cmd(f"M:{l:.1f},{r:.1f}")


def torque(l, r, cap_cms=config.OBSTACLE_CAP_CMS):
    """Момент с потолком скорости колеса - см. OBSTACLE_CAP_CMS."""
    _drive_cmd(f"T:{l:.2f},{r:.2f},{cap_cms:.1f}")


def stop():       _drive_cmd_now("M:0.0,0.0")


def zero_odo():   _drive_cmd_now("Z")


def brake():      _drive_cmd_now("B")


def motors_on():  _drive_cmd_now("W")    # включить моторы перед заездом (0x88)


def motors_off(): _drive_cmd_now("X")    # выключить после заезда (0x80): не пищат, не греются


def settle_brake(pause_s=0.10):
    """Тормоз после манёвра: "B", и ESP держит 0 dps ещё BRAKE_HOLD_US - и на
    ровном полу, и на препятствии. Раньше на перекосе моторы отпускали (от
    тряски на люфте редукторов), но отпущенный робот скатывался с
    препятствия: заезд 24.10, после поворотов на нём уехал назад на 57мм."""
    brake()
    time.sleep(pause_s)


def hold_while(fn, *args, period=1.0, **kwargs):
    """Держит активный тормоз фоновым потоком, пока выполняется fn(*args).

    Протокол сам отпускает 'B' через BRAKE_HOLD_US=3с на ESP (см.
    robot_link.h) - это рассчитано на короткие паузы между сериями команд,
    а не на растянутые действия вроде 5+с мигания лентой при находке жертвы.
    Без переподтверждения робота на последних секундах такого мигания ничто
    не держит - на наклонном полу или от толчка его может увести с центра
    клетки, а именно там начинается 5-секундный отсчёт по правилам.
    """
    stop_evt = threading.Event()

    def _keepalive():
        while not stop_evt.wait(period):
            try:
                brake()
            except (serial.SerialException, OSError) as e:
                print(f"  [WARN] keepalive тормоза: {e}")

    brake()
    t = threading.Thread(target=_keepalive, daemon=True)
    t.start()
    try:
        return fn(*args, **kwargs)
    finally:
        stop_evt.set()
        t.join(timeout=0.5)
