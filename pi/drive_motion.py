"""Шаги поездки: скорость, центровка, руль, команда моторам, статус."""
from __future__ import annotations

import math
import sys
import time

import config
import state
import esp_link
import turning


SIDE_CENTER_MM = 140.0

# Разгон - ПО ВРЕМЕНИ, не по пройденному пути. Профиль "по пути" (было
# ACCEL_MM=60) считает скорость от traveled_cm, а сам traveled_cm растёт тем
# быстрее, чем выше уже скорость - положительная обратная связь даёт РАЗГОН
# ПО ЭКСПОНЕНТЕ: требуемый dv/dt на подходе к MOVE_MAX_CMS улетает далеко за
# любое разумное ограничение мотора, мотор не успевает, расхождение копится
# и потом отыгрывается рывком. Разгон по времени (smoothstep) даёт постоянный,
# предсказуемый и заранее известный пик dv/dt - можно держать его безопасно
# ниже MOTOR_ACCEL_DPS_S на ESP.
RAMP_UP_S = 0.25
OBSTACLE_STEER_A = 0.35
TILT_CAP_CMS = 35.0            # ход, пока нос задран на препятствии/горке
DESCENT_CAP_CMS = 28.0         # ход, пока нос опущен (съезд с препятствия)
RAMP_TORQUE_A = 1.9
RAMP_STEER_A = 0.55
RAMP_WALL_KP_DEG = 0.08
RAMP_WALL_MAX_DEG = 5.0
MAX_STEER_CMS = 28.0   # рулёжка в см/с, растёт вместе с MOVE_MAX_CMS

# Ограничение скорости изменения руля - страховка от скачков команды.
# Было 800, и это оказалось ошибкой: мотор и так способен менять руль лишь на
# ~410 см/с за секунду (MOTOR_ACCEL_DPS_S), то есть программный лимит дублировал
# физический и при этом вмешивался в КАЖДУЮ штатную коррекцию, задерживая её на
# 10-25мс при постоянной времени контура ~43мс. Ограничение скорости внутри
# контура обратной связи - классическая причина медленного предельного цикла,
# то есть той самой езды волнами. Поднято заведомо выше возможностей привода:
# теперь срезает только патологические скачки, как и задумывалось.
STEER_SLEW_CMS_S = 3000.0

# ── Центровка по стенам (медленный, внешний контур) ──────────
# Лидар даёт всего ~10 Гц, и подмешивать его прямо в руль нельзя - запаздывание
# раскачивает робота. Поэтому центровка не рулит, а лишь сдвигает ЦЕЛЬ курса,
# а отрабатывает её быстрый гиро-контур.
#
# Ослаблено сознательно: ехать чуть не по центру клетки не страшно, а вот
# дёрганье мешает камерам. Так же расставлены приоритеты в микромаусе - там
# гироскоп ведёт курс, а боковые стены только медленно подтягивают, и на
# быстрых прогонах их вклад намеренно небольшой.
# Было 0.05/3.5: на 55см/с это постоянная времени центровки ~2с, то есть
# ~1.15м пути - почти 4 клетки, за клетку смещение почти не убиралось.
# 0.12 даёт ~0.87с (~48см). Задержка контура ~0.3с (скан лидара 0.1с +
# HEAD_OFF_SMOOTH 0.15с + гироконтур), запас по фазе около 70° - без
# раскачки. Начнёт вилять - снижать WALL_KP_DEG, а не HEAD_OFF_SMOOTH.
WALL_KP_DEG = 0.12      # градусов поправки курса на мм смещения
WALL_MAX_DEG = 6.0      # предел поправки - больше этого робот едет боком
# Было 12мм, и это неудачно совпадало с рабочей точкой: цель SIDE_CENTER_MM=140
# при замеренном центре клетки в 128мм даёт постоянную ошибку ровно ~12мм, то
# есть робот всё время сидел НА границе мёртвой зоны и коррекция щёлкала
# включено/выключено. 5мм убирает это совпадение, оставаясь выше шума лидара.
WALL_DEADBAND_MM = 5.0
WALL_CENTER_MM = 240    # дальше этого - стены нет, центроваться не по чему

# head_off берётся из wall_e, а тот меняется только раз в скан лидара
# (~100мс) - без сглаживания цель курса прыгает скачком каждые 100мс, и
# руль реагирует на этот шаг мгновенно, это и есть периодические рывки
# посреди прямой. Растянуто до ~150мс: центровке спешить некуда, а плавность
# для камер важнее.
HEAD_OFF_SMOOTH = 0.07   # доля нового значения на каждый тик (10мс)


STATUS_PERIOD_S = 0.1     # строка статуса в drive_cells не чаще


def speed_profile(trip):
    """Скорость: плавный разгон, потолок на наклоне, торможение к цели."""
    # ── Профиль скорости ──────────────────────────────────
    # Разгон - по времени (RAMP_UP_S). Торможение включается только когда
    # решено остановиться, и начинается ровно с текущей скорости.
    trip.ss = min(1.0, (time.monotonic() - trip.t0) / RAMP_UP_S)
    trip.ss = trip.ss * trip.ss * (3.0 - 2.0 * trip.ss)         # smoothstep, без ступеньки
    ramp_up = trip.v_max * trip.ss                     # от 0, не от MOVE_MIN_CMS -
                                              # иначе на t=0 снова ступенька

    # Наклон: нос задран (заезд) или опущен (съезд) - ход ограничен, в том
    # числе и после импульса момента, иначе на гребне режим скорости сам
    # разгонял бы робота обратно до крейсерской прямо на спуск.
    if trip.pitch_delta <= -config.OBSTACLE_PITCH_DEG:
        tilt_target = DESCENT_CAP_CMS
    elif trip.pitch_delta >= config.OBSTACLE_PITCH_DEG or trip.assist_mode is not None:
        tilt_target = TILT_CAP_CMS
    else:
        tilt_target = trip.v_max
    tilt_step = config.BRAKE_DECEL_CMS2 * trip.loop_dt
    trip.tilt_lim = max(trip.tilt_lim - tilt_step, min(trip.tilt_lim + tilt_step, tilt_target))
    ramp_up = min(ramp_up, trip.tilt_lim)

    trip.braking = False
    if (trip.stopping or trip.mark_first) and trip.ramp_state != 'cand':
        # Постоянное замедление: пока запас пути велик, sqrt даёт больше
        # крейсерской и ничего не ограничивает - робот едет полным ходом.
        # Тормозить начинает сам, ровно когда пути остаётся в обрез.
        decel = config.MARK_BRAKE_DECEL_CMS2 if trip.mark_first else config.BRAKE_DECEL_CMS2
        v_brake = math.sqrt(2.0 * decel * max(0.0, trip.rem_eff))
        trip.braking = v_brake < ramp_up
        trip.fwd = min(ramp_up, v_brake)
        if trip.braking:
            trip.fwd = max(trip.fwd, config.MOVE_MIN_CMS * 0.5)   # чтобы не заглохнуть у самой цели
    else:
        trip.fwd = ramp_up


def wall_offset(trip):
    """Смещение от центра коридора по боковым стенам."""
    # ── Центровка: сдвигает цель курса, а не руль ─────────
    wl, wr = trip.walls["left"], trip.walls["right"]
    wall_l = wl is not None and wl < WALL_CENTER_MM
    wall_r = wr is not None and wr < WALL_CENTER_MM

    if wall_l and wall_r:
        trip.wall_e = (wr - wl) * 0.5        # >0: прижались к левой стене
    elif wall_l:
        trip.wall_e = SIDE_CENTER_MM - wl
    elif wall_r:
        trip.wall_e = wr - SIDE_CENTER_MM
    else:
        trip.wall_e = 0.0
    if abs(trip.wall_e) < WALL_DEADBAND_MM:
        trip.wall_e = 0.0


def steering(trip):
    """Руль: курс по гироскопу с поправкой к центру коридора."""
    trip.yaw, yaw_rate = esp_link.get_yaw_and_rate()   # один снимок на тик
    trip.axis_e = turning.angle_err(trip.yaw, trip.target_yaw)
    on_ramp = abs(trip.pitch_delta) >= config.RAMP_PITCH_DEG
    wall_kp = RAMP_WALL_KP_DEG if on_ramp else WALL_KP_DEG
    wall_max = RAMP_WALL_MAX_DEG if on_ramp else WALL_MAX_DEG
    # На длинной горке гироскоп не накапливает поперечную ошибку: боковые
    # стены по лидару непрерывно сдвигают целевой курс к центру.
    trip.head_off = max(-wall_max, min(wall_max, trip.wall_e * wall_kp))
    trip.head_off_f += (trip.head_off - trip.head_off_f) * HEAD_OFF_SMOOTH
    heading = (trip.target_yaw + trip.head_off_f) % 360

    # ── Руль: один быстрый контур по гироскопу ────────────
    # P по ошибке курса, D по ИЗМЕРЕННОЙ угловой скорости (не по разности
    # углов) - см. GYRO_KD_RATE. Фильтр производной больше не нужен:
    # фильтровать было нечего, кроме шума собственного дифференцирования.
    yaw_e = turning.angle_err(trip.yaw, heading)
    trip.steer = -(yaw_e * config.GYRO_KP + yaw_rate * config.GYRO_KD_RATE)
    trip.steer = max(-MAX_STEER_CMS, min(MAX_STEER_CMS, trip.steer))
    # Та же кривая, что и у fwd: без этого при fwd≈0 и заметной ошибке
    # курса (например, сразу после поворота) команда получалась вида
    # (+20,-20) - робот резко крутился на месте вместо старта с места.
    trip.steer *= trip.ss
    # Ограничение рывка: срезает только скачки команды, в норме не влияет.
    d_max = STEER_SLEW_CMS_S * 0.010
    trip.steer = max(trip.last_steer - d_max, min(trip.last_steer + d_max, trip.steer))
    trip.last_steer = trip.steer

    # Ход сбрасываем по отклонению от ОСИ клетки, а не от смещённой
    # цели - иначе робот тормозил бы просто потому, что центруется.
    trip.fwd *= max(0.35, 1.0 - abs(trip.axis_e) / 18.0)


def send_motor_command(trip):
    """Команда моторам: момент (ассист) или скорости колёс."""
    if trip.assist_mode == 'ramp':
        if not trip.ramp_announced:
            print(f"\n    горка: pitch={trip.pitch_up:+.1f}°, "
                  f"момент {RAMP_TORQUE_A:.1f}A + коррекция курса")
            trip.ramp_announced = True
        # steer уже ограничен ±MAX_STEER_CMS, так что отношение лежит
        # в ±1 и внешний clamp здесь - только страховка от правок выше.
        turn_a = max(-RAMP_STEER_A, min(RAMP_STEER_A,
                     (trip.steer * config.TURN_SIGN / MAX_STEER_CMS) * RAMP_STEER_A))
        esp_link.torque(RAMP_TORQUE_A + turn_a, RAMP_TORQUE_A - turn_a, config.RAMP_CAP_CMS)
    elif trip.assist_mode == 'kick':
        turn_a = max(-OBSTACLE_STEER_A, min(OBSTACLE_STEER_A,
                     (trip.steer * config.TURN_SIGN / MAX_STEER_CMS) * OBSTACLE_STEER_A))
        esp_link.torque(config.OBSTACLE_TORQUE_A + turn_a, config.OBSTACLE_TORQUE_A - turn_a, config.OBSTACLE_CAP_CMS)
    else:
        esp_link.motors(trip.fwd + trip.steer * config.TURN_SIGN, trip.fwd - trip.steer * config.TURN_SIGN)


def report_status(trip):
    """Строка статуса и данные для пульта, не чаще STATUS_PERIOD_S."""
    # Статус ≤10 Гц: на 200 Гц забитый pipe SSH тормозил бы сам цикл.
    if time.monotonic() - trip.last_status_t >= STATUS_PERIOD_S:
        trip.last_status_t = time.monotonic()
        state.drive_live = {"cell": trip.cells + 1, "rem_mm": round(trip.rem * 10), "spd": round(trip.fwd, 1),
                       "steer": round(trip.steer, 1), "wall_e": round(trip.wall_e),
                       "head_off": round(trip.head_off, 1), "mode": trip.assist_mode,
                       "t": trip.last_status_t}
        sys.stdout.write(f"\r    кл.{trip.cells + 1} ост.{trip.rem*10:4.0f}мм  "
                         f"yaw={trip.yaw:.1f}°(e={trip.axis_e:+.1f})  "
                         f"we={trip.wall_e:+.0f}→{trip.head_off:+.1f}°  spd={trip.fwd:.0f}   ")
        sys.stdout.flush()


def wait_next_tick(trip):
    """Ждать следующий пакет телеметрии."""
    # Такт по приходу пакета, а не sleep(0.010): два независимых периода
    # по 10мс давали биение до целого периода сверху - самый дешёвый
    # источник запаздывания в контуре (см. STOP_LEAD_S). Таймаут только
    # страхует от зависания петли при обрыве линка.
    trip.telem_seq = esp_link.wait_telem(trip.telem_seq)
