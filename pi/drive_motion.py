"""Шаги поездки: скорость, центровка, руль, команда моторам, статус."""
from __future__ import annotations

import math
import sys
import time

import config
import state
import esp_link
import turning
from motion_guard import CenterFilter
from continuous import stopping_speed, stop_suspended


SIDE_CENTER_MM = config.SIDE_CENTER_MM

# Разгон - ПО ВРЕМЕНИ, не по пройденному пути. Профиль "по пути" (было
# ACCEL_MM=60) считает скорость от traveled_cm, а сам traveled_cm растёт тем
# быстрее, чем выше уже скорость - положительная обратная связь даёт РАЗГОН
# ПО ЭКСПОНЕНТЕ: требуемый dv/dt на подходе к MOVE_MAX_CMS улетает далеко за
# любое разумное ограничение мотора, мотор не успевает, расхождение копится
# и потом отыгрывается рывком. Разгон по времени (smoothstep) даёт постоянный,
# предсказуемый и заранее известный пик dv/dt - можно держать его безопасно
# ниже MOTOR_ACCEL_DPS_S на ESP.
RAMP_UP_S = 0.25
# Ограничение камеры меняется после каждого кадра. Не передавать его
# скачком в моторы: замедляемся быстро, разгоняемся обратно плавнее.
VISION_CAP_DOWN_CMS2 = 180.0
VISION_CAP_UP_CMS2 = 60.0
OBSTACLE_STEER_A = 0.175
TILT_CAP_CMS = 35.0            # ход, пока нос задран на препятствии/горке
DESCENT_CAP_CMS = 28.0         # ход, пока нос опущен (съезд с препятствия)
RAMP_DESCENT_CMS = 12.0        # потолок каждого борта на спуске и на известном подходе
RAMP_TORQUE_A = 0.95
RAMP_STEER_A = 0.275
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
# 19.11: при 0.16/6° поправка по стене достигала 6°, а курс уходил до 11°
# при исчезновении стены. Меньший угол сохраняет центровку без резких дуг.
WALL_KP_DEG = 0.10      # градусов поправки курса на мм смещения
WALL_MAX_DEG = 4.0      # предел поправки курса по стене
# Было 12мм, и это неудачно совпадало с рабочей точкой: цель SIDE_CENTER_MM=140
# при замеренном центре клетки в 128мм даёт постоянную ошибку ровно ~12мм, то
# есть робот всё время сидел НА границе мёртвой зоны и коррекция щёлкала
# включено/выключено. 5мм убирает это совпадение, оставаясь выше шума лидара.
WALL_DEADBAND_MM = 5.0
# Дальше этого - стены нет, центроваться не по чему. Было 240: в коридоре
# 280мм своя стена не дальше ~210мм, а 216-231мм - торец стены или стена
# соседней клетки, по ним робот доворачивал на 10° в пустоту (заезд 24.10).
WALL_CENTER_MM = 200

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
    # cand включается по сглаженному углу ещё до подтверждения горки по пути.
    trip.descending = ((trip.ramp_state in ('cand', 'on') and trip.ramp_sign < 0)
                       or (getattr(trip, 'known_descent', False) and trip.ramp_state != 'exit'))
    if trip.descending:
        trip.tilt_lim = min(trip.tilt_lim, RAMP_DESCENT_CMS)
    ramp_up = min(ramp_up, trip.tilt_lim)
    # Until a decision is made, retain enough room for a stop at that center.
    if trip.pending and trip.ramp_state != 'cand':
        ramp_up = min(ramp_up, stopping_speed(max(trip.rem, trip.lead_cm),
            config.STOP_LEAD_S + max(.02, trip.loop_dt), config.BRAKE_DECEL_CMS2))
    vision_cap = getattr(trip, 'perception_cap', float('inf'))
    vision_active = math.isfinite(vision_cap)
    target_cap = vision_cap if vision_active else trip.v_max
    previous = getattr(trip, 'vision_cap_smoothed', None)
    if previous is None or (vision_active and not getattr(trip, 'vision_cap_active', False)):
        previous = target_cap  # новая стена: замедлиться сразу ради камеры
    cap = max(previous - VISION_CAP_DOWN_CMS2 * trip.loop_dt,
              min(previous + VISION_CAP_UP_CMS2 * trip.loop_dt, target_cap))
    trip.vision_cap_smoothed = cap
    trip.vision_cap_active = vision_active
    ramp_up = min(ramp_up, cap)

    trip.braking = False
    if (trip.stopping or trip.mark_first) and not stop_suspended(trip):
        # Решаем v*задержка + v²/(2*a) = остаток сразу относительно v.
        # sqrt от rem_eff использовал ПРЕДЫДУЩУЮ команду: после снижения
        # скорости запас внезапно рос, и перед стопом получалось 5 -> 18см/с.
        # Здесь задержка учитывается один раз, без такого повторного разгона.
        decel = config.MARK_BRAKE_DECEL_CMS2 if trip.mark_first else config.BRAKE_DECEL_CMS2
        v_brake = stopping_speed(trip.rem, config.STOP_LEAD_S, decel, margin_cm=0.0)
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
    if not hasattr(trip, "center_filter"):
        trip.center_filter = CenterFilter()
    scan_t = getattr(trip, "scan_t", state.telem.get("scan_t"))
    # A stale scan must not keep steering; elevated stairs can hide all walls.
    if (scan_t is None or not 0 <= time.monotonic() - scan_t < 0.25
            or abs(trip.pitch_delta) >= config.RAMP_PITCH_DEG):
        trip.wall_e = 0.0
        trip.center_filter.update(None, None, None)
        return
    projection = math.cos(math.radians(turning.angle_err(esp_link.get_yaw(), trip.target_yaw)))
    if projection < 0.94:  # >20 degrees: end faces and openings contaminate ranges
        trip.wall_e = 0.0
        trip.center_filter.update(None, None, None)
        return
    wl = wl * projection if wl is not None else None
    wr = wr * projection if wr is not None else None
    wall_l = wl is not None and wl < WALL_CENTER_MM
    wall_r = wr is not None and wr < WALL_CENTER_MM

    if wall_l and wall_r:
        # Reject two surfaces that cannot bound this one-tile corridor.
        trip.wall_e = ((wr - wl) * 0.5 + config.SIDE_LIDAR_OFFSET_MM
                       if abs(wr + wl - 2 * SIDE_CENTER_MM) <= 50 else 0.0)
    elif wall_l:
        trip.wall_e = SIDE_CENTER_MM + config.SIDE_LIDAR_OFFSET_MM - wl
    elif wall_r:
        trip.wall_e = wr - SIDE_CENTER_MM + config.SIDE_LIDAR_OFFSET_MM
    else:
        trip.wall_e = 0.0
    mode = "both" if wall_l and wall_r else "left" if wall_l else "right" if wall_r else None
    if wall_l and wall_r and abs(wr + wl - 2 * SIDE_CENTER_MM) > 50:
        mode = None
    trip.wall_e = trip.center_filter.update(getattr(trip, "scan_id", scan_t),
                                            trip.wall_e if mode else None, mode)
    if abs(trip.wall_e) < WALL_DEADBAND_MM:
        trip.wall_e = 0.0


def steering(trip):
    """Руль: курс по гироскопу с поправкой к центру коридора."""
    trip.yaw, yaw_rate = esp_link.get_yaw_and_rate()   # один снимок на тик
    trip.axis_e = turning.angle_err(trip.yaw, trip.target_yaw)
    on_ramp = abs(trip.pitch_delta) >= config.RAMP_PITCH_DEG
    wall_kp = RAMP_WALL_KP_DEG if on_ramp else WALL_KP_DEG
    wall_max = RAMP_WALL_MAX_DEG if on_ramp else WALL_MAX_DEG
    # Significant tilt suppresses wall_offset; retain the gyro heading there.
    trip.head_off = max(-wall_max, min(wall_max, trip.wall_e * wall_kp))
    alpha = 1.0 - math.exp(-max(0.0, trip.loop_dt) / 0.14)
    trip.head_off_f += (trip.head_off - trip.head_off_f) * alpha
    heading = (trip.target_yaw + trip.head_off_f) % 360

    # ── Руль: один быстрый контур по гироскопу ────────────
    # P по ошибке курса, D по ИЗМЕРЕННОЙ угловой скорости (не по разности
    # углов) - см. DRIVE_YAW_KD_RATE. Фильтр производной больше не нужен:
    # фильтровать было нечего, кроме шума собственного дифференцирования.
    yaw_e = turning.angle_err(trip.yaw, heading)
    # При медленном прямом ходе большой руль разворачивал одно колесо назад:
    # вместо удержания курса робот вилял даже без боковых стен. Оставляем
    # обоим колёсам движение вперёд; на горке и при ассисте предел прежний.
    forward = trip.fwd * max(0.35, 1.0 - abs(trip.axis_e) / 18.0)
    steer_limit = (MAX_STEER_CMS if trip.assist_mode is not None or on_ramp else
                   min(MAX_STEER_CMS, 0.6 * forward))
    pd = yaw_e * config.DRIVE_YAW_KP + yaw_rate * config.DRIVE_YAW_KD_RATE
    # Интеграл копится только на ровном ходу своим ходом и не растёт,
    # если руль уже в пределе: иначе после смены знака ошибки он ещё долго
    # толкает робота в прежнюю сторону.
    if trip.assist_mode is None and not on_ramp and trip.fwd >= config.MOVE_MIN_CMS:
        next_i = max(-config.DRIVE_YAW_I_MAX, min(config.DRIVE_YAW_I_MAX,
                     trip.yaw_i + yaw_e * config.DRIVE_YAW_KI * trip.loop_dt))
        if abs(pd + next_i) <= steer_limit or abs(pd + next_i) < abs(pd + trip.yaw_i):
            trip.yaw_i = next_i
    trip.steer = -max(-steer_limit, min(steer_limit, pd + trip.yaw_i))
    # Та же кривая, что и у fwd: без этого при fwd≈0 и заметной ошибке
    # курса (например, сразу после поворота) команда получалась вида
    # (+20,-20) - робот резко крутился на месте вместо старта с места.
    trip.steer *= trip.ss
    # Ограничение рывка: срезает только скачки команды, в норме не влияет.
    d_max = STEER_SLEW_CMS_S * trip.loop_dt
    trip.steer = max(trip.last_steer - d_max, min(trip.last_steer + d_max, trip.steer))
    if getattr(trip, 'descending', False):
        # Оба борта катятся вперёд. При большом руле уменьшаем обе команды
        # пропорционально, чтобы сохранить кривизну и потолок внешнего борта.
        trip.steer = max(-0.6 * forward, min(0.6 * forward, trip.steer))
        scale = RAMP_DESCENT_CMS / max(RAMP_DESCENT_CMS, forward + abs(trip.steer))
        forward *= scale
        trip.steer *= scale
    trip.last_steer = trip.steer

    # Ход сбрасываем по отклонению от ОСИ клетки, а не от смещённой
    # цели - иначе робот тормозил бы просто потому, что центруется.
    trip.fwd = forward


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
                       "travel_mm": round(trip.traveled_cm * 10, 1),
                       "origin": list(trip.cell) if trip.cell is not None else None,
                       "hdg": trip.hdg, "slip": trip.slip_guard.evidence,
                       "range_fusion": trip.range_fusion.evidence,
                       "raw_travel_mm": round(trip.raw_traveled_cm * 10, 1),
                       "vision_cap_cms": round(trip.perception_cap, 1) if math.isfinite(trip.perception_cap) else None,
                       "t": trip.last_status_t}
        sys.stdout.write(f"\r    кл.{trip.cells + 1} ост.{trip.rem*10:4.0f}мм  "
                         f"yaw={trip.yaw:.1f}°(e={trip.axis_e:+.1f})  "
                         f"we={trip.wall_e:+.0f}→{trip.head_off:+.1f}°  "
                         f"spd={trip.fwd:.0f} st={trip.steer:+.1f}   ")
        sys.stdout.flush()


def wait_next_tick(trip):
    """Ждать следующий пакет телеметрии."""
    # Такт по приходу пакета, а не sleep(0.010): два независимых периода
    # по 10мс давали биение до целого периода сверху - самый дешёвый
    # источник запаздывания в контуре (см. STOP_LEAD_S). Таймаут только
    # страхует от зависания петли при обрыве линка.
    trip.telem_seq = esp_link.wait_telem(trip.telem_seq)
