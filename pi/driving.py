"""Езда вперёд по клеткам: такт поездки по шагам (run_tick) и короткие подъезды (creep_mm)."""
from __future__ import annotations

import math
import time

import config
import state
import timing
import bumper
import drive_checks
import drive_front
import drive_motion
import drive_ramp
import drive_stop
import drive_trip
import esp_link
import floor_tiles
import marks
import recovery
import turning
from motion_estimate import ProjectedOdometry, MotionDiscontinuity, rebase


ALIGN_SPEED_CMS = 18.0    # вдвое быстрее прежней доводки (была ~9 см/с)


@timing._timed("ход")
def drive_cells(max_cells, expect_ramp=False, cell=None, hdg=None, plan=None):
    """См. _drive_cells. cell, hdg - клетка, из которой едем, и курс по карте:
    без них наблюдатель меток на ходу не включается (victim_watch.py). Что бы
    ни остановило поездку, наблюдатель после неё выключен.

    plan (_DrivePlan) - проезд без остановок: в каждой клетке решает план
    ("прямо или стоп", по стенам впереди из corridor.py), а не прежнее
    "справа открыто или спереди стена"; проеханные клетки план сам заносит в
    карту. Без plan - прежнее поведение."""
    try:
        return _drive_cells(max_cells, expect_ramp, cell, hdg, plan)
    finally:
        state.drive_live = None
        if state.watcher is not None:
            state.watcher.arm(set())
            line = state.watcher.take_stats()
            if line:
                print(line)


def run_tick(trip):
    """Один такт поездки: шаги по порядку. Шаг возвращает None - дальше,
    "stop" - закончить поездку, "next_tick" - сразу следующий такт,
    "quit" - выйти сразу, без доводки (причина уже в trip.reason)."""
    steps = (
        drive_checks.check_abort,
        drive_checks.update_odometry,
        drive_checks.check_progress,
        bumper.check_bumper_while_driving,
        floor_tiles.check_floor_while_driving,
        drive_ramp.track_slope,
        drive_front.track_front_wall,
        drive_checks.check_lidar_progress,
        drive_checks.correct_longitudinal_drift,
        drive_checks.commit_passages,
        drive_front.collect_cell_walls,
        marks.watch_marks_while_driving,
        drive_checks.check_wall_too_close,
        drive_front.remaining_distance,
        drive_front.brake_if_wall_close,
        drive_front.decide_at_boundary,
        drive_stop.aim_at_mark,
        drive_stop.check_arrived,
        drive_motion.speed_profile,
        drive_motion.wall_offset,
        drive_motion.steering,
        drive_ramp.update_assist,
        drive_motion.send_motor_command,
        drive_motion.report_status,
        drive_motion.wait_next_tick,
    )
    for step in steps:
        result = step(trip)
        if result is not None:
            return result
    return None


def finish_trip(trip):
    """После цикла: тормоз, горка в карту, ожидание моторов, откат или доводка."""
    if trip.watcher is not None:
        trip.watcher.arm(set())      # доводке и скану стоя камера нужна в обычном режиме

    if (getattr(trip, 'passages', None) is not None and trip.passages.pending
            and trip.reason not in ('done', 'stop', 'black', 'ramp')):
        # Остановились между решением "прямо" и центром клетки: она не
        # пройдена. Засчитаны только клетки, чей центр уже пересекли.
        trip.cells = min(trip.cells, trip.passages.pending[0][0] - 1)

    # Гребень уже позади - что бы ни остановило робота дальше (стена
    # вплотную, бампер, таймаут), он на другом этаже, и вызывающий обязан
    # занести горку в карту, а не шагнуть на соседа по одометрии.
    if trip.ramp_state == 'exit' and trip.reason not in ("lifted", "paused", "link_lost", "lidar_lost", "vision_lost", "gyro_lost", "odo_fault", "pose_lost", "slip"):
        trip.cells, trip.reason = trip.ramp_cells0 + 1, "ramp"

    # Никакого доката: профиль уже привёл скорость почти к нулю, поэтому сразу
    # в фиксацию. ESP держит 0 dps активно и повторяет команду, пока мы меряем
    # стены - так же жёстко, как в конце поворота.
    #
    # Исключение - робот в руках: _lift_thread уже заглушил моторы командой
    # stop(), и фиксировать 0 dps на весу значит драться с человеком.
    if trip.reason in ("slip", "lidar_lost", "vision_lost", "pose_lost"):
        esp_link.settle_brake(0.10)
    elif state.lifted.is_set():
        esp_link.stop()
    else:
        esp_link.settle_brake(0.06)

    # Include coast-down before calculating obstacle rollback. A link outage
    # is checked after recovery, when its final observations become available.
    if (not state.lifted.is_set() and trip.reason not in
            ("paused", "link_lost", "gyro_lost", "odo_fault")):
        drive_checks.integrate_motion(trip)

    # Связь или моторы пропали посреди поездки (выключили питание моторов,
    # отошёл провод) - это не конец заезда. Ждём их здесь же, а потом
    # ставим робота в центр ближайшей засчитанной клетки: стоит он там, где
    # его застал обрыв, а вызывающий считает от центра клетки. Энкодеры живы,
    # но одометрия не росла - не сюда: это застревание, его разбирает
    # вызывающий (отъезд и закрытое ребро).
    # Так же - пропавшие лидар и курс BNO085: ждём их и встаём в центр.
    if (trip.reason in ("odo_fault", "link_lost", "gyro_lost", "lidar_lost")
            and not state.lifted.is_set()):
        c0 = trip.cell if trip.cell is not None else (state.live["cx"], state.live["cy"])
        h0 = trip.hdg if trip.hdg is not None else state.live["hdg"]
        waited = recovery.wait_for_robot(c0[0], c0[1], h0, state.live["cp"], state.live["phase"])
        if waited or trip.reason in ("gyro_lost", "lidar_lost"):
            if state.lifted.is_set():
                return  # ESP rebooted while waiting: no odometry-based retreat
            drive_checks.integrate_motion(trip)
            k = trip.cells + 1 - int(round((trip.boundary_cm - trip.traveled_cm) / trip.cell_cm))
            k = max(0, min(trip.cells, k))
            off_cm = trip.traveled_cm - (trip.boundary_cm - (trip.cells + 1 - k) * trip.cell_cm)
            print(f"    продолжаю: в центр клетки {k} поездки ({-off_cm*10:+.0f}мм)")
            if abs(off_cm) > 0.5:
                creep_mm(-off_cm * 10.0, trip.target_yaw,
                         timeout_s=2.0 + 1.5 * abs(off_cm) / ALIGN_SPEED_CMS)
            trip.cells, trip.reason = k, "resumed"

    if trip.reason == "ramp_stall":
        # Скатываемся задом ровно на пройденное - обратно в центр клетки, из
        # которой выехали. Карта при этом не меняется: робот там же, где был.
        if trip.traveled_cm > 0.2:
            creep_mm(-trip.traveled_cm * 10.0, trip.target_yaw,
                     timeout_s=2.0 + 1.5 * trip.traveled_cm / ALIGN_SPEED_CMS)
    elif trip.reason == "front_obstacle":
        # Назад в центр клетки, из которой выехали: по передней "стене" тут
        # не выровняться - это препятствие, а не граница клетки.
        back_cm = trip.traveled_cm - trip.forced_from_cm
        if back_cm > 0.2:
            creep_mm(-back_cm * 10.0, trip.target_yaw,
                     timeout_s=2.0 + 1.5 * back_cm / ALIGN_SPEED_CMS)
    elif trip.reason in ("lifted", "paused", "link_lost", "lidar_lost", "vision_lost", "gyro_lost", "pose_lost", "obstacle_stall", "slip", "timeout",
                         "bumper_left", "bumper_right", "bumper_both"):
        # Выравниваться нечем и незачем: робота либо держат в руках, либо
        # связи нет, либо он упёрся в препятствие и решение принимает
        # вызывающий (flood_fill_run/return_to_start: откат и повтор).
        pass
    elif trip.reason == "black":
        # Сдаём назад на ту клетку, с которой заехали. Доводку по стене здесь
        # не делаем: она встала бы по передней стене, то есть потащила бы
        # робота обратно к чёрному.
        if trip.black_back_cm > 0.2:
            creep_mm(-trip.black_back_cm * 10.0, trip.target_yaw)
    elif trip.reason == "bumper_entered":
        # Клетка засчитана, впереди стена или препятствие: только отпустить
        # концевики. Доводку не делаем - по стене за препятствием она снова
        # повела бы носом в него.
        creep_mm(-bumper.BUMPER_ENTERED_BACKOFF_MM, trip.target_yaw)
    else:
        # Проверяем СВЕЖИМ замером всегда. Раньше остановка "по стене" считалась
        # заведомо точной и доводка пропускалась - но дистанция там берётся из
        # f_est, продлённой одометрией между сканами лидара. Если та оценка
        # врала, ошибка ничем не ловилась, и следующий проезд начинался ближе к
        # стене, чем робот думал - ровно так он и попал бампером в стену.
        #
        # Горку отменили, а "до гребня" ехали без счёта клеток и проскочили
        # центр засчитанной: назад в него, иначе карта отстаёт от робота на
        # клетку-другую (заезд 17.10 15:31: 740мм засчитаны одной клеткой).
        over_cm = trip.traveled_cm - trip.boundary_cm
        if trip.ramp_rejected and trip.reason in ("stop", "done") and over_cm * 10.0 > config.ALIGN_TOL_MM:
            print(f"    горка отменена, проскочили центр клетки на {over_cm*10:.0f}мм - назад")
            creep_mm(-over_cm * 10.0, trip.target_yaw,
                     timeout_s=2.0 + 1.5 * over_cm / ALIGN_SPEED_CMS)
        turning.align_at_wall(trip.target_yaw)
    print()


def _drive_cells(max_cells, expect_ramp, cell, hdg, plan):
    """Едет вперёд, пересекая границы клеток БЕЗ остановки, пока очередная
    клетка не потребует поворота (справа открылось или спереди стена), пока
    не кончится лимит клеток, или пока не вмешается бампер/авария/пауза.

    expect_ramp - по карте это ребро - горка (см. _ramps). Тогда подъём не
    отбраковывается как "лестница" по длине/высоте: невезучий замер на
    повторном проезде не должен остановить робота посреди известной горки.

    Возвращает (пройдено_клеток, причина):
      'stop'           - встали, вызывающий пусть замерит стены и решит, куда
                         поворачивать
      'done'           - выбран лимит клеток
      'timeout'        - не уложились в бюджет времени на клетку
      'paused'         - кнопка на ESP
      'lifted'         - робота сняли с поля (моторы уже заглушены)
      'link_lost'      - телеметрия с ESP пропала
      'odo_fault'      - энкодеры молчат или одометрия не растёт на ровном полу
      'obstacle_stall' - не взяли препятствие; НЕ авария, вызывающий делает
                         откат и повтор
      'ramp'           - проехали горку целиком и стоим в центре первой
                         клетки после неё; замер в _last_ramp, в карту его
                         заносит вызывающий через record_ramp()
      'ramp_stall'     - встали на горке; робот уже откатился обратно в
                         клетку, откуда выехал, дальше - как obstacle_stall
      'black'          - въехали на чёрную плитку
      'wall_close'     - лидар видит стену ближе FRONT_EMERGENCY_MM; при n=0 -
                         стена прямо перед текущей клеткой
      'front_obstacle' - впереди внезапно что-то близко (не на границе
                         клетки), остановились, не доехав до середины;
                         робот уже вернулся в центр исходной клетки
      'bumper_*'       - концевики; 'bumper_entered' - упёрлись обоими, уже
                         заехав в следующую клетку (засчитана, робот откатился)
    """
    trip = drive_trip.start_trip(max_cells, expect_ramp, cell, hdg, plan)
    while True:
        result = run_tick(trip)
        if result == "quit":
            return trip.cells, trip.reason
        if result == "stop":
            break
    finish_trip(trip)
    return trip.cells, trip.reason


def creep_mm(dist_mm, target_yaw, timeout_s=2.0, speed_cms=None, accel_cms2=None):
    """Проехать ровно dist_mm (знак = направление), держа курс по гироскопу.

    Меряем лидаром, а ЕДЕМ по одометрии: лидар обновляется 10 Гц, по нему
    15мм не отработать - прежняя доводка отмеряла ход секундомером и поэтому
    промахивалась. Разворот руля здесь работает одинаково вперёд и назад:
    вращение задаёт разность колёс, а не их знак.

    Возвращает измеренное знаковое перемещение в мм: вызывающий (возврат от
    метки) отыгрывает именно его, а не заказанное.
    accel_cms2 - плавно: разгон и торможение с этим ускорением (не ниже
    CREEP_MIN_CMS), иначе сразу speed_cms (см. creep_soft).
    """
    if not math.isfinite(dist_mm) or not math.isfinite(target_yaw):
        esp_link.brake()
        raise ValueError("creep distance and heading must be finite")
    if abs(dist_mm) < 1.0:
        return 0.0
    v_max = ALIGN_SPEED_CMS if speed_cms is None else speed_cms
    if (not math.isfinite(v_max) or v_max <= 0 or not math.isfinite(timeout_s) or timeout_s <= 0
            or (accel_cms2 is not None and (not math.isfinite(accel_cms2) or accel_cms2 <= 0))):
        esp_link.brake()
        raise ValueError("creep speed, timeout and acceleration must be positive")
    v = math.copysign(v_max, dist_mm)
    goal_cm = abs(dist_mm) / 10.0
    actual_cm = 0.0
    try:
        odometry = ProjectedOdometry(esp_link.motion_snapshot(), target_yaw)
    except MotionDiscontinuity:
        esp_link.brake()            # энкодеры молчат - мерить подъезд нечем
        print("    Подъезд: нет одометрии - пропускаю")
        return 0.0

    def consume_motion():
        nonlocal actual_cm, odometry
        try:
            samples = esp_link.motion_since(odometry.last.seq)
        except MotionDiscontinuity:
            samples = [esp_link.motion_snapshot()]
        for sample in samples:
            try:
                actual_cm += odometry.update(sample)
            except MotionDiscontinuity as error:
                odometry, gap = rebase(odometry, sample)
                actual_cm += gap
                print(f"    [WARN] подъезд: разрыв одометрии ({error}) - по колёсам {gap:+.1f}см")

    t0 = time.monotonic()
    telem_seq = state.telem_seq
    while time.monotonic() - t0 < timeout_s:
        # Бампер останавливает только движение вперёд: отъезд назад от стены -
        # это как раз тот случай, когда концевик заведомо прижат.
        if esp_link.interrupted() or not esp_link.encoders_online() or (dist_mm > 0 and bumper.get_bumper()):
            break
        consume_motion()
        # Та же поправка на запаздывание, что и в основном профиле: одометрия
        # показывает, где робот был ~30мс назад, поэтому цель считается
        # достигнутой чуть раньше. Без этого доводка регулярно проскакивала и
        # следующая итерация отыгрывала назад.
        done_cm = actual_cm * (1 if dist_mm >= 0 else -1)
        lead_cm = min(abs(v) * config.STOP_LEAD_S, max(0.0, goal_cm - 0.2))
        if done_cm >= goal_cm - lead_cm:
            break
        if accel_cms2:
            # плавно: разгон по времени, торможение по остатку пути
            v = math.copysign(max(CREEP_MIN_CMS, min(
                v_max, CREEP_MIN_CMS + accel_cms2 * (time.monotonic() - t0),
                math.sqrt(2.0 * accel_cms2 * max(0.0, goal_cm - done_cm)))), dist_mm)
        yaw_e = turning.angle_err(esp_link.get_yaw(), target_yaw)
        st = -(yaw_e * config.GYRO_KP + esp_link.get_yaw_rate() * config.GYRO_KD_RATE)
        # медленно - руль слабее, чтобы ни одно колесо не крутилось назад
        lim = 8.0 if not accel_cms2 else max(2.0, 0.6 * abs(v))
        st = max(-lim, min(lim, st))
        esp_link.motors(v + st * config.TURN_SIGN, v - st * config.TURN_SIGN)
        telem_seq = esp_link.wait_telem(telem_seq)

    esp_link.brake()
    time.sleep(0.08)
    consume_motion()  # Include braking travel before a caller plans the return.
    error_cm = abs(dist_mm / 10.0 - actual_cm)
    if error_cm > max(0.5, abs(v) * config.STOP_LEAD_S + 0.3):
        print(f"    [WARN] подъезд не завершён: цель {dist_mm:+.0f}мм, "
              f"измерено {actual_cm * 10:+.0f}мм")
    return actual_cm * 10.0


# Подъезды у метки (к ней, сдвиг для другого блика, отъезд передней камерой и
# обратно) - медленнее доводки и без рывков: робот рядом со стеной, камера
# снимает, а прежние 18 см/с с места выглядели как дёрганье туда-обратно.
MARK_CREEP_CMS = 9.0
MARK_CREEP_ACCEL_CMS2 = 20.0
CREEP_MIN_CMS = 4.0


def creep_soft(dist_mm, target_yaw):
    """creep_mm у метки: MARK_CREEP_CMS с плавным разгоном и торможением."""
    t = 1.5 + abs(dist_mm) / 10.0 / (MARK_CREEP_CMS * 0.5)
    return creep_mm(dist_mm, target_yaw, timeout_s=t, speed_cms=MARK_CREEP_CMS,
             accel_cms2=MARK_CREEP_ACCEL_CMS2)
