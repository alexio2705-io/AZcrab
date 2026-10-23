"""Шаги поездки: аварийные проверки (снят, пауза, связь, энкодеры, застревание) и одометрия."""
from __future__ import annotations

import time
import math

import config
import state
import esp_link
from motion_estimate import MotionDiscontinuity, rebase


def integrate_motion(trip):
    """Consume every received pose sample, also after a blocking marker stop.
    Разрыв истории не останавливает поездку: см. motion_estimate.rebase."""
    try:
        samples = esp_link.motion_since(trip.odometry.last.seq)
    except MotionDiscontinuity:
        samples = [esp_link.motion_snapshot()]   # буфер переполнился за паузу
    for sample in samples:
        dyaw = (sample.yaw - trip.odometry.last.yaw + 180) % 360 - 180
        try:
            delta = trip.odometry.update(sample)
        except MotionDiscontinuity as error:
            trip.odometry, delta = rebase(trip.odometry, sample)
            print(f"\n    [WARN] разрыв одометрии ({error}) - по среднему ходу колёс {delta:+.1f}см")
        trip.traveled_cm += delta
        if hasattr(trip, 'raw_odo_t'):
            trip.raw_traveled_cm += delta
            trip.raw_odo_t.add(sample.t, trip.raw_traveled_cm)
        trip.odo_t.add(sample.t, trip.traveled_cm)
        trip.yaw_t.add(sample.t, trip.yaw_t.d[-1] + dyaw)
    last = trip.odometry.last
    trip.cl = trip.last_l = last.left
    trip.cr = trip.last_r = last.right


# Слепая зона лидара (walls.py: self_outline_mm + SELF_MARGIN) в окне ±22°
# фронта доходит до ~119мм от центра - точки ближе отбрасываются как шум от
# собственного корпуса, а не потому что стены нет. Прежний порог 122 стоял
# ВНУТРИ этой зоны: к моменту, когда стена подходила настолько близко, лидар
# уже переставал её видеть (f_now становился None), и аварийный стоп не
# успевал сработать по валидному замеру - оставался только бампер. 140 даёт
# ~20мм запаса над слепой зоной и всё ещё ниже штатной точки остановки
# (FRONT_CENTER_MM=145), чтобы не срабатывать ложно на обычных подъездах.
FRONT_EMERGENCY_MM = 140

# Одометрия — единственный безусловный ограничитель 300 мм. Если при поданном ходе
# она не меняется дольше этого времени, дальше ехать нельзя.
ODO_STALL_S = 0.7
SPEED_WIN_S = 0.06             # окно оценки скорости по одометрии

# Пока нос задран, ODO_STALL_S не применяется: упереться в препятствие на
# полсекунды - это штатная часть его преодоления, а не отказ одометрии.
# Работает более длинный таймер, и его срабатывание даёт НЕ фатальный
# odo_fault, а "obstacle_stall" - откат и повтор в flood_fill_run.
ASSIST_STALL_S = 2.5

# Поле, где все мелкие препятствия заведомо проезжаемы (challenge.py ставит
# True): не взяли бугорок с ходу - не откатываемся, а толкаем дальше, взведя
# импульсы момента заново, до PUSH_THROUGH_MAX раз подряд. Подтверждённая
# горка (ramp_state 'on') - как раньше: застряв на ней, робот скатывается.
# Кандидат в горку ('cand') - это и есть бугорок, задравший нос выше
# RAMP_MAP_DEG, его тоже толкаем, а не скатываемся.
PUSH_THROUGH_SMALL = False
PUSH_THROUGH_MAX = 4

# Штатный бюджет времени на клетку. На препятствии он продлевается ровно на
# столько, сколько длится ассист (иначе подъём упирался в таймаут и клетка
# просто терялась: cells не инкрементировался, а карта уезжала молча).
CELL_TIMEOUT_S = 8.0


def check_abort(trip):
    """Сняли с поля, пауза кнопкой, нет телеметрии или кончилось время клетки."""
    if state.lifted.is_set():
        trip.reason = "lifted"
        return "quit"
    if esp_link.get_esp_paused():
        # BTN_1 - перезапуск с чекпоинта, в том числе с горки: робота
        # всё равно переносят руками, недоезженная горка не записывается.
        trip.reason = "paused"
        return "quit"
    if not esp_link.telem_fresh():
        trip.reason = "link_lost"
        print(f"\n    АВАРИЯ: нет телеметрии {esp_link.telem_age():.2f}с — торможу")
        return "stop"
    if not esp_link.gyro_fresh():
        trip.reason = "gyro_lost"
        print("\n    АВАРИЯ: нет свежего курса BNO085 — торможу")
        return "stop"
    if not esp_link.encoders_online():
        # Offline sides are encoded as distance zero. Never integrate that
        # sentinel as real backward motion before noticing the offline flag.
        trip.reason = "odo_fault"
        print("\n    АВАРИЯ: нет свежей одометрии — торможу")
        return "stop"
    # Бюджет времени клетки продлевается ровно на длительность ассиста:
    # подъём по горке законно занимает секунды, и раньше он упирался в
    # этот таймаут, а клетка при этом молча терялась (cells не рос).
    if time.monotonic() - trip.t_cell > CELL_TIMEOUT_S + trip.assist_extra_s:
        trip.reason = "timeout"
        print("    ТАЙМАУТ!")
        return "stop"


def update_odometry(trip):
    """Наклон, пройденный путь и скорость по энкодерам."""
    # Наклон нужен уже здесь: по нему решается, каким таймером мерить
    # залипание одометрии - штатным или «мы на препятствии».
    trip.pitch_delta = esp_link.get_pitch() - trip.pitch_start

    # ── Одометрия ─────────────────────────────────────────
    if integrate_motion(trip):
        return "stop"

    trip.now_m = time.monotonic()
    state.tick_ms["вкл" if trip.watch_armed else "выкл"].add((trip.now_m - trip.last_loop_t) * 1000.0)
    trip.loop_dt = min(0.05, trip.now_m - trip.last_loop_t)
    trip.last_loop_t = trip.now_m
    trip.odo_hist.append((trip.now_m, trip.traveled_cm))
    while len(trip.odo_hist) > 2 and trip.now_m - trip.odo_hist[1][0] >= SPEED_WIN_S:
        trip.odo_hist.popleft()
    t_a, d_a = trip.odo_hist[0]
    if trip.now_m - t_a > 0.02:
        trip.v_meas = (trip.traveled_cm - d_a) / (trip.now_m - t_a)


def check_progress(trip):
    """Энкодеры живы и робот движется; иначе - застревание или авария."""
    # Без двух энкодеров нельзя гарантировать остановку через 300 мм.
    if not esp_link.encoders_online():
        trip.reason = "odo_fault"
        l_ok, r_ok = esp_link.encoder_sides()
        # Печатаем, КАКАЯ сторона молчит, и сырую одометрию рядом: бит в
        # телеметрии и строка debug_task на ESP считаются из одного и того
        # же isnan(), так что разойтись они не могут - и если ESP при этом
        # показывает живые энкодеры, значит расходятся версии или момент
        # замера, а не сами данные.
        print(f"\n    АВАРИЯ: энкодер не отвечает "
              f"(левый={'ok' if l_ok else 'НЕТ'}, "
              f"правый={'ok' if r_ok else 'НЕТ'}, "
              f"L={trip.cl:.1f} R={trip.cr:.1f}см) — торможу")
        return "stop"
    # Нос задран - робот сейчас берёт препятствие, и постоять полсекунды
    # на месте для этого нормально. Раньше здесь работал один общий
    # ODO_STALL_S=0.7, который кончался на 50мс позже единственного
    # импульса момента: «не взял бордюр» = [FATAL] на весь заезд.
    on_obstacle = trip.pitch_delta >= config.OBSTACLE_REARM_DEG or trip.assist_mode is not None
    stall_limit = ASSIST_STALL_S if on_obstacle else ODO_STALL_S
    if trip.traveled_cm >= trip.last_progress_cm + config.ODO_PROGRESS_CM:
        trip.last_progress_cm = trip.traveled_cm
        trip.last_progress_t = time.monotonic()
    elif (PUSH_THROUGH_SMALL and on_obstacle and trip.ramp_state != 'on'
          and trip.fwd >= config.MOVE_MIN_CMS and time.monotonic() - trip.last_progress_t > stall_limit
          and trip.push_tries < PUSH_THROUGH_MAX):
        trip.push_tries += 1
        trip.assist_kicks = 0
        trip.assist_gap_until = 0.0
        trip.last_progress_t = time.monotonic()
        trip.t_cell = max(trip.t_cell, time.monotonic() - CELL_TIMEOUT_S / 2)   # бюджет клетки тоже
        print(f"\n    бугорок не взят с ходу (pitch={trip.pitch_delta:+.1f}°) - "
              f"толкаю ещё, без отката ({trip.push_tries}/{PUSH_THROUGH_MAX})")
    elif trip.fwd >= config.MOVE_MIN_CMS and time.monotonic() - trip.last_progress_t > stall_limit:
        if trip.ramp_state == 'on' or (trip.ramp_state == 'cand' and not PUSH_THROUGH_SMALL):
            trip.reason = "ramp_stall"
            print(f"\n    ГОРКА НЕ ВЗЯТА на {trip.traveled_cm*10:.0f}мм "
                  f"(pitch={trip.pitch_delta:+.1f}°) — скатываюсь обратно")
        elif on_obstacle or trip.ramp_state == 'cand':
            trip.reason = "obstacle_stall"
            print(f"\n    ПРЕПЯТСТВИЕ НЕ ВЗЯТО (pitch={trip.pitch_delta:+.1f}°, "
                  f"{trip.assist_kicks} импульс(ов)) — откат и повтор")
        else:
            trip.reason = "odo_fault"
            print("\n    АВАРИЯ: одометрия не растёт — торможу")
        return "stop"


def check_wall_too_close(trip):
    """Стена вплотную (ближе FRONT_EMERGENCY_MM) - стоп сразу."""
    # Не глушится и на склоне: при лидаре на 130мм пол носом вниз ближе
    # FRONT_EMERGENCY_MM - это уклон больше 40°, а стена вплотную - всегда
    # стена. Поэтому берём сырое показание, а не обнулённое выше.
    f_raw = trip.walls["front"]
    if f_raw is not None and f_raw <= FRONT_EMERGENCY_MM:
        print(f"\n    СТЕНА ВПЛОТНУЮ: f={f_raw:.0f}мм  d={trip.traveled_cm*10:.0f}мм")
        # Раньше причина оставалась "stop"/"done" и клетка не засчитывалась
        # никогда. Два случая:
        #  - почти не сдвинулись: стена прямо перед клеткой, где стоим, а
        #    карта считала её открытой - вызывающий записывает её в карту
        #    (иначе BFS раз за разом выбирал тот же путь и робот "застревал"
        #    стоя на месте, см. MAX_STALLS);
        #  - проехали больше полклетки: мы уже в следующей клетке, просто
        #    ближе к стене, чем надо - засчитываем, доводка отъедет назад.
        trip.reason = "wall_close"
        if trip.traveled_cm >= (trip.boundary_cm - trip.cell_cm) + trip.cell_cm * 0.5:
            trip.cells += 1
        return "stop"


def check_lidar_progress(trip):
    observed_yaw = trip.yaw_t.at_checked(trip.front_scan_t)
    if trip.slip_guard.update(trip.scan_id, time.monotonic(),
            trip.front_scan_t if trip.front_scan_cm is not None and observed_yaw is not None else None,
            ((trip.raw_odo_t.at_checked(trip.front_scan_t) or 0) if hasattr(trip, 'raw_odo_t')
             else (trip.front_scan_cm or 0)) * 10,
            trip.target_yaw + (observed_yaw or 0), trip.pitch_delta, trip.walls, trip.fwd):
        esp_link.brake()
        # Колёса крутятся, корпус стоит - то же, что не взятое препятствие:
        # вызывающий откатится и попробует ребро ещё раз (_obstacle_stall).
        trip.reason = "obstacle_stall"
        print("\n    ПРОБУКСОВКА: колёса вращаются, стены не смещаются - откат и повтор")
        return "stop"


def correct_longitudinal_drift(trip):
    if trip.front_scan_cm is None or trip.on_slope_now:
        return
    yaw = trip.yaw_t.at_checked(trip.front_scan_t)
    if yaw is None:
        return
    # Require steady motion for compensation between the two sector times.
    if abs(trip.v_meas - trip.fwd) > 5:
        trip.range_fusion.anchor = None
        return
    # Shift the rear range from scan-end to the front sector's timestamp.
    back, back_t = trip.back_observation
    end_pose = trip.odo_t.at_checked(back_t)
    projection = math.cos(math.radians(yaw))
    if back is not None and end_pose is not None:
        back = back * projection - (end_pose - trip.front_scan_cm) * 10
    else:
        back = None
    front = trip.walls['front']
    delta, lost = trip.range_fusion.update(trip.scan_id, time.monotonic(), trip.front_scan_t,
        trip.front_scan_cm, None if front is None else front*projection,
        back, yaw, trip.pitch_delta)
    if lost:
        # Стены и колёса разошлись больше 6 см - одной из сторон верить
        # нельзя. Не правим и не стоим: новая опора со следующего скана.
        trip.range_fusion.anchor = None
        print("\n    [WARN] торцевые стены спорят с одометрией >6см - поправку пропускаю")
        return
    if not delta:
        return
    # Rebase the recent local trajectory and pending observed features together.
    # Tile centers remain fixed. Raw wheel history remains untouched for SlipGuard.
    trip.traveled_cm += delta
    trip.odo_t.d[:] = [v + delta for v in trip.odo_t.d]
    trip.front_scan_cm += delta
    if trip.f_anchor is not None:
        trip.f_anchor_cm += delta
    trip.odo_hist = type(trip.odo_hist)((t, d+delta) for t, d in trip.odo_hist)
    trip.last_progress_cm += delta
    if trip.vmark is not None:
        trip.vmark['pos_mm'] += delta*10
        trip.vmark['stop_cm'] += delta
    for mark in getattr(trip, 'mark_queue', []):
        mark['pos_mm'] += delta*10
    if trip.cell_sides is not None:
        for side in trip.cell_sides.band_x:
            trip.cell_sides.band_x[side] = [x+delta*10 for x in trip.cell_sides.band_x[side]]


def commit_passages(trip):
    for k, _, walls in trip.passages.advance(trip.traveled_cm):
        if trip.plan is not None:
            trip.plan.passed(k, walls)
