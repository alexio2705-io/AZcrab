"""Шаги поездки: аварийные проверки (снят, пауза, связь, энкодеры, застревание) и одометрия."""
from __future__ import annotations

import time

import config
import state
import esp_link


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
    trip.cl, trip.cr = esp_link.get_odo()
    dl, dr = trip.cl - trip.last_l, trip.cr - trip.last_r
    if abs(dl) < 100 and abs(dr) < 100:   # скачок = только что было 'Z'
        trip.traveled_cm += (dl + dr) / 2.0
    trip.last_l, trip.last_r = trip.cl, trip.cr

    trip.now_m = time.monotonic()
    state.tick_ms["вкл" if trip.watch_armed else "выкл"].add((trip.now_m - trip.last_loop_t) * 1000.0)
    trip.loop_dt = min(0.05, trip.now_m - trip.last_loop_t)
    trip.last_loop_t = trip.now_m
    trip.odo_hist.append((trip.now_m, trip.traveled_cm))
    trip.odo_t.add(trip.now_m, trip.traveled_cm)
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
