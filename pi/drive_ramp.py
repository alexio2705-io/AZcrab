"""Шаги поездки: наклон пола (ступенька, горка) и ассист моментом."""
from __future__ import annotations

import config
import state
import ramps


OBSTACLE_TORQUE_MAX_S = 0.65
# Толчок - только если нос задран не меньше KICK_PITCH_DEG дольше KICK_HOLD_S.
# Заезд 10.11 10:52: на ровных плитках качка на шинах при трогании (+3,1° при
# v=0) давала толчок, а с 3 А на задних - рывок и занос. Невысокие препятствия
# робот переезжает и так.
KICK_PITCH_DEG = 5.0
KICK_HOLD_S = 0.15
OBSTACLE_SLOW_FRAC = 0.6       # импульс, только если едем медленнее этой доли

# Импульс был РОВНО ОДИН на событие: перевзвод (obstacle_armed) требовал
# возврата носа в горизонт, а он на непреодолённом препятствии не наступает
# никогда. При этом OBSTACLE_TORQUE_MAX_S=0.65 кончался за 50мс до
# ODO_STALL_S=0.7 - то есть "не взял бордюр с первого раза" гарантированно
# превращалось в odo_fault, а это [FATAL] на ВЕСЬ заезд.
# Теперь попыток несколько, а счётчик обнуляется любым реальным продвижением,
# так что длинный неровный подъём не съедает лимит.
OBSTACLE_KICKS_MAX = 3
OBSTACLE_KICK_GAP_S = 0.15     # пауза между импульсами - дать колёсам зацепиться
OBSTACLE_PROGRESS_CM = 1.0     # сдвинулись меньше - импульс считаем безрезультатным
RAMP_CONFIRM_S = 0.25
# Было RAMP_TORQUE_MAX_S=5с абсолютным потолком на подъём. Потолок по ОБЩЕМУ
# времени здесь неправильный: горка длиной 2.5м на крейсерской - это уже 4.5с,
# а если робот на ней замедлится (а он замедлится), помощь исчезнет до вершины
# и не вернётся, потому что перевзвод ждёт горизонта. Ограничиваем не подъём,
# а ОТСУТСТВИЕ ПРОГРЕССА: пока одометрия растёт и нос задран - момент держим.
RAMP_STALL_S = 1.2
RAMP_TOTAL_MAX_S = 20.0        # предохранитель от бесконечной подачи тока
RAMP_MAP_HYST_DEG = 3.0        # кандидат сбрасывается ниже RAMP_MAP_DEG - этого
RAMP_CONFIRM_CM = 8.0
RAMP_FLAT_CM = 2.0
RAMP_EXIT_MIN_MM = 50.0
# Горка - устойчивый крутой наклон: длина и высота по колёсам не годятся
# (заезды 09.11: спуск ~470мм колёса считали за 166..225мм, робот скользит).
# Настоящие горки в тех заездах круче 22° и держатся от 0,33с (быстрый спуск),
# ложные наклоны - до 12°. Время отсекает только всплеск на лежачем (~0,1с).
# То, что не прошло, считается препятствием, цель клетки возвращается обычная.
RAMP_MIN_SLOPE_DEG = 18.0
RAMP_MIN_HOLD_S = 0.15
# Детектор горки и счётчик всплесков смотрят на сглаженный pitch (~30мс
# при 200Гц): на пологой горке 8-10° дрожь в пару градусов иначе рвёт
# кандидата, а на ровном - изображает ступеньки. Сам всплеск на лежачем
# длится ~100мс и через фильтр проходит.
RAMP_PITCH_ALPHA = 0.15

# Ступеньки и лежачие: каждый переезд кромки колесом - это поворот колеса
# вокруг ребра, энкодер насчитывает больше, чем робот проехал по полу
# (для колеса 86мм и ступеньки 2см - около +7мм), плюс проскальзывание
# под импульсом момента. Каждый всплеск |pitch| выше OBSTACLE_PITCH_DEG
# отодвигает цель клетки на BUMP_EXTRA_MM, но не больше BUMP_EXTRA_MAX_MM
# за клетку. Подобрано на глаз - проверить на лестнице по задней стене.
BUMP_EXTRA_MM = 5.0
BUMP_EXTRA_MAX_MM = 40.0


def track_slope(trip):
    """Наклон пола: ступенька, кандидат в горку, горка, съезд с неё."""
    # ── Горка или всплеск: что под колёсами ──────────────
    trip.pitch_f = trip.pitch_delta if trip.pitch_f is None else \
        trip.pitch_f + (trip.pitch_delta - trip.pitch_f) * RAMP_PITCH_ALPHA
    trip.p_abs = abs(trip.pitch_f)
    trip.ramp_log.append((trip.traveled_cm, trip.pitch_f))
    if trip.ramp_state is None:
        # Каждый всплеск на ступеньке/лежачем - это лишние тики энкодера,
        # цель клетки отодвигаем (см. BUMP_EXTRA_MM). Начало горки сюда
        # тоже попадает, но там цель всё равно пересчитается от гребня.
        if not trip.bump_on and trip.p_abs >= config.OBSTACLE_PITCH_DEG:
            trip.bump_on = True
            add = min(BUMP_EXTRA_MM / 10.0, BUMP_EXTRA_MAX_MM / 10.0 - trip.bump_extra_cm)
            if add > 0:
                trip.bump_extra_cm += add
                trip.boundary_cm += add
        elif trip.bump_on and trip.p_abs < config.OBSTACLE_REARM_DEG:
            trip.bump_on = False
        if trip.p_abs >= config.RAMP_MAP_DEG:
            trip.ramp_state = 'cand'
            trip.ramp_sign = 1 if trip.pitch_f > 0 else -1
            trip.ramp_cand_cm = trip.traveled_cm
            trip.ramp_hold_s = 0.0
            # Цель клетки до горки - вернуть её, если "горка" окажется
            # лестницей (RAMP_MIN_SLOPE_DEG, RAMP_MIN_HOLD_S).
            trip.ramp_saved = (trip.boundary_cm, trip.stopping, trip.pending, trip.reason)
    elif trip.ramp_state == 'cand':
        if trip.p_abs < config.RAMP_MAP_DEG - RAMP_MAP_HYST_DEG or trip.pitch_f * trip.ramp_sign < 0:
            print(f"\n    не горка: {trip.pitch_f:+.1f}°, наклон держался "
                  f"{(trip.traveled_cm - trip.ramp_cand_cm)*10:.0f}мм")
            trip.ramp_state = None
        elif trip.traveled_cm - trip.ramp_cand_cm >= RAMP_CONFIRM_CM:
            trip.ramp_state = 'on'
            trip.ramp_cells0 = trip.cells
            trip.stopping = trip.pending = False
            print(f"\n    ГОРКА {'ВВЕРХ' if trip.ramp_sign > 0 else 'ВНИЗ'} "
                  f"({trip.pitch_f:+.1f}°) - еду без остановок до гребня")
    elif trip.ramp_state == 'on':
        if trip.p_abs < config.RAMP_FLAT_DEG:
            if trip.ramp_flat_cm is None:
                trip.ramp_flat_cm = trip.traveled_cm
            elif trip.traveled_cm - trip.ramp_flat_cm >= RAMP_FLAT_CM:
                info = ramps._ramp_measure(trip.ramp_log, trip.ramp_sign)
                trip.f_anchor = None      # всё, что лидар видел на склоне, - мусор
                if not trip.expect_ramp and (info["slope"] < RAMP_MIN_SLOPE_DEG
                        or trip.ramp_hold_s < RAMP_MIN_HOLD_S):
                    # Лестница внутри клетки. Цель - прежняя; если уже
                    # проскочили её, остановка сразу, доводка по стене.
                    print(f"    не горка: уклон {info['slope']:.1f}°, "
                          f"круто {trip.ramp_hold_s:.2f}с - "
                          f"лестница/препятствие")
                    trip.boundary_cm, trip.stopping, trip.pending, trip.reason = trip.ramp_saved
                    trip.ramp_state = None
                    trip.ramp_rejected = True
                    trip.ramp_flat_cm = None
                    trip.ramp_log = []
                else:
                    state.last_ramp = info
                    # Центр первой клетки - полклетки за гребнем, но не
                    # ближе RAMP_EXIT_MIN_MM от момента, когда стало ровно.
                    trip.boundary_cm = max(info["crest_cm"] + trip.cell_cm * 0.5,
                                      trip.ramp_flat_cm + RAMP_EXIT_MIN_MM / 10.0)
                    trip.ramp_state = 'exit'
                    trip.stopping, trip.pending, trip.reason = True, False, "ramp"
                    print(f"    съехали: до центра клетки "
                          f"{(trip.boundary_cm - trip.traveled_cm)*10:.0f}мм")
        else:
            trip.ramp_flat_cm = None
    trip.on_slope_now = trip.ramp_state in ('cand', 'on')
    if trip.on_slope_now and trip.p_abs >= RAMP_MIN_SLOPE_DEG:
        trip.ramp_hold_s += trip.loop_dt
    if trip.on_slope_now:
        # Подъём законно длится дольше CELL_TIMEOUT_S - пока одометрия
        # растёт, бюджет клетки не тратится (залипание ловит stall выше).
        trip.t_cell = max(trip.t_cell, trip.last_progress_t)


def update_assist(trip):
    """Ассист моментом на препятствии и горке."""
    # ── Ассист моментом: препятствие и горка ──────────────
    # Автомат состояний вместо прежней «одна попытка на событие»:
    #   горизонт  -> всё сбрасывается, попытки взводятся заново;
    #   pitch>=3° -> короткий сильный импульс ('kick'), и так до
    #                OBSTACLE_KICKS_MAX раз, причём счётчик обнуляет
    #                любое реальное продвижение;
    #   pitch>=10° дольше RAMP_CONFIRM_S -> это горка ('ramp'): ток ниже,
    #                держится не по общему времени, а пока одометрия
    #                растёт (RAMP_STALL_S).
    # Во время торможения у границы клетки момент не подаём (ток - это не
    # скорость, тормозной профиль им не отработать), но событие не
    # закрываем: перевзвод по-прежнему только по возврату в горизонт.
    # "Торможение" - это именно фаза спада скорости (braking), а не
    # "остановка запланирована": в one_cell она запланирована с места.
    #
    # Импульс - только если режим скорости не справляется (v_meas заметно
    # ниже заданного, см. OBSTACLE_SLOW_FRAC), и снимается сразу, как
    # робот разогнался выше потолка: значит, препятствие уже позади.
    trip.pitch_up = trip.pitch_delta
    if trip.pitch_up < KICK_PITCH_DEG:
        trip.kick_tilt_t0 = None
    elif trip.kick_tilt_t0 is None:
        trip.kick_tilt_t0 = trip.now_m
    nose_held_up = trip.kick_tilt_t0 is not None and trip.now_m - trip.kick_tilt_t0 >= KICK_HOLD_S

    if trip.pitch_up < config.OBSTACLE_REARM_DEG:
        trip.assist_mode = None
        trip.assist_kicks = 0
        trip.assist_gap_until = 0.0
        trip.ramp_announced = False
    elif trip.braking:
        trip.assist_mode = None
    else:
        if trip.assist_mode is not None:
            cap = config.RAMP_CAP_CMS if trip.assist_mode == 'ramp' else config.OBSTACLE_CAP_CMS
            if trip.v_meas > cap * 1.15:
                print(f"\n    ассист снят: {trip.v_meas:.0f}см/с > потолка {cap:.0f} - "
                      f"препятствие позади")
                trip.assist_mode = None
                trip.assist_kicks = 0
                trip.assist_gap_until = trip.now_m + OBSTACLE_KICK_GAP_S
                trip.ramp_announced = False

        if (trip.assist_mode is None and nose_held_up
                and trip.v_meas < trip.fwd * OBSTACLE_SLOW_FRAC
                and trip.now_m >= trip.assist_gap_until
                and trip.assist_kicks < OBSTACLE_KICKS_MAX):
            trip.assist_mode = 'kick'
            trip.assist_kicks += 1
            trip.assist_t0 = trip.now_m
            if trip.assist_kicks == 1:
                trip.assist_total_t0 = trip.now_m
            trip.assist_progress_cm, trip.assist_progress_t = trip.traveled_cm, trip.now_m
            print(f"\n    препятствие #{trip.assist_kicks}: pitch={trip.pitch_up:+.1f}°, "
                  f"v={trip.v_meas:.0f}/{trip.fwd:.0f}см/с, момент {config.OBSTACLE_TORQUE_A:.1f}A "
                  f"(потолок {config.OBSTACLE_CAP_CMS:.0f}см/с)")

        if trip.assist_mode == 'kick':
            if (trip.pitch_up >= config.RAMP_PITCH_DEG
                    and trip.now_m - trip.assist_t0 >= RAMP_CONFIRM_S):
                trip.assist_mode = 'ramp'
                trip.assist_t0 = trip.now_m
                trip.assist_progress_cm, trip.assist_progress_t = trip.traveled_cm, trip.now_m
            elif trip.now_m - trip.assist_t0 > OBSTACLE_TORQUE_MAX_S:
                trip.assist_mode = None
                trip.assist_gap_until = trip.now_m + OBSTACLE_KICK_GAP_S
                if trip.traveled_cm - trip.assist_progress_cm >= OBSTACLE_PROGRESS_CM:
                    trip.assist_kicks = 0     # сдвинулись - лимит попыток сбрасываем

        elif trip.assist_mode == 'ramp':
            if trip.traveled_cm >= trip.assist_progress_cm + config.ODO_PROGRESS_CM:
                trip.assist_progress_cm, trip.assist_progress_t = trip.traveled_cm, trip.now_m
                trip.assist_kicks = 0
            if (trip.now_m - trip.assist_progress_t > RAMP_STALL_S
                    or trip.now_m - trip.assist_total_t0 > RAMP_TOTAL_MAX_S):
                # Встали на подъёме: момент снимаем, но следующий 'kick'
                # (он сильнее) сам попробует сдвинуть робота.
                trip.assist_mode = None
                trip.assist_gap_until = trip.now_m + OBSTACLE_KICK_GAP_S
                trip.ramp_announced = False
                print("\n    горка: прогресса нет - снимаю момент")

    if trip.assist_mode is not None:
        trip.assist_extra_s = max(trip.assist_extra_s, trip.now_m - trip.assist_total_t0)
