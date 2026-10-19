"""Бампер: концевики и числа отката после удара."""
from __future__ import annotations

import time

import state


# На сколько откатиться после удара бампером. Вплотную к стене лидар её не
# видит вовсе - она ближе собственного габарита робота и отсекается фильтром
# в walls.py, - поэтому робот считал путь свободным и бил в стену снова и
# снова. Откат возвращает стену в поле зрения.
BUMPER_BACKOFF_MM = 70.0

# Откат при ОДНОСТОРОННЕМ ударе (bumper_side): здесь это не "стена вплотную
# и её не видно", а просто освободить зацепленный уголком корпус после
# поворота на 45° - большого хода не нужно, 70мм в этом манёвре оказалось
# заметно больше, чем требуется.
BUMPER_SIDE_BACKOFF_MM = 30.0

# Откат, когда упёрлись бампером, уже заехав в клетку (bumper_entered):
# клетка засчитана, перед роботу не нужен - только отпустить концевики и
# не задеть препятствие на повороте.
BUMPER_ENTERED_BACKOFF_MM = 25.0
# Засчитываем клетку, только если после отката до её центра не больше
# 100 мм (центр робота на 200 мм за центром прежней).
BUMPER_ENTER_MM = 200.0


def get_bumper():
    with state.lock: return state.telem["clicker1"] or state.telem["clicker2"]


# Концевики (CLICKER_1=левый, CLICKER_2=правый, см. buttons.h) стоят по бокам
# переднего бампера и каждый срабатывает начиная примерно с 1см от центра и
# до края корпуса - то есть удар точно по центру нередко давит ОБА сразу, а
# смещённый в сторону - только один. Окно ниже отличает эти два случая: если
# второй концевик подтягивается следом за первым почти сразу, это тот же
# центральный удар (дребезг/микроразница контактов), а не однобокий.
BUMPER_CONFIRM_S = 0.05


def bumper_side():
    """'left' | 'right' - нажат только один концевик (и второй не подтянулся
    за BUMPER_CONFIRM_S); 'both' - нажаты оба (сразу или почти сразу);
    'none' - ни один."""
    with state.lock:
        c1, c2 = state.telem["clicker1"], state.telem["clicker2"]
    if c1 and c2:
        return "both"
    if not c1 and not c2:
        return "none"
    first = "left" if c1 else "right"
    t0 = time.time()
    while time.time() - t0 < BUMPER_CONFIRM_S:
        with state.lock:
            c1, c2 = state.telem["clicker1"], state.telem["clicker2"]
        if c1 and c2:
            return "both"
        time.sleep(0.005)
    return first


def check_bumper_while_driving(trip):
    """Нажат бампер: остановить поездку и решить, засчитать ли клетку."""
    # ── Бампер: концевики спереди ─────────────────────────
    if get_bumper():
        side = bumper_side()
        trip.reason = {"left": "bumper_left", "right": "bumper_right"}.get(side, "bumper_both")
        print(f"\n    БАМПЕР ({side}) на {trip.traveled_cm*10:.0f}мм")
        # Упёрлись, заехав в следующую клетку, и после отката останемся
        # близко к её центру - значит, мы в ней (заезд 17.10 20:40: стена
        # в 146мм после отката, а карта осталась в прежней клетке и
        # потеряла по оси клетку).
        if trip.reason == "bumper_both":
            from_cm = trip.forced_from_cm if trip.wall_forced else trip.boundary_cm - trip.cell_cm
            past_mm = (trip.traveled_cm - from_cm) * 10.0 - BUMPER_ENTERED_BACKOFF_MM
            if trip.ramp_state is None and past_mm >= BUMPER_ENTER_MM:
                trip.cells += 1
                trip.reason = "bumper_entered"
                print(f"    уже в следующей клетке ({past_mm:.0f}мм за центром "
                      f"после отката) - засчитываю")
        return "stop"


# Сколько раз подряд asymметричный бампер в одном направлении пробуем
# объехать маневром, прежде чем сдаться и считать ребро стеной.
BUMPER_SIDE_RETRIES = 3
