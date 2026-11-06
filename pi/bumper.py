"""Бампер: концевики, что делать при ударе на ходу и после остановки.

Всё про бампер в одном файле:
  - get_bumper(), bumper_side() - прочитать концевики;
  - check_bumper_while_driving() - шаг поездки drive_cells: удар останавливает
    поездку, причина 'bumper_both' / 'bumper_left' / 'bumper_right' /
    'bumper_entered';
  - back_off_from_wall(), dodge_side_hit() - что делает обход лабиринта и
    возврат на старт (explore.py) после такой остановки.
'bumper_entered' (клетка засчитана, только отпустить концевики) доделывает
сама поездка: driving.finish_trip() отъезжает на BUMPER_ENTERED_BACKOFF_MM.
"""
from __future__ import annotations

import time

import state
import driving
import esp_link
import turning


# На сколько откатиться после удара бампером. Вплотную к стене лидар её не
# видит вовсе - она ближе собственного габарита робота и отсекается фильтром
# в walls.py, - поэтому робот считал путь свободным и бил в стену снова и
# снова. Откат возвращает стену в поле зрения.
BUMPER_BACKOFF_MM = 70.0

# Откат при ОДНОСТОРОННЕМ ударе (bumper_side): здесь это не "стена вплотную
# и её не видно", а просто освободить зацепленный уголком корпус после
# поворота на 45° - большого хода не нужно, 70мм в этом манёвре оказалось
# заметно больше, чем требуется. 24.10: 30 давало в сторону всего ~21мм, и
# робот снова цеплял тот же угол - 55мм: ~39мм в сторону от препятствия.
BUMPER_SIDE_BACKOFF_MM = 55.0

# Поворот при одностороннем ударе: на столько градусов В СТОРОНУ ОТ задетого
# бока, потом откат и возврат курса.
BUMPER_DODGE_DEG = 45

# Сколько раз подряд асимметричный бампер в одном направлении пробуем
# объехать маневром, прежде чем сдаться и считать ребро стеной.
BUMPER_SIDE_RETRIES = 3

# Откат, когда упёрлись бампером, уже заехав в клетку (bumper_entered):
# клетка засчитана, перед роботу не нужен - только отпустить концевики и
# не задеть препятствие на повороте.
BUMPER_ENTERED_BACKOFF_MM = 25.0
# Засчитываем клетку, только если после отката до её центра не больше
# 100 мм (центр робота на 200 мм за центром прежней).
BUMPER_ENTER_MM = 200.0

# Концевики (CLICKER_1=левый, CLICKER_2=правый, см. buttons.h) стоят по бокам
# переднего бампера и каждый срабатывает начиная примерно с 1см от центра и
# до края корпуса - то есть удар точно по центру нередко давит ОБА сразу, а
# смещённый в сторону - только один. Окно ниже отличает эти два случая: если
# второй концевик подтягивается следом за первым почти сразу, это тот же
# центральный удар (дребезг/микроразница контактов), а не однобокий.
BUMPER_CONFIRM_S = 0.05


def get_bumper():
    """True - нажат хотя бы один концевик."""
    with state.lock:
        return state.telem["clicker1"] or state.telem["clicker2"]


def both_pressed():
    """True - нажаты оба концевика: упёрлись серединой бампера (стена прямо
    впереди). Один концевик - задели угол или что-то сбоку."""
    with state.lock:
        return state.telem["clicker1"] and state.telem["clicker2"]


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
    t0 = time.monotonic()
    while time.monotonic() - t0 < BUMPER_CONFIRM_S:
        with state.lock:
            c1, c2 = state.telem["clicker1"], state.telem["clicker2"]
        if c1 and c2:
            return "both"
        time.sleep(0.005)
    return first


def check_bumper_while_driving(trip):
    """Шаг поездки: нажат бампер - остановить поездку и решить, засчитать ли
    клетку."""
    if not get_bumper():
        return None
    # Stop before the 50 ms side-classification window, even on old firmware.
    esp_link.brake()
    side = bumper_side()
    if side == "left":
        trip.reason = "bumper_left"
    elif side == "right":
        trip.reason = "bumper_right"
    else:
        trip.reason = "bumper_both"
    print(f"\n    БАМПЕР ({side}) на {trip.traveled_cm*10:.0f}мм")
    # Упёрлись, заехав в следующую клетку, и после отката останемся
    # близко к её центру - значит, мы в ней (заезд 17.10 20:40: стена
    # в 146мм после отката, а карта осталась в прежней клетке и
    # потеряла по оси клетку).
    if trip.reason == "bumper_both":
        if trip.wall_forced:
            from_cm = trip.forced_from_cm
        else:
            from_cm = trip.boundary_cm - trip.cell_cm
        past_mm = (trip.traveled_cm - from_cm) * 10.0 - BUMPER_ENTERED_BACKOFF_MM
        if trip.ramp_state is None and past_mm >= BUMPER_ENTER_MM:
            trip.cells += 1
            trip.reason = "bumper_entered"
            print(f"    уже в следующей клетке ({past_mm:.0f}мм за центром "
                  f"после отката) - засчитываю")
    return "stop"


def back_off_from_wall():
    """Удар обоими концевиками ('bumper_both'): откат назад по курсу сетки.

    Пока робот прижат к стене, лидар её не видит - следующий же замер сказал
    бы "впереди свободно". Откат возвращает стену в поле зрения. Ребро в
    карте закрывает вызывающий."""
    print(f"    откат {BUMPER_BACKOFF_MM:.0f}мм - вплотную стену не видно")
    driving.creep_mm(-BUMPER_BACKOFF_MM, turning.snap_target())


def dodge_side_hit(reason):
    """Удар одним концевиком ('bumper_left' / 'bumper_right'): задели что-то
    одним боком, не стеной по центру. Поворот на BUMPER_DODGE_DEG В СТОРОНУ ОТ
    задетого бока, откат, возврат на исходный курс. Знак подобран
    эмпирически (как TURN_SIGN) - по факту "слева" оказалось направо.
    Пробовать ли ребро ещё раз, решает вызывающий."""
    base_yaw = turning.snap_target()
    if reason == "bumper_left":
        away_deg = -BUMPER_DODGE_DEG
        side_ru = "слева"
    else:
        away_deg = BUMPER_DODGE_DEG
        side_ru = "справа"
    print(f"    асимметричный бампер ({side_ru}) - "
          f"поворот {away_deg:+.0f}°, откат {BUMPER_SIDE_BACKOFF_MM:.0f}мм, возврат курса")
    turning.turn(away_deg)
    if not esp_link.get_esp_paused():
        driving.creep_mm(-BUMPER_SIDE_BACKOFF_MM, esp_link.get_yaw())
    if not esp_link.get_esp_paused():
        turning.turn_to(base_yaw)
