"""Старт и восстановление: кнопки, снятие с поля, чекпоинт, ожидание моторов."""
from __future__ import annotations

import time

import RPi.GPIO as GPIO

import config
import state
import dash_live
import esp_link
import lidar_walls
import maze_map
import ramps
import turning


# Кнопка старта на плате RPI_BTNS (SW1, подтянута к +3.3V через R11=10K,
# нажатие тянет в GND через 750R+0.1мкФ RC-фильтр от дребезга) - см. схему.
# Активный уровень LOW.
START_BTN_GPIO = 25
# Вторая кнопка - та же, что у tile_calib.py (к GND, подтяжка внутренняя).
# Стартуют/перезапускают обе, а какая нажата - задаёт приоритет стороны:
# GPIO27 - левая, GPIO25 - правая (см. _side_pref).
LEFT_BTN_GPIO = 27

# Обратно на поле: столько робот должен простоять ровно и неподвижно, прежде
# чем мы вообще начнём ждать кнопку.
LIFT_STEADY_MS2 = 0.30
LIFT_SETTLE_S = 1.0
LIFT_SETTLE_ROLL_DEG = 10.0
LIFT_SETTLE_RATE_DPS = 15.0


def wait_start_button():
    """Ждёт нажатия одной из двух кнопок старта (активный уровень LOW):
    GPIO27 - приоритет левой руки, GPIO25 - правой. Выставляет _side_pref.
    Аппаратный RC-фильтр (750R+0.1мкФ) на GPIO25 уже гасит дребезг, здесь
    только подтверждение вторым чтением и ожидание отпускания, чтобы одно
    нажатие не срабатывало дважды."""
    pins = {START_BTN_GPIO: "right", LEFT_BTN_GPIO: "left"}
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(START_BTN_GPIO, GPIO.IN)
    GPIO.setup(LEFT_BTN_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print(f"Нажмите GPIO{LEFT_BTN_GPIO} (левая рука) или "
          f"GPIO{START_BTN_GPIO} (правая рука) для старта...")
    pin = None
    while pin is None:
        for p in pins:
            if GPIO.input(p) == GPIO.LOW:
                time.sleep(0.02)
                if GPIO.input(p) == GPIO.LOW:
                    pin = p
                    break
        time.sleep(0.01)
    while GPIO.input(pin) == GPIO.LOW:
        time.sleep(0.01)
    state.side_pref = pins[pin]
    print(f"  приоритет: {'ЛЕВАЯ' if state.side_pref == 'left' else 'ПРАВАЯ'} рука "
          f"(GPIO{pin})")


def handle_lift(resume_hdg):
    """Робота сняли с поля. Ждём, пока его поставят ровно и нажмут кнопку,
    и восстанавливаем систему отсчёта. Сейчас не используется (сторож
    подъёма отключён) - перезапуск идёт по BTN_1, см. restart_from_checkpoint.

    resume_hdg - курс (0=N,1=E,2=S,3=W) ЧЕКПОИНТА, то есть направление, в
    котором робота надо поставить. Знать его обязательно: судья ставит робота
    как придётся, а вся карта (клетки, дедупликация жертв в VictimDetector)
    живёт в системе, заданной YAW_ORIGIN. Поэтому YAW_ORIGIN пересчитывается
    ОТ ФАКТИЧЕСКОГО курса так, чтобы текущее направление означало именно
    resume_hdg - заодно это поглощает неточность установки рукой, ведь
    snap_target() всё равно округляет к сетке 90°.
    """
    print("\n  ✋ ТРЕБУЕТСЯ ВОССТАНОВЛЕНИЕ ПОЗИЦИИ - моторы выключены")
    esp_link.stop()
    dash_live.leds_lifted(True)
    print(f"  Поставьте робота на чекпоинт курсом {config._DIR_NAME[resume_hdg]} "
          f"и нажмите GPIO{LEFT_BTN_GPIO} (левая) или GPIO{START_BTN_GPIO} (правая)")

    # Сначала - ровно и неподвижно: кнопку, нажатую пока робот ещё в руках,
    # принимать нельзя, иначе YAW_ORIGIN зафиксируется по курсу в воздухе.
    # Лента всё это время горит красным и гаснет ровно в тот момент, когда
    # датчик цвета снова видит плитку - то есть когда робота поставили, а не
    # когда нажали кнопку.
    steady = 0.0
    on_floor = False
    last = time.monotonic()
    while True:
        time.sleep(config.LIFT_POLL_S)
        now = time.monotonic()
        dt, last = now - last, now

        r, g, b, c = esp_link.get_rgbc()
        floor_now = c >= config.TILE_MIN_C and (r - b) / c <= config.LIFT_AIR_RB
        if floor_now != on_floor:
            on_floor = floor_now
            dash_live.leds_lifted(not on_floor)
            if on_floor:
                print("  Робот на полу - лента погашена")
        # На полу |a| не превышает 0.06, а сразу после установки, пока
        # человек ещё рядом, доходит до 0.25 - LIFT_STEADY_MS2 между ними.
        level = (esp_link.telem_fresh()
                 and esp_link.gyro_fresh()
                 and abs(esp_link.get_roll() - state.roll_level) < LIFT_SETTLE_ROLL_DEG
                 and abs(esp_link.get_yaw_rate()) < LIFT_SETTLE_RATE_DPS
                 and esp_link.get_accel() < LIFT_STEADY_MS2)
        steady = steady + dt if level else 0.0
        if steady >= LIFT_SETTLE_S:
            break
    print("  Робот стоит ровно - жду кнопку")
    wait_start_button()

    state.YAW_ORIGIN = (esp_link.get_yaw() - 90.0 * resume_hdg + 360.0) % 360.0
    esp_link.zero_odo()
    time.sleep(0.1)
    # _c_white НЕ пересчитываем: под роботом чекпоинт (зеркальный), а не
    # стартовая белая клетка - калибровка по нему испортила бы порог чёрной.
    state.lifted.clear()
    dash_live.leds_lifted(False)
    print(f"  ▶ Продолжаем, YAW_ORIGIN={state.YAW_ORIGIN:.1f}° "
          f"(текущий yaw={esp_link.get_yaw():.1f}° = {config._DIR_NAME[resume_hdg]})")


# Отклонение от ближайшей стороны сетки больше этого - робот поставлен
# заметно криво, печатаем (курс всё равно выберется, см. detect_heading).
RESTART_SKEW_WARN_DEG = 25.0


def detect_heading(cx, cy):
    """Какой стороной робот стоит в клетке (cx,cy) - 0=N..3=W.

    Гироскоп: BNO085 продолжает считать курс, пока робота несут в руках,
    поэтому угол от YAW_ORIGIN сразу даёт ближайшую из четырёх сторон.
    Лидар: свежий замер стен сверяется с картой этой клетки при каждом из
    четырёх поворотов. Рёбра, закрытые не стеной, а по факту (чёрная
    плитка, бампер - _blocked), не сравниваются: лидар видит там открыто.

    Решение: сторона по гироскопу, если стены с ней не спорят (или ни одна
    сторона не подходит - тогда лидару верить не по чему). Если спорят, а
    другая сторона подходит - берём её, ближайшую к гироскопу: стена по
    лидару медианой из трёх сканов ошибается реже, чем гироскоп на 45°+.

    Возвращает (курс, совпал_ли_с_гироскопом)."""
    off = turning.angle_err(esp_link.get_yaw(), state.YAW_ORIGIN)
    k = int(round(off / 90.0))
    h_gyro = k % 4
    skew = off - 90.0 * k

    lidar_walls.wait_scan(2)
    wr = maze_map._walls_rel_map(lidar_walls.measure_walls(config.WALL_VOTE_SCANS, fresh=True))
    rel = [wr["front"], wr["right"], wr["back"], wr["left"]]
    known = state.maze.get((cx, cy))

    def mismatches(h):
        bad = n = 0
        for i in range(4):
            d = (h + i) % 4
            if known is None or known[d] is None or maze_map._next_cell(cx, cy, d) in state.blocked:
                continue
            n += 1
            bad += known[d] != rel[i]
        return bad, n

    score = {h: mismatches(h) for h in range(4)}
    fits = [h for h in range(4) if score[h][0] == 0 and score[h][1] > 0]
    circ = lambda h: min((h - h_gyro) % 4, (h_gyro - h) % 4)

    seen = "".join("FRBL"[i] if rel[i] else "." for i in range(4))
    print(f"  курс по гироскопу: {config._DIR_NAME[h_gyro]} (перекос {skew:+.0f}°), "
          f"стены F R B L: {seen}, подходят: "
          f"{','.join(config._DIR_NAME[h] for h in fits) or 'ни одна'}")
    if abs(skew) > RESTART_SKEW_WARN_DEG:
        print("  [WARN] робот стоит заметно криво относительно сетки")

    if h_gyro in fits or not fits:
        if not fits:
            print("  [WARN] стены не сходятся с картой ни при одном курсе - "
                  "верю гироскопу")
        return h_gyro, True
    h = min(fits, key=circ)
    print(f"  [WARN] по стенам {config._DIR_NAME[h]}, а гироскоп говорил "
          f"{config._DIR_NAME[h_gyro]} - верю стенам")
    return h, False


def restart_from_checkpoint(checkpoint):
    """BTN_1 (GPIO47) на ESP: lack of progress, робота переносят на последний
    чекпоинт. ESP уже заглушил моторы сам; здесь только ждём GPIO25 на
    малине - никаких автоматических проверок "поставили ли ровно" (как в
    handle_lift): они срабатывали плохо, кнопку жмёт человек, когда робот
    стоит. Карта сохраняется целиком.

    Ставить можно любой стороной - сторону определяет detect_heading()
    (гироскоп до ближайших 90° плюс сверка стен с картой). А сетку поля
    после этого ВСЕГДА строим заново от того, как робот стоит сейчас:
    ставят его ровно, а гироскоп за время в руках уплывает на 5-10°, и с
    его сеткой робот потом ехал бы криво."""
    cx, cy, _ = checkpoint
    esp_link.stop()
    print(f"\n  ✋ ПЕРЕЗАПУСК С ЧЕКПОИНТА - моторы выключены.\n"
          f"  Поставьте робота на чекпоинт ({cx},{cy}, этаж {ramps.level_of(cy)}) "
          f"любой стороной и нажмите GPIO{LEFT_BTN_GPIO} (левая рука) "
          f"или GPIO{START_BTN_GPIO} (правая рука)")
    dash_live.leds_sleep(True)
    wait_start_button()
    dash_live.leds_sleep(False)
    time.sleep(config.RESTART_SETTLE_S)
    hdg, _ = detect_heading(cx, cy)
    old = state.YAW_ORIGIN
    state.YAW_ORIGIN = (esp_link.get_yaw() - 90.0 * hdg + 360.0) % 360.0
    print(f"  сетка поля сдвинута на {turning.angle_err(state.YAW_ORIGIN, old):+.1f}°")
    esp_link.zero_odo()
    time.sleep(0.1)
    esp_link.resume_esp()
    print(f"  ▶ Продолжаем с ({cx},{cy}) курс={config._DIR_NAME[hdg]}, "
          f"YAW_ORIGIN={state.YAW_ORIGIN:.1f}°")
    return cx, cy, hdg


LIDAR_STALE_S = 1.0     # нет оборота дольше - лидар молчит


def _lidar_fresh():
    with state.lock:
        scan_t = state.telem.get("scan_t")
    return scan_t is not None and time.monotonic() - scan_t < LIDAR_STALE_S


def wait_for_robot(cx, cy, hdg, checkpoint, phase):
    """Нет связи с ESP или моторы молчат (энкодеры не отвечают по CAN) - это
    не конец заезда: моторы просто выключили (или отошёл провод), их включат.
    Ждём сколько угодно, на пульте - фаза wait_link / wait_motors. Вернулось -
    фаза снова phase. True - ждали (вызывающему стоит заново оценить, где он)."""
    def why():
        return ("wait_link" if not esp_link.telem_fresh() else
                "wait_gyro" if not esp_link.gyro_fresh() else
                "wait_motors" if not esp_link.encoders_online() else
                "wait_lidar" if not _lidar_fresh() else None)
    w = why()
    if w is None:
        return False
    t0 = time.monotonic()
    shown = None
    while w is not None:
        if w != shown:
            shown = w
            print("  [WAIT] " + ("нет связи с ESP" if w == "wait_link" else
                                 "нет свежего курса BNO085" if w == "wait_gyro" else
                                 "лидар молчит" if w == "wait_lidar" else
                                 "моторы не отвечают (выключены?)") + " - жду, заезд продолжится")
            dash_live._write_state(cx, cy, hdg, checkpoint, w)
        time.sleep(0.2)
        w = why()
    print(f"  связь, датчики и моторы снова в порядке (ждали {time.monotonic() - t0:.1f}с)")
    time.sleep(0.5)          # пакеты и одометрия успокоились
    dash_live._write_state(cx, cy, hdg, checkpoint, phase)
    return True
