"""Поездка drive_cells: объект trip со всем её состоянием и подготовка к старту."""
from __future__ import annotations

import time
from collections import deque

import corridor
import watch_logic
from motion_guard import SlipGuard
from motion_estimate import ProjectedOdometry
from continuous import PassageLedger, RangeFusion
from walls import MASKS as _WALLS_MASKS

import config
import state
import esp_link
import turning


# ── Скорость ─────────────────────────────────────────────────
# На предыдущем роботе рабочая MOVE_MAX_CMS была 65 (81 в "быстром" режиме).
# Разгон - см. RAMP_UP_S ниже, он по времени. Отдельной константы тормозного
# пути больше нет: тормозить раньше, чем принято решение об остановке, всё
# равно нельзя, поэтому тормозной путь равен DECIDE_LEAD_MM.
#
# Было 55. Потолок здесь задавал не мотор, а DECIDE_LEAD_MM: тормозить можно
# было только после решения за 110мм до центра. Но робот и так встаёт на
# КАЖДОЙ клетке (drive_cells(1)), решать там нечего - и теперь при
# max_cells=1 торможение запланировано с самого старта, профиль сам начинает
# тормозить, когда пора (см. drive_cells). Для непрерывного проезда по-прежнему
# действует старый предел - MOVE_MAX_NONSTOP_CMS.
#
# Пробовали 75 (при BRAKE_DECEL_CMS2=350) - робот стал заметно переезжать
# центр клетки, а переезд портит и замер боковых стен в новой клетке. Точность
# важнее: вернули проверенные 55/300, выигрыш от торможения "с места" остался.
# 24.10: 40 - заднее правое колесо не крутится быстрее ~30 см/с (предел в
# самом моторе), на 69 правая сторона отставала и робота уводило вправо.
# Настроят мотор в LK Motor Tool - можно вернуть 55-69.
# 28.10: в моторе стоял Max Speed 400°/с (= 30 см/с), сняли - 55/55 в воздухе
# дало L=R. Едем 60.
MOVE_MAX_CMS = 60.0
MOVE_MAX_NONSTOP_CMS = 60.0

# За сколько до центра клетки принимается решение "поворачивать или ехать
# дальше". Раньше нельзя: боковое окно замера в walls.py берёт точки со
# смещением вперёд от -SIDE_BACK_LIMIT до +SIDE_FWD_LIMIT, и оно должно целиком
# лежать уже в НОВОЙ клетке. Позже нельзя: не останется дистанции на
# торможение (при 55см/с нужно ~50мм). С SIDE_BACK_LIMIT=10 окно здесь
# начинается в 120мм от центра при границе клетки в 150мм - запас 30мм.
DECIDE_LEAD_MM = 110.0
# Маски лидара в виде (центр, полуширина) - что corridor.visible_steps
# считает невидимым (стойки, ручки, леска, зад).
_LIDAR_MASKS = tuple((m["center"], m["hw"]) for m in _WALLS_MASKS)


class Trip:
    """Всё, что поездка _drive_cells помнит между тактами. Поля заводит
    start_trip(): там же комментарий к каждому."""


def _pitch_zero():
    """Ноль наклона поездки - наклон сейчас. Но если робот стоит перекошенным
    (доводка задом заехала колёсами на препятствие), это не ноль: заезд
    24.10, ноль взят при +7.6°, съезд на пол дал -8.5° - "горка вниз",
    клетка не засчиталась, карта сдвинулась. Тогда - ровный пол со старта."""
    pitch = esp_link.get_pitch()
    if abs(pitch - state.pitch_level) > config.OBSTACLE_PITCH_DEG:
        print(f"    стою с наклоном {pitch - state.pitch_level:+.1f}° "
              f"(препятствие под колёсами) - ноль наклона со старта")
        return state.pitch_level
    return pitch


def _start_offset_cm(target_yaw):
    """Доводка после поворота, оставленная поездке (turning.align_at_wall,
    move=False): робот не в центре клетки - цель ближе или дальше на столько
    же, без отдельного подъезда. Только тем же курсом; берётся один раз."""
    off, state.start_offset = state.start_offset, None
    if off is None or abs(turning.angle_err(off[1], target_yaw)) > 5.0:
        return 0.0
    return off[0] / 10.0


def _zero_odometry():
    """Ноль одометрии с подтверждением ESP (z_cnt сменился). Раньше - sleep(0.04)
    вслепую: пакет с одометрией ДО "Z" брался за базу, и защита от скачка
    (только >=100см) пропускала лишнюю клетку. Не подтвердил - это не конец
    заезда: тормоз и повтор "Z", пока ESP не ответит."""
    warned = False
    while True:
        z0 = esp_link.get_z_cnt()
        esp_link.zero_odo()
        if esp_link.wait_odo_zeroed(z0):
            return
        esp_link.brake()
        if not warned:
            print("    [WAIT] ESP не подтвердил сброс одометрии - повторяю 'Z'")
            warned = True
        time.sleep(0.2)


def start_trip(max_cells, expect_ramp, cell, hdg, plan):
    """Подготовка поездки: ноль одометрии, курс, все поля trip с начальными значениями."""
    trip = Trip()
    trip.max_cells = max_cells
    trip.expect_ramp = expect_ramp
    trip.cell = cell
    trip.hdg = hdg
    trip.plan = plan
    _zero_odometry()

    trip.target_yaw = turning.snap_target()
    # Ноль наклона - стоя в начале поездки: робот всегда выезжает из центра
    # клетки на ровном полу (на горке он не останавливается). BNO после
    # поворотов показывает на ровном полу до +9° от pitch0 стартовой клетки
    # и держит это секунды (заезд 17.10 15:31), а за одну поездку уходит не
    # больше чем на 5° - от pitch0 это была "горка" на ровном полу.
    trip.pitch_start = _pitch_zero()

    trip.last_l, trip.last_r = esp_link.get_odo()
    initial_motion = esp_link.motion_snapshot()
    trip.odometry = ProjectedOdometry(initial_motion, trip.target_yaw)
    trip.last_l, trip.last_r = initial_motion.left, initial_motion.right
    trip.traveled_cm = 0.0
    trip.slip_guard = SlipGuard()
    trip.range_fusion = RangeFusion()
    trip.passages = PassageLedger()
    trip.perception_cap = float('inf')
    trip.head_off_f = 0.0
    trip.yaw_i = 0.0                # интеграл курса, см/с (drive_motion.steering)
    trip.last_steer = 0.0
    trip.cell_cm = config.CELL_MM / 10.0
    trip.boundary_cm = trip.cell_cm                 # центр ближайшей клетки по одометрии
    trip.boundary_cm += _start_offset_cm(trip.target_yaw)
    state.watched_sides = set()
    trip.watch_boundary = None
    trip.lead_cm = DECIDE_LEAD_MM / 10.0
    trip.fwd = 0.0
    trip.last_progress_cm = 0.0
    trip.last_progress_t = time.monotonic()
    trip.last_status_t = 0.0

    # Состояние ассиста. assist_mode: None | 'kick' | 'ramp'.
    #   assist_t0        - начало ТЕКУЩЕЙ фазы
    #   assist_total_t0  - начало всей попытки (предохранитель RAMP_TOTAL_MAX_S)
    #   assist_kicks     - сколько импульсов подряд без продвижения
    #   assist_progress_* - одометрия и время последнего реального продвижения
    #   assist_extra_s   - на сколько продлён бюджет времени клетки
    trip.assist_mode = None
    trip.assist_t0 = trip.assist_total_t0 = 0.0
    trip.assist_kicks = 0
    trip.assist_gap_until = 0.0
    trip.push_tries = 0             # PUSH_THROUGH_SMALL: сколько раз уже дотолкивали
    trip.assist_progress_cm = 0.0
    trip.assist_progress_t = 0.0
    trip.assist_extra_s = 0.0
    trip.ramp_announced = False
    trip.telem_seq = state.telem_seq

    # Горка для карты (см. RAMP_MAP_DEG):
    #   None   - ровно, или обычное препятствие/ступенька
    #   'cand' - наклон >= RAMP_MAP_DEG, ждём RAMP_CONFIRM_CM хода на нём;
    #            тормозить у границы клетки уже нельзя - вдруг это горка
    #   'on'   - горка: едем без цели по одометрии, передний лидар не слушаем
    #   'exit' - снова ровно, цель - центр первой клетки после гребня
    state.last_ramp = None
    trip.ramp_state = None
    trip.ramp_rejected = False    # горку отменили после езды "до гребня" без счёта клеток
    trip.ramp_sign = 0
    trip.ramp_cand_cm = 0.0
    trip.ramp_flat_cm = None
    trip.ramp_saved = None
    trip.ramp_log = []
    trip.ramp_cells0 = 0         # сколько клеток проехали до начала горки
    # Всплески наклона на ступеньках/лежачих - см. BUMP_EXTRA_MM.
    trip.bump_on = False
    trip.bump_extra_cm = 0.0
    trip.pitch_f = None          # сглаженный pitch_delta, см. RAMP_PITCH_ALPHA

    trip.cells = 0
    # Одна клетка - остановка у первой же границы известна заранее, решать
    # на ходу нечего. Тормозной профиль тогда действует с самого старта и сам
    # начинает тормозить, когда пора, - крейсерская больше не ограничена
    # тормозным путём внутри DECIDE_LEAD_MM (см. MOVE_MAX_CMS).
    one_cell = trip.max_cells <= 1
    trip.pending = not one_cell  # решение по ближайшей границе ещё не принято
    trip.stopping = one_cell     # решили тормозить у этой границы
    trip.reason = "done" if one_cell else "stop"
    trip.v_max = MOVE_MAX_CMS if one_cell else MOVE_MAX_NONSTOP_CMS
    trip.black_back_cm = 0.0     # сколько отъехать назад, если поймали чёрное

    # Скорость по одометрии (для ассиста): окно SPEED_WIN_S, см. v_meas.
    trip.odo_hist = deque()
    trip.v_meas = 0.0
    # Ограничение хода на наклоне, с плавным (BRAKE_DECEL_CMS2) изменением,
    # чтобы сам переход на него не был рывком.
    trip.tilt_lim = trip.v_max
    trip.last_loop_t = time.monotonic()

    # Опора для оценки дистанции до передней стены между сканами лидара
    trip.f_anchor = None        # мм, последнее показание лидара
    trip.f_anchor_t = 0.0
    trip.f_anchor_cm = 0.0      # одометрия в момент этого показания
    trip.last_seq = state.scan_cnt
    trip.wall_forced = False    # цель уже перенесена к внезапно близкой стене
    trip.forced_from_cm = 0.0   # центр клетки, из которой ехали, когда это случилось

    # Наблюдатель меток на ходу (victim_watch.py): только на ровном и с той
    # стороны, где лидар видит стену. vmark - метка, ради которой тормозим
    # ({"side", "pos_mm", "stop_cm"}); пока она есть, наблюдатель выключен.
    # watcher = None - в этой поездке наблюдателя нет (или он умер).
    trip.watcher = (state.watcher if trip.cell is not None and trip.hdg is not None and not trip.expect_ramp
               else None)
    trip.side_hold = watch_logic.SideHold()
    trip.odo_t = watch_logic.OdoHistory()
    trip.odo_t.add(initial_motion.t, 0.0)
    trip.raw_odo_t = watch_logic.OdoHistory()
    trip.raw_odo_t.add(initial_motion.t, 0.0)
    trip.raw_traveled_cm = 0.0
    trip.yaw_t = watch_logic.OdoHistory()
    trip.yaw_t.add(initial_motion.t, (initial_motion.yaw - trip.target_yaw + 180) % 360 - 180)
    trip.vmark = None

    # Проезд без остановок по плану (flood_fill_run/return_to_start): стены
    # клеток впереди - по точкам лидара на подходе (corridor.py), решение
    # "прямо или стоп" в каждой клетке - plan.decide(). Клетка k - k-я по
    # ходу от исходной (0), N = cells + 1 - следующая, её центр - boundary_cm.
    trip.cell_sides = corridor.CellSides(_LIDAR_MASKS) if trip.plan is not None else None
    trip.scan_mark = state.scan_cnt        # обороты до старта учтены замером стоя
    trip.front_votes = [0, 0]         # передняя стена N по оборотам: [есть, нет]
    trip.tiles = {}                   # k -> цвета плитки под роботом над клеткой k
    trip.cp_done = set()              # клетки, уже отмеченные чекпоинтом на ходу
    trip.side_known = {}              # (k, сторона) -> True/False - для столбиков
    trip.posts_done = set()           # границы, по которым путь уже поправлен
    if trip.plan is not None:
        for side in ("right", "left"):
            trip.side_known[(0, side)] = trip.plan.known_side(0, side)

    print(f"    движение  курс={trip.target_yaw:.0f}°")

    trip.watch_armed = False
    trip.t0 = time.monotonic()
    trip.t_cell = time.monotonic()    # таймаут считаем на клетку, а не на весь прогон
    return trip
