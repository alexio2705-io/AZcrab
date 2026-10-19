"""Езда вперёд по клеткам и короткие подъезды (creep_mm)."""
from __future__ import annotations

import math
import sys
import time
from collections import deque

import corridor
import runplan
import watch_logic
from walls import SIDE_THRESH, WALL_THRESH, MASKS as _WALLS_MASKS

import config
import state
import timing
import bumper
import esp_link
import floor_tiles
import lidar_walls
import marks
import ramps
import recovery
import turning


SIDE_CENTER_MM = 140.0

# Слепая зона лидара (walls.py: self_outline_mm + SELF_MARGIN) в окне ±22°
# фронта доходит до ~119мм от центра - точки ближе отбрасываются как шум от
# собственного корпуса, а не потому что стены нет. Прежний порог 122 стоял
# ВНУТРИ этой зоны: к моменту, когда стена подходила настолько близко, лидар
# уже переставал её видеть (f_now становился None), и аварийный стоп не
# успевал сработать по валидному замеру - оставался только бампер. 140 даёт
# ~20мм запаса над слепой зоной и всё ещё ниже штатной точки остановки
# (FRONT_CENTER_MM=145), чтобы не срабатывать ложно на обычных подъездах.
FRONT_EMERGENCY_MM = 140

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
MOVE_MAX_CMS = 55.0
MOVE_MAX_NONSTOP_CMS = 55.0
MOVE_MIN_CMS = 9.0

# Торможение с ПОСТОЯННЫМ замедлением: v = sqrt(2*a*остаток). Раньше скорость
# падала линейно по остатку пути - это размазывало торможение на все 110мм и
# заканчивалось долгим сползанием, отчего казалось, что робот тормозит поздно
# и вяло. При постоянном замедлении робот идёт на крейсерской дольше, а потом
# тормозит коротко и жёстко: с 55см/с при 300см/с² тормозной путь ~50мм, то
# есть вдвое меньше, чем есть в запасе.
# 300 см/с² = ~4000 dps/с - втрое ниже лимита MOTOR_ACCEL_DPS_S на ESP.
BRAKE_DECEL_CMS2 = 300.0
# Торможение к метке, замеченной на ходу (см. drive_cells) - жёстче, чем к
# центру клетки: камера видит всего ±62мм стены, и каждый сантиметр
# тормозного пути - это подъезд назад после остановки. С 55см/с - ~30мм
# вместо ~50. Мотору по силам: MOTOR_ACCEL_DPS_S=11000 на ESP - это ~820см/с².
MARK_BRAKE_DECEL_CMS2 = 500.0

# Разгон - ПО ВРЕМЕНИ, не по пройденному пути. Профиль "по пути" (было
# ACCEL_MM=60) считает скорость от traveled_cm, а сам traveled_cm растёт тем
# быстрее, чем выше уже скорость - положительная обратная связь даёт РАЗГОН
# ПО ЭКСПОНЕНТЕ: требуемый dv/dt на подходе к MOVE_MAX_CMS улетает далеко за
# любое разумное ограничение мотора, мотор не успевает, расхождение копится
# и потом отыгрывается рывком. Разгон по времени (smoothstep) даёт постоянный,
# предсказуемый и заранее известный пик dv/dt - можно держать его безопасно
# ниже MOTOR_ACCEL_DPS_S на ESP.
RAMP_UP_S = 0.25

# Петля управления (команда → мотор → телеметрия обратно) запаздывает, и на
# скорости это превращается в перелёт: одометрия показывает, где робот был
# на эту задержку назад. Поэтому и профиль торможения, и условие остановки
# считаются от остатка, уменьшенного на "скорость × эту задержку" - запас сам
# сжимается по мере замедления.
# Было 0.03 - робот всё равно докатывался по инерции после команды тормоза,
# значит реальная задержка (UART -> CAN -> мотор -> физика) больше заложенной.
# Поднято до эмпирического подбора; если всё ещё доезжает - увеличивать дальше,
# если начнёт недоезжать до стены/центра - уменьшать.
STOP_LEAD_S = 0.05

# Одометрия — единственный безусловный ограничитель 300 мм. Если при поданном ходе
# она не меняется дольше этого времени, дальше ехать нельзя.
ODO_STALL_S = 0.7
ODO_PROGRESS_CM = 0.5
OBSTACLE_REARM_DEG = 1.2
OBSTACLE_TORQUE_A = 2.5
OBSTACLE_STEER_A = 0.35
OBSTACLE_TORQUE_MAX_S = 0.65
OBSTACLE_SLOW_FRAC = 0.6       # импульс, только если едем медленнее этой доли
TILT_CAP_CMS = 35.0            # ход, пока нос задран на препятствии/горке
DESCENT_CAP_CMS = 28.0         # ход, пока нос опущен (съезд с препятствия)
SPEED_WIN_S = 0.06             # окно оценки скорости по одометрии

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

RAMP_PITCH_DEG = 10.0
RAMP_CONFIRM_S = 0.25
RAMP_TORQUE_A = 1.9
RAMP_STEER_A = 0.55
# Было RAMP_TORQUE_MAX_S=5с абсолютным потолком на подъём. Потолок по ОБЩЕМУ
# времени здесь неправильный: горка длиной 2.5м на крейсерской - это уже 4.5с,
# а если робот на ней замедлится (а он замедлится), помощь исчезнет до вершины
# и не вернётся, потому что перевзвод ждёт горизонта. Ограничиваем не подъём,
# а ОТСУТСТВИЕ ПРОГРЕССА: пока одометрия растёт и нос задран - момент держим.
RAMP_STALL_S = 1.2
RAMP_TOTAL_MAX_S = 20.0        # предохранитель от бесконечной подачи тока
RAMP_WALL_KP_DEG = 0.08
RAMP_WALL_MAX_DEG = 5.0
RAMP_CAP_CMS = 40.0            # потолок скорости колеса на горке под моментом

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
RAMP_MAP_HYST_DEG = 3.0        # кандидат сбрасывается ниже RAMP_MAP_DEG - этого
RAMP_CONFIRM_CM = 8.0
RAMP_FLAT_CM = 2.0
RAMP_EXIT_MIN_MM = 50.0
# Горка - это целые плитки, то есть не короче ~30см по уклону (замер 1
# плитки в симуляции - 31..34см). Крутая лестница (ступеньки 2см через
# 4-5см) по наклону неотличима от горки ~22°, но внутри одной клетки её
# наклонная часть не длиннее ~15см (верхняя ступень >=15см), и замер с
# базой робота даёт до ~24см - по длине её и отсекаем: такой "подъём"
# считается препятствием, цель клетки возвращается обычная.
RAMP_MIN_LEN_CM = 26.0
# Островок, на который ведут горки, не ниже одноплиточной горки ~25°
# (12.7см); лежачий и лестница в 2-3 ступеньки поднимают на 0..6см.
RAMP_MIN_RISE_CM = 8.0
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

# Штатный бюджет времени на клетку. На препятствии он продлевается ровно на
# столько, сколько длится ассист (иначе подъём упирался в таймаут и клетка
# просто терялась: cells не инкрементировался, а карта уезжала молча).
CELL_TIMEOUT_S = 8.0

# За сколько до центра клетки принимается решение "поворачивать или ехать
# дальше". Раньше нельзя: боковое окно замера в walls.py берёт точки со
# смещением вперёд от -SIDE_BACK_LIMIT до +SIDE_FWD_LIMIT, и оно должно целиком
# лежать уже в НОВОЙ клетке. Позже нельзя: не останется дистанции на
# торможение (при 55см/с нужно ~50мм). С SIDE_BACK_LIMIT=10 окно здесь
# начинается в 120мм от центра при границе клетки в 150мм - запас 30мм.
DECIDE_LEAD_MM = 110.0

# Ближе этого расстояния до передней стены остаток пути считаем по лидару,
# а не по одометрии - именно так робот встаёт у стены сразу, без доводки
# после остановки. Дальше - лидару верить рано (можем видеть стену через
# клетку), правит одометрия.
#
# Было 400 - поднято до 600, чтобы лидар подхватывал переднюю стену раньше
# (робот тогда дольше едет по её реальному расстоянию, а не по чистой
# одометрии). Ложное срабатывание через открытый проём в соседнюю клетку не
# страшнее, чем раньше: оценка "по лидару" всё равно принимается только если
# согласна с оценкой "по одометрии" в пределах ALIGN_MAX_MM (см. ниже,
# rem_wall/rem_odo) - несогласная просто игнорируется, и правит одометрия.
FRONT_TRACK_MM = 600.0

# Насколько далеко (по одометрии) можно уехать от последней РЕАЛЬНОЙ точки
# лидара, продолжая ей верить, если текущий скан пуст. Больше - точка
# слишком старая, дальше едем как при полном отсутствии показаний.
ANCHOR_STALE_CM = 15.0

# ── Удержание курса при движении ─────────────────────────────
# Как в микромаусе: демпфирование берём из ИЗМЕРЕННОЙ угловой скорости
# (телеметрия, 1кГц-источник), а не из разности двух соседних показаний угла.
# Разность угла - это шум датчика, поделённый на 10мс, и именно он раньше
# превращался в дрожь руля; измеренная скорость шум не усиливает. Чистое
# демпфирование позволяет держать заметно более высокий KP, не срываясь в
# автоколебания - поэтому KP поднят, как и просили.
GYRO_KP = 4.2          # см/с на градус ошибки курса
GYRO_KD_RATE = 0.07    # см/с на (градус/с) - демпфирование по угловой скорости
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
ALIGN_SPEED_CMS = 18.0    # вдвое быстрее прежней доводки (была ~9 см/с)
# Маски лидара в виде (центр, полуширина) - что corridor.visible_steps
# считает невидимым (стойки, ручки, леска, зад).
_LIDAR_MASKS = tuple((m["center"], m["hw"]) for m in _WALLS_MASKS)


STATUS_PERIOD_S = 0.1     # строка статуса в drive_cells не чаще


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
    z0 = esp_link.get_z_cnt()
    esp_link.zero_odo()
    # Раньше - sleep(0.04) вслепую: пакет с одометрией ДО "Z" брался за базу,
    # и защита от скачка (только >=100см) пропускала лишнюю клетку.
    if not esp_link.wait_odo_zeroed(z0):
        print("    [WARN] ESP не подтвердил 'Z' за 0.2с - база одометрии без гарантии")

    target_yaw = turning.snap_target()
    # Ноль наклона - стоя в начале поездки: робот всегда выезжает из центра
    # клетки на ровном полу (на горке он не останавливается). BNO после
    # поворотов показывает на ровном полу до +9° от pitch0 стартовой клетки
    # и держит это секунды (заезд 17.10 15:31), а за одну поездку уходит не
    # больше чем на 5° - от pitch0 это была "горка" на ровном полу.
    pitch_start = esp_link.get_pitch()

    last_l, last_r = esp_link.get_odo()
    traveled_cm = 0.0
    head_off_f = 0.0
    last_steer = 0.0
    cell_cm = config.CELL_MM / 10.0
    boundary_cm = cell_cm                 # центр ближайшей клетки по одометрии
    lead_cm = DECIDE_LEAD_MM / 10.0
    fwd = 0.0
    last_progress_cm = 0.0
    last_progress_t = time.monotonic()
    last_status_t = 0.0

    # Состояние ассиста. assist_mode: None | 'kick' | 'ramp'.
    #   assist_t0        - начало ТЕКУЩЕЙ фазы
    #   assist_total_t0  - начало всей попытки (предохранитель RAMP_TOTAL_MAX_S)
    #   assist_kicks     - сколько импульсов подряд без продвижения
    #   assist_progress_* - одометрия и время последнего реального продвижения
    #   assist_extra_s   - на сколько продлён бюджет времени клетки
    assist_mode = None
    assist_t0 = assist_total_t0 = 0.0
    assist_kicks = 0
    assist_gap_until = 0.0
    push_tries = 0             # PUSH_THROUGH_SMALL: сколько раз уже дотолкивали
    assist_progress_cm = 0.0
    assist_progress_t = 0.0
    assist_extra_s = 0.0
    ramp_announced = False
    telem_seq = state.telem_seq

    # Горка для карты (см. RAMP_MAP_DEG):
    #   None   - ровно, или обычное препятствие/ступенька
    #   'cand' - наклон >= RAMP_MAP_DEG, ждём RAMP_CONFIRM_CM хода на нём;
    #            тормозить у границы клетки уже нельзя - вдруг это горка
    #   'on'   - горка: едем без цели по одометрии, передний лидар не слушаем
    #   'exit' - снова ровно, цель - центр первой клетки после гребня
    state.last_ramp = None
    ramp_state = None
    ramp_rejected = False    # горку отменили после езды "до гребня" без счёта клеток
    ramp_sign = 0
    ramp_cand_cm = 0.0
    ramp_flat_cm = None
    ramp_saved = None
    ramp_log = []
    ramp_cells0 = 0         # сколько клеток проехали до начала горки
    # Всплески наклона на ступеньках/лежачих - см. BUMP_EXTRA_MM.
    bump_on = False
    bump_extra_cm = 0.0
    pitch_f = None          # сглаженный pitch_delta, см. RAMP_PITCH_ALPHA

    cells = 0
    # Одна клетка - остановка у первой же границы известна заранее, решать
    # на ходу нечего. Тормозной профиль тогда действует с самого старта и сам
    # начинает тормозить, когда пора, - крейсерская больше не ограничена
    # тормозным путём внутри DECIDE_LEAD_MM (см. MOVE_MAX_CMS).
    one_cell = max_cells <= 1
    pending = not one_cell  # решение по ближайшей границе ещё не принято
    stopping = one_cell     # решили тормозить у этой границы
    reason = "done" if one_cell else "stop"
    v_max = MOVE_MAX_CMS if one_cell else MOVE_MAX_NONSTOP_CMS
    black_back_cm = 0.0     # сколько отъехать назад, если поймали чёрное

    # Скорость по одометрии (для ассиста): окно SPEED_WIN_S, см. v_meas.
    odo_hist = deque()
    v_meas = 0.0
    # Ограничение хода на наклоне, с плавным (BRAKE_DECEL_CMS2) изменением,
    # чтобы сам переход на него не был рывком.
    tilt_lim = v_max
    last_loop_t = time.monotonic()

    # Опора для оценки дистанции до передней стены между сканами лидара
    f_anchor = None        # мм, последнее показание лидара
    f_anchor_cm = 0.0      # одометрия в момент этого показания
    last_seq = state.scan_cnt
    wall_forced = False    # цель уже перенесена к внезапно близкой стене
    forced_from_cm = 0.0   # центр клетки, из которой ехали, когда это случилось

    # Наблюдатель меток на ходу (victim_watch.py): только на ровном и с той
    # стороны, где лидар видит стену. vmark - метка, ради которой тормозим
    # ({"side", "pos_mm", "stop_cm"}); пока она есть, наблюдатель выключен.
    # watcher = None - в этой поездке наблюдателя нет (или он умер).
    watcher = (state.watcher if cell is not None and hdg is not None and not expect_ramp
               else None)
    side_hold = watch_logic.SideHold()
    odo_t = watch_logic.OdoHistory()
    vmark = None

    # Проезд без остановок по плану (flood_fill_run/return_to_start): стены
    # клеток впереди - по точкам лидара на подходе (corridor.py), решение
    # "прямо или стоп" в каждой клетке - plan.decide(). Клетка k - k-я по
    # ходу от исходной (0), N = cells + 1 - следующая, её центр - boundary_cm.
    cell_sides = corridor.CellSides(_LIDAR_MASKS) if plan is not None else None
    scan_mark = state.scan_cnt        # обороты до старта учтены замером стоя
    front_votes = [0, 0]         # передняя стена N по оборотам: [есть, нет]
    tiles = {}                   # k -> цвета плитки под роботом над клеткой k
    cp_done = set()              # клетки, уже отмеченные чекпоинтом на ходу
    side_known = {}              # (k, сторона) -> True/False - для столбиков
    posts_done = set()           # границы, по которым путь уже поправлен
    if plan is not None:
        for side in ("right", "left"):
            side_known[(0, side)] = plan.known_side(0, side)

    print(f"    движение  курс={target_yaw:.0f}°")

    watch_armed = False
    t0 = time.monotonic()
    t_cell = time.monotonic()    # таймаут считаем на клетку, а не на весь прогон
    while True:
        if state.lifted.is_set():
            return cells, "lifted"
        if esp_link.get_esp_paused():
            # BTN_1 - перезапуск с чекпоинта, в том числе с горки: робота
            # всё равно переносят руками, недоезженная горка не записывается.
            return cells, "paused"
        if not esp_link.telem_fresh():
            reason = "link_lost"
            print(f"\n    АВАРИЯ: нет телеметрии {esp_link.telem_age():.2f}с — торможу")
            break
        # Бюджет времени клетки продлевается ровно на длительность ассиста:
        # подъём по горке законно занимает секунды, и раньше он упирался в
        # этот таймаут, а клетка при этом молча терялась (cells не рос).
        if time.monotonic() - t_cell > CELL_TIMEOUT_S + assist_extra_s:
            reason = "timeout"
            print("    ТАЙМАУТ!")
            break

        # Наклон нужен уже здесь: по нему решается, каким таймером мерить
        # залипание одометрии - штатным или «мы на препятствии».
        pitch_delta = esp_link.get_pitch() - pitch_start

        # ── Одометрия ─────────────────────────────────────────
        cl, cr = esp_link.get_odo()
        dl, dr = cl - last_l, cr - last_r
        if abs(dl) < 100 and abs(dr) < 100:   # скачок = только что было 'Z'
            traveled_cm += (dl + dr) / 2.0
        last_l, last_r = cl, cr

        now_m = time.monotonic()
        state.tick_ms["вкл" if watch_armed else "выкл"].add((now_m - last_loop_t) * 1000.0)
        loop_dt = min(0.05, now_m - last_loop_t)
        last_loop_t = now_m
        odo_hist.append((now_m, traveled_cm))
        odo_t.add(now_m, traveled_cm)
        while len(odo_hist) > 2 and now_m - odo_hist[1][0] >= SPEED_WIN_S:
            odo_hist.popleft()
        t_a, d_a = odo_hist[0]
        if now_m - t_a > 0.02:
            v_meas = (traveled_cm - d_a) / (now_m - t_a)

        # Без двух энкодеров нельзя гарантировать остановку через 300 мм.
        if not esp_link.encoders_online():
            reason = "odo_fault"
            l_ok, r_ok = esp_link.encoder_sides()
            # Печатаем, КАКАЯ сторона молчит, и сырую одометрию рядом: бит в
            # телеметрии и строка debug_task на ESP считаются из одного и того
            # же isnan(), так что разойтись они не могут - и если ESP при этом
            # показывает живые энкодеры, значит расходятся версии или момент
            # замера, а не сами данные.
            print(f"\n    АВАРИЯ: энкодер не отвечает "
                  f"(левый={'ok' if l_ok else 'НЕТ'}, "
                  f"правый={'ok' if r_ok else 'НЕТ'}, "
                  f"L={cl:.1f} R={cr:.1f}см) — торможу")
            break
        # Нос задран - робот сейчас берёт препятствие, и постоять полсекунды
        # на месте для этого нормально. Раньше здесь работал один общий
        # ODO_STALL_S=0.7, который кончался на 50мс позже единственного
        # импульса момента: «не взял бордюр» = [FATAL] на весь заезд.
        on_obstacle = pitch_delta >= OBSTACLE_REARM_DEG or assist_mode is not None
        stall_limit = ASSIST_STALL_S if on_obstacle else ODO_STALL_S
        if traveled_cm >= last_progress_cm + ODO_PROGRESS_CM:
            last_progress_cm = traveled_cm
            last_progress_t = time.monotonic()
        elif (PUSH_THROUGH_SMALL and on_obstacle and ramp_state != 'on'
              and fwd >= MOVE_MIN_CMS and time.monotonic() - last_progress_t > stall_limit
              and push_tries < PUSH_THROUGH_MAX):
            push_tries += 1
            assist_kicks = 0
            assist_gap_until = 0.0
            last_progress_t = time.monotonic()
            t_cell = max(t_cell, time.monotonic() - CELL_TIMEOUT_S / 2)   # бюджет клетки тоже
            print(f"\n    бугорок не взят с ходу (pitch={pitch_delta:+.1f}°) - "
                  f"толкаю ещё, без отката ({push_tries}/{PUSH_THROUGH_MAX})")
        elif fwd >= MOVE_MIN_CMS and time.monotonic() - last_progress_t > stall_limit:
            if ramp_state == 'on' or (ramp_state == 'cand' and not PUSH_THROUGH_SMALL):
                reason = "ramp_stall"
                print(f"\n    ГОРКА НЕ ВЗЯТА на {traveled_cm*10:.0f}мм "
                      f"(pitch={pitch_delta:+.1f}°) — скатываюсь обратно")
            elif on_obstacle or ramp_state == 'cand':
                reason = "obstacle_stall"
                print(f"\n    ПРЕПЯТСТВИЕ НЕ ВЗЯТО (pitch={pitch_delta:+.1f}°, "
                      f"{assist_kicks} импульс(ов)) — откат и повтор")
            else:
                reason = "odo_fault"
                print("\n    АВАРИЯ: одометрия не растёт — торможу")
            break

        # ── Бампер: концевики спереди ─────────────────────────
        if bumper.get_bumper():
            side = bumper.bumper_side()
            reason = {"left": "bumper_left", "right": "bumper_right"}.get(side, "bumper_both")
            print(f"\n    БАМПЕР ({side}) на {traveled_cm*10:.0f}мм")
            # Упёрлись, заехав в следующую клетку, и после отката останемся
            # близко к её центру - значит, мы в ней (заезд 17.10 20:40: стена
            # в 146мм после отката, а карта осталась в прежней клетке и
            # потеряла по оси клетку).
            if reason == "bumper_both":
                from_cm = forced_from_cm if wall_forced else boundary_cm - cell_cm
                past_mm = (traveled_cm - from_cm) * 10.0 - bumper.BUMPER_ENTERED_BACKOFF_MM
                if ramp_state is None and past_mm >= bumper.BUMPER_ENTER_MM:
                    cells += 1
                    reason = "bumper_entered"
                    print(f"    уже в следующей клетке ({past_mm:.0f}мм за центром "
                          f"после отката) - засчитываю")
            break

        # ── Чёрная плитка: заезжать нельзя ────────────────────
        # Реагируем на первое же показание, без подтверждения вторым: датчик
        # обновляется ~10Гц, и одно лишнее подтверждение стоит ещё ~5см хода
        # вглубь чёрного. Запас по порогу большой (C=100 против порога ~210),
        # так что ложные срабатывания маловероятны.
        tile = floor_tiles.tile_color()
        if tile == config.TILE_BLACK:
            reason = "black"
            # Назад - в центр клетки перед чёрной. Без остановок чёрная бывает
            # и над клеткой, которую план уже засчитал пройденной.
            cells, black_back_cm = runplan.black_retreat(traveled_cm, boundary_cm,
                                                         cells, cell_cm)
            print(f"\n    ЧЁРНАЯ (C={esp_link.get_rgbc()[3]}) на {traveled_cm*10:.0f}мм - "
                  f"откат {black_back_cm*10:.0f}мм")
            break
        if plan is not None and tile != config.TILE_UNKNOWN:
            # Над какой клеткой датчик: N - уже за её границей (+15мм).
            k_under = cells + 1 if traveled_cm >= boundary_cm - cell_cm / 2.0 + 1.5 else cells
            tiles.setdefault(k_under, set()).add(tile)
            if (tile == config.TILE_BLUE and k_under == cells >= 1 and not stopping
                    and traveled_cm < boundary_cm - cell_cm - 3.0):
                # Синюю заметили уже после решения "прямо": её центр ещё
                # впереди - встаём там (правила: стоянка на синей).
                boundary_cm -= cell_cm
                stopping, pending, reason = True, False, "stop"
                print(f"\n    → кл.{cells}: синяя после решения - стоп в её центре")
            if tile == state.CHECKPOINT_TILE and 1 <= k_under <= cells and k_under not in cp_done:
                cp_done.add(k_under)
                plan.checkpoint(k_under)

        # ── Горка или всплеск: что под колёсами ──────────────
        pitch_f = pitch_delta if pitch_f is None else \
            pitch_f + (pitch_delta - pitch_f) * RAMP_PITCH_ALPHA
        p_abs = abs(pitch_f)
        ramp_log.append((traveled_cm, pitch_f))
        if ramp_state is None:
            # Каждый всплеск на ступеньке/лежачем - это лишние тики энкодера,
            # цель клетки отодвигаем (см. BUMP_EXTRA_MM). Начало горки сюда
            # тоже попадает, но там цель всё равно пересчитается от гребня.
            if not bump_on and p_abs >= config.OBSTACLE_PITCH_DEG:
                bump_on = True
                add = min(BUMP_EXTRA_MM / 10.0, BUMP_EXTRA_MAX_MM / 10.0 - bump_extra_cm)
                if add > 0:
                    bump_extra_cm += add
                    boundary_cm += add
            elif bump_on and p_abs < OBSTACLE_REARM_DEG:
                bump_on = False
            if p_abs >= config.RAMP_MAP_DEG:
                ramp_state = 'cand'
                ramp_sign = 1 if pitch_f > 0 else -1
                ramp_cand_cm = traveled_cm
                # Цель клетки до горки - вернуть её, если "горка" окажется
                # короткой лестницей (RAMP_MIN_LEN_CM).
                ramp_saved = (boundary_cm, stopping, pending, reason)
        elif ramp_state == 'cand':
            if p_abs < config.RAMP_MAP_DEG - RAMP_MAP_HYST_DEG or pitch_f * ramp_sign < 0:
                print(f"\n    не горка: {pitch_f:+.1f}°, наклон держался "
                      f"{(traveled_cm - ramp_cand_cm)*10:.0f}мм")
                ramp_state = None
            elif traveled_cm - ramp_cand_cm >= RAMP_CONFIRM_CM:
                ramp_state = 'on'
                ramp_cells0 = cells
                stopping = pending = False
                print(f"\n    ГОРКА {'ВВЕРХ' if ramp_sign > 0 else 'ВНИЗ'} "
                      f"({pitch_f:+.1f}°) - еду без остановок до гребня")
        elif ramp_state == 'on':
            if p_abs < config.RAMP_FLAT_DEG:
                if ramp_flat_cm is None:
                    ramp_flat_cm = traveled_cm
                elif traveled_cm - ramp_flat_cm >= RAMP_FLAT_CM:
                    info = ramps._ramp_measure(ramp_log, ramp_sign)
                    f_anchor = None      # всё, что лидар видел на склоне, - мусор
                    if not expect_ramp and (info["len_cm"] < RAMP_MIN_LEN_CM
                            or abs(info["rise_cm"]) < RAMP_MIN_RISE_CM):
                        # Лестница внутри клетки. Цель - прежняя; если уже
                        # проскочили её, остановка сразу, доводка по стене.
                        print(f"    не горка: наклон {info['len_cm']*10:.0f}мм, "
                              f"высота {info['rise_cm']*10:+.0f}мм - "
                              f"лестница/препятствие")
                        boundary_cm, stopping, pending, reason = ramp_saved
                        ramp_state = None
                        ramp_rejected = True
                        ramp_flat_cm = None
                        ramp_log = []
                    else:
                        state.last_ramp = info
                        # Центр первой клетки - полклетки за гребнем, но не
                        # ближе RAMP_EXIT_MIN_MM от момента, когда стало ровно.
                        boundary_cm = max(info["crest_cm"] + cell_cm * 0.5,
                                          ramp_flat_cm + RAMP_EXIT_MIN_MM / 10.0)
                        ramp_state = 'exit'
                        stopping, pending, reason = True, False, "ramp"
                        print(f"    съехали: до центра клетки "
                              f"{(boundary_cm - traveled_cm)*10:.0f}мм")
            else:
                ramp_flat_cm = None
        on_slope_now = ramp_state in ('cand', 'on')
        if on_slope_now:
            # Подъём законно длится дольше CELL_TIMEOUT_S - пока одометрия
            # растёт, бюджет клетки не тратится (залипание ловит stall выше).
            t_cell = max(t_cell, last_progress_t)

        # ── Дистанция до передней стены на частоте петли ──────
        # Лидар даёт ~10 Гц, на подходе это замер раз в пару сантиметров.
        # Между сканами продлеваем последнее показание по одометрии, иначе
        # стена "прыгает" и по ней нельзя тормозить.
        #
        # Раньше ЛЮБОЙ пустой скан (f_now=None) мгновенно стирал якорь -
        # один битый оборот лидара (например, из-за нагрузки CPU от YOLO
        # сразу после победы жертвы на предыдущей клетке) полностью ослеплял
        # эту защиту до следующего валидного скана, и робот ехал по чистой
        # одометрии до самой стены. Теперь пустой скан якорь НЕ трогает -
        # экстраполяция по одометрии продолжается со старой, но реальной
        # точки. Сбрасываем якорь только когда лидар явно говорит "далеко"
        # (валидное f_now >= FRONT_TRACK_MM) или когда экстраполяция от него
        # зашла дальше ANCHOR_STALE_CM - тогда старой точке верить рискованно.
        seq = state.scan_cnt
        walls = lidar_walls.get_walls()      # один снимок стен на весь тик
        f_now = walls["front"]
        if on_slope_now or p_abs >= config.RAMP_MAP_DEG:
            # Перед лидара на склоне - это пол или небо. Стоя у подножия, он
            # вообще видит саму горку как стену в ~40см (130мм/tg25..30° + 15см),
            # и без этого робот вставал бы "у стены" на первой плитке горки.
            f_now = None
            f_anchor = None
        if seq != last_seq:
            last_seq = seq
            if f_now is not None:
                if f_now < FRONT_TRACK_MM:
                    f_anchor, f_anchor_cm = f_now, traveled_cm
                else:
                    f_anchor = None
            elif f_anchor is not None and traveled_cm - f_anchor_cm > ANCHOR_STALE_CM:
                f_anchor = None
        f_est = None if f_anchor is None else \
            f_anchor - (traveled_cm - f_anchor_cm) * 10.0

        # ── Стены клеток впереди (проезд без остановок) ───────
        # Каждый новый оборот лидара: точки коридоров - в систему пути, в
        # копилку стен N и следующей за ней; передняя стена N - голос по
        # этому же обороту; конец/начало боковой стены у границы с N -
        # поправка пути (столбик). На склоне лидар видит пол - не копим.
        if cell_sides is not None:
            if on_slope_now or p_abs >= config.RAMP_MAP_DEG:
                scan_mark = state.scan_cnt
            else:
                yaw_err = turning.angle_err(esp_link.get_yaw(), target_yaw)
                for cnt, t_end, trev, pts in lidar_walls._new_scans(scan_mark):
                    scan_mark = cnt
                    path_pts = corridor.to_path_frame(
                        pts, t_end, trev, lambda t: (odo_t.at(t) or 0.0) * 10.0, yaw_err)
                    x_robot = (odo_t.at(t_end - trev / 2.0) or 0.0) * 10.0
                    kN, cN = cells + 1, boundary_cm * 10.0
                    cell_sides.add_scan(path_pts, x_robot,
                                        [(kN, cN), (kN + 1, cN + corridor.CELL_MM)])
                    cell_sides.prune(x_robot - corridor.CELL_MM)
                    to_n = (boundary_cm - traveled_cm) * 10.0
                    # Нет переднего замера (мало точек в окне) - не голос
                    # "прохода": это не то же, что "впереди пусто".
                    if 0.0 < to_n <= corridor.CELL_MM and walls["front"] is not None:
                        f_c = walls["front"] - to_n
                        front_votes[0 if lidar_walls.wall_present(f_c, WALL_THRESH) else 1] += 1
                    border = cN - corridor.HALF_CELL_MM
                    if kN not in posts_done:
                        for side in ("right", "left"):
                            e = corridor.post_error(cell_sides.band_x[side], border,
                                                    side_known.get((kN - 1, side)),
                                                    cell_sides.result(kN, side)[0])
                            if e is not None:
                                boundary_cm += e / 20.0
                                posts_done.add(kN)
                                print(f"\n    [odo] столбик {side} у границы кл.{kN}: "
                                      f"{e:+.0f}мм - цель {e / 2:+.0f}мм")
                                break

        # ── Наблюдатель меток ─────────────────────────────────
        # Только на ровном (не горка, не всплеск, не ассист) и только к
        # стене. Заметил - тормозим тем же профилем к stop_point (ниже, см.
        # mark_first), а наблюдатель гасим сразу: камера вернётся к обычной
        # выдержке, пока робот ещё тормозит.
        if watcher is not None:
            if watcher.dead:
                watcher, watch_armed = None, False
            else:
                sides = side_hold.update(now_m, {
                    "right": lidar_walls.wall_present(walls["right"], SIDE_THRESH),
                    "left": lidar_walls.wall_present(walls["left"], SIDE_THRESH)})
                flat = watch_logic.watch_allowed(ramp_state, bump_on, assist_mode,
                                                 pitch_delta, OBSTACLE_REARM_DEG, wall_forced)
                watch_armed = watcher.arm(sides if flat and vmark is None else set())
                for b in watcher.poll():
                    if not side_hold.frame_ok(b.side, b.t_frame):
                        # Кадр снят, когда стены с этой стороны не было:
                        # метка на дальней стене, через проём.
                        print(f"\n    [W] {b.side} {b.cls} {b.conf:.2f} - в момент кадра "
                              f"стены нет, не беру  {b.image or ''}")
                        continue
                    m = marks._watch_decide(b, cell, hdg, odo_t, vmark is not None)
                    if m is not None:
                        # Тормозим К МЕТКЕ: встать с ней в центре кадра, тем
                        # же профилем. Не успеваем - как можно раньше и
                        # подъезд назад (см. watch_logic.brake_target_cm).
                        m["stop_cm"], late = watch_logic.brake_target_cm(
                            m["pos_mm"], traveled_cm, fwd, MARK_BRAKE_DECEL_CMS2, STOP_LEAD_S)
                        vmark = m
                        watch_armed = watcher.arm(set())
                        print(f"    метка на ходу: торможу {'(поздно) ' if late else 'к метке '}"
                              f"через {(m['stop_cm'] - traveled_cm) * 10:.0f}мм")

        # Не глушится и на склоне: при лидаре на 130мм пол носом вниз ближе
        # FRONT_EMERGENCY_MM - это уклон больше 40°, а стена вплотную - всегда
        # стена. Поэтому берём сырое показание, а не обнулённое выше.
        f_raw = walls["front"]
        if f_raw is not None and f_raw <= FRONT_EMERGENCY_MM:
            print(f"\n    СТЕНА ВПЛОТНУЮ: f={f_raw:.0f}мм  d={traveled_cm*10:.0f}мм")
            # Раньше причина оставалась "stop"/"done" и клетка не засчитывалась
            # никогда. Два случая:
            #  - почти не сдвинулись: стена прямо перед клеткой, где стоим, а
            #    карта считала её открытой - вызывающий записывает её в карту
            #    (иначе BFS раз за разом выбирал тот же путь и робот "застревал"
            #    стоя на месте, см. MAX_STALLS);
            #  - проехали больше полклетки: мы уже в следующей клетке, просто
            #    ближе к стене, чем надо - засчитываем, доводка отъедет назад.
            reason = "wall_close"
            if traveled_cm >= (boundary_cm - cell_cm) + cell_cm * 0.5:
                cells += 1
            break

        # ── Сколько ещё ехать ─────────────────────────────────
        # Есть стена - правда у неё: едем ДО НЕЁ, а не до отметки одометрии,
        # поэтому робот встаёт в центре клетки сразу, без доводки после
        # остановки. Но верим ей, только если она согласна с одометрией в
        # пределах доводки: разошлись сильнее - значит это стена не этой
        # клетки (видно через открытый проём), и правит одометрия.
        rem_odo = boundary_cm - traveled_cm
        rem = rem_odo
        at_wall = False
        if f_est is not None:
            rem_wall = (f_est - config.FRONT_CENTER_MM) / 10.0
            if abs(rem_wall - rem_odo) <= config.ALIGN_MAX_MM / 10.0:
                rem = rem_wall
                at_wall = True

        # ── Страховка: стена ближе, чем тормозной путь ────────
        # Решение "ехать дальше" не должно быть неотменяемым: стену могли
        # увидеть позже (она была за порогом видимости, или лидар моргнул).
        # Если до места остановки у стены осталось меньше, чем нужно на
        # торможение - тормозим немедленно, не дожидаясь следующей границы.
        # Действует и когда остановка запланирована с места (one_cell): тогда
        # это случай "стена ближе, чем цель, и с одометрией не согласна".
        if f_est is not None and not wall_forced:
            need_cm = fwd * fwd / (2.0 * BRAKE_DECEL_CMS2)
            to_stop_cm = (f_est - config.FRONT_CENTER_MM) / 10.0
            if to_stop_cm <= need_cm * 1.3 and (not stopping or to_stop_cm < rem - 1.0):
                wall_forced = True
                forced_from_cm = boundary_cm - cell_cm
                stopping, pending = True, False
                boundary_cm = traveled_cm + to_stop_cm
                rem = to_stop_cm
                at_wall = True
                print(f"\n    стена близко: {f_est:.0f}мм - торможу немедленно")

        # ── Решение по подходящей границе клетки ──────────────
        # Принимается один раз за клетку, ровно за DECIDE_LEAD_MM до её
        # центра - раньше нельзя (боковое окно замера ещё читает предыдущую
        # клетку), позже нельзя (не успеем затормозить).
        # С планом решение ждёт, пока кандидат в горку не выяснится: иначе
        # клетку перед горкой план записал бы посещённой.
        if pending and rem <= lead_cm and not (plan is not None and ramp_state == 'cand'):
            pending = False
            br = lidar_walls.wall_present(walls["right"], SIDE_THRESH)

            # Переднюю стену сравниваем не с сырым показанием, а с тем, каким
            # оно станет В ЦЕНТРЕ клетки: решение принимается за DECIDE_LEAD_MM
            # до центра, и та же стена читается на эти 110мм дальше. Без
            # пересчёта стена, дающая в центре нормальные ~180мм, здесь читалась
            # как 290мм, не проходила порог 260 - и робот уезжал в клетку со
            # стеной, считая её открытой. Прижатый концевик тут авторитетнее
            # лидара: вплотную к стене лидар её уже не видит (она ближе
            # собственного габарита робота).
            f_ref = f_est if f_est is not None else walls["front"]
            f_center = None if f_ref is None else f_ref - rem * 10.0
            bf = lidar_walls.wall_present(f_center, WALL_THRESH) or bumper.get_bumper()

            if cells + 1 >= max_cells:
                stopping, reason = True, "done"
            elif plan is not None and cell_sides is not None:
                kN = cells + 1
                res = {s: cell_sides.result(kN, s) for s in ("right", "left")}
                fv, fn = front_votes
                f_wall = (True if bumper.get_bumper() or (fv >= 2 and fv > fn) else
                          False if fn >= 2 and fn > fv else None)
                if config.TILE_BLUE in tiles.get(kN, ()):
                    print(f"\n    → кл.{kN}: синяя - стоп в её центре")
                    go, walls_rel = False, {}
                else:
                    go, walls_rel = plan.decide(kN, res, (f_wall, fv, fn))
                if go:
                    cells += 1
                    boundary_cm += cell_cm
                    rem += cell_cm
                    pending = True
                    t_cell = time.monotonic()
                    plan.passed(kN, walls_rel)
                    for s in ("right", "left"):
                        side_known[(kN, s)] = walls_rel.get(s)
                    front_votes = [0, 0]
                else:
                    stopping, reason = True, "stop"
            elif not br or bf:
                stopping, reason = True, "stop"
            else:
                # Прямая продолжается - не тормозим вообще, просто отодвигаем
                # цель на клетку вперёд и едем дальше на крейсерской.
                cells += 1
                boundary_cm += cell_cm
                rem += cell_cm
                pending = True
                t_cell = time.monotonic()
                sys.stdout.write(f"\r    клетка {cells} пройдена с ходу      \n")

            if stopping:
                brake_dist = fwd * fwd / (2.0 * BRAKE_DECEL_CMS2)
                print(f"    торможение с {fwd:.0f}см/с: путь {brake_dist*10:.0f}мм "
                      f"из {rem*10:.0f}мм в запасе")

        # Остаток с поправкой на запаздывание петли: одометрия отстаёт, и на
        # ходу робот уже дальше, чем показывает. Поправка пропорциональна
        # скорости, поэтому у самой цели она сама сходит на нет.
        rem_eff = rem - fwd * STOP_LEAD_S

        # Точка остановки у метки ближе цели - тормозим к ней. Иначе робот
        # штатно встанет в центре (или у стены), и метку читаем там же.
        mark_first = vmark is not None and vmark["stop_cm"] - traveled_cm < rem
        if mark_first:
            rem = vmark["stop_cm"] - traveled_cm
            rem_eff = rem - fwd * STOP_LEAD_S

        if (stopping or mark_first) and rem_eff <= 0.0 and ramp_state != 'cand':
            if vmark is not None and not (wall_forced
                                          and traveled_cm < forced_from_cm + cell_cm * 0.5):
                # Стоим у метки (или в центре, если он был ближе): читаем её
                # здесь, потом - та же поездка дальше, без Z и без новой цели.
                t_stop = time.monotonic()
                out = marks._mark_stop(vmark, cell, hdg, target_yaw, traveled_cm)
                vmark = None
                if out in ("lifted", "paused"):
                    return cells, out
                if out == "link_lost":
                    reason = out
                    break
                pause_s = time.monotonic() - t_stop
                # Таймер клетки и сторожа застревания - на длительность паузы,
                # иначе после неё сразу timeout/odo_fault.
                (t_cell, last_progress_t, assist_gap_until, assist_t0, assist_total_t0,
                 assist_progress_t) = watch_logic.shift_timers(
                    (t_cell, last_progress_t, assist_gap_until, assist_t0, assist_total_t0,
                     assist_progress_t), pause_s)
                # Подъезд и развороты уже в энкодерах - добираем их в путь сразу.
                cl, cr = esp_link.get_odo()
                traveled_cm += ((cl - last_l) + (cr - last_r)) / 2.0
                last_l, last_r = cl, cr
                last_progress_cm = traveled_cm
                t0 = last_loop_t = time.monotonic()     # новый плавный разгон с места
                fwd = last_steer = 0.0
                odo_hist.clear()
                f_anchor = None                         # перед - заново со свежего оборота
                last_seq = state.scan_cnt
                # Обороты за время стоянки сняты с подъезда и поворотов к
                # аптечкам - для стен впереди они не годятся.
                scan_mark = state.scan_cnt
                side_hold.reset()
                left_cm = boundary_cm - traveled_cm
                print(f"    продолжаю: пауза {pause_s:.1f}с, до центра {left_cm * 10:.0f}мм")
                if stopping and left_cm * 10.0 < config.ALIGN_TOL_MM:
                    # Уже в центре или подъезд увёл за него - остаток доводкой.
                    if abs(left_cm) > 0.2:
                        creep_mm(left_cm * 10.0, target_yaw)
                    cells += 1
                    break
                telem_seq = esp_link.wait_telem(telem_seq)
                continue
            if wall_forced and traveled_cm < forced_from_cm + cell_cm * 0.5:
                # Встали перед тем, что внезапно оказалось близко, не доехав
                # и до середины пути: это не стена следующей клетки (она
                # была бы на границе, и по ней остановка шла бы штатно), а
                # препятствие - например, кирпич у входа в неё. Клетку НЕ
                # засчитываем: раньше она засчитывалась, и робот считал, что
                # стоит в следующей клетке, недоехав до неё 10-25см.
                reason = "front_obstacle"
                f_txt = "—" if f_est is None else f"{f_est:.0f}"
                print(f"\n    препятствие впереди: f={f_txt}мм, "
                      f"проехали {(traveled_cm - forced_from_cm)*10:.0f}мм")
                break
            cells += 1
            if at_wall:
                print(f"\n    у стены: f={f_est:.0f}мм")
            break

        # ── Профиль скорости ──────────────────────────────────
        # Разгон - по времени (RAMP_UP_S). Торможение включается только когда
        # решено остановиться, и начинается ровно с текущей скорости.
        ss = min(1.0, (time.monotonic() - t0) / RAMP_UP_S)
        ss = ss * ss * (3.0 - 2.0 * ss)         # smoothstep, без ступеньки
        ramp_up = v_max * ss                     # от 0, не от MOVE_MIN_CMS -
                                                  # иначе на t=0 снова ступенька

        # Наклон: нос задран (заезд) или опущен (съезд) - ход ограничен, в том
        # числе и после импульса момента, иначе на гребне режим скорости сам
        # разгонял бы робота обратно до крейсерской прямо на спуск.
        if pitch_delta <= -config.OBSTACLE_PITCH_DEG:
            tilt_target = DESCENT_CAP_CMS
        elif pitch_delta >= config.OBSTACLE_PITCH_DEG or assist_mode is not None:
            tilt_target = TILT_CAP_CMS
        else:
            tilt_target = v_max
        tilt_step = BRAKE_DECEL_CMS2 * loop_dt
        tilt_lim = max(tilt_lim - tilt_step, min(tilt_lim + tilt_step, tilt_target))
        ramp_up = min(ramp_up, tilt_lim)

        braking = False
        if (stopping or mark_first) and ramp_state != 'cand':
            # Постоянное замедление: пока запас пути велик, sqrt даёт больше
            # крейсерской и ничего не ограничивает - робот едет полным ходом.
            # Тормозить начинает сам, ровно когда пути остаётся в обрез.
            decel = MARK_BRAKE_DECEL_CMS2 if mark_first else BRAKE_DECEL_CMS2
            v_brake = math.sqrt(2.0 * decel * max(0.0, rem_eff))
            braking = v_brake < ramp_up
            fwd = min(ramp_up, v_brake)
            if braking:
                fwd = max(fwd, MOVE_MIN_CMS * 0.5)   # чтобы не заглохнуть у самой цели
        else:
            fwd = ramp_up

        # ── Центровка: сдвигает цель курса, а не руль ─────────
        wl, wr = walls["left"], walls["right"]
        wall_l = wl is not None and wl < WALL_CENTER_MM
        wall_r = wr is not None and wr < WALL_CENTER_MM

        if wall_l and wall_r:
            wall_e = (wr - wl) * 0.5        # >0: прижались к левой стене
        elif wall_l:
            wall_e = SIDE_CENTER_MM - wl
        elif wall_r:
            wall_e = wr - SIDE_CENTER_MM
        else:
            wall_e = 0.0
        if abs(wall_e) < WALL_DEADBAND_MM:
            wall_e = 0.0

        yaw, yaw_rate = esp_link.get_yaw_and_rate()   # один снимок на тик
        axis_e = turning.angle_err(yaw, target_yaw)
        on_ramp = abs(pitch_delta) >= RAMP_PITCH_DEG
        wall_kp = RAMP_WALL_KP_DEG if on_ramp else WALL_KP_DEG
        wall_max = RAMP_WALL_MAX_DEG if on_ramp else WALL_MAX_DEG
        # На длинной горке гироскоп не накапливает поперечную ошибку: боковые
        # стены по лидару непрерывно сдвигают целевой курс к центру.
        head_off = max(-wall_max, min(wall_max, wall_e * wall_kp))
        head_off_f += (head_off - head_off_f) * HEAD_OFF_SMOOTH
        heading = (target_yaw + head_off_f) % 360

        # ── Руль: один быстрый контур по гироскопу ────────────
        # P по ошибке курса, D по ИЗМЕРЕННОЙ угловой скорости (не по разности
        # углов) - см. GYRO_KD_RATE. Фильтр производной больше не нужен:
        # фильтровать было нечего, кроме шума собственного дифференцирования.
        yaw_e = turning.angle_err(yaw, heading)
        steer = -(yaw_e * GYRO_KP + yaw_rate * GYRO_KD_RATE)
        steer = max(-MAX_STEER_CMS, min(MAX_STEER_CMS, steer))
        # Та же кривая, что и у fwd: без этого при fwd≈0 и заметной ошибке
        # курса (например, сразу после поворота) команда получалась вида
        # (+20,-20) - робот резко крутился на месте вместо старта с места.
        steer *= ss
        # Ограничение рывка: срезает только скачки команды, в норме не влияет.
        d_max = STEER_SLEW_CMS_S * 0.010
        steer = max(last_steer - d_max, min(last_steer + d_max, steer))
        last_steer = steer

        # Ход сбрасываем по отклонению от ОСИ клетки, а не от смещённой
        # цели - иначе робот тормозил бы просто потому, что центруется.
        fwd *= max(0.35, 1.0 - abs(axis_e) / 18.0)

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
        pitch_up = pitch_delta

        if pitch_up < OBSTACLE_REARM_DEG:
            assist_mode = None
            assist_kicks = 0
            assist_gap_until = 0.0
            ramp_announced = False
        elif braking:
            assist_mode = None
        else:
            if assist_mode is not None:
                cap = RAMP_CAP_CMS if assist_mode == 'ramp' else config.OBSTACLE_CAP_CMS
                if v_meas > cap * 1.15:
                    print(f"\n    ассист снят: {v_meas:.0f}см/с > потолка {cap:.0f} - "
                          f"препятствие позади")
                    assist_mode = None
                    assist_kicks = 0
                    assist_gap_until = now_m + OBSTACLE_KICK_GAP_S
                    ramp_announced = False

            if (assist_mode is None and pitch_up >= config.OBSTACLE_PITCH_DEG
                    and v_meas < fwd * OBSTACLE_SLOW_FRAC
                    and now_m >= assist_gap_until
                    and assist_kicks < OBSTACLE_KICKS_MAX):
                assist_mode = 'kick'
                assist_kicks += 1
                assist_t0 = now_m
                if assist_kicks == 1:
                    assist_total_t0 = now_m
                assist_progress_cm, assist_progress_t = traveled_cm, now_m
                print(f"\n    препятствие #{assist_kicks}: pitch={pitch_up:+.1f}°, "
                      f"v={v_meas:.0f}/{fwd:.0f}см/с, момент {OBSTACLE_TORQUE_A:.1f}A "
                      f"(потолок {config.OBSTACLE_CAP_CMS:.0f}см/с)")

            if assist_mode == 'kick':
                if (pitch_up >= RAMP_PITCH_DEG
                        and now_m - assist_t0 >= RAMP_CONFIRM_S):
                    assist_mode = 'ramp'
                    assist_t0 = now_m
                    assist_progress_cm, assist_progress_t = traveled_cm, now_m
                elif now_m - assist_t0 > OBSTACLE_TORQUE_MAX_S:
                    assist_mode = None
                    assist_gap_until = now_m + OBSTACLE_KICK_GAP_S
                    if traveled_cm - assist_progress_cm >= OBSTACLE_PROGRESS_CM:
                        assist_kicks = 0     # сдвинулись - лимит попыток сбрасываем

            elif assist_mode == 'ramp':
                if traveled_cm >= assist_progress_cm + ODO_PROGRESS_CM:
                    assist_progress_cm, assist_progress_t = traveled_cm, now_m
                    assist_kicks = 0
                if (now_m - assist_progress_t > RAMP_STALL_S
                        or now_m - assist_total_t0 > RAMP_TOTAL_MAX_S):
                    # Встали на подъёме: момент снимаем, но следующий 'kick'
                    # (он сильнее) сам попробует сдвинуть робота.
                    assist_mode = None
                    assist_gap_until = now_m + OBSTACLE_KICK_GAP_S
                    ramp_announced = False
                    print("\n    горка: прогресса нет - снимаю момент")

        if assist_mode is not None:
            assist_extra_s = max(assist_extra_s, now_m - assist_total_t0)

        if assist_mode == 'ramp':
            if not ramp_announced:
                print(f"\n    горка: pitch={pitch_up:+.1f}°, "
                      f"момент {RAMP_TORQUE_A:.1f}A + коррекция курса")
                ramp_announced = True
            # steer уже ограничен ±MAX_STEER_CMS, так что отношение лежит
            # в ±1 и внешний clamp здесь - только страховка от правок выше.
            turn_a = max(-RAMP_STEER_A, min(RAMP_STEER_A,
                         (steer * config.TURN_SIGN / MAX_STEER_CMS) * RAMP_STEER_A))
            esp_link.torque(RAMP_TORQUE_A + turn_a, RAMP_TORQUE_A - turn_a, RAMP_CAP_CMS)
        elif assist_mode == 'kick':
            turn_a = max(-OBSTACLE_STEER_A, min(OBSTACLE_STEER_A,
                         (steer * config.TURN_SIGN / MAX_STEER_CMS) * OBSTACLE_STEER_A))
            esp_link.torque(OBSTACLE_TORQUE_A + turn_a, OBSTACLE_TORQUE_A - turn_a, config.OBSTACLE_CAP_CMS)
        else:
            esp_link.motors(fwd + steer * config.TURN_SIGN, fwd - steer * config.TURN_SIGN)

        # Статус ≤10 Гц: на 200 Гц забитый pipe SSH тормозил бы сам цикл.
        if time.monotonic() - last_status_t >= STATUS_PERIOD_S:
            last_status_t = time.monotonic()
            state.drive_live = {"cell": cells + 1, "rem_mm": round(rem * 10), "spd": round(fwd, 1),
                           "steer": round(steer, 1), "wall_e": round(wall_e),
                           "head_off": round(head_off, 1), "mode": assist_mode,
                           "t": last_status_t}
            sys.stdout.write(f"\r    кл.{cells + 1} ост.{rem*10:4.0f}мм  "
                             f"yaw={yaw:.1f}°(e={axis_e:+.1f})  "
                             f"we={wall_e:+.0f}→{head_off:+.1f}°  spd={fwd:.0f}   ")
            sys.stdout.flush()
        # Такт по приходу пакета, а не sleep(0.010): два независимых периода
        # по 10мс давали биение до целого периода сверху - самый дешёвый
        # источник запаздывания в контуре (см. STOP_LEAD_S). Таймаут только
        # страхует от зависания петли при обрыве линка.
        telem_seq = esp_link.wait_telem(telem_seq)

    if watcher is not None:
        watcher.arm(set())      # доводке и скану стоя камера нужна в обычном режиме

    # Гребень уже позади - что бы ни остановило робота дальше (стена
    # вплотную, бампер, таймаут), он на другом этаже, и вызывающий обязан
    # занести горку в карту, а не шагнуть на соседа по одометрии.
    if ramp_state == 'exit' and reason not in ("lifted", "link_lost", "odo_fault"):
        cells, reason = ramp_cells0 + 1, "ramp"

    # Никакого доката: профиль уже привёл скорость почти к нулю, поэтому сразу
    # в фиксацию. ESP держит 0 dps активно и повторяет команду, пока мы меряем
    # стены - так же жёстко, как в конце поворота.
    #
    # Исключение - робот в руках: _lift_thread уже заглушил моторы командой
    # stop(), и фиксировать 0 dps на весу значит драться с человеком.
    if state.lifted.is_set():
        esp_link.stop()
    else:
        esp_link.settle_brake(0.06)

    # Связь или моторы пропали посреди поездки (выключили питание моторов,
    # отошёл провод) - это не конец заезда. Ждём их здесь же, а потом
    # ставим робота в центр ближайшей засчитанной клетки: стоит он там, где
    # его застал обрыв, а вызывающий считает от центра клетки. Энкодеры живы,
    # но одометрия не росла - не сюда: это застревание, его разбирает
    # вызывающий (отъезд и закрытое ребро).
    if reason in ("odo_fault", "link_lost") and not state.lifted.is_set():
        c0 = cell if cell is not None else (state.live["cx"], state.live["cy"])
        h0 = hdg if hdg is not None else state.live["hdg"]
        if recovery.wait_for_robot(c0[0], c0[1], h0, state.live["cp"], state.live["phase"]):
            k = cells + 1 - int(round((boundary_cm - traveled_cm) / cell_cm))
            k = max(0, min(cells, k))
            off_cm = traveled_cm - (boundary_cm - (cells + 1 - k) * cell_cm)
            print(f"    продолжаю: в центр клетки {k} поездки ({-off_cm*10:+.0f}мм)")
            if abs(off_cm) > 0.5:
                creep_mm(-off_cm * 10.0, target_yaw,
                         timeout_s=2.0 + 1.5 * abs(off_cm) / ALIGN_SPEED_CMS)
            cells, reason = k, "resumed"

    if reason == "ramp_stall":
        # Скатываемся задом ровно на пройденное - обратно в центр клетки, из
        # которой выехали. Карта при этом не меняется: робот там же, где был.
        if traveled_cm > 0.2:
            creep_mm(-traveled_cm * 10.0, target_yaw,
                     timeout_s=2.0 + 1.5 * traveled_cm / ALIGN_SPEED_CMS)
    elif reason == "front_obstacle":
        # Назад в центр клетки, из которой выехали: по передней "стене" тут
        # не выровняться - это препятствие, а не граница клетки.
        back_cm = traveled_cm - forced_from_cm
        if back_cm > 0.2:
            creep_mm(-back_cm * 10.0, target_yaw,
                     timeout_s=2.0 + 1.5 * back_cm / ALIGN_SPEED_CMS)
    elif reason in ("lifted", "link_lost", "obstacle_stall"):
        # Выравниваться нечем и незачем: робота либо держат в руках, либо
        # связи нет, либо он упёрся в препятствие и решение принимает
        # вызывающий (flood_fill_run/return_to_start: откат и повтор).
        pass
    elif reason == "black":
        # Сдаём назад на ту клетку, с которой заехали. Доводку по стене здесь
        # не делаем: она встала бы по передней стене, то есть потащила бы
        # робота обратно к чёрному.
        if black_back_cm > 0.2:
            creep_mm(-black_back_cm * 10.0, target_yaw)
    elif reason == "bumper_entered":
        # Клетка засчитана, впереди стена или препятствие: только отпустить
        # концевики. Доводку не делаем - по стене за препятствием она снова
        # повела бы носом в него.
        creep_mm(-bumper.BUMPER_ENTERED_BACKOFF_MM, target_yaw)
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
        over_cm = traveled_cm - boundary_cm
        if ramp_rejected and reason in ("stop", "done") and over_cm * 10.0 > config.ALIGN_TOL_MM:
            print(f"    горка отменена, проскочили центр клетки на {over_cm*10:.0f}мм - назад")
            creep_mm(-over_cm * 10.0, target_yaw,
                     timeout_s=2.0 + 1.5 * over_cm / ALIGN_SPEED_CMS)
        turning.align_at_wall(target_yaw)
    print()
    return cells, reason


def creep_mm(dist_mm, target_yaw, timeout_s=2.0, speed_cms=None, accel_cms2=None):
    """Проехать ровно dist_mm (знак = направление), держа курс по гироскопу.

    Меряем лидаром, а ЕДЕМ по одометрии: лидар обновляется 10 Гц, по нему
    15мм не отработать - прежняя доводка отмеряла ход секундомером и поэтому
    промахивалась. Разворот руля здесь работает одинаково вперёд и назад:
    вращение задаёт разность колёс, а не их знак.

    accel_cms2 - плавно: разгон и торможение с этим ускорением (не ниже
    CREEP_MIN_CMS), иначе сразу speed_cms (см. creep_soft).
    """
    l0, r0 = esp_link.get_odo()
    v_max = ALIGN_SPEED_CMS if speed_cms is None else speed_cms
    v = math.copysign(v_max, dist_mm)
    goal_cm = abs(dist_mm) / 10.0

    t0 = time.time()
    telem_seq = state.telem_seq
    while time.time() - t0 < timeout_s:
        # Бампер останавливает только движение вперёд: отъезд назад от стены -
        # это как раз тот случай, когда концевик заведомо прижат.
        if esp_link.interrupted() or (dist_mm > 0 and bumper.get_bumper()):
            break
        cl, cr = esp_link.get_odo()
        # Та же поправка на запаздывание, что и в основном профиле: одометрия
        # показывает, где робот был ~30мс назад, поэтому цель считается
        # достигнутой чуть раньше. Без этого доводка регулярно проскакивала и
        # следующая итерация отыгрывала назад.
        done_cm = abs(((cl - l0) + (cr - r0)) * 0.5)
        if done_cm >= goal_cm - abs(v) * STOP_LEAD_S:
            break
        if accel_cms2:
            # плавно: разгон по времени, торможение по остатку пути
            v = math.copysign(max(CREEP_MIN_CMS, min(
                v_max, CREEP_MIN_CMS + accel_cms2 * (time.time() - t0),
                math.sqrt(2.0 * accel_cms2 * max(0.0, goal_cm - done_cm)))), dist_mm)
        yaw_e = turning.angle_err(esp_link.get_yaw(), target_yaw)
        st = -(yaw_e * GYRO_KP + esp_link.get_yaw_rate() * GYRO_KD_RATE)
        # медленно - руль слабее, чтобы ни одно колесо не крутилось назад
        lim = 8.0 if not accel_cms2 else max(2.0, 0.6 * abs(v))
        st = max(-lim, min(lim, st))
        esp_link.motors(v + st * config.TURN_SIGN, v - st * config.TURN_SIGN)
        telem_seq = esp_link.wait_telem(telem_seq)

    esp_link.brake()
    time.sleep(0.08)


# Подъезды у метки (к ней, сдвиг для другого блика, отъезд передней камерой и
# обратно) - медленнее доводки и без рывков: робот рядом со стеной, камера
# снимает, а прежние 18 см/с с места выглядели как дёрганье туда-обратно.
MARK_CREEP_CMS = 9.0
MARK_CREEP_ACCEL_CMS2 = 20.0
CREEP_MIN_CMS = 4.0


def creep_soft(dist_mm, target_yaw):
    """creep_mm у метки: MARK_CREEP_CMS с плавным разгоном и торможением."""
    t = 1.5 + abs(dist_mm) / 10.0 / (MARK_CREEP_CMS * 0.5)
    creep_mm(dist_mm, target_yaw, timeout_s=t, speed_cms=MARK_CREEP_CMS,
             accel_cms2=MARK_CREEP_ACCEL_CMS2)
