"""Повороты: поворот на угол, доворот на курс, выравнивание у стены."""
from __future__ import annotations

import math
import time
from collections import deque

import watch_logic
from victim_detect import CONF_FAST, MIN_BOX_PX
from walls import BACK_THRESH, SIDE_THRESH, WALL_THRESH

import config
import state
import timing
import driving
import esp_link
import lidar_walls
import ramps
import turn_unstick


BACK_CENTER_MM = 135.0

# Задняя стена на наклоне недостоверна: при задранном носе задний сектор
# лидара смотрит в пол и ловит его на ~h/tg(pitch) - при высоте лидара 130мм
# и наклоне 30° это ~225мм, то есть ВНУТРИ BACK_THRESH=260. Доводка "по
# задней стене" на горке - это доводка по полу.
SLOPE_BACK_GUARD_DEG = 6.0

# ── Поворот (по гироскопу) ───────────────────────────────────
# Разворот на месте у 4WD-танка - это всегда боковое скольжение колёс по
# полу. С цепкими шинами сила этого скольжения велика, и прежний P-регулятор
# по углу делал ровно то, от чего такой привод скачет:
#   - рывок на старте (P сразу в насыщение, 40см/с за 0.3с);
#   - долгое доползание на TURN_MIN_CMS у цели - медленное скольжение с
#     цепкой резиной и есть "залип-сорвался" (stick-slip), то есть прыжки;
#   - колебание вокруг цели и резкий тормоз из него.
# Теперь поворот едет по ПРОФИЛЮ УГЛОВОЙ СКОРОСТИ, как прямой ход по профилю
# линейной: плавный разгон (smoothstep, без удара ускорением), полка, и
# торможение с постоянным замедлением ровно в ноль у цели - sqrt(2*a*остаток),
# остаток считается по реальному углу, так что профиль сам закрывает контур
# по положению. Колёса получают прямую подачу (TURN_FF, см/с колеса на
# градус/с корпуса) плюс небольшую поправку по измеренной угловой скорости.
#
# TURN_FF заранее не известен: сколько колесу надо проехать на градус
# поворота, зависит от проскальзывания, то есть от шин и покрытия. Поэтому он
# подстраивается на ходу по отношению "команда / достигнутая скорость" и
# переживает поворот - после первого поворота подача уже подобрана. Итоговое
# значение печатается после каждого поворота: когда устоится, его можно
# вписать в TURN_FF_INIT.
TURN_RATE_MAX_DPS = 100.0   # полка угловой скорости корпуса
TURN_ACCEL_DPS2 = 400.0     # пик углового ускорения на разгоне
TURN_DECEL_DPS2 = 350.0     # замедление к цели - мягче разгона: у цели прыжки хуже всего
TURN_FF_MIN = 0.10
TURN_FF_MAX = 0.60
TURN_FF_ADAPT = 0.04        # доля нового замера за тик (200Гц)
TURN_RATE_KP = 0.06         # см/с на градус/с ошибки угловой скорости
TURN_LEAD_S = 0.04          # поправка остатка угла на запаздывание (как STOP_LEAD_S)
TURN_MAX_CMS = 60.0         # предел команды колесу
TURN_MIN_CMS = 5.0          # страховка от залипания, если профиль недотянул
TURN_DONE_RATE_DPS = 20.0   # "доехал" - только если и вращение уже почти стихло
TURN_SETTLE = 3
TURN_DT = 0.015
TURN_TIMEOUT_S = 6.0        # на сам поворот; время манёвров выхода сюда не входит
TURN_TILT_TOL_DEG = 3.0
TURN_TILT_DONE_RATE_DPS = 45.0
# Застревание на повороте (колесо зацепилось за угол стены, корпус упёрся):
# цель ещё не достигнута, а за TURN_STUCK_S угол к цели почти не уменьшился.
# Мгновенный gyro rate на косом профиле скачет даже без продвижения:
# он не должен каждый раз сбрасывать обнаружение застревания.
# Порог средней скорости ниже TURN_DONE_RATE_DPS: у самой цели робот законно
# доползает медленно на TURN_MIN_CMS, это не застревание. Ступени по
# нарастающей - в turn_unstick.py (момент, раскачка, одной стороной, откат).
TURN_STUCK_RATE_DPS = 8.0
TURN_STUCK_S = 0.5
# Застревание - только дальше этого от цели. Ближе робот законно доползает
# последние градусы на TURN_MIN_CMS и иногда замирает на трении; дожим
# 20см/с там перелетал цель, робот возвращался, снова "застревал" и
# уходил в откат - отсюда тряска на поворотах. У цели работает прежняя
# страховка TURN_MIN_CMS.
TURN_STUCK_MIN_DEG = 10.0
ALIGN_TRIES = 2
CORNER_ARM_FROM_DEG = 0.0   # прогреть камеру уже у соседней стены клетки
CORNER_SIDE_FRAME_FROM_DEG = 25.0
CORNER_FRAME_FROM_DEG = 45.0
CORNER_FRAME_TO_DEG = 75.0
CORNER_LIDAR_MAX_AGE_S = 0.15
CORNER_WALL_HOLD_S = 0.20  # один пропущенный оборот лидара не перезапускает камеру


def angle_err(cur, tgt):
    e = cur - tgt
    while e > 180: e -= 360
    while e < -180: e += 360
    return e


# ══════════════════════════════════════════════════════════════
#  Поворот по гироскопу
# ══════════════════════════════════════════════════════════════
def snap_target(delta_deg=0.0):
    """Текущий курс, подтянутый к сетке 90° от YAW_ORIGIN, плюс delta_deg."""
    rel_now = (esp_link.get_yaw() - state.YAW_ORIGIN + 360) % 360
    snap_rel = round(rel_now / 90) * 90 % 360
    return (state.YAW_ORIGIN + snap_rel + delta_deg + 360) % 360


def turn(delta_deg):
    """Поворот на delta_deg от снэпнутого к сетке 90° текущего курса.
    False - не доехал (см. turn_to)."""
    return turn_to(snap_target(delta_deg), label=f"{delta_deg:+.0f}°")


def gyro_heading():
    """Ближайшая к гироскопу сторона сетки поля, 0=N..3=W - как в
    detect_heading(), только без сверки со стенами."""
    return int(round(angle_err(esp_link.get_yaw(), state.YAW_ORIGIN) / 90.0)) % 4


def corner_watch_side(progress_deg, turn_deg, front_wall, side_wall, lidar_fresh,
                      side_ranges, last_wall_t, now):
    """Камера, проходящая мимо передней стены на 45° четверти поворота.

    Камера заранее набирает кадры у реальной боковой стены. До 45° метка
    относится к ней, после 45° — к передней. Короткий провал
    бокового замера при вращении не перезапускает выдержку и arm_id, но
    рамка годится только при свежем близком замере в момент самого кадра.
    """
    if not front_wall or abs(turn_deg) < 60.0 or abs(turn_deg) > 120.0:
        return None, False, None
    sign = 1 if turn_deg > 0 else -1
    side = "left" if sign > 0 else "right"
    if not CORNER_ARM_FROM_DEG <= progress_deg <= CORNER_FRAME_TO_DEG:
        return None, False, None
    near = lidar_fresh and lidar_walls.wall_present(side_ranges.get(side), SIDE_THRESH)
    if near:
        last_wall_t = now
    armed = last_wall_t is not None and 0.0 <= now - last_wall_t <= CORNER_WALL_HOLD_S
    on_wall = ((side_wall and CORNER_SIDE_FRAME_FROM_DEG <= progress_deg < CORNER_FRAME_FROM_DEG)
               or progress_deg >= CORNER_FRAME_FROM_DEG)
    return (side if armed else None,
            near and on_wall,
            last_wall_t)


def corner_quarter(progress_deg, turn_deg, start_h):
    """Локальный угол, передняя стена и новый курс для 45°/135° поворота."""
    stage = int(progress_deg // 90.0)
    quarters = 2 if abs(turn_deg) > 120.0 else 1
    if stage < 0 or stage >= quarters:
        return None
    sign = 1 if turn_deg > 0 else -1
    return (progress_deg - 90.0 * stage,
            (start_h + sign * stage) % 4,
            (start_h + sign * (stage + 1)) % 4)


def corner_frame_wall(quarter, sign):
    """Стена и курс сетки для кадра до/после биссектрисы угла."""
    local, before_h, after_h = quarter
    if local < CORNER_FRAME_FROM_DEG:
        return (before_h - sign) % 4, before_h
    return before_h, after_h


@timing._timed("поворот")
def turn_to(target, label=None, seen=None, walls=None, turn_sign=None):
    """Поворот на АБСОЛЮТНЫЙ курс target (град), без привязки к сетке 90°.

    Нужен отдельно от turn(): у объезда препятствия (bumper_side) курс после
    поворота на 45° лежит МЕЖДУ узлами сетки, и turn(delta) на возврат снова
    вызвал бы snap_target() - а тот привязывается к БЛИЖАЙШЕМУ узлу сетки от
    ТЕКУЩЕГО (уже 45°-смещённого) курса, что неоднозначно и может увести не
    туда. turn_to() возвращает точно на тот курс, что был до манёвра.

    Возвращает True, если доехал до цели; False - прервали (пауза, пропала
    телеметрия) или вышел TURN_TIMEOUT_S. Тогда курс в карте "как просили"
    считать нельзя - вызывающий берёт его с гироскопа (gyro_heading()).

    seen - кадры обеих стен угла, снятые около 45°. При первой подходящей
    рамке поворот тормозится и возвращается "mark" для немедленного чтения.
    При 180° без промежуточной остановки кадры приходят у 45° и 135°.
    Наблюдатель включается лишь при близкой стене по свежему лидару и
    передней стене соответствующей четверти по карте. Для кадра до 45°
    дополнительно нужна боковая стена по карте. Чтение - после поворота.
    turn_sign=1 принудительно выбирает поворот по часовой на 180°, в том
    числе когда кратчайшая дуга из-за ошибки курса была бы против часовой.
    """
    settled = 0
    start_yaw = esp_link.get_yaw()
    turn_deg = -angle_err(start_yaw, target)
    if turn_sign == 1:
        turn_deg = (target - start_yaw + 360.0) % 360.0
    start_h = int(round(angle_err(start_yaw, state.YAW_ORIGIN) / 90.0)) % 4
    corner_side = "left" if turn_deg > 0 else "right"
    watch = (seen is not None and state.watcher is not None and not state.watcher.dead
             and corner_side in state.watcher.cams
             and walls is not None and 60.0 <= abs(turn_deg) <= 240.0)
    yaw_hist = watch_logic.OdoHistory(keep_s=2.0)
    yaw_unw = last_yaw = start_yaw      # без перескока 360 -> 0
    side_hist = watch_logic.SideHold(hold_s=0.0, t0=time.monotonic())
    sign = 1 if turn_deg > 0 else -1
    last_wall_t = None
    last_stage = None
    armed_before = False
    arm_starts = 0
    first_fid = state.watcher.live_info()[corner_side]["fid"] if watch else 0

    def _update_yaw():
        nonlocal yaw_unw, last_yaw
        y = esp_link.get_yaw()
        yaw_unw += angle_err(y, last_yaw)
        last_yaw = y

    def _watch_poll():
        yaw_hist.add(time.monotonic(), yaw_unw)
        for b in state.watcher.poll():
            x1, y1, x2, y2 = b.xyxy
            # по высоте не меньше 1/8 кадра - как требует чтение стоя: рамку,
            # обрезанную краем до полоски, стоя не прочитать
            if (side_hist.frame_ok(b.side, b.t_frame)
                    and b.conf >= CONF_FAST and max(x2 - x1, y2 - y1) >= MIN_BOX_PX
                    and y2 - y1 >= watch_logic.FRAME_PX * 0.125):
                frame_yaw = yaw_hist.at_checked(b.t_frame)
                if frame_yaw is not None:
                    progress = sign * (frame_yaw - start_yaw)
                    quarter = corner_quarter(progress, turn_deg, start_h)
                    if (b.side == corner_side and quarter is not None
                            and CORNER_SIDE_FRAME_FROM_DEG <= quarter[0] <= CORNER_FRAME_TO_DEG):
                        wall_dir, corner_h = corner_frame_wall(quarter, sign)
                        seen.append((b.side, frame_yaw % 360.0, b.xyxy, b.conf, b.cls,
                                     corner_h, wall_dir))
    tag = label if label is not None else f"{target:.1f}°"

    print(f"    поворот {tag}  цель={target:.1f}°  yaw={esp_link.get_yaw():.1f}°")

    # Разгон по времени (smoothstep): пик ускорения = 1.5 * полка / время.
    t_up = 1.5 * TURN_RATE_MAX_DPS / TURN_ACCEL_DPS2
    # Команды колёсам за последние ~100мс - чтобы сравнивать достигнутую
    # скорость с командой, поданной на запаздывание петли раньше, а не с
    # текущей (на разгоне/торможении они заметно расходятся).
    hist = deque()

    t0 = time.monotonic()
    deadline = t0 + TURN_TIMEOUT_S
    t_ramp = t0            # начало разгона; после манёвра выхода - заново
    turn_progress = turn_unstick.TurnProgress(TURN_STUCK_S, TURN_STUCK_RATE_DPS)
    mark_seen = False
    unstick = turn_unstick.Unstick()
    telem_seq = state.telem_seq
    while time.monotonic() < deadline:
        if esp_link.interrupted():
            # Пауза ESP глушит моторы сама. Иначе (пропала телеметрия)
            # последняя команда вращения работала бы до failsafe ESP.
            if not esp_link.get_esp_paused() and not state.lifted.is_set():
                esp_link.brake()
            if watch:
                state.watcher.arm(set())
            return False
        now = time.monotonic()
        elapsed = now - t_ramp
        rate = esp_link.get_yaw_rate()
        _update_yaw()
        if watch:
            _, scan_t, ranges, _ = lidar_walls.get_wall_snapshot()
            fresh = scan_t is not None and 0.0 <= now - scan_t <= CORNER_LIDAR_MAX_AGE_S
            progress = sign * (yaw_unw - start_yaw)
            quarter = corner_quarter(progress, turn_deg, start_h)
            stage = int(progress // 90.0) if quarter is not None else None
            if stage != last_stage:
                last_wall_t = None
                last_stage = stage
            front_wall = quarter is not None and walls[quarter[1]] is True
            side_wall = quarter is not None and walls[(quarter[1] - sign) % 4] is True
            side, frame_allowed, last_wall_t = corner_watch_side(
                quarter[0] if quarter else -1.0, 90.0 * sign,
                front_wall, side_wall, fresh, ranges, last_wall_t, now)
            side_hist.update(now, {corner_side: side is not None and frame_allowed})
            if side is not None and not armed_before:
                arm_starts += 1
            armed_before = side is not None
            state.watcher.arm({side} if side is not None else set())
            _watch_poll()
            if seen:
                mark_seen = True
                break
        # Остаток угла В СТОРОНУ ЦЕЛИ (>0 - крутиться по часовой), и он же с
        # поправкой на запаздывание: к моменту исполнения команды робот уже
        # довернётся на rate * TURN_LEAD_S.
        rem = (turn_deg - (yaw_unw - start_yaw)) if turn_sign == 1 else -angle_err(last_yaw, target)
        rem_eff = rem - rate * TURN_LEAD_S

        tilted = ramps.is_tilted()
        tol = TURN_TILT_TOL_DEG if tilted else config.TURN_TOL_DEG
        done_rate = TURN_TILT_DONE_RATE_DPS if tilted else TURN_DONE_RATE_DPS
        if abs(rem) <= tol and abs(rate) <= done_rate:
            settled += 1
            if settled >= TURN_SETTLE:
                break
        else:
            settled = 0

        # ── Выход из застревания ─────────────────────────────
        # Ступень блокирующая: на её время сдвигаем дедлайн. Разгон заново -
        # только если не закрутился (иначе профиль затормозил бы вращение).
        if abs(rem) > TURN_STUCK_MIN_DEG and elapsed > t_up:
            if turn_progress.stalled(now, rem) and not unstick.exhausted():
                t_m = time.monotonic()
                freed = unstick.next_stage(rem)
                deadline += time.monotonic() - t_m
                if not freed:
                    t_ramp = time.monotonic()
                turn_progress.reset()
                settled = 0
                hist.clear()
                telem_seq = state.telem_seq
                continue
        else:
            turn_progress.reset()

        # ── Профиль угловой скорости ──────────────────────────
        ss = min(1.0, elapsed / t_up)
        ss = ss * ss * (3.0 - 2.0 * ss)
        w_ref = min(TURN_RATE_MAX_DPS * ss,
                    math.sqrt(2.0 * TURN_DECEL_DPS2 * abs(rem_eff)))
        w_ref = math.copysign(w_ref, rem_eff)

        # ── Колёса: прямая подача + поправка по скорости ─────
        spd = w_ref * state.turn_ff + (w_ref - rate) * TURN_RATE_KP
        spd = max(-TURN_MAX_CMS, min(TURN_MAX_CMS, spd))
        # Страховка от залипания: профиль у самой цели просит почти ноль, и
        # если трение успело остановить робота раньше - дожимаем минимумом.
        if (abs(rem) > tol and abs(spd) < TURN_MIN_CMS
                and abs(rate) < done_rate):
            spd = math.copysign(TURN_MIN_CMS, rem)
        # На профиле краткого срыва с места мало: держим момент, пока
        # скорость ниже профиля и до цели больше тормозного пути.
        toward_rate = max(0.0, math.copysign(1.0, rem) * rate)
        brake_deg = max(TURN_STUCK_MIN_DEG, tol + toward_rate * TURN_LEAD_S
                        + toward_rate ** 2 / (2.0 * TURN_DECEL_DPS2))
        if not unstick.hold_turn(spd, w_ref, rate, rem, brake_deg):
            esp_link.motors(spd * config.TURN_SIGN, -spd * config.TURN_SIGN)

        # ── Подстройка подачи ────────────────────────────────
        # Берём команду ~TURN_LEAD_S назад и сегодняшнюю скорость; только на
        # уверенном вращении (медленное - это трение покоя и шум) и в ту же
        # сторону, что команда.
        hist.append((now, spd))
        while len(hist) > 1 and now - hist[1][0] >= TURN_LEAD_S:
            hist.popleft()
        spd_then = hist[0][1]
        if (not unstick.holding and not tilted
                and abs(rate) > 60.0 and abs(spd_then) > TURN_MIN_CMS * 2
                and (rate > 0) == (spd_then > 0)):
            k = abs(spd_then) / abs(rate)
            k = max(TURN_FF_MIN, min(TURN_FF_MAX, k))
            state.turn_ff += (k - state.turn_ff) * TURN_FF_ADAPT

        # TURN_DT (15мс) был длиннее периода телеметрии и накладывал на
        # контур поворота собственное запаздывание; теперь шаг задаёт пакет.
        telem_seq = esp_link.wait_telem(telem_seq, TURN_DT)

    esp_link.settle_brake(0.10)
    done = settled >= TURN_SETTLE
    # При таймауте/метке цель ещё не достигнута. Возврат вдоль случайного
    # курса после отката может снова посадить колёса на тот же профиль.
    if done:
        unstick.return_after_turn()
    if watch:
        _update_yaw()
        _watch_poll()               # кадры, что ещё в обработке
        state.watcher.arm(set())
        if mark_seen:
            state.watcher.wait_idle()  # статическое чтение берёт эту же камеру
        frames = state.watcher.live_info()[corner_side]["fid"] - first_fid
        print(f"    [W] угол {corner_side}: кадров={frames}, "
              f"включений={arm_starts}, рамок={len(seen or ())}")
    print(f"    итог yaw={esp_link.get_yaw():.1f}°  ошибка={angle_err(esp_link.get_yaw(), target):+.1f}°  "
          f"за {time.monotonic() - t0:.2f}с  TURN_FF={state.turn_ff:.3f}"
          + (f"  ступеней выхода={unstick.stage}" if unstick.stage else "")
          + ("  МЕТКА — поворот прерван" if mark_seen else
             "" if done else "  НЕ ДОВЕРНУЛ (таймаут)"))
    return "mark" if mark_seen else done


# После поворота (move=False): смещение меньше этого - учесть в поездке, больше
# - одним подъездом встать в центр (сильно не по центру - видно глазом и
# мешает замеру стен клетки).
ALIGN_MOVE_FROM_MM = 50.0


@timing._timed("доводка")
def align_at_wall(target_yaw, move=True):
    """Доводка вдоль движения: встать в центр клетки по передней стене, а
    если её нет (например, после разворота) - по задней. move=False - не
    подъезжать, а оставить поправку следующей поездке (state.start_offset):
    после поворота подъезд туда-сюда перед стартом - лишнее время. Но если
    смещение не меньше ALIGN_MOVE_FROM_MM - всё равно один подъезд."""
    state.start_offset = None
    on_slope = abs(esp_link.get_pitch() - state.pitch_level) > SLOPE_BACK_GUARD_DEG
    # Inspect an already available fresh scan first; no three-scan pause at
    # every tile and no repeated forward/back corrections for small errors.
    scan_t = state.telem.get("scan_t")
    if scan_t is None or not 0 <= time.monotonic() - scan_t < 0.25 or on_slope:
        return
    w0 = lidar_walls.get_walls()
    f0, b0 = w0["front"], w0["back"]
    need0 = (f0 - config.FRONT_CENTER_MM if f0 is not None and f0 < WALL_THRESH else
             BACK_CENTER_MM - b0 if b0 is not None and b0 < BACK_THRESH else None)
    if need0 is None or abs(need0) < config.ALIGN_MIN_CORRECTION_MM:
        return
    for it in range(1):
        if esp_link.interrupted():
            return
        # Было measure_walls(2) - медиана из 2 это фактически БОЛЬШЕЕ из двух
        # значений (_median берёт sorted[len//2]), а не устойчивая середина.
        # На шумном замере это гоняло доводку туда-обратно (F=116 -> назад
        # 29мм -> следующий замер F=165 -> снова вперёд 20мм). WALL_VOTE_SCANS
        # (3) даёт настоящую медиану и заметно спокойнее.
        w = lidar_walls.measure_walls(config.WALL_VOTE_SCANS)
        f, b = w["front"], w["back"]

        if f is not None and f < WALL_THRESH:
            need = f - config.FRONT_CENTER_MM          # >0: не доехали до центра
            src = f"F={f:.0f}"
        elif b is not None and b < BACK_THRESH and not on_slope:
            need = BACK_CENTER_MM - b           # >0: откатились назад
            src = f"B={b:.0f}"
        else:
            # На наклоне задняя ось - это пол, а не стена (SLOPE_BACK_GUARD_DEG),
            # и вставать по ней значит вставать по полу.
            if on_slope and b is not None and b < BACK_THRESH:
                print(f"    [выр] B={b:.0f}мм на наклоне - игнорирую")
            return                               # не по чему выравниваться

        if abs(need) < max(config.ALIGN_TOL_MM, config.ALIGN_MIN_CORRECTION_MM):
            if it:
                print(f"    [выр] ок, {src}мм")
            return
        if abs(need) > config.ALIGN_MAX_MM:
            print(f"    [выр] {src}мм: {need:+.0f}мм - это не доводка, пропускаю")
            return

        if not move and abs(need) < ALIGN_MOVE_FROM_MM:
            state.start_offset = (need, target_yaw)
            print(f"    [выр] {src}мм: до центра {need:+.0f}мм - учту в поездке")
            return
        print(f"    [выр {it+1}] {src}мм  "
              f"{'вперёд' if need > 0 else 'назад'} {abs(need):.0f}мм")
        driving.creep_mm(need, target_yaw)
