"""Жертвы: осмотр сторон камерами, сообщение о жертве, выдача аптечки."""
from __future__ import annotations

import threading
import time

import watch_logic
from leds import MARGIN_S, VICTIM_STOP_S, blink_pink
from walls import SIDE_THRESH

import config
import state
import timing
import driving
import esp_link
import lidar_walls
import marks
import turning


# Передняя камера включается, только когда стена ближе этого (лидар, от
# центра робота): в центре клетки до передней стены ~145 мм.
FRONT_CAM_WALL_MM = 200.0


# Ход сервы дозатора занимает ~1с (servo.h) - столько ждём после команды,
# прежде чем слать следующую или продолжать движение, иначе следующая же
# команда 'M:' встанет в очередь на ESP за ещё не отработавшим 'K:'.
DISPENSE_MOVE_S = 1.4   # K: на ESP блокирует cmd_task ~1.32с


def dispense(side):
    """side: 'L' или 'R' - высыпает один спасательный комплект из заднего
    бокового отсека (см. servo.h/robot_link.h, команда 'K:<side>')."""
    esp_link._cmd(f"K:{side}")
    time.sleep(DISPENSE_MOVE_S)


def kits_left():
    return sum(config.KITS_PER_COLUMN - n for n in state.kits_used.values())


def dispense_kit():
    """Один комплект из того столбика, где их осталось больше, при
    равенстве - из левого. Так столбики чередуются и расходуются поровну.
    Раньше один комплект всегда шёл из левого, а два - из обоих, и левый
    пустел быстрее на каждую жертву с одним комплектом. Пустые столбики
    не трогаются; вызывающий проверяет kits_left() заранее."""
    side = "L" if state.kits_used["L"] <= state.kits_used["R"] else "R"
    if state.kits_used[side] >= config.KITS_PER_COLUMN:
        side = "R" if side == "L" else "L"
    if state.kits_used[side] >= config.KITS_PER_COLUMN:
        print("    комплекты кончились - выдавать нечего")
        return
    dispense(side)
    state.kits_used[side] += 1
    print(f"    комплект из {'левого' if side == 'L' else 'правого'} столбика "
          f"(выдано: L={state.kits_used['L']} R={state.kits_used['R']})")


FRONT_BACK_CLEAR_MM = 120.0


# Переднюю стену читаем с отъезда: в кадр входит больше стены, и метка у края
# реже обрезана. Назад - не ближе FRONT_BACK_CLEAR_MM к задней стене.
FRONT_VIEW_BACK_MM = 30.0


def _front_back_step(want_mm):
    """Сколько можно отъехать назад (мм), не ближе FRONT_BACK_CLEAR_MM к
    задней стене."""
    w = lidar_walls.measure_walls(1, fresh=True)
    room = want_mm if w["back"] is None else w["back"] - FRONT_BACK_CLEAR_MM
    return max(0.0, min(want_mm, room))


def _scan_front(cx, cy, hdg):
    """Передняя стена передней камерой: по сетке, назад на FRONT_VIEW_BACK_MM,
    прочитать (не читается - _front_reread, так и не прочиталась -
    _decide_unread), обратно в центр клетки; жертва - мигание и комплекты
    сразу (report_victim). Возвращает курс, на котором робот остался."""
    det, wall_dir = state.victim_f, hdg
    g = turning.snap_target()
    turning.turn_to(g, label="по сетке")
    step = _front_back_step(FRONT_VIEW_BACK_MM)
    if step >= 1.0:
        driving.creep_soft(-step, g)
    marks._reset_unread(det)
    try:
        result = det.scan(cx, cy, wall_dir)
    except Exception as e:              # сбой камеры не должен ронять заезд
        print(f"    [WARN] скан front: {e}")
        result = None
    if result is None:
        unread = getattr(det, "last_unread", None)
        if unread is not None or getattr(det, "last_seen", None) is not None:
            result, unread = marks._front_reread(det, (cx, cy), wall_dir, unread)
            if result is None and unread is not None:
                result = marks._decide_unread(det, unread)
            if result is None:
                print("    перёд: похожее на метку не подтвердилось")
    if step >= 1.0:
        driving.creep_soft(step, g)                 # обратно в центр клетки
    if result is None:
        return hdg
    # метку на этой стене передняя прочитала - боковым после поворота её
    # больше не смотреть (ничего не нашла - смотрят, у них кадр шире)
    state.scanned_walls.add((cx, cy, wall_dir))
    # передняя камера смотрит по оси робота - метка в середине отрезка стены
    key, along = watch_logic.world_along((cx, cy), 0, wall_dir, 0.0, config.LEVEL_STRIDE)
    dup = state.marks.near(key, along)
    if dup is not None and dup[2] == "read":
        print(f"    [W] front: {result['label']} - уже прочитана рядом, не репортю")
        return hdg
    state.marks.add(key, along, "read")
    state.victim_log.append({**result, "side": "front", "wall_dir": wall_dir,
                        "ts": time.strftime("%H:%M:%S")})
    if result.get("blink") and not esp_link.get_esp_paused():
        return report_victim(result, "front", wall_dir, hdg)
    return hdg


@timing._timed("камеры")
def scan_sides(cx, cy, hdg, walls_rel=None):
    """Проверяет ОБЕ боковые стены (лево и право по камерам) и, если стена
    есть, сканирует её YOLO на жертву - при находке буквенной метки сразу
    разворачивается и репортит (см. report_victim).

    walls_rel - словарь с уже готовыми 'right'/'left' (True=стена), если он
    уже посчитан вызывающим (не мерить лидар второй раз); иначе меряется
    здесь. Вызывать и после поворота на месте (камеры смотрят уже на другие
    стены той же клетки), и после проезда в новую клетку: стена, которую
    уже сканировала ЛЮБАЯ из камер (_scanned_walls), второй раз не
    сканируется - после разворота на 180° правая и левая просто меняются
    местами, и раньше каждая камера пересматривала стену соседки.

    Возвращает курс, на котором робот остался: после выдачи аптечек он
    стоит спиной к метке и обратно не поворачивается (см. report_victim) -
    вызывающий обязан взять этот курс себе.
    """
    if state.victim_r is None and state.victim_l is None and state.victim_f is None:
        return hdg
    if esp_link.get_esp_paused():
        return hdg
    # Передняя стена - передней камерой, стоя, пока не развернулись (в тупике
    # и перед поворотом её иначе видят только боковые с поворота). Отдельным
    # шагом и первой: она отъезжает от стены, а боковые снимают с выдержкой.
    # Выдала комплекты - робот развёрнут, scan_cell() повторит скан с нового
    # курса.
    if (state.victim_f is not None and (cx, cy, hdg) not in state.front_scanned
            and (cx, cy, hdg) not in state.scanned_walls):
        w = lidar_walls.measure_walls(2)
        if lidar_walls.wall_present(w["front"], FRONT_CAM_WALL_MM):
            state.front_scanned.add((cx, cy, hdg))
            h = _scan_front(cx, cy, hdg)
            if h != hdg or esp_link.get_esp_paused():
                return h
    sides = []
    for side, det in (("right", state.victim_r), ("left", state.victim_l)):
        wall_dir = (hdg + (1 if side == "right" else 3)) % 4
        if det is not None and (cx, cy, wall_dir) not in state.scanned_walls:
            sides.append((side, det, wall_dir))
    if not sides:
        return hdg      # смотреть нечего - и лидар не ждём
    if walls_rel is None:
        known = state.maze.get((cx, cy))
        if (cx, cy) in state.visited and known is not None:
            # Клетка уже снята при приезде - стены берём из карты, а не
            # ждём ещё два оборота лидара после каждого поворота.
            walls_rel = {"right": known[(hdg + 1) % 4] is True,
                         "left": known[(hdg + 3) % 4] is True}
        else:
            w = lidar_walls.measure_walls(2)
            walls_rel = {"right": lidar_walls.wall_present(w["right"], SIDE_THRESH),
                         "left": lidar_walls.wall_present(w["left"], SIDE_THRESH)}
    jobs = [j for j in sides if walls_rel.get(j[0])]
    for _, _, wall_dir in jobs:
        state.scanned_walls.add((cx, cy, wall_dir))

    # Обе камеры - одновременно: у каждой свой детектор (своя модель, своя
    # камера), делить им нечего. Съёмка кадра - это ожидание камеры, а YOLO
    # (torch) отпускает GIL, так что два скана вместе идут почти за время
    # одного, а не за сумму. Репорт - уже по очереди, после обоих сканов:
    # оба кадра сняты с одного и того же места.
    results = {}

    def _scan(side, det, wall_dir):
        marks._reset_unread(det)
        try:
            results[side] = det.scan(cx, cy, wall_dir)
        except Exception as e:   # сбой одной камеры не должен ронять заезд
            print(f"    [WARN] скан {side}: {e}")
            results[side] = None

    threads = [threading.Thread(target=_scan, args=j, daemon=True) for j in jobs[1:]]
    for t in threads:
        t.start()
    if jobs:
        _scan(*jobs[0])
    for t in threads:
        t.join()

    for side, det, wall_dir in jobs:
        result = results.get(side)
        moved = 0.0
        if result is None:
            unread = getattr(det, "last_unread", None)
            if unread is None:
                continue
            # Цель в кадре, а кольца не прочитались: подравняться по ней
            # (в центр кадра, потом сдвиг - другой блик) и прочитать ещё раз.
            g = turning.snap_target()
            result, unread2, moved = marks._reread_aligned(det, (cx, cy), wall_dir, g, 0.0,
                                                     f"скан {side}", config.MARK_READ_TRIES - 1)
            unread = unread2 or unread
            if result is None:
                result = marks._decide_unread(det, unread)
            if abs(moved) >= 1.0:
                driving.creep_soft(-moved, g)       # обратно в центр клетки
        # Та же метка могла быть прочитана на ходу из соседней клетки (или
        # с другой стороны стыка) - второй раз не репортим.
        key, along = marks._mark_world(result, (cx, cy), hdg, wall_dir, moved)
        dup = state.marks.near(key, along)
        if dup is not None and dup[2] == "read":
            print(f"    [W] {side}: {result['label']} в {along:.0f}мм - уже прочитана "
                  f"рядом ({dup[1]:.0f}мм), не репортю")
            continue
        state.marks.add(key, along, "read")
        if esp_link.get_esp_paused():
            # Усыпили посреди скана: находку в лог, но ни мигать, ни
            # разворачиваться - робот должен замереть сразу.
            state.victim_log.append({**result, "side": side,
                                 "wall_dir": wall_dir, "ts": time.strftime("%H:%M:%S")})
            continue
        # В лог для веб-страницы (map_web.py) идёт КАЖДЫЙ непустой
        # результат, включая фейковые когнитивные цели. Разворот, индикация
        # и выдача аптечек - только для настоящих жертв (blink): буквы
        # Φ/Ψ/Ω и когнитивные цели с суммой колец 0/1/2.
        state.victim_log.append({**result, "side": side,
                             "wall_dir": wall_dir, "ts": time.strftime("%H:%M:%S")})
        if result.get("blink"):
            hdg = report_victim(result, side, wall_dir, hdg)
    return hdg


def scan_cell(cx, cy, hdg, walls_rel=None):
    """scan_sides(), пока курс не перестанет меняться: развернувшись спиной
    к метке для выдачи, робот смотрит камерами на две другие стены той же
    клетки - их тоже надо проверить. Каждая стена сканируется один раз
    (_scanned_walls), поэтому цикл конечен. Возвращает итоговый курс."""
    h = scan_sides(cx, cy, hdg, walls_rel)
    while h != hdg and not esp_link.get_esp_paused():
        hdg = h
        h = scan_sides(cx, cy, hdg)
    return h


def _blink_victim(result, side):
    """Индикация лентой (как требуют правила - 5+с неподвижно). Возвращает,
    сколько комплектов выдавать: сколько нужно, но не больше, чем осталось -
    нечего выдать, значит только индикация, она засчитывается и без них."""
    need = config.KITS_BY_TYPE.get(result["type"], 0)
    n_kits = min(need, kits_left())
    short = f" (нужно {need}, в запасе {kits_left()})" if n_kits < need else ""
    print(f"    ▓ {result['label']} ({result['type']}, {side}): индикация "
          f"{VICTIM_STOP_S + MARGIN_S:.1f}с, аптечек={n_kits}{short}")
    if state.leds is not None:
        esp_link.hold_while(blink_pink, state.leds, stop=esp_link.get_esp_paused)
    return n_kits


@timing._timed("жертва")
def report_victim(result, side, wall_dir, hdg):
    """Сначала мигает лентой (робот и так в пределах 15см от стены сбоку -
    SIDE_CENTER_MM=140мм, разворот для этого не нужен). Дальше - только если
    реально есть что высыпать: разворачивается спиной к стене с меткой (та
    стена, что была сбоку по ходу - справа или слева, см. side - после
    поворота на ∓90° оказывается сзади), встаёт к ней по правилам (см.
    BACK_CENTER_MM) и высыпает нужное число аптечек. Обратно НЕ
    поворачивается: раньше робот возвращался на исходный курс, а следующим
    шагом маршрута нередко снова поворачивал туда же - два лишних поворота
    на каждую жертву. Теперь маршрут строится от того курса, на котором
    робот остался. Для unharmed (0 аптечек) разворот и подъезд не нужны
    вовсе - только мигание. Вызывать сразу после scan(), пока робот ещё
    стоит в той же клетке.

    wall_dir - абсолютное направление стены с меткой, hdg - текущий курс.
    Возвращает курс после выдачи.
    """
    n_kits = _blink_victim(result, side)
    if esp_link.get_esp_paused():
        return hdg

    if n_kits == 0:
        return hdg

    # Спиной к метке: курс, противоположный стене. Считаем от ТЕКУЩЕГО
    # курса, а не от стороны камеры - если в клетке две жертвы, после
    # выдачи первой робот уже развёрнут, и вторая может оказаться спереди.
    back_hdg = (wall_dir + 2) % 4
    delta = config.DIR_DELTA[(back_hdg - hdg) % 4]
    if delta != 0:
        ok = turning.turn(delta)
        if esp_link.get_esp_paused():
            return hdg
        if not ok and turning.gyro_heading() != back_hdg:
            print("    не развернулся спиной к метке - комплекты не выдаю")
            return turning.gyro_heading()
        hdg = back_hdg
        turning.align_at_wall(turning.snap_target())     # "зад" - к метке, по правилам

    for _ in range(n_kits):
        if esp_link.get_esp_paused():
            return hdg
        dispense_kit()
    return hdg


@timing._timed("жертва")
def _report_on_move(result, side, wall_dir, hdg, target_yaw):
    """Жертва, найденная посреди клетки (см. _mark_stop): мигание; если есть
    что выдавать - спиной к стене, выдача и поворот ровно обратно на
    target_yaw, потому что поездка продолжается. align_at_wall не нужен:
    робот и так в ~14см от стены. Возвращает причину бросить поездку или None."""
    n_kits = _blink_victim(result, side)
    r = marks._interrupt_reason()
    if r or n_kits == 0:
        return r
    ok = turning.turn(config.DIR_DELTA[((wall_dir + 2) % 4 - hdg) % 4])
    r = marks._interrupt_reason()
    if r:
        return r
    if ok:
        for _ in range(n_kits):
            r = marks._interrupt_reason()
            if r:
                return r
            dispense_kit()
    else:
        print("    не развернулся спиной к метке - комплекты не выдаю")
    for _ in range(2):
        if turning.turn_to(target_yaw):
            return None
        r = marks._interrupt_reason()
        if r:
            return r
    print("    [WARN] не вернулся на курс - доведёт руль drive_cells")
    return None
