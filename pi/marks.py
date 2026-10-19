"""Метки жертв, замеченные не стоя в центре клетки.

Три случая, по функции на каждый:
  - на ходу: watch_marks_while_driving() - шаг поездки drive_cells, заметил
    метку у стены - тормозим к ней; _mark_stop() - стоя у неё: найти,
    подъехать, прочитать, выдать комплекты;
  - на повороте: _turn_marks() - доворот на курс кадра, где метка была ближе
    всего к перпендикуляру, и чтение стоя (шаги - в её докстринге);
  - перёд не читается: _front_reread() - отъехать, довернуть корпус к метке,
    прочитать ещё раз.
Не прочиталась целиком - решаем по надёжным кольцам (_decide_unread): робот
не остаётся с «не знаю».
"""
from __future__ import annotations

import time

import watch_logic
from victim_detect import CONF_FAST, MIN_BOX_PX, RING_SCORE, SUM_TO_TYPE
from walls import SIDE_THRESH

import config
import state
import timing
import driving
import esp_link
import lidar_walls
import maze_map
import turning
import victims


def watch_marks_while_driving(trip):
    """Наблюдатель меток: заметил метку у стены - тормозим к ней."""
    # ── Наблюдатель меток ─────────────────────────────────
    # Только на ровном (не горка, не всплеск, не ассист) и только к
    # стене. Заметил - тормозим тем же профилем к stop_point (ниже, см.
    # mark_first), а наблюдатель гасим сразу: камера вернётся к обычной
    # выдержке, пока робот ещё тормозит.
    if trip.watcher is not None:
        if trip.watcher.dead:
            trip.watcher, trip.watch_armed = None, False
        else:
            sides = trip.side_hold.update(trip.now_m, {
                "right": lidar_walls.wall_present(trip.walls["right"], SIDE_THRESH),
                "left": lidar_walls.wall_present(trip.walls["left"], SIDE_THRESH)})
            flat = watch_logic.watch_allowed(trip.ramp_state, trip.bump_on, trip.assist_mode,
                                             trip.pitch_delta, config.OBSTACLE_REARM_DEG, trip.wall_forced)
            trip.watch_armed = trip.watcher.arm(sides if flat and trip.vmark is None else set())
            for b in trip.watcher.poll():
                if not trip.side_hold.frame_ok(b.side, b.t_frame):
                    # Кадр снят, когда стены с этой стороны не было:
                    # метка на дальней стене, через проём.
                    print(f"\n    [W] {b.side} {b.cls} {b.conf:.2f} - в момент кадра "
                          f"стены нет, не беру  {b.image or ''}")
                    continue
                m = _watch_decide(b, trip.cell, trip.hdg, trip.odo_t, trip.vmark is not None)
                if m is not None:
                    # Тормозим К МЕТКЕ: встать с ней в центре кадра, тем
                    # же профилем. Не успеваем - как можно раньше и
                    # подъезд назад (см. watch_logic.brake_target_cm).
                    m["stop_cm"], late = watch_logic.brake_target_cm(
                        m["pos_mm"], trip.traveled_cm, trip.fwd, config.MARK_BRAKE_DECEL_CMS2, config.STOP_LEAD_S)
                    trip.vmark = m
                    trip.watch_armed = trip.watcher.arm(set())
                    print(f"    метка на ходу: торможу {'(поздно) ' if late else 'к метке '}"
                          f"через {(m['stop_cm'] - trip.traveled_cm) * 10:.0f}мм")


def _interrupt_reason():
    """Причина бросить поездку посреди остановки у метки - те же, что
    проверяет такт drive_cells."""
    if state.lifted.is_set():
        return "lifted"
    if esp_link.get_esp_paused():
        return "paused"
    if not esp_link.telem_fresh():
        return "link_lost"
    return None


def _watch_decide(b, cell, hdg, odo_t, busy):
    """Рамка наблюдателя -> метка, ради которой тормозить ({"side", "pos_mm"}),
    или None. Каждое срабатывание - строка [W] с картинкой: по ним подбирают
    порог, лишняя остановка стоит ~2с."""
    x1, y1, x2, y2 = b.xyxy
    side_px = max(x2 - x1, y2 - y1)
    t_cm = odo_t.at(b.t_frame) or 0.0
    off = watch_logic.px_to_mm(watch_logic.mark_center_y(b.xyxy))
    pos = watch_logic.mark_pos_mm(t_cm * 10.0, off)
    key, along = watch_logic.world_along(cell, hdg, watch_logic.wall_of(hdg, b.side),
                                         pos, config.LEVEL_STRIDE)
    stop, why = watch_logic.decide_stop(b.conf, side_px, key, along, state.marks, busy,
                                        CONF_FAST, MIN_BOX_PX)
    print(f"\n    [W] {b.side} {b.cls} {b.conf:.2f} {side_px:.0f}px  путь {t_cm * 10:.0f}мм  "
          f"метка {pos:+.0f}мм (стена {key} {along:.0f}мм) → {why}  {b.image or ''}")
    return {"side": b.side, "pos_mm": pos} if stop else None


def _reject_mark(cell, hdg, side, pos_mm):
    """Наблюдатель сработал, а стоя не подтвердилось - больше здесь не стоим."""
    key, along = watch_logic.world_along(cell, hdg, watch_logic.wall_of(hdg, side),
                                         pos_mm, config.LEVEL_STRIDE)
    state.marks.add(key, along, "rejected")


@timing._timed("камеры")
def _mark_stop(m, cell, hdg, target_yaw, base_cm):
    """Остановка на метку, замеченную на ходу (см. drive_cells): найти её стоя,
    подъехать так, чтобы была целиком в кадре, прочитать read_wall() и
    приписать клетке. Робот уже затормозил профилем; одометрия не обнулялась,
    base_cm - пройдено от центра исходной клетки. Возвращает причину бросить
    поездку ('paused'/'lifted'/'link_lost') или None - ехать дальше."""
    side = m["side"]
    det = state.victim_r if side == "right" else state.victim_l
    l0, r0 = esp_link.get_odo()

    def trav_mm():
        cl, cr = esp_link.get_odo()
        return (base_cm + ((cl - l0) + (cr - r0)) / 2.0) * 10.0

    esp_link.settle_brake(0.06)
    if state.watcher is not None:
        state.watcher.wait_idle()
    r = _interrupt_reason()
    if r or det is None:
        return r

    def look():
        try:
            return det.best_fast()
        except Exception as e:
            print(f"    [WARN] кадр стоя ({side}): {e}")
            return None

    # 1. Где метка сейчас. Обычно робот проехал её на тормозном пути и в
    # кадре её уже нет - тогда подъезд по координате, запомненной на ходу.
    found = look()
    est = m["pos_mm"] - watch_logic.mark_pos_mm(trav_mm(), 0.0)
    creep, src = watch_logic.creep_to_mark(None if found is None else found[2], est,
                                           config.ALIGN_TOL_MM)
    if creep is None:
        print(f"    метка: нужен подъезд {est:+.0f}мм ({src}) - не верю, отбраковываю")
        _reject_mark(cell, hdg, side, m["pos_mm"])
        return None
    # 2. Подъезд
    if creep:
        print(f"    метка: подъезд {creep:+.0f}мм по {src}")
        driving.creep_soft(creep, target_yaw)
        r = _interrupt_reason()
        if r:
            return r
        if found is not None:
            again = look()
            if again is not None:
                before = abs(watch_logic.px_to_mm(sum(found[2]) / 2.0))
                after = abs(watch_logic.px_to_mm(sum(again[2]) / 2.0))
                if after > before:
                    print(f"    [WARN] после подъезда метка дальше от центра кадра "
                          f"({before:.0f} -> {after:.0f}мм) - знак оси камеры? "
                          f"(watch_logic.CAM_CALIB_Y_MM)")
    # 3. Чтение - та же логика, что стоя в центре, без дедупа по стене
    trav = trav_mm()
    tcell = maze_map._cell_at(cell, hdg, watch_logic.cell_index(trav))
    wall_dir = watch_logic.wall_of(hdg, side)
    # Читаем, только если с этой стороны стена: по карте её нет - не читаем,
    # и стоя лидар должен её видеть. Иначе камера смотрит в проём и метка -
    # на дальней стене.
    no_wall = _no_wall_reason(tcell, wall_dir, side)
    if no_wall:
        print(f"    метка: стены {config._DIR_NAME[wall_dir]} у {tcell} {no_wall} нет - "
              f"это дальняя стена, не читаю")
        _reject_mark(cell, hdg, side, m["pos_mm"])
        return _interrupt_reason()
    what = f"чтение метки ({side})"
    _reset_unread(det)
    result = _read_wall(det, tcell, wall_dir, what, expect=True)
    unread = getattr(det, "last_unread", None)
    if result is None:
        result, unread2, _ = _reread_aligned(det, tcell, wall_dir, target_yaw, 0.0,
                                             what, config.MARK_READ_TRIES - 1)
        unread = unread2 or unread
        trav = trav_mm()
    if result is None and unread is not None:
        result = _decide_unread(det, unread)
    if result is None:
        print(f"    метка не подтвердилась - место отбраковано ({side}, {trav:.0f}мм)")
        _reject_mark(cell, hdg, side, m["pos_mm"])
        return _interrupt_reason()
    key, along = _mark_world(result, cell, hdg, wall_dir, trav)
    state.marks.add(key, along, "read")
    state.scanned_walls.add((tcell[0], tcell[1], wall_dir))
    det.scanned.add((tcell[0], tcell[1], wall_dir))
    state.victim_log.append({**result, "side": side, "wall_dir": wall_dir, "on_move": True,
                        "ts": time.strftime("%H:%M:%S")})
    print(f"    метка на ходу: {result['label']} → клетка {tcell}, стена "
          f"{config._DIR_NAME[wall_dir]} (проехали {trav:.0f}мм)")
    r = _interrupt_reason()
    if r:
        return r
    if result.get("blink"):
        return victims._report_on_move(result, side, wall_dir, hdg, target_yaw)
    return None


def _no_wall_reason(tcell, wall_dir, side):
    """Почему у метки нет стены: "по карте", "по лидару" (замер стоя) или
    None - стена есть, читать можно."""
    known = state.maze.get(tcell)
    if known is not None and known[wall_dir] is False:
        return "по карте"
    if not lidar_walls.wall_present(lidar_walls.measure_walls(config.WALL_VOTE_SCANS)[side], SIDE_THRESH):
        return "по лидару"
    return None


def _mark_world(result, cell, hdg, wall_dir, traveled_mm):
    """(ключ стены, мм вдоль неё) прочитанной метки - по её рамке на кадре;
    traveled_mm - сколько робот проехал от центра cell курсом hdg (стоя в
    центре - 0)."""
    y1, y2 = result["box_y"]
    pos = watch_logic.mark_pos_mm(traveled_mm, watch_logic.px_to_mm((y1 + y2) / 2.0))
    return watch_logic.world_along(cell, hdg, wall_dir, pos, config.LEVEL_STRIDE)


# Метка в центре кадра, а не читается (блик, трещина) - сдвиг на столько,
# попеременно вперёд и назад: другой угол - другой блик.
MARK_NUDGE_MM = 25.0
# Дальше этого от места, где начали, вдоль стены не уезжаем: впереди может
# быть стена клетки.
MARK_ALIGN_MAX_MM = 80.0


def _read_wall(det, cell, wall_dir, what, expect=False):
    try:
        return det.read_wall(cell[0], cell[1], wall_dir, expect=expect)
    except Exception as e:
        print(f"    [WARN] {what}: {e}")
        return None


def _reread_aligned(det, cell, wall_dir, yaw, est_mm, what, tries, expect=True):
    """Метка не прочиталась - подровняться по ней и прочитать ещё раз, до
    tries раз. Метка в кадре - подъезд вдоль стены, чтобы встала в центр
    кадра; её нет в кадре - по оценке est_mm (сколько ехать, чтобы она
    оказалась напротив камеры); уже в центре - сдвиг MARK_NUDGE_MM. Едем
    курсом yaw (по сетке - вдоль стены). Возвращает (метка или None,
    det.last_unread последнего чтения с целью или None, сколько проехали мм:
    вернуть робота на место - забота вызывающего)."""
    moved, unread = 0.0, getattr(det, "last_unread", None)
    for k in range(tries):
        if _interrupt_reason():
            break
        try:
            found = det.best_fast()
        except Exception as e:
            print(f"    [WARN] {what}: кадр стоя: {e}")
            found = None
        creep, src = watch_logic.creep_to_mark(None if found is None else found[2],
                                               est_mm - moved, config.ALIGN_TOL_MM, max_mm=1e9)
        if creep is None or abs(creep) < config.ALIGN_TOL_MM:
            creep, src = (MARK_NUDGE_MM if k % 2 == 0 else -MARK_NUDGE_MM), "сдвиг"
        creep = max(-MARK_ALIGN_MAX_MM, min(MARK_ALIGN_MAX_MM, moved + creep)) - moved
        if abs(creep) >= 1.0:
            print(f"    {what}: не прочиталась - подравниваюсь по метке {creep:+.0f}мм ({src})")
            driving.creep_soft(creep, yaw)
            moved += creep
        result = _read_wall(det, cell, wall_dir, what, expect=expect)
        if result is not None:
            return result, None, moved
        unread = getattr(det, "last_unread", None) or unread
    return None, unread, moved


# Передняя камера видит метку, а прочитать не может (обрезана краем кадра,
# блик, неуверенная рамка): столько повторных чтений. Перед каждым - встать
# по сетке, отъехать назад на FRONT_BACK_MM (не ближе FRONT_BACK_CLEAR_MM к
# задней стене: полкорпуса 97 мм + запас), чтобы метка вошла в кадр целиком,
# и довернуть корпус к ней, если её центр дальше FRONT_AIM_TOL_DEG от оси
# камеры. Отъезд на 2x50 мм в заезде 18.10 делал мишень такой мелкой, что
# чёрное кольцо читалось зелёным - поэтому шаг 30 мм.
FRONT_READ_TRIES = 2
FRONT_AIM_TOL_DEG = 2.0
FRONT_BACK_MM = 30.0


def _front_reread(det, cell, wall_dir, unread):
    """Повторное чтение передней стены (см. FRONT_READ_TRIES). Потом
    обратно на курс сетки и на место. -> (метка или None, last_unread)."""
    g = turning.snap_target()
    backed, result = 0.0, None
    for _ in range(FRONT_READ_TRIES):
        if _interrupt_reason():
            break
        turning.turn_to(g, label="по сетке")
        step = victims._front_back_step(FRONT_BACK_MM)
        if step >= 1.0:
            print(f"    перёд: метка не читается - назад {step:.0f}мм")
            driving.creep_soft(-step, g)
            backed += step
        else:
            print("    перёд: метка не читается, сзади стена - назад некуда")
        try:
            ang = det.locate()
        except Exception as e:
            print(f"    [WARN] перёд: где метка: {e}")
            ang = None
        if ang is not None and abs(ang) >= FRONT_AIM_TOL_DEG:
            print(f"    перёд: метка {ang:+.0f}° от оси камеры - доворачиваю к ней")
            turning.turn_to((esp_link.get_yaw() + ang) % 360.0, label="к метке")
        result = _read_wall(det, cell, wall_dir, "перёд", expect=True)
        if result is not None:
            break
        unread = getattr(det, "last_unread", None) or unread
    if abs(turning.angle_err(esp_link.get_yaw(), g)) > config.TURN_TOL_DEG:
        turning.turn_to(g, label="по сетке")
    if backed >= 1.0:
        driving.creep_soft(backed, g)               # обратно в центр клетки
    return result, unread


def _reset_unread(det):
    """Перед серией чтений одной метки: история неполных чтений - с нуля."""
    if det is not None and hasattr(det, "unread_hist"):
        det.unread_hist = []


def _decide_unread(det, unread):
    """Мишень в кадре, а кольца так и не прочитались ни разу целиком -
    решение всё равно нужно (робот не остаётся с «не знаю»): каждое кольцо
    - из самого надёжного из чтений этой серии (watch_logic.guess_rings).
    Формат - как у прочитанной метки, плюс 'guessed'."""
    hist = list(getattr(det, "unread_hist", None) or []) or [unread]
    rings = watch_logic.guess_rings(hist)
    checksum = sum(RING_SCORE.get(c) or 0 for c in rings)
    vtype = SUM_TO_TYPE.get(checksum, "fake")
    print(f"    кольца так и не прочитались ({len(hist)} чтен.) - по надёжным: "
          f"{rings} = {checksum:+d} → {vtype}")
    return {**unread, "rings": rings, "checksum": checksum, "type": vtype,
            "label": f"{checksum:+d}", "blink": vtype in config.KITS_BY_TYPE, "guessed": True}


def _corner_dup(cell, wall_dir, along, cls):
    """Та же метка уже прочитана на соседней стене у того же угла клетки
    (см. watch_logic.CORNER_MM)."""
    for key, a in watch_logic.corner_neighbours(cell, wall_dir, along, config.LEVEL_STRIDE):
        dup = state.marks.near(key, a, cls)
        if dup is not None and dup[2] == "read":
            return True
    return False


def _best_turn_frames(cx, cy, seen):
    """Шаг 1 доворота: из кадров, снятых на повороте, для каждой (камера,
    стена) - рамка, снятая ближе всего к перпендикуляру к стене.
    -> {(side, wall_dir): (px, yf, xyxy, h, phi, cls)}"""
    best = {}
    for side, yf, xyxy, conf, cls in seen:
        rel = turning.angle_err(yf, state.YAW_ORIGIN)
        h = int(round(rel / 90.0)) % 4
        phi = turning.angle_err(rel, 90.0 * h)
        if not watch_logic.TURN_MARK_PHI_MIN <= abs(phi) <= watch_logic.TURN_MARK_PHI_MAX:
            continue                      # диагональ (какая стена - неясно) или сетка
        wall_dir = watch_logic.wall_of(h, side)
        if (cx, cy, wall_dir) in state.turn_mark_tried:
            continue
        px = max(xyxy[2] - xyxy[0], xyxy[3] - xyxy[1])
        k = (side, wall_dir)
        if k not in best or abs(phi) < abs(best[k][4]):
            best[k] = (px, yf, xyxy, h, phi, cls)
    return best


def _turn_mark_place(cx, cy, side, wall_dir, px, xyxy, h, phi, cls):
    """Шаг 2 доворота: где метка на стене и стоит ли к ней доворачиваться.
    -> (pos, key, along) или None - пропустить (почему - в логе)."""
    pos = watch_logic.turned_mark_pos(phi, side, watch_logic.px_to_mm(
        watch_logic.mark_center_y(xyxy)))
    if pos is None:
        return None
    key, along = watch_logic.world_along((cx, cy), h, wall_dir, pos, config.LEVEL_STRIDE)
    if abs(along - watch_logic.wall_center_along((cx, cy), wall_dir, config.LEVEL_STRIDE)) \
            > watch_logic.CELL_MM / 2:
        # Луч лёг на эту стену за углом клетки: метка на соседней стене
        # (с поворота угол виден с обеих сторон).
        print(f"    [W] на повороте: {side} {px:.0f}px, стена {config._DIR_NAME[wall_dir]} "
              f"{along:.0f}мм - за углом клетки, пропускаю")
        return None
    if _corner_dup((cx, cy), wall_dir, along, cls):
        print(f"    [W] на повороте: {side} {px:.0f}px у угла - уже прочитана "
              f"на соседней стене, пропускаю")
        return None
    known = state.maze.get((cx, cy))
    if known is None or known[wall_dir] is not True:
        # Стены с этой стороны клетки нет: камера смотрит в проём на
        # стену соседней клетки или дальше. Метку там не читаем.
        print(f"    [W] на повороте: {side} {px:.0f}px, стены {config._DIR_NAME[wall_dir]} "
              f"по карте нет - дальняя стена, пропускаю")
        return None
    if state.marks.near(key, along, cls) is not None:
        return None                       # уже читали (или отбраковали) рядом
    return pos, key, along


def _read_turn_mark(det, cx, cy, side, wall_dir, h, pos, key, along):
    """Шаг 3 доворота: робот уже довернулся на курс кадра - прочитать метку.
    Не прочиталась под углом - встать по сетке, подровняться вдоль стены,
    прочитать оттуда и вернуться в центр клетки.
    -> (метка или None, неполное чтение или None, key, along)"""
    what = f"чтение метки ({side})"
    _reset_unread(det)
    result = _read_wall(det, (cx, cy), wall_dir, what, expect=True)
    unread = None
    if result is not None:
        phi_now = turning.angle_err(turning.angle_err(esp_link.get_yaw(), state.YAW_ORIGIN), 90.0 * h)
        pos2 = watch_logic.turned_mark_pos(
            phi_now, side, watch_logic.px_to_mm(sum(result["box_y"]) / 2.0))
        if pos2 is not None:
            key, along = watch_logic.world_along((cx, cy), h, wall_dir, pos2, config.LEVEL_STRIDE)
    else:
        # Под углом не прочиталась (метка у края кадра, кольца наискось):
        # встать по сетке, подъехать вдоль стены, чтобы метка оказалась
        # напротив камеры, прочитать оттуда - и обратно в центр клетки.
        unread = getattr(det, "last_unread", None)
        g = turning.snap_target()
        print(f"    {what}: под углом не прочиталась - встаю по сетке и подравниваюсь")
        turning.turn_to(g, label="по сетке")
        result, unread2, moved = _reread_aligned(det, (cx, cy), wall_dir, g,
                                                 pos + watch_logic.CAM_BACK_MM,
                                                 what, config.MARK_READ_TRIES)
        unread = unread2 or unread
        src = result or unread2
        if src is not None:
            key, along = _mark_world(src, (cx, cy), h, wall_dir, moved)
        if abs(moved) >= 1.0:
            driving.creep_soft(-moved, g)
    return result, unread, key, along


def _record_turn_mark(det, cx, cy, side, wall_dir, h, key, along, result, unread):
    """Шаг 4 доворота: записать прочитанную метку и выдать комплекты.
    -> (True, курс после выдачи) или (False, None) - комплектов не выдавали."""
    cls_now = (result or unread or {}).get("class")
    dup = state.marks.near(key, along, cls_now)
    if (dup is not None and dup[2] == "read") or \
            _corner_dup((cx, cy), wall_dir, along, cls_now):
        print(f"    [W] метка в {along:.0f}мм - уже прочитана, не репортю")
        return False, None
    if result is None and unread is not None:
        result = _decide_unread(det, unread)
    if result is None:
        # кадр на повороте - слабое свидетельство: место не
        # отбраковываем, наблюдатель на ходу и скан стоя ещё могут её
        # прочитать
        print("    метка с поворота не подтвердилась")
        return False, None
    state.marks.add(key, along, "read", result.get("class"))
    state.victim_log.append({**result, "side": side, "wall_dir": wall_dir,
                        "ts": time.strftime("%H:%M:%S")})
    print(f"    метка с поворота: {result['label']} → стена {config._DIR_NAME[wall_dir]} у ({cx},{cy})")
    if result.get("blink"):
        return True, victims.report_victim(result, side, wall_dir, h)
    return False, None


def _turn_marks(cx, cy, seen):
    """Доворот на метки, замеченные наблюдателем на повороте в клетке (cx, cy)
    (см. turn_to(seen=...)). По шагам:
      1. _best_turn_frames  - для каждой стены рамка, снятая ближе всего к
         перпендикуляру (watch_logic.TURN_MARK_PHI_MIN..MAX);
      2. _turn_mark_place   - где метка; пропуск, если за углом, уже прочитана,
         стены по карте нет или рядом уже читали;
      3. довернуться на курс того кадра (под ним метка в центре кадра) и
         _read_turn_mark   - прочитать стоя (read_wall);
      4. _record_turn_mark  - записать и выдать комплекты.
    Возвращает курс сетки после выдачи комплектов или None (курс не менялся;
    робот может стоять не по сетке - вызывающий доворачивает сам)."""
    best = _best_turn_frames(cx, cy, seen)
    out_hdg = None
    for (side, wall_dir), (px, yf, xyxy, h, phi, cls) in best.items():
        det = state.victim_r if side == "right" else state.victim_l
        if det is None or esp_link.get_esp_paused():
            continue
        place = _turn_mark_place(cx, cy, side, wall_dir, px, xyxy, h, phi, cls)
        if place is None:
            continue
        pos, key, along = place
        state.turn_mark_tried.add((cx, cy, wall_dir))
        print(f"    [W] на повороте: {side} {px:.0f}px под {phi:+.0f}°, стена "
              f"{config._DIR_NAME[wall_dir]} {along:.0f}мм - доворачиваю на {yf:.0f}°")
        turning.turn_to(yf, label="к метке")
        result, unread, key, along = _read_turn_mark(det, cx, cy, side, wall_dir, h, pos, key, along)
        reported, hdg_after = _record_turn_mark(det, cx, cy, side, wall_dir, h, key, along,
                                                result, unread)
        if reported:
            out_hdg = hdg_after
    if out_hdg is not None:
        if abs(turning.angle_err(esp_link.get_yaw(), turning.snap_target())) > config.TURN_TOL_DEG:
            turning.turn_to(turning.snap_target(), label="по сетке")     # без комплектов стоим, как читали
        # курс - по гироскопу: после первой метки робот мог довернуть ко
        # второй, и курс выдачи комплектов первой уже не тот
        out_hdg = turning.gyro_heading()
    return out_hdg
