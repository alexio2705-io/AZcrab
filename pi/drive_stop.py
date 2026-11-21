"""Шаги поездки: где тормозить и что делать, когда доехали."""
from __future__ import annotations

import time

import watch_logic

import config
import state
import driving
import esp_link
import marks
import drive_checks
import lidar_walls
from continuous import stop_suspended


def aim_at_mark(trip):
    """Куда тормозить: к метке, если она ближе цели."""
    # Остаток с поправкой на запаздывание петли: одометрия отстаёт, и на
    # ходу робот уже дальше, чем показывает. Поправка пропорциональна
    # скорости, поэтому у самой цели она сама сходит на нет.
    trip.rem_eff = trip.rem - trip.fwd * config.STOP_LEAD_S

    # Точка остановки у метки ближе цели - тормозим к ней. Иначе робот
    # штатно встанет в центре (или у стены), и метку читаем там же.
    trip.mark_first = trip.vmark is not None and trip.vmark["stop_cm"] - trip.traveled_cm < trip.rem
    if trip.mark_first:
        trip.rem = trip.vmark["stop_cm"] - trip.traveled_cm
        trip.rem_eff = trip.rem - trip.fwd * config.STOP_LEAD_S


def _left_to_center_cm(trip):
    """Сколько до центра клетки после остановки у метки. Передняя стена,
    видная стоя в пределах полклетки, точнее колёс - то же правило, что на
    ходу (drive_front: расхождение не больше ALIGN_MAX_MM). Заезд 24.10:
    правые колёса проскальзывали, по колёсам "за центром 122мм", по стене -
    в центре; робот отъехал назад, и карта сдвинулась на клетку."""
    left = trip.boundary_cm - trip.traveled_cm
    f = lidar_walls.get_walls()["front"]       # свежий: робот стоит после подъезда
    if f is not None and f < config.FRONT_CENTER_MM + config.CELL_MM / 2:
        wall_left = (f - config.FRONT_CENTER_MM) / 10.0
        if abs(wall_left - left) <= config.ALIGN_MAX_MM / 10.0:
            return wall_left
    return left


def check_arrived(trip):
    """Доехали: стоп у метки (и дальше), в центре клетки или перед препятствием."""
    if (trip.stopping or trip.mark_first) and trip.rem_eff <= 0.0 and not stop_suspended(trip):
        if trip.vmark is not None and not (trip.wall_forced
                                      and trip.traveled_cm < trip.forced_from_cm + trip.cell_cm * 0.5):
            # Стоим у метки (или в центре, если он был ближе): читаем её
            # здесь, потом - та же поездка дальше, без Z и без новой цели.
            t_stop = time.monotonic()
            out = marks._mark_stop(trip.vmark, trip.cell, trip.hdg, trip.target_yaw,
                                   trip.traveled_cm, trip.odometry.last)
            trip.vmark = None
            if out in ("lifted", "paused"):
                trip.reason = out
                return "quit"
            if out == "link_lost":
                trip.reason = out
                return "stop"
            pause_s = time.monotonic() - t_stop
            # Таймер клетки и сторожа застревания - на длительность паузы,
            # иначе после неё сразу timeout/odo_fault.
            (trip.t_cell, trip.last_progress_t, trip.assist_gap_until, trip.assist_t0, trip.assist_total_t0,
             trip.assist_progress_t) = watch_logic.shift_timers(
                (trip.t_cell, trip.last_progress_t, trip.assist_gap_until, trip.assist_t0, trip.assist_total_t0,
                 trip.assist_progress_t), pause_s)
            # Подъезд и развороты уже в энкодерах - добираем их в путь сразу.
            drive_checks.integrate_motion(trip)
            trip.last_progress_cm = trip.traveled_cm
            trip.t0 = trip.last_loop_t = time.monotonic()     # новый плавный разгон с места
            trip.fwd = trip.last_steer = 0.0
            trip.odo_hist.clear()
            trip.f_anchor = None                         # перед - заново со свежего оборота
            trip.last_seq = state.scan_cnt
            # Обороты за время стоянки сняты с подъезда и поворотов к
            # аптечкам - для стен впереди они не годятся.
            trip.scan_mark = state.scan_cnt
            trip.side_hold.reset()
            queued = getattr(trip, 'mark_queue', [])
            if queued and queued[0]['pos_mm'] <= trip.boundary_cm * 10 + config.ALIGN_TOL_MM:
                # A second wall can have a mark at the same stopping point.
                # Service it before ending this trip; do not cross a turn target.
                trip.vmark = queued.pop(0)
                trip.vmark['stop_cm'], _ = watch_logic.brake_target_cm(
                    trip.vmark['pos_mm'], trip.traveled_cm, 0,
                    config.MARK_BRAKE_DECEL_CMS2, config.STOP_LEAD_S)
                trip.telem_seq = esp_link.wait_telem(trip.telem_seq)
                return 'next_tick'
            # Камера и выдача комплектов занимают секунды. Первый кадр лидара
            # после возврата на курс может ещё не видеть переднюю стену;
            # дождёмся нового, прежде чем решать, нужен ли короткий подъезд.
            if trip.stopping:
                lidar_walls.wait_scan(1, timeout=0.5)
            left_cm = _left_to_center_cm(trip)
            print(f"    продолжаю: пауза {pause_s:.1f}с, до центра {left_cm * 10:.0f}мм")
            if trip.stopping and left_cm * 10.0 < config.ALIGN_TOL_MM:
                # В пределах допуска стоим; если проехали центр дальше
                # допуска, возвращаемся назад перед поворотом.
                if left_cm * 10.0 <= -config.ALIGN_TOL_MM:
                    driving.creep_mm(left_cm * 10.0, trip.target_yaw)
                    if state.lifted.is_set():
                        trip.reason = "pose_lost"
                        return "stop"
                trip.cells += 1
                return "stop"
            trip.telem_seq = esp_link.wait_telem(trip.telem_seq)
            return "next_tick"
        if trip.wall_forced and trip.traveled_cm < trip.forced_from_cm + trip.cell_cm * 0.5:
            # Встали перед тем, что внезапно оказалось близко, не доехав
            # и до середины пути: это не стена следующей клетки (она
            # была бы на границе, и по ней остановка шла бы штатно), а
            # препятствие - например, кирпич у входа в неё. Клетку НЕ
            # засчитываем: раньше она засчитывалась, и робот считал, что
            # стоит в следующей клетке, недоехав до неё 10-25см.
            trip.reason = "front_obstacle"
            f_txt = "—" if trip.f_est is None else f"{trip.f_est:.0f}"
            print(f"\n    препятствие впереди: f={f_txt}мм, "
                  f"проехали {(trip.traveled_cm - trip.forced_from_cm)*10:.0f}мм")
            return "stop"
        trip.cells += 1
        if trip.at_wall:
            print(f"\n    у стены: f={trip.f_est:.0f}мм")
        return "stop"
