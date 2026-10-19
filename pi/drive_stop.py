"""Шаги поездки: где тормозить и что делать, когда доехали."""
from __future__ import annotations

import time

import watch_logic

import config
import state
import driving
import esp_link
import marks


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


def check_arrived(trip):
    """Доехали: стоп у метки (и дальше), в центре клетки или перед препятствием."""
    if (trip.stopping or trip.mark_first) and trip.rem_eff <= 0.0 and trip.ramp_state != 'cand':
        if trip.vmark is not None and not (trip.wall_forced
                                      and trip.traveled_cm < trip.forced_from_cm + trip.cell_cm * 0.5):
            # Стоим у метки (или в центре, если он был ближе): читаем её
            # здесь, потом - та же поездка дальше, без Z и без новой цели.
            t_stop = time.monotonic()
            out = marks._mark_stop(trip.vmark, trip.cell, trip.hdg, trip.target_yaw, trip.traveled_cm)
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
            trip.cl, trip.cr = esp_link.get_odo()
            trip.traveled_cm += ((trip.cl - trip.last_l) + (trip.cr - trip.last_r)) / 2.0
            trip.last_l, trip.last_r = trip.cl, trip.cr
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
            left_cm = trip.boundary_cm - trip.traveled_cm
            print(f"    продолжаю: пауза {pause_s:.1f}с, до центра {left_cm * 10:.0f}мм")
            if trip.stopping and left_cm * 10.0 < config.ALIGN_TOL_MM:
                # Уже в центре или подъезд увёл за него - остаток доводкой.
                if abs(left_cm) > 0.2:
                    driving.creep_mm(left_cm * 10.0, trip.target_yaw)
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
