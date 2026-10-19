"""Шаги поездки: передняя стена, стены клеток впереди и решение у границы клетки."""
from __future__ import annotations

import sys
import time

import corridor
from walls import SIDE_THRESH, WALL_THRESH

import config
import state
import bumper
import esp_link
import lidar_walls
import turning


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


def track_front_wall(trip):
    """Расстояние до передней стены между сканами лидара (по одометрии)."""
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
    trip.walls = lidar_walls.get_walls()      # один снимок стен на весь тик
    f_now = trip.walls["front"]
    if trip.on_slope_now or trip.p_abs >= config.RAMP_MAP_DEG:
        # Перед лидара на склоне - это пол или небо. Стоя у подножия, он
        # вообще видит саму горку как стену в ~40см (130мм/tg25..30° + 15см),
        # и без этого робот вставал бы "у стены" на первой плитке горки.
        f_now = None
        trip.f_anchor = None
    if seq != trip.last_seq:
        trip.last_seq = seq
        if f_now is not None:
            if f_now < FRONT_TRACK_MM:
                trip.f_anchor, trip.f_anchor_cm = f_now, trip.traveled_cm
            else:
                trip.f_anchor = None
        elif trip.f_anchor is not None and trip.traveled_cm - trip.f_anchor_cm > ANCHOR_STALE_CM:
            trip.f_anchor = None
    trip.f_est = None if trip.f_anchor is None else \
        trip.f_anchor - (trip.traveled_cm - trip.f_anchor_cm) * 10.0


def collect_cell_walls(trip):
    """Стены клеток впереди по новым оборотам лидара (езда по плану)."""
    # ── Стены клеток впереди (проезд без остановок) ───────
    # Каждый новый оборот лидара: точки коридоров - в систему пути, в
    # копилку стен N и следующей за ней; передняя стена N - голос по
    # этому же обороту; конец/начало боковой стены у границы с N -
    # поправка пути (столбик). На склоне лидар видит пол - не копим.
    if trip.cell_sides is not None:
        if trip.on_slope_now or trip.p_abs >= config.RAMP_MAP_DEG:
            trip.scan_mark = state.scan_cnt
        else:
            yaw_err = turning.angle_err(esp_link.get_yaw(), trip.target_yaw)
            for cnt, t_end, trev, pts in lidar_walls._new_scans(trip.scan_mark):
                trip.scan_mark = cnt
                path_pts = corridor.to_path_frame(
                    pts, t_end, trev, lambda t: (trip.odo_t.at(t) or 0.0) * 10.0, yaw_err)
                x_robot = (trip.odo_t.at(t_end - trev / 2.0) or 0.0) * 10.0
                trip.kN, cN = trip.cells + 1, trip.boundary_cm * 10.0
                trip.cell_sides.add_scan(path_pts, x_robot,
                                    [(trip.kN, cN), (trip.kN + 1, cN + corridor.CELL_MM)])
                trip.cell_sides.prune(x_robot - corridor.CELL_MM)
                to_n = (trip.boundary_cm - trip.traveled_cm) * 10.0
                # Нет переднего замера (мало точек в окне) - не голос
                # "прохода": это не то же, что "впереди пусто".
                if 0.0 < to_n <= corridor.CELL_MM and trip.walls["front"] is not None:
                    f_c = trip.walls["front"] - to_n
                    trip.front_votes[0 if lidar_walls.wall_present(f_c, WALL_THRESH) else 1] += 1
                border = cN - corridor.HALF_CELL_MM
                if trip.kN not in trip.posts_done:
                    for side in ("right", "left"):
                        e = corridor.post_error(trip.cell_sides.band_x[side], border,
                                                trip.side_known.get((trip.kN - 1, side)),
                                                trip.cell_sides.result(trip.kN, side)[0])
                        if e is not None:
                            trip.boundary_cm += e / 20.0
                            trip.posts_done.add(trip.kN)
                            print(f"\n    [odo] столбик {side} у границы кл.{trip.kN}: "
                                  f"{e:+.0f}мм - цель {e / 2:+.0f}мм")
                            break


def remaining_distance(trip):
    """Сколько ещё ехать: по стене, если она согласна с одометрией."""
    # ── Сколько ещё ехать ─────────────────────────────────
    # Есть стена - правда у неё: едем ДО НЕЁ, а не до отметки одометрии,
    # поэтому робот встаёт в центре клетки сразу, без доводки после
    # остановки. Но верим ей, только если она согласна с одометрией в
    # пределах доводки: разошлись сильнее - значит это стена не этой
    # клетки (видно через открытый проём), и правит одометрия.
    rem_odo = trip.boundary_cm - trip.traveled_cm
    trip.rem = rem_odo
    trip.at_wall = False
    if trip.f_est is not None:
        rem_wall = (trip.f_est - config.FRONT_CENTER_MM) / 10.0
        if abs(rem_wall - rem_odo) <= config.ALIGN_MAX_MM / 10.0:
            trip.rem = rem_wall
            trip.at_wall = True


def brake_if_wall_close(trip):
    """Стена ближе тормозного пути - тормозить немедленно."""
    # ── Страховка: стена ближе, чем тормозной путь ────────
    # Решение "ехать дальше" не должно быть неотменяемым: стену могли
    # увидеть позже (она была за порогом видимости, или лидар моргнул).
    # Если до места остановки у стены осталось меньше, чем нужно на
    # торможение - тормозим немедленно, не дожидаясь следующей границы.
    # Действует и когда остановка запланирована с места (one_cell): тогда
    # это случай "стена ближе, чем цель, и с одометрией не согласна".
    if trip.f_est is not None and not trip.wall_forced:
        need_cm = trip.fwd * trip.fwd / (2.0 * config.BRAKE_DECEL_CMS2)
        to_stop_cm = (trip.f_est - config.FRONT_CENTER_MM) / 10.0
        if to_stop_cm <= need_cm * 1.3 and (not trip.stopping or to_stop_cm < trip.rem - 1.0):
            trip.wall_forced = True
            trip.forced_from_cm = trip.boundary_cm - trip.cell_cm
            trip.stopping, trip.pending = True, False
            trip.boundary_cm = trip.traveled_cm + to_stop_cm
            trip.rem = to_stop_cm
            trip.at_wall = True
            print(f"\n    стена близко: {trip.f_est:.0f}мм - торможу немедленно")


def decide_at_boundary(trip):
    """Решение у границы клетки: ехать дальше или тормозить."""
    # ── Решение по подходящей границе клетки ──────────────
    # Принимается один раз за клетку, ровно за DECIDE_LEAD_MM до её
    # центра - раньше нельзя (боковое окно замера ещё читает предыдущую
    # клетку), позже нельзя (не успеем затормозить).
    # С планом решение ждёт, пока кандидат в горку не выяснится: иначе
    # клетку перед горкой план записал бы посещённой.
    if trip.pending and trip.rem <= trip.lead_cm and not (trip.plan is not None and trip.ramp_state == 'cand'):
        trip.pending = False
        br = lidar_walls.wall_present(trip.walls["right"], SIDE_THRESH)

        # Переднюю стену сравниваем не с сырым показанием, а с тем, каким
        # оно станет В ЦЕНТРЕ клетки: решение принимается за DECIDE_LEAD_MM
        # до центра, и та же стена читается на эти 110мм дальше. Без
        # пересчёта стена, дающая в центре нормальные ~180мм, здесь читалась
        # как 290мм, не проходила порог 260 - и робот уезжал в клетку со
        # стеной, считая её открытой. Прижатый концевик тут авторитетнее
        # лидара: вплотную к стене лидар её уже не видит (она ближе
        # собственного габарита робота).
        f_ref = trip.f_est if trip.f_est is not None else trip.walls["front"]
        f_center = None if f_ref is None else f_ref - trip.rem * 10.0
        bf = lidar_walls.wall_present(f_center, WALL_THRESH) or bumper.get_bumper()

        if trip.cells + 1 >= trip.max_cells:
            trip.stopping, trip.reason = True, "done"
        elif trip.plan is not None and trip.cell_sides is not None:
            trip.kN = trip.cells + 1
            res = {s: trip.cell_sides.result(trip.kN, s) for s in ("right", "left")}
            fv, fn = trip.front_votes
            f_wall = (True if bumper.get_bumper() or (fv >= 2 and fv > fn) else
                      False if fn >= 2 and fn > fv else None)
            if config.TILE_BLUE in trip.tiles.get(trip.kN, ()):
                print(f"\n    → кл.{trip.kN}: синяя - стоп в её центре")
                go, walls_rel = False, {}
            else:
                go, walls_rel = trip.plan.decide(trip.kN, res, (f_wall, fv, fn))
            if go:
                trip.cells += 1
                trip.boundary_cm += trip.cell_cm
                trip.rem += trip.cell_cm
                trip.pending = True
                trip.t_cell = time.monotonic()
                trip.plan.passed(trip.kN, walls_rel)
                for s in ("right", "left"):
                    trip.side_known[(trip.kN, s)] = walls_rel.get(s)
                trip.front_votes = [0, 0]
            else:
                trip.stopping, trip.reason = True, "stop"
        elif not br or bf:
            trip.stopping, trip.reason = True, "stop"
        else:
            # Прямая продолжается - не тормозим вообще, просто отодвигаем
            # цель на клетку вперёд и едем дальше на крейсерской.
            trip.cells += 1
            trip.boundary_cm += trip.cell_cm
            trip.rem += trip.cell_cm
            trip.pending = True
            trip.t_cell = time.monotonic()
            sys.stdout.write(f"\r    клетка {trip.cells} пройдена с ходу      \n")

        if trip.stopping:
            brake_dist = trip.fwd * trip.fwd / (2.0 * config.BRAKE_DECEL_CMS2)
            print(f"    торможение с {trip.fwd:.0f}см/с: путь {brake_dist*10:.0f}мм "
                  f"из {trip.rem*10:.0f}мм в запасе")
