"""Обход лабиринта и возврат на старт."""
from __future__ import annotations

import time

import runplan
import watch_logic

import config
import state
import timing
import bumper
import dash_live
import driving
import esp_link
import floor_tiles
import lidar_walls
import marks
import maze_map
import ramps
import recovery
import turning
import victims


OBSTACLE_RETRIES = 2           # столько откатов+повторов, дальше считаем проезд закрытым

# Откат перед повтором. Задача скромная - дать колёсам сойти с кромки и снова
# зацепиться за пол, поэтому не BUMPER_BACKOFF_MM (70мм, он про "стена
# вплотную, её не видно"), а меньше.
#
# Важное ограничение, общее с откатом по бамперу: следующий drive_cells
# обнуляет одометрию и снова отмеряет свои 300мм ОТ НОВОГО МЕСТА, то есть на
# величину отката робот заедет в клетку глубже. Пока есть передняя стена,
# это подбирает f_est/align_at_wall (расхождение должно укладываться в
# ALIGN_MAX_MM), а на открытом препятствии - не подбирает ничем. 45мм
# оставляет двойной откат (90мм) с запасом внутри ALIGN_MAX_MM=150.
OBSTACLE_BACKOFF_MM = 45.0
# Сколько итераций подряд без единого продвижения (по любой причине) прежде
# чем считать разведку зашедшей в тупик и прекращать заезд как FATAL.
MAX_STALLS = 5


_WALL_TXT = {True: "есть", False: "нет", None: "?"}


class _DrivePlan:
    """План поездки без остановок для drive_cells(plan=...): какая клетка
    k-я по ходу, ехать ли через следующую клетку N прямо и что записать в
    карту, когда робот её проехал. Решение - тот же BFS, что выбирает
    маршрут стоя (runplan.go_straight): в новой клетке - по стенам, которые
    лидар собрал на подходе (corridor.py), в известной - по карте.

    goal_test - цель маршрута: фронтир разведки или старт. home=True -
    возврат: цель в самой N значит встать в ней (при разведке N сама
    становится посещённой, цель ищется дальше). on_checkpoint(N) - под
    клеткой N серебряная плитка."""

    def __init__(self, cell, hdg, goal_test, home=False, on_checkpoint=None):
        self.cell0, self.hdg, self.goal_test, self.home = cell, hdg, goal_test, home
        self.on_checkpoint = on_checkpoint
        self._measured = {}     # k -> {сторона: замер лидара или None}

    def cell(self, k):
        return maze_map._cell_at(self.cell0, self.hdg, k)

    def known_side(self, k, side):
        """Боковая стена k-й клетки по карте (True/False/None)."""
        c = state.maze.get(self.cell(k))
        return None if c is None else c[watch_logic.wall_of(self.hdg, side)]

    def decide(self, k, sides_res, front):
        """sides_res - {'right'/'left': CellSides.result()}, front - (стена
        спереди по голосам или None, голосов "есть", голосов "нет").
        Возвращает (ехать прямо, {'right'/'left': стена}) - стены для карты
        и для столбиков: замер, а где его нет - карта."""
        N, hdg = self.cell(k), self.hdg
        rel = {"right": (hdg + 1) % 4, "left": (hdg + 3) % 4, "front": hdg}
        measured = {"right": sides_res["right"][0], "left": sides_res["left"][0],
                    "front": front[0]}
        self._measured[k] = measured
        known = state.maze.get(N)
        walls = {s: (measured[s] if measured[s] is not None
                     else None if known is None else known[d])
                 for s, d in rel.items()}
        if N in state.visited and known is not None:
            walls = {s: known[d] for s, d in rel.items()}   # стоя мерили - карте верим
        go, why = False, None
        if (N[0], N[1], hdg) in state.ramps:
            why = "дальше горка"
        elif self.home and self.goal_test(N):
            why = "цель"
        elif None in walls.values():
            why = "не уверен"
        else:
            abs_w = {d: walls[s] for s, d in rel.items()}
            abs_w[(hdg + 2) % 4] = False       # оттуда приехали

            def od(cx, cy):
                if (cx, cy) != N:
                    return maze_map.open_dirs(cx, cy)
                return [d for d in range(4)
                        if abs_w[d] is False and maze_map._next_cell(cx, cy, d) not in state.blocked]

            goal = self.goal_test if self.home else (lambda c: c != N and self.goal_test(c))
            go = runplan.go_straight(N, hdg, od, maze_map._next_cell, goal, config.SIDE_ORDER[state.side_pref])
            why = "известна" if N in state.visited else "новая"
        r, l = sides_res["right"], sides_res["left"]
        print(f"\n    → кл.{N}: R={_WALL_TXT[walls['right']]}({r[1]}т,{r[2]:.0f}мм) "
              f"L={_WALL_TXT[walls['left']]}({l[1]}т,{l[2]:.0f}мм) "
              f"F={_WALL_TXT[walls['front']]}({front[1]}:{front[2]}) → "
              f"{'прямо' if go else 'стоп'} ({why})")
        return go, {"right": walls["right"], "left": walls["left"]}

    def passed(self, k, walls_rel):
        """Клетку k проехали без остановки: ребро в неё - проход по факту,
        она посещена, боковые стены - голосом лидара (только где был свой
        замер: повторять карту её же голосом незачем)."""
        prev, N = self.cell(k - 1), self.cell(k)
        maze_map.open_edge(prev[0], prev[1], self.hdg)
        state.visited.add(N)
        for s in ("right", "left"):
            v = self._measured.get(k, {}).get(s)
            if v is not None:
                maze_map.vote_wall(N[0], N[1], watch_logic.wall_of(self.hdg, s), v)

    def checkpoint(self, k):
        if self.on_checkpoint is not None:
            self.on_checkpoint(self.cell(k))


def _arrived_cell(cx, cy, hdg, reason):
    """Где робот оказался после drive_cells(), проехавшего одну клетку."""
    if reason == "ramp":
        return ramps.record_ramp(cx, cy, hdg, state.last_ramp,
                           measure=lambda: lidar_walls._measure_cell(hdg))
    if (cx, cy, hdg) in state.ramps:
        # По карте здесь горка, а drive_cells её не узнал (наклон не дотянул
        # до RAMP_MAP_DEG). Шагнуть на соседа по одометрии нельзя - его в
        # карте нет, это плитка горки. Верим карте и печатаем - если это
        # когда-нибудь всплывёт в логе, порог RAMP_MAP_DEG велик для поля.
        print("    [WARN] по карте здесь горка, а наклона не было - "
              "считаю, что проехали её")
    return maze_map._next_cell(cx, cy, hdg)


# ══════════════════════════════════════════════════════════════
#  Разведка: BFS / flood-fill
# ══════════════════════════════════════════════════════════════
def flood_fill_run(max_cells):
    """Разведка неизвестного лабиринта: на каждой клетке, где робот ещё не
    стоял, снимает все 4 стены одним замером (карта в _maze), затем BFS
    ищет кратчайший путь по известным открытым рёбрам до ближайшей ещё не
    посещённой клетки и едет туда. Останавливается сама, когда такой клетки
    не остаётся - возвращает (cx, cy, hdg, checkpoint, status), status:
    'explored' (лабиринт разведан), 'cap' (упёрлись в max_cells),
    'fatal' (связь/одометрия/полный затор - дальше ехать нельзя)."""
    cx, cy, hdg = 0, 0, 0
    checkpoint = (cx, cy, hdg)
    stalls = 0
    obstacle_fails = 0
    bumper_side_fails = 0
    total = 0

    def arrive(new_cell):
        """Стоим в центре (cx,cy). Если клетка новая - снимаем стены и
        сканируем обе боковые камеры; в любом случае проверяем плитку.
        Курс может измениться: после выдачи аптечек робот остаётся спиной
        к метке (см. report_victim)."""
        nonlocal checkpoint, hdg
        # Позицию - на веб сразу по приезде, не дожидаясь замера стен, синей
        # стоянки и камер: иначе робот на карте отставал на всё это время.
        dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")
        # Стены мерим и на повторном проезде: каждый замер - лишний голос
        # (см. vote_wall), и ошибка первого приезда исправляется сама.
        walls_rel = lidar_walls._measure_cell(hdg)
        maze_map.set_cell_walls(cx, cy, hdg, walls_rel)
        if new_cell:
            state.visited.add((cx, cy))
        dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")   # стены

        if esp_link.get_esp_paused():
            return      # спим - ни синей стоянки, ни сканов; разберётся цикл
        tile = floor_tiles.tile_color()
        # Живое распознавание ИЛИ клетка уже когда-то была признана синей -
        # см. blue_stop()/_blue_cells: однажды подтверждённому факту доверяем
        # больше, чем одному текущему замеру датчика цвета.
        if tile == config.TILE_BLUE or (cx, cy) in state.blue_cells:
            floor_tiles.blue_stop((cx, cy))
        # Чекпоинт запоминается стоя в центре клетки: на ходу цвет не
        # читается при задранном носе (PITCH_COLOR_GUARD_DEG).
        if tile == state.CHECKPOINT_TILE and checkpoint[:2] != (cx, cy):
            checkpoint = (cx, cy, hdg)
            print(f"  ✓ ЧЕКПОИНТ ({cx},{cy}) курс={config._DIR_NAME[hdg]}")

        if new_cell:
            hdg = victims.scan_cell(cx, cy, hdg, walls_rel)

        dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")

    arrive(True)     # стартовая клетка
    t_step = t_snap = None   # хронометраж шага, см. time_report()

    while total < max_cells:
        # Подъём - раньше паузы: пока робот в руках, тормозить и мерить
        # стены бессмысленно.
        if state.lifted.is_set():
            recovery.handle_lift(checkpoint[2])
            cx, cy, hdg = checkpoint
            print(f"  ↩ Продолжаем с чекпоинта ({cx},{cy}) курс={config._DIR_NAME[hdg]}")
            dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")
            stalls = obstacle_fails = bumper_side_fails = 0
            continue

        if esp_link.get_esp_paused():
            dash_live._write_state(cx, cy, hdg, checkpoint, "sleep")
            cx, cy, hdg = recovery.restart_from_checkpoint(checkpoint)
            stalls = obstacle_fails = bumper_side_fails = 0
            dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")
            continue

        if recovery.wait_for_robot(cx, cy, hdg, checkpoint, "exploring"):
            continue

        path = maze_map.find_frontier_path(cx, cy, hdg)
        if path is None:
            print("  ✓ Неразведанных клеток, достижимых по известной карте, "
                  "больше нет - разведка завершена")
            dash_live._write_state(cx, cy, hdg, checkpoint, "explored")
            return cx, cy, hdg, checkpoint, "explored"

        d = path[0]
        now = time.monotonic()
        if t_step is not None:
            print(timing.time_report(t_snap, now - t_step))   # прошлый шаг целиком
        t_step, t_snap = now, dict(state.tacc)
        print(f"─── ({cx},{cy}) {config._DIR_NAME[hdg]} → {config._DIR_NAME[d]}  {total}/{max_cells} ───")

        delta = config.DIR_DELTA[(d - hdg) % 4]
        if delta != 0:
            # Разворот на 180 - двумя поворотами по 90 со сканом между ними.
            # Камеры смотрят только вбок, и разворот одним махом оставлял
            # переднюю стену тупика (единственную, какую робот ещё не видел)
            # непросканированной: до разворота она спереди, после - сзади.
            # На середине она оказывается сбоку.
            steps = (90, 90) if abs(delta) == 180 else (delta,)
            replan = False
            for i, step in enumerate(steps):
                tgt = turning.snap_target(step)
                seen = []
                ok = turning.turn_to(tgt, label=f"{step:+.0f}°", seen=seen, walls=state.maze.get((cx, cy)))
                if esp_link.get_esp_paused():
                    replan = True
                    break
                if not ok:
                    # Не доворачивал - курс в карте с гироскопа, маршрут заново.
                    hdg = turning.gyro_heading()
                    dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")
                    replan = True
                    break
                if seen:
                    h = marks._turn_marks(cx, cy, seen)
                    if h is not None:
                        # выдали комплекты - стоим спиной к метке, шаг заново
                        hdg = h
                        dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")
                        replan = True
                        break
                    if abs(turning.angle_err(esp_link.get_yaw(), tgt)) > config.TURN_TOL_DEG:
                        turning.turn_to(tgt, label="обратно")
                # После поворота стены поменялись местами - встаём в центр
                # клетки по тому, что видно. Мерить после поворота точнее, чем
                # предсказывать до него. Посередине разворота на 180° - не
                # надо: робот никуда не ехал, а через секунду ось опять
                # сменится, и доводка была чистым простоем.
                if i == len(steps) - 1:
                    turning.align_at_wall(turning.snap_target())
                hdg = (hdg + step // 90) % 4
                dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")
                # Камеры смотрят теперь на другие стены той же клетки -
                # сканируем их, не дожидаясь проезда вперёд.
                h = victims.scan_cell(cx, cy, hdg)
                if esp_link.get_esp_paused():
                    replan = True
                    break
                if h != hdg:
                    # Выдали аптечку и остались спиной к метке - остаток
                    # поворота посчитан от старого курса, строим шаг заново.
                    hdg = h
                    dash_live._write_state(cx, cy, hdg, checkpoint, "exploring")
                    replan = True
                    break
            if replan:
                continue

        # Прямые - без остановок: drive_cells встанет сам там, где нужен
        # поворот, где не уверен в стенах или на синей (см. _DrivePlan).
        # Веб-карта обновится по приезде (arrive): файл состояния из петли
        # управления не пишем.
        def on_checkpoint(N, h=hdg):
            nonlocal checkpoint
            if checkpoint[:2] != N:
                checkpoint = (N[0], N[1], h)
                print(f"\n  ✓ ЧЕКПОИНТ ({N[0]},{N[1]}) курс={config._DIR_NAME[h]} (на ходу)")

        plan = _DrivePlan((cx, cy), hdg, lambda c: c not in state.visited,
                          on_checkpoint=on_checkpoint)
        n, reason = driving.drive_cells(max(1, max_cells - total), expect_ramp=(cx, cy, hdg) in state.ramps,
                                cell=(cx, cy), hdg=hdg, plan=plan)
        if n > 0:
            # Клетки до последней уже в карте (_DrivePlan.passed); последний
            # шаг - как раньше: проход по факту или горка, и замер стоя.
            cx, cy = plan.cell(n - 1)
            if reason != "ramp":
                maze_map.open_edge(cx, cy, hdg)   # проехали - значит, проход (горку пишет record_ramp)
            cx, cy = _arrived_cell(cx, cy, hdg, reason)
            total += n
            arrive((cx, cy) not in state.visited)

        if reason in ("paused", "lifted", "link_lost"):
            stalls = 0
            continue      # разбирается в начале следующей итерации

        if reason == "odo_fault":
            # Одометрия не растёт. Энкодеры молчат - моторы выключены: ждём.
            # Отвечают - колёса крутить не может, то есть застряли: отъехать
            # и считать ребро закрытым, как непреодолённое препятствие.
            if recovery.wait_for_robot(cx, cy, hdg, checkpoint, "exploring"):
                continue
            print("    одометрия не растёт при живых энкодерах - застряли, ребро закрыто")
            driving.creep_mm(-OBSTACLE_BACKOFF_MM, turning.snap_target())
            maze_map.block_direction(cx, cy, hdg)
            continue

        if reason in ("obstacle_stall", "ramp_stall"):
            # НЕ фатально: это «не взял препятствие», а не отказ одометрии.
            # Откатываемся, чтобы колёса сошли с бордюра и снова зацепились
            # за пол, и пробуем то же ребро заново - ассист в drive_cells
            # при этом взведётся с нуля (нос вернулся в горизонт на откате).
            # С горки drive_cells уже скатился в центр исходной клетки сам.
            obstacle_fails += 1
            if reason == "obstacle_stall":
                driving.creep_mm(-OBSTACLE_BACKOFF_MM, turning.snap_target())
            if obstacle_fails <= OBSTACLE_RETRIES:
                print(f"    препятствие не взято "
                      f"({obstacle_fails}/{OBSTACLE_RETRIES}) - откат и повтор")
                stalls = 0
                continue
            # Исчерпали попытки - считаем ребро непроезжим, BFS обойдёт его.
            print("    препятствие не взято - считаю ребро непроезжим")
            maze_map.block_direction(cx, cy, hdg)
            obstacle_fails = 0
            stalls = 0
            continue
        obstacle_fails = 0

        if reason == "front_obstacle":
            # Кирпич или что-то ещё у входа в следующую клетку - проехать
            # прямо нельзя. Ребро закрыто только с этой стороны (а не вся
            # клетка, как у чёрной): в неё можно попасть с другой стороны.
            print("    препятствие у входа в клетку - ребро закрыто")
            maze_map.close_edge(cx, cy, hdg)
            stalls = 0
            continue

        if reason == "wall_close" and n == 0:
            # Стена прямо перед нами, а карта считала проход открытым (например,
            # боковой замер соврал после переезда центра). Без записи в карту
            # BFS выбирал тот же путь снова и снова - робот "застревал" стоя.
            print("    стена вплотную перед клеткой - записываю её в карту")
            maze_map.close_edge(cx, cy, hdg)
            stalls = 0
            continue

        if reason == "black":
            # Ребро в чёрную клетку непроезжее - BFS обойдёт его сам, без
            # жёстко зашитого разворота на 180.
            print("    чёрная клетка - ребро непроезжее")
            maze_map.block_direction(cx, cy, hdg)
            stalls = 0
            continue

        if reason == "bumper_both":
            # Пока робот прижат к стене, лидар её не видит - следующий же
            # замер сказал бы "впереди свободно". Откат возвращает стену в
            # поле зрения, а карта сразу узнаёт про эту стену.
            print(f"    откат {bumper.BUMPER_BACKOFF_MM:.0f}мм - вплотную стену не видно")
            driving.creep_mm(-bumper.BUMPER_BACKOFF_MM, turning.snap_target())
            maze_map.block_direction(cx, cy, hdg)
            stalls = 0
            continue

        elif reason in ("bumper_left", "bumper_right"):
            # Сработал только ОДИН концевик - задели что-то одним боком, не
            # стеной по центру. Объезд: поворот на 45° В СТОРОНУ ОТ задетого
            # бока, откат, возврат на исходный курс - и следующая итерация
            # сама попробует то же ребро заново. Знак подобран эмпирически
            # (как TURN_SIGN) - по факту "слева" оказалось направо.
            base_yaw = turning.snap_target()
            away_deg = -45 if reason == "bumper_left" else 45
            side_ru = "слева" if reason == "bumper_left" else "справа"
            print(f"    асимметричный бампер ({side_ru}) - "
                  f"поворот {away_deg:+.0f}°, откат {bumper.BUMPER_SIDE_BACKOFF_MM:.0f}мм, возврат курса")
            turning.turn(away_deg)
            if not esp_link.get_esp_paused():
                driving.creep_mm(-bumper.BUMPER_SIDE_BACKOFF_MM, esp_link.get_yaw())
            if not esp_link.get_esp_paused():
                turning.turn_to(base_yaw)
            bumper_side_fails += 1
            if bumper_side_fails >= bumper.BUMPER_SIDE_RETRIES:
                print("    асимметричный бампер повторился - считаю ребро непроезжим")
                maze_map.block_direction(cx, cy, hdg)
                bumper_side_fails = 0
            stalls = 0
            continue

        bumper_side_fails = 0
        # Защита от залипания: если робот несколько раз подряд не смог
        # продвинуться ни на клетку по ЛЮБОЙ причине, дальше крутиться
        # бессмысленно - что-то системно не так.
        # Ждали выключенные моторы - это не попытка проехать.
        stalls = stalls + 1 if n == 0 and reason != "resumed" else 0
        if stalls >= MAX_STALLS:
            print("  [WARN] застряли - прекращаю разведку")
            dash_live._write_state(cx, cy, hdg, checkpoint, "fatal")
            return cx, cy, hdg, checkpoint, "fatal"

    print(f"  [WARN] потолок клеток ({max_cells}) исчерпан - прекращаю разведку")
    dash_live._write_state(cx, cy, hdg, checkpoint, "cap")
    return cx, cy, hdg, checkpoint, "cap"


# ══════════════════════════════════════════════════════════════
#  Возврат на старт (BFS по уже известной карте)
# ══════════════════════════════════════════════════════════════
def return_to_start(cx, cy, hdg, checkpoint):
    """Едет на (0,0) по уже разведанной карте - тот же BFS, что и в
    flood_fill_run, только цель фиксированная и новых клеток не открывает.
    Возвращает (cx, cy, hdg); если старт не оказался достижим по известной
    карте (не должно случиться при связном лабиринте), возвращает текущую
    позицию, не доехав."""
    if (cx, cy) == (0, 0):
        return cx, cy, hdg
    print("\n  ↩ Возврат на старт (BFS по известной карте)")

    while (cx, cy) != (0, 0):
        if state.lifted.is_set():
            recovery.handle_lift(checkpoint[2])
            cx, cy, hdg = checkpoint
            dash_live._write_state(cx, cy, hdg, checkpoint, "returning")
            continue

        if esp_link.get_esp_paused():
            dash_live._write_state(cx, cy, hdg, checkpoint, "sleep")
            cx, cy, hdg = recovery.restart_from_checkpoint(checkpoint)
            dash_live._write_state(cx, cy, hdg, checkpoint, "returning")
            continue

        if recovery.wait_for_robot(cx, cy, hdg, checkpoint, "returning"):
            continue

        path = maze_map.find_path_bfs((cx, cy), lambda c: c == (0, 0), hdg)
        if path is None:
            print("  [FATAL] Старт недостижим по известной карте — возврат прерван")
            dash_live._write_state(cx, cy, hdg, checkpoint, "fatal")
            return cx, cy, hdg

        d = path[0]
        delta = config.DIR_DELTA[(d - hdg) % 4]
        if delta != 0:
            tgt = turning.snap_target(delta)
            seen = []
            ok = turning.turn_to(tgt, label=f"{delta:+.0f}°", seen=seen, walls=state.maze.get((cx, cy)))
            if esp_link.get_esp_paused():
                continue
            if not ok:
                hdg = turning.gyro_heading()
                dash_live._write_state(cx, cy, hdg, checkpoint, "returning")
                continue
            if seen:
                h = marks._turn_marks(cx, cy, seen)
                if h is not None:
                    hdg = h
                    dash_live._write_state(cx, cy, hdg, checkpoint, "returning")
                    continue
                if abs(turning.angle_err(esp_link.get_yaw(), tgt)) > config.TURN_TOL_DEG:
                    turning.turn_to(tgt, label="обратно")
            turning.align_at_wall(turning.snap_target())
            hdg = d
            dash_live._write_state(cx, cy, hdg, checkpoint, "returning")

        plan = _DrivePlan((cx, cy), hdg, lambda c: c == (0, 0), home=True)
        n, reason = driving.drive_cells(300, expect_ramp=(cx, cy, hdg) in state.ramps,
                                cell=(cx, cy), hdg=hdg, plan=plan)
        if n > 0:
            cx, cy = plan.cell(n - 1)
            if reason != "ramp":
                maze_map.open_edge(cx, cy, hdg)
            cx, cy = _arrived_cell(cx, cy, hdg, reason)
            # Позицию - до синей стоянки (она 6с), а не после.
            dash_live._write_state(cx, cy, hdg, checkpoint, "returning")
            # Голос и на обратном пути: если карта ошиблась, BFS сразу
            # перестроит маршрут до старта.
            if not esp_link.get_esp_paused():
                maze_map.set_cell_walls(cx, cy, hdg, lidar_walls._measure_cell(hdg))
            # Синяя клетка требует стоянки на КАЖДОМ визите, включая обратный
            # путь - см. blue_stop()/_blue_cells.
            if floor_tiles.tile_color() == config.TILE_BLUE or (cx, cy) in state.blue_cells:
                floor_tiles.blue_stop((cx, cy))

        if reason in ("paused", "lifted", "link_lost"):
            continue
        if reason == "odo_fault":
            if recovery.wait_for_robot(cx, cy, hdg, checkpoint, "returning"):
                continue
            driving.creep_mm(-OBSTACLE_BACKOFF_MM, turning.snap_target())
            maze_map.block_direction(cx, cy, hdg)
            continue

        # Карта уже известна, но реальность может разойтись с ней (стену
        # чуть подвинули, лидар ошибся при разведке) - те же реакции, что и
        # при разведке: считаем ребро непроезжим и просим BFS искать заново.
        if reason in ("obstacle_stall", "ramp_stall"):
            if reason == "obstacle_stall":
                driving.creep_mm(-OBSTACLE_BACKOFF_MM, turning.snap_target())
            maze_map.block_direction(cx, cy, hdg)
            continue
        if reason == "black":
            maze_map.block_direction(cx, cy, hdg)
            continue
        if reason == "front_obstacle":
            maze_map.close_edge(cx, cy, hdg)          # см. то же место в flood_fill_run
            continue
        if reason == "wall_close" and n == 0:
            maze_map.close_edge(cx, cy, hdg)          # см. то же место в flood_fill_run
            continue
        if reason == "bumper_both":
            driving.creep_mm(-bumper.BUMPER_BACKOFF_MM, turning.snap_target())
            maze_map.block_direction(cx, cy, hdg)
            continue
        if reason in ("bumper_left", "bumper_right"):
            base_yaw = turning.snap_target()
            away_deg = -45 if reason == "bumper_left" else 45
            turning.turn(away_deg)
            if not esp_link.get_esp_paused():
                driving.creep_mm(-bumper.BUMPER_SIDE_BACKOFF_MM, esp_link.get_yaw())
            if not esp_link.get_esp_paused():
                turning.turn_to(base_yaw)
            maze_map.block_direction(cx, cy, hdg)
            continue

    print(f"  ✓ На старте ({cx},{cy}) курс={config._DIR_NAME[hdg]}")
    dash_live._write_state(cx, cy, hdg, checkpoint, "at_start")
    return cx, cy, hdg
