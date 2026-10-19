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
import tile_rules
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
# Сколько шагов подряд без единого продвижения (по любой причине) прежде
# чем считать, что застряли, и перезапускаться с чекпоинта.
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
#  Шаг к цели: общий для разведки, возврата и заданий
# ══════════════════════════════════════════════════════════════
class Walk:
    """Где робот и что шаг помнит между вызовами: клетка, курс, чекпоинт,
    фаза для пульта, сколько клеток проехано и счётчики неудач на ребре."""

    def __init__(self, cx=0, cy=0, hdg=0, checkpoint=None, phase="exploring", max_cells=300):
        self.cx, self.cy, self.hdg = cx, cy, hdg
        self.checkpoint = checkpoint if checkpoint is not None else (cx, cy, hdg)
        self.phase = phase
        self.max_cells = max_cells      # потолок клеток (страховка от зацикливания)
        self.total = 0                  # клеток проехано
        self.t_step = self.t_snap = None   # хронометраж шага, см. time_report()
        self.reset_fails()

    def reset_fails(self):
        self.stalls = self.obstacle_fails = self.bumper_side_fails = 0

    @property
    def cell(self):
        return self.cx, self.cy

    def show(self, phase=None):
        """Позицию и фазу - на пульт и в файл состояния."""
        dash_live._write_state(self.cx, self.cy, self.hdg, self.checkpoint, phase or self.phase)


def step(w, goal_test, home=False, arrive=None):
    """Один шаг к ближайшей клетке, где goal_test(клетка) верно: дождаться
    робота, путь BFS, поворот, поездка, разбор причины остановки.
    home=True - встать в самой цели (возврат, задания); False - разведка.
    arrive(w, new_cell) - что делать по приезде в клетку (стены, плитка,
    камеры) - задаёт вызывающий.

    Возвращает "goal" - уже стоим в цели, "no_path" - пути по карте нет,
    "step" - шаг сделан (доехали или разобрали остановку), звать снова."""
    if home and goal_test(w.cell):
        return "goal"
    if _wait_robot(w):
        return "step"
    path = maze_map.find_path_bfs(w.cell, goal_test, w.hdg)
    if path is None:
        return "no_path"
    _print_step(w, path[0])
    if not _turn_toward(w, path[0]):
        return "step"
    n, reason = _drive(w, goal_test, home, arrive)
    _handle_stop(w, n, reason)
    return "step"


def go_to(w, goal_test, arrive):
    """Шагами до клетки, где goal_test верно, и встать в ней. Пути нет -
    не выход, а перезапуск с чекпоинта (см. _no_path)."""
    while True:
        r = step(w, goal_test, home=True, arrive=arrive)
        if r == "goal":
            return
        if r == "no_path":
            _no_path(w, goal_test)


def _restart(w):
    """Перезапуск с чекпоинта: моторы выкл, лента синяя, ждём GPIO25/27,
    карта та же (recovery.restart_from_checkpoint)."""
    w.show("sleep")
    w.cx, w.cy, w.hdg = recovery.restart_from_checkpoint(w.checkpoint)
    w.reset_fails()
    w.show()


def _no_path(w, goal_test):
    """Пути к цели по карте нет - это ошибка карты, а не конец заезда.
    Сначала перезапуск с чекпоинта. Если и оттуда пути нет - забыть рёбра,
    закрытые препятствиями и бампером (кроме чёрных клеток): одно из них
    и отрезало цель. Дальше вызывающий ищет путь снова."""
    print("  [WARN] пути к цели по карте нет - перезапуск с чекпоинта")
    _restart(w)
    if maze_map.find_path_bfs(w.cell, goal_test, w.hdg) is None:
        n = maze_map.forget_soft_blocks()
        print(f"  пути всё ещё нет - забыл рёбер, закрытых препятствиями и бампером: {n}")


def _wait_robot(w):
    """Робота сняли, ESP на паузе или моторы молчат - дождаться.
    True - ждали, шаг заново."""
    # Подъём - раньше паузы: пока робот в руках, тормозить и мерить
    # стены бессмысленно.
    if state.lifted.is_set():
        recovery.handle_lift(w.checkpoint[2])
        w.cx, w.cy, w.hdg = w.checkpoint
        print(f"  ↩ Продолжаем с чекпоинта ({w.cx},{w.cy}) курс={config._DIR_NAME[w.hdg]}")
        w.show()
        w.reset_fails()
        return True
    if esp_link.get_esp_paused():
        _restart(w)
        return True
    return recovery.wait_for_robot(w.cx, w.cy, w.hdg, w.checkpoint, w.phase)


def _print_step(w, d):
    """Хронометраж прошлого шага и заголовок нового."""
    now = time.monotonic()
    if w.t_step is not None:
        print(timing.time_report(w.t_snap, now - w.t_step))   # прошлый шаг целиком
    w.t_step, w.t_snap = now, dict(state.tacc)
    print(f"─── ({w.cx},{w.cy}) {config._DIR_NAME[w.hdg]} → {config._DIR_NAME[d]}  {w.total}/{w.max_cells} ───")


def _turn_toward(w, d):
    """Повернуть на курс d стоя. Разворот на 180 - двумя поворотами по 90
    со сканом между ними. Камеры смотрят только вбок, и разворот одним махом
    оставлял переднюю стену тупика (единственную, какую робот ещё не видел)
    непросканированной: до разворота она спереди, после - сзади. На середине
    она оказывается сбоку. False - курс вышел не тот (не доворачивали, выдали
    комплекты, пауза ESP): маршрут заново."""
    delta = config.DIR_DELTA[(d - w.hdg) % 4]
    if delta == 0:
        return True
    steps = (90, 90) if abs(delta) == 180 else (delta,)
    for i, turn_deg in enumerate(steps):
        if not _turn_once(w, turn_deg, last=(i == len(steps) - 1)):
            return False
    return True


def _turn_once(w, turn_deg, last):
    """Один поворот на turn_deg (±90/180), доворот на метку, скан стен."""
    tgt = turning.snap_target(turn_deg)
    seen = []
    ok = turning.turn_to(tgt, label=f"{turn_deg:+.0f}°", seen=seen, walls=state.maze.get(w.cell))
    if esp_link.get_esp_paused():
        return False
    if not ok:
        # Не доворачивал - курс в карте с гироскопа, маршрут заново.
        w.hdg = turning.gyro_heading()
        w.show()
        return False
    if seen:
        h = marks._turn_marks(w.cx, w.cy, seen)
        if h is not None:
            # выдали комплекты - стоим спиной к метке, шаг заново
            w.hdg = h
            w.show()
            return False
        if abs(turning.angle_err(esp_link.get_yaw(), tgt)) > config.TURN_TOL_DEG:
            turning.turn_to(tgt, label="обратно")
    # После поворота стены поменялись местами - встаём в центр клетки по
    # тому, что видно. Мерить после поворота точнее, чем предсказывать до
    # него. Посередине разворота на 180° - не надо: робот никуда не ехал, а
    # через секунду ось опять сменится, и доводка была чистым простоем.
    if last:
        turning.align_at_wall(turning.snap_target())
    w.hdg = (w.hdg + turn_deg // 90) % 4
    w.show()
    # Камеры смотрят теперь на другие стены той же клетки - сканируем их,
    # не дожидаясь проезда вперёд.
    h = victims.scan_cell(w.cx, w.cy, w.hdg)
    if esp_link.get_esp_paused():
        return False
    if h != w.hdg:
        # Выдали аптечку и остались спиной к метке - остаток поворота
        # посчитан от старого курса, строим шаг заново.
        w.hdg = h
        w.show()
        return False
    return True


def _drive(w, goal_test, home, arrive):
    """Прямые - без остановок: drive_cells встанет сам там, где нужен
    поворот, где не уверен в стенах или на синей (см. _DrivePlan). Веб-карта
    обновится по приезде (arrive): файл состояния из петли управления не
    пишем. Возвращает (проехано клеток, причина остановки)."""
    def on_checkpoint(N, h=w.hdg):
        if w.checkpoint[:2] != N:
            w.checkpoint = (N[0], N[1], h)
            print(f"\n  ✓ ЧЕКПОИНТ ({N[0]},{N[1]}) курс={config._DIR_NAME[h]} (на ходу)")

    plan = _DrivePlan(w.cell, w.hdg, goal_test, home=home, on_checkpoint=on_checkpoint)
    n, reason = driving.drive_cells(max(1, w.max_cells - w.total),
                                    expect_ramp=(w.cx, w.cy, w.hdg) in state.ramps,
                                    cell=w.cell, hdg=w.hdg, plan=plan)
    if n > 0:
        # Клетки до последней уже в карте (_DrivePlan.passed); последний
        # шаг - как раньше: проход по факту или горка, и замер стоя.
        w.cx, w.cy = plan.cell(n - 1)
        if reason != "ramp":
            maze_map.open_edge(w.cx, w.cy, w.hdg)   # проехали - значит, проход (горку пишет record_ramp)
        w.cx, w.cy = _arrived_cell(w.cx, w.cy, w.hdg, reason)
        w.total += n
        if arrive is not None:
            arrive(w, w.cell not in state.visited)
    return n, reason


def _close_soft(w, whole_cell=True):
    """Ребро вперёд непроезжее из-за препятствия или бампера. Запоминаем в
    state.soft_closed: если путь к цели пропадёт, такие рёбра забываются
    (maze_map.forget_soft_blocks). whole_cell=False - закрыть только ребро,
    в клетку можно попасть с другой стороны."""
    if whole_cell:
        maze_map.block_direction(w.cx, w.cy, w.hdg)
    else:
        maze_map.close_edge(w.cx, w.cy, w.hdg)
    state.soft_closed.add((w.cx, w.cy, w.hdg))


def _handle_stop(w, n, reason):
    """Разбор причины, по которой drive_cells остановился."""
    if reason in ("paused", "lifted", "link_lost"):
        w.stalls = 0
        return      # разбирается в начале следующего шага

    if reason == "odo_fault":
        # Одометрия не растёт. Энкодеры молчат - моторы выключены: ждём.
        # Отвечают - колёса крутить не может, то есть застряли: отъехать
        # и считать ребро закрытым, как непреодолённое препятствие.
        if recovery.wait_for_robot(w.cx, w.cy, w.hdg, w.checkpoint, w.phase):
            return
        print("    одометрия не растёт при живых энкодерах - застряли, ребро закрыто")
        driving.creep_mm(-OBSTACLE_BACKOFF_MM, turning.snap_target())
        _close_soft(w)
        return

    if reason in ("obstacle_stall", "ramp_stall"):
        _obstacle_stall(w, reason)
        return
    w.obstacle_fails = 0

    if _close_ahead(w, n, reason):
        w.stalls = 0
        return

    if reason in ("bumper_left", "bumper_right"):
        _bumper_side(w, reason)
        return
    w.bumper_side_fails = 0

    # Защита от залипания: если робот несколько раз подряд не смог
    # продвинуться ни на клетку по ЛЮБОЙ причине, что-то системно не так -
    # перезапуск с чекпоинта. Ждали выключенные моторы - это не попытка проехать.
    w.stalls = w.stalls + 1 if n == 0 and reason != "resumed" else 0
    if w.stalls >= MAX_STALLS:
        print("  [WARN] застряли - перезапуск с чекпоинта")
        _restart(w)


def _close_ahead(w, n, reason):
    """Причины, после которых ребро вперёд закрывается сразу.
    True - причина из этих, ребро закрыто."""
    if reason == "front_obstacle":
        # Кирпич или что-то ещё у входа в следующую клетку - проехать
        # прямо нельзя. Ребро закрыто только с этой стороны (а не вся
        # клетка, как у чёрной): в неё можно попасть с другой стороны.
        print("    препятствие у входа в клетку - ребро закрыто")
        _close_soft(w, whole_cell=False)
    elif reason == "wall_close" and n == 0:
        # Стена прямо перед нами, а карта считала проход открытым (например,
        # боковой замер соврал после переезда центра). Без записи в карту
        # BFS выбирал тот же путь снова и снова - робот "застревал" стоя.
        print("    стена вплотную перед клеткой - записываю её в карту")
        maze_map.close_edge(w.cx, w.cy, w.hdg)
    elif reason == "black":
        # Ребро в чёрную клетку непроезжее - BFS обойдёт его сам, без
        # жёстко зашитого разворота на 180. Чёрная не забывается никогда.
        print("    чёрная клетка - ребро непроезжее")
        maze_map.block_direction(w.cx, w.cy, w.hdg)
        black = maze_map._next_cell(w.cx, w.cy, w.hdg)
        state.black_cells.add(black)
        tile_rules.remember(black, state.last_wall_tile)
    elif reason == "bumper_both":
        # Откат, и карта сразу узнаёт про эту стену (см. bumper.py).
        bumper.back_off_from_wall()
        _close_soft(w)
    else:
        return False
    return True


def _obstacle_stall(w, reason):
    """НЕ фатально: это «не взял препятствие», а не отказ одометрии.
    Откатываемся, чтобы колёса сошли с бордюра и снова зацепились за пол,
    и пробуем то же ребро заново - ассист в drive_cells при этом взведётся
    с нуля (нос вернулся в горизонт на откате). С горки drive_cells уже
    скатился в центр исходной клетки сам."""
    w.obstacle_fails += 1
    if reason == "obstacle_stall":
        driving.creep_mm(-OBSTACLE_BACKOFF_MM, turning.snap_target())
    w.stalls = 0
    if w.obstacle_fails <= OBSTACLE_RETRIES:
        print(f"    препятствие не взято "
              f"({w.obstacle_fails}/{OBSTACLE_RETRIES}) - откат и повтор")
        return
    # Исчерпали попытки - считаем ребро непроезжим, BFS обойдёт его.
    print("    препятствие не взято - считаю ребро непроезжим")
    _close_soft(w)
    w.obstacle_fails = 0


def _bumper_side(w, reason):
    """Объезд одним боком (см. bumper.py) - и следующий шаг сам попробует
    то же ребро заново, до BUMPER_SIDE_RETRIES раз."""
    bumper.dodge_side_hit(reason)
    w.bumper_side_fails += 1
    if w.bumper_side_fails >= bumper.BUMPER_SIDE_RETRIES:
        print("    асимметричный бампер повторился - считаю ребро непроезжим")
        _close_soft(w)
        w.bumper_side_fails = 0
    w.stalls = 0


# ══════════════════════════════════════════════════════════════
#  Разведка: BFS / flood-fill
# ══════════════════════════════════════════════════════════════
def unvisited(c):
    return c not in state.visited


def arrive_explore(w, new_cell):
    """Стоим в центре клетки. Если она новая - снимаем стены и сканируем
    обе боковые камеры; в любом случае проверяем плитку. Курс может
    измениться: после выдачи аптечек робот остаётся спиной к метке (см.
    report_victim)."""
    # Позицию - на веб сразу по приезде, не дожидаясь замера стен, синей
    # стоянки и камер: иначе робот на карте отставал на всё это время.
    w.show()
    # Стены мерим и на повторном проезде: каждый замер - лишний голос
    # (см. vote_wall), и ошибка первого приезда исправляется сама.
    walls_rel = lidar_walls._measure_cell(w.hdg)
    maze_map.set_cell_walls(w.cx, w.cy, w.hdg, walls_rel)
    if new_cell:
        state.visited.add(w.cell)
    w.show()   # стены

    if esp_link.get_esp_paused():
        return      # спим - ни синей стоянки, ни сканов; разберётся шаг
    tile = floor_tiles.tile_color()
    # Реакция по таблице (синяя - стоянка): на живой цвет ИЛИ на цвет, по
    # которому клетка уже реагировала - см. tile_rules.on_arrive.
    tile_rules.on_arrive(w.cell, tile)
    # Чекпоинт запоминается стоя в центре клетки: на ходу цвет не
    # читается при задранном носе (PITCH_COLOR_GUARD_DEG).
    if tile_rules.role(tile) == tile_rules.CHECKPOINT and w.checkpoint[:2] != w.cell:
        w.checkpoint = (w.cx, w.cy, w.hdg)
        print(f"  ✓ ЧЕКПОИНТ ({w.cx},{w.cy}) курс={config._DIR_NAME[w.hdg]}")

    if new_cell:
        w.hdg = victims.scan_cell(w.cx, w.cy, w.hdg, walls_rel)

    w.show()


def flood_fill_run(max_cells):
    """Разведка неизвестного лабиринта: на каждой клетке, где робот ещё не
    стоял, снимает все 4 стены одним замером (карта в _maze), затем BFS
    ищет кратчайший путь по известным открытым рёбрам до ближайшей ещё не
    посещённой клетки и едет туда. Останавливается сама, когда такой клетки
    не остаётся - возвращает (cx, cy, hdg, checkpoint, status), status:
    'explored' (лабиринт разведан), 'cap' (упёрлись в max_cells)."""
    w = Walk(phase="exploring", max_cells=max_cells)
    arrive_explore(w, True)     # стартовая клетка
    while w.total < max_cells:
        if step(w, unvisited, arrive=arrive_explore) == "no_path":
            print("  ✓ Неразведанных клеток, достижимых по известной карте, "
                  "больше нет - разведка завершена")
            w.show("explored")
            return w.cx, w.cy, w.hdg, w.checkpoint, "explored"

    print(f"  [WARN] потолок клеток ({max_cells}) исчерпан - прекращаю разведку")
    w.show("cap")
    return w.cx, w.cy, w.hdg, w.checkpoint, "cap"


# ══════════════════════════════════════════════════════════════
#  Возврат на старт (BFS по уже известной карте)
# ══════════════════════════════════════════════════════════════
def arrive_home(w, new_cell):
    """По приезде на обратном пути: стены и синяя, без камер и чекпоинта."""
    # Позицию - до синей стоянки (она 6с), а не после.
    w.show()
    # Голос и на обратном пути: если карта ошиблась, BFS сразу
    # перестроит маршрут до старта.
    if not esp_link.get_esp_paused():
        maze_map.set_cell_walls(w.cx, w.cy, w.hdg, lidar_walls._measure_cell(w.hdg))
    # Синяя клетка требует стоянки на КАЖДОМ визите, включая обратный
    # путь - см. tile_rules.on_arrive.
    tile_rules.on_arrive(w.cell, floor_tiles.tile_color())


def return_to_start(cx, cy, hdg, checkpoint):
    """Едет на (0,0) по уже разведанной карте - тот же шаг, что и в
    разведке, только цель фиксированная. Возвращает (cx, cy, hdg). Старт
    недостижим по карте - не выход, а перезапуск с чекпоинта (go_to)."""
    if (cx, cy) == (0, 0):
        return cx, cy, hdg
    print("\n  ↩ Возврат на старт (BFS по известной карте)")
    w = Walk(cx, cy, hdg, checkpoint, phase="returning")
    go_to(w, lambda c: c == (0, 0), arrive_home)
    print(f"  ✓ На старте ({w.cx},{w.cy}) курс={config._DIR_NAME[w.hdg]}")
    w.show("at_start")
    return w.cx, w.cy, w.hdg
