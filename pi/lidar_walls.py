"""Лидар: поток сканов и решение, есть ли стена с каждой стороны.

Цепочка от лидара до «стена есть»:
  1. lidar_proc.py (отдельный процесс) читает LD19 (ld19.py) и на каждый
     оборот считает расстояния до стен: walls.measure() - маски на детали
     робота, окна по углам, медиана. Там же окна и контур корпуса.
  2. _lidar_thread() здесь принимает эти замеры строками JSON и кладёт в
     state.telem: "wall" (на ходу), "wall_still" (стоя, симметричное окно
     бортов), "wall_n" (сколько точек вошло в замер).
  3. Читать: get_walls() - последний оборот (на ходу), measure_walls() -
     медиана нескольких оборотов стоя, measure_front_confirmed() - то же, но
     пустой перёд переспрашивает.
  4. «Есть стена» - wall_present(мм, порог). Пороги - в walls.py:
     WALL_THRESH (перёд), SIDE_THRESH (борта), BACK_THRESH (зад); для карты
     стоя - FRONT_MAP_WALL_MM / BACK_MAP_WALL_MM в maze_map.py (_walls_rel_map).
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time

from ld19 import BAUDRATE as LIDAR_BAUD, DEFAULT_PORT as LIDAR_PORT

import config
import state
import timing
import esp_link
import maze_map
import turning


# ── ВРЕМЕННАЯ диагностика "стена справа/слева там, где её нет" ──────
# Пишет в отдельный файл (не в консоль, не мешает обычному логу) момент,
# когда right/left переходит из "нет стены" в "есть стена" - вместе с
# сырыми точками лидара в этом окне, чтобы поймать спорадический ложный
# скачок в момент, когда он реально происходит (стоя за 145с багов не
# нашлось - похоже, завязано на вибрацию/крен на ходу). Отдельный поток от
# моторной петли - не влияет на управление. Убрать после того, как баг
# найден и исправлен.
_WALL_DEBUG_LOG = os.path.expanduser("~/maze/wall_debug.log")
LIDAR_RESTART_S = 1.0


def _new_scans(after_cnt):
    """Обороты с точками коридора новее after_cnt, по порядку."""
    return [s for s in list(state.scan_pts) if s[0] > after_cnt]


def _lidar_spawn():
    """lidar_proc.py отдельным процессом: свой GIL, порт читается без
    пропусков, пока здесь работает YOLO. Ctrl+C он получает вместе со всей
    группой, а если nav.py упадёт - умрёт сам на закрытом pipe."""
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lidar_proc.py")
    return subprocess.Popen([sys.executable, "-u", script, LIDAR_PORT, str(LIDAR_BAUD)],
                            stdout=subprocess.PIPE, text=True, bufsize=1)


def _lidar_thread():
    """Принимает готовые замеры от lidar_proc.py и кладёт их в _state. Сам
    лидар здесь больше не читается - только ~10 строк JSON в секунду."""
    with open(_WALL_DEBUG_LOG, "a") as dbg:
        while True:
            proc = state.lidar_proc = _lidar_spawn()
            for line in proc.stdout:
                try:
                    msg = json.loads(line)
                    w = msg["w"]
                    _ = (w["front"], w["right"], w["left"], w["back"], w["right_still"],
                         w["left_still"], w["front_n"], w["right_still_n"],
                         w["left_still_n"], w["back_n"])
                except (ValueError, KeyError, TypeError):
                    continue
                _store_scan(msg, w)
                if msg.get("dbg"):
                    _write_wall_debug(dbg, msg["dbg"], w)
            # stdout закрылся - процесс умер (порт отвалился, исключение).
            # Заезд без лидара невозможен, поэтому поднимаем заново; старые
            # замеры тем временем стареют, и wait_scan() это увидит.
            code = proc.wait()
            if state.lidar_stop.is_set():
                return
            state.lidar_restarts += 1
            print(f"\n  [WARN] лидарный процесс завершился (код {code}) - "
                  f"перезапуск #{state.lidar_restarts}")
            time.sleep(LIDAR_RESTART_S)


def _store_scan(msg, w):
    """Один оборот от lidar_proc.py -> state.telem и счётчик оборотов."""
    with state.lock:
        state.telem["wall"] = {"front": w["front"],
                               "right": w["right"],
                               "left": w["left"],
                               "back": w["back"]}
        # Бока симметричным окном - для замеров СТОЯ. Передняя и
        # задняя оси от окна бортов не зависят, поэтому те же.
        state.telem["wall_still"] = {"front": w["front"],
                                     "right": w["right_still"],
                                     "left": w["left_still"],
                                     "back": w["back"]}
        state.telem["wall_n"] = {"front": w["front_n"],
                                 "right": w["right_still_n"],
                                 "left": w["left_still_n"],
                                 "back": w["back_n"]}
        state.telem["scan_t"] = msg.get("t")
        state.telem["scan_period"] = msg.get("trev", 0.1)
        state.telem['timed_back'] = msg.get('timed_back', (None, None))
        state.scan_cnt += 1
    state.lidar_dropped = msg.get("dropped", 0)
    pts = msg.get("pts")
    if pts is not None:
        state.scan_pts.append((state.scan_cnt, msg.get("t"), msg.get("trev", 0.1), pts))


def _write_wall_debug(dbg, d, w):
    """Строка в wall_debug.log: бок только что стал "стена есть" (см. выше)."""
    dbg.write(f"{time.strftime('%H:%M:%S')}.{int(time.time()*1000)%1000:03d} "
              f"{d['side']} rise  right={w['right']}(n={w['right_n']})  "
              f"left={w['left']}(n={w['left_n']})  "
              f"pitch={esp_link.get_pitch():.1f} roll={esp_link.get_roll():.1f}  "
              f"pts=" + " ".join(f"{a}:{dd}" for a, dd in d["pts"]) + "\n")
    dbg.flush()


def get_walls():
    """Стены по последнему обороту (окно бортов для езды) - {'front',
    'right', 'left', 'back': мм или None}."""
    with state.lock:
        return dict(state.telem["wall"])


def get_wall_snapshot():
    """Sequence, end time, ranges and revolution duration from one scan."""
    with state.lock:
        return (state.scan_cnt, state.telem.get("scan_t"), dict(state.telem["wall"]),
                state.telem.get("scan_period", 0.1))


def get_timed_back(seq):
    with state.lock:
        return state.telem.get('timed_back', (None, None)) if seq == state.scan_cnt else (None, None)


def get_walls_still():
    """Замер симметричным боковым окном - только когда робот СТОИТ."""
    with state.lock:
        return dict(state.telem["wall_still"])


def get_wall_points():
    """Сколько точек вошло в медиану по каждой оси (диагностика: после масок
    ручек боковая дуга тонкая, и полезно видеть, не голодает ли она)."""
    with state.lock:
        return dict(state.telem["wall_n"])


def wall_present(d, thresh):
    """Стена есть: замер d (мм) есть и ближе порога thresh."""
    return d is not None and d < thresh


def wait_scan(n=2, timeout=3.0):
    """False - свежих сканов нет (лидар молчит): get_walls*() отдаст старое."""
    start = state.scan_cnt
    t0 = time.monotonic()
    while state.scan_cnt - start < n:
        if time.monotonic() - t0 > timeout:
            print(f"\n  [WARN] лидар молчит {timeout:.0f}с - стены могут быть устаревшими")
            return False
        time.sleep(0.04)
    return True


def wait_lidar_back():
    """Лидар молчит - это не конец заезда. Тормоз, ждём новые сканы сколько
    угодно, на пульте и дисплее фаза wait_lidar. Старые стены в карту не идут:
    замер после ожидания берётся с нового скана."""
    import dash_live            # здесь, а не наверху: dash_live импортирует нас
    esp_link.brake()
    phase = state.live.get("phase")
    print("\n  [WAIT] лидар молчит - жду, заезд продолжится")
    dash_live._note_phase("wait_lidar")
    t0 = time.monotonic()
    start = state.scan_cnt
    while state.scan_cnt - start < 3:
        time.sleep(0.1)
    dash_live._note_phase(phase)
    print(f"  лидар снова в порядке (ждали {time.monotonic() - t0:.1f}с)")


# Последний замер стоя - для повторного использования, пока робот не сдвинулся.
# На новой клетке стены мерились дважды подряд с места: align_at_wall() (он
# решает, что доводка не нужна) и тут же _measure_cell() - ещё ~0.3с на тот же
# самый вид. После поворота - то же с scan_sides(). Замер годен, пока робот
# не сдвинулся и не повернулся (одометрия, курс) и пока он свежий.
WALLS_REUSE_S = 1.5
WALLS_REUSE_CM = 0.3
WALLS_REUSE_DEG = 1.5


def _walls_cached(n):
    if state.walls_cache is None:
        return None
    t, cn, (l0, r0), yaw0, w = state.walls_cache
    l, r = esp_link.get_odo()
    if (cn >= n and time.monotonic() - t <= WALLS_REUSE_S
            and 0 <= time.monotonic() - (state.telem.get("scan_t") or 0) < 0.25
            and abs(l - l0) <= WALLS_REUSE_CM and abs(r - r0) <= WALLS_REUSE_CM
            and abs(turning.angle_err(esp_link.get_yaw(), yaw0)) <= WALLS_REUSE_DEG):
        return dict(w)
    return None


@timing._timed("лидар")
def measure_walls(n=config.WALL_VOTE_SCANS, fresh=False):
    """Медиана n сканов - одиночный промах лидара не решает за нас.

    Берёт замер СИММЕТРИЧНЫМ боковым окном (get_walls_still): все вызовы
    этой функции происходят, когда робот уже стоит в центре клетки, а там
    асимметричное окно SIDE_BACK_LIMIT только отнимает точки - см.
    SIDE_BACK_LIMIT_STILL в walls.py.

    Если робот с прошлого замера не двигался - возвращает его же (см.
    _walls_cache); fresh=True - всегда новый замер.
    """
    if not fresh:
        w = _walls_cached(n)
        if w is not None:
            return w
    FAR = 9999.0
    odo0, yaw0 = esp_link.get_odo(), esp_link.get_yaw()
    samples = {"front": [], "right": [], "left": [], "back": []}
    for _ in range(max(1, n)):
        if not wait_scan(1):
            state.walls_cache = None
            wait_lidar_back()
        w = get_walls_still()
        for k in samples:
            samples[k].append(w[k] if w[k] is not None else FAR)
    out = {}
    for k, lst in samples.items():
        m = sorted(lst)[len(lst) // 2]
        out[k] = None if m >= FAR else m
    # Положение - на НАЧАЛО замера: если робот сдвинулся во время него, то
    # следующая проверка это увидит и замер не переиспользует.
    state.walls_cache = (time.monotonic(), max(1, n), odo0, yaw0, dict(out))
    return out


def measure_front_confirmed(n=config.WALL_VOTE_SCANS, retries=3):
    """Как measure_walls(), но если перед вышел пустым - переспрашивает ещё
    несколько раз перед тем, как поверить, что там правда открыто.

    Нужно из-за YOLO: скан жертвы грузит CPU на сотни мс - секунду, фоновый
    поток лидара (чистый Python, тот же GIL) на это время отстаёт, и первые
    замеры после скана могут приходить пустыми оборотами. Ложное "стена
    есть" за переспрашивание ничего не стоит (лишний поворот), а ложное
    "открыто" ведёт прямо в стену - разная цена ошибки оправдывает разную
    осторожность.
    """
    w = measure_walls(n)
    for _ in range(retries):
        if w["front"] is not None:
            break
        if not wait_scan(2):
            wait_lidar_back()
        w = measure_walls(n, fresh=True)   # переспрос - это всегда новый замер
    return w


def _measure_cell(hdg):
    """Один голосующий замер стоя -> {'front','right','left','back': bool}
    (True=стена). confirmed, а не обычный measure_walls(): сразу после
    YOLO-скана жертвы лидар мог отдать пустой перед просто из-за нагрузки
    на CPU, а верить "открыто" здесь - ехать в стену. Прижатый концевик
    авторитетнее лидара: стена вплотную ближе собственного габарита робота
    и из замера отфильтрована."""
    return maze_map._walls_rel_map(measure_front_confirmed())
