"""Дашборд, живая карта, светодиоды и детектор подъёма. Перенесено из nav.py как есть."""
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time

from leds import RED, SLEEP, blink_return
from walls import BACK_THRESH, SIDE_THRESH, WALL_THRESH

import config
import state
import esp_link
import ramps


# ── Подъём робота с поля (возврат на чекпоинт) ───────────────
# ── Детектор в двух шагах: вертикаль, потом цвет ─────────────
# Шаг 1: длительное вертикальное ускорение - повод посмотреть.
# Шаг 2: датчик цвета видит не плитку - значит, робота нет на поле.
# Всё. Предыдущая версия проверяла ещё дрожание корпуса, прокрутку колёс и
# пропажу стен по лидару, и каждое из трёх оказалось вредным:
#
#   ДРОЖАНИЕ  - робот трясёт сам себя. Гул моторов на удержании и серво
#               дозатора дали ложное срабатывание прямо посреди заезда.
#   КОЛЁСА    - доказывают что-либо, только пока мы не крутим моторы сами;
#               под командой разгруженные колёса в воздухе вертятся не хуже,
#               чем по полу, и сбрасывали подтверждение именно тогда, когда
#               робот точно был на руках.
#   ЛИДАР     - слепнет ровно в нужный момент: рука и корпус человека честно
#               читаются как стены. В логе подряд шли back=183, right=139
#               left=90, front=186 right=68 - шестьдесят восемь миллиметров
#               это ладонь вплотную.
#
# Датчик цвета свободен от всех трёх бед: он смотрит в пол с высоты пары
# миллиметров, рука его не заслоняет, моторы и серво на него не влияют.

# Шаг 1. Подъём меняет ВЫСОТУ, и всё это время вертикальное ускорение держит
# один знак; прыжки на препятствии знак меняют и за секунду усредняются
# почти в ноль, а гудящие моторы робота не двигают вообще. Замеры: подъём
# рукой до 3.37 и 1.21 (разброс большой), катание через препятствие до 1.68,
# неподвижно на полу 0.01 при шуме +-0.055.
#
# Порог низкий и лишние взводы безвредны: решает шаг 2, а он молчит, пока
# под роботом плитка. В логе заезда видно, что взводится он часто.
LIFT_AZ_MS2 = 1.0
LIFT_AZ_WINDOW_S = 1.0
# Одиночный выброс решать не должен - датчик обновляется ~30Гц, так что
# 0.4с это больше десятка подряд согласных замеров.
LIFT_AIR_CONFIRM_S = 0.4
LIFT_ARM_TIMEOUT_S = 5.0    # посмотрели, плитка на месте - снимаем подозрение

# Снимок карты/меток для веб-страницы. Пишется маленьким JSON-файлом в
# редкие моменты (приезд в клетку, находка метки), а не каждый тик петли
# управления - в тайминг самого контура (см. STOP_LEAD_S, MOTOR_CMD_HZ)
# лишний файловый I/O добавлять нельзя. Отдельный процесс (map_web.py) этот
# файл читает и раздаёт странице - тот же приём, что и у camera_web/app.py
# и detect_app.py, только без Flask внутри самого nav.py.
STATE_PATH = os.path.expanduser("~/maze/state.json")


# ══════════════════════════════════════════════════════════════
#  Подъём робота с поля
# ══════════════════════════════════════════════════════════════
def leds_lifted(on):
    """Красный на ленте, пока робота держат на руках.

    Безопасна во всех положениях: лента опциональна (её может не быть -
    тогда _leds is None), а во время подъёма параллельно может доигрывать
    мигание жертвы, которое тоже пишет в ленту. Поэтому вызывается
    повторно из цикла ожидания в handle_lift() - последний писавший и
    выигрывает, а цикл пишет чаще.
    """
    if state.leds is None:
        return
    try:
        if on:
            state.leds.fill(*RED)
        else:
            state.leds.clear()
        state.leds.show()
    except Exception:
        pass


def leds_sleep(on):
    """Синий на ленте, пока робот спит по BTN_1 - см. restart_from_checkpoint."""
    if state.leds is None:
        return
    try:
        if on:
            state.leds.fill(*SLEEP)
        else:
            state.leds.clear()
        state.leds.show()
    except Exception:
        pass


def _lift_thread():
    """Сторожевой поток: вертикальное ускорение, потом датчик цвета.

    Отдельный поток, а не проверка в петле: во время YOLO-скана жертвы
    основные петли стоят до секунды, и подъём в этот момент иначе не
    заметили бы вовсе. Поток только выставляет флаг и глушит моторы -
    всё остальное делает handle_lift() из основного потока.

    Два шага, и ничего сверх них - см. комментарии над LIFT_* о том, какие
    признаки пробовались до этого и почему каждый из них пришлось убрать.
    """
    hist = []            # (время, az) за LIFT_AZ_WINDOW_S
    armed_at = None      # когда возникло подозрение
    air_since = None     # с какого момента датчик непрерывно видит не плитку
    while True:
        time.sleep(config.LIFT_POLL_S)
        # Пока подъём разбирает основной поток, детектору делать нечего.
        if state.lifted.is_set() or not esp_link.telem_fresh():
            hist.clear()
            armed_at = air_since = None
            continue

        now = time.monotonic()
        hist.append((now, esp_link.get_az()))
        while hist and now - hist[0][0] > LIFT_AZ_WINDOW_S:
            hist.pop(0)

        # ── Шаг 1: длительное вертикальное ускорение ──────────
        if armed_at is None:
            az_mean = sum(z for _, z in hist) / len(hist)
            if abs(az_mean) > LIFT_AZ_MS2:
                armed_at = now
                air_since = None
                print(f"\n  [подъём?] вертикаль {az_mean:+.2f} м/с² за "
                      f"{LIFT_AZ_WINDOW_S:.1f}с - смотрю под робота")
            continue

        if now - armed_at > LIFT_ARM_TIMEOUT_S:
            armed_at = air_since = None
            continue

        # ── Шаг 2: что под роботом ────────────────────────────
        r, g, b, c = esp_link.get_rgbc()
        rb = (r - b) / c if c >= config.TILE_MIN_C else -1.0
        if rb <= config.LIFT_AIR_RB:
            if air_since is not None:
                print(f"  [подъём?] отбой: под роботом плитка, "
                      f"(r-b)/c={rb:+.3f} (порог {config.LIFT_AIR_RB:.2f})")
            air_since = None
            continue

        if air_since is None:
            air_since = now
            continue
        if now - air_since < LIFT_AIR_CONFIRM_S:
            continue

        print(f"  [подъём?] под роботом не плитка: (r-b)/c={rb:+.3f} "
              f"держится {LIFT_AIR_CONFIRM_S:.1f}с")
        state.lifted.set()
        leds_lifted(True)
        # Именно stop(), а не brake(): держать 0 dps в руках - это драка с
        # человеком и четыре греющихся мотора. Короткий brake перед этим
        # гасит инерцию, если робота подняли прямо на ходу.
        try:
            esp_link.brake()
            time.sleep(0.05)
            esp_link.stop()
        except Exception:
            pass


def _map_dict(cx, cy, hdg, checkpoint, phase):
    """Карта + метки одним словарём - для state.json и для пульта. Читается
    и из потока пульта, пока главный поток карту меняет: list() над dict/set
    в CPython проходит под GIL целиком, так что копии согласованы."""
    return {
        "ts": time.strftime("%H:%M:%S"),
        "phase": phase,
        "robot": {"cx": cx, "cy": cy, "hdg": hdg, "dir": config._DIR_NAME[hdg]},
        "checkpoint": {"cx": checkpoint[0], "cy": checkpoint[1],
                       "hdg": checkpoint[2]},
        # "x,y" вместо кортежа - ключи JSON обязаны быть строками.
        "maze": {f"{x},{y}": list(walls) for (x, y), walls in list(state.maze.items())},
        "visited": [[x, y] for x, y in list(state.visited)],
        "blocked": [[x, y] for x, y in list(state.blocked)],
        "blue_done": [[x, y] for x, y in list(state.blue_cells)],
        "blue_visits": {f"{x},{y}": n for (x, y), n in list(state.blue_visits.items())},
        "level_stride": config.LEVEL_STRIDE,
        "ramps": [[x, y, d, bx, by, n, sgn]
                  for (x, y, d), (bx, by, n, sgn) in list(state.ramps.items()) if sgn > 0],
        "victims": list(state.victim_log),
    }


def _note_state(cx, cy, hdg, checkpoint, phase):
    """Поза и фаза для пульта (live.py) - в памяти, без I/O."""
    state.live.update(cx=cx, cy=cy, hdg=hdg, cp=tuple(checkpoint), phase=phase,
                 map_ver=state.live["map_ver"] + 1)


def _note_phase(phase):
    """Фаза до заезда (кнопка, калибровка на старте) - позы ещё нет, только
    пульту и дисплею сказать, чего ждём."""
    state.live.update(phase=phase, map_ver=state.live["map_ver"] + 1)


def _write_state(cx, cy, hdg, checkpoint, phase):
    """Снимок карты + найденных меток -> STATE_PATH, для map_web.py.

    Пишется во ВРЕМЕННЫЙ файл и переименовывается (os.replace) - атомарно
    с точки зрения читателя: map_web.py никогда не увидит наполовину
    записанный JSON, даже если откроет файл ровно в момент записи.
    Ошибки диска/прав не должны ронять заезд - лог и молча пропускаем.
    """
    _note_state(cx, cy, hdg, checkpoint, phase)
    try:
        data = _map_dict(cx, cy, hdg, checkpoint, phase)
        tmp = STATE_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, STATE_PATH)
    except OSError as e:
        print(f"  [WARN] не удалось записать {STATE_PATH}: {e}")


def blink_start_return():
    """Индикация возврата на старт - ТЕМ ЖЕ светодиодом/лентой, что и для
    жертв, но своим рисунком: ВКЛ 1с/ВЫКЛ 1с, не менее 10 секунд (см.
    leds.blink_return). Тормоз держим фоновым потоком всё это время - иначе
    на наклонном полу робота может увести с центра стартовой клетки
    (то же самое hold_while, что и при индикации жертвы)."""
    if state.leds is None:
        print("  [WARN] лента недоступна — мигание при возврате пропущено")
        return
    print("  ● Мигание на старте (возврат распознан): 1с/1с, ≥10с")
    esp_link.hold_while(blink_return, state.leds)


MAP_WEB_PORT = 8001   # см. map_web.py


def _local_ip():
    """Лучшая догадка о собственном IP в локальной сети - только чтобы
    напечатать ссылку на карту. connect() на UDP-сокете ничего не
    отправляет в сеть (он лишь выбирает исходящий интерфейс по таблице
    маршрутизации), поэтому работает и без интернета - было бы это самое
    подключение к Wi-Fi/роутеру."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return "<IP робота>"


def start_map_web():
    """Поднимает map_web.py ОТДЕЛЬНЫМ процессом, чтобы карта открывалась
    сама вместе с заездом - без второго терминала. Именно процесс (через
    subprocess), а не поток в этом же nav.py: у него свой интерпретатор и
    свой GIL, так что Flask-сервер никак не отнимает CPU у моторной петли,
    лидара и YOLO (см. шапку файла про GIL-конкуренцию). Если порт занят
    или что-то не так - падает в СВОЙ лог (map_web.log), а не в этот заезд.
    """
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_web.py")
    log_path = os.path.expanduser("~/maze/map_web.log")
    try:
        log_f = open(log_path, "w")
        proc = subprocess.Popen([sys.executable, script],
                                 stdout=log_f, stderr=subprocess.STDOUT)
        log_f.close()   # у дочернего процесса уже своя копия дескриптора
    except OSError as e:
        print(f"  [WARN] карта не запущена: {e} (лог: {log_path})")
        return None
    print(f"  Карта: http://{_local_ip()}:{MAP_WEB_PORT}/  (лог: {log_path})")
    return proc


# ══════════════════════════════════════════════════════════════
# ══════════════════════════════════════════════════════════════
#  Пульт: живой снимок для dash_web.py и дисплея (см. live.py)
# ══════════════════════════════════════════════════════════════
DASH_WEB_PORT = 8888     # см. dash_web.py


def _r(v, nd=1):
    return None if v is None else round(v, nd)


def _live_snapshot(n):
    """Снимок для пульта - из потока live.py, 10 раз в секунду. Только
    читает: копии под _lock, остальное - атомарные чтения глобалов. Карта
    целиком - раз в секунду и сразу, как только поменялась."""
    now = time.monotonic()
    with state.lock:
        st = dict(state.telem)
    lv = dict(state.live)
    scan = state.scan_pts[-1] if state.scan_pts else None
    try:
        act = state.tstack[-1][0] if state.tstack else None
    except IndexError:
        act = None
    tick = {}
    for k, s in state.tick_ms.items():
        v = s.v[-200:]
        if v:
            v = sorted(v)
            tick[k] = [round(v[len(v) // 2], 2), round(v[-1], 2)]
    d = {
        "t": round(now, 3),
        "run_s": None if state.t_run is None else round(now - state.t_run, 2),
        "phase": lv["phase"],
        "act": act,
        "pose": {"cx": lv["cx"], "cy": lv["cy"], "hdg": lv["hdg"], "cp": list(lv["cp"]),
                 "level": ramps.level_of(lv["cy"])},
        "drive": state.drive_live,
        "front_floor": getattr(state, "front_floor_live", None),
        "motors": {"ids": [4, 1, 2, 3], "voltage": st.get("motor_voltage", []),
                   "temp": st.get("motor_temp", []), "error": st.get("motor_error", [])},
        "imu": {"yaw": _r((st["yaw"] - state.YAW_ORIGIN) % 360.0, 2), "rate": _r(st["yaw_rate"]),
                "pitch": _r(st["pitch"], 2), "roll": _r(st["roll"], 2), "az": _r(st["az"], 3)},
        "odo": [_r(st["lcm"]), _r(st["rcm"])],
        "enc": [st["enc_left_online"], st["enc_right_online"]],
        "tile": state.tile_last,
        "rgbc": list(st["rgbc"]),
        "walls": {k: _r(v) for k, v in st["wall"].items()},
        "wall_n": st["wall_n"],
        "scan": scan[3] if scan else [],
        "scan_n": state.scan_cnt,
        "bump": [st["clicker1"], st["clicker2"]],
        "btn1": st["btn1"],
        "esp": st["esp_status"],
        "fresh": [st["gyro_fresh"], st["color_fresh"]],
        "telem_n": state.telem_seq,
        "telem_age_ms": round((now - state.telem_t) * 1000) if state.telem_t else None,
        "link": {"crc": state.bad_crc, "lost": state.seq_lost, "ver": state.bad_ver,
                 "cmd_err": st["esp_cmd_err"]},
        "lidar": {"dropped": state.lidar_dropped, "restarts": state.lidar_restarts,
                  "age_ms": round((now - st["scan_t"]) * 1000) if st.get("scan_t") else None},
        "kits": {s: config.KITS_PER_COLUMN - u for s, u in state.kits_used.items()},
        "kits_max": config.KITS_PER_COLUMN,
        "tick_ms": tick,
        "cams": state.watcher.live_info() if state.watcher is not None else None,
        "victims_n": len(state.victim_log),
        # настоящие жертвы (мигали), без фейковых мишеней - для дисплея
        "victims_real": sum(1 for v in list(state.victim_log) if v.get("blink")),
        "visited_n": len(state.visited),
        "t_left": None,          # в этой версии нет бюджета времени
        "side": state.side_pref,
        "map_ver": lv["map_ver"],
        "color_on": state.USE_COLOR_SENSOR,
        "sim": None,
        "thr": [WALL_THRESH, SIDE_THRESH, BACK_THRESH],
    }
    if (n % 10 == 0 or lv["map_ver"] != state.live_sent["map_ver"]
            or len(state.victim_log) != state.live_sent["victims"]):
        d["map"] = _map_dict(lv["cx"], lv["cy"], lv["hdg"], lv["cp"], lv["phase"])
        state.live_sent["map_ver"], state.live_sent["victims"] = lv["map_ver"], len(state.victim_log)
    return d


def start_live():
    """Поток публикации снимков для пульта + перехват stdout для его консоли.
    Не поднялся (порт занят прошлым nav.py) - заезд от этого не зависит."""
    import live
    if state.live_tee is None:
        state.live_tee = live.LogTee(sys.stdout)
        sys.stdout = state.live_tee
    try:
        state.live_pub = live.LivePublisher(_live_snapshot, tee=state.live_tee)
    except OSError as e:
        print(f"  [WARN] пульт: порт {live.LIVE_PORT} занят ({e}) - живых данных не будет")
        state.live_pub = None
    return state.live_pub


def start_dash_web():
    """dash_web.py отдельным процессом - как start_map_web()."""
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dash_web.py")
    log_path = os.path.expanduser("~/maze/dash_web.log")
    try:
        log_f = open(log_path, "w")
        proc = subprocess.Popen([sys.executable, script],
                                stdout=log_f, stderr=subprocess.STDOUT)
        log_f.close()
    except OSError as e:
        print(f"  [WARN] пульт не запущен: {e} (лог: {log_path})")
        return None
    print(f"  Пульт: http://{_local_ip()}:{DASH_WEB_PORT}/  (лог: {log_path})")
    return proc
