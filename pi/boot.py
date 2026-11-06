"""Запуск железа и уборка - общие для обычного заезда (nav.py) и заданий
(task_*.py). Вынесено из nav.main() дословно.

    run = boot.start()          # порт ESP, телеметрия, лидар, камеры, лента,
                                # кнопка старта, калибровка
    ...сценарий...
    boot.stop(run)              # моторы стоп, порт, лидар, камеры, лента, GPIO

или одной строкой: boot.run(сценарий) - сценарий(run) между ними, Ctrl+C
не роняет уборку.

Флаги командной строки те же, что у nav.py: [N] --no-vision --no-web
--no-color --red-checkpoint.
"""
from __future__ import annotations

import os
import math
import signal
import subprocess
import sys
import threading
import time

import RPi.GPIO as GPIO
import serial
import watch_logic
from front_victim import FrontVictimDetector
from leds import NeoPixels
from victim_detect import CONF_FAST, SAVE_DIR, VictimDetector, YOLO_MODEL_PATH
from victim_watch import VictimWatcher

import config
import state
import timing
import dash_live
import esp_link
import floor_tiles
import lidar_walls
import recovery
import tile_rules


LINK_PORT = "/dev/serial0"
LINK_BAUD = 921600

# Отсчёт перед стартом - робот трогается сам, это только чтобы успеть
# отойти. 0 = ехать сразу.
START_DELAY_S = 0


# ══════════════════════════════════════════════════════════════
HUP_LOG = os.path.expanduser("~/maze/nav_hup.log")


def _on_hangup(_signum, _frame):
    """Оборвался SSH: терминала больше нет, и любой print упал бы, а то и сам
    SIGHUP убил бы заезд. Дальше едем, а вывод уходит в файл."""
    fd = os.open(HUP_LOG, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    os.dup2(fd, 1)
    os.dup2(fd, 2)
    os.close(fd)


class Run:
    """Что запуск отдаёт сценарию и уборке."""

    def __init__(self, cells, map_web_proc, t_run, run_seconds=config.RUN_BUDGET_S):
        self.cells = cells                  # потолок клеток (аргумент N)
        self.map_web_proc = map_web_proc    # процесс веб-карты или None
        self.t_run = t_run                  # time.monotonic() старта заезда
        self.run_seconds = run_seconds


def start(args=None, cameras=True):
    """Initialize hardware with cleanup even when startup fails midway."""
    context = Run(0, None, time.monotonic())
    state.link_stop.clear()
    state.lidar_stop.clear()
    try:
        return _start(args, cameras, context)
    except BaseException:
        stop(context)
        raise


def _configure_motion(args):
    if "--stop-each-cell" in args and "--nonstop" in args:
        raise ValueError("choose --stop-each-cell or --nonstop, not both")
    state.PER_CELL_STOP = "--stop-each-cell" in args


def _start(args, cameras, context):
    """Запуск железа до первого шага заезда. args - флаги и N (по умолчанию
    sys.argv[1:]); cameras=False - без камер и YOLO, лента остаётся."""
    signal.signal(signal.SIGHUP, _on_hangup)


    args = list(sys.argv[1:] if args is None else args)
    for arg in args:
        if arg.startswith("--run-seconds="):
            context.run_seconds = float(arg.split("=", 1)[1])
            if not math.isfinite(context.run_seconds) or context.run_seconds <= config.RETURN_RESERVE_S:
                raise ValueError("--run-seconds must exceed the return reserve")
    _configure_motion(args)
    print("Движение: " + ("остановка на каждой клетке (диагностика)" if state.PER_CELL_STOP
                         else "прямые участки без остановок"))
    use_vision = "--no-vision" not in args
    use_web = "--no-web" not in args
    if "--no-color" in args:
        # Без датчика цвета: ни чёрных, ни синих, ни чекпоинтов - для заездов,
        # где проверяется что-то другое (например, наблюдатель меток).
        state.USE_COLOR_SENSOR = False
        print("Датчик цвета отключён (--no-color): чёрные/синие/чекпоинты не видны")
    if "--red-checkpoint" in args:
        tile_rules.use_checkpoint(config.TILE_RED)
    print(f"Чекпоинт: {'КРАСНАЯ (--red-checkpoint, тренировка)' if state.CHECKPOINT_TILE == config.TILE_RED else 'зеркальная'} плитка")
    args = [a for a in args if not a.startswith("-")]
    # N теперь потолок на ОБЩЕЕ число разведанных клеток (страховка от
    # зацикливания) - нормальная разведка кончается сама, когда BFS не
    # находит больше непосещённых клеток. 300 с запасом покрывает любое
    # реальное поле RoboCup Rescue Maze.
    cells = int(args[0]) if args else 300

    # Раньше всего остального: карте нечего ждать телеметрию/YOLO, а
    # открыть её в браузере и увидеть "waiting" удобно уже во время
    # остального boot'а. --no-web выключает совсем (например, при
    # тестовых заездах без сети).
    map_web_proc = dash_live.start_map_web() if use_web else None
    context.map_web_proc = map_web_proc
    # Пульт: поток снимков поднимается всегда (без клиента он ничего не
    # делает), сама страница - только с вебом.
    dash_live.start_live()
    if use_web:
        dash_live.start_dash_web()
    # Дисплей сзади: отдельный процесс, берёт тот же живой снимок, что пульт.
    # Не на Pi или I2C выключен - ничего не делает.
    import oled
    oled.ensure_daemon()

    try:
        state.link = serial.Serial(LINK_PORT, LINK_BAUD, timeout=0.1, write_timeout=0.2)
    except serial.SerialException as e:
        print(f"[FATAL] Порт {LINK_PORT}: {e}")
        sys.exit(1)

    threading.Thread(target=esp_link._telem_thread, daemon=True).start()
    threading.Thread(target=lidar_walls._lidar_thread, daemon=True).start()

    print(f"BFS-разведка, потолок {cells} клеток.")

    print("Ожидание телеметрии ESP...")
    t0 = time.time()
    while not state.telem["telem_ok"]:
        if time.time() - t0 > 5:
            print("[FATAL] Нет телеметрии с ESP")
            print("        " + esp_link.link_report())
            sys.exit(1)
        time.sleep(0.05)

    # Реально достигнутая частота, а не заявленная. Измерять до того, как
    # поднимутся YOLO и лента: это опорное число, с которым потом сравнивать
    # просадки под нагрузкой (поток телеметрии делит GIL и с лидаром, и с
    # детектором, и именно это, а не размер пакета, делает её "медленной").
    n0, tm0 = state.telem_seq, time.monotonic()
    time.sleep(0.5)
    hz = (state.telem_seq - n0) / max(1e-3, time.monotonic() - tm0)
    print(f"  OK  yaw={esp_link.get_yaw():.1f}°  телеметрия {hz:.0f} Гц "
          f"(ожидается ~200), возраст пакета {esp_link.telem_age()*1000:.0f}мс")
    print("  линк: " + esp_link.link_report())
    esp_link.motors_on()       # после прошлого заезда они выключены (stop() ниже)
    if state.bad_ver or (state.magic_gaps and
                    max(state.magic_gaps.items(), key=lambda kv: kv[1])[0] != config.PKT_SIZE):
        print("[FATAL] Версии прошивки и nav.py разошлись - дальше ехать нельзя:\n"
              "        часть пакетов случайно проходит CRC, и yaw/одометрия/флаги\n"
              "        читаются из мусора. Перезалей прошивку и скопируй .py на Pi.")
        sys.exit(1)
    if hz < 120:
        print("  [WARN] телеметрия идёт медленнее половины номинала - "
              "смотреть надо на загрузку CPU и GIL, не на размер пакета")

    # Сторож подъёма отключён: остановка из-за снятия робота с поля не нужна.
    # _lifted теперь никогда не выставляется, так что handle_lift() и все
    # проверки _lifted.is_set() остаются неактивными.
    # threading.Thread(target=_lift_thread, daemon=True).start()

    if use_vision:
        # cameras=False - задание без меток: ленту поднимаем, камеры и YOLO нет.
        if cameras:
            print("Инициализация детекторов жертв (YOLO, правая камера)...")
            try:
                state.victim_r = VictimDetector(cam_idx=watch_logic.CAM_IDX["right"])
            except Exception as e:
                print(f"  [WARN] правая камера недоступна: {e}")
                state.victim_r = None

            print("Инициализация детекторов жертв (YOLO, левая камера)...")
            try:
                state.victim_l = VictimDetector(cam_idx=watch_logic.CAM_IDX["left"])
            except Exception as e:
                print(f"  [WARN] левая камера недоступна: {e}")
                state.victim_l = None

            cams = {s: d.cam for s, d in (("right", state.victim_r), ("left", state.victim_l))
                    if d is not None}
            if cams:
                # Воркер грузит YOLO в своём процессе, пока здесь поднимается
                # лента и ждём кнопку; arm() до его готовности ничего не делает.
                print("Запуск наблюдателя меток на ходу (YOLO отдельным процессом)...")
                try:
                    state.watcher = VictimWatcher(cams, watch_logic.prefer_ncnn(YOLO_MODEL_PATH),
                                             SAVE_DIR, CONF_FAST)
                except Exception as e:
                    print(f"  [WARN] наблюдатель не поднялся: {e}")
                    state.watcher = None

            print("Инициализация детектора жертв (YOLO, передняя USB-камера)...")
            try:
                state.victim_f = FrontVictimDetector()
            except Exception as e:
                print(f"  [WARN] передняя камера недоступна: {e}")
                state.victim_f = None

        print("Инициализация ленты индикации...")
        try:
            state.leds = NeoPixels()
        except Exception as e:
            print(f"  [WARN] лента недоступна: {e}")
            state.leds = None
    else:
        print("Зрение отключено (--no-vision)")

    # Кнопка - раньше первого замера лидара и тем более раньше проезда:
    # пока не нажата, лидар не проверяем и никуда не едем.
    dash_live._note_phase("wait_button")
    recovery.wait_start_button()
    context.t_run = time.monotonic()  # count post-button calibration as well
    dash_live._note_phase("calib")
    if state.watcher is not None and not state.watcher.ready.is_set():
        print("  [WARN] наблюдатель меток ещё грузится - включится, когда будет готов")
    # Курс поля - именно сейчас: кнопку жмут, когда робот уже стоит на старте
    # ровно по сетке. Всё, что было до нажатия (запуск за полигоном, перенос
    # в руках), в систему отсчёта не попадает. Если робота тронут во время
    # отсчёта ниже - гироскоп это поворот, он его учтёт.
    time.sleep(config.RESTART_SETTLE_S)
    if not esp_link.gyro_fresh():
        # Не выход: ждём гироскоп сколько угодно, на пульте - wait_gyro.
        print("  [WAIT] нет свежего курса BNO085 - жду, заезд начнётся сам")
        dash_live._note_phase("wait_gyro")
        while not esp_link.gyro_fresh():
            time.sleep(0.2)
        dash_live._note_phase("calib")
        time.sleep(config.RESTART_SETTLE_S)
    state.YAW_ORIGIN = esp_link.get_yaw()
    print(f"  YAW_ORIGIN={state.YAW_ORIGIN:.1f}° (по кнопке)")
    # Пауза BTN_1 живёт на ESP до команды "R" и переживает перезапуск
    # nav.py: нажали BTN_1 в прошлом заезде или на стенде - и новый заезд
    # сразу ушёл бы в сон. Кнопка старта и есть "поехали", снимаем её.
    if esp_link.get_esp_paused():
        print("  ESP на паузе с прошлого раза - снимаю")
        esp_link.resume_esp()

    print("Ожидание лидара...")
    if not lidar_walls.wait_scan(3, timeout=5.0):
        print("[FATAL] Лидар молчит")
        sys.exit(1)
    w0 = lidar_walls.get_walls()

    def _f(v): return f"{v:.0f}" if v is not None else "—"
    print(f"  OK  F={_f(w0['front'])} R={_f(w0['right'])} L={_f(w0['left'])}"
          f"  (отдельный процесс, неполных оборотов: {state.lidar_dropped})")

    # Отсчёт после нажатия - чтобы успеть убрать руки / отойти от робота.
    # BTN_1 на ESP во время заезда - перезапуск с чекпоинта (restart_from_checkpoint).
    for i in range(START_DELAY_S, 0, -1):
        print(f"\r  Старт через {i}...", end="", flush=True)
        time.sleep(1.0)
    print("\r  Старт!            ")

    floor_tiles.load_tile_calib()
    floor_tiles.calibrate_white()          # pitch0; белая - только если нет калибровки
    esp_link.zero_odo()
    time.sleep(0.1)
    # Робота могли поправить руками, пока шёл отсчёт и калибровка - начинаем
    # заезд с чистым флагом, иначе flood_fill_run сразу уйдёт в handle_lift.
    state.lifted.clear()

    state.t_run = time.monotonic()
    context.cells = cells
    return context


def stop(run):
    """Уборка после заезда: моторы стоп, порт ESP, лидар, веб, камеры, лента."""
    print(timing.time_report({}, time.monotonic() - run.t_run, prefix="⏱ весь заезд"))
    print(timing.tick_report())
    # Ошибка порта не должна обрывать остальную уборку ниже.
    try:
        if state.link is not None and state.link.is_open:
            esp_link.brake()
            time.sleep(0.1)
            esp_link.stop()
            esp_link.motors_off()  # между заездами моторы выключены
            state.link.flush()     # последние команды - на провод до закрытия
    except (serial.SerialException, OSError) as e:
        print(f"  [WARN] порт ESP при выходе: {e}")
    finally:
        state.link_stop.set()
        if state.link is not None:
            try:
                state.link.close()
            except (serial.SerialException, OSError):
                pass
    state.lidar_stop.set()
    print(f"  лидар: неполных оборотов отброшено {state.lidar_dropped}, "
          f"перезапусков процесса {state.lidar_restarts}")
    print("  линк ESP: " + esp_link.link_errors())
    # Карта не критична для самого заезда, поэтому без долгого ожидания:
    # если процесс не отвечает на SIGTERM за секунду, просто оставляем
    # его - следующий запуск лишь получит [WARN] про занятый порт.
    for name, proc in (("lidar", state.lidar_proc), ("map", run.map_web_proc)):
        if proc is None:
            continue
        try:
            if proc.poll() is None:
                proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)
        except (OSError, subprocess.TimeoutExpired) as e:
            print(f"  [WARN] cleanup {name}: {e}")
    for name, resource in (("watcher", state.watcher), ("right camera", state.victim_r),
                           ("left camera", state.victim_l), ("front camera", state.victim_f),
                           ("LEDs", state.leds)):
        if resource is not None:
            try:
                resource.close()
            except Exception as e:
                print(f"  [WARN] cleanup {name}: {e}")
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"  [WARN] GPIO cleanup: {e}")


def run(scenario, args=None, cameras=True):
    """start() → scenario(run) → stop(): уборка и при Ctrl+C, и при ошибке."""
    r = start(args, cameras)
    try:
        scenario(r)
    except KeyboardInterrupt:
        print("\nОстановлено.")
    finally:
        stop(r)
