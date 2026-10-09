"""Симулятор вживую - для пульта: python3 sim/live_sim.py [сценарий] [--speed 1] [--loop]

Тот же настоящий nav.py, что в run_scenarios.py, но в темпе реального
времени (--speed 2 - вдвое быстрее) и с поднятым публикатором live.py, как
на роботе. Пульт открывается отдельно:

    python3 pi/dash_web.py          ->  http://localhost:8888/

(или --dash: поднять его отсюда же). Так страницу пульта можно делать и
проверять без робота, а ночью - смотреть, что делает навигация.
--loop - после заезда начать сценарий заново; пульт переподключается и
пишет каждый заезд отдельным файлом.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import maze_sim as ms  # noqa: E402
import run_scenarios as rs  # noqa: E402


def run_once(name, speed, maze=None, noise=1.0):
    nav = ms.nav
    if maze is not None:
        # случайный лабиринт ночного прогона: плитки, метки, камеры, шумы
        import fake_hw
        import mazegen
        cap = 300
        sim = ms.Sim(mazegen.build_world(mazegen.generate(maze)), seed=maze, color=True,
                     noise=ms.Noise.level(noise, __import__("random").Random(maze)))
        name = f"лабиринт {maze}"
    else:
        make, cap, _ = rs.SCENARIOS[name]
        sim = ms.Sim(make())
    sim.realtime = speed
    ms.setup(sim)
    if maze is not None:
        fake_hw.install(sim)
    nav._live["sim"] = name
    nav._t_run = nav.time.monotonic()
    print(f"── сценарий {name}, темп x{speed} ──")
    cx, cy, hdg, cp, status = nav.flood_fill_run(cap)
    print(f"  статус разведки: {status}")
    if status != "fatal":
        cx, cy, hdg = nav.return_to_start(cx, cy, hdg, cp)
        nav._write_state(cx, cy, hdg, cp, "done")
    print(f"── конец: ({cx},{cy}), упёрся {sim.crashes} раз, {sim.t - 1000:.0f}с модели ──")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", nargs="?", default="two_ramps", choices=list(rs.SCENARIOS))
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--dash", action="store_true", help="поднять pi/dash_web.py")
    ap.add_argument("--maze", type=int, help="случайный лабиринт с этим номером (sim/mazegen.py)")
    ap.add_argument("--noise", type=float, default=1.0, help="уровень шумов для --maze")
    a = ap.parse_args()

    dash = None
    if a.dash:
        dash = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "pi", "dash_web.py")])
    ms.nav.start_live()
    try:
        while True:
            run_once(a.scenario, a.speed, a.maze, a.noise)
            if not a.loop:
                break
            time.sleep(3.0)
            # Новый nav - новый публикатор (пульт переподключится и начнёт
            # новую запись), stdout-перехват - прежний.
            old = ms.nav
            if old._live_pub is not None:
                old._live_pub.close()
            rs.fresh_nav()
            ms.nav._live_tee = old._live_tee
            ms.nav.start_live()
        print("заезд окончен - пульт ещё 5 с получает последний снимок")
        time.sleep(5.0)
    except KeyboardInterrupt:
        pass
    finally:
        if dash is not None:
            dash.terminate()


if __name__ == "__main__":
    main()
