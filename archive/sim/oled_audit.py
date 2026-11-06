"""Дисплей робота (pi/oled.py) на настоящих заездах симулятора.

    python3 sim/oled_audit.py -n 50 --seed 8000 --noise 0,1
    python3 sim/oled_audit.py --one --seed 8033 --noise 1     # один заезд, JSON

Заезд идёт как night.run_one, а на каждом такте собирается тот же снимок,
что nav.py раздаёт по live.py (с логом и картой), и скармливается
oled.Feed - как демону дисплея на роботе. Экран рисуется 5 раз в секунду
модельного времени, и проверяется:

  - отрисовка не падает и не показывает «ОШИБКА ЭКРАНА»;
  - текст не обрезается (oled.clip ничего не режет);
  - на каждую прочитанную метку появляется её экран, и без метки его нет;
  - сколько стоит кадр (мс на этой машине; на Pi 5 - того же порядка).

Образцы в pi/tests/test_oled.py проверяют экраны на придуманных данных; здесь -
на тех, что nav.py выдаёт на самом деле (так нашлись «толчок» длиннее
строки и фейки в счёте жертв).
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import random
import subprocess
import sys
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))

import night  # noqa: E402

SNAP_PERIOD_S = 0.1          # как live.py (LIVE_HZ = 10)


class Watcher:
    """Хук такта модели: снимок -> Feed, кадр -> проверки."""

    def __init__(self, sim, nav, tee, epoch0):
        import oled
        self.oled = oled
        self.sim, self.nav, self.tee, self.epoch0 = sim, nav, tee, epoch0
        self.feed = oled.Feed(start=False)
        self.feed.connected = True
        self.sys = oled.SysInfo(start=False)
        self.sys.info.update(ip="192.168.78.26", ssid="maze", temp=55.0, mem=20.0,
                             uptime=600.0, throttled=0)
        self.cv = oled.Canvas()
        self.n = 0
        self.log_seq = 0
        self.next_snap = self.next_frame = sim.t
        self.frames = 0
        self.ms = []
        self.errors = []
        self.cut = Counter()
        self.screens = Counter()
        self.victim_shown = set()     # когда пришли метки, чей экран был нарисован
        self.cur_screen = None
        real_clip = oled.clip

        def clip(s, n=oled.COLS):
            r = real_clip(s, n)
            if r != str(s):
                self.cut[(self.cur_screen, str(s))] += 1
            return r
        oled.clip = clip

    def on_step(self, sim):
        if sim.t >= self.next_snap:
            self.next_snap += SNAP_PERIOD_S
            try:
                s = self.nav._live_snapshot(self.n)
            except Exception as e:          # noqa: BLE001
                self.errors.append(f"снимок: {type(e).__name__}: {e}")
                return
            self.n += 1
            s["status"] = self.tee.status
            lines = self.tee.since(self.log_seq)
            if lines:
                s["log"] = lines
                self.log_seq = lines[-1][0]
            # через JSON, как по сокету: кортежи -> списки, numpy -> числа
            s = json.loads(json.dumps(s, default=str))
            self.feed.take(s, sim.t)
        if sim.t >= self.next_frame and self.feed.snap is not None:
            self.next_frame += 1.0 / self.oled.FPS
            self.frame(sim.t)

    def which(self, st):
        o = self.oled
        if st.get("msg"):
            return "msg"
        snap = st["snap"]
        ph = snap.get("phase")
        if ph == "wait_button":
            return "preflight"
        if ph in ("boot", "calib"):
            return "boot"
        if ph in o.WAIT_HINT:
            return "wait"
        if ph in ("blinking", "done"):
            return "finish"
        if st.get("victim"):
            return "victim"
        return "run"

    def frame(self, t):
        o = self.oled
        st = o.build_state(t, self.feed, self.sys)
        st["wall"] = self.epoch0 + (t - self.sim.t0)     # время лога - модельное
        self.cur_screen = self.which(st)
        self.screens[self.cur_screen] += 1
        t0 = time.perf_counter()
        try:
            o.compose(self.cv, st)
        except Exception as e:              # noqa: BLE001
            self.errors.append(f"{self.cur_screen}: {type(e).__name__}: {e}")
            return
        self.ms.append((time.perf_counter() - t0) * 1000.0)
        self.frames += 1
        if self.cur_screen == "victim" and self.feed.victim is not None:
            self.victim_shown.add(self.feed.victim[1])


def audit_run(seed, noise, spec):
    import maze_sim as ms
    import fake_hw
    import mazegen
    import live

    nav = ms.nav
    world = mazegen.build_world(spec)
    rng_noise = random.Random(seed * 7919 + int(noise * 100))
    sim = ms.Sim(world, noise=ms.Noise.level(noise, rng_noise), seed=seed, color=True)
    sim.limit = night.RUN_LIMIT_S
    epoch0 = time.time()
    tee = live.LogTee(io.StringIO(), keep=100000, clock=lambda: epoch0 + (sim.t - sim.t0))
    w = Watcher(sim, nav, tee, epoch0)
    sim.hooks.append(w.on_step)
    status = "?"
    with contextlib.redirect_stdout(tee):
        try:
            ms.setup(sim)
            fake_hw.install(sim)
            nav._t_run = nav.time.monotonic()
            cx, cy, hdg, cp, status = nav.flood_fill_run(300)
            if status != "fatal":
                cx, cy, hdg = nav.return_to_start(cx, cy, hdg, cp)
                if (cx, cy) == (0, 0):
                    nav._write_state(cx, cy, hdg, cp, "blinking")
                    nav.blink_start_return()
                    nav._write_state(cx, cy, hdg, cp, "done")
        except ms.SimTimeUp:
            status = "time_up"

    # Метки: каждая прочитанная должна попасть на экран, и экран метки -
    # только после чтения (не раньше, не без неё).
    logged = [v.get("label") for v in nav._victim_log]
    shown = w.victim_shown
    missing = max(0, len(logged) - len(shown))
    ms_sorted = sorted(w.ms)
    return {"seed": seed, "noise": noise, "status": status, "frames": w.frames,
            "errors": w.errors[:5], "n_errors": len(w.errors),
            "cut": [[scr, s, n] for (scr, s), n in w.cut.most_common(10)],
            "screens": dict(w.screens), "victims_logged": len(logged),
            "victim_screens": len(shown), "victims_missing": missing,
            "ms_median": round(ms_sorted[len(ms_sorted) // 2], 2) if ms_sorted else None,
            "ms_max": round(ms_sorted[-1], 2) if ms_sorted else None}


def main():
    ap = argparse.ArgumentParser(description="Дисплей на заездах симулятора")
    ap.add_argument("-n", type=int, default=24)
    ap.add_argument("--seed", type=int, default=8000)
    ap.add_argument("--noise", default="0")
    ap.add_argument("-j", "--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--one", action="store_true", help="один заезд в этом процессе, JSON в stdout")
    a = ap.parse_args()

    if a.one:
        import mazegen
        r = audit_run(a.seed, float(a.noise), mazegen.generate(a.seed))
        print("OLED_AUDIT " + json.dumps(r, ensure_ascii=False))
        return

    noises = [float(x) for x in a.noise.split(",")]
    jobs = [(s, nz) for s in range(a.seed, a.seed + a.n) for nz in noises]

    def go(job):
        s, nz = job
        p = subprocess.run([sys.executable, __file__, "--one", "--seed", str(s), "--noise", str(nz)],
                           capture_output=True, text=True, timeout=night.WALL_TIMEOUT_S)
        for ln in p.stdout.splitlines():
            if ln.startswith("OLED_AUDIT "):
                return json.loads(ln[len("OLED_AUDIT "):])
        return {"seed": s, "noise": nz, "crash": (p.stderr or p.stdout)[-400:]}

    t0 = time.monotonic()
    with ThreadPoolExecutor(a.jobs) as ex:
        rs = list(ex.map(go, jobs))
    crashed = [r for r in rs if "crash" in r]
    rs = [r for r in rs if "crash" not in r]
    frames = sum(r["frames"] for r in rs)
    errs = [(r["seed"], r["noise"], e) for r in rs for e in r["errors"]]
    cut = Counter()
    for r in rs:
        for scr, s, n in r["cut"]:
            cut[(scr, s)] += n
    screens = Counter()
    for r in rs:
        screens.update(r["screens"])
    vl = sum(r["victims_logged"] for r in rs)
    miss = [(r["seed"], r["noise"], r["victims_missing"]) for r in rs if r["victims_missing"]]
    worst_ms = max((r["ms_max"] or 0) for r in rs) if rs else 0
    med = sorted(r["ms_median"] for r in rs if r["ms_median"] is not None)
    print(f"заездов {len(rs)} ({time.monotonic() - t0:.0f} с), процесс заезда упал: {len(crashed)}")
    print(f"кадров {frames}: " + ", ".join(f"{k} {v}" for k, v in screens.most_common()))
    print(f"ошибок отрисовки: {len(errs)}")
    for s, nz, e in errs[:8]:
        print(f"  поле {s} шум {nz:g}: {e}")
    print(f"обрезанных строк: {sum(cut.values())} кадров, разных {len(cut)}")
    for (scr, s), n in cut.most_common(10):
        print(f"  {scr}: {s!r} ×{n}")
    print(f"меток прочитано {vl}, без своего экрана: {sum(m for _, _, m in miss)}")
    for s, nz, m in miss[:8]:
        print(f"  поле {s} шум {nz:g}: {m}")
    if med:
        print(f"кадр: медиана {med[len(med) // 2]:.1f} мс, худший {worst_ms:.1f} мс (на этой машине)")
    for r in crashed[:3]:
        print("  упал:", r["seed"], r["noise"], r["crash"][-300:])


if __name__ == "__main__":
    main()
