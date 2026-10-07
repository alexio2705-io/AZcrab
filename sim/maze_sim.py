"""Симулятор робота MAZE2027: настоящий pi/nav.py без железа.

Запускается на Маке (python3 sim/run_scenarios.py). Подменяются только
модули железа (serial, RPi.GPIO, лента, камеры) и время: часы виртуальные,
nav.time.sleep и такт по телеметрии (wait_telem) двигают модель на 5 мс.
Всё остальное - разведка, drive_cells, ассист, горки, карта, план без
остановок - работает как на роботе.

Модель (грубая, но с теми эффектами, ради которых она нужна):
  - колёса: уставки M:/T:/B с инерцией и пределом ускорения; в режиме
    скорости подъём и бугорок "не тянутся" (скорость падает), режим момента
    (T:) тянет до своего потолка - так проверяется ассист;
  - рельеф: горки (наклонные прямоугольники), острова (верхний этаж),
    бугорки; наклон корпуса - по высоте под передней и задней осью,
    одометрия - по поверхности;
  - лидар: 360 лучей по 1°, 10 Гц, на высоте 130 мм над полом под роботом;
    лучи наклоняются вместе с корпусом; стены - прямоугольники толщиной
    20 мм с высотой от z0 до z1, пол и горки - по высоте рельефа; дальше -
    настоящие walls.measure() и corridor.corridor_points();
  - бампер: нос робота упёрся в стену.
Система мира: x - на восток, y - на север, мм; курс робота - градусы по
часовой от севера (как yaw в nav.py).
"""
from __future__ import annotations

import math
import os
import sys
import time as _time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))


def _stub(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


class _SerialException(Exception):
    pass


_stub("serial", Serial=object, SerialException=_SerialException)
_gpio = _stub("RPi.GPIO", BCM=11, IN=1, OUT=0, PUD_UP=22, LOW=0, HIGH=1,
              setmode=lambda *a, **k: None, setup=lambda *a, **k: None,
              input=lambda *a, **k: 1, cleanup=lambda *a, **k: None,
              setwarnings=lambda *a, **k: None)
_stub("RPi", GPIO=_gpio)
_stub("leds", NeoPixels=object, blink_pink=lambda *a, **k: None,
      blink_return=lambda *a, **k: None, RED=(255, 0, 0), SLEEP=(0, 0, 0),
      VICTIM_STOP_S=5.0, MARGIN_S=0.3)
_stub("victim_detect", VictimDetector=object, CONF_FAST=0.25, MIN_BOX_PX=40,
      YOLO_MODEL_PATH="", SAVE_DIR="")
_stub("victim_watch", VictimWatcher=object)

import corridor  # noqa: E402
import nav  # noqa: E402
import walls  # noqa: E402

_BLOCKING = [m for m in walls.MASKS if m is not walls.REAR_MASK]

CELL = 300.0
WALL_T = 20.0             # толщина стены
WALL_H = 150.0            # высота стены
LIDAR_Z = 130.0           # лидар над полом
AXLE = 75.0               # оси колёс от центра робота
NOSE = 96.5               # нос от центра (ROBOT_LEN_MM / 2)
HALF_W = 62.5
TURN_FF = 0.22            # см/с колеса на °/с корпуса (как TURN_FF_INIT)
ACCEL = 800.0             # см/с² - предел ускорения колеса
TAU = 0.04                # постоянная времени колеса
V_FLAT = 60.0             # режим скорости на ровном тянет до этого


class FakeTime:
    """nav.time: часы модели вместо настоящих."""

    def __init__(self, sim):
        self.sim = sim

    def monotonic(self):
        return self.sim.t

    def time(self):
        return self.sim.t

    def sleep(self, s):
        self.sim.advance(s)

    strftime = staticmethod(_time.strftime)
    localtime = staticmethod(_time.localtime)


class World:
    """Стены, рельеф. Клетка (i, j) нижнего этажа - центр (i*300, j*300)."""

    def __init__(self):
        self.walls = []       # (x0, y0, x1, y1, z0, z1) - прямоугольник в плане
        self.pieces = []      # (x0, y0, x1, y1, h(x, y)) - рельеф поверх пола

    def wall(self, cx, cy, d, z0=0.0, z1=WALL_H):
        """Стена клетки (cx, cy) с направления d (0=N, 1=E, 2=S, 3=W)."""
        x, y = cx * CELL, cy * CELL
        h, t = CELL / 2, WALL_T / 2
        if d == 0:
            r = (x - h - t, y + h - t, x + h + t, y + h + t)
        elif d == 2:
            r = (x - h - t, y - h - t, x + h + t, y - h + t)
        elif d == 1:
            r = (x + h - t, y - h - t, x + h + t, y + h + t)
        else:
            r = (x - h - t, y - h - t, x - h + t, y + h + t)
        self.walls.append((*r, z0, z1))

    def box(self, cells, z0=0.0, z1=WALL_H):
        """Обнести клетки стенами по контуру (внутренние рёбра открыты)."""
        s = set(cells)
        for cx, cy in s:
            for d, (dx, dy) in enumerate(((0, 1), (1, 0), (0, -1), (-1, 0))):
                if (cx + dx, cy + dy) not in s:
                    self.wall(cx, cy, d, z0, z1)

    def height(self, x, y):
        h = 0.0
        for x0, y0, x1, y1, f in self.pieces:
            if x0 <= x <= x1 and y0 <= y <= y1:
                h = max(h, f(x, y))
        return h


class Sim:
    TICK = 0.005

    def __init__(self, world, cell=(0, 0), heading=0.0, yaw_offset=37.0):
        self.w = world
        self.x, self.y = cell[0] * CELL, cell[1] * CELL
        self.theta = heading
        self.yaw_offset = yaw_offset
        self.t = 1000.0
        self.vl = self.vr = 0.0
        self.cmd = ("M", 0.0, 0.0, 0.0)
        self.lcm = self.rcm = 0.0
        self.z_cnt = 0
        self.next_scan = self.t
        self.bumped = False
        self.max_pitch = 0.0
        self.crashes = 0

    # ── связь с nav.py ──────────────────────────────────────
    def write(self, b):
        s = bytes(b).split(b"*")[0].decode()
        if s.startswith("M:"):
            l, r = (float(v) for v in s[2:].split(","))
            self.cmd = ("M", l, r, 0.0)
        elif s.startswith("T:"):
            l, r, cap = (float(v) for v in s[2:].split(","))
            self.cmd = ("T", l, r, cap)
        elif s == "B":
            self.cmd = ("M", 0.0, 0.0, 0.0)
        elif s == "Z":
            self.lcm = self.rcm = 0.0
            self.z_cnt = (self.z_cnt + 1) % 256

    def flush(self):
        pass

    def advance(self, dt):
        end = self.t + dt
        while self.t < end - 1e-9:
            self.step(self.TICK)

    # ── рельеф и поза ──────────────────────────────────────
    def fwd(self):
        r = math.radians(self.theta)
        return math.sin(r), math.cos(r)

    def axle_heights(self):
        fx, fy = self.fwd()
        hf = self.w.height(self.x + fx * AXLE, self.y + fy * AXLE)
        hr = self.w.height(self.x - fx * AXLE, self.y - fy * AXLE)
        return hf, hr

    def pitch(self):
        hf, hr = self.axle_heights()
        return math.degrees(math.atan2(hf - hr, 2 * AXLE))

    def bump_contact(self):
        """Колесо на кромке рельефа (бугорок) - в режиме скорости проседает."""
        fx, fy = self.fwd()
        for s in (AXLE, -AXLE):
            px, py = self.x + fx * s, self.y + fy * s
            a = self.w.height(px - fx * 8, py - fy * 8)
            b = self.w.height(px + fx * 8, py + fy * 8)
            if abs(b - a) > 5.0:
                return True
        return False

    # ── шаг модели ──────────────────────────────────────────
    def step(self, dt):
        p = self.pitch()
        self.max_pitch = max(self.max_pitch, abs(p))
        mode, a, b, cap = self.cmd
        if mode == "M":
            # режим скорости: на подъёме и на кромке бугорка мотор не тянет
            lim = V_FLAT - 1.8 * max(0.0, p)
            if self.bump_contact():
                lim = min(lim, 18.0)
            tl = max(-V_FLAT, min(lim, a))
            tr = max(-V_FLAT, min(lim, b))
        else:
            # момент: тянет до потолка, если тока хватает на уклон
            need = 0.06 * max(0.0, p)
            k = 1.0 if (a + b) / 2 >= need else 0.2
            tl = math.copysign(cap * k, a) if abs(a) > 0.05 else 0.0
            tr = math.copysign(cap * k, b) if abs(b) > 0.05 else 0.0
            if (a - b) != 0:      # руление моментом - разность скоростей
                d = (a - b) / max(abs(a) + abs(b), 1e-6) * cap * 0.3
                tl, tr = tl + d, tr - d
        for side in ("l", "r"):
            v = getattr(self, "v" + side)
            tgt = tl if side == "l" else tr
            dv = (tgt - v) * min(1.0, dt / TAU)
            dv = max(-ACCEL * dt, min(ACCEL * dt, dv))
            setattr(self, "v" + side, v + dv)
        v = (self.vl + self.vr) / 2.0            # см/с по поверхности
        omega = (self.vl - self.vr) / 2.0 / TURN_FF   # °/с, по часовой
        fx, fy = self.fwd()
        adv = v * 10.0 * dt * math.cos(math.radians(p))   # мм по горизонтали
        nx, ny = self.x + fx * adv, self.y + fy * adv
        if adv > 0 and self._nose_hits(nx, ny):
            self.bumped = True
            self.vl = self.vr = 0.0
            self.crashes += 1
            adv = 0.0
        else:
            self.bumped = False
            self.x, self.y = nx, ny
            self.lcm += self.vl * dt
            self.rcm += self.vr * dt
        self.theta = (self.theta + omega * dt) % 360.0
        self.t += dt
        self._telemetry(p, omega)
        if self.t >= self.next_scan:
            self.next_scan += 0.1
            self._scan(p)

    def _nose_hits(self, x, y):
        fx, fy = self.fwd()
        rx, ry = fy, -fx
        zf = self.w.height(x, y) + 40.0
        for s in (-HALF_W + 5, 0.0, HALF_W - 5):
            px, py = x + fx * NOSE + rx * s, y + fy * NOSE + ry * s
            for x0, y0, x1, y1, z0, z1 in self.w.walls:
                if x0 <= px <= x1 and y0 <= py <= y1 and z0 <= zf <= z1:
                    return True
        return False

    def _telemetry(self, p, omega):
        with nav._lock:
            st = nav._state
            st["yaw"] = (self.theta + self.yaw_offset) % 360.0
            st["yaw_rate"] = omega
            st["pitch"] = p
            st["roll"] = 0.0
            st["lcm"], st["rcm"] = self.lcm, self.rcm
            st["z_cnt"] = self.z_cnt
            st["clicker1"] = st["clicker2"] = self.bumped
            st["enc_left_online"] = st["enc_right_online"] = True
            st["telem_ok"] = True
        nav._telem_t = self.t
        nav._telem_seq += 1

    # ── лидар ──────────────────────────────────────────────
    def _ray_walls(self, ox, oy, dx, dy, z, te):
        best = 4000.0
        for x0, y0, x1, y1, z0, z1 in self.w.walls:
            # пересечение луча с прямоугольником (slab)
            tmin, tmax = 0.0, best
            ok = True
            for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
                if abs(d) < 1e-9:
                    if o < lo or o > hi:
                        ok = False
                        break
                else:
                    t1, t2 = (lo - o) / d, (hi - o) / d
                    if t1 > t2:
                        t1, t2 = t2, t1
                    tmin, tmax = max(tmin, t1), min(tmax, t2)
                    if tmin > tmax:
                        ok = False
                        break
            if ok and tmin > 1.0:
                zz = z + tmin * te
                if z0 <= zz <= z1:
                    best = tmin
        return best

    def _ray_terrain(self, ox, oy, dx, dy, z, te, limit):
        """Где луч уходит в пол/рельеф (шаг 10 мм, не дальше limit)."""
        if te < -1e-6:
            s_floor = z / -te       # ровный пол на 0
            limit = min(limit, s_floor)
        s = 20.0
        while s < limit:
            if z + s * te <= self.w.height(ox + dx * s, oy + dy * s):
                return s
            s += 10.0
        return limit

    def _scan(self, p):
        hf, hr = self.axle_heights()
        z = (hf + hr) / 2.0 + LIDAR_Z
        rev = []
        for a in range(360):
            # Стойки, ручки и леска закрывают лучи физически; задняя зона -
            # нет: walls.measure() снимает её маску для задней стены.
            if walls.is_masked(a, _BLOCKING):
                continue
            phi = math.radians(self.theta + a)
            dx, dy = math.sin(phi), math.cos(phi)
            te = math.tan(math.radians(p * math.cos(math.radians(a))))
            s = self._ray_walls(self.x, self.y, dx, dy, z, te)
            s = self._ray_terrain(self.x, self.y, dx, dy, z, te, min(s, 1500.0) if s < 4000 else 1500.0)
            if s < 3000.0:
                rev.append((float(a), s / math.cos(math.atan(te)), 200))
        filt = walls.filtered(rev)
        w = walls.measure(rev, filt)
        with nav._lock:
            nav._state["wall"] = {k: w[k] for k in ("front", "right", "left", "back")}
            nav._state["wall_still"] = {"front": w["front"], "right": w["right_still"],
                                       "left": w["left_still"], "back": w["back"]}
            nav._state["wall_n"] = {"front": w["front_n"], "right": w["right_still_n"],
                                   "left": w["left_still_n"], "back": w["back_n"]}
        nav._state["scan_t"] = self.t
        nav._scan_cnt += 1
        # Оборот модели снят мгновенно (с одной позы) - длительность 0, чтобы
        # corridor.point_time не размазывал точки по 100 мс, как у LD19.
        nav._scan_pts.append((nav._scan_cnt, self.t, 1e-6, corridor.corridor_points(filt[0])))


def setup(sim):
    """nav.py как в main(), но без железа: связь, часы, отсчёт курса."""
    nav.time = FakeTime(sim)
    nav.wait_telem = lambda last_seq, timeout=nav.TELEM_WAIT_S: (sim.advance(sim.TICK),
                                                                  nav._telem_seq)[1]
    nav._link = sim
    nav._write_state = lambda *a, **k: None
    nav.USE_COLOR_SENSOR = False
    nav._esp_running.set()
    sim.advance(0.3)
    nav.YAW_ORIGIN = nav.get_yaw()
    nav.calibrate_white()
