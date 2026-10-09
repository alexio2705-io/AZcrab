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
import random
import sys
import time as _time
import types
from dataclasses import dataclass

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
_leds = _stub("leds", NeoPixels=object, blink_pink=lambda *a, **k: None,
              blink_return=lambda *a, **k: None, RED=(255, 0, 0), SLEEP=(0, 0, 0),
              VICTIM_STOP_S=5.0, MARGIN_S=0.3)
_stub("victim_detect", VictimDetector=object, CONF_FAST=0.25, MIN_BOX_PX=40,
      YOLO_MODEL_PATH="", SAVE_DIR="")
_stub("victim_watch", VictimWatcher=object)

import corridor  # noqa: E402
import nav  # noqa: E402
import walls  # noqa: E402

_BLOCKING = [m for m in walls.MASKS if m is not walls.REAR_MASK]
_OPEN_ANGLES = [a for a in range(360) if not walls.is_masked(a, _BLOCKING)]
_COS = [math.cos(math.radians(a)) for a in range(360)]

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
BUCKET = 300.0            # шаг сетки индекса стен/рельефа
RAY_MAX = 1500.0          # дальше лучи модели не идут (см. _scan)


class SimTimeUp(BaseException):
    """Модельное время заезда кончилось (Sim.limit). BaseException, а не
    Exception: nav.py местами ловит Exception, а этот выход ловить нельзя."""


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


@dataclass
class Victim:
    """Метка на стене: клетка мира (i, j), стена d (0=N..3=W, со стороны этой
    клетки), тип ("damaged"/"stable"/"unharmed" или "fake" - ложная цель),
    сдвиг вдоль стены от её середины, мм (по x для N/S, по y для E/W)."""
    cell: tuple
    d: int
    type: str
    along: float = 0.0

    def point(self, w=None):
        """Центр метки на поверхности стены (x, y), мм."""
        i, j = self.cell
        dx, dy = ((0, 1), (1, 0), (0, -1), (-1, 0))[self.d]
        f = CELL / 2 - WALL_T / 2
        return (i * CELL + dx * f + (self.along if dy else 0.0),
                j * CELL + dy * f + (self.along if dx else 0.0))


@dataclass
class Noise:
    """Несовершенства железа. Всё по нулям - идеальный робот (сценарии)."""
    gyro_drift_dps: float = 0.0     # уход курса, °/с (BNO08x game RV: 0.5°/мин = 0.0083)
    gyro_scale: float = 0.0         # ошибка масштаба гироскопа: поворот на A даёт A*(1+s)
    gyro_sigma: float = 0.0         # шум курса, °
    lidar_sigma: float = 0.0        # шум дальности, мм
    lidar_drop: float = 0.0         # доля потерянных лучей
    wheel_l: float = 1.0            # настоящий диаметр колеса / номинальный
    wheel_r: float = 1.0
    color_sigma: float = 0.0        # шум каналов датчика цвета, доля
    vis_miss: float = 0.0           # камера не видит метку в кадре
    vis_wrong: float = 0.0          # видит, но путает тип
    vis_false: float = 0.0          # ложная метка на пустой стене (на кадр стоя)

    @staticmethod
    def level(k, rng=None):
        """Набор шумов уровня k (0 - идеально, 1 - как на поле, 2 - плохой
        день); с rng - каждый параметр слегка случайный."""
        r = rng or random.Random(0)
        j = lambda v: v * (0.5 + r.random()) if rng else v      # noqa: E731
        d = r.choice((-1, 1)) * j(0.006) * k
        # Гироскоп - по даташиту BNO08x (game rotation vector, без магнитометра):
        # уход курса 0.5°/мин, динамическая ошибка до 2.5° - здесь это ошибка
        # масштаба, которая копится на поворотах (0.4% - 1.4° на оборот).
        return Noise(gyro_drift_dps=r.choice((-1, 1)) * j(0.5 / 60.0) * k,
                     gyro_scale=r.choice((-1, 1)) * j(0.004) * k, gyro_sigma=0.15 * k,
                     lidar_sigma=4.0 * k, lidar_drop=0.03 * k,
                     wheel_l=1.0 + d, wheel_r=1.0 - d, color_sigma=0.03 * k,
                     vis_miss=0.05 * k, vis_wrong=0.02 * k, vis_false=0.005 * k)


# Датчик цвета: замеры этого робота (см. таблицу в nav.py) + зеркальная
# (её C намного выше белой, доли почти серые). r, g, b, c.
TILE_RGBC = {
    "white": (82, 120, 135, 335),
    "black": (7, 9, 9, 26),
    "red": (39, 19, 37, 92),
    "blue": (13, 27, 48, 87),
    "silver": (250, 290, 300, 860),
    "air": (68, 42, 27, 134),
}
COLOR_FWD_MM = 90.0       # датчик цвета - на переднем бампере по центру (нос на 96.5 мм)
# APDS9960 в прошивке: ATIME=0xF6 - 10 циклов по 2.78 мс = 27.8 мс
# интегрирования, ESP опрашивает 30 раз в секунду (COLOR_POLL_HZ). Показание
# - среднее за время интегрирования (на 55 см/с это 15 мм пути) и по пятну
# на полу (поле зрения в даташите не дано - пятно ~6 мм, допущение): на
# стыке плиток датчик видит смесь, как настоящий.
COLOR_PERIOD_S = 1.0 / 30.0
COLOR_SPOT_MM = 6.0


def tile_samples():
    """Образцы для nav._tile_samples - как их снял бы tile_calib.py."""
    return {k: [{"r": r, "g": g, "b": b, "c": c, "n": 30, "ts": "sim"}]
            for k, (r, g, b, c) in TILE_RGBC.items()}


def cell_of(x, y):
    return int(math.floor(x / CELL + 0.5)), int(math.floor(y / CELL + 0.5))


class World:
    """Стены, рельеф. Клетка (i, j) нижнего этажа - центр (i*300, j*300)."""

    def __init__(self):
        self.walls = []       # (x0, y0, x1, y1, z0, z1) - прямоугольник в плане
        self.pieces = []      # (x0, y0, x1, y1, h(x, y)) - рельеф поверх пола
        self._idx = None      # сетка BUCKET мм -> стены/рельеф, что её задевают
        self.tiles = {}       # (i, j) клетки мира -> "black"/"blue"/"silver"/"red"
        self.victims = []     # Victim - метки на стенах

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

    # Индекс по сетке: в большом лабиринте сотни стен, а лучу лидара нужны
    # только те, что в 1.5 м, колесу - рельеф под ним. Списки стен и рельефа
    # сценарии дописывают напрямую, поэтому индекс пересобирается сам, как
    # только их длина поменялась.
    def _index(self):
        key = (len(self.walls), len(self.pieces))
        if self._idx is not None and self._idx[0] == key:
            return self._idx
        walls, pieces = {}, {}
        for dst, items in ((walls, self.walls), (pieces, self.pieces)):
            for it in items:
                x0, y0, x1, y1 = it[:4]
                for i in range(int(math.floor(x0 / BUCKET)), int(math.floor(x1 / BUCKET)) + 1):
                    for j in range(int(math.floor(y0 / BUCKET)), int(math.floor(y1 / BUCKET)) + 1):
                        dst.setdefault((i, j), []).append(it)
        self._idx = (key, walls, pieces)
        return self._idx

    def height(self, x, y):
        h = 0.0
        for x0, y0, x1, y1, f in self._index()[2].get((int(math.floor(x / BUCKET)),
                                                        int(math.floor(y / BUCKET))), ()):
            if x0 <= x <= x1 and y0 <= y <= y1:
                h = max(h, f(x, y))
        return h

    def near(self, x, y, r):
        """(стены, рельеф), задевающие квадрат 2r вокруг точки."""
        _, wi, pi = self._index()
        i0, i1 = int(math.floor((x - r) / BUCKET)), int(math.floor((x + r) / BUCKET))
        j0, j1 = int(math.floor((y - r) / BUCKET)), int(math.floor((y + r) / BUCKET))
        ws, ps = {}, {}
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                for w in wi.get((i, j), ()):
                    ws[id(w)] = w
                for q in pi.get((i, j), ()):
                    ps[id(q)] = q
        return list(ws.values()), list(ps.values())


class Sim:
    TICK = 0.005

    def __init__(self, world, cell=(0, 0), heading=0.0, yaw_offset=37.0,
                 noise=None, seed=0, color=None):
        self.w = world
        self.noise = noise or Noise()
        self.rng = random.Random(seed)
        # датчик цвета - по умолчанию, если в мире есть цветные плитки
        self.color = bool(world.tiles) if color is None else color
        self.t0 = 1000.0
        self.events = []          # (t, вид, x, y, данные) - мигания, выдачи, плитки
        self.kits = {"L": 0, "R": 0}
        self._next_color = 0.0
        self._color_acc = [0.0, 0.0, 0.0, 0.0, 0]   # сумма r,g,b,c и число замеров
        self.gyro_yaw = heading       # курс, как его интегрирует гироскоп
        self._tile_now = None     # плитка под центром робота
        self._tile_since = None   # (плитка, клетка, t входа, t последнего движения)
        self.limit: float | None = None   # секунд модельного времени на заезд (SimTimeUp)
        self.hooks = []           # f(sim) после каждого шага (запись для пульта)
        self.cells_seen = set()   # клетки мира, где побывал центр робота
        self.motors_on = True     # False - питание моторов выключено: колёса стоят, энкодеры молчат
        self.black_depth = 0.0    # глубже всего, мм, заезжал центр робота на чёрную от её края
        self.x, self.y = cell[0] * CELL, cell[1] * CELL
        self.theta = heading
        self.yaw_offset = yaw_offset
        self.t = 1000.0
        self.vl = self.vr = 0.0
        self.cmd = ("M", 0.0, 0.0, 0.0)
        self.cmd_raw = "M"        # M/T/B как пришло от nav (B модель сводит к M 0,0)
        self.rec = None           # регистратор трассы (sim/trace.Recorder) или None
        self.lcm = self.rcm = 0.0
        self.z_cnt = 0
        self.next_scan = self.t
        self.bumped = False
        self.max_pitch = 0.0
        self.crashes = 0
        # Темп: None - как можно быстрее (сценарии, тесты); число - во столько
        # раз быстрее настоящего времени (1.0 - вживую, для пульта).
        self.realtime = None
        self._pace0 = None

    # ── связь с nav.py ──────────────────────────────────────
    def write(self, b):
        s = bytes(b).split(b"*")[0].decode()
        if s.startswith("M:"):
            l, r = (float(v) for v in s[2:].split(","))
            self.cmd = ("M", l, r, 0.0)
            self.cmd_raw = "M"
        elif s.startswith("T:"):
            l, r, cap = (float(v) for v in s[2:].split(","))
            self.cmd = ("T", l, r, cap)
            self.cmd_raw = "T"
        elif s == "B":
            self.cmd = ("M", 0.0, 0.0, 0.0)
            self.cmd_raw = "B"
        elif s == "Z":
            self.lcm = self.rcm = 0.0
            self.z_cnt = (self.z_cnt + 1) % 256
        elif s.startswith("K:"):
            side = s[2:3]
            self.kits[side] = self.kits.get(side, 0) + 1
            self.event("kit", side=side)

    def event(self, kind, **data):
        self.events.append((round(self.t - self.t0, 3), kind, round(self.x), round(self.y), data))

    def flush(self):
        pass

    def advance(self, dt):
        # Положительный сон - хотя бы один такт: иначе цикл вида
        # "while time() < end: sleep(end - time())" с остатком в доли
        # наносекунды не двигал бы часы никогда (так висела blue_stop).
        if dt <= 0:
            return
        end = self.t + dt
        self.step(self.TICK)
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
        if not self.motors_on:
            tl = tr = 0.0
        for side in ("l", "r"):
            v = getattr(self, "v" + side)
            tgt = tl if side == "l" else tr
            dv = (tgt - v) * min(1.0, dt / TAU)
            dv = max(-ACCEL * dt, min(ACCEL * dt, dv))
            setattr(self, "v" + side, v + dv)
        # настоящие скорости колёс по поверхности: энкодер считает обороты,
        # а колесо другого диаметра проходит за оборот другой путь
        vla, vra = self.vl * self.noise.wheel_l, self.vr * self.noise.wheel_r
        v = (vla + vra) / 2.0                    # см/с по поверхности
        omega = (vla - vra) / 2.0 / TURN_FF      # °/с, по часовой
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
        n = self.noise
        self.gyro_yaw += omega * dt * (1.0 + n.gyro_scale) + n.gyro_drift_dps * dt
        self.t += dt
        self._telemetry(p, omega)
        if self.color:
            self._color_sample(p)
            if self.t >= self._next_color:
                self._next_color = self.t + COLOR_PERIOD_S
                self._color_publish()
        self._track_tiles(v)
        self.cells_seen.add(cell_of(self.x, self.y))
        for f in self.hooks:
            f(self)
        if self.limit is not None and self.t - self.t0 > self.limit:
            raise SimTimeUp()
        if self.rec is not None:
            self.rec.on_step(self, p)
        if self.t >= self.next_scan:
            self.next_scan += 0.1
            self._scan(p)
        if self.realtime:
            self._pace()

    def _pace(self):
        now = _time.monotonic()
        if self._pace0 is None:
            self._pace0 = (now, self.t)
            return
        ahead = (self.t - self._pace0[1]) / self.realtime - (now - self._pace0[0])
        if ahead > 0.002:
            _time.sleep(ahead)
        elif ahead < -0.5:      # не успеваем (лидар считается долго) - не копим долг
            self._pace0 = (now, self.t)

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
            n = self.noise
            jit = self.rng.gauss(0.0, n.gyro_sigma) if n.gyro_sigma else 0.0
            st["yaw"] = (self.gyro_yaw + self.yaw_offset + jit) % 360.0
            st["yaw_rate"] = omega
            st["pitch"] = p
            st["roll"] = 0.0
            st["lcm"], st["rcm"] = self.lcm, self.rcm
            st["z_cnt"] = self.z_cnt
            st["clicker1"] = st["clicker2"] = self.bumped
            st["enc_left_online"] = st["enc_right_online"] = self.motors_on
            st["telem_ok"] = True
            st["gyro_fresh"] = st["color_fresh"] = True
        nav._telem_t = self.t
        nav._telem_seq += 1

    # ── пол: датчик цвета и что под роботом ─────────────────
    def tile_at(self, x, y):
        return self.w.tiles.get(cell_of(x, y), "white")

    def _color_sample(self, p):
        """Один такт интегрирования: пятно датчика (центр и 4 точки по краю)."""
        fx, fy = self.fwd()
        x, y = self.x + fx * COLOR_FWD_MM, self.y + fy * COLOR_FWD_MM
        a = self._color_acc
        if abs(p) > 12.0:                    # нос задран - датчик видит зал
            pts = ((x, y),) * 5
            vals = [TILE_RGBC["air"]] * 5
        else:
            sp = COLOR_SPOT_MM
            pts = ((x, y), (x + fx * sp, y + fy * sp), (x - fx * sp, y - fy * sp),
                   (x + fy * sp, y - fx * sp), (x - fy * sp, y + fx * sp))
            vals = [TILE_RGBC[self.tile_at(px, py)] for px, py in pts]
        for v in vals:
            for i in range(4):
                a[i] += v[i]
        a[4] += len(vals)

    def _color_publish(self):
        a = self._color_acc
        if not a[4]:
            return
        r, g, b, c = (a[i] / a[4] for i in range(4))
        self._color_acc = [0.0, 0.0, 0.0, 0.0, 0]
        k = self.noise.color_sigma
        if k:
            r, g, b, c = (v * (1.0 + self.rng.gauss(0.0, k)) for v in (r, g, b, c))
        r, g, b, c = (max(0, round(v)) for v in (r, g, b, c))
        with nav._lock:
            nav._state["rgbc"] = (r, g, b, c)
            nav._state["color_fresh"] = True

    def _track_tiles(self, v):
        """События пола для оценки: заехал центром на чёрную, сколько стоял
        на синей, проехал чекпоинт."""
        tile = self.tile_at(self.x, self.y)
        cell = cell_of(self.x, self.y)
        if tile == "black":
            dx, dy = abs(self.x - cell[0] * CELL), abs(self.y - cell[1] * CELL)
            self.black_depth = max(self.black_depth, CELL / 2 - max(dx, dy))
        cur = self._tile_since
        if cur is None or cur[1] != cell:
            if cur is not None and cur[0] == "blue":
                self.event("blue_left", cell=cur[1], still_s=round(cur[4], 2))
            if tile != "white":
                self.event("tile", tile=tile, cell=cell)
            self._tile_since = [tile, cell, self.t, self.t, 0.0]
            return
        if abs(v) > 0.5:
            cur[3] = self.t
        else:
            cur[4] = max(cur[4], self.t - cur[3])     # дольше всего стоял на месте

    def _ray_walls(self, ox, oy, dx, dy, z, te, walls):
        best = 4000.0
        for x0, y0, x1, y1, z0, z1 in walls:
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

    def _ray_terrain(self, ox, oy, dx, dy, z, te, limit, flat=False):
        """Где луч уходит в пол/рельеф (шаг 10 мм, не дальше limit).
        flat - рядом нет рельефа: только пол, ответ без шагов (тот же, что
        дал бы шаг 10 мм)."""
        if te < -1e-6:
            s_floor = z / -te       # ровный пол на 0
            limit = min(limit, s_floor)
        if flat:
            # шаги ниже пола не дойдут: s < limit <= s_floor, а над ровным
            # полом луч касается только на s_floor - ровно limit
            return limit
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
        hits = [] if self.rec is not None else None   # (x, y, z, стена?) концов лучей
        near_w, near_p = self.w.near(self.x, self.y, RAY_MAX + 50.0)
        flat = not near_p
        for a in _OPEN_ANGLES:
            # Стойки, ручки и леска закрывают лучи физически (_OPEN_ANGLES без
            # них); задняя зона - нет: walls.measure() снимает её маску сам.
            phi = math.radians(self.theta + a)
            dx, dy = math.sin(phi), math.cos(phi)
            te = math.tan(math.radians(p * _COS[a])) if p else 0.0
            sw = self._ray_walls(self.x, self.y, dx, dy, z, te, near_w)
            s = self._ray_terrain(self.x, self.y, dx, dy, z, te,
                                  min(sw, RAY_MAX) if sw < 4000 else RAY_MAX, flat)
            if self.noise.lidar_drop and self.rng.random() < self.noise.lidar_drop:
                continue
            if s < 3000.0:
                rng_mm = s / math.cos(math.atan(te))
                if self.noise.lidar_sigma:
                    rng_mm += self.rng.gauss(0.0, self.noise.lidar_sigma)
                rev.append((float(a), rng_mm, 200))
                if hits is not None:
                    hits.append((self.x + dx * s, self.y + dy * s, z + s * te, s == sw))
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
        pts = corridor.corridor_points(filt[0])
        nav._scan_pts.append((nav._scan_cnt, self.t, 1e-6, pts))
        if hits is not None:
            self.rec.on_scan(self, hits, pts)


CURRENT = None            # Sim текущего заезда - для подделок ленты (fake_hw)


def setup(sim):
    """nav.py как в main(), но без железа: связь, часы, отсчёт курса."""
    global CURRENT
    CURRENT = sim
    nav.time = FakeTime(sim)
    nav.wait_telem = lambda last_seq, timeout=nav.TELEM_WAIT_S: (sim.advance(sim.TICK),
                                                                  nav._telem_seq)[1]
    nav._link = sim
    nav._write_state = nav._note_state      # без файла, но поза для пульта
    nav.USE_COLOR_SENSOR = sim.color
    if sim.color:
        nav._tile_samples = tile_samples()      # как load_tile_calib()
    nav._esp_running.set()
    sim.advance(0.3)
    nav.YAW_ORIGIN = nav.get_yaw()
    nav.calibrate_white()
