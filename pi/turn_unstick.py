"""Выход из застревания на повороте на месте: момент, раскачка, одна сторона, откат."""
from __future__ import annotations

import math
import time

import config
import driving
import esp_link
import state

# Поворот едет в режиме скорости (0xA2), и когда колесо упёрлось в профиль,
# ток там ограничен "Max Torque Current" из LK Motor Tool - просить скорость
# больше бесполезно. Поэтому каждая ступень толкает МОМЕНТОМ ("T:" с
# потолком скорости колеса, см. assist_one() в motor.c). Ступень кончается,
# как только корпус закрутился к цели - дальше моментом ведёт основной
# контур поворота, а у цели переключается на штатное торможение.
# На косом профиле 1.25 A не хватало даже после раскачки (run_2040).
# Отдельная команда для поворота; ESP усиливает задние моторы и ограничивает
# каждый мотор 3 A. Здесь задаётся ток IQ, а не ток от аккумулятора.
UNSTICK_TORQUE_A = 2.0
UNSTICK_CAP_CMS = 20.0      # потолок скорости колеса под моментом
FREED_RATE_DPS = 20.0       # корпус крутится к цели быстрее - освободился
FREED_ANGLE_DEG = 4.0       # всплеск скорости без изменения курса - ещё не выход
HOLD_RATE_BAND_DPS = 20.0   # плавно снять ток у заданной угловой скорости
SPIN_S = 0.5                # ступени 1 и 4: момент на месте
ROCK_TIMES = 3              # ступень 2: качков "от цели - к цели"
ROCK_AWAY_S = 0.12
ROCK_TOWARD_S = 0.30
PIVOT_S = 0.5               # ступень 3: на каждую сторону
BACK_MM = 40.0              # ступень 4: откат перед новой попыткой
RETURN_MIN_MM = 10.0        # меньше - не возвращаться, доберёт центровка поездки
PUSH_TICK_S = 0.05          # ожидание пакета телеметрии между командами


class TurnProgress:
    """Застревание по продвижению к цели за окно, а не по дрожанию gyro rate."""

    def __init__(self, window_s, min_rate_dps):
        self.window_s = window_s
        self.min_rate_dps = min_rate_dps
        self.reset()

    def reset(self):
        self.since = None
        self.remaining = 0.0

    def stalled(self, now, rem):
        if self.since is None:
            self.since, self.remaining = now, abs(rem)
            return False
        dt = now - self.since
        if dt < self.window_s:
            return False
        progress = self.remaining - abs(rem)
        self.since, self.remaining = now, abs(rem)
        return progress < self.min_rate_dps * dt


def _push(left_a, right_a, dur_s, sign):
    """Ток на стороны (в знаках поворота) до dur_s. True - корпус крутится
    в сторону sign и действительно изменил курс, а не только качнулся."""
    end = time.monotonic() + dur_s
    seq = state.telem_seq
    last_yaw = esp_link.get_yaw()
    progress = 0.0
    while time.monotonic() < end:
        if esp_link.interrupted():
            return False
        esp_link.torque(left_a * config.TURN_SIGN, right_a * config.TURN_SIGN,
                        UNSTICK_CAP_CMS)
        seq = esp_link.wait_telem(seq, PUSH_TICK_S)
        yaw = esp_link.get_yaw()
        progress += ((yaw - last_yaw + 180.0) % 360.0 - 180.0) * sign
        last_yaw = yaw
        if progress >= FREED_ANGLE_DEG and esp_link.get_yaw_rate() * sign > FREED_RATE_DPS:
            return True
    return False


def _spin(s):
    a = UNSTICK_TORQUE_A * s
    return _push(a, -a, SPIN_S, s)


def _rock(s):
    a = UNSTICK_TORQUE_A * s
    for _ in range(ROCK_TIMES):
        _push(-a, a, ROCK_AWAY_S, -s)      # качок от цели, результат не важен
        if _push(a, -a, ROCK_TOWARD_S, s):
            return True
    return False


def _pivot(s):
    # Одна сторона тянет, другая без тока: точка вращения смещается к ней,
    # и колёсам не надо лезть на профиль всем сразу.
    a = UNSTICK_TORQUE_A * s
    return _push(a, 0.0, PIVOT_S, s) or _push(0.0, -a, PIVOT_S, s)


class Unstick:
    """Ступени по нарастающей на один поворот: next_stage() при каждом
    застревании, return_after_turn() после поворота."""

    STAGES = (("момент", _spin), ("раскачка", _rock), ("одной стороной", _pivot),
              ("откат", None))

    def __init__(self):
        self.stage = 0          # сколько ступеней уже было
        self.backed = None      # (смещение мм, курс отката) - вернуть после поворота
        self.holding = False    # после срыва с места всё ещё нужен момент

    def exhausted(self):
        return self.stage >= len(self.STAGES)

    def next_stage(self, rem):
        """Следующая ступень. rem - остаток угла к цели (>0 - по часовой).
        True - корпус закрутился."""
        name, fn = self.STAGES[self.stage]
        self.stage += 1
        s = 1.0 if rem > 0 else -1.0
        print(f"    застрял на повороте (осталось {rem:+.1f}°): ступень {self.stage} - {name}, "
              f"команда {UNSTICK_TORQUE_A:.2f}A, потолок {UNSTICK_CAP_CMS:.0f}см/с")
        if fn is None:
            esp_link.brake()
            yaw = esp_link.get_yaw()
            self.backed = (driving.creep_mm(-BACK_MM, yaw), yaw)
            fn = _spin
        freed = fn(s)
        if freed and not self.holding:
            self.holding = True
            print(f"    поддержка поворота моментом {UNSTICK_TORQUE_A:.2f}A до торможения")
        print(f"    ступень {self.stage}: {'закрутился' if freed else 'не помогло'}, "
              f"yaw={esp_link.get_yaw():.1f}°, rate={esp_link.get_yaw_rate():+.1f}°/с")
        return freed

    def hold_turn(self, spd, w_ref, rate, rem, brake_deg):
        """Поддержать вращение в основном контуре. False - нужен режим скорости."""
        sign = 1.0 if rem > 0 else -1.0
        if (not self.holding or abs(rem) <= brake_deg or spd * sign <= 0
                or w_ref * sign <= 0 or rate * sign >= abs(w_ref)):
            return False
        k = min(1.0, (abs(w_ref) - rate * sign) / HOLD_RATE_BAND_DPS)
        a = UNSTICK_TORQUE_A * k * sign * config.TURN_SIGN
        cap = max(5.0, min(UNSTICK_CAP_CMS, abs(spd)))
        esp_link.torque(a, -a, cap)
        return True

    def return_after_turn(self):
        """Вернуть вдоль нового курса то, что откат сместил по нему; поперёк
        доберёт центровка поездки."""
        if self.backed is None or esp_link.interrupted():
            return
        moved_mm, yaw_then = self.backed
        self.backed = None
        yaw = esp_link.get_yaw()
        back_mm = -moved_mm * math.cos(math.radians(yaw - yaw_then))
        if abs(back_mm) < RETURN_MIN_MM:
            return
        print(f"    возврат после отката: {back_mm:+.0f}мм")
        driving.creep_mm(back_mm, yaw)
