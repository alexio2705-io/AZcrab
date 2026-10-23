"""Checked wheel/heading integration. Distances are cm; yaw is clockwise degrees."""
from dataclasses import dataclass
import math

# Conservative validity limits, not motion-control setpoints. Verify against
# captured telemetry before tightening them for a particular chassis.
MAX_SAMPLE_GAP_S = 0.3
MAX_WHEEL_SPEED_CMS = 150.0
HOST_JITTER_S = 0.025
WHEEL_JUMP_MARGIN_CM = 2.0
MAX_YAW_RATE_DPS = 720.0
YAW_JUMP_MARGIN_DEG = 12.0


class MotionDiscontinuity(RuntimeError):
    pass


@dataclass(frozen=True)
class MotionSample:
    seq: int
    t: float
    left: float
    right: float
    yaw: float
    zero: int
    ready: bool = True


class ProjectedOdometry:
    """Integrate along a fixed trip axis, including intermediate turns.

    This cannot measure tyre slip or sideways skid. Bounds detect invalid
    observations, not every physical source of drift.
    """
    def __init__(self, sample, target_yaw):
        if not sample.ready or not all(math.isfinite(v) for v in
                (sample.t, sample.left, sample.right, sample.yaw, target_yaw)):
            raise MotionDiscontinuity("invalid initial motion sample")
        self.last = sample
        self.target = target_yaw
        self.wheel_cm = 0.0

    def update(self, sample):
        old = self.last
        if sample.seq == old.seq:
            return 0.0
        dt = sample.t - old.t
        values = (sample.t, sample.left, sample.right, sample.yaw)
        if (not all(math.isfinite(v) for v in values) or not sample.ready
                or sample.seq != old.seq + 1 or not 0 < dt <= MAX_SAMPLE_GAP_S
                or sample.zero != old.zero):
            raise MotionDiscontinuity("missing/invalid motion sample or odometry reset")
        dl, dr = sample.left - old.left, sample.right - old.right
        # UART host timestamps can bunch up. Allow 25 ms jitter and 2 cm noise,
        # but reject jumps far below the old one-metre-per-tick threshold.
        bound = MAX_WHEEL_SPEED_CMS * max(dt, HOST_JITTER_S) + WHEEL_JUMP_MARGIN_CM
        angle = (sample.yaw - old.yaw + 180) % 360 - 180
        if max(abs(dl), abs(dr)) > bound or abs(angle) > MAX_YAW_RATE_DPS * max(dt, HOST_JITTER_S) + YAW_JUMP_MARGIN_DEG:
            raise MotionDiscontinuity("wheel/heading jump exceeds physical bound")
        ds = (dl + dr) * 0.5
        half = math.radians(angle) * 0.5
        arc = math.sin(half) / half if abs(half) > 1e-8 else 1.0
        heading = math.radians(old.yaw - self.target + angle * 0.5)
        self.last = sample
        self.wheel_cm += ds
        return ds * math.cos(heading) * arc


def rebase(odometry, sample):
    """Разрыв истории (пропуск пакетов, долгая пауза дольше буфера, сброс Z):
    робот не останавливается. Путь за разрыв - средним ходом колёс, как
    считали до проекции; скачок от "Z" или неправдоподобный ход - 0. Дальше
    интегрируем от этого пакета. -> (новая odometry, путь за разрыв в см)."""
    old = odometry.last
    if not sample.ready or not all(math.isfinite(v) for v in
                                   (sample.t, sample.left, sample.right, sample.yaw)):
        return odometry, 0.0        # энкодеры молчат - это odo_fault, не здесь
    dl, dr = sample.left - old.left, sample.right - old.right
    gap = (dl + dr) / 2.0 if sample.zero == old.zero and max(abs(dl), abs(dr)) < 100 else 0.0
    return ProjectedOdometry(sample, odometry.target), gap
