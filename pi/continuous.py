"""Hardware-free continuous-motion policies shared with the simulator (cm, s)."""
import math


def stopping_speed(distance_cm, latency_s, decel_cms2, margin_cm=1.0):
    """Largest speed satisfying v*latency + v²/(2*a) + margin <= distance."""
    d = max(0.0, distance_cm - margin_cm)
    a, lag = max(1.0, decel_cms2), max(0.0, latency_s)
    return max(0.0, math.sqrt((a * lag) ** 2 + 2 * a * d) - a * lag)


def observation_speed(gap_s, visible_cm=8.0, samples=2):
    """Sampling opportunity, not a guarantee that a neural net recognizes a mark."""
    return visible_cm / (max(0.05, gap_s) * samples)


def timed_rear(points, end_t, period):
    """Use only the late half of the rear sector; never mix both seam ends."""
    values = []
    for angle, distance in points.items():
        if 165 <= angle < 180:
            a = math.radians(angle)
            if abs(distance * math.sin(a)) <= 90:
                values.append((-distance * math.cos(a), end_t-period*(180-angle)/360))
    values.sort()
    return values[len(values)//2] if len(values) >= 3 else (None, None)


def floor_stop_target(boundary_cm, planned_cells, tile, cell_cm=30.0):
    """Can shorten an existing stop, including a pending stop for a marker."""
    return boundary_cm - (planned_cells + 1 - tile) * cell_cm


class PassageLedger:
    """A decision to continue is not a physical arrival at that tile's center."""
    def __init__(self):
        self.pending = []

    def schedule(self, cell, center_cm, walls):
        self.pending.append((cell, center_cm, dict(walls)))

    def advance(self, position_cm):
        reached = [p for p in self.pending if position_cm >= p[1]]
        self.pending = [p for p in self.pending if position_cm < p[1]]
        return reached

    def cancel_from(self, cell):
        self.pending = [p for p in self.pending if p[0] < cell]


class RangeFusion:
    """Bounded longitudinal correction from two agreeing, stable end walls.

    Missing walls are unobservable. Disagreement/turns reset the reference.
    Large coherent errors request recovery instead of snapping to another tile.
    The caller supplies the historical corrected pose at the scan timestamp.
    """
    def __init__(self):
        self.anchor = None
        self.previous = None
        self.last_seq = None
        self.evidence = 'unobservable'
        self.confirmations = 0

    def update(self, seq, now, scan_t, pose_cm, front_mm, back_mm, yaw, pitch):
        values = (pose_cm, front_mm, back_mm, yaw, pitch)
        if (scan_t is None or not 0 <= now-scan_t < .25 or
                any(v is None or not math.isfinite(v) for v in values) or
                abs(pitch) > 3 or not 130 < front_mm < 6000 or not 130 < back_mm < 6000):
            self.anchor = self.previous = None
            self.confirmations = 0
            self.evidence = 'unobservable'
            return 0.0, False
        if seq == self.last_seq:
            return 0.0, False
        self.last_seq = seq
        if self.anchor is None:
            self.anchor = (pose_cm, front_mm, back_mm, yaw)
            self.previous = (front_mm, back_mm, scan_t)
            self.confirmations = 0
            self.evidence = 'observing'
            return 0.0, False
        p0, f0, b0, y0 = self.anchor
        pf, pb, pt = self.previous
        self.previous = (front_mm, back_mm, scan_t)
        front_step, back_step = f0-front_mm, back_mm-b0
        if (abs((yaw+180)%360-180) > 12 or abs((yaw-y0+180)%360-180) > 12 or scan_t-pt > .3 or
                abs(front_step-back_step) > 20 or
                max(abs(pf-front_mm), abs(pb-back_mm)) > 180):
            self.anchor = self.previous = None
            self.confirmations = 0
            self.evidence = 'scene_changed'
            return 0.0, False
        self.confirmations += 1
        error_cm = p0 + (front_step+back_step)/20 - pose_cm
        if self.confirmations < 2:
            return 0.0, False
        if abs(error_cm) > 6:
            self.evidence = 'drift'
            return 0.0, True
        self.evidence = 'correcting' if abs(error_cm) > .15 else 'consistent'
        return max(-.4, min(.4, error_cm)) if abs(error_cm) > .15 else 0.0, False
