"""Conservative longitudinal slip evidence; missing ranges mean unobservable.

Side walls alone cannot measure travel along a straight corridor. Never infer
stuck from their disappearance, particularly on a raised, level stair landing.
"""
import math


class SlipGuard:
    def __init__(self):
        self.anchor = None
        self.last_scan = None
        self.evidence = "unobservable"
        self.partial_hits = 0

    def update(self, scan_id, now, scan_t, odo_mm, yaw, pitch, walls, speed):
        ranges = {k: v for k in ("front", "back")
                  if (v := walls.get(k)) is not None and math.isfinite(v) and 140 < v < 2500}
        if (scan_t is None or not 0 <= now - scan_t < 0.25 or
                abs(pitch) > 3 or speed < 9 or not ranges):
            self.anchor = None
            self.partial_hits = 0
            self.evidence = "unobservable"
            return False
        if scan_id == self.last_scan:
            return False
        self.last_scan = scan_id
        if self.anchor is not None and now - self.anchor[0] > 0.65:
            self.anchor = None
        if self.anchor is None:
            self.partial_hits = 0
            self.anchor = (now, odo_mm, yaw, ranges)
            self.evidence = "observing"
            return False
        t0, o0, y0, r0 = self.anchor
        da = abs((yaw - y0 + 180) % 360 - 180)
        steps = [(r0[k] - v) if k == "front" else (v - r0[k])
                 for k, v in ranges.items() if k in r0]
        # Changed scene or a turn: no comparison between unrelated surfaces.
        if da > 5 or not steps or max(steps) - min(steps) > 20 or any(abs(d) > 120 for d in steps):
            self.anchor = None
            self.partial_hits = 0
            self.evidence = "scene_changed"
            return False
        dt, wheel = now - t0, odo_mm - o0
        actual = sum(steps) / len(steps)
        if dt >= 0.25 and wheel >= 45 and abs(actual) <= 12:
            self.evidence = "slip"
            return True
        # Partial slip needs two agreeing walls and two distinct confirmations.
        # A single obstacle face is insufficient evidence for this weaker signal.
        partial = len(steps) == 2 and dt >= 0.25 and wheel >= 80 and 0 <= actual < wheel * 0.55
        self.partial_hits = self.partial_hits + 1 if partial else 0
        if self.partial_hits >= 2:
            self.evidence = "partial_slip"
            return True
        if dt >= 0.45 or (abs(actual) > 20 and not partial and actual >= wheel * 0.55):
            self.anchor = (now, odo_mm, yaw, ranges)
            self.partial_hits = 0
        self.evidence = "observing"
        return False


class CenterFilter:
    """Require another independent scan before accepting a lateral jump."""
    def __init__(self):
        self.last_scan = None
        self.value = None
        self.mode = None
        self.pending = None

    def update(self, scan_id, value, mode):
        if value is None:
            self.value = self.pending = self.mode = None
            return 0.0
        if scan_id == self.last_scan and self.value is not None:
            return self.value
        self.last_scan = scan_id
        if self.value is None:
            self.value, self.mode = value, mode
        elif abs(value - self.value) > 25 or mode != self.mode:
            if self.pending is not None and self.pending[1] == mode and abs(value - self.pending[0]) <= 12:
                self.value, self.mode = value, mode
                self.pending = None
            else:
                self.pending = (value, mode)
        else:
            self.value, self.mode, self.pending = value, mode, None
        return self.value
