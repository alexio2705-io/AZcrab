"""Navigation pose for floor projection. x forward, y left, yaw left-positive."""
import math


def from_snapshot(snap, now):
    if not snap or now - snap.get("t", -1e9) > 0.3:
        return None
    imu, dr = snap.get("imu") or {}, snap.get("drive") or {}
    if imu.get("yaw") is None or imu.get("pitch") is None:
        return None
    hdg = dr.get("hdg")
    if hdg is None:
        hdg = (snap.get("pose") or {}).get("hdg", 0)
    delta = (imu["yaw"] - hdg * 90 + 180) % 360 - 180
    walls = snap.get("walls") or {}
    left, right = walls.get("left"), walls.get("right")
    oy = 0.0
    if abs(delta) < 20:
        c = math.cos(math.radians(delta))
        if left is not None and right is not None and abs((left + right) * c - 280) < 50:
            oy = (right - left) * c / 2
        elif left is not None and left < 210:
            oy = 140 - left * c
        elif right is not None and right < 210:
            oy = right * c - 140
    # Tile currently occupied, not the original tile of a multi-cell trip.
    ox = (dr.get("travel_mm", 0.0) + 150) % 300 - 150
    return (ox, oy, -math.radians(delta)), imu["pitch"]
