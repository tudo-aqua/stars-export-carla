from __future__ import annotations

import math
from typing import Optional, List

import carla


def signed_lateral_offset(tf: carla.Transform, point: carla.Location) -> float:
    """Signed lateral offset of 'point' from the ego longitudinal axis (left positive)."""
    dx = point.x - tf.location.x
    dy = point.y - tf.location.y
    yaw = math.radians(tf.rotation.yaw)
    nx = -math.sin(yaw)  # left normal
    ny = math.cos(yaw)
    return dx * nx + dy * ny


def straightest_of(base_wp: carla.Waypoint, cands: List[carla.Waypoint]) -> Optional[carla.Waypoint]:
    if not cands:
        return None
    byaw = math.radians(base_wp.transform.rotation.yaw)

    def dyaw(c: carla.Waypoint) -> float:
        cy = math.radians(c.transform.rotation.yaw)
        return abs(math.atan2(math.sin(cy - byaw), math.cos(cy - byaw)))

    return min(cands, key=dyaw)


def target_lookahead(lookahead_min: float, lookahead_max: float, lookahead_speed_gain: float,
                     speed_mps: float) -> float:
    return max(lookahead_min, min(lookahead_max, lookahead_min + lookahead_speed_gain * speed_mps))
