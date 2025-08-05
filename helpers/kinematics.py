# helpers/kinematics.py
from __future__ import annotations

import math
from typing import List, Dict, Optional

from carla_data_classes.dynamic import DataActor, TickData
from carla_data_classes.static import DataVector3D


def vector_norm(v: Optional[DataVector3D]) -> float:
    if v is None:
        return 0.0
    return math.sqrt(float(v.x) ** 2 + float(v.y) ** 2 + float(v.z) ** 2)


def mps_to_kmh(mps: float) -> float:
    return mps * 3.6


def mps_to_mph(mps: float) -> float:
    return mps * 2.2369362921


def actor_speed_mps(actor: DataActor) -> float:
    return vector_norm(getattr(actor, "velocity", None))


def actor_speed_kmh(actor: DataActor) -> float:
    return mps_to_kmh(actor_speed_mps(actor))


def actor_speed_mph(actor: DataActor) -> float:
    return mps_to_mph(actor_speed_mps(actor))


# --- acceleration helpers ---

def mps2_to_kmhps(a: float) -> float:
    """Convert acceleration from m/s² to km/h per second."""
    return a * 3.6


def mps2_to_mphps(a: float) -> float:
    """Convert acceleration from m/s² to mph per second."""
    return a * 2.2369362921


def actor_accel_mps2(actor) -> float:
    """Acceleration magnitude in m/s²."""
    return vector_norm(getattr(actor, "acceleration", None))


def actor_accel_kmhps(actor) -> float:
    """Acceleration magnitude in km/h per second."""
    return mps2_to_kmhps(actor_accel_mps2(actor))


def actor_accel_mphps(actor) -> float:
    """Acceleration magnitude in mph per second."""
    return mps2_to_mphps(actor_accel_mps2(actor))


# Convenience for vector construction
def _vec(x: float, y: float, z: float) -> DataVector3D:
    return DataVector3D(x=x, y=y, z=z)


# -------------------- velocity & acceleration fill --------------------

def compute_vel_acc_for_ticks(ticks: List[TickData]) -> None:
    """
    Fill per-actor velocity (m/s) and acceleration (m/s^2) by finite differences
    from consecutive ticks. Modifies TickData in-place.

    Strategy:
      v_i   = (p_i - p_{i-1}) / dt_i  (set on tick i)
      a_i   = (v_i - v_{i-1}) / dt_i  (set on tick i; first available is i>=2)
    Missing actors between ticks are handled by using their last seen state.
    """
    # per-actor rolling state
    last_time: Dict[int, float] = {}
    last_pos: Dict[int, DataVector3D] = {}
    last_vel: Dict[int, DataVector3D] = {}

    for t_idx, tick in enumerate(ticks):
        cur_time = float(getattr(tick, "current_tick", t_idx))  # fallback: index
        # index actor positions by id in this tick
        positions = getattr(tick, "actor_positions", []) or []

        for ap in positions:
            actor: DataActor = ap.actor
            if actor is None or actor.location is None:
                continue

            # current position vector from DataLocation
            px = float(actor.location.x)
            py = float(actor.location.y)
            pz = float(getattr(actor.location, "z", 0.0))
            cur_pos = _vec(px, py, pz)

            aid = int(actor.id)

            if aid in last_time:
                dt = cur_time - last_time[aid]
                if dt > 0:
                    # velocity (m/s)
                    vx = (cur_pos.x - last_pos[aid].x) / dt
                    vy = (cur_pos.y - last_pos[aid].y) / dt
                    vz = (cur_pos.z - last_pos[aid].z) / dt
                    vel = _vec(vx, vy, vz)
                    actor.velocity = vel

                    # acceleration (m/s^2) if we already have a previous velocity
                    if aid in last_vel:
                        ax = (vel.x - last_vel[aid].x) / dt
                        ay = (vel.y - last_vel[aid].y) / dt
                        az = (vel.z - last_vel[aid].z) / dt
                        actor.acceleration = _vec(ax, ay, az)
                    else:
                        # first velocity → no acceleration yet
                        actor.acceleration = _vec(0.0, 0.0, 0.0)

                    # update rolling velocity
                    last_vel[aid] = vel
                else:
                    # Zero/negative dt (shouldn't happen with fixed_delta) → leave as-is
                    actor.velocity = getattr(actor, "velocity", _vec(0.0, 0.0, 0.0))
                    actor.acceleration = getattr(actor, "acceleration", _vec(0.0, 0.0, 0.0))
            else:
                # first time we see this actor
                actor.velocity = _vec(0.0, 0.0, 0.0)
                actor.acceleration = _vec(0.0, 0.0, 0.0)

            # update rolling position/time
            last_time[aid] = cur_time
            last_pos[aid] = cur_pos
