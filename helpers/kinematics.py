# helpers/kinematics.py
from __future__ import annotations

import bisect
import math
from collections import defaultdict
from typing import List, Dict, Optional, Tuple, TYPE_CHECKING

from carla_data_classes.dynamic import DataActor, TickData
from carla_data_classes.static import DataVector3D

if TYPE_CHECKING:
    from helpers.collisions import RecorderIndex


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


# -------------------- velocity from the recorder log --------------------

def _wrap_deg(delta: float) -> float:
    """Normalize an angle delta (degrees) into (-180, 180] so a crossing like 179 -> -179
    is treated as a 2-degree turn instead of a 358-degree spike."""
    return ((delta + 180.0) % 360.0) - 180.0


def compute_recorded_velocities(rec_idx: "RecorderIndex") -> Dict[int, Dict[int, Tuple[DataVector3D, DataVector3D]]]:
    """
    Build per-recorder-id, per-frame (linear_velocity, angular_velocity).

    Prefers CARLA's own ground-truth "Dynamic actors" linear_velocity/angular_velocity data
    (parsed into RecorderIndex.velocities_by_frame). That block is only present when the
    recording was made with additional_data=True - i.e. client.start_recorder(name, True),
    which every recorder call in this project already uses - and is the real physics
    velocity, not something derived after the fact.

    Falls back to finite-differencing consecutive recorded Location/Rotation entries
    (RecorderIndex.positions_by_frame) for any actor/frame the ground truth isn't available
    for (e.g. a recording made without additional_data). This is also what sidesteps a
    CARLA replay limitation: a replayed actor's get_velocity()/get_angular_velocity() stay
    at 0 because the replayer moves actors by teleporting them to recorded transforms each
    frame rather than driving them through the physics engine, so that live query is no
    substitute for either of the above during replay.

    Both paths are converted into the same convention DataLocation/DataRotation already
    export (Y negated, yaw negated), so the result lines up with a DataActor's own
    location/rotation. Angular velocity axes are (x=pitch-rate, y=roll-rate, z=yaw-rate) -
    verified against a real "additional_data=True" recorder dump by cross-referencing its
    linear_velocity/angular_velocity against finite-differenced Location/Rotation deltas
    frame-by-frame. This is NOT the same order as CARLA's Rotation struct (pitch, yaw, roll).

    Returns {recorder_id: {frame_index: (linear_velocity_mps, angular_velocity_degps)}}.
    """
    out: Dict[int, Dict[int, Tuple[DataVector3D, DataVector3D]]] = defaultdict(dict)
    last_frame: Dict[int, int] = {}
    last_pos: Dict[int, Tuple[float, float, float]] = {}
    last_rot: Dict[int, Tuple[float, float, float]] = {}

    def _time_of(frame_idx: int) -> float:
        if 1 <= frame_idx <= len(rec_idx.frame_times):
            return rec_idx.frame_times[frame_idx - 1]
        return float(frame_idx)

    zero = (DataVector3D(x=0.0, y=0.0, z=0.0), DataVector3D(x=0.0, y=0.0, z=0.0))

    all_frames = sorted(set(rec_idx.positions_by_frame.keys()) | set(rec_idx.velocities_by_frame.keys()))

    for frame_idx in all_frames:
        frame_positions = rec_idx.positions_by_frame.get(frame_idx, {})
        frame_velocities = rec_idx.velocities_by_frame.get(frame_idx, {})
        actor_ids = set(frame_positions.keys()) | set(frame_velocities.keys())

        for rec_id in actor_ids:
            ground_truth = frame_velocities.get(rec_id)
            pos_entry = frame_positions.get(rec_id)

            if ground_truth is not None:
                lv, av = ground_truth
                out[rec_id][frame_idx] = (
                    DataVector3D(x=lv.x, y=-lv.y, z=lv.z),
                    DataVector3D(x=av.x, y=av.y, z=-av.z),
                )
            elif pos_entry is not None and rec_id in last_frame:
                loc, rot = pos_entry
                px, py, pz = loc.x / 100.0, -loc.y / 100.0, loc.z / 100.0
                pitch, yaw, roll = rot.pitch, -rot.yaw, rot.roll
                dt = _time_of(frame_idx) - _time_of(last_frame[rec_id])
                if dt > 0:
                    lpx, lpy, lpz = last_pos[rec_id]
                    lpitch, lyaw, lroll = last_rot[rec_id]
                    out[rec_id][frame_idx] = (
                        DataVector3D(x=(px - lpx) / dt, y=(py - lpy) / dt, z=(pz - lpz) / dt),
                        DataVector3D(
                            x=_wrap_deg(pitch - lpitch) / dt,
                            y=_wrap_deg(roll - lroll) / dt,
                            z=_wrap_deg(yaw - lyaw) / dt,
                        ),
                    )
                else:
                    out[rec_id][frame_idx] = zero
            else:
                out[rec_id][frame_idx] = zero

            if pos_entry is not None:
                loc, rot = pos_entry
                last_frame[rec_id] = frame_idx
                last_pos[rec_id] = (loc.x / 100.0, -loc.y / 100.0, loc.z / 100.0)
                last_rot[rec_id] = (rot.pitch, -rot.yaw, rot.roll)

    return out


def velocity_at_time(
        vel_idx: Dict[int, Dict[int, Tuple[DataVector3D, DataVector3D]]],
        rec_idx: "RecorderIndex",
        rec_id: Optional[int],
        sim_time_rel: float,
) -> Tuple[Optional[DataVector3D], Optional[DataVector3D]]:
    """
    Look up the (linear_velocity, angular_velocity) computed by compute_recorded_velocities
    for rec_id, at the recorder frame closest to sim_time_rel.

    Returns (None, None) - rather than zero vectors - when rec_id is None/unmapped or has
    no recorded frames, so callers like DataVehicle.from_vehicle/DataPedestrian.from_walker
    fall back to their own live-query default instead of being forced to zero.
    """
    not_found = (None, None)
    if rec_id is None:
        return not_found

    per_frame = vel_idx.get(rec_id)
    ft = rec_idx.frame_times
    if not per_frame or not ft:
        return not_found

    i = bisect.bisect_left(ft, sim_time_rel)
    candidates = [c for c in (i - 1, i) if 0 <= c < len(ft)]
    if not candidates:
        return not_found
    best_k = min(candidates, key=lambda c: abs(ft[c] - sim_time_rel))

    return per_frame.get(best_k + 1, not_found)  # frames are 1-based


# -------------------- acceleration fill (post-processing) --------------------

def compute_acceleration_for_ticks(ticks: List[TickData]) -> None:
    """
    Fill per-actor acceleration (m/s^2) by finite-differencing velocity (already set at
    construction time from compute_recorded_velocities/velocity_at_time) across
    consecutive ticks. Modifies TickData in-place.
    """
    last_time: Dict[int, float] = {}
    last_vel: Dict[int, DataVector3D] = {}

    for t_idx, tick in enumerate(ticks):
        cur_time = float(getattr(tick, "current_tick", t_idx))  # fallback: index
        positions = getattr(tick, "actor_positions", []) or []

        for ap in positions:
            actor: DataActor = ap.actor
            vel = getattr(actor, "velocity", None)
            if actor is None or vel is None:
                continue

            aid = int(actor.id)

            if aid in last_time:
                dt = cur_time - last_time[aid]
                if dt > 0:
                    ax = (vel.x - last_vel[aid].x) / dt
                    ay = (vel.y - last_vel[aid].y) / dt
                    az = (vel.z - last_vel[aid].z) / dt
                    actor.acceleration = _vec(ax, ay, az)
                else:
                    # Zero/negative dt (shouldn't happen with fixed_delta) -> leave as-is
                    actor.acceleration = getattr(actor, "acceleration", _vec(0.0, 0.0, 0.0))
            else:
                # first time we see this actor -> no prior velocity to diff against
                actor.acceleration = _vec(0.0, 0.0, 0.0)

            last_time[aid] = cur_time
            last_vel[aid] = vel
