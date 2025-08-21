# agent_motion_plan.py
import math
from typing import List

import carla

from .agent_constants import constants as C
from .agent_parameters import AgentParameters

# OpenDRIVE landmark types (TM checks strings). :contentReference[oaicite:3]{index=3}
_LM_TL = "1000001"
_LM_STOP = "206"
_LM_YIELD = "205"
_LM_SPEED = "274"


def _three_point_circle_radius(a: carla.Location, b: carla.Location, c: carla.Location) -> float:
    # MotionPlanStage::GetThreePointCircleRadius (algebra identical). :contentReference[oaicite:4]{index=4}
    x1, y1 = a.x, a.y;
    x2, y2 = b.x, b.y;
    x3, y3 = c.x, c.y
    x12, x13 = x1 - x2, x1 - x3;
    y12, y13 = y1 - y2, y1 - y3
    y31, y21 = y3 - y1, y2 - y1;
    x31, x21 = x3 - x1, x2 - x1
    sx13, sy13 = x1 * x1 - x3 * x3, y1 * y1 - y3 * y3
    sx21, sy21 = x2 * x2 - x1 * x1, y2 * y2 - y1 * y1
    f_denom = 2.0 * (y31 * x12 - y21 * x13)
    if abs(f_denom) < 1e-6: return float("inf")
    f = (sx13 * x12 + sy13 * x12 + sx21 * x13 + sy21 * x13) / f_denom
    g_denom = 2.0 * (x31 * y12 - x21 * y13)
    if abs(g_denom) < 1e-6: return float("inf")
    g = (sx13 * y12 + sy13 * y12 + sx21 * y13 + sy21 * y13) / g_denom
    c_ = -(x1 * x1 + y1 * y1) - 2.0 * g * x1 - 2.0 * f * y1
    h, k = -g, -f
    rad_sq = h * h + k * k - c_
    return math.sqrt(rad_sq) if rad_sq > 0.0 else float("inf")


def _turn_target_velocity(waypoint_buffer: List[carla.Waypoint], max_target_velocity: float) -> float:
    # MotionPlanStage::GetTurnTargetVelocity → sqrt(R * FRICTION * GRAVITY). :contentReference[oaicite:5]{index=5}
    if len(waypoint_buffer) < 3:
        return max_target_velocity
    first = waypoint_buffer[0].transform.location
    last = waypoint_buffer[-1].transform.location
    mid = waypoint_buffer[len(waypoint_buffer) // 2].transform.location
    radius = _three_point_circle_radius(first, mid, last)
    if math.isinf(radius):
        return max_target_velocity
    return math.sqrt(radius * C.MotionPlan.FRICTION * C.MotionPlan.GRAVITY)


def _landmark_target_velocity(front_wp: carla.Waypoint,
                              vehicle_location: carla.Location,
                              actor_id: int,
                              max_target_velocity: float,
                              params: AgentParameters) -> float:
    # MotionPlanStage::GetLandmarkTargetVelocity translated literally. :contentReference[oaicite:6]{index=6}
    max_distance = C.MotionPlan.LANDMARK_DETECTION_TIME * max_target_velocity
    landmark_target_velocity = float("inf")
    try:
        landmarks = front_wp.get_landmarks(max_distance, False) or []
    except Exception:
        landmarks = []
    for lm in landmarks:
        try:
            wpl = lm.get_waypoint().transform.location if hasattr(lm, "get_waypoint") else None
            if wpl is None:
                continue
            distance = wpl.distance(vehicle_location)
            if distance > max_distance:
                continue
            minimum_velocity = max_target_velocity
            lt = str(getattr(lm, "type", ""))
            if lt == _LM_TL:
                minimum_velocity = C.MotionPlan.TL_TARGET_VELOCITY
            elif lt == _LM_STOP:
                minimum_velocity = C.MotionPlan.STOP_TARGET_VELOCITY
            elif lt == _LM_YIELD:
                minimum_velocity = C.MotionPlan.YIELD_TARGET_VELOCITY
            elif lt == _LM_SPEED:
                raw = float(getattr(lm, "value", 0.0)) / 3.6
                # parameters.GetVehicleTargetVelocity(actor_id, value)
                value = getattr(params, "get_vehicle_target_velocity", lambda aid, v: v)(actor_id, raw)
                minimum_velocity = value if value < max_target_velocity else max_target_velocity
            else:
                continue
            v = max(((max_target_velocity - minimum_velocity) / max_distance) * distance + minimum_velocity,
                    minimum_velocity)
            landmark_target_velocity = min(landmark_target_velocity, v)
        except Exception:
            pass
    return landmark_target_velocity if landmark_target_velocity != float("inf") else max_target_velocity


def compute_control(
        world: carla.World,
        vehicle: carla.Vehicle,
        params: AgentParameters,
        localization,
        collision,
        tl_hazard: bool,
):
    # Pull kinematics
    vel = vehicle.get_velocity()
    speed = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
    heading = vehicle.get_transform().get_forward_vector()
    ego_id = vehicle.id

    # Base target from speed-limit (Parameters::GetVehicleTargetVelocity / 3.6) :contentReference[oaicite:7]{index=7}
    raw_limit_mps = max(1.0, vehicle.get_speed_limit() / 3.6)
    max_target_velocity = getattr(params, "get_vehicle_target_velocity", lambda aid, v: v)(ego_id, raw_limit_mps)

    # Reduce by landmarks and turn radius, then collision policy
    front_wp = localization.buffer[0]
    landmark_target = _landmark_target_velocity(front_wp, vehicle.get_location(), ego_id, max_target_velocity, params)
    turn_target = _turn_target_velocity(localization.buffer, max_target_velocity)
    max_target_velocity = min(max_target_velocity, landmark_target, turn_target)

    col_res = collision.detect()
    col_emergency, dyn_target_v = collision.collision_handling_speed(
        carla.Vector3D(vel.x, vel.y, vel.z),
        carla.Vector3D(heading.x, heading.y, heading.z),
        max_target_velocity,
        col_res,
    )

    # SafeAfterJunction: full TM needs TrackTraffic; we’ll gate entry like TM when info exists.
    safe_after_junction = True
    if localization.is_at_junction_entrance and not (
            localization.junction_end_point and localization.safe_point_after_junction
    ):
        safe_after_junction = False

    emergency_stop = tl_hazard or col_emergency or (not safe_after_junction)

    # Lateral target point: max(v * TARGET_WAYPOINT_TIME_HORIZON, MIN_TARGET_WAYPOINT_DISTANCE) :contentReference[oaicite:8]{index=8}
    target_point_distance = max(speed * C.WaypointSelection.TARGET_WAYPOINT_TIME_HORIZON,
                                C.WaypointSelection.MIN_TARGET_WAYPOINT_DISTANCE)
    target_wp, _ = localization.get_target_waypoint(target_point_distance)
    target_loc = target_wp.transform.location

    # Lane offset parameter
    offset = params.lane_offset
    if abs(offset) > 1e-6:
        right = target_wp.transform.get_right_vector()
        target_loc = carla.Location(target_loc.x + offset * right.x,
                                    target_loc.y + offset * right.y,
                                    target_loc.z)

    # Angular deviation: acos(dot)/PI with cross sign (TM logic). :contentReference[oaicite:9]{index=9}
    vloc = vehicle.get_location()
    to_target = carla.Vector3D(target_loc.x - vloc.x, target_loc.y - vloc.y, target_loc.z - vloc.z)
    n = (to_target.x ** 2 + to_target.y ** 2 + to_target.z ** 2) ** 0.5 + 1e-6
    to_unit = carla.Vector3D(to_target.x / n, to_target.y / n, to_target.z / n)
    dot = max(-1.0, min(1.0, heading.x * to_unit.x + heading.y * to_unit.y + heading.z * to_unit.z))
    ang = math.acos(dot) / C.MotionPlan.PI
    cross_z = heading.x * to_unit.y - heading.y * to_unit.x
    if cross_z < 0.0:
        ang *= -1.0

    # Velocity deviation (TM): (dyn_target - speed) / dyn_target  :contentReference[oaicite:10]{index=10}
    target_v = 0.0 if emergency_stop else max(0.0, dyn_target_v)
    vel_dev = 0.0 if target_v <= 0.1 else (target_v - speed) / target_v

    # Select PID gains (urban/highway) from constants::PID  :contentReference[oaicite:11]{index=11}
    if speed > C.SpeedThreshold.HIGHWAY_SPEED:
        long_params = C.PID.LONGITUDIAL_HIGHWAY_PARAM
        lat_params = C.PID.LATERAL_HIGHWAY_PARAM
    else:
        long_params = C.PID.LONGITUDIAL_PARAM
        lat_params = C.PID.LATERAL_PARAM

    return ang, vel_dev, emergency_stop, long_params, lat_params
