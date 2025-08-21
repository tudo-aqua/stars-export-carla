# agent_collision.py
import math
from typing import Optional, Tuple

import carla

from .agent_constants import constants as C
from .agent_parameters import AgentParameters


class CollisionResult:
    def __init__(self, hazard: bool, other_actor: Optional[carla.Actor], available_distance_margin: float):
        self.hazard = hazard
        self.other_actor = other_actor
        self.available_distance_margin = available_distance_margin


class CollisionLogic:
    """
    Ego-only proxy for TM's CollisionHandling.
    Lead detection is simplified (same-lane, ahead); the *math* is identical to TM. :contentReference[oaicite:12]{index=12}
    """

    def __init__(self, vehicle: carla.Vehicle, params: AgentParameters):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.params = params

    def _lead_vehicle_on_same_lane(self) -> Optional[Tuple[carla.Vehicle, float]]:
        ego = self.vehicle
        m = ego.get_world().get_map()
        ego_wp = m.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        if not ego_wp:
            return None
        fwd = ego.get_transform().get_forward_vector()
        best = None;
        best_d = float("inf")
        for a in self.world.get_actors().filter("vehicle.*"):
            if a.id == ego.id: continue
            wp = m.get_waypoint(a.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
            if not wp: continue
            if (wp.road_id, wp.lane_id) != (ego_wp.road_id, ego_wp.lane_id): continue
            rel = a.get_location() - ego.get_location()
            ahead = rel.x * fwd.x + rel.y * fwd.y + rel.z * fwd.z
            if ahead <= 0: continue
            d = math.sqrt(rel.x ** 2 + rel.y ** 2 + rel.z ** 2)
            if d < best_d:
                best_d = d;
                best = a
        return (best, best_d) if best is not None else None

    def detect(self) -> CollisionResult:
        lead = self._lead_vehicle_on_same_lane()
        if not lead:
            return CollisionResult(False, None, float("inf"))
        other, dist = lead
        return CollisionResult(True, other, dist)

    def collision_handling_speed(
            self,
            vehicle_velocity: carla.Vector3D,
            vehicle_heading: carla.Vector3D,
            max_target_velocity: float,
            result: CollisionResult,
    ) -> Tuple[bool, float]:
        # MotionPlanStage::CollisionHandling math (verbatim). :contentReference[oaicite:13]{index=13}
        veh_speed = math.sqrt(vehicle_velocity.x ** 2 + vehicle_velocity.y ** 2 + vehicle_velocity.z ** 2)
        dynamic_target_velocity = max_target_velocity
        collision_emergency_stop = False

        if result.hazard:
            other = result.other_actor
            other_vel = other.get_velocity()
            other_speed_along_heading = (other_vel.x * vehicle_heading.x +
                                         other_vel.y * vehicle_heading.y +
                                         other_vel.z * vehicle_heading.z)
            available_distance_margin = result.available_distance_margin
            vehicle_relative_speed = math.sqrt(
                (vehicle_velocity.x - other_vel.x) ** 2 +
                (vehicle_velocity.y - other_vel.y) ** 2 +
                (vehicle_velocity.z - other_vel.z) ** 2
            )

            if vehicle_relative_speed > C.MotionPlan.EPSILON_RELATIVE_SPEED:
                follow_lead_distance = C.MotionPlan.FOLLOW_LEAD_FACTOR * veh_speed + C.MotionPlan.MIN_FOLLOW_LEAD_DISTANCE
                if available_distance_margin > follow_lead_distance:
                    dynamic_target_velocity = other_speed_along_heading
                elif available_distance_margin > C.MotionPlan.CRITICAL_BRAKING_MARGIN:
                    dynamic_target_velocity = max(other_speed_along_heading, C.MotionPlan.RELATIVE_APPROACH_SPEED)
                else:
                    collision_emergency_stop = True

            if available_distance_margin < C.MotionPlan.CRITICAL_BRAKING_MARGIN:
                collision_emergency_stop = True

        # Gradual slowdown cap: don't decrease more than PERC_MAX_SLOWDOWN * speed per frame.
        max_gradual_velocity = C.MotionPlan.PERC_MAX_SLOWDOWN * veh_speed
        if dynamic_target_velocity < veh_speed - max_gradual_velocity:
            dynamic_target_velocity = veh_speed - max_gradual_velocity

        dynamic_target_velocity = min(max_target_velocity, dynamic_target_velocity)
        return collision_emergency_stop, dynamic_target_velocity
