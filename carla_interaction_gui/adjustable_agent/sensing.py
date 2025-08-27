from __future__ import annotations

import math
from typing import Optional, Tuple

import carla

from .types import SensedState


class Sensing:
    def __init__(self, agent_ctx):
        self.ctx = agent_ctx  # RuleBasedAgent
        self.ego = agent_ctx.ego
        self.map = agent_ctx.map
        self.cfg = agent_ctx.cfg
        self.world = agent_ctx.world

    def sense(self) -> SensedState:
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        ego_wp = self.map.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

        speed_vec = self.ego.get_velocity()
        speed_mps = (speed_vec.x ** 2 + speed_vec.y ** 2 + speed_vec.z ** 2) ** 0.5

        # Waypoint speed limit is km/h in CARLA → convert to m/s
        speed_limit_kmh = getattr(ego_wp, "speed_limit", 30.0)
        speed_limit_mps = speed_limit_kmh / 3.6

        # Next options (branch fan-out ahead). Small lookahead to catch splits.
        next_options = ego_wp.next(2.0)

        # Traffic light sensing
        tl_state = None
        at_tl = False
        if self.ego.is_at_traffic_light():
            at_tl = True
            tl = self.ego.get_traffic_light()
            if tl is not None:
                tl_state = tl.get_state()

        # Stop/Yield landmarks ahead (from map landmarks)
        stop_or_yield = self._detect_stop_or_yield_ahead(ego_wp, self.cfg.landmark_lookahead)

        # Sun altitude for light decision
        weather = self.world.get_weather()
        sun_alt = float(getattr(weather, "sun_altitude_angle", 15.0))

        # Lead vehicle detection (same lane forward, within corridor)
        lead, dist, rel_v = self._detect_lead_vehicle(ego_tf, ego_wp)

        # Curvature (estimate using heading change over a short arc)
        curvature = self._estimate_curvature(ego_wp, arc_len=8.0)

        return SensedState(
            wp=ego_wp,
            speed_mps=speed_mps,
            speed_limit_mps=speed_limit_mps,
            in_junction=ego_wp.is_junction,
            next_options=next_options,
            traffic_light_state=tl_state,
            at_traffic_light=at_tl,
            stop_or_yield_ahead=stop_or_yield,
            sun_altitude_angle=sun_alt,
            lead_vehicle=lead,
            lead_distance=dist,
            lead_rel_speed=rel_v,
            curvature=curvature
        )

    def _detect_stop_or_yield_ahead(self, wp: carla.Waypoint, lookahead: float) -> Optional[Tuple[str, float]]:
        """
        Scan forward along the current lane for STOP / YIELD landmarks.
        Returns ("STOP"/"YIELD", distance_ahead_in_m) or None.
        """
        try:
            # Prefer map landmarks API
            lms = self.map.get_landmarks_from_waypoint(wp, lookahead)
            if not lms:
                return None

            # Only consider those on our lane side (same road, small lateral offset)
            # Note: LandmarkType.Stop / LandmarkType.Yield exist in CARLA API.
            for lm in lms:
                if lm.road_id != wp.road_id:
                    continue
                # Skip if behind
                lm_wp = self.map.get_waypoint(lm.transform.location, project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
                if lm_wp.road_id != wp.road_id:
                    continue

                dist = lm.transform.location.distance(wp.transform.location)
                if dist < 1.0:
                    continue

                if lm.type == carla.LandmarkType.Stop:
                    return ("STOP", dist)
                if lm.type == carla.LandmarkType.Yield:
                    return ("YIELD", dist)
        except Exception:
            # Fallback: best-effort (skip if API not available)
            pass
        return None

    def _detect_lead_vehicle(self, ego_tf: carla.Transform, ego_wp: carla.Waypoint
                             ) -> Tuple[Optional[carla.Actor], float, float]:
        """
        Simple forward sensor using world actors: pick a vehicle in front on same road/lane,
        within a corridor and compute distance and relative speed (ego - lead).
        """
        ego_loc = ego_tf.location
        ego_forward = ego_tf.get_forward_vector()
        vehicles = self.world.get_actors().filter("vehicle.*")
        best = None
        best_dist = math.inf
        best_rel_v = 0.0

        ego_vel = self.ego.get_velocity()
        ego_speed = math.sqrt(ego_vel.x ** 2 + ego_vel.y ** 2 + ego_vel.z ** 2)

        for v in vehicles:
            if v.id == self.ego.id:
                continue

            v_wp = self.map.get_waypoint(v.get_transform().location, project_to_road=True,
                                         lane_type=carla.LaneType.Driving)

            # Only consider same road & lane direction
            if v_wp.road_id != ego_wp.road_id or v_wp.lane_id != ego_wp.lane_id:
                continue

            rel = v.get_transform().location - ego_loc

            # Forward check via dot product
            fwd = ego_forward.x * rel.x + ego_forward.y * rel.y + ego_forward.z * rel.z
            if fwd <= 0.0:
                continue

            dist = math.sqrt(rel.x ** 2 + rel.y ** 2 + rel.z ** 2)

            # Lateral corridor (approx lane width)
            lateral = abs(self._signed_lateral_offset(ego_tf, v.get_transform().location))
            if lateral > max(2.5, ego_wp.lane_width * 0.6):
                continue

            if dist < best_dist:
                best = v
                best_dist = dist

                v_vel = v.get_velocity()
                v_speed = math.sqrt(v_vel.x ** 2 + v_vel.y ** 2 + v_vel.z ** 2)
                best_rel_v = ego_speed - v_speed

        return best, best_dist, best_rel_v

    def _estimate_curvature(self, wp: carla.Waypoint, arc_len: float = 8.0) -> float:
        """Estimate curvature kappa ≈ |Δψ| / arc_len using heading change over a short forward arc."""
        pts = wp.next_until_lane_end(arc_len) if hasattr(wp, "next_until_lane_end") else wp.next(arc_len)
        if len(pts) < 2:
            return 0.0
        psi0 = math.radians(pts[0].transform.rotation.yaw)
        psin = math.radians(pts[-1].transform.rotation.yaw)
        dpsi = math.atan2(math.sin(psin - psi0), math.cos(psin - psi0))
        return abs(dpsi) / max(arc_len, 1e-3)

    def _signed_lateral_offset(self, tf: carla.Transform, point: carla.Location) -> float:
        """Signed lateral offset of 'point' from the ego longitudinal axis (left positive)."""
        dx = point.x - tf.location.x
        dy = point.y - tf.location.y
        yaw = math.radians(tf.rotation.yaw)
        nx = -math.sin(yaw)  # left normal
        ny = math.cos(yaw)
        return dx * nx + dy * ny
