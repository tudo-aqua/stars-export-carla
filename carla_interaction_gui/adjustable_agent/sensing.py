from __future__ import annotations

import math
from typing import Optional, Tuple, List, Set

import carla

from .types import SensedState


class Sensing:
    def __init__(self, agent_ctx):
        self.ctx = agent_ctx  # RuleBasedAgent
        self.ego = agent_ctx.ego
        self.map = agent_ctx.map
        self.cfg = agent_ctx.cfg
        self.world = agent_ctx.world

    # ----------------------------- public -----------------------------

    def sense(self) -> SensedState:
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        ego_wp = self.map.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

        speed_vec = self.ego.get_velocity()
        speed_mps = (speed_vec.x ** 2 + speed_vec.y ** 2 + speed_vec.z ** 2) ** 0.5

        speed_limit_kmh = getattr(ego_wp, "speed_limit", 30.0)
        speed_limit_mps = speed_limit_kmh / 3.6

        next_options = ego_wp.next(2.0)

        # Build lane keys on our current path to filter TLs/signs
        lane_keys_on_path = self._lane_keys_on_path_ahead(ego_wp, look_ahead=60.0, step=2.0)

        # -------- Traffic light sensing (on-path, tolerant near the line) --------
        tl_state: Optional[carla.TrafficLightState] = None
        at_tl: bool = False
        tl_distance: float = float("inf")

        bound_tl = self.ego.get_traffic_light() if self.ego.is_at_traffic_light() else None
        best_tl = None
        best_stop_wp = None
        best_dist = float("inf")

        if bound_tl is not None:
            ok, stop_wp, dist = self._tl_controls_path(bound_tl, lane_keys_on_path, ego_tf)
            if ok:
                best_tl, best_stop_wp, best_dist = bound_tl, stop_wp, dist

        if best_tl is None:
            for tl in self.world.get_actors().filter("traffic.traffic_light*"):
                ok, stop_wp, dist = self._tl_controls_path(tl, lane_keys_on_path, ego_tf)
                if ok and dist < best_dist:
                    best_tl, best_stop_wp, best_dist = tl, stop_wp, dist

        if best_tl is not None:
            tl_state = best_tl.get_state()
            tl_distance = best_dist
            at_tl = best_dist < 6.0  # near the line

        # -------- STOP/YIELD (on-path only) --------
        stop_or_yield = self._detect_stop_or_yield_on_path(ego_wp, lane_keys_on_path)

        weather = self.world.get_weather()
        sun_alt = float(getattr(weather, "sun_altitude_angle", 15.0))

        lead, dist, rel_v = self._detect_lead_vehicle(ego_tf, ego_wp)
        curvature = self._estimate_curvature(ego_wp, arc_len=8.0)

        return SensedState(
            wp=ego_wp,
            speed_mps=speed_mps,
            speed_limit_mps=speed_limit_mps,
            in_junction=ego_wp.is_junction,
            next_options=next_options,
            traffic_light_state=tl_state,
            at_traffic_light=at_tl,
            traffic_light_distance_m=tl_distance,  # <-- NEW
            stop_or_yield_ahead=stop_or_yield,
            sun_altitude_angle=sun_alt,
            lead_vehicle=lead,
            lead_distance=dist,
            lead_rel_speed=rel_v,
            curvature=curvature
        )

    # ----------------------------- TL detection (path-aware) -----------------------------

    def _flatten_stop_wps(self, tl: carla.TrafficLight) -> List[carla.Waypoint]:
        """
        Returns all stop-line waypoints for this TL (handles API variants).
        NOTE: use extend(), not append(), because the modern API returns a list of lists.
        """
        wps: List[carla.Waypoint] = []
        # Modern API: list[list[Waypoint]] (one list per lane)
        try:
            groups = tl.get_stop_waypoints()
            for g in groups:
                # g is a List[Waypoint]
                wps.extend(g)
            return wps
        except Exception:
            pass
        # Fallback: older API that returns raw points
        try:
            for loc in tl.get_stop_waypoints_points():
                wp = self.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                if wp is not None:
                    wps.append(wp)
        except Exception:
            pass
        return wps

    def _tl_controls_path(
            self,
            tl: carla.TrafficLight,
            lane_keys_on_path: Set[Tuple[int, int]],
            ego_tf: carla.Transform
    ) -> Tuple[bool, Optional[carla.Waypoint], float]:
        """
        True only if any stop-line waypoint of TL lies on the *ego path ahead/nearby*:
        - (road_id, lane_id) must be on our lane path,
        - waypoint must be ahead OR within a small 'near' window around the ego,
        - heading roughly aligned.
        Returns (ok, stop_wp, distance).
        """
        best_wp = None
        best_dist = float("inf")
        for wp in self._flatten_stop_wps(tl):
            # Must be a Driving lane and on our path (not the opposite side)
            if (wp.lane_type & carla.LaneType.Driving) != carla.LaneType.Driving:
                continue
            key = (wp.road_id, wp.lane_id)
            if key not in lane_keys_on_path:
                continue

            loc = wp.transform.location
            # Robust "ahead or near" check (instead of strict ahead only)
            if not self._ahead_or_near(ego_tf, loc):
                continue

            # Heading alignment (< ~70°) to avoid picking cross traffic
            if not self._heading_aligned(ego_tf, wp.transform.rotation.yaw, max_deg=70.0):
                continue

            d = loc.distance(ego_tf.location)
            if d < best_dist:
                best_dist = d
                best_wp = wp

        return (best_wp is not None), best_wp, (best_dist if best_wp is not None else float("inf"))

    # ----------------------------- STOP/YIELD on-path -----------------------------

    def _detect_stop_or_yield_on_path(
            self,
            ego_wp: carla.Waypoint,
            lane_keys_on_path: Set[Tuple[int, int]],
            probe_step: float = 2.0,
            max_ahead: float = 60.0
    ) -> Optional[Tuple[str, float]]:
        """
        Probe along the current lane (until lane end) and return the nearest STOP/YIELD
        landmark that lies on that lane ahead.
        """
        best_kind = None
        best_dist = float("inf")

        # Walk along this lane only (robust to nearby/behind signs)
        wps = self._path_ahead_on_lane(ego_wp, look_ahead=max_ahead, step=probe_step)
        for w in wps:
            # Landmarks very near this waypoint
            try:
                lms = self.map.get_landmarks_from_waypoint(w, 1.5)
            except Exception:
                lms = []
            for lm in lms:
                lm_wp = self.map.get_waypoint(lm.transform.location, project_to_road=True,
                                              lane_type=carla.LaneType.Driving)
                if lm_wp is None:
                    continue
                # Must sit on our lane path
                if (lm_wp.road_id, lm_wp.lane_id) not in lane_keys_on_path:
                    continue
                # Keep STOP/YIELD only
                lm_name = (lm.name or "").lower() if hasattr(lm, "name") else ""
                is_stop = (lm.type == carla.LandmarkType.Stop) or ("stop" in lm_name)
                is_yield = (lm.type == carla.LandmarkType.Yield) or ("yield" in lm_name) or ("give_way" in lm_name)
                if not (is_stop or is_yield):
                    continue
                d = lm.transform.location.distance(ego_wp.transform.location)
                if d < best_dist:
                    best_dist = d
                    best_kind = "STOP" if is_stop else "YIELD"

        if best_kind is not None:
            return (best_kind, best_dist)
        return None

    # ----------------------------- common helpers -----------------------------

    def _path_ahead_on_lane(self, start_wp: carla.Waypoint, look_ahead: float = 60.0, step: float = 2.0) -> List[
        carla.Waypoint]:
        """Waypoints along THIS lane until lane end or distance budget."""
        pts: List[carla.Waypoint] = []
        # Prefer CARLA helper if available
        try:
            pts = start_wp.next_until_lane_end(step)
        except Exception:
            # Fallback: manual forward stepping on the same lane id
            cur = start_wp
            dist = 0.0
            while dist < look_ahead:
                nxts = cur.next(step)
                if not nxts:
                    break
                cur = nxts[-1]
                pts.append(cur)
                dist += step
                if cur.road_id != start_wp.road_id or cur.lane_id != start_wp.lane_id:
                    break
        # Trim to lookahead distance
        out: List[carla.Waypoint] = []
        for w in pts:
            if w.transform.location.distance(start_wp.transform.location) <= (look_ahead + 0.5 * step):
                out.append(w)
            else:
                break
        return out

    def _lane_keys_on_path_ahead(self, start_wp: carla.Waypoint, look_ahead: float = 60.0, step: float = 2.0) -> Set[
        Tuple[int, int]]:
        """Collect (road_id, lane_id) keys for the current lane path ahead (until lane end)."""
        keys: Set[Tuple[int, int]] = set()
        for w in self._path_ahead_on_lane(start_wp, look_ahead=look_ahead, step=step):
            keys.add((w.road_id, w.lane_id))
        # Ensure we include the current position
        keys.add((start_wp.road_id, start_wp.lane_id))
        return keys

    def _ahead_or_near(
            self,
            ego_tf: carla.Transform,
            loc: carla.Location,
            near_back_m: float = 1.2,  # allow up to 1.2 m behind due to actuation/latency
            near_side_m: float = 1.6  # allow small lateral offset near the line
    ) -> bool:
        """
        True if the point is ahead of ego OR very close to the ego along the longitudinal axis,
        allowing a small tolerance behind the front bumper and small lateral error.
        This makes TL detection robust when we are essentially *at* the stop line.
        """
        rel = loc - ego_tf.location
        fwd = ego_tf.get_forward_vector()
        lon = fwd.x * rel.x + fwd.y * rel.y + fwd.z * rel.z  # signed longitudinal projection

        # Lateral projection using left-normal
        yaw = math.radians(ego_tf.rotation.yaw)
        nx, ny = -math.sin(yaw), math.cos(yaw)
        lat = rel.x * nx + rel.y * ny

        # Consider "valid" if strictly ahead, or within a small window behind & close laterally
        return (lon >= 0.0) or (abs(lon) <= near_back_m and abs(lat) <= near_side_m)

    def _heading_aligned(self, ego_tf: carla.Transform, other_yaw_deg: float, max_deg: float = 70.0) -> bool:
        ego_yaw = math.radians(ego_tf.rotation.yaw)
        other = math.radians(other_yaw_deg)
        d = math.atan2(math.sin(other - ego_yaw), math.cos(other - ego_yaw))
        return abs(math.degrees(d)) <= max_deg

    # ----------------------------- existing vehicle / curvature helpers -----------------------------

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
            lateral = self._signed_lateral_offset(ego_tf, v.get_transform().location)
            if abs(lateral) > max(2.5, ego_wp.lane_width * 0.6):
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
