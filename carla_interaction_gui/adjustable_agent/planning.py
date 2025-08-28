from __future__ import annotations

import math
from typing import Tuple, Optional, List

import carla

from .types import SensedState, Plan


class Planning:
    def __init__(self, agent_ctx):
        self.ctx = agent_ctx  # RuleBasedAgent
        self.cfg = agent_ctx.cfg
        self.rng = agent_ctx.rng
        self._active_junction_id: Optional[int] = None
        self.ego = agent_ctx.ego

    def plan(self, s: SensedState) -> Plan:
        # --- base target speed: respect speed limit ---
        target_speed = s.speed_limit_mps

        # --- Traffic lights: decelerate to stop by the stop line, even from far away ---
        stop_now = False
        stop_distance = 0.0
        if s.traffic_light_state is not None:
            if s.traffic_light_state in (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow):
                # Distance to stop line from sensing (∞ if none)
                dist = s.traffic_light_distance_m
                if math.isfinite(dist):
                    stop_distance = max(0.0, dist - self.ctx.cfg.stop_buffer)
                    # Cap speed based on distance-to-stop (independent of current speed)
                    target_speed = min(target_speed,
                                       self._speed_cap_to_stop_in_distance(stop_distance, comfort_dec=3.0))
                    # Force the final stop very near the line
                    if dist <= (self.ctx.cfg.stop_buffer + 0.8):
                        stop_now = True
                        target_speed = 0.0
        # If we are extremely close and CARLA also flags "at a TL", keep the stop
        elif s.at_traffic_light:
            stop_now = True
            target_speed = 0.0

        # --- stop / yield signs: always stop ---
        if s.stop_or_yield_ahead:
            kind, dist = s.stop_or_yield_ahead
            # Plan a full stop at stop_buffer before the sign
            stop_distance = max(0.0, dist - self.cfg.stop_buffer)
            # Reduce target speed to ensure comfortable decel to rest before the sign
            target_speed = min(target_speed, self._speed_to_stop_in_distance(s.speed_mps, stop_distance))

        # --- lead vehicle: keep headway and avoid collisions ---
        if s.lead_vehicle is not None and math.isfinite(s.lead_distance):
            desired_gap = self.cfg.min_gap + self.cfg.time_headway * s.speed_mps
            if s.lead_distance < desired_gap:
                # too close: slow down
                target_speed = min(target_speed, max(0.0, s.speed_mps - 1.5))
            # Emergency: small TTC
            rel_speed = max(0.1, s.lead_rel_speed)  # avoid zero division
            ttc = s.lead_distance / rel_speed if rel_speed > 0 else float("inf")
            if ttc < self.cfg.emergency_ttc:
                target_speed = 0.0

        # --- junction path choice (uniform random among branches) ---
        target_wp = self._choose_target_waypoint(s)

        # --- turn signals (based on planned target waypoint heading change) ---
        blink_left, blink_right = self._decide_blinkers(s.wp, target_wp)

        # --- headlights ---
        headlights_on = s.sun_altitude_angle < self.cfg.sun_angle_headlights_deg

        return Plan(
            target_speed_mps=max(0.0, target_speed),
            target_wp=target_wp,
            blink_left=blink_left,
            blink_right=blink_right,
            headlights_on=headlights_on,
            stop_now=stop_now,
            stop_distance=stop_distance
        )

    def _speed_cap_to_stop_in_distance(self, dist: float, comfort_dec: float = 3.0) -> float:
        """Max allowable speed to come to a comfortable stop within dist."""
        if dist <= 0.0:
            return 0.0
        return math.sqrt(2.0 * comfort_dec * dist)

    def _choose_target_waypoint(self, s: SensedState) -> carla.Waypoint:
        """
        CARLA-only lane following with *persistent* lane paths:

        - Outside junctions: follow a persisted *straight* lane path built with next_until_lane_end().
          This keeps targeting waypoints on the same lane even if the actuator drifts and the ego briefly
          crosses onto an adjacent/opposing lane.
        - Before entering a junction: preselect one outgoing DRIVING lane at the boundary and build a
          persisted *junction branch* path (also via next_until_lane_end()).
        - Inside junctions: stick to the preselected branch path until the junction is exited.
        - After leaving a junction: clear the junction choice but keep/rebuild the straight path as needed.
        """
        # ---- persistent state (lazy init) ----
        if not hasattr(self, "_entered_active_junction"):
            self._entered_active_junction: bool = False
        if not hasattr(self, "_active_junction_id"):
            self._active_junction_id: Optional[int] = None
        if not hasattr(self, "_active_branch_lane_key"):
            self._active_branch_lane_key: Optional[Tuple[int, int]] = None  # (road_id, lane_id)
        if not hasattr(self, "_active_path"):
            self._active_path: Optional[List[carla.Waypoint]] = None
        if not hasattr(self, "_active_path_idx"):
            self._active_path_idx: int = 0
        if not hasattr(self, "_straight_path"):
            self._straight_path: Optional[List[carla.Waypoint]] = None
        if not hasattr(self, "_straight_lane_key"):
            self._straight_lane_key: Optional[Tuple[int, int]] = None
        if not hasattr(self, "_straight_idx"):
            self._straight_idx: int = 0

        wp: carla.Waypoint = s.wp
        ego_tf: carla.Transform = self.ego.get_transform()
        fwd: carla.Vector3D = ego_tf.get_forward_vector()

        def is_ahead_loc(loc: carla.Location) -> bool:
            rel = loc - ego_tf.location
            return (fwd.x * rel.x + fwd.y * rel.y + fwd.z * rel.z) > 0.0

        def driving_only(cands: List[carla.Waypoint]) -> List[carla.Waypoint]:
            return [c for c in cands if (c.lane_type & carla.LaneType.Driving) == carla.LaneType.Driving]

        def straightest_of(base_wp: carla.Waypoint, cands: List[carla.Waypoint]) -> Optional[carla.Waypoint]:
            if not cands:
                return None
            byaw = math.radians(base_wp.transform.rotation.yaw)

            def dyaw(c: carla.Waypoint) -> float:
                cy = math.radians(c.transform.rotation.yaw)
                return abs(math.atan2(math.sin(cy - byaw), math.cos(cy - byaw)))

            return min(cands, key=dyaw)

        def first_junction_and_last_non(start: carla.Waypoint, probe_step: float = 1.0, max_ahead: float = 25.0
                                        ) -> Tuple[Optional[carla.Waypoint], carla.Waypoint]:
            """Walk forward on the straightest continuation to find the first junction boundary."""
            traveled = 0.0
            cursor = start
            last_non = start
            first_junc = None
            while traveled < max_ahead:
                nxts = cursor.next(probe_step)
                if not nxts:
                    break
                nxt = straightest_of(cursor, nxts)
                if nxt is None:
                    break
                traveled += probe_step
                if nxt.is_junction:
                    first_junc = nxt
                    break
                last_non = nxt
                cursor = nxt
            return first_junc, last_non

        def lane_path_from(start: carla.Waypoint, step: float = 2.0) -> List[carla.Waypoint]:
            """Waypoints along this lane until lane end, spaced by ~step meters."""
            try:
                pts = start.next_until_lane_end(step)
                if pts:
                    return pts
            except Exception:
                pass
            # Fallback if API not present
            out: List[carla.Waypoint] = []
            cur = start
            for _ in range(80):
                nxts = cur.next(step)
                if not nxts:
                    break
                cur = nxts[-1]
                out.append(cur)
                # stop if lane identity changes unexpectedly
                if cur.road_id != start.road_id or cur.lane_id != start.lane_id:
                    break
            return out

        def pick_ahead_from_path(path: List[carla.Waypoint], min_forward: float = 1.5) -> Optional[carla.Waypoint]:
            """Pick the first waypoint on path that lies ahead of ego and ~min_forward meters away."""
            if not path:
                return None
            best_idx: Optional[int] = None
            best_d = float("inf")
            for i, w in enumerate(path):
                loc = w.transform.location
                if not is_ahead_loc(loc):
                    continue
                d = loc.distance(ego_tf.location)
                if d >= min_forward and d < best_d:
                    best_d = d
                    best_idx = i
            if best_idx is None:
                # fallback to nearest ahead or last
                for i, w in enumerate(path):
                    if is_ahead_loc(w.transform.location):
                        best_idx = i
                        break
            if best_idx is None:
                best_idx = len(path) - 1
            self._active_path_idx = best_idx
            return path[best_idx]

        # ---------------- NOT in a junction: manage straight path and preselect upcoming junction branch ----------------
        if not wp.is_junction:
            # If we just exited a junction, clear junction state (keep/rebuild straight path separately)
            if self._entered_active_junction:
                self._active_junction_id = None
                self._active_branch_lane_key = None
                self._active_path = None
                self._active_path_idx = 0
                self._entered_active_junction = False

            # Persist/maintain a STRAIGHT path for the current lane
            if self._straight_path is None:
                # Initialize with the *current* lane
                self._straight_lane_key = (wp.road_id, wp.lane_id)
                self._straight_path = lane_path_from(wp, step=2.0)
                self._straight_idx = 0
            else:
                # Decide whether to keep following the existing straight path even if lane_id temporarily differs
                # Heuristic: if a usable target on the stored path exists ahead and we're not at the path end, keep it.
                tgt_on_stored = pick_ahead_from_path(self._straight_path)
                if tgt_on_stored is None or (
                        ego_tf.location.distance(self._straight_path[-1].transform.location) < 2.0):
                    # Path exhausted or unusable -> rebuild from current waypoint
                    self._straight_lane_key = (wp.road_id, wp.lane_id)
                    self._straight_path = lane_path_from(wp, step=2.0)
                    self._straight_idx = 0

            # Preselect next junction branch (build its path) but keep targeting the straight path until entry
            first_junc_wp, last_non_junc = first_junction_and_last_non(wp)
            if first_junc_wp is not None:
                junc_id = first_junc_wp.get_junction().id
                raw_opts = last_non_junc.next(2.0) or []
                branch_opts = driving_only([o for o in raw_opts if is_ahead_loc(o.transform.location)])
                if branch_opts:
                    # Keep existing choice if same junction; otherwise choose uniformly and build its path
                    if self._active_junction_id == junc_id and self._active_branch_lane_key is not None:
                        chosen = None
                        for cand in branch_opts:
                            if (cand.road_id, cand.lane_id) == self._active_branch_lane_key:
                                chosen = cand
                                break
                        if chosen is None:
                            chosen = self.rng.choice(branch_opts)
                    else:
                        chosen = self.rng.choice(branch_opts)
                        self._active_junction_id = junc_id
                        self._active_branch_lane_key = (chosen.road_id, chosen.lane_id)
                    self._active_path = lane_path_from(chosen, step=2.0)
                    self._active_path_idx = 0

            # Target from the *persisted straight* path
            tgt = pick_ahead_from_path(self._straight_path or [])
            return tgt if tgt is not None else (self._straight_path[0] if self._straight_path else wp)

        # ---------------- Inside a junction: stick to the preselected branch path ----------------
        if wp.is_junction:
            junc_id = wp.get_junction().id
            self._straight_idx = 0
            self._straight_lane_key = None
            self._straight_path = None
            if self._active_junction_id == junc_id and self._active_path:
                self._entered_active_junction = True
                tgt = pick_ahead_from_path(self._active_path)
                if tgt is not None:
                    return tgt
                # If path exists but no ahead target, return its last point as a safe fallback
                return self._active_path[-1]

            # Entered a junction without preselection (rare): choose an ahead DRIVING option and build its path
            opts = driving_only(s.next_options if s.next_options else wp.next(2.0))
            if opts:
                ahead_opts = [o for o in opts if is_ahead_loc(o.transform.location)]
                choice = self.rng.choice(ahead_opts or opts)
                self._active_junction_id = junc_id
                self._active_branch_lane_key = (choice.road_id, choice.lane_id)
                self._active_path = lane_path_from(choice, step=2.0)
                self._active_path_idx = 0
                self._entered_active_junction = True
                tgt = pick_ahead_from_path(self._active_path)
                return tgt if tgt is not None else choice

        # ---------------- Default: best-effort follow ahead DRIVING option ----------------
        opts = driving_only(s.next_options if s.next_options else wp.next(2.0))
        ahead_opts = [o for o in opts if is_ahead_loc(o.transform.location)]
        return (ahead_opts[0] if ahead_opts else (opts[0] if opts else wp))

    def _decide_blinkers(self, wp: carla.Waypoint, target_wp: carla.Waypoint) -> Tuple[bool, bool]:
        """Blinkers based on signed heading change between current and target waypoint."""
        yaw_now = math.radians(wp.transform.rotation.yaw)
        yaw_tgt = math.radians(target_wp.transform.rotation.yaw)
        dpsi = math.atan2(math.sin(yaw_tgt - yaw_now), math.cos(yaw_tgt - yaw_now))
        # Right turn has negative yaw change in CARLA coordinates; use threshold
        left = dpsi > math.radians(10.0)
        right = dpsi < -math.radians(10.0)
        return left, right

    def _speed_to_stop_in_distance(self, v0: float, dist: float, comfort_dec: float = 3.0) -> float:
        """
        Max allowable speed to be able to stop within 'dist' using v^2 = v0^2 + 2 a s, with a = -comfort_dec.
        """
        if dist <= 0.0:
            return 0.0
        # If moving too fast to stop, reduce target speed proportional to needed decel
        # Invert to compute a_k = (v^2 - 2 a s) etc.; here just compute a cap on speed
        vmax = math.sqrt(max(0.0, 2.0 * comfort_dec * dist))
        return min(v0, vmax)
