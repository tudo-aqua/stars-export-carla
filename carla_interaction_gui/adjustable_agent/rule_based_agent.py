"""
Rule-based CARLA Agent (modular, debuggable)

This agent is intentionally simple, deterministic per stage, and split into
sensing → planning → acting phases so you can tweak each part independently.

Key properties implemented:
- Follows speed limits (from the map waypoint's speed_limit; converted to m/s)
- Always stops for red traffic lights, stop signs, and yield signs
- Keeps safe gaps to avoid collisions (basic TTC + headway model)
- Chooses a random road at junctions with equal probability among options
- Uses turn signals when changing branch direction
- Turns on lights when it is dark (based on weather sun altitude angle)
- Stays centered in lane and keeps its lane while turning (pure‑pursuit‑like)
- Avoids harsh braking and harsh acceleration (rate‑limited, comfort limits)

Usage (example):
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    ego = <spawn a vehicle, set role_name='hero'>
    agent = RuleBasedAgent(ego, world)

    while True:
        control, debug_info = agent.run_step()
        ego.apply_control(control)
        world.tick()

NOTE: This file only uses the public CARLA Python API. If you want to tightly
integrate with your static map structures (DataWorld/DataLane/etc.), see the
hooks marked with "OPTIONAL: integrate MapRasterizer".
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict

import carla


# ===============================
# Config & Utilities
# ===============================

@dataclass
class AgentConfig:
    # Control loop
    dt: float = 0.05  # seconds; sync world recommended

    # Speed control (PID-ish)
    v_kp: float = 0.8
    v_ki: float = 0.05
    v_kd: float = 0.1

    # Lateral control (pure‑pursuit‑like with heading + cross-track terms)
    lookahead_min: float = 4.0  # m
    lookahead_max: float = 14.0  # m
    lookahead_speed_gain: float = 0.4  # m per (m/s)
    lat_k_heading: float = 1.8
    lat_k_cte: float = 0.2

    # Comfort limits
    max_throttle: float = 0.6
    max_brake: float = 0.8
    max_throttle_rate: float = 0.12  # per step
    max_brake_rate: float = 0.15  # per step
    max_steer_rate: float = 0.15  # per step
    max_steer: float = 0.9  # absolute

    # Desired headway & emergency TTC thresholds
    time_headway: float = 1.5  # seconds
    min_gap: float = 3.0  # meters (standstill gap)
    emergency_ttc: float = 2.0  # seconds

    # Red/yellow light behavior
    stop_buffer: float = 1.3  # meters before the line/landmark

    # Stop/Yield landmark lookahead distance
    landmark_lookahead: float = 35.0  # meters

    # If True, fully stop at YIELD (per user request).
    treat_yield_as_stop: bool = True

    # Lights
    sun_angle_headlights_deg: float = 5.0  # turn on if sun altitude below this

    # Random seed for route choices (makes tests deterministic if set)
    seed: Optional[int] = None


# ===============================
# Internal State Containers
# ===============================

@dataclass
class SensedState:
    wp: carla.Waypoint
    speed_mps: float
    speed_limit_mps: float
    in_junction: bool
    next_options: List[carla.Waypoint]
    traffic_light_state: Optional[carla.TrafficLightState]
    at_traffic_light: bool
    stop_or_yield_ahead: Optional[Tuple[str, float]]  # ("STOP"/"YIELD", distance)
    sun_altitude_angle: float
    lead_vehicle: Optional[carla.Actor]
    lead_distance: float = math.inf
    lead_rel_speed: float = 0.0
    curvature: float = 0.0  # local curvature estimate (1/m)


@dataclass
class Plan:
    target_speed_mps: float
    target_wp: carla.Waypoint
    blink_left: bool = False
    blink_right: bool = False
    headlights_on: bool = False
    stop_now: bool = False  # used to force immediate stop (red light at line, etc.)
    stop_distance: float = 0.0  # meters to stop line


# ===============================
# Agent
# ===============================

class RuleBasedAgent:
    def __init__(self, ego: carla.Vehicle, client: carla.Client, config: Optional[AgentConfig] = None):
        self.ego = ego
        self.client = client
        self.world = client.get_world()
        self.map = self.world.get_map()
        self.cfg = config or AgentConfig()

        # Random for route choices
        self.rng = random.Random(self.cfg.seed)

        # Controller accumulators / memory
        self._throttle_i = 0.0
        self._last_speed_err = 0.0
        self._last_control = carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0)

        # Junction decision memory (persist one choice per junction)
        self._active_junction_id: Optional[int] = None
        self._active_branch_lane_id: Optional[int] = None  # choose lane_id when inside junction

        # Light state base (we preserve non‑blinker bits separately)
        self._base_light_state = carla.VehicleLightState.NONE

        # Make sure ego has physics control enabled
        physics_control = self.ego.get_physics_control()
        physics_control.use_sweep_wheel_collision = True  # safer collisions on wheels
        self.ego.apply_physics_control(physics_control)

    # -------------- public --------------

    def run_step(self) -> Tuple[carla.VehicleControl, Dict]:
        """
        One update tick: sense → plan → act.
        Returns (control, debug_info_dict).
        """
        state = self.sense()
        plan = self.plan(state)
        control = self.act(state, plan)

        # Small debug bundle for plotting / logs
        debug = {
            "speed_mps": state.speed_mps,
            "speed_limit_mps": state.speed_limit_mps,
            "target_speed_mps": plan.target_speed_mps,
            "in_junction": state.in_junction,
            "next_options": len(state.next_options),
            "lead_distance": state.lead_distance,
            "lead_rel_speed": state.lead_rel_speed,
            "tl_state": str(state.traffic_light_state) if state.traffic_light_state else None,
            "stop_or_yield": state.stop_or_yield_ahead,
            "blink_left": plan.blink_left,
            "blink_right": plan.blink_right,
            "headlights_on": plan.headlights_on,
            "steer": control.steer,
            "throttle": control.throttle,
            "brake": control.brake,
        }
        return control, debug

    # ===============================
    # SENSING
    # ===============================

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

    # ===============================
    # PLANNING
    # ===============================

    def plan(self, s: SensedState) -> Plan:
        # --- base target speed: respect speed limit ---
        target_speed = s.speed_limit_mps

        # --- traffic lights: always stop at red ---
        stop_now = False
        stop_distance = 0.0
        if s.at_traffic_light and s.traffic_light_state is not None:
            if s.traffic_light_state in (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow):
                # When at the stop line, force stop. (Yellow treated conservatively.)
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

    def _choose_target_waypoint(self, s: SensedState) -> carla.Waypoint:
        """
        If branching (len(next_options) > 1), pick a lane uniformly at random.
        Persist the decision while inside the same junction.
        """
        wp = s.wp
        options = s.next_options if s.next_options else wp.next(2.0)

        # Track junction id if applicable (persist choice)
        junction_id = wp.get_junction().id if wp.is_junction else None

        if wp.is_junction:
            # Continue active choice if same junction
            if self._active_junction_id == junction_id and self._active_branch_lane_id is not None:
                for cand in options:
                    if cand.lane_id == self._active_branch_lane_id:
                        return cand
            # Pick new choice uniformly
            choice = self.rng.choice(options) if options else wp.next(3.0)[0]
            self._active_junction_id = junction_id
            self._active_branch_lane_id = choice.lane_id
            return choice
        else:
            # Reset when outside
            self._active_junction_id = None
            self._active_branch_lane_id = None
            return options[0] if options else wp

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

    # ===============================
    # ACTING
    # ===============================

    def act(self, s: SensedState, p: Plan) -> carla.VehicleControl:
        # --- Lights (headlights + blinkers) ---
        self._apply_lights(p)

        # --- Lateral control (stay in middle of lane, keep lane on turns) ---
        steer_cmd = self._lateral_control(p.target_wp)

        # --- Longitudinal control (respect target speed; avoid harsh accel/brake) ---
        throttle_cmd, brake_cmd = self._longitudinal_control(s, p)

        # --- Rate limiting for smoothness ---
        last = self._last_control
        steer_cmd = self._rate_limit(steer_cmd, last.steer, self.cfg.max_steer_rate, -self.cfg.max_steer,
                                     self.cfg.max_steer)
        throttle_cmd = self._rate_limit(throttle_cmd, last.throttle, self.cfg.max_throttle_rate, 0.0,
                                        self.cfg.max_throttle)
        brake_cmd = self._rate_limit(brake_cmd, last.brake, self.cfg.max_brake_rate, 0.0, self.cfg.max_brake)

        control = carla.VehicleControl(
            throttle=float(throttle_cmd),
            brake=float(brake_cmd),
            steer=float(steer_cmd),
            hand_brake=False,
            reverse=False
        )

        self._last_control = control
        return control

    def _rate_limit(self, value: float, last: float, max_delta: float, lo: float, hi: float) -> float:
        """
        Clamp the step-to-step change to ±max_delta and keep within [lo, hi].
        The rate limits are *per control step* (matching AgentConfig comments).
        """
        # Sanitize input
        if value is None or not math.isfinite(value):
            value = 0.0

        # First clamp target into bounds
        value = max(lo, min(hi, float(value)))

        # Limit change relative to last command
        delta = value - float(last)
        if delta > max_delta:
            value = last + max_delta
        elif delta < -max_delta:
            value = last - max_delta

        # Small deadband to avoid tiny jitter around zero
        if abs(value) < 1e-4 and lo <= 0.0 <= hi:
            value = 0.0

        # Final clamp
        return max(lo, min(hi, value))

    def _apply_lights(self, p: Plan) -> None:
        """Set vehicle lights: headlights when dark; blinkers for turns."""
        state = self._base_light_state

        # Headlights
        if p.headlights_on:
            state |= carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
        else:
            state &= ~(carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)

        # Blinkers
        state &= ~(carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.RightBlinker)
        if p.blink_left:
            state |= carla.VehicleLightState.LeftBlinker
        if p.blink_right:
            state |= carla.VehicleLightState.RightBlinker

        try:
            self.ego.set_light_state(carla.VehicleLightState(state))
        except Exception:
            # Some vehicle types may not support all lights; ignore safely.
            pass

    def _target_lookahead(self, speed_mps: float) -> float:
        return max(self.cfg.lookahead_min,
                   min(self.cfg.lookahead_max, self.cfg.lookahead_min + self.cfg.lookahead_speed_gain * speed_mps))

    def _lateral_control(self, target_wp: carla.Waypoint) -> float:
        """
        Pure‑pursuit‑like steering using heading error and cross‑track error to the target waypoint.
        """
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        speed_vec = self.ego.get_velocity()
        speed = math.sqrt(speed_vec.x ** 2 + speed_vec.y ** 2 + speed_vec.z ** 2)

        # Choose a point ahead along the target branch based on speed
        Ld = self._target_lookahead(speed)
        fut = target_wp.next(Ld)
        if not fut:
            fut = [target_wp]
        tgt = fut[-1].transform.location

        # Heading error
        yaw = math.radians(ego_tf.rotation.yaw)
        path_yaw = math.radians(target_wp.transform.rotation.yaw)
        err_heading = math.atan2(math.sin(path_yaw - yaw), math.cos(path_yaw - yaw))

        # Cross‑track error (signed)
        cte = self._signed_lateral_offset(ego_tf, tgt)

        steer = self.cfg.lat_k_heading * err_heading + self.cfg.lat_k_cte * (cte / max(Ld, 1e-3))
        steer = max(-self.cfg.max_steer, min(self.cfg.max_steer, steer))
        return steer

    def _longitudinal_control(self, s: SensedState, p: Plan) -> Tuple[float, float]:
        """
        PID‑like speed control with comfort deceleration and explicit full stops.
        Outputs (throttle, brake).
        """
        v_ref = p.target_speed_mps
        v = s.speed_mps
        e = v_ref - v

        # Integral / derivative
        self._throttle_i += e * self.cfg.dt
        de = (e - self._last_speed_err) / max(self.cfg.dt, 1e-3)
        self._last_speed_err = e

        # Base "throttle" command
        raw = self.cfg.v_kp * e + self.cfg.v_ki * self._throttle_i + self.cfg.v_kd * de

        throttle = 0.0
        brake = 0.0

        if p.stop_now or v_ref <= 0.1:
            # Hard request to stop (red at line / stop sign)
            throttle = 0.0
            # Brake proportional to speed
            brake = min(self.cfg.max_brake, 0.3 + 0.2 * v)
            # Reset integrator to avoid windup
            self._throttle_i = 0.0
        elif raw >= 0.0:
            throttle = max(0.0, min(self.cfg.max_throttle, raw))
            brake = 0.0
        else:
            # Need deceleration
            desired_decel = min(3.0, -raw)  # cap to comfort
            brake = max(0.0, min(self.cfg.max_brake, desired_decel / 3.0))
            throttle = 0.0
            # Avoid integrator windup when braking
            self._throttle_i = 0.0

        return throttle, brake

    # ===============================
    # OPTIONAL: integrate MapRasterizer / DataWorld
    # ===============================
    # If you want to pull speed limits or stop lines from your static map model,
    # you can inject a feature provider here (e.g., MapRasterizer) and read:
    # - lane midpoints to compute more precise "middle of lane" targets,
    # - DataLane.speed_limits (stored as m/s in your model),
    # - DataStaticTrafficLight.stop_locations for exact stop lines, etc.
    #
    # For quick drop-in, the agent uses CARLA's waypoint API which already
    # provides speed limits and connectivity information.


# -------------------------
# Simple helper to run agent outside of your framework
# -------------------------
def drive_ego_hero(client: carla.Client, ego: carla.Vehicle, seed: Optional[int] = 0) -> RuleBasedAgent:
    """
    Convenience function: create agent with deterministic route choices.
    Make sure world is in synchronous mode with a fixed delta.
    """
    cfg = AgentConfig(seed=seed)
    agent = RuleBasedAgent(ego, client, cfg)
    return agent
