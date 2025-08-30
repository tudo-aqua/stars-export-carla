from __future__ import annotations

import math
from typing import List, Tuple, Optional

import carla

from .types import SensedState, Plan


class Acting:
    def __init__(self, agent_ctx):
        self.ctx = agent_ctx  # RuleBasedAgent
        self.cfg = agent_ctx.cfg
        self.ego = agent_ctx.ego

        # Persistent state
        self._steer_prev = 0.0
        self._last_control = carla.VehicleControl()
        self._launch_ts: Optional[float] = None

        # Headlight base when enabled
        self._base_light_state = carla.VehicleLightState(
            carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
        )

        # Vehicle geometry / limits
        self.wheel_base_m: float = getattr(self, "wheel_base_m", 2.80)
        self.front_axle_offset_m: float = getattr(self, "front_axle_offset_m", self.wheel_base_m * 0.5)
        self.max_steer_rad: float = getattr(self, "max_steer_rad", math.radians(70.0))

        # Stanley gains (as in the example formula)
        self.stanley_k: float = getattr(self, "stanley_k",
                                        0.5)  # close to example’s default :contentReference[oaicite:4]{index=4}
        self.stanley_soft: float = getattr(self, "stanley_soft", 1e-4)

        # Longitudinal shaping
        self.a_lat_comf: float = getattr(self.cfg, "a_lat_comf", 1.8)
        self.service_decel: float = 6.0

        # Smoothing
        self.steer_lpf_beta_fast = 0.33
        self.steer_lpf_beta_slow = 0.20
        self.low_speed_switch = 1.0

        self._throttle_i = 0.0
        self._last_speed_err = 0.0

    # ----------------------------- public -----------------------------

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

    # ----------------------------- internals -----------------------------

    def _apply_lights(self, p: Plan) -> None:
        """Set blinkers & headlights from plan."""
        state = 0
        if p.headlights_on:
            state |= int(self._base_light_state)
        if p.blink_left:
            state |= int(carla.VehicleLightState.LeftBlinker)
        if p.blink_right:
            state |= int(carla.VehicleLightState.RightBlinker)
        try:
            self.ego.set_light_state(carla.VehicleLightState(state))
        except Exception:
            pass

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

    def _shape_commands(self, throttle: float, brake: float, steer: float, dt: float
                        ) -> Tuple[float, float, float]:
        """Apply caps and rate limits from config."""
        cfg = self.cfg
        # Caps
        max_th = getattr(cfg, "max_throttle", 0.6)
        max_br = getattr(cfg, "max_brake", 0.8)
        max_st = getattr(cfg, "max_steer", 0.9)
        throttle = max(0.0, min(max_th, throttle))
        brake = max(0.0, min(max_br, brake))
        steer = max(-max_st, min(max_st, steer))

        # Rate limits
        pr = self._last_control
        d_th_max = getattr(cfg, "max_throttle_rate", 0.12)
        d_br_max = getattr(cfg, "max_brake_rate", 0.15)
        d_st_max = getattr(cfg, "max_steer_rate", 0.10)

        def clamp_rate(u, u_prev, du_max):
            lo = u_prev - du_max
            hi = u_prev + du_max
            return min(max(u, lo), hi)

        throttle = clamp_rate(throttle, getattr(pr, "throttle", 0.0), d_th_max)
        brake = clamp_rate(brake, getattr(pr, "brake", 0.0), d_br_max)
        steer = clamp_rate(steer, getattr(pr, "steer", 0.0), d_st_max)

        return throttle, brake, steer

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

    # ----------------------------- geometry helpers -----------------------------

    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def _loc_xy(loc: carla.Location) -> Tuple[float, float]:
        return (loc.x, loc.y)

    @staticmethod
    def _yaw_of_wp(wp: carla.Waypoint) -> float:
        return math.radians(wp.transform.rotation.yaw)

    def _forward_point_along_path(self, ego_xy: Tuple[float, float], path: List[carla.Waypoint],
                                  min_ahead: float = 4.0) -> Tuple[float, float]:
        for w in path:
            wx, wy = self._loc_xy(w.transform.location)
            if ((wx - ego_xy[0]) ** 2 + (wy - ego_xy[1]) ** 2) ** 0.5 >= min_ahead:
                return (wx, wy)
        # fallback: last point
        w = path[-1]
        return self._loc_xy(w.transform.location)

    def _project_front_onto_path(self, front_xy: Tuple[float, float], waypoints: List[carla.Waypoint]
                                 ) -> Tuple[float, float, float, float]:
        """
        Project the front-axle point onto the polyline path.
        Returns (px, py, path_yaw, signed_cte).
        """
        if not waypoints:
            wp = self.ctx.map.get_waypoint(self.ego.get_location(), project_to_road=True,
                                           lane_type=carla.LaneType.Driving)
            wx, wy = self._loc_xy(wp.transform.location)
            seg_yaw = self._yaw_of_wp(wp)
            nx, ny = -math.sin(seg_yaw), math.cos(seg_yaw)
            e_cte = (front_xy[0] - wx) * nx + (front_xy[1] - wy) * ny
            return wx, wy, seg_yaw, e_cte

        best = (float("inf"), 0.0, 0.0, 0.0, 0.0)  # (d2, px, py, yaw, e_cte)
        for i in range(len(waypoints) - 1):
            a = waypoints[i].transform.location
            b = waypoints[i + 1].transform.location
            ax, ay = a.x, a.y
            bx, by = b.x, b.y
            vx, vy = bx - ax, by - ay
            seg_len2 = vx * vx + vy * vy
            if seg_len2 < 1e-6:
                continue
            wx, wy = front_xy[0] - ax, front_xy[1] - ay
            t = (wx * vx + wy * vy) / seg_len2
            t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
            px, py = ax + t * vx, ay + t * vy
            seg_yaw = math.atan2(vy, vx)
            nx, ny = -math.sin(seg_yaw), math.cos(seg_yaw)
            e_cte = (front_xy[0] - px) * nx + (front_xy[1] - py) * ny
            d2 = (front_xy[0] - px) ** 2 + (front_xy[1] - py) ** 2
            if d2 < best[0]:
                best = (d2, px, py, seg_yaw, e_cte)

        if best[0] == float("inf"):
            w = waypoints[-1]
            wx, wy = self._loc_xy(w.transform.location)
            seg_yaw = self._yaw_of_wp(w)
            nx, ny = -math.sin(seg_yaw), math.cos(seg_yaw)
            e_cte = (front_xy[0] - wx) * nx + (front_xy[1] - wy) * ny
            return wx, wy, seg_yaw, e_cte

        _, px, py, seg_yaw, e_cte = best
        return px, py, seg_yaw, e_cte
