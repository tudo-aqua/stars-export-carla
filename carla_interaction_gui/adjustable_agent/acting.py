from __future__ import annotations

import math
from typing import Tuple

import carla

from .types import SensedState, Plan


class Acting:
    def __init__(self, agent_ctx):
        self.ctx = agent_ctx  # RuleBasedAgent
        self.cfg = agent_ctx.cfg
        self._last_control = agent_ctx.last_control
        self.ego = agent_ctx.ego
        self._base_light_state = carla.VehicleLightState(
            carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam)
        self._throttle_i = 0.0
        self._last_speed_err = 0.0

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

    def _signed_lateral_offset(self, tf: carla.Transform, point: carla.Location) -> float:
        """Signed lateral offset of 'point' from the ego longitudinal axis (left positive)."""
        dx = point.x - tf.location.x
        dy = point.y - tf.location.y
        yaw = math.radians(tf.rotation.yaw)
        nx = -math.sin(yaw)  # left normal
        ny = math.cos(yaw)
        return dx * nx + dy * ny
