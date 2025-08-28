from __future__ import annotations

import math

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
        """
        Tick-wise control.
        Lateral: Stanley controller on full polyline path (front-axle reference).
        Longitudinal: simple speed P + stop logic + curve speed cap.

        Requires:
          - self.ego: carla.Vehicle
          - Optional tunables on self: wheel_base_m, front_axle_offset_m, max_steer_rad,
            stanley_k, stanley_soft, lat_accel_max, service_decel.
        """
        # ---------------- parameters (safe defaults, override on self if you want) ----------
        wheel_base_m = getattr(self, "wheel_base_m", 2.80)
        front_axle_offset_m = getattr(self, "front_axle_offset_m",
                                      wheel_base_m * 0.5)  # from actor origin to front axle
        max_steer_rad = getattr(self, "max_steer_rad", math.radians(70.0))

        # Stanley gains (per article): delta = theta_e + atan2(k * e_cte, v + k_soft)
        stanley_k = getattr(self, "stanley_k", 1.6)  # cross-track gain
        stanley_soft = getattr(self, "stanley_soft", 1.0)  # softening, helps at very low v

        # Longitudinal shaping
        lat_accel_max = getattr(self, "lat_accel_max", 2.8)  # curve speed cap via v_max = sqrt(a_lat / |kappa|)
        service_decel = getattr(self, "service_decel", 6.0)  # comfortable braking
        launch_base = getattr(self, "launch_base", 0.16)  # small base throttle below ~0.5 m/s
        kp_speed = getattr(self, "kp_speed", 0.28)

        # Smoothing
        steer_lpf_beta_fast = 0.33
        steer_lpf_beta_slow = 0.20
        low_speed_switch = 1.0  # below this, allow pointing-to-target fallback

        if not hasattr(self, "_steer_prev"):
            self._steer_prev = 0.0

        # ---------------- helpers ----------------------------------------------------------
        def clip(x, lo, hi):
            return lo if x < lo else hi if x > hi else x

        def wrap(a):
            # wrap to [-pi, pi]
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            return a

        def loc_xy(loc: carla.Location):
            return (loc.x, loc.y)

        def yaw_of_wp(wp: carla.Waypoint) -> float:
            return math.radians(wp.transform.rotation.yaw)

        # ---------------- longitudinal target speed ---------------------------------------
        target_speed = min(p.target_speed_mps, s.speed_limit_mps)

        # Optional curve-speed limiting (keeps lateral under control on tight bends)
        if abs(s.curvature) > 1e-6:
            target_speed = min(target_speed, math.sqrt(lat_accel_max / abs(s.curvature)))

        # Stop logic (TL/signs/plan)
        if p.stop_now:
            target_speed = 0.0
        else:
            if p.stop_distance > 0.0 and math.isfinite(p.stop_distance) and s.speed_mps > 0.05:
                # Brake if physics says we must to stop in time (+ buffer)
                d_need = (s.speed_mps ** 2) / (2.0 * service_decel) + 2.0
                if p.stop_distance <= d_need:
                    target_speed = 0.0

        # ---------------- lateral control: Stanley on polyline -----------------------------
        ego_tf: carla.Transform = self.ego.get_transform()
        ego_loc: carla.Location = ego_tf.location
        ego_yaw = math.radians(ego_tf.rotation.yaw)
        # front axle reference point (Stanley uses front axle)
        fx = ego_loc.x + front_axle_offset_m * math.cos(ego_yaw)
        fy = ego_loc.y + front_axle_offset_m * math.sin(ego_yaw)

        path = p.path_waypoints if p.path_waypoints else [p.target_wp or s.wp]

        # Project the front-axle point onto the path (piecewise-linear polyline)
        # Return: nearest point (px,py), tangent yaw at projection, signed cross-track error
        def project_front_onto_path(front_xy, waypoints) -> tuple[float, float, float, float]:
            if not waypoints:
                # Fallback: use target waypoint
                wp = p.target_wp or s.wp
                wx, wy = loc_xy(wp.transform.location)
                tx = math.cos(yaw_of_wp(wp))
                ty = math.sin(yaw_of_wp(wp))
                # Signed lateral error relative to path normal (left normal)
                nx, ny = -ty, tx
                e_cte = (front_xy[0] - wx) * nx + (front_xy[1] - wy) * ny
                return wx, wy, yaw_of_wp(wp), e_cte

            best = (float("inf"), 0.0, 0.0, 0.0, 0.0)  # (dist2, px, py, yaw, e_cte)
            # Consider a sliding window of the next ~50m to avoid snapping to far-back segments
            # Build a cheap subset: first N points or those within radius
            subset = waypoints
            # Iterate consecutive segments
            for i in range(len(subset) - 1):
                a = subset[i].transform.location
                b = subset[i + 1].transform.location
                ax, ay = a.x, a.y
                bx, by = b.x, b.y
                vx, vy = bx - ax, by - ay
                seg_len2 = vx * vx + vy * vy
                if seg_len2 < 1e-6:
                    continue
                # parametric projection t in [0, 1]
                wx, wy = front_xy[0] - ax, front_xy[1] - ay
                t = (wx * vx + wy * vy) / seg_len2
                t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
                px, py = ax + t * vx, ay + t * vy
                # tangent and left-normal at projection
                seg_yaw = math.atan2(vy, vx)
                nx, ny = -math.sin(seg_yaw), math.cos(seg_yaw)
                e_cte = (front_xy[0] - px) * nx + (front_xy[1] - py) * ny
                d2 = (front_xy[0] - px) ** 2 + (front_xy[1] - py) ** 2
                if d2 < best[0]:
                    best = (d2, px, py, seg_yaw, e_cte)

            if best[0] == float("inf"):
                # degenerate; fall back to last waypoint
                wp = waypoints[-1]
                wx, wy = loc_xy(wp.transform.location)
                seg_yaw = yaw_of_wp(wp)
                nx, ny = -math.sin(seg_yaw), math.cos(seg_yaw)
                e_cte = (front_xy[0] - wx) * nx + (front_xy[1] - wy) * ny
                return wx, wy, seg_yaw, e_cte

            _, px, py, seg_yaw, e_cte = best
            return px, py, seg_yaw, e_cte

        px, py, path_yaw, e_cte = project_front_onto_path((fx, fy), path)

        # Heading error (path heading - vehicle heading), wrapped
        theta_e = wrap(path_yaw - ego_yaw)

        # Stanley steering law (per article): delta = theta_e + atan2(k * e_cte, v + k_soft)
        v = max(s.speed_mps, 0.05)
        delta = theta_e + math.atan2(stanley_k * e_cte, v + stanley_soft)

        # Low-speed fallback: if crawling, point toward a forward waypoint to avoid jitter
        if v < low_speed_switch:
            # pick a waypoint ~3–8 m ahead to aim at
            target = None
            ego_xy = (ego_loc.x, ego_loc.y)
            for w in path:
                wx, wy = loc_xy(w.transform.location)
                dist = ((wx - ego_xy[0]) ** 2 + (wy - ego_xy[1]) ** 2) ** 0.5
                if dist >= 4.0:
                    target = (wx, wy)
                    break
            if target is None:
                twp = path[-1]
                target = loc_xy(twp.transform.location)
            ang_to_tgt = math.atan2(target[1] - ego_xy[1], target[0] - ego_xy[0])
            delta = wrap(ang_to_tgt - ego_yaw)

        # Normalize to [-1, 1] steering command with light low-pass smoothing
        steer_raw = clip(delta / max_steer_rad, -1.0, 1.0)
        beta = steer_lpf_beta_fast if v > 8.0 else steer_lpf_beta_slow
        steer_cmd = (1.0 - beta) * self._steer_prev + beta * steer_raw
        self._steer_prev = steer_cmd

        # ---------------- longitudinal control -------------------------------------------
        e_v = target_speed - s.speed_mps
        throttle = 0.0
        brake = 0.0

        if target_speed <= 0.05:
            throttle, brake = 0.0, (1.0 if s.speed_mps > 0.2 else 0.3)
        else:
            if e_v >= 0.0:
                base = launch_base if s.speed_mps < 0.5 else 0.05
                throttle = base + kp_speed * e_v
                brake = 0.0
            else:
                throttle = 0.0
                brake = clip(0.35 * (-e_v), 0.0, 1.0)

            # Ease throttle in very sharp curves to help lateral tracking
            if abs(s.curvature) > 0.01:
                throttle *= clip(1.0 - 0.7 * min(1.0, abs(s.curvature) / 0.1), 0.3, 1.0)

            # Near target → coast
            if abs(e_v) < 0.1 and target_speed > 0.5:
                throttle *= 0.5
                brake = 0.0

        throttle = clip(throttle, 0.0, 1.0)
        brake = clip(brake, 0.0, 1.0)

        # ---------------- finalize --------------------------------------------------------
        return carla.VehicleControl(
            throttle=throttle,
            steer=steer_cmd,
            brake=brake,
            hand_brake=False,
            reverse=False,
            manual_gear_shift=False
        )
