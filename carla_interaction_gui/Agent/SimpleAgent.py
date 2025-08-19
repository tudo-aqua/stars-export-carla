import math
import os


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _dot(a, b) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z


class _PID:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.prev = None
        self.out_min, self.out_max = out_min, out_max

    def step(self, err, dt):
        self.i += err * dt
        d = 0.0 if self.prev is None else (err - self.prev) / max(dt, 1e-5)
        self.prev = err
        return _clamp(self.kp * err + self.ki * self.i + self.kd * d, self.out_min, self.out_max)


class SimpleAgent:
    """
    Minimal lane-following agent with:
     - waypoint lookahead + forward-aligned candidate selection
     - PID heading control
     - simple adaptive cruise & red-light stop
    """

    def __init__(self, vehicle, target_speed_kph: float = 35.0):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.map = self.world.get_map()
        self.target_speed_kph = float(target_speed_kph)
        self.steer_pid = _PID(1.6, 0.0, 0.25, -1.0, 1.0)
        self.speed_pid = _PID(0.10, 0.02, 0.0, -1.0, 1.0)

    def _select_ahead_wp(self, current_wp, ego_tf, lookahead: float = 2.0):
        candidates = current_wp.next(lookahead) or current_wp.next(2.0)
        if not candidates:
            return None
        fwd = ego_tf.get_forward_vector()
        loc = ego_tf.location
        best, best_score = None, None
        for wp in candidates:
            tgt = wp.transform.location
            vecx, vecy = (tgt.x - loc.x), (tgt.y - loc.y)
            norm = (vecx * vecx + vecy * vecy) ** 0.5 + 1e-3
            score = ((tgt.x - loc.x) * fwd.x + (tgt.y - loc.y) * fwd.y) / norm
            if best is None or score > best_score:
                best, best_score = wp, score
        return best

    def _lead_vehicle_distance(self, max_range=25.0):
        import carla  # available after manual_control injected CARLA
        ego = self.vehicle
        ego_wp = self.map.get_waypoint(ego.get_location(), project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
        if not ego_wp:
            return None
        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        best_d = None
        for veh in self.world.get_actors().filter('vehicle.*'):
            if veh.id == ego.id:
                continue
            wp = self.map.get_waypoint(veh.get_location(), project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
            if not wp or (wp.road_id, wp.lane_id) != (ego_wp.road_id, ego_wp.lane_id):
                continue
            rel = veh.get_location() - ego_tf.location
            ahead = _dot(rel, fwd)
            if ahead <= 0:
                continue
            d = (rel.x * rel.x + rel.y * rel.y + rel.z * rel.z) ** 0.5
            if d <= max_range and (best_d is None or d < best_d):
                best_d = d
        return best_d

    def _red_light_ahead(self):
        return self.vehicle.is_at_traffic_light()

    def run_step(self, dt=0.05):
        import carla
        v = self.vehicle
        tf = v.get_transform()
        loc = tf.location
        yaw = math.radians(tf.rotation.yaw)

        current_wp = self.map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if not current_wp:
            return carla.VehicleControl(hand_brake=False, brake=1.0)

        target_wp = self._select_ahead_wp(current_wp, tf, lookahead=8.0)
        if not target_wp:
            return carla.VehicleControl(hand_brake=False, brake=1.0)

        tgt = target_wp.transform.location
        desired_yaw = math.atan2(tgt.y - loc.y, tgt.x - loc.x)
        err_yaw = ((desired_yaw - yaw + math.pi) % (2 * math.pi)) - math.pi
        steer = self.steer_pid.step(err_yaw, dt)

        # target speed
        lead = self._lead_vehicle_distance()
        red = self._red_light_ahead()
        # compute current speed in kph
        vlin = v.get_velocity()
        speed = 3.6 * (vlin.x * vlin.x + vlin.y * vlin.y + vlin.z * vlin.z) ** 0.5
        target = self.target_speed_kph
        if red:
            target = 0.0
        elif lead is not None:
            target = min(target, max(0.0, (lead - 6.0) * 2.2))  # ~2.2 kph/m past 6 m

        accel = self.speed_pid.step(target - speed, dt)
        if target <= 0.1 or (speed < 1.0 and accel < 0):
            throttle, brake = 0.0, _clamp(-accel, 0.0, 1.0)
        else:
            throttle, brake = _clamp(accel, 0.0, 1.0), 0.0

        return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)


def _patch_keyboard_for_agent(mc):
    """
    Patch KeyboardControl.parse_events in the dynamically loaded manual_control module
    so that when autopilot is toggled ON, we run SimpleAgent each tick instead of TM.
    """
    import carla  # will succeed after manual_control import
    orig_parse = mc.KeyboardControl.parse_events

    def parse_events_with_agent(self, client, world, clock, sync_mode):
        ret = orig_parse(self, client, world, clock, sync_mode)
        try:
            if isinstance(world.player, carla.Vehicle) and getattr(self, "_autopilot_enabled", False):
                world.player.set_autopilot(False)  # ensure TM is OFF
                # bind agent to current vehicle if missing / changed
                try:
                    tgt_kph = float(os.getenv("AGENT_TARGET_KPH", "35"))
                except Exception:
                    tgt_kph = 35.0
                agent = getattr(self, "_agent", None)
                if not agent or not hasattr(agent, "vehicle") or agent.vehicle.id != world.player.id:
                    agent = SimpleAgent(world.player, target_speed_kph=tgt_kph)
                    self._agent = agent

                # dt from settings if sync, otherwise from clock
                dt = 0.0
                try:
                    settings = world.world.get_settings() if hasattr(world, "world") else None
                    if settings:
                        dt = settings.fixed_delta_seconds or 0.0
                except Exception:
                    dt = 0.0
                if not dt or dt <= 0:
                    dt = max(clock.get_time() / 1000.0, 1 / 60)

                ctrl = agent.run_step(dt=dt)
                world.player.apply_control(ctrl)
        except Exception as e:
            try:
                world.hud.notification(f"Agent error: {e}")
            except Exception:
                pass
        return ret

    mc.KeyboardControl.parse_events = parse_events_with_agent
