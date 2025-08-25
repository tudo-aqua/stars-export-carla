# agent_traffic_lights.py
import math
import random
from collections import deque
from typing import Dict, Deque, Optional, Tuple, List

import carla

from .agent_constants import constants as C
from .agent_localization import LocalizationBuffer
from .agent_parameters import AgentParameters


class TrafficLightLogic:
    def __init__(self, vehicle: carla.Vehicle, params: AgentParameters):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.params = params
        self.entering_vehicles_map: Dict[Tuple[int, int, int], Deque[int]] = {}
        self.vehicle_last_junction: Dict[int, Tuple[int, int, int]] = {}
        self.vehicle_stop_time: Dict[int, float] = {}

        # Sticky red‑light handling: remember the controlling TL until it turns green or we pass it
        self.active_tl_id: Dict[int, int] = {}
        self.active_stop_wp: Dict[int, carla.Waypoint] = {}
        self.active_will_stop: Dict[int, bool] = {}  # TM semantics: stop if get_running_light <= rand

    @staticmethod
    def _junction_id(wp: Optional[carla.Waypoint]):
        if not wp or not wp.is_junction: return None
        return (wp.road_id, wp.section_id, wp.lane_id)

    def _get_affected_junction(self, loc: LocalizationBuffer):
        look_wp, _ = loc.get_target_waypoint(C.WaypointSelection.JUNCTION_LOOK_AHEAD)
        front_wp = loc.buffer[0]
        look_id = self._junction_id(look_wp);
        front_id = self._junction_id(front_wp)
        current_id = self.vehicle_last_junction.get(self.vehicle.id)
        if current_id is not None:
            if current_id == look_id: return look_id
            if look_id is not None: return look_id
            if current_id == front_id: return front_id
            return None
        return look_id

    def _add_actor_to_non_signalised(self, actor_id: int, junction_id) -> None:
        q = self.entering_vehicles_map.setdefault(junction_id, deque())
        if actor_id not in q: q.append(actor_id)
        if actor_id in self.vehicle_last_junction and self.vehicle_last_junction[actor_id] != junction_id:
            self._remove_actor(actor_id)
        self.vehicle_last_junction[actor_id] = junction_id

    def _remove_actor(self, actor_id: int) -> None:
        if actor_id in self.vehicle_last_junction:
            junc_id = self.vehicle_last_junction[actor_id]
            dq = self.entering_vehicles_map.get(junc_id, deque())
            try:
                dq.remove(actor_id)
            except ValueError:
                pass
            self.vehicle_stop_time.pop(actor_id, None)
            self.vehicle_last_junction.pop(actor_id, None)

    # probe ahead for controlling TL
    def _flatten_stop_wps(self, tl: carla.TrafficLight) -> List[carla.Waypoint]:
        stops: List[carla.Waypoint] = []
        if hasattr(tl, "get_stop_waypoints"):
            for entry in tl.get_stop_waypoints() or []:
                stops += entry if isinstance(entry, list) else [entry]
        elif hasattr(tl, "get_stop_lines"):
            pts = tl.get_stop_lines() or []
            m = self.world.get_map()
            for p in pts:
                wp = m.get_waypoint(p, project_to_road=True, lane_type=carla.LaneType.Driving)
                if wp: stops.append(wp)
        return stops

    def _find_controlling_tl_ahead(self, loc: LocalizationBuffer) -> Optional[carla.TrafficLight]:
        v = self.vehicle;
        vloc = v.get_location();
        fwd = v.get_transform().get_forward_vector()
        front_wp = loc.buffer[0]
        best, best_d = None, float("inf")
        for tl in self.world.get_actors().filter("traffic.traffic_light*"):
            for wp in self._flatten_stop_wps(tl):
                if (wp.road_id, wp.lane_id) != (front_wp.road_id, front_wp.lane_id): continue
                sl = wp.transform.location
                rel = carla.Vector3D(sl.x - vloc.x, sl.y - vloc.y, sl.z - vloc.z)
                if rel.x * fwd.x + rel.y * fwd.y + rel.z * fwd.z <= 0: continue
                d = math.sqrt((sl.x - vloc.x) ** 2 + (sl.y - vloc.y) ** 2 + (sl.z - vloc.z) ** 2)
                if d < best_d and d <= 45.0: best, best_d = tl, d
        return best

    def _ahead_path_lane_keys(self, loc: LocalizationBuffer,
                              look_dist: float = C.WaypointSelection.JUNCTION_LOOK_AHEAD) -> List[tuple]:
        """Collect (road_id, lane_id) pairs along the buffered path up to look_dist."""
        keys = []
        acc = 0.0
        buf = list(loc.buffer)
        for i in range(1, len(buf)):
            a = buf[i - 1].transform.location;
            b = buf[i].transform.location
            dx, dy, dz = a.x - b.x, a.y - b.y, a.z - b.z
            d = (dx * dx + dy * dy + dz * dz) ** 0.5
            acc += d
            keys.append((buf[i].road_id, buf[i].lane_id))
            if acc >= look_dist: break
        if not keys and buf:
            keys.append((buf[0].road_id, buf[0].lane_id))
        return keys

    def _find_controlling_tl_on_path(self, loc: LocalizationBuffer):
        """Return (tl, stop_wp, dist) for the closest red/yellow/any TL whose stop line lies on the path ahead."""
        v = self.vehicle
        m = self.world.get_map()
        vloc = v.get_location()
        fwd = v.get_transform().get_forward_vector()
        lane_keys = set(self._ahead_path_lane_keys(loc))
        best = None
        best_wp = None
        best_d = float('inf')
        for tl in self.world.get_actors().filter('traffic.traffic_light*'):
            for wp in self._flatten_stop_wps(tl):
                key = (wp.road_id, wp.lane_id)
                if key not in lane_keys:
                    continue
                sl = wp.transform.location
                rel = carla.Vector3D(sl.x - vloc.x, sl.y - vloc.y, sl.z - vloc.z)
                if rel.x * fwd.x + rel.y * fwd.y + rel.z * fwd.z <= 0:
                    continue
                d = ((sl.x - vloc.x) ** 2 + (sl.y - vloc.y) ** 2 + (sl.z - vloc.z) ** 2) ** 0.5
                if d < best_d:
                    best, best_wp, best_d = tl, wp, d
        return best, best_wp, best_d

    def _handle_non_signalised(self, ego_id: int, junc_id) -> bool:
        dq = self.entering_vehicles_map.get(junc_id, deque())
        now = self.world.get_snapshot().timestamp.elapsed_seconds
        if ego_id not in self.vehicle_stop_time:
            self.vehicle_stop_time[ego_id] = now;
            return True
        if dq and dq[0] == ego_id:
            entry_time = self.vehicle_stop_time.get(ego_id, now)
            if now - entry_time < C.TrafficLight.MINIMUM_STOP_TIME: return True
            try:
                dq.popleft()
            except Exception:
                pass
            self.vehicle_last_junction.pop(ego_id, None)
            self.vehicle_stop_time.pop(ego_id, None)
            return False
        return True

    def update(self, loc: LocalizationBuffer) -> bool:
        ego_id = self.vehicle.id

        # Determine affected junction (for non‑signalized logic)
        affected = self._get_affected_junction(loc)

        # Prefer CARLA's own binding if we are AT a TL
        tl = self.vehicle.get_traffic_light() if self.vehicle.is_at_traffic_light() else None
        at_tl = bool(tl)
        if tl is None and affected is not None:
            # Path‑aware search for controlling TL ahead (closest on our buffered path)
            tl, stop_wp, dist = self._find_controlling_tl_on_path(loc)
        else:
            stop_wp = None
            dist = float('inf')
            if tl is not None:
                # Try to determine stop waypoint for the bound TL
                stops = self._flatten_stop_wps(tl)
                stop_wp = stops[0] if stops else None
                if stop_wp is not None:
                    sl = stop_wp.transform.location;
                    vloc = self.vehicle.get_location()
                    dx, dy, dz = sl.x - vloc.x, sl.y - vloc.y, sl.z - vloc.z
                    dist = (dx * dx + dy * dy + dz * dz) ** 0.5

        # Read light state (default Green if none found)
        tl_state = tl.get_state() if tl is not None else carla.TrafficLightState.Green

        # --- Sticky red‑light handling ---
        # If we have an active TL for this vehicle, keep using it until it turns green or we pass the stop line.
        active_id = self.active_tl_id.get(ego_id)
        if active_id is not None:
            # Refresh references
            try:
                active_tl = self.world.get_actor(active_id)
            except Exception:
                active_tl = None
            keep = False
            if active_tl is not None:
                tl = active_tl
                tl_state = active_tl.get_state()
                stop_wp = self.active_stop_wp.get(ego_id, stop_wp)
                # Check whether stop line is still ahead
                if stop_wp is not None:
                    vtf = self.vehicle.get_transform()
                    fwd = vtf.get_forward_vector()
                    sl = stop_wp.transform.location
                    rel = carla.Vector3D(sl.x - vtf.location.x, sl.y - vtf.location.y, sl.z - vtf.location.z)
                    ahead = (rel.x * fwd.x + rel.y * fwd.y + rel.z * fwd.z) > 0.0
                    dx, dy, dz = rel.x, rel.y, rel.z
                    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                    keep = ahead and dist < 60.0
            if not keep or tl_state == carla.TrafficLightState.Green or tl_state == carla.TrafficLightState.Off:
                # Clear sticky TL
                self.active_tl_id.pop(ego_id, None)
                self.active_stop_wp.pop(ego_id, None)
                self.active_will_stop.pop(ego_id, None)
                active_id = None

        # If no sticky TL, and we found a TL ahead that is not green, arm it as active.
        if active_id is None and tl is not None and tl_state not in (carla.TrafficLightState.Green,
                                                                     carla.TrafficLightState.Off):
            self.active_tl_id[ego_id] = tl.id
            if stop_wp is not None:
                self.active_stop_wp[ego_id] = stop_wp
            # TM semantics: stop if get_running_light <= random()
            self.active_will_stop[ego_id] = (self.params.get_running_light(ego_id) <= random.random())

        # Resolve final hazard based on sticky decision
        will_stop = self.active_will_stop.get(ego_id, True)
        tl_hazard = False
        if self.active_tl_id.get(ego_id) is not None:
            # If we decided to stop for the active TL, keep stopping until it turns green or we pass
            if will_stop:
                tl_hazard = True

        # Non‑signalized handling (unchanged logic), but only if no active TL
        if not tl_hazard:
            current_junc_id = self.vehicle_last_junction.get(ego_id)
            if current_junc_id is not None:
                if affected is None or affected != current_junc_id:
                    self._remove_actor(ego_id)
                else:
                    return self._handle_non_signalised(ego_id, current_junc_id)

            if affected is not None and not at_tl and tl_state != carla.TrafficLightState.Green:
                if self.params.get_running_sign(ego_id) <= random.random():
                    self._add_actor_to_non_signalised(ego_id, affected)
                    return True

            # If we get here, tl_hazard is the decision for signals; non‑signalized otherwise unchanged
            return tl_hazard

        if at_tl and tl_state not in (carla.TrafficLightState.Green, carla.TrafficLightState.Off):
            if self.params.get_running_light(ego_id) <= random.random():
                if ego_id in self.vehicle_last_junction: self._remove_actor(ego_id)
                return True

        current_junc_id = self.vehicle_last_junction.get(ego_id)
        if current_junc_id is not None:
            if affected is None or affected != current_junc_id:
                self._remove_actor(ego_id)
            else:
                return self._handle_non_signalised(ego_id, current_junc_id)

        if affected is not None and not at_tl and tl_state != carla.TrafficLightState.Green:
            if self.params.get_running_sign(ego_id) <= random.random():
                self._add_actor_to_non_signalised(ego_id, affected)
                return True

        return False
