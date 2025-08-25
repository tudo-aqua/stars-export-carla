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
        affected = self._get_affected_junction(loc)
        tl = self.vehicle.get_traffic_light()
        at_tl = bool(tl) and self.vehicle.is_at_traffic_light()
        if tl is None and affected is not None:
            tl = self._find_controlling_tl_ahead(loc)
        tl_state = (tl.get_state() if tl is not None else carla.TrafficLightState.Green)

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
