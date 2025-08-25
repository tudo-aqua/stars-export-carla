# agent_localization.py
import math
from typing import List, Tuple, Optional

import carla

from .agent_constants import constants as C


class LocalizationBuffer:
    """
    Forward waypoint buffer and junction markers.
    Uses TM constants (horizon, MAX_START_DISTANCE, look-ahead, safe distance). :contentReference[oaicite:14]{index=14}
    """

    def __init__(self, world: carla.World, vehicle: carla.Vehicle):
        self.world = world
        self.map = world.get_map()
        self.vehicle = vehicle
        self.buffer: List[carla.Waypoint] = []
        self.junction_end_point: Optional[carla.Waypoint] = None
        self.safe_point_after_junction: Optional[carla.Waypoint] = None
        self.is_at_junction_entrance: bool = False

    @staticmethod
    def _dist(a: carla.Location, b: carla.Location) -> float:
        dx, dy, dz = a.x - b.x, a.y - b.y, a.z - b.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _ensure_front_wp(self):
        vloc = self.vehicle.get_location()
        wp = self.map.get_waypoint(vloc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp:
            self.buffer = [wp]

    def update(self) -> None:
        v = self.vehicle
        vloc = v.get_location()
        vel = v.get_velocity()
        spd = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

        # Speed-dependent horizon (TM constants)
        horizon_len = max(spd * C.PathBufferUpdate.HORIZON_RATE, C.PathBufferUpdate.MINIMUM_HORIZON_LENGTH)
        if spd > C.SpeedThreshold.HIGHWAY_SPEED:
            horizon_len = max(spd * C.PathBufferUpdate.HIGH_SPEED_HORIZON_RATE,
                              C.PathBufferUpdate.MINIMUM_HORIZON_LENGTH)

        if not self.buffer:
            self._ensure_front_wp()
        else:
            if self._dist(self.buffer[0].transform.location, vloc) > C.PathBufferUpdate.MAX_START_DISTANCE:
                self._ensure_front_wp()

        if not self.buffer:
            return

        # purge passed points using forward-frame dot test (DeviationDotProduct analog)
        fwd = v.get_transform().get_forward_vector()

        def ahead(loc: carla.Location) -> float:
            rel = carla.Vector3D(loc.x - vloc.x, loc.y - vloc.y, loc.z - vloc.z)
            return rel.x * fwd.x + rel.y * fwd.y + rel.z * fwd.z

        while len(self.buffer) > 1 and ahead(self.buffer[0].transform.location) <= 0.0:
            self.buffer.pop(0)

        # grow buffer to horizon
        acc = 0.0
        for i in range(1, len(self.buffer)):
            acc += self._dist(self.buffer[i - 1].transform.location, self.buffer[i].transform.location)
        while acc < horizon_len:
            nxts = self.buffer[-1].next(2.0)
            if not nxts: break
            nxt = nxts[0]
            acc += self._dist(self.buffer[-1].transform.location, nxt.transform.location)
            self.buffer.append(nxt)

        # junction & safe-point (ego-only analogue)
        self.is_at_junction_entrance = False
        self.junction_end_point = None
        self.safe_point_after_junction = None

        front_wp = self.buffer[0]
        la_wp, _ = self.get_target_waypoint(C.WaypointSelection.JUNCTION_LOOK_AHEAD)
        front_is_junc = bool(front_wp and front_wp.is_junction)
        la_is_junc = bool(la_wp and la_wp.is_junction)
        self.is_at_junction_entrance = (not front_is_junc) and la_is_junc

        if self.is_at_junction_entrance:
            cur = front_wp
            entered = False
            end_wp: Optional[carla.Waypoint] = None
            for _ in range(300):
                if not cur: break
                if not entered and cur.is_junction: entered = True
                if entered and not cur.is_junction:
                    end_wp = cur;
                    break
                nxts = cur.next(2.0);
                cur = nxts[0] if nxts else None

            if end_wp:
                safe = end_wp;
                dist = 0.0;
                cur = end_wp
                for _ in range(300):
                    nxts = cur.next(2.0)
                    if not nxts: break
                    nxt = nxts[0]
                    dist += self._dist(cur.transform.location, nxt.transform.location)
                    if dist >= C.WaypointSelection.SAFE_DISTANCE_AFTER_JUNCTION or len(nxts) > 1 or nxt.is_junction:
                        safe = nxt;
                        break
                    cur = nxt
                self.junction_end_point = end_wp
                self.safe_point_after_junction = safe

    def get_target_waypoint(self, distance: float) -> Tuple[carla.Waypoint, int]:
        if not self.buffer:
            self._ensure_front_wp()
        if not self.buffer:
            raise RuntimeError("Localization buffer empty")
        acc = 0.0
        for i in range(1, len(self.buffer)):
            acc += self._dist(self.buffer[i - 1].transform.location, self.buffer[i].transform.location)
            if acc >= distance:
                return self.buffer[i], i
        return self.buffer[-1], len(self.buffer) - 1
