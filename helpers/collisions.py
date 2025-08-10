from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional

import carla

from carla_data_classes.dynamic.DataCollision import DataCollision
from carla_data_classes.enums.DataCollisionKind import DataCollisionKind


def _kind_from_type_id(type_id: Optional[str]) -> DataCollisionKind:
    if not type_id:
        return DataCollisionKind.OTHER
    t = type_id.lower()
    if t.startswith("vehicle."):
        return DataCollisionKind.VEHICLE
    if t.startswith("walker."):
        return DataCollisionKind.WALKER
    if t.startswith("traffic."):
        return DataCollisionKind.TRAFFIC
    if t == "world":
        return DataCollisionKind.STATIC
    return DataCollisionKind.OTHER


@dataclass
class _ParentInfo:
    parent_id: int
    parent_type: str


class CollisionCollector:
    """
    Subscribes to all existing `sensor.other.collision` sensors.
    Buffers events per frame; exposes `consume_frame(frame)` which returns:
        { runtime_actor_id : [DataCollision, ...] }  for that exact frame.
    """

    def __init__(self, world: carla.World, debug: bool = False):
        self.world = world
        self.debug = debug
        self._sensors: List[carla.Sensor] = []
        self._parent_info: Dict[int, _ParentInfo] = {}  # sensor_id -> _ParentInfo
        self._parent_type: Dict[int, str] = {}  # parent_actor_id -> type_id
        self._events_by_frame: Dict[int, List[DataCollision]] = defaultdict(list)

    # ---------- setup / teardown ----------

    def attach_existing_collision_sensors(self) -> int:
        """
        Find all `sensor.other.collision` sensors already in the world and listen to them.
        Returns the number of sensors subscribed.
        """
        sensors = self.world.get_actors().filter("sensor.other.collision")
        count = 0
        print(f">> [CARLA] Attaching '{len(sensors)}' existing collision sensors.")
        for s in sensors:
            try:
                parent = s.get_parent()
            except Exception:
                parent = getattr(s, "parent", None)

            if parent is None:
                # orphan sensor (shouldn't happen with recorder), skip
                continue

            pid = parent.id
            ptype = (parent.type_id or "unknown")
            self._parent_info[s.id] = _ParentInfo(parent_id=pid, parent_type=ptype)
            self._parent_type[pid] = ptype

            # Bind callback with defaults so we don't capture 's' late
            s.listen(lambda ev, _pid=pid, _ptype=ptype: self._on_event(ev, _pid, _ptype))
            self._sensors.append(s)
            count += 1

        if self.debug:
            print(f"[collision] subscribed to {count} collision sensors")
        return count

    def ensure_vehicle_sensors(self, vehicles: List[carla.Actor]) -> int:
        """
        Optional safety net: if the replay didn’t spawn collision sensors,
        attach one to each provided vehicle.
        Returns the number of sensors created.
        """
        bp_lib = self.world.get_blueprint_library()
        coll_bp = bp_lib.find("sensor.other.collision")
        made = 0
        for v in vehicles:
            # skip if we already see some sensor parented to this vehicle
            if v.id in self._parent_type:
                continue
            try:
                sensor = self.world.spawn_actor(coll_bp, carla.Transform(), attach_to=v)
            except RuntimeError:
                # sometimes spawn can fail during replay start-up, skip
                continue
            ptype = (v.type_id or "unknown")
            self._parent_info[sensor.id] = _ParentInfo(parent_id=v.id, parent_type=ptype)
            self._parent_type[v.id] = ptype
            sensor.listen(lambda ev, _pid=v.id, _ptype=ptype: self._on_event(ev, _pid, _ptype))
            self._sensors.append(sensor)
            made += 1
        if self.debug:
            print(f"[collision] created {made} fallback sensors for vehicles")
        return made

    def stop(self):
        """Stop all listeners (do not destroy; your cleanup will kill actors at the end)."""
        for s in self._sensors:
            try:
                s.stop()
            except Exception:
                pass

    # ---------- consumption API ----------

    def consume_frame(self, frame: int) -> Dict[int, List[DataCollision]]:
        """
        Return collisions bucketed by ACTOR for this exact simulation frame,
        and remove them from the internal buffer.
        """
        events = self._events_by_frame.pop(frame, [])
        per_actor: Dict[int, List[DataCollision]] = defaultdict(list)
        for dc in events:
            # attach to both sides (so “other” actor also gets the event even if it had no sensor)
            if dc.actor1_id != -1:
                per_actor[dc.actor1_id].append(dc)
            if dc.actor2_id != -1:
                per_actor[dc.actor2_id].append(dc)
        return per_actor

    # ---------- internal: callback ----------

    def _on_event(self, ev: carla.CollisionEvent, parent_id: int, parent_type: str):
        """
        Build a DataCollision from a CollisionEvent. Add it to that frame's buffer.
        """
        # Other actor may be static “world” or a real actor
        other = getattr(ev, "other_actor", None)
        if other is not None:
            other_id = other.id
            other_type = other.type_id or "unknown"
            other_kind = _kind_from_type_id(other_type)
        else:
            other_id = -1
            other_type = "WORLD"
            other_kind = DataCollisionKind.STATIC

        coll = DataCollision(
            actor1_kind=_kind_from_type_id(parent_type),
            actor2_kind=other_kind,
            actor1_id=parent_id,
            actor1_type_id=parent_type,
            actor2_id=other_id,
            actor2_type_id=other_type,
        )
        self._events_by_frame[ev.frame].append(coll)
