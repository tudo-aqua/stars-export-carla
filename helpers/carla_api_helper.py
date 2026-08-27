import os
import re
from math import hypot
from typing import List, Optional

from carla import *

from carla_data_classes.dynamic import TickData, DataActor, DataVehicle, DataTrafficSign, DataPedestrian
from carla_data_classes.static import DataLocation, DataBlock, DataVector3D
from data_av_static import MapRasterizer
from helpers.json_helper import JSONHelper

ACTOR_BLOCK_FILTERING_SWITCH = True
USABLE_MAPS = ["Town01", "Town02", "Town10", "Town04", "Town06"]


class CarlaAPIHelper:
    """
    This class provides methods that return information of dynamic objects of the Carla simulation
    """

    def __init__(self, client: Client, world: World, rasterizer: MapRasterizer):
        super(CarlaAPIHelper, self).__init__()

        self.client = client
        self._world: World = world
        self._debug = world.debug
        self._map: Map = self._world.get_map()
        self._rasterizer = rasterizer

    @staticmethod
    def save_dynamic_data(ticks: List[TickData], file_path: os.path) -> bool:
        """
        Saves the ticks created for the current map to disk.
        """
        if ticks.__len__() == 0:
            # Something in the simulation has gone wrong. Log for later analysis
            JSONHelper.log_aborted_run("No Ticks in: " + file_path)
            return False
        JSONHelper.log_tick_data(ticks, file_path)
        return True

    def get_actors_in_block(self, block: DataBlock, remove_sensors: bool = True) -> List[Actor]:
        """
        Returns a list of all actors in the world
        :return: List of all actors
        """
        all_actors = list(self._world.get_actors())
        filtered_actors = []
        # Filter actors
        if ACTOR_BLOCK_FILTERING_SWITCH:
            for actor in all_actors:
                # Check if the current actor is in the given block
                if self._rasterizer.is_actor_in_block(actor=actor, block=block):
                    # Remove Sensors from the result list
                    if remove_sensors and isinstance(actor, Sensor):
                        continue
                    filtered_actors.append(actor)
        return filtered_actors

    def get_actors(self, remove_sensors: bool = True) -> List[Actor]:
        """
        Returns a list of all actors in the world
        :return: List of all actors
        """
        all_actors = list(self._world.get_actors())
        if not remove_sensors:
            return all_actors
        filtered_actors = []
        # Go through the actors
        for actor in all_actors:
            # Check if the Actor is a Sensor
            if isinstance(actor, Sensor):
                continue
            # Actor is no Sensor: Keep
            filtered_actors.append(actor)
        return filtered_actors

    def get_vehicles(self) -> List[Actor]:
        """
        Return the list of all Vehicles in the world
        :return: List of all Vehicles currently active in the world
        """
        # It's the first time asking for the traffic light
        # Get all actors of the world
        actors = self.get_actors()
        # Filter actors to get traffic lights only
        vehicles = list(filter(lambda actor: type(actor) is Vehicle, actors))
        return vehicles

    def start_replaying(self, replay_file_path, time_factor=1.0, show_file_info=False, start_time=None, duration=None,
                        camera_id=None):
        """
        This method starts the replay of the file under the given replay_file_path
        @param replay_file_path: The file path to the replay file
        @param time_factor: Time factor at which the replay should be replayed with
        @param show_file_info: Decide whether additional file information should be retrieved
        @param start_time: Declare a specific start time for the replay
        @param duration: Declare a specific duration for which the replay should last
        @param camera_id: Specify a specific camera by id that should be used
        """
        file = str(replay_file_path)
        if not start_time:
            start_time = 0.0
        if not duration:
            duration = 0.0
        if not camera_id:
            camera_id = 0
        self.client.replay_file(name=file, time_start=start_time, duration=duration, follow_id=camera_id,
                                replay_sensors=True)
        self.client.set_replayer_time_factor(time_factor)
        if show_file_info:
            self.client.show_recorder_file_info(file, True)

    @staticmethod
    def create_recorder_to_sim_id_map(world: World,
                                      info_text: str,
                                      actor_filters: tuple[str, ...] = ("vehicle.*",),
                                      position_tolerance_m: float = 5.0) -> dict[int, int]:
        """
        Build a mapping from *recorder* actor IDs (ground truth from the .log) to the
        *dynamic* CARLA actor IDs that exist in the currently replaying world.

        Notes:
          - Call this AFTER you started replaying the recording and advanced at least one tick,
            so that the actors from the recording exist in the world.
          - This version preserves and uses the actor *type* parsed from "Create <id>: <type> ..."
            lines (e.g., spectator, vehicle.*, walker.*, traffic light) and prefers 'at (...)'
            locations to avoid picking up rotation tuples.
          - Default actor_filters is vehicle-only on purpose. Traffic lights (and to a lesser
            extent walkers) share the same type_id/role_name and are often clustered within a
            few meters of each other, so the nearest-position fallback below can mis-assign them
            depending on the map's exact layout. Callers that compare len(mapping) against a
            vehicle count (e.g. carla_camera_recorder.py) will spuriously fail whenever a
            non-vehicle actor happens to match, so only widen actor_filters if you actually
            consume those extra mapped ids.
          - 'traffic light' is normalized to 'traffic.traffic_light'.

        Returns:
          dict[recorder_id] = runtime_actor.id
        """
        # --- 2) Parse recorder actors (id, type_id, role_name, approx location) ---
        recorded: dict[int, dict] = {}

        # Regexes
        # e.g. "Create 24: spectator (0) at (10828, 30786, 431)"
        create_rx = re.compile(
            r"Create\s+(\d+)\s*:\s*([A-Za-z0-9_.]+|traffic\s+light|spectator)",
            re.IGNORECASE
        )
        # Fallback id patterns (covers normalized "Id: 24", "Actor 24", etc.)
        id_rx = re.compile(r"(?:^|\s)(?:Actor\s*|Id\s*[=:]\s*)(\d+)\b", re.IGNORECASE)

        # Type patterns (now also accept bare labels like 'spectator' or 'traffic light')
        type_rx = re.compile(
            r"(vehicle\.[\w\.]+|walker\.[\w\.]+|sensor\.[\w\.]+|static\.[\w\.]+|traffic\.traffic_light|traffic\s+light|spectator)",
            re.IGNORECASE
        )
        role_rx = re.compile(r"role_name\s*[=:]\s*([^\s,)\]]+)", re.IGNORECASE)

        # CARLA prints very small values in scientific notation (e.g. "8.24296e-05"); a plain
        # "-?\d+(?:\.\d+)?" stops at the "e" and the whole match fails, silently dropping the
        # location for that actor.
        _FLOAT = r"-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?"

        # Prefer 'at (x, y, z)' to avoid matching rotation tuples
        at_loc_rx = re.compile(
            rf"at\s*\(\s*({_FLOAT})\s*,\s*({_FLOAT})\s*,\s*({_FLOAT})\s*\)",
            re.IGNORECASE
        )
        # Generic first tuple as a fallback
        loc_rx = re.compile(
            rf"\(\s*({_FLOAT})\s*,\s*({_FLOAT})\s*,\s*({_FLOAT})\s*\)"
        )

        def _norm_type(tok: str | None) -> str | None:
            if not tok:
                return None
            t = tok.strip().lower()
            if t == "traffic light":
                return "traffic.traffic_light"
            return t

        lines = info_text.splitlines()
        current_id: int | None = None
        for line in lines:
            # A) Preferred: "Create <id>: <type> ..."
            cm = create_rx.search(line)
            if cm:
                current_id = int(cm.group(1))
                t = _norm_type(cm.group(2))
                if current_id not in recorded:
                    recorded[current_id] = {"type_id": t, "role_name": None, "loc": None}
                else:
                    recorded[current_id]["type_id"] = recorded[current_id]["type_id"] or t

                # Prefer 'at (x, y, z)' on the same line
                am = at_loc_rx.search(line)
                if am and recorded[current_id]["loc"] is None:
                    try:
                        x, y, z = float(am.group(1)) / 100, float(am.group(2)) / 100, float(am.group(3)) / 100
                        recorded[current_id]["loc"] = (x, y, z)
                    except Exception:
                        pass
                # Try to grab role_name if present inline
                rm = role_rx.search(line)
                if rm and recorded[current_id]["role_name"] is None:
                    recorded[current_id]["role_name"] = rm.group(1)
                continue  # We've handled this line; go next

            # B) Fallback flows (normalized formats, multi-line blocks)
            id_m = id_rx.search(line)
            if id_m:
                current_id = int(id_m.group(1))
                if current_id not in recorded:
                    recorded[current_id] = {"type_id": None, "role_name": None, "loc": None}
            if current_id is None:
                continue

            # Type: accept dotted + bare tokens (spectator/traffic light)
            if recorded[current_id]["type_id"] is None:
                t = type_rx.search(line)
                if t:
                    recorded[current_id]["type_id"] = _norm_type(t.group(1))

            # Role name
            if recorded[current_id]["role_name"] is None:
                r = role_rx.search(line)
                if r:
                    recorded[current_id]["role_name"] = r.group(1)

            # Location: prefer 'at (...)', else first tuple
            if recorded[current_id]["loc"] is None:
                am = at_loc_rx.search(line)
                if am:
                    try:
                        x, y, z = float(am.group(1)) / 100, float(am.group(2)) / 100, float(am.group(3)) / 100
                        recorded[current_id]["loc"] = (x, y, z)
                    except Exception:
                        pass
                else:
                    p = loc_rx.search(line)
                    if p:
                        try:
                            x, y, z = float(p.group(1)) / 100, float(p.group(2)) / 100, float(p.group(3)) / 100
                            recorded[current_id]["loc"] = (x, y, z)
                        except Exception:
                            pass

        # --- 3) Collect current simulation actors we care about ---
        alist = world.get_actors()
        sim_actors = []
        for f in actor_filters:
            sim_actors.extend(alist.filter(f))

        sim_pool = [{
            "id": a.id,
            "type_id": getattr(a, "type_id", None),
            "role_name": (a.attributes.get("role_name") if hasattr(a, "attributes") else None),
            "loc": (lambda L: (L.x, L.y, L.z))(a.get_transform().location)
        } for a in sim_actors]

        # --- 4) Helper: distance on ground plane (x, y) ---
        def dist_xy(p, q) -> float:
            return hypot(p[0] - q[0], p[1] - q[1])

        # --- 5) Greedy matching: role_name+type_id; then type_id+nearest; then role_name only; then nearest ---
        mapping: dict[int, int] = {}
        used_sim_ids: set[int] = set()

        # A) role_name + type_id exact (tie-break by nearest position if multiple)
        for rid, rinfo in recorded.items():
            r_role = (rinfo["role_name"] or "").lower()
            r_type = (rinfo["type_id"] or "").lower()
            if not r_role and not r_type:
                continue

            candidates = [s for s in sim_pool
                          if s["id"] not in used_sim_ids
                          and (s["type_id"] or "").lower() == r_type
                          and (s["role_name"] or "").lower() == r_role]

            if not candidates:
                continue

            if len(candidates) == 1:
                # unique match ⇒ accept
                mapping[rid] = candidates[0]["id"]
                used_sim_ids.add(candidates[0]["id"])
                continue

            # multiple matches ⇒ if we have a recorded location, pick the nearest within tolerance
            r_loc = rinfo.get("loc")
            if r_loc is not None:
                best = min(candidates, key=lambda s: dist_xy(r_loc, s["loc"]))
                if dist_xy(r_loc, best["loc"]) <= position_tolerance_m:
                    mapping[rid] = best["id"]
                    used_sim_ids.add(best["id"])
                    continue
                # If we get here, we had multiple role+type matches but no location (or none within tolerance).
                # Leave unresolved for later stages (B/C/D) to handle.

        # B) type_id + nearest position (within tolerance)
        for rid, rinfo in recorded.items():
            if rid in mapping:
                continue
            r_type = (rinfo["type_id"] or "").lower()
            r_loc = rinfo["loc"]
            if not r_type or r_loc is None:
                continue
            candidates = [s for s in sim_pool
                          if s["id"] not in used_sim_ids
                          and (s["type_id"] or "").lower() == r_type]
            if not candidates:
                continue
            best = min(candidates, key=lambda s: dist_xy(r_loc, s["loc"]))
            if dist_xy(r_loc, best["loc"]) <= position_tolerance_m:
                mapping[rid] = best["id"]
                used_sim_ids.add(best["id"])

        # C) role_name only (unique)
        for rid, rinfo in recorded.items():
            if rid in mapping:
                continue
            r_role = (rinfo["role_name"] or "").lower()
            if not r_role:
                continue
            candidates = [s for s in sim_pool
                          if s["id"] not in used_sim_ids
                          and (s["role_name"] or "").lower() == r_role]
            if len(candidates) == 1:
                mapping[rid] = candidates[0]["id"]
                used_sim_ids.add(candidates[0]["id"])

        # D) final nearest (require being reasonably close)
        for rid, rinfo in recorded.items():
            if rid in mapping:
                continue
            r_loc = rinfo["loc"]
            if r_loc is None:
                continue
            candidates = [s for s in sim_pool if s["id"] not in used_sim_ids]
            if not candidates:
                continue
            best = min(candidates, key=lambda s: dist_xy(r_loc, s["loc"]))
            if dist_xy(r_loc, best["loc"]) <= position_tolerance_m:
                mapping[rid] = best["id"]
                used_sim_ids.add(best["id"])

        return mapping

    # region Static methods
    ########################################
    #         static methods               #
    ########################################

    @staticmethod
    def get_usable_maps(client: Client) -> List[str]:
        available_maps = client.get_available_maps()
        usable_maps = []
        for map in available_maps:
            if "_Opt" in map:
                continue
            for usable_map in USABLE_MAPS:
                if usable_map in map:
                    usable_maps.append(map)
        return usable_maps

    @staticmethod
    def get_data_actor_from_actor(
            actor: Actor,
            ego_vehicle: bool = False,
            velocity: Optional["DataVector3D"] = None,
            angular_velocity: Optional["DataVector3D"] = None,
    ) -> Optional[DataActor]:
        """
        Returns the filled DataActor from the carla Actor
        :param actor: The actor which should be transformed into the DataActor class
        :param velocity: Linear velocity to use instead of the (during replay, always-zero)
            live actor.get_velocity(); see helpers.kinematics.compute_recorded_velocities.
        :param angular_velocity: Same as velocity, for actor.get_angular_velocity().
        :return: Filled DataActor object
        """
        data_actor: Optional[DataActor] = None
        # Check of which type the given actor is and transform it into the correct dataclass
        if type(actor) is Vehicle:
            data_actor = DataVehicle.from_vehicle(actor, ego_vehicle, velocity, angular_velocity)
        elif type(actor) is TrafficSign:
            data_actor = DataTrafficSign.from_traffic_sign(actor)
        elif type(actor) is TrafficLight:
            data_actor = None
        elif type(actor) is Walker:
            data_actor = DataPedestrian.from_walker(actor, velocity, angular_velocity)
        else:
            if actor.type_id == "spectator":
                return None
            elif "pedestrian" in actor.type_id:
                data_actor = DataPedestrian.from_walker(actor, velocity, angular_velocity)
            # TODO: If an actor of another type is tracked
        if data_actor:
            data_actor.location = DataLocation.from_location(location=actor.get_location())
        return data_actor
    # endregion
