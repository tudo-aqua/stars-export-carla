from typing import TYPE_CHECKING, List, Optional

from carla import Landmark, TrafficLight

from carla_data_classes.dynamic import DataBlock
from carla_data_classes.static import DataStaticTrafficLight, DataLandmark, DataLocation

if TYPE_CHECKING:
    from .rasterizer import MapRasterizer


class _TrafficLightUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    def get_all_traffic_lights(self) -> List[DataStaticTrafficLight]:
        roads = self.ctx.flatten(list(map(lambda b: b.roads, self.ctx.blocks)))
        lanes = self.ctx.flatten(list(map(lambda r: r.lanes, roads)))
        return self.ctx.flatten(list(map(lambda l: l.traffic_lights, lanes)))

    def update_static_traffic_lights_from_landmarks(self, blocks: List[DataBlock]) -> None:
        """
        Rebuild lane.traffic_lights from the lane-attached landmarks.
        Must be called AFTER add_landmarks_to_lanes().
        """
        # get the CARLA world object depending on your architecture
        world = self.ctx.world

        for block in blocks:
            for road in block.roads:
                for lane in road.lanes:
                    # (Re)build the static list from the attached landmarks
                    new_statics: List[DataStaticTrafficLight] = []
                    for lm in (lane.landmarks or []):
                        if not self._is_light_landmark(lm):
                            continue
                        tl_actor = self._try_get_tl_actor(world, lm) if world else None
                        # Use a converter that accepts your DataLandmark
                        static_tl = self.get_data_static_traffic_light_for_traffic_light(lm, tl_actor)
                        new_statics.append(static_tl)
                    lane.traffic_lights = new_statics

    @staticmethod
    def _is_light_landmark(lm: DataLandmark) -> bool:
        # robust test; accepts numeric or enum-like values
        t = str(getattr(lm, "type", ""))
        return t == "1000001" or "TrafficLight" in t or "Light" in t

    @staticmethod
    def _try_get_tl_actor(world, lm: DataLandmark) -> Optional[TrafficLight]:
        """
        Best-effort: try to fetch the dynamic TrafficLight actor for extra info (e.g., stop locations).
        Safe to return None (converter should handle it).
        """
        try:
            # prefer explicit OpenDRIVE id if you store it separately
            od = str(getattr(lm, "open_drive_id", getattr(lm, "id", "")))
            if od:
                return world.get_traffic_light_from_opendrive_id(od)
        except Exception:
            pass
        return None

    @staticmethod
    def get_data_static_traffic_light_for_traffic_light(static_traffic_light: DataLandmark,
                                                        traffic_light: TrafficLight) -> DataStaticTrafficLight:
        """
        Returns the DataStaticTrafficLight object based on the given traffic lights.

        Args:
            static_traffic_light (Landmark): The landmark object containing location and ID information
            traffic_light (TrafficLight): Optional CARLA traffic light containing dynamic state information

        Returns:
            DataStaticTrafficLight: Contains the static properties of the traffic light
        """
        location = static_traffic_light.location
        rotation = static_traffic_light.rotation
        if traffic_light is not None:
            stop_locations = list(
                map(lambda waypoint: DataLocation.from_waypoint(waypoint), traffic_light.get_stop_waypoints()))
        else:
            stop_locations = []
        return DataStaticTrafficLight(open_drive_id=static_traffic_light.id, location=location, rotation=rotation,
                                      stop_locations=stop_locations, position_distance=static_traffic_light.s)
