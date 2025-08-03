from typing import TYPE_CHECKING, List

from carla import Landmark, TrafficLight

from carla_data_classes import DataStaticTrafficLight, DataLocation, DataRotation

if TYPE_CHECKING:
    from .rasterizer import MapRasterizer


class _TrafficLightUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    def get_all_traffic_lights(self) -> List[DataStaticTrafficLight]:
        roads = self.ctx.flatten(list(map(lambda b: b.roads, self.ctx.blocks)))
        lanes = self.ctx.flatten(list(map(lambda r: r.lanes, roads)))
        return self.ctx.flatten(list(map(lambda l: l.traffic_lights, lanes)))

    @staticmethod
    def get_data_static_traffic_light_for_traffic_light(static_traffic_light: Landmark,
                                                        traffic_light: TrafficLight) -> DataStaticTrafficLight:
        """
        Returns the DataStaticTrafficLight object based on the given traffic lights.

        Args:
            static_traffic_light (Landmark): The landmark object containing location and ID information
            traffic_light (TrafficLight): Optional CARLA traffic light containing dynamic state information

        Returns:
            DataStaticTrafficLight: Contains the static properties of the traffic light
        """
        location = DataLocation.from_location(static_traffic_light.transform.location)
        rotation = DataRotation.from_rotation(static_traffic_light.transform.rotation)
        if traffic_light is not None:
            stop_locations = list(
                map(lambda waypoint: DataLocation.from_waypoint(waypoint), traffic_light.get_stop_waypoints()))
        else:
            stop_locations = []
        return DataStaticTrafficLight(open_drive_id=static_traffic_light.id, location=location, rotation=rotation,
                                      stop_locations=stop_locations, position_distance=static_traffic_light.s)
