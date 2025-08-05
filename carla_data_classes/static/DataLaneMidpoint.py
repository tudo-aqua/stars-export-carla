from dataclasses import dataclass

from carla_data_classes.static.DataLocation import DataLocation
from carla_data_classes.static.DataRotation import DataRotation


@dataclass
class DataLaneMidpoint:
    """
    DataClass to wrap waypoint locations for a given lane. Each LaneMidpoint is in the middle of the lane
    has a distance to the start of the lane and its location
    """
    lane_id: int
    road_id: int
    distance_to_start: float
    location: "DataLocation"
    rotation: "DataRotation"
