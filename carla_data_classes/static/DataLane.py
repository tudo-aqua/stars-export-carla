from dataclasses import dataclass
from typing import List

from shapely import LineString

from carla_data_classes.enums.DataLaneType import DataLaneType
from carla_data_classes.static.DataContactArea import DataContactArea
from carla_data_classes.static.DataContactLaneInfo import DataContactLaneInfo
from carla_data_classes.static.DataLandmark import DataLandmark
from carla_data_classes.static.DataLaneMidpoint import DataLaneMidpoint
from carla_data_classes.static.DataSpeedLimit import DataSpeedLimit
from carla_data_classes.static.DataStaticTrafficLight import DataStaticTrafficLight


@dataclass
class DataLane:
    """
    DataClass mapper to serialize carla.Lane objects and additional information
    """
    road_id: int
    lane_id: int
    lane_type: "DataLaneType"
    lane_width: float
    lane_length: float
    s: float
    predecessor_lanes: List["DataContactLaneInfo"]
    successor_lanes: List["DataContactLaneInfo"]
    intersecting_lanes: List["DataContactLaneInfo"]
    lane_midpoints: List["DataLaneMidpoint"]
    speed_limits: List["DataSpeedLimit"]
    landmarks: List["DataLandmark"]
    contact_areas: List["DataContactArea"]
    traffic_lights: List["DataStaticTrafficLight"]

    def get_linestring(self) -> LineString:
        """Return (and cache) a Shapely LineString for the lane."""
        if not hasattr(self, "_geom"):
            self._geom = LineString(
                [(m.location.x, m.location.y) for m in self.lane_midpoints]
            )
        return self._geom
