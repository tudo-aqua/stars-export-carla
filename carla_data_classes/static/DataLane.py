from dataclasses import dataclass, field
from typing import List, Optional

from shapely import LineString

from carla_data_classes.enums.DataLaneType import DataLaneType
from carla_data_classes.static.DataContactArea import DataContactArea
from carla_data_classes.static.DataContactLaneInfo import DataContactLaneInfo
from carla_data_classes.static.DataLandmark import DataLandmark
from carla_data_classes.static.DataLaneMarking import DataLaneMarking
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
    left_lane_marking: Optional["DataLaneMarking"]
    right_lane_marking: Optional["DataLaneMarking"]
    left_lane: Optional["DataContactLaneInfo"]
    right_lane: Optional["DataContactLaneInfo"]
    # Other Driving lanes whose centerline runs within a small distance of this lane's for a
    # significant portion of its length — i.e. lanes that physically share the same road surface
    # for a stretch, such as a highway on-/off-ramp's acceleration/deceleration lane running
    # alongside the mainline lane. Populated by MapRasterizer.compute_lane_overlaps().
    overlapping_lanes: List["DataContactLaneInfo"] = field(default_factory=list)
    # "Merging" (this lane's centerline converges into an overlapping lane toward its end),
    # "Diverging" (splits away from an overlapping lane after its start), "Merging & Diverging"
    # (both, via different overlap partners), "Overlapping" (no clear directional trend), or ""
    # (no physical overlap detected).
    lane_topology: str = ""

    def get_linestring(self) -> LineString:
        """Return (and cache) a Shapely LineString for the lane."""
        if not hasattr(self, "_geom"):
            self._geom = LineString(
                [(m.location.x, m.location.y) for m in self.lane_midpoints]
            )
        return self._geom
