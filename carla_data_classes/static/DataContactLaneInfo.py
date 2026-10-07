from dataclasses import dataclass


@dataclass
class DataContactLaneInfo:
    """
    DataClass wrapper to describe contact location with other lanes
    """
    road_id: int
    lane_id: int
