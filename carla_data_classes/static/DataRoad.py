from dataclasses import dataclass
from typing import List

from carla_data_classes.static.DataLane import DataLane


@dataclass
class DataRoad:
    """
    DataClass to encapsulate a road with its lanes
    """
    road_id: int
    is_junction: bool
    lanes: List["DataLane"]
