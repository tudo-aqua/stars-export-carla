from dataclasses import dataclass
from typing import List

from carla_data_classes.static.DataLocation import DataLocation


@dataclass
class DataCrosswalk:
    """
    Polygonal footprint of a crosswalk as returned by CARLA's Map API.
    `vertices` are in world coordinates, ordered around the polygon.
    """
    crosswalk_id: int
    vertices: List[DataLocation]
