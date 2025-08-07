from dataclasses import dataclass
from typing import List, Optional

from dataclass_wizard import JSONWizard

from carla_data_classes.static import DataLane


@dataclass
class DataRoad:
    """
    DataClass to encapsulate a road with its lanes
    """

    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'

    road_id: int
    is_junction: bool
    lanes: List[DataLane]
    junction_id: Optional[int] = None
