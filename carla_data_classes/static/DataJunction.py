from dataclasses import dataclass
from typing import List

from dataclass_wizard import JSONWizard

from carla_data_classes.static import DataRoad


@dataclass
class DataJunction(JSONWizard):
    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'

    junction_id: int
    roads: List[DataRoad]
