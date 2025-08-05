from dataclasses import dataclass
from typing import List

from dataclass_wizard import JSONWizard

from carla_data_classes.static.DataRoad import DataRoad


@dataclass
class DataBlock(JSONWizard):
    """
    DataClass to encapsulate a block with its roads
    """

    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'

    id: str
    roads: List["DataRoad"]
