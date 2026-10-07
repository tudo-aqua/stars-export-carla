from dataclasses import dataclass
from typing import List

from dataclass_wizard import JSONWizard

from carla_data_classes.static import DataRoad
from carla_data_classes.static.DataCrosswalk import DataCrosswalk
from carla_data_classes.static.DataJunction import DataJunction


@dataclass
class DataWorld(JSONWizard):
    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'
        recursive_classes = True

    straights: List[DataRoad]
    junctions: List[DataJunction]
    crosswalks: List[DataCrosswalk]

    def get_all_lanes(self) -> List["DataLane"]:
        lanes = []
        for road in self.get_all_roads():
            lanes.extend(road.lanes)
        return lanes

    def get_all_roads(self) -> List["DataRoad"]:
        roads = []
        for junction in self.junctions:
            roads.extend(junction.roads)
        for straight in self.straights:
            roads.append(straight)
        return roads
