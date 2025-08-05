from dataclasses import dataclass
from typing import Union

from dataclass_wizard import JSONWizard

from carla_data_classes.dynamic.DataActor import DataActor
from carla_data_classes.dynamic.DataPedestrian import DataPedestrian
from carla_data_classes.dynamic.DataTrafficLight import DataTrafficLight
from carla_data_classes.dynamic.DataTrafficSign import DataTrafficSign
from carla_data_classes.dynamic.DataVehicle import DataVehicle

ActorT = Union[
    DataTrafficLight,
    DataVehicle,
    DataPedestrian,
    DataTrafficSign,
    DataActor  # fallback if nothing matches
]


@dataclass
class DataActorPosition(JSONWizard):
    """
    DataClass to wrap the position of actors, including the lane and road id
    """
    position_on_lane: float
    road_id: int
    lane_id: int
    actor: ActorT
