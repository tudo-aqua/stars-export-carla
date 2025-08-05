from dataclasses import dataclass
from typing import List

from dataclass_wizard import JSONWizard

from carla_data_classes.dynamic.DataActorPosition import DataActorPosition
from carla_data_classes.dynamic.DataWeatherParameters import DataWeatherParameters


@dataclass
class TickData(JSONWizard):
    """
    DataClass to encapsulate ticks with its actors and their positions
    """

    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'
        tag_key = "type"  # look at the “type” field in JSON
        auto_assign_tags = False  # we supplied the tag values ourselves

    current_tick: float
    actor_positions: List["DataActorPosition"]
    weather_parameters: "DataWeatherParameters"
