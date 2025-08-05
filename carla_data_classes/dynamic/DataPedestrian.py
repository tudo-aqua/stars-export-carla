from dataclasses import dataclass

from carla import Walker
from dataclass_wizard import JSONWizard

# top of the file (before the @dataclass)
from carla_data_classes import ensure_core_types as _ensure_core_types
from carla_data_classes.dynamic.DataActor import DataActor

_ensure_core_types(globals())



@dataclass
class DataPedestrian(DataActor):
    """
    DataClass mapper to serialize carla.Pedestrian objects
    """

    class _(JSONWizard.Meta):
        tag = "Pedestrian"

    @staticmethod
    def from_walker(actor: Walker) -> "DataPedestrian":
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "Pedestrian"
        return DataPedestrian(
            **base,
            type_id=actor.type_id,
        )

    type_id: str
