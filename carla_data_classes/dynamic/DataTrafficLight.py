from dataclasses import dataclass
from typing import Optional

from carla import TrafficLight
from dataclass_wizard import JSONWizard

# top of the file (before the @dataclass)
from carla_data_classes import ensure_core_types as _ensure_core_types
from carla_data_classes.dynamic.DataActor import DataActor
from carla_data_classes.static.DataLocation import DataLocation
from carla_data_classes.static.DataRotation import DataRotation
from carla_data_classes.static.DataStaticTrafficLight import DataStaticTrafficLight

_ensure_core_types(globals())


@dataclass
class DataTrafficLight(DataActor):
    """
    DataClass mapper to serialize carla.TrafficLight objects.
    This dataclass contains the dynamic data for a TrafficLight
    in the carla simulation
    """

    class _(JSONWizard.Meta):
        tag = "TrafficLight"

    state: int  # TODO convert to enum
    related_open_drive_id: int

    @staticmethod
    def from_traffic_light(
            actor: Optional[TrafficLight],
            static_tl: DataStaticTrafficLight,
    ) -> "DataTrafficLight":
        """
        Build a *new* DataTrafficLight from a live TrafficLight actor
        and its static counterpart.
        """
        if actor is None:
            # synthetic “off‑world” traffic light
            return DataTrafficLight(
                attributes={},
                id=-1,
                type="TrafficLight",
                type_id="traffic.traffic_light",
                is_alive=False,
                is_active=False,
                is_dormant=False,
                semantic_tags=[],
                bounding_box=None,
                location=DataLocation(-1, -1, -1),
                rotation=DataRotation(-1, -1, -1),
                state=4,  # unknown
                related_open_drive_id=static_tl.open_drive_id,
            )

        # live traffic‑light → base fields
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "TrafficLight"
        return DataTrafficLight(
            **base,
            state=int(actor.state),
            related_open_drive_id=static_tl.open_drive_id,
        )
