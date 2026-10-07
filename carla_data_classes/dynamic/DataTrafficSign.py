from dataclasses import dataclass
from typing import Optional

from carla import TrafficSign
from dataclass_wizard import JSONWizard

# top of the file (before the @dataclass)
from carla_data_classes import ensure_core_types as _ensure_core_types
from carla_data_classes.dynamic.DataActor import DataActor
from carla_data_classes.enums.DataTrafficSignType import DataTrafficSignType

_ensure_core_types(globals())



@dataclass
class DataTrafficSign(DataActor):
    """
    DataClass mapper to serialize carla.TrafficSign objects
    """

    class _(JSONWizard.Meta):
        tag = "TrafficSign"

    traffic_sign_type: DataTrafficSignType
    speed_limit: Optional[float] = None

    @staticmethod
    def from_traffic_sign(actor: TrafficSign) -> "DataTrafficSign":
        base = DataActor.from_actor(actor)

        sign_type = DataTrafficSignType.UNKNOWN
        speed = None

        # parse the CARLA type_id, e.g. "traffic.speed_limit.30"
        parts = actor.type_id.split('.')
        if len(parts) >= 2:
            match parts[1]:
                case "speed_limit":
                    sign_type = DataTrafficSignType.MAX_SPEED
                    if len(parts) == 3:
                        speed = float(parts[2])
                case "stop":
                    sign_type = DataTrafficSignType.STOP
                case "yield":
                    sign_type = DataTrafficSignType.YIELD
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "TrafficSign"  # overwrite – no duplicate any more
        return DataTrafficSign(
            **base,  # ← now contains the final "type"
            traffic_sign_type=sign_type,
            speed_limit=speed,
        )
