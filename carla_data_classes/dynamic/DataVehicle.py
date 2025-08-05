from dataclasses import dataclass

from carla import Vehicle
from dataclass_wizard import JSONWizard

# top of the file (before the @dataclass)
from carla_data_classes import ensure_core_types as _ensure_core_types
from carla_data_classes.dynamic.DataActor import DataActor
from carla_data_classes.static.DataVector3D import DataVector3D

_ensure_core_types(globals())


@dataclass
class DataVehicle(DataActor):
    """
    DataClass mapper to serialize carla.Vehicle objects
    """

    class _(JSONWizard.Meta):
        tag = "Vehicle"

    ego_vehicle: bool
    velocity: "DataVector3D"
    acceleration: "DataVector3D"
    forward_vector: "DataVector3D"
    angular_velocity: "DataVector3D"

    @staticmethod
    def from_vehicle(actor: Vehicle, ego_vehicle: bool = False) -> "DataVehicle":
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "Vehicle"
        return DataVehicle(
            **base,
            ego_vehicle=ego_vehicle,
            velocity=DataVector3D.from_vector3d(actor.get_velocity()),
            acceleration=DataVector3D.from_vector3d(actor.get_acceleration()),
            angular_velocity=DataVector3D.from_vector3d(actor.get_angular_velocity()),
            forward_vector=DataVector3D.from_vector3d(
                actor.get_transform().get_forward_vector()
            ),
        )
