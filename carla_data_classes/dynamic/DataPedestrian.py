from dataclasses import dataclass
from typing import Optional

from carla import Walker
from dataclass_wizard import JSONWizard

# top of the file (before the @dataclass)
from carla_data_classes import ensure_core_types as _ensure_core_types
from carla_data_classes.dynamic.DataActor import DataActor
from carla_data_classes.static.DataVector3D import DataVector3D

_ensure_core_types(globals())


@dataclass
class DataPedestrian(DataActor):
    """
    DataClass mapper to serialize carla.Pedestrian objects
    """

    class _(JSONWizard.Meta):
        tag = "Pedestrian"

    velocity: "DataVector3D"
    acceleration: "DataVector3D"
    angular_velocity: "DataVector3D"

    @staticmethod
    def from_walker(
            actor: Walker,
            velocity: Optional["DataVector3D"] = None,
            angular_velocity: Optional["DataVector3D"] = None,
    ) -> "DataPedestrian":
        """
        velocity/angular_velocity should be supplied by the caller during a log replay
        (see helpers.kinematics.compute_recorded_velocities), since a replayed actor's own
        get_velocity()/get_angular_velocity() always report 0 - the replayer teleports
        actors to recorded transforms instead of driving them through physics. When not
        supplied (e.g. inspecting a live, non-replayed actor) this falls back to the live
        query, which is accurate outside of replay.

        Acceleration is always taken from the live query here; during replay that's 0 for
        the same reason velocity is, but that's fine - helpers.kinematics.
        compute_acceleration_for_ticks overwrites it afterward once real velocities are in.
        """
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "Pedestrian"
        return DataPedestrian(
            **base,
            velocity=velocity if velocity is not None else DataVector3D.from_vector3d(actor.get_velocity()),
            acceleration=DataVector3D.from_vector3d(actor.get_acceleration()),
            angular_velocity=angular_velocity if angular_velocity is not None
            else DataVector3D.from_vector3d(actor.get_angular_velocity()),
        )
