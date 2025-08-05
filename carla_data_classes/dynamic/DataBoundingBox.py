from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, List

from carla import Actor, BoundingBox
from dataclass_wizard import JSONWizard

from carla_data_classes.static.DataLocation import DataLocation
from carla_data_classes.static.DataRotation import DataRotation
from carla_data_classes.static.DataVector3D import DataVector3D


@dataclass
class DataBoundingBox(JSONWizard):
    """
    DataClass mapper to serialize bounding box information from carla.Actor objects
    """
    extent: DataVector3D
    location: DataLocation
    rotation: DataRotation
    vertices: List[DataLocation]

    @staticmethod
    def from_actor(actor: Optional[Actor]) -> Optional[DataBoundingBox]:
        """
        Build a new DataBoundingBox from a carla.Actor.

        Args:
            actor: The CARLA actor to convert.

        Returns:
            DataBoundingBox: A new data bounding box instance with copied values,
                           or None if the actor has no bounding box.
        """
        if actor is None or not hasattr(actor, 'bounding_box'):
            return None

        bounding_box: BoundingBox = actor.bounding_box

        return DataBoundingBox(
            extent=DataVector3D.from_bounding_box(bounding_box),
            location=DataLocation.from_bounding_box(bounding_box),
            rotation=DataRotation.from_bounding_box(bounding_box),
            vertices=list(map(
                lambda x: DataLocation.from_location(x),
                actor.bounding_box.get_world_vertices(actor.get_transform())
            ))
        )
