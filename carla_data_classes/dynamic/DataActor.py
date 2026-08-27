from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from carla import Actor
from dataclass_wizard import JSONWizard

from carla_data_classes.dynamic.DataBoundingBox import DataBoundingBox
from carla_data_classes.static.DataLaneMarkingContact import DataLaneMarkingContact
from carla_data_classes.static.DataLocation import DataLocation
from carla_data_classes.static.DataRotation import DataRotation


@dataclass
class DataActor(JSONWizard):
    """
    DataClass mapper to serialize carla.Actor objects
    """
    attributes: dict
    id: int
    type: str
    type_id: str
    is_alive: bool
    is_active: bool
    is_dormant: bool
    semantic_tags: list[int]
    bounding_box: Optional[DataBoundingBox]
    location: DataLocation
    rotation: DataRotation
    collisions: list[int]
    # Lane markings the actor's bounding box is touching this tick; empty when
    # the box lies within its lane. Populated by the monitor after construction
    # (see helpers.lane_marking_contacts), like ``collisions``.
    lane_marking_contacts: list[DataLaneMarkingContact]

    @staticmethod
    def from_actor(actor: Optional[Actor]) -> DataActor:
        """
        Build a new DataActor from a carla.Actor.

        Args:
            actor: The CARLA actor to convert. Must not be None.

        Returns:
            DataActor: A new data actor instance with copied values.

        Raises:
            ValueError: If actor is None
        """
        if actor is None:
            raise ValueError("Actor cannot be None")

        return DataActor(
            attributes=dict(actor.attributes),
            id=actor.id,
            type="Actor",
            type_id=actor.type_id,
            is_alive=bool(actor.is_alive),
            is_active=bool(actor.is_active),
            is_dormant=bool(actor.is_dormant),
            semantic_tags=list(actor.semantic_tags),
            bounding_box=DataBoundingBox.from_actor(actor),
            location=DataLocation.from_location(location=actor.get_location()),
            rotation=DataRotation.from_actor(actor),
            collisions=[],
            lane_marking_contacts=[]
        )
