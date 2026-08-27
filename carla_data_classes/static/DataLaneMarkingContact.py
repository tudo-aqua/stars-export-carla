from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from carla_data_classes.static.DataLaneMarking import DataLaneMarking


@dataclass
class DataLaneMarkingContact:
    """
    Describes a single lane marking that an actor's world-space bounding box is
    currently touching (or has crossed).

    A contact is emitted when the outermost bounding box corner on one side of
    the actor reaches laterally past the inner edge of a lane marking band -
    measured in the reference frame of the waypoint underneath the actor
    (see helpers.lane_marking_contacts.compute_lane_marking_contacts).

    An empty ``lane_marking_contacts`` list on the actor therefore means the
    bounding box lies within its lane's markings.
    """
    # "Left" or "Right" - side of the reference lane, in driving direction
    side: str
    # Reference lane the contact is measured against
    road_id: int
    lane_id: int
    # The touched marking (type / color / width). Never None for an emitted
    # contact, but kept optional to mirror DataLaneMarking.from_lane_marking.
    marking: Optional[DataLaneMarking]
    # True when the box reaches past the *outer* edge of the marking band, i.e.
    # it is not merely touching the marking but overlapping the neighbour lane.
    is_crossing: bool
    # How far (metres) the box extends past the *inner* edge of the marking band.
    penetration: float
