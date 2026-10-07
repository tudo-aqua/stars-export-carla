from __future__ import annotations

from typing import List

import carla

from carla_data_classes.static.DataLaneMarking import DataLaneMarking
from carla_data_classes.static.DataLaneMarkingContact import DataLaneMarkingContact


def _lateral_offsets(vertices, origin: carla.Location, right: carla.Vector3D) -> List[float]:
    """
    Signed lateral distance (metres) of each vertex from the lane centre-line.

    Positive points towards the lane's right-hand side (``right`` is the waypoint's
    unit right vector). ``z`` is ignored, so the roof and floor corners of the
    bounding box collapse onto the same lateral value.
    """
    return [
        (v.x - origin.x) * right.x + (v.y - origin.y) * right.y
        for v in vertices
    ]


def compute_lane_marking_contacts(
        actor: carla.Actor,
        carla_map: carla.Map,
        *,
        touch_epsilon: float = 0.0,
) -> List[DataLaneMarkingContact]:
    """
    Return the lane markings that ``actor``'s world-space bounding box is touching.

    Method ("corners vs lane-center offset"):
      * take the waypoint on the lane the actor centre sits on (any lane type -
        driving, parking, shoulder, ...) as the reference frame,
      * project all 8 world bounding box vertices onto that waypoint's right
        vector to get their signed lateral offset from the lane centre-line,
      * a marking on a given side spans the band
        ``[half_width - w/2, half_width + w/2]`` from the centre-line; the box
        *touches* it once the outermost vertex on that side reaches past the
        inner edge, and *crosses* it once that vertex reaches past the outer edge.

    An empty result means the bounding box lies within its lane's markings.
    Sides where CARLA reports no marking (type NONE) are never emitted.
    """
    bounding_box = getattr(actor, "bounding_box", None)
    if bounding_box is None:
        return []

    transform = actor.get_transform()
    try:
        vertices = bounding_box.get_world_vertices(transform)
    except RuntimeError:
        return []

    waypoint = carla_map.get_waypoint(
        transform.location, project_to_road=True, lane_type=carla.LaneType.Any
    )
    if waypoint is None or waypoint.lane_width <= 0.0:
        return []

    right = waypoint.transform.get_right_vector()
    offsets = _lateral_offsets(vertices, waypoint.transform.location, right)
    half_width = waypoint.lane_width / 2.0

    # Outermost reach of the box on each side, as a positive distance from centre.
    reach_by_side = {
        "Right": max(offsets),
        "Left": -min(offsets),
    }
    marking_by_side = {
        "Right": waypoint.right_lane_marking,
        "Left": waypoint.left_lane_marking,
    }

    contacts: List[DataLaneMarkingContact] = []
    for side, reach in reach_by_side.items():
        carla_marking = marking_by_side[side]
        data_marking = DataLaneMarking.from_lane_marking(carla_marking)
        if data_marking is None:
            # No real marking on this side (type NONE) - nothing to touch.
            continue

        inner_edge = half_width - data_marking.width / 2.0
        outer_edge = half_width + data_marking.width / 2.0
        if reach < inner_edge - touch_epsilon:
            continue

        contacts.append(DataLaneMarkingContact(
            side=side,
            road_id=waypoint.road_id,
            lane_id=waypoint.lane_id,
            marking=data_marking,
            is_crossing=reach >= outer_edge,
            penetration=round(reach - inner_edge, 4),
        ))
    return contacts
