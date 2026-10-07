from dataclasses import dataclass
from typing import TYPE_CHECKING

from carla_data_classes.static.DataLocation import DataLocation

if TYPE_CHECKING:
    from carla_data_classes.static.DataLane import DataLane

CONTACT_AREA_MARGIN: float = 3.0


@dataclass
class DataContactArea:
    """
    A contact area is spanned from a contact point of two crossing lanes.
    Based from this contact point the CONTACT_AREA_MARGIN is moved to the
    start and end of the lane.
    Therefore, 4 important points are created:
    contact_location = lane_1_start_pos + CONTACT_AREA_MARGIN
    contact_location = lane_1_end_pos - CONTACT_AREA_MARGIN
    contact_location = lane_2_start_pos + CONTACT_AREA_MARGIN
    contact_location = lane_2_end_pos - CONTACT_AREA_MARGIN
    """
    id: str  # combination of the lane and road ids of the given two lanes
    contact_location: "DataLocation"
    lane_1_road_id: int
    lane_1_id: int
    lane_1_start_pos: float
    lane_1_end_pos: float

    lane_2_road_id: int
    lane_2_id: int
    lane_2_start_pos: float
    lane_2_end_pos: float

    @staticmethod
    def from_lanes(contact_location: "DataLocation", lane_1: "DataLane", start_pos_lane_1: float, lane_2: "DataLane",
                   start_pos_lane_2: float) -> "DataContactArea":
        # Check if the lanes have to be switched
        if lane_2.road_id < lane_1.road_id:
            # Order the lanes such that the smaller road id is stored in lane_1
            save = lane_1
            lane_1 = lane_2
            lane_2 = save
            save = start_pos_lane_1
            start_pos_lane_1 = start_pos_lane_2
            start_pos_lane_2 = save
        # Build id from the lane_1 and lane_2 road and lane ids
        contact_area_id = f"{lane_1.road_id}_{lane_1.lane_id}+{lane_2.road_id}_{lane_2.lane_id}"
        contact_location = contact_location

        # Build critical section for lane_1
        lane_1_road_id = lane_1.road_id
        lane_1_id = lane_1.lane_id
        # Move CONTACT_AREA_MARGIN to start and end of lane
        # Also includes if the start of the lane is reached
        lane_1_start_pos = float(max(0.0, start_pos_lane_1 - CONTACT_AREA_MARGIN))
        # Also includes if the end of the lane is reached
        lane_1_end_pos = float(min(lane_1.lane_length, start_pos_lane_1 + CONTACT_AREA_MARGIN))

        # Build critical section for lane_2
        lane_2_road_id = lane_2.road_id
        lane_2_id = lane_2.lane_id
        # Move CONTACT_AREA_MARGIN to start and end of lane
        # Also includes if the start of the lane is reached
        lane_2_start_pos = float(max(0.0, start_pos_lane_2 - CONTACT_AREA_MARGIN))
        # Also includes if the end of the lane is reached
        lane_2_end_pos = float(min(lane_2.lane_length, start_pos_lane_2 + CONTACT_AREA_MARGIN))
        return DataContactArea(id=contact_area_id, contact_location=contact_location, lane_1_road_id=lane_1_road_id,
                               lane_1_id=lane_1_id, lane_1_start_pos=lane_1_start_pos, lane_1_end_pos=lane_1_end_pos,
                               lane_2_road_id=lane_2_road_id, lane_2_id=lane_2_id, lane_2_start_pos=lane_2_start_pos,
                               lane_2_end_pos=lane_2_end_pos)
