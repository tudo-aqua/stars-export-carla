from dataclasses import dataclass
from typing import Optional

from carla import LaneMarking

from carla_data_classes.enums.DataLaneMarkingColor import DataLaneMarkingColor
from carla_data_classes.enums.DataLaneMarkingType import DataLaneMarkingType


@dataclass
class DataLaneMarking:
    """
    DataClass mapper to serialize carla.LaneMarking objects
    """
    marking_type: "DataLaneMarkingType"
    color: "DataLaneMarkingColor"
    width: float

    @staticmethod
    def from_lane_marking(marking: Optional[LaneMarking]) -> Optional["DataLaneMarking"]:
        """
        Returns the DataLaneMarking object based on the given lane marking
        @param marking: The carla.LaneMarking that should be converted into a DataLaneMarking
        @return: The filled DataLaneMarking object, or None if there is no marking on that side
        """
        if marking is None:
            return None
        marking_type = DataLaneMarkingType(int(marking.type))
        if marking_type == DataLaneMarkingType.NONE:
            return None
        return DataLaneMarking(marking_type=marking_type,
                               color=DataLaneMarkingColor(int(marking.color)),
                               width=marking.width)
