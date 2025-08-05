from dataclasses import dataclass

from carla import Landmark

from carla_data_classes.enums.DataLandmarkOrientation import DataLandmarkOrientation
from carla_data_classes.enums.DataLandmarkType import DataLandmarkType
from carla_data_classes.static.DataLocation import DataLocation
from carla_data_classes.static.DataRotation import DataRotation


@dataclass
class DataLandmark:
    """
    DataClass mapper to serialize carla.Landmark objects
    """
    id: int
    road_id: int
    name: str
    distance: float  # in meters
    s: float  # in meters (position along the geometry of the road)
    is_dynamic: bool
    orientation: "DataLandmarkOrientation"
    z_offset: float
    country: str
    type: "DataLandmarkType"
    sub_type: str
    value: float
    unit: str
    height: float  # in meters
    width: float  # in meters
    text: str
    h_offset: float  # in meters
    pitch: float  # Y-axis rotation
    roll: float  # X-axis rotation
    location: "DataLocation"
    rotation: "DataRotation"

    @staticmethod
    def from_landmark(landmark: Landmark) -> "DataLandmark":
        """
        Returns the DataLandmark object based on the given landmark
        @param landmark: The landmark that should be converted into a DataLandmark
        @return: The filled DataLandmark object
        """
        orientation = DataLandmarkOrientation(int(landmark.orientation))
        landmark_type = DataLandmarkType(int(landmark.type))
        location = DataLocation.from_location(landmark.transform.location)
        rotation = DataRotation.from_rotation(landmark.transform.rotation)
        return DataLandmark(id=landmark.id, road_id=landmark.road_id, name=landmark.name, distance=landmark.distance,
                            s=landmark.s, is_dynamic=landmark.is_dynamic, orientation=orientation,
                            z_offset=landmark.z_offset, country=landmark.country, type=landmark_type,
                            sub_type=landmark.sub_type, value=landmark.value, unit=landmark.unit,
                            height=landmark.height, width=landmark.width, text=landmark.text,
                            h_offset=landmark.h_offset, pitch=landmark.pitch, roll=landmark.roll, location=location,
                            rotation=rotation)
