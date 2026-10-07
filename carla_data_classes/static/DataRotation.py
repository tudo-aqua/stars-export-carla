from dataclasses import dataclass

from carla import Location, BoundingBox, Waypoint, Actor, Rotation


@dataclass
class DataRotation:
    """
    DataClass mapper to serialize carla.Rotation objects
    """
    pitch: float
    yaw: float
    roll: float

    def to_rotation(self) -> Rotation:
        """
        Returns a carla.Rotation object based on the pitch, yaw and roll values of the given DataRotation
        @return: The carla.Rotation object based on the DataRotation
        """
        return Rotation(pitch=self.pitch, yaw=self.yaw, roll=self.roll)

    @staticmethod
    def from_rotation(rotation: Rotation) -> "DataRotation":
        """
        Convenience method to get a DataRotation from a Rotation
        @param rotation: The rotation that should be transformed
        @return: The DataRotation based on the given rotation
        """
        return DataRotation(pitch=rotation.pitch, yaw=-rotation.yaw, roll=rotation.roll)

    @staticmethod
    def from_actor(actor: Actor) -> "DataRotation":
        """
        Convenience method to get a DataRotation from an Actor
        @param actor: The rotation that should be transformed
        @return: The DataRotation based on the given rotation
        """
        # Get the carla.Rotation from the Actor
        rotation: Rotation = actor.get_transform().rotation
        # Map into DataRotation
        return DataRotation.from_rotation(rotation)

    @staticmethod
    def from_waypoint(waypoint: Waypoint) -> "DataRotation":
        """
        Convenience method to get a DataLocation from a Waypoint
        @param waypoint: The Waypoint from which the location should be transformed
        @return: The DataLocation based on the given waypoint's location
        """
        # Get the carla.Location from the Waypoint
        rotation: Location = waypoint.transform.rotation
        # Map into DataLocation
        return DataRotation.from_rotation(rotation)

    @staticmethod
    def from_bounding_box(bounding_box: BoundingBox) -> "DataRotation":
        """
        Convenience method to get a DataLocation from a BoundingBox
        @param bounding_box: The BoundingBox from which the location should be transformed
        @return: The DataLocation based on the given bounding box
        """
        # Get the carla.Location from the Waypoint
        rotation: Location = bounding_box.rotation
        # Map into DataLocation
        return DataRotation.from_rotation(rotation)
