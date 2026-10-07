from dataclasses import dataclass
from typing import Tuple

from carla import Actor, Location, Waypoint, BoundingBox


@dataclass
class DataLocation:
    """
    DataClass mapper to serialize carla.Location objects
    """
    x: float
    y: float
    z: float

    def to_location(self, lift_z: bool = False) -> Location:
        """
        Returns a carla.Location object based on the x,y,z values of the given DataLocation
        @param lift_z: Decides, whether the z value should be lifted by 3 meters
        @return: The carla.Location object based on the DataLocation
        """
        if lift_z:
            # Add 3 meters to the z value
            return Location(x=self.x, y=self.y, z=self.z + 3.0)
        # Return as is
        return Location(x=self.x, y=self.y, z=self.z)

    def to_tuple(self) -> Tuple[float, float]:
        """
        Returns the x and y coordinates as a tuple
        @return: Tuple of the x and y value
        """
        return self.x, self.y

    @staticmethod
    def from_waypoint(waypoint: Waypoint) -> "DataLocation":
        """
        Convenience method to get a DataLocation from a Waypoint
        @param waypoint: The Waypoint from which the location should be transformed
        @return: The DataLocation based on the given waypoint's location
        """
        # Get the carla.Location from the Waypoint
        location: Location = waypoint.transform.location
        # Map into DataLocation
        return DataLocation.from_location(location)

    @staticmethod
    def from_actor(actor: Actor) -> "DataLocation":
        """
        Convenience method to get a DataLocation from an Actor
        @param actor: The Actor from which the location should be transformed
        @return: The DataLocation based on the given actor's location
        """
        # Get the carla.Location from the Waypoint
        location: Location = actor.get_location()
        # Map into DataLocation
        return DataLocation.from_location(location)

    @staticmethod
    def from_location(location: Location):
        """
        Convenience method to get a DataLocation from a Location
        @param location: The location that should be transformed
        @return: The DataLocation based on the given location
        """
        return DataLocation(x=location.x, y=-location.y, z=location.z)

    @staticmethod
    def from_bounding_box(bounding_box: BoundingBox) -> "DataLocation":
        """
        Convenience method to get a DataLocation from a BoundingBox
        @param bounding_box: The bounding box that should be transformed
        @return: The DataLocation based on the given bounding box
        """
        location: Location = bounding_box.location
        return DataLocation(x=location.x, y=location.y, z=location.z)
