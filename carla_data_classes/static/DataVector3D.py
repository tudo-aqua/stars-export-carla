from dataclasses import dataclass

from carla import Vector3D, BoundingBox


@dataclass
class DataVector3D:
    """
    DataClass mapper to serialize carla.Vector3D objects
    """
    x: float
    y: float
    z: float

    @staticmethod
    def from_vector3d(vector: Vector3D) -> "DataVector3D":
        """
        Convenience method to get a DataVector3D from a Vector3D
        @param vector: The vector that should be transformed
        @return: The DataVector3D based on the given Vector3D
        """
        return DataVector3D(x=vector.x, y=vector.y, z=vector.z)

    @staticmethod
    def from_bounding_box(bounding_box: BoundingBox) -> "DataVector3D":
        """
        Convenience method to get a DataVector3D from a BoundingBox
        @param bounding_box: The bounding box that should be transformed
        @return: The DataVector3D based on the given BoundingBox
        """
        vector: Vector3D = bounding_box.extent
        return DataVector3D(x=vector.x, y=vector.y, z=vector.z)
