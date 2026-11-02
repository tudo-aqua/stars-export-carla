from enum import Enum


class CameraPosition(Enum):
    """
    Camera position enum class. Values are ((x, y, z), (pitch, yaw, roll)),
    plain tuples rather than carla.Location/carla.Rotation so the GUI can list
    the positions without importing the CARLA Python API.
    """

    # View from the front of the vehicle just above street level
    FRONT_ON_STREET = \
        ((1, 0, 0), (0, 0, 0))

    # View from in front of the windshield (outside)
    WINDSHIELD = \
        ((1, 0, 1.2), (0, 0, 0))

    # View from inside the vehicle behind the windshield
    INSIDE = \
        ((0, 0, 1.2), (0, 0, 0))

    # View from the back of the vehicle
    BACK_ABOVE = \
        ((-6, 0, 2.5), (0, 0, 0))

    # View from the top of the vehicle near
    TOP_DOWN_NEAR = \
        ((0, 0, 10), (-90, 0, 0))

    # View from the top of the vehicle far
    TOP_DOWN_FAR = \
        ((0, 0, 25), (-90, 0, 0))

    # View from the top of the vehicle far
    TOP_DOWN_VERY_FAR = \
        ((0, 0, 50), (-90, 0, 0))

    # Rear view
    REAR = \
        ((6, 0, 2.5), (0, 180, 0))
