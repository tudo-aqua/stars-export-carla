from enum import Enum

import carla


class CameraPosition(Enum):
    """Camera position enum class"""

    # View from the front of the vehicle just above street level
    FRONT_ON_STREET = \
        (carla.Location(1, 0, 0), carla.Rotation(0, 0, 0))

    # View from in front of the windshield (outside)
    WINDSHIELD = \
        (carla.Location(1, 0, 1.2), carla.Rotation(0, 0, 0))

    # View from inside the vehicle behind the windshield
    INSIDE = \
        (carla.Location(0, 0, 1.2), carla.Rotation(0, 0, 0))

    # View from the back of the vehicle
    BACK_ABOVE = \
        (carla.Location(-6, 0, 2.5), carla.Rotation(0, 0, 0))

    # View from the top of the vehicle near
    TOP_DOWN_NEAR = \
        (carla.Location(0, 0, 10), carla.Rotation(-90, 0, 0))

    # View from the top of the vehicle far
    TOP_DOWN_FAR = \
        (carla.Location(0, 0, 25), carla.Rotation(-90, 0, 0))

    # Rear view
    REAR = \
        (carla.Location(6, 0, 2.5), carla.Rotation(0, 180, 0))
