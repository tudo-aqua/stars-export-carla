from carla_data_classes.enums.ComparableEnum import ComparableEnum


class DataLandmarkOrientation(ComparableEnum):
    """
    Matching enum class for carla.LandmarkOrientation values
    """
    Positive = 0
    Negative = 1
    Both = 2
