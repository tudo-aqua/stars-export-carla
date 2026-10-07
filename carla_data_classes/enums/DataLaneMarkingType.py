from carla_data_classes.enums.ComparableEnum import ComparableEnum


class DataLaneMarkingType(ComparableEnum):
    """
    Matching enum class for carla.LaneMarkingType values
    """
    Other = 0
    Broken = 1
    Solid = 2
    SolidSolid = 3
    SolidBroken = 4
    BrokenSolid = 5
    BrokenBroken = 6
    BottsDots = 7
    Grass = 8
    Curb = 9
    NONE = 10
