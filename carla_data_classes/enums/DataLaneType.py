from carla_data_classes.enums.ComparableEnum import ComparableEnum


class DataLaneType(ComparableEnum):
    """
    Matching enum class for carla.LaneType values
    """
    Any = -2
    Bidirectional = 512
    Biking = 16
    Border = 64
    Driving = 2
    Entry = 131072
    Exit = 262144
    Median = 1024
    NONE = 1
    OffRamp = 524288
    OnRamp = 1048576
    Parking = 256
    Rail = 65536
    Restricted = 128
    RoadWorks = 16384
    Shoulder = 8
    Sidewalk = 32
    Special1 = 2048
    Special2 = 4096
    Special3 = 8192
    Stop = 4
    Tram = 32768
