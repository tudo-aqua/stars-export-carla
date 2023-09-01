from enum import Enum


class ComparableEnum(Enum):
    """
    This class allows all inheriting Enum classes to be compared by name
    """

    def __eq__(self, other):
        return self.name == other.name


class DataWeatherParametersType(ComparableEnum):
    """
    Matching enum class for the pre-defined WeatherParameters of carla
    """
    Default = 0
    ClearNoon = 1
    CloudyNoon = 2
    WetNoon = 3
    WetCloudyNoon = 4
    SoftRainNoon = 5
    MidRainyNoon = 6
    HardRainNoon = 7
    ClearSunset = 8
    CloudySunset = 9
    WetSunset = 10
    WetCloudySunset = 11
    SoftRainSunset = 12
    MidRainSunset = 13
    HardRainSunset = 14


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


class DataLandmarkType(ComparableEnum):
    """
    Matching enum class for carla.LandmarkType values
    """
    Danger = 101
    LanesMerging = 121
    CautionPedestrian = 133
    CautionBicycle = 138
    LevelCrossing = 150
    StopSign = 206
    YieldSign = 205
    MandatoryTurnDirection = 209
    MandatoryLeftRightDirection = 211
    TwoChoiceTurnDirection = 214
    Roundabout = 215
    PassRightLeft = 222
    AccessForbidden = 250
    AccessForbiddenMotorvehicles = 251
    AccessForbiddenTrucks = 253
    AccessForbiddenBicycle = 254
    AccessForbiddenWeight = 263
    AccessForbiddenWidth = 264
    AccessForbiddenHeight = 265
    AccessForbiddenWrongDirection = 267
    ForbiddenUTurn = 272
    MaximumSpeed = 274
    ForbiddenOvertakingMotorvehicles = 276
    ForbiddenOvertakingTrucks = 277
    AbsoluteNoStop = 283
    RestrictedStop = 286
    HasWayNextIntersection = 301
    PriorityWay = 306
    PriorityWayEnd = 307
    CityBegin = 310
    CityEnd = 311
    Highway = 330
    DeadEnd = 357
    RecommendedSpeed = 380
    RecommendedSpeedEnd = 381
    LightPost = 1000001


class DataLandmarkOrientation(ComparableEnum):
    """
    Matching enum class for carla.LandmarkOrientation values
    """
    Positive = 0
    Negative = 1
    Both = 2


class DataTrafficSignType(ComparableEnum):
    """
    Matching enum class for carla.TrafficSignType values
    """
    INVALID = 0
    SUPPLEMENT_ARROW_APPLIES_LEFT = 1
    SUPPLEMENT_ARROW_APPLIES_RIGHT = 2
    SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT = 3
    SUPPLEMENT_ARROW_APPLIES_UP_DOWN = 4
    SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE = 5
    SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE = 6
    SUPPLEMENT_APPLIES_NEXT_N_KM_TIME = 7
    SUPPLEMENT_ENDS = 8
    SUPPLEMENT_RESIDENTS_ALLOWED = 9
    SUPPLEMENT_BICYCLE_ALLOWED = 10
    SUPPLEMENT_MOPED_ALLOWED = 11
    SUPPLEMENT_TRAM_ALLOWED = 12
    SUPPLEMENT_FORESTAL_ALLOWED = 13
    SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED = 14
    SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN = 15
    SUPPLEMENT_RAILWAY_ONLY = 16
    SUPPLEMENT_APPLIES_FOR_WEIGHT = 17
    DANGER = 18
    LANES_MERGING = 19
    CAUTION_PEDESTRIAN = 20
    CAUTION_CHILDREN = 21
    CAUTION_BICYCLE = 22
    CAUTION_ANIMALS = 23
    CAUTION_RAIL_CROSSING_WITH_BARRIER = 24
    CAUTION_RAIL_CROSSING = 25
    YIELD_TRAIN = 26
    YIELD = 27
    STOP = 28
    REQUIRED_RIGHT_TURN = 29
    REQUIRED_LEFT_TURN = 30
    REQUIRED_STRAIGHT = 31
    REQUIRED_STRAIGHT_OR_RIGHT_TURN = 32
    REQUIRED_STRAIGHT_OR_LEFT_TURN = 33
    ROUNDABOUT = 34
    PASS_RIGHT = 35
    PASS_LEFT = 36
    BICYCLE_PATH = 37
    FOOTWALK = 38
    FOOTWALK_BICYCLE_SHARED = 39
    FOOTWALK_BICYCLE_SEP_RIGHT = 40
    FOOTWALK_BICYCLE_SEP_LEFT = 41
    PEDESTRIAN_AREA_BEGIN = 42
    ACCESS_FORBIDDEN = 43
    ACCESS_FORBIDDEN_TRUCKS = 44
    ACCESS_FORBIDDEN_BICYCLE = 45
    ACCESS_FORBIDDEN_MOTORVEHICLES = 46
    ACCESS_FORBIDDEN_WEIGHT = 47
    ACCESS_FORBIDDEN_WIDTH = 48
    ACCESS_FORBIDDEN_HEIGHT = 49
    ACCESS_FORBIDDEN_WRONG_DIR = 50
    ENVIRONMENT_ZONE_BEGIN = 51
    ENVIRONMENT_ZONE_END = 52
    MAX_SPEED = 53
    SPEED_ZONE_30_BEGIN = 54
    SPEED_ZONE_30_END = 55
    HAS_WAY_NEXT_INTERSECTION = 56
    PRIORITY_WAY = 57
    CITY_BEGIN = 58
    CITY_END = 59
    MOTORWAY_BEGIN = 60
    MOTORWAY_END = 61
    MOTORVEHICLE_BEGIN = 62
    MOTORVEHICLE_END = 63
    INFO_MOTORWAY_INFO = 64
    CUL_DE_SAC = 65
    CUL_DE_SAC_EXCEPT_PED_BICYCLE = 66
    INFO_NUMBER_OF_AUTOBAHN = 67
    DIRECTION_TURN_TO_AUTOBAHN = 68
    DIRECTION_TURN_TO_LOCAL = 69
    DESTINATION_BOARD = 70
    FREE_TEXT = 71
    UNKNOWN = 72
