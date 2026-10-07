from carla_data_classes.enums.ComparableEnum import ComparableEnum


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
