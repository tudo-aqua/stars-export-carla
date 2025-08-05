from carla_data_classes.enums.ComparableEnum import ComparableEnum


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
    DustStorm = 15
