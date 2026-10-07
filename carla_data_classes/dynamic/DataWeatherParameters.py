from dataclasses import dataclass

from carla import WeatherParameters
from dataclass_wizard import JSONWizard

from carla_data_classes.enums.DataWeatherParametersType import DataWeatherParametersType


@dataclass
class DataWeatherParameters(JSONWizard):
    """
    DataClass to encapsulate the weather parameters of the world
    """

    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'

    type: DataWeatherParametersType
    dust_storm: float
    cloudiness: float
    precipitation: float
    precipitation_deposits: float
    wind_intensity: float
    sun_azimuth_angle: float
    sun_altitude_angle: float
    fog_density: float
    fog_distance: float
    wetness: float
    fog_falloff: float
    scattering_intensity: float
    mie_scattering_scale: float
    rayleigh_scattering_scale: float

    @staticmethod
    def from_enum_value(weather_value: str) -> "DataWeatherParameters":
        # Retrieve the enum value dynamically
        if hasattr(DataWeatherParametersType, weather_value):
            enum_value = getattr(DataWeatherParametersType, weather_value)
            weather_mapping = {
                "Default": WeatherParameters.Default,
                "DustStorm": WeatherParameters.DustStorm,
                "ClearNoon": WeatherParameters.ClearNoon,
                "CloudyNoon": WeatherParameters.CloudyNoon,
                "WetNoon": WeatherParameters.WetNoon,
                "WetCloudyNoon": WeatherParameters.WetCloudyNoon,
                "SoftRainNoon": WeatherParameters.SoftRainNoon,
                "MidRainyNoon": WeatherParameters.MidRainyNoon,
                "HardRainNoon": WeatherParameters.HardRainNoon,
                "ClearSunset": WeatherParameters.ClearSunset,
                "CloudySunset": WeatherParameters.CloudySunset,
                "WetSunset": WeatherParameters.WetSunset,
                "WetCloudySunset": WeatherParameters.WetCloudySunset,
                "SoftRainSunset": WeatherParameters.SoftRainSunset,
                "MidRainSunset": WeatherParameters.MidRainSunset,
                "HardRainSunset": WeatherParameters.HardRainSunset,
            }
            weather_parameters = weather_mapping.get(weather_value, WeatherParameters.Default)
            return DataWeatherParameters.from_weather(weather=weather_parameters, weather_enum=enum_value)
        else:
            raise ValueError(f"Weather value '{weather_value}' is not in the enum.")

    @staticmethod
    def from_weather(weather: WeatherParameters, weather_enum: DataWeatherParametersType) -> "DataWeatherParameters":
        return DataWeatherParameters(cloudiness=weather.cloudiness, precipitation=weather.precipitation,
                                     dust_storm=weather.dust_storm,
                                     precipitation_deposits=weather.precipitation_deposits,
                                     wind_intensity=weather.wind_intensity, type=weather_enum,
                                     sun_azimuth_angle=weather.sun_azimuth_angle,
                                     sun_altitude_angle=weather.sun_altitude_angle, fog_density=weather.fog_density,
                                     fog_distance=weather.fog_distance, wetness=weather.wetness,
                                     fog_falloff=weather.fog_falloff, scattering_intensity=weather.scattering_intensity,
                                     mie_scattering_scale=weather.mie_scattering_scale,
                                     rayleigh_scattering_scale=weather.rayleigh_scattering_scale)
