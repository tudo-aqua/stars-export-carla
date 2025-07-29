from __future__ import annotations
from dataclasses import dataclass
from dataclass_wizard import JSONWizard
from typing import Tuple, List, Optional, Union
from carla import Rotation, Vector3D, Actor, Location, Vehicle, Waypoint, TrafficLight, TrafficSign, Walker, \
    WeatherParameters, BoundingBox

from carla_data_classes.data_enums import DataLaneType, DataLandmarkOrientation, DataLandmarkType, DataTrafficSignType, \
    DataWeatherParametersType


@dataclass
class TickData(JSONWizard):
    """
    DataClass to encapsulate ticks with its actors and their positions
    """

    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'
        tag_key = "type"  # look at the “type” field in JSON
        auto_assign_tags = False  # we supplied the tag values ourselves

    current_tick: float
    actor_positions: List[DataActorPosition]
    weather_parameters: DataWeatherParameters


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
    def from_enum_value(weather_value: str) -> DataWeatherParameters:
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
    def from_weather(weather: WeatherParameters, weather_enum: DataWeatherParametersType) -> DataWeatherParameters:
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


@dataclass
class DataBlock(JSONWizard):
    """
    DataClass to encapsulate a block with its roads
    """

    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'

    id: str
    roads: List[DataRoad]


@dataclass
class DataRoad:
    """
    DataClass to encapsulate a road with its lanes
    """
    road_id: int
    is_junction: bool
    lanes: List[DataLane]


@dataclass
class DataLane:
    """
    DataClass mapper to serialize carla.Lane objects and additional information
    """
    road_id: int
    lane_id: int
    lane_type: DataLaneType
    lane_width: float
    lane_length: float
    s: float
    predecessor_lanes: List[DataContactLaneInfo]
    successor_lanes: List[DataContactLaneInfo]
    intersecting_lanes: List[DataContactLaneInfo]
    lane_midpoints: List[DataLaneMidpoint]
    speed_limits: List[DataSpeedLimit]
    landmarks: List[DataLandmark]
    contact_areas: List[DataContactArea]
    traffic_lights: List[DataStaticTrafficLight]


@dataclass
class DataLaneMidpoint:
    """
    DataClass to wrap waypoint locations for a given lane. Each LaneMidpoint is in the middle of the lane
    has a distance to the start of the lane and its location
    """
    lane_id: int
    road_id: int
    distance_to_start: float
    location: DataLocation
    rotation: DataRotation


@dataclass
class DataSpeedLimit:
    """
    DataClass to wrap a speed limit section for a lane
    """
    speed_limit: float
    from_distance: float
    to_distance: float


@dataclass
class DataLocation:
    """
    DataClass mapper to serialize carla.Location objects
    """
    x: float
    y: float
    z: float

    def to_location(self, lift_z: bool = False) -> Location:
        """
        Returns a carla.Location object based on the x,y,z values of the given DataLocation
        @param lift_z: Decides, whether the z value should be lifted by 3 meters
        @return: The carla.Location object based on the DataLocation
        """
        if lift_z:
            # Add 3 meters to the z value
            return Location(x=self.x, y=self.y, z=self.z + 3.0)
        # Return as is
        return Location(x=self.x, y=self.y, z=self.z)

    def to_tuple(self) -> Tuple[float, float]:
        """
        Returns the x and y coordinates as a tuple
        @return: Tuple of the x and y value
        """
        return self.x, self.y

    @staticmethod
    def from_waypoint(waypoint: Waypoint) -> DataLocation:
        """
        Convenience method to get a DataLocation from a Waypoint
        @param waypoint: The Waypoint from which the location should be transformed
        @return: The DataLocation based on the given waypoint's location
        """
        # Get the carla.Location from the Waypoint
        location: Location = waypoint.transform.location
        # Map into DataLocation
        return DataLocation.from_location(location)

    @staticmethod
    def from_actor(actor: Actor) -> DataLocation:
        """
        Convenience method to get a DataLocation from an Actor
        @param actor: The Actor from which the location should be transformed
        @return: The DataLocation based on the given actor's location
        """
        # Get the carla.Location from the Waypoint
        location: Location = actor.get_location()
        # Map into DataLocation
        return DataLocation.from_location(location)

    @staticmethod
    def from_location(location: Location):
        """
        Convenience method to get a DataLocation from a Location
        @param location: The location that should be transformed
        @return: The DataLocation based on the given location
        """
        return DataLocation(x=location.x, y=-location.y, z=location.z)

    @staticmethod
    def from_bounding_box(bounding_box: BoundingBox) -> DataLocation:
        """
        Convenience method to get a DataLocation from a BoundingBox
        @param bounding_box: The bounding box that should be transformed
        @return: The DataLocation based on the given bounding box
        """
        location: Location = bounding_box.location
        return DataLocation(x=location.x, y=location.y, z=location.z)


@dataclass
class DataVector3D:
    """
    DataClass mapper to serialize carla.Vector3D objects
    """
    x: float
    y: float
    z: float

    @staticmethod
    def from_vector3d(vector: Vector3D) -> DataVector3D:
        """
        Convenience method to get a DataVector3D from a Vector3D
        @param vector: The vector that should be transformed
        @return: The DataVector3D based on the given Vector3D
        """
        return DataVector3D(x=vector.x, y=vector.y, z=vector.z)

    @staticmethod
    def from_bounding_box(bounding_box: BoundingBox) -> DataVector3D:
        """
        Convenience method to get a DataVector3D from a BoundingBox
        @param bounding_box: The bounding box that should be transformed
        @return: The DataVector3D based on the given BoundingBox
        """
        vector: Vector3D = bounding_box.extent
        return DataVector3D(x=vector.x, y=vector.y, z=vector.z)


@dataclass
class DataRotation:
    """
    DataClass mapper to serialize carla.Rotation objects
    """
    pitch: float
    yaw: float
    roll: float

    def to_rotation(self) -> Rotation:
        """
        Returns a carla.Rotation object based on the pitch, yaw and roll values of the given DataRotation
        @return: The carla.Rotation object based on the DataRotation
        """
        return Rotation(pitch=self.pitch, yaw=self.yaw, roll=self.roll)

    @staticmethod
    def from_rotation(rotation: Rotation) -> DataRotation:
        """
        Convenience method to get a DataRotation from a Rotation
        @param rotation: The rotation that should be transformed
        @return: The DataRotation based on the given rotation
        """
        return DataRotation(pitch=rotation.pitch, yaw=rotation.yaw, roll=rotation.roll)

    @staticmethod
    def from_actor(actor: Actor) -> DataRotation:
        """
        Convenience method to get a DataRotation from an Actor
        @param actor: The rotation that should be transformed
        @return: The DataRotation based on the given rotation
        """
        # Get the carla.Rotation from the Actor
        rotation: rotation = actor.get_transform().rotation
        # Map into DataRotation
        return DataRotation.from_rotation(rotation)

    @staticmethod
    def from_waypoint(waypoint: Waypoint) -> DataRotation:
        """
        Convenience method to get a DataLocation from a Waypoint
        @param waypoint: The Waypoint from which the location should be transformed
        @return: The DataLocation based on the given waypoint's location
        """
        # Get the carla.Location from the Waypoint
        rotation: Location = waypoint.transform.rotation
        # Map into DataLocation
        return DataRotation.from_rotation(rotation)

    @staticmethod
    def from_bounding_box(bounding_box: BoundingBox) -> DataRotation:
        """
        Convenience method to get a DataLocation from a BoundingBox
        @param bounding_box: The BoundingBox from which the location should be transformed
        @return: The DataLocation based on the given bounding box
        """
        # Get the carla.Location from the Waypoint
        rotation: Location = bounding_box.rotation
        # Map into DataLocation
        return DataRotation.from_rotation(rotation)


CONTACT_AREA_MARGIN: float = 3.0


@dataclass
class DataContactArea:
    """
    A contact area is spanned from a contact point of two crossing lanes.
    Based from this contact point the CONTACT_AREA_MARGIN is moved to the
    start and end of the lane.
    Therefore, 4 important points are created:
    contact_location = lane_1_start_pos + CONTACT_AREA_MARGIN
    contact_location = lane_1_end_pos - CONTACT_AREA_MARGIN
    contact_location = lane_2_start_pos + CONTACT_AREA_MARGIN
    contact_location = lane_2_end_pos - CONTACT_AREA_MARGIN
    """
    id: str  # combination of the lane and road ids of the given two lanes
    contact_location: DataLocation
    lane_1_road_id: int
    lane_1_id: int
    lane_1_start_pos: float
    lane_1_end_pos: float

    lane_2_road_id: int
    lane_2_id: int
    lane_2_start_pos: float
    lane_2_end_pos: float

    @staticmethod
    def from_lanes(contact_location: DataLocation, lane_1: DataLane, start_pos_lane_1: float, lane_2: DataLane,
                   start_pos_lane_2: float) -> DataContactArea:
        # Check if the lanes have to be switched
        if lane_2.road_id < lane_1.road_id:
            # Order the lanes such that the smaller road id is stored in lane_1
            save = lane_1
            lane_1 = lane_2
            lane_2 = save
            save = start_pos_lane_1
            start_pos_lane_1 = start_pos_lane_2
            start_pos_lane_2 = save
        # Build id from the lane_1 and lane_2 road and lane ids
        contact_area_id = f"{lane_1.road_id}_{lane_1.lane_id}+{lane_2.road_id}_{lane_2.lane_id}"
        contact_location = contact_location

        # Build critical section for lane_1
        lane_1_road_id = lane_1.road_id
        lane_1_id = lane_1.lane_id
        # Move CONTACT_AREA_MARGIN to start and end of lane
        # Also includes if the start of the lane is reached
        lane_1_start_pos = float(max(0.0, start_pos_lane_1 - CONTACT_AREA_MARGIN))
        # Also includes if the end of the lane is reached
        lane_1_end_pos = float(min(lane_1.lane_length, start_pos_lane_1 + CONTACT_AREA_MARGIN))

        # Build critical section for lane_2
        lane_2_road_id = lane_2.road_id
        lane_2_id = lane_2.lane_id
        # Move CONTACT_AREA_MARGIN to start and end of lane
        # Also includes if the start of the lane is reached
        lane_2_start_pos = float(max(0.0, start_pos_lane_2 - CONTACT_AREA_MARGIN))
        # Also includes if the end of the lane is reached
        lane_2_end_pos = float(min(lane_2.lane_length, start_pos_lane_2 + CONTACT_AREA_MARGIN))
        return DataContactArea(id=contact_area_id, contact_location=contact_location, lane_1_road_id=lane_1_road_id,
                               lane_1_id=lane_1_id, lane_1_start_pos=lane_1_start_pos, lane_1_end_pos=lane_1_end_pos,
                               lane_2_road_id=lane_2_road_id, lane_2_id=lane_2_id, lane_2_start_pos=lane_2_start_pos,
                               lane_2_end_pos=lane_2_end_pos)


@dataclass
class DataLandmark:
    """
    DataClass mapper to serialize carla.Landmark objects
    """
    id: int
    road_id: int
    name: str
    distance: float  # in meters
    s: float  # in meters (position along the geometry of the road)
    is_dynamic: bool
    orientation: DataLandmarkOrientation
    z_offset: float
    country: str
    type: DataLandmarkType
    sub_type: str
    value: float
    unit: str
    height: float  # in meters
    width: float  # in meters
    text: str
    h_offset: float  # in meters
    pitch: float  # Y-axis rotation
    roll: float  # X-axis rotation
    location: DataLocation
    rotation: DataRotation


@dataclass
class DataStaticTrafficLight:
    """
    DataClass mapper to serialize carla.TrafficLight objects
    These are only necessary for the static map information and do not
    contain the state of the TrafficLight! For the dynamic information
    head to DataTrafficLight(DataActor). TrafficLights are represented
    as Actors in carla and therefore the dynamic information is
    stored in the DataTrafficLight which inherits DataActor
    """
    open_drive_id: int
    position_distance: float
    location: DataLocation
    rotation: DataRotation
    stop_locations: List[DataLocation]


@dataclass
class DataBoundingBox:
    """
    DataClass mapper to serialize carla.BoundingBox objects
    """
    # Vector from the center of the box to one vertex.
    # The value in each axis equals half the size of
    # the box for that axis. extent.x * 2 would return
    # the size of the box in the X-axis.
    extent: DataVector3D
    location: DataLocation
    rotation: DataRotation
    vertices: List[DataLocation]

    @staticmethod
    def from_actor(actor: Actor) -> DataBoundingBox:
        """
        Convenience method to get a DataBoundingBox from an Actor
        @param actor: The Actor from which the bounding box should be transformed
        @return: The DataBoundingBox based on the given actor
        """
        bounding_box: BoundingBox = actor.bounding_box

        return DataBoundingBox(
            extent=DataVector3D.from_bounding_box(bounding_box),
            location=DataLocation.from_bounding_box(bounding_box),
            rotation=DataRotation.from_bounding_box(bounding_box),
            vertices=list(map(
                lambda x: DataLocation.from_location(x),
                actor.bounding_box.get_world_vertices(actor.get_transform())
            ))
        )


@dataclass
class DataActor(JSONWizard):
    """
    DataClass mapper to serialize carla.Actor objects
    """
    attributes: dict
    id: int
    type: str
    type_id: str
    is_alive: bool
    is_active: bool
    is_dormant: bool
    semantic_tags: List[int]
    bounding_box: DataBoundingBox | None
    location: DataLocation
    rotation: DataRotation

    @staticmethod
    def from_actor(actor: Actor) -> DataActor:
        """
        Build a *new* DataActor from a carla.Actor.
        """
        return DataActor(
            attributes=dict(actor.attributes),
            id=actor.id,
            type="Actor",
            type_id=actor.type_id,
            is_alive=actor.is_alive,
            is_active=actor.is_active,
            is_dormant=actor.is_dormant,
            semantic_tags=list(actor.semantic_tags),
            bounding_box=DataBoundingBox.from_actor(actor)
            if actor is not None else None,
            location=DataLocation.from_actor(actor),
            rotation=DataRotation.from_actor(actor),
        )


@dataclass
class DataTrafficLight(DataActor):
    """
    DataClass mapper to serialize carla.TrafficLight objects.
    This dataclass contains the dynamic data for a TrafficLight
    in the carla simulation
    """

    class _(JSONWizard.Meta):
        tag = "TrafficLight"

    state: int  # TODO convert to enum
    related_open_drive_id: int

    @staticmethod
    def from_traffic_light(
            actor: Optional[TrafficLight],
            static_tl: DataStaticTrafficLight,
    ) -> DataTrafficLight:
        """
        Build a *new* DataTrafficLight from a live TrafficLight actor
        and its static counterpart.
        """
        if actor is None:
            # synthetic “off‑world” traffic light
            return DataTrafficLight(
                attributes={},
                id=-1,
                type="TrafficLight",
                type_id="traffic.traffic_light",
                is_alive=False,
                is_active=False,
                is_dormant=False,
                semantic_tags=[],
                bounding_box=None,
                location=DataLocation(-1, -1, -1),
                rotation=DataRotation(-1, -1, -1),
                state=4,  # unknown
                related_open_drive_id=static_tl.open_drive_id,
            )

        # live traffic‑light → base fields
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "TrafficLight"
        return DataTrafficLight(
            **base,
            state                 = int(actor.state),
            related_open_drive_id = static_tl.open_drive_id,
        )


@dataclass
class DataPedestrian(DataActor):
    """
    DataClass mapper to serialize carla.Pedestrian objects
    """

    class _(JSONWizard.Meta):
        tag = "Pedestrian"

    @staticmethod
    def from_walker(actor: Walker) -> DataPedestrian:
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "Pedestrian"
        return DataPedestrian(
            **base,
            type_id = actor.type_id,
        )

    type_id: str


@dataclass
class DataTrafficSign(DataActor):
    """
    DataClass mapper to serialize carla.TrafficSign objects
    """

    class _(JSONWizard.Meta):
        tag = "TrafficSign"

    traffic_sign_type: DataTrafficSignType
    speed_limit: Optional[float] = None

    @staticmethod
    def from_traffic_sign(actor: TrafficSign) -> DataTrafficSign:
        base = DataActor.from_actor(actor)

        sign_type = DataTrafficSignType.UNKNOWN
        speed = None

        # parse the CARLA type_id, e.g. "traffic.speed_limit.30"
        parts = actor.type_id.split('.')
        if len(parts) >= 2:
            match parts[1]:
                case "speed_limit":
                    sign_type = DataTrafficSignType.MAX_SPEED
                    if len(parts) == 3:
                        speed = float(parts[2])
                case "stop":
                    sign_type = DataTrafficSignType.STOP
                case "yield":
                    sign_type = DataTrafficSignType.YIELD
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "TrafficSign"          # overwrite – no duplicate any more
        return DataTrafficSign(
            **base,                           # ← now contains the final "type"
            traffic_sign_type = sign_type,
            speed_limit       = speed,
        )


@dataclass
class DataVehicle(DataActor):
    """
    DataClass mapper to serialize carla.Vehicle objects
    """

    class _(JSONWizard.Meta):
        tag = "Vehicle"

    ego_vehicle: bool
    velocity: DataVector3D
    acceleration: DataVector3D
    forward_vector: DataVector3D
    angular_velocity: DataVector3D

    @staticmethod
    def from_vehicle(actor: Vehicle, ego_vehicle: bool = False) -> DataVehicle:
        base = DataActor.from_actor(actor).__dict__.copy()
        base["type"] = "Vehicle"
        return DataVehicle(
            **base,
            ego_vehicle      = ego_vehicle,
            velocity         = DataVector3D.from_vector3d(actor.get_velocity()),
            acceleration     = DataVector3D.from_vector3d(actor.get_acceleration()),
            angular_velocity = DataVector3D.from_vector3d(actor.get_angular_velocity()),
            forward_vector   = DataVector3D.from_vector3d(
                actor.get_transform().get_forward_vector()
            ),
        )


@dataclass
class DataContactLaneInfo:
    """
    DataClass wrapper to describe contact location with other lanes
    """
    road_id: int
    lane_id: int


ActorT = Union[
    DataTrafficLight,
    DataVehicle,
    DataPedestrian,
    DataTrafficSign,
    DataActor  # fallback if nothing matches
]


@dataclass
class DataActorPosition(JSONWizard):
    """
    DataClass to wrap the position of actors, including the lane and road id
    """
    position_on_lane: float
    road_id: int
    lane_id: int
    actor: ActorT
