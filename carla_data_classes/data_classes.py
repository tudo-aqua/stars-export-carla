from __future__ import annotations
from dataclasses import dataclass
from dataclass_wizard import JSONWizard
from typing import Tuple, List, Optional
from carla import Rotation, Vector3D, Actor, Location, Vehicle, Waypoint, TrafficLight, TrafficSign, Walker, \
    WeatherParameters

from carla_data_classes.data_enums import DataLaneType, DataLandmarkOrientation, DataLandmarkType, DataTrafficSignType, \
    DataWeatherParametersType


@dataclass
class TickData(JSONWizard):
    """
    DataClass to encapsulate ticks with its actors and their positions
    """

    class _(JSONWizard.Meta):
        key_transform_with_dump = 'SNAKE'
        debug_enabled = True

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
        debug_enabled = True

    type: DataWeatherParametersType
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
    def from_weather(weather: WeatherParameters, weather_enum: DataWeatherParametersType) -> DataWeatherParameters:
        return DataWeatherParameters(cloudiness=weather.cloudiness, precipitation=weather.precipitation,
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
        debug_enabled = True

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
class DataActorPosition:
    """
    DataClass to wrap the position of actors, including the lane and road id
    """
    position_on_lane: float
    road_id: int
    lane_id: int
    actor: DataActor


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
class DataActor:
    """
    DataClass mapper to serialize carla.Actor objects
    """
    id: int
    type: str
    type_id: str
    location: DataLocation
    rotation: DataRotation

    def __init__(self, actor: Actor):
        self.id = actor.id
        self.type = "Actor"
        self.type_id = actor.type_id
        self.location = DataLocation.from_actor(actor)
        self.rotation = DataRotation.from_actor(actor)


@dataclass
class DataTrafficLight(DataActor):
    """
    DataClass mapper to serialize carla.TrafficLight objects.
    This dataclass contains the dynamic data for a TrafficLight
    in the carla simulation
    """
    state: int  # TODO convert to enum
    related_open_drive_id: int

    def __init__(self, actor: Actor, static_traffic_light: DataStaticTrafficLight):
        self.related_open_drive_id = static_traffic_light.open_drive_id
        if actor is None:
            self.id = -1
            self.type = "TrafficLight"
            self.type_id = "traffic.traffic_light"
            self.state = 4
            self.location = DataLocation(-1, -1, -1)
            self.rotation = DataRotation(-1, -1, -1)
        else:
            super().__init__(actor)
            self.type = "TrafficLight"
            # Check if the given actor is actually a TrafficLight
            if type(actor) is TrafficLight:
                actor: TrafficLight
                self.state = actor.state


@dataclass
class DataPedestrian(DataActor):
    """
    DataClass mapper to serialize carla.Pedestrian objects
    """

    def __init__(self, actor: Walker):
        super().__init__(actor)
        self.type = "Pedestrian"
        self.type_id = actor.type_id

    type_id: str


@dataclass
class DataTrafficSign(DataActor):
    """
    DataClass mapper to serialize carla.TrafficSign objects
    """
    traffic_sign_type: DataTrafficSignType
    speed_limit: Optional[float] = None

    def __init__(self, actor: Actor):
        super().__init__(actor)
        self.type = "TrafficSign"
        # Check if the given Actor is actually a TrafficSign
        if type(actor) is TrafficSign:
            actor: TrafficSign
            types = actor.type_id.split('.')
            # Get the type of the TrafficSign based of the type_id
            if types[1] == "speed_limit":
                self.traffic_sign_type = DataTrafficSignType(DataTrafficSignType.MAX_SPEED.value)
                self.speed_limit = float(types[2])
            elif types[1] == "stop":
                self.traffic_sign_type = DataTrafficSignType(DataTrafficSignType.STOP.value)
            elif types[1] == "unknown":
                self.traffic_sign_type = DataTrafficSignType(DataTrafficSignType.UNKNOWN.value)
            elif types[1] == "yield":
                self.traffic_sign_type = DataTrafficSignType(DataTrafficSignType.YIELD.value)
            else:
                # There might be more TrafficSign in the future
                # I could not find a complete list of possible type_ids
                # in the carla documentation
                # TODO
                NotImplemented


@dataclass
class DataVehicle(DataActor):
    """
    DataClass mapper to serialize carla.Vehicle objects
    """
    ego_vehicle: bool
    location: DataLocation
    rotation: DataRotation
    velocity: DataVector3D
    acceleration: DataVector3D
    forward_vector: DataVector3D
    angular_velocity: DataVector3D

    def __init__(self, actor: Vehicle, ego_vehicle: bool = False):
        super().__init__(actor)
        self.type = "Vehicle"
        self.ego_vehicle = ego_vehicle
        self.location = DataLocation.from_actor(actor)
        self.rotation = DataRotation.from_actor(actor)
        self.velocity = DataVector3D.from_vector3d(actor.get_velocity())
        self.acceleration = DataVector3D.from_vector3d(actor.get_acceleration())
        self.angular_velocity = DataVector3D.from_vector3d(actor.get_angular_velocity())
        self.forward_vector = DataVector3D.from_vector3d(actor.get_transform().get_forward_vector())


@dataclass
class DataContactLaneInfo:
    """
    DataClass wrapper to describe contact location with other lanes
    """
    road_id: int
    lane_id: int
