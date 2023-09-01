import os

from carla import *

from carla_data_classes.data_classes import *
from helpers.json_helper import JSONHelper
from helpers.map_rasterizer import MapRasterizer

ACTOR_BLOCK_FILTERING_SWITCH = True
USABLE_MAPS = ["Town01", "Town02", "Town10"]

class CarlaAPIHelper:
    """
    This class provides methods that return information of dynamic objects of the Carla simulation
    """

    def __init__(self, client: Client, world: World, rasterizer: MapRasterizer):
        super(CarlaAPIHelper, self).__init__()

        self.client = client
        self._world: World = world
        self._debug = world.debug
        self._map: Map = self._world.get_map()
        self._rasterizer = rasterizer

    @staticmethod
    def save_dynamic_data(ticks: [TickData], file_path: os.path) -> bool:
        """
        Saves the ticks created for the current map to disk.
        """
        if ticks.__len__() == 0:
            # Something in the simulation has gone wrong. Log for later analysis
            JSONHelper.log_aborted_run("No Ticks in: " + file_path)
            return False
        JSONHelper.log_tick_data(ticks, file_path)
        return True

    def get_actors_in_block(self, block: DataBlock, remove_sensors: bool = True) -> List[Actor]:
        """
        Returns a list of all actors in the world
        :return: List of all actors
        """
        all_actors = list(self._world.get_actors())
        filtered_actors = []
        # Filter actors
        if ACTOR_BLOCK_FILTERING_SWITCH:
            for actor in all_actors:
                # Check if the current actor is in the given block
                if self._rasterizer.is_actor_in_block(actor=actor, block=block):
                    # Remove Sensors from the result list
                    if remove_sensors and isinstance(actor, Sensor):
                        continue
                    filtered_actors.append(actor)
        return filtered_actors

    def get_actors(self, remove_sensors: bool = True) -> List[Actor]:
        """
        Returns a list of all actors in the world
        :return: List of all actors
        """
        all_actors = list(self._world.get_actors())
        if not remove_sensors:
            return all_actors
        filtered_actors = []
        # Go through the actors
        for actor in all_actors:
            # Check if the Actor is a Sensor
            if isinstance(actor, Sensor):
                continue
            # Actor is no Sensor: Keep
            filtered_actors.append(actor)
        return filtered_actors

    def get_vehicles(self) -> List[Actor]:
        """
        Return the list of all Vehicles in the world
        :return: List of all Vehicles currently active in the world
        """
        # It's the first time asking for the traffic light
        # Get all actors of the world
        actors = self.get_actors()
        # Filter actors to get traffic lights only
        vehicles = list(filter(lambda actor: type(actor) is Vehicle, actors))
        return vehicles

    def start_replaying(self, replay_file_path, time_factor=1.0, show_file_info=False, start_time=None, duration=None,
                        camera_id=None):
        """
        This method starts the replay of the file under the given replay_file_path
        @param replay_file_path: The file path to the replay file
        @param time_factor: Time factor at which the replay should be replayed with
        @param show_file_info: Decide whether additional file information should be retrieved
        @param start_time: Declare a specific start time for the replay
        @param duration: Declare a specific duration for which the replay should last
        @param camera_id: Specify a specific camera by id that should be used
        """
        file = str(replay_file_path)
        if not start_time:
            start_time = 0.0
        if not duration:
            duration = 0.0
        if not camera_id:
            camera_id = 0
        self.client.replay_file(name=file, time_start=start_time, duration=duration, follow_id=camera_id)
        self.client.set_replayer_time_factor(time_factor)
        if show_file_info:
            self.client.show_recorder_file_info(file)

    # region Static methods
    ########################################
    #         static methods               #
    ########################################

    @staticmethod
    def get_usable_maps(client: Client) -> List[str]:
        available_maps = client.get_available_maps()
        usable_maps = []
        for map in available_maps:
            if "_Opt" in map:
                continue
            for usable_map in USABLE_MAPS:
                if usable_map in map:
                    usable_maps.append(map)
        return usable_maps

    @staticmethod
    def get_data_actor_from_actor(actor: Actor, ego_vehicle: bool = False) -> Optional[DataActor]:
        """
        Returns the filled DataActor from the carla Actor
        :param actor: The actor which should be transformed into the DataActor class
        :return: Filled DataActor object
        """
        data_actor: Optional[DataActor] = None
        # Check of which type the given actor is and transform it into the correct dataclass
        if type(actor) is Vehicle:
            data_actor = DataVehicle(actor, ego_vehicle)
        elif type(actor) is TrafficSign:
            data_actor = DataTrafficSign(actor)
        elif type(actor) is TrafficLight:
            data_actor = None
        elif type(actor) is Walker:
            data_actor = DataPedestrian(actor)
        else:
            if actor.type_id == "spectator":
                return None
            elif "pedestrian" in actor.type_id:
                data_actor = DataPedestrian(actor)
            # TODO: If an actor of another type is tracked
        if data_actor:
            data_actor.location = DataLocation.from_location(location=actor.get_location())
        return data_actor
    # endregion
