import argparse
import math
import os
import time
from datetime import datetime
from typing import List

import carla
from carla import World, Client
from carla import WorldSnapshot, Vehicle, WeatherParameters
from carla.libcarla import TrafficLight

from carla_data_classes import DataActorPosition, TickData, DataWeatherParameters, DataWeatherParametersType, \
    DataTrafficLight, DataActor
from helpers.carla_api_helper import CarlaAPIHelper
from helpers.carla_recording_generator import CarlaDataGenerator
from helpers.json_helper import JSONHelper
from helpers.map_rasterizer import MapRasterizer


class CarlaMonitor:
    FORCE_JSON_FILE_UPDATES = False
    ONLY_TRACK_AT_SPECIFIC_INTERVAL = True
    SPECIFIC_TRACK_INTERVAL = 0.5  # in seconds

    DEFAULT_LOG_FILE = "D:/aqua/stas-main/stars-experiments-data/recordings/_Game_Carla_Maps_Town01/_Game_Carla_Maps_Town01_seed2.zip"
    DEFAULT_LOG_FOLDER = "D:/aqua/stas-main/stars-experiments-data/recordings/_Game_Carla_Maps_Town01"

    def __init__(self, carla_client: Client):
        self.ego_vehicle = None
        self.client = carla_client
        self.world: World = carla_client.get_world()
        self.map = self.world.get_map()

    @staticmethod
    def get_simulation_run_weather(file_name: str, folder_name: str) -> DataWeatherParameters:
        """
        Read Weather data from given json data into DataWeatherParameters data class
        @param file_name: The file name for the simulation
        @param folder_name: The folder in which the simulation is saved
        @return: DataWeatherParameters from given json file, or DataWeatherParameters. Default if file does not exist
        """
        # Get path for the weather json file (for some simulations runs, there is no weather data recorded)
        existing_file = JSONHelper.get_file_path_for_name(name=file_name, map_name=folder_name,
                                                          folder=JSONHelper.RECORDINGS_RUNS_FOLDER, file_ending="zip",
                                                          prefix=JSONHelper.WEATHER_FILE_NAME_PREFIX)
        # Unzip recorder file at path
        JSONHelper.extract_from_zip(existing_file)
        # Log file path
        log_data_path = existing_file.replace(".zip", ".json")
        print("Evaluating weather data at path:", log_data_path)
        # Check if th weather file exists
        if not os.path.exists(log_data_path):
            # Take default weather as no weather was saved
            print("There is no weather data for the recording file:", file_name,
                  "Take Default")
            return DataWeatherParameters.from_weather(WeatherParameters.Default, DataWeatherParametersType.Default)
        # Load weather data from file
        weather_data = JSONHelper.load_weather(log_data_path)
        JSONHelper.delete_file(log_data_path)
        return weather_data

    def monitor_simulation_run(self, folder_path: str, file_path: str, update_existing: bool) -> None:
        """
        Monitor the simulation run of the given file
        @param folder_path: The folder in which the simulation is saved in (is necessary for weather data)
        @param file_path: The file name of the simulation run to monitor
        @param update_existing: Decide whether existing monitor results should be overwritten
        @return: None
        """
        print("Evaluate recorder data at path", file_path)
        if ".log" in file_path:
            JSONHelper.delete_file(file_path)
            file_path = file_path.replace(".log", ".zip")
        # Unzip recorder file at path
        JSONHelper.extract_from_zip(file_path)
        # Log file path
        log_data_path = file_path.replace(".zip", ".log")

        try:
            # Get folder and file names
            file_name = os.path.basename(file_path).split(".")[0]
            folder_name = os.path.basename(folder_path)

            # Build path to existing data
            existing_file = JSONHelper.get_file_path_for_name(map_name=folder_name, name=file_name,
                                                              folder=JSONHelper.SIMULATION_RUNS_FOLDER,
                                                              prefix=JSONHelper.DYNAMIC_FILE_NAME_PREFIX,
                                                              file_ending="zip")

            # Check if the recording was already analyzed
            if not update_existing and os.path.exists(existing_file):
               print("The dynamic information for the recording file:", file_name,
                     "has already been calculated. Skip to next file.")
               return

            # Check data from carla recording for validity
            info = self.client.show_recorder_file_info(log_data_path, False)
            if info == "File is not a CARLA recorder/n":
                print("The file at path", file_path, "is not a CARLA recorder")
                return

            if info.__contains__("not found"):
                print("The file at path", file_path, "cannot be found.")
                return

            # Get count of all ticks in the recorded file using the recorder_file_info and split
            replay_tick_count = int(info.split("Frames: ")[1].split("Duration")[0])

            print("Create dynamic information for the recording file:", existing_file)

            weather_parameters: DataWeatherParameters = CarlaMonitor.get_simulation_run_weather(file_name=file_name,
                                                                                                folder_name=folder_name)

            # Get map name of recording
            map_name = info.split("Map: ")[1].split("\nDate")[0]
            # Load map from recording
            self.client.load_world(map_name)

            # Get world for later use
            world: World = self.client.get_world()

            # Initialize necessary helper classes
            rasterizer = MapRasterizer(world)
            api_helper = CarlaAPIHelper(client, world, rasterizer)

            # Calculate the static data for the current map
            blocks = rasterizer.load_or_calculate_data_blocks(map_name=folder_name, file_name=folder_name,
                                                              update_existing=update_existing)

            traffic_lights = rasterizer.get_all_traffic_lights()

            # Set synchronous mode settings
            new_settings = world.get_settings()
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = CarlaDataGenerator.SIMULATOR_FIXED_TICK_DELTA
            world.apply_settings(new_settings)

            # Start replay of simulation
            api_helper.start_replaying(log_data_path)
            # A tick is necessary for the server to process the replay_file command
            world.tick()

            # Get the current tick and save it
            snapshot: WorldSnapshot = world.get_snapshot()
            first_tick_timestamp = snapshot.timestamp.elapsed_seconds
            start_time = datetime.now()
            ticks = []

            print("Start with simulation replay")

            # Tick the world for each frame in the replay
            for tick in range(1, replay_tick_count):
                # Advance simulation by one tick
                world.tick()

                # Update current time duration
                snapshot: WorldSnapshot = world.get_snapshot()
                now = snapshot.timestamp.elapsed_seconds
                current_tick = (now - first_tick_timestamp)

                # If ONLY_TRACK_AT_SPECIFIC_INTERVAL flag is set: Monitor only every SPECIFIC_TRACK_INTERVAL seconds
                if CarlaMonitor.ONLY_TRACK_AT_SPECIFIC_INTERVAL and math.fmod(round(current_tick, 2),
                                                                              CarlaMonitor.SPECIFIC_TRACK_INTERVAL) != 0:
                    continue
                elapsed_time = (datetime.now() - start_time).total_seconds()
                print("Simulation tick:", tick, "of", replay_tick_count, "Result Tick:", current_tick,
                      f"Elapsed time: {elapsed_time}s")

                # Get all vehicles
                vehicles = api_helper.get_vehicles()
                # Check if there are already vehicles spawned
                if len(vehicles) == 0:
                    # Skip monitoring, as there are no vehicles to monitor
                    print("There are no vehicles at the current tick. Skip")
                    continue

                # Get all actors that are in the block of the ego vehicle
                actors = api_helper.get_actors()
                actors = list(filter(lambda a: a is not TrafficLight, actors))

                actor_positions: List[DataActorPosition] = []
                data_actors: List[DataActor] = []
                # Calculate the actor position for each actor (Vehicle, Pedestrian, TrafficSign, TrafficLight)
                for actor in actors:
                    is_ego = False
                    # Transform the carla.Actor into a DataActor
                    data_actor = api_helper.get_data_actor_from_actor(actor, is_ego)
                    if data_actor is None:
                        continue
                    data_actors.append(data_actor)

                for traffic_light in traffic_lights:
                    dynamic_traffic_light = self.world.get_traffic_light_from_opendrive_id(
                        str(traffic_light.open_drive_id))
                    data_traffic_light = DataTrafficLight(dynamic_traffic_light, traffic_light)
                    data_actors.append(data_traffic_light)

                for data_actor in data_actors:
                    # Get road and lane information for the current actor
                    nearest_lane_midpoint = rasterizer.get_closest_lane_midpoint(data_actor.location)

                    wp_is_in_blocks = rasterizer.blocks_contain_waypoint(nearest_lane_midpoint.lane_id,
                                                                         nearest_lane_midpoint.road_id)
                    if not wp_is_in_blocks:
                        print("The waypoint for the current actor is not in the rasterized blocks")
                        JSONHelper.log_invalid_run(file_name)
                        abort = True
                        raise KeyboardInterrupt

                    # Create ActorPosition object
                    actor_position = DataActorPosition(position_on_lane=nearest_lane_midpoint.distance_to_start,
                                                       road_id=nearest_lane_midpoint.road_id,
                                                       lane_id=nearest_lane_midpoint.lane_id,
                                                       actor=data_actor)
                    # Save the ActorPosition object in the list
                    actor_positions.append(actor_position)

                # Collect all ActorPositions and wrap them in a TickData object
                tick = TickData(current_tick=current_tick, actor_positions=actor_positions,
                                weather_parameters=weather_parameters)
                # Save the TickData object in the list
                ticks.append(tick)
            print("Analysis complete. Save to disk.")
            # Save Dynamic data to disk
            dynamic_data_path = existing_file.replace(".zip", ".json")
            saved_dynamic_data = api_helper.save_dynamic_data(ticks=ticks, file_path=dynamic_data_path)
            if saved_dynamic_data:
                JSONHelper.zip_and_delete_file(dynamic_data_path)
        except RuntimeError as err:
            print("Logged failed Carla run")
            print(f"Unexpected {err}, {type(err)}")
            JSONHelper.log_error("failed_run", name=file_name, error_message=f"{err}")
        finally:
            settings = self.world.get_settings()

            # Reset world setting to default values
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

            # Destroy all actors for the current simulation
            actors = self.world.get_actors()
            print(f"destroying {len(actors)} actors")
            self.client.apply_batch([carla.command.DestroyActor(x) for x in actors])
            time.sleep(10)
            print("Remove", log_data_path)
            # Remove extracted zip content
            JSONHelper.delete_file(log_data_path)


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-f', '--file',
        metavar='F',
        type=str,
        default=CarlaMonitor.DEFAULT_LOG_FILE,
        help='Set explicit recording file path')
    argparser.add_argument(
        '-a', '--seed',
        metavar='S',
        type=int,
        default=-1,
        help='Set explicit seed')
    argparser.add_argument(
        '-u', '--update-existing-files',
        metavar='U',
        type=bool,
        default=False,
        help='Decide whether existing files should be updated.')
    args = argparser.parse_args()

    seed = args.seed
    if seed != -1:
        file_path = JSONHelper.get_path_from_seed(seed=args.seed, recording=True)
    else:
        file_path = args.file
    file_path = os.path.abspath(file_path)
    print("Analyze file at:", file_path)
    folder_name = os.path.dirname(file_path)
    print("Analyze file in:", folder_name)

    update_existing = args.update_existing_files or CarlaMonitor.FORCE_JSON_FILE_UPDATES
    print("Update existing files:", update_existing)
    print("Connect to Carla")

    try:
        # Find carla simulator at localhost on port 2000
        client = carla.Client('localhost', 2000)

        # Try to connect for 20 seconds. Fail if not successful
        client.set_timeout(60.0)
        client.get_world().get_actors()
        monitor = CarlaMonitor(carla_client=client)
        print("Connected to carla")

        print("Analyze recording", file_path)
        monitor.monitor_simulation_run(folder_path=folder_name, file_path=file_path, update_existing=update_existing)
        print("Done with monitoring the recording")
    except RuntimeError as err:
        print("Logged failed Carla run in main")
        print(f"Unexpected {err}, {type(err)}")
        JSONHelper.log_error("failed_run", name=file_path, error_message=f"{err}")
