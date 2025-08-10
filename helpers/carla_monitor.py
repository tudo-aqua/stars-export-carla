import argparse
import math
import os
from datetime import datetime
from typing import List

import carla
from carla import World, Client
from carla import WorldSnapshot, Vehicle, WeatherParameters
from carla.libcarla import TrafficLight

from carla_data_classes.dynamic import (
    DataWeatherParameters,
    DataActorPosition,
    DataActor,
    DataTrafficLight,
    TickData
)
from carla_data_classes.enums.DataWeatherParametersType import DataWeatherParametersType
from data_av_static import MapRasterizer
from helpers.carla_api_helper import CarlaAPIHelper
from helpers.collisions import _parse_recorder_info, _IdMapper, _collisions_for_frame
from helpers.json_helper import JSONHelper
from helpers.kinematics import compute_vel_acc_for_ticks


class CarlaMonitor:
    FORCE_JSON_FILE_UPDATES = False
    ONLY_TRACK_AT_SPECIFIC_INTERVAL = False
    SPECIFIC_TRACK_INTERVAL = 0.5  # in seconds

    DEFAULT_LOG_FOLDER = "C:/Users/Till/Downloads/scenarios/scenarios/scenario_1"

    def __init__(self, carla_client: Client):
        self.ego_vehicle = None
        self.client = carla_client
        self.world: World = carla_client.get_world()
        self.map = self.world.get_map()

    @staticmethod
    def get_simulation_run_weather(weather_file: str) -> DataWeatherParameters:
        """
        Read Weather data from given json data into DataWeatherParameters data class
        @return: DataWeatherParameters from given json file, or DataWeatherParameters. Default if file does not exist
        """
        print(f">> [IO] Evaluating weather data at path: '{weather_file}'")
        # Check if th weather file exists
        if not os.path.exists(weather_file):
            # Take default weather as no weather was saved
            print(f">> [IO] There is no weather data for the recording file: '{weather_file}'")
            print(">> [CARLA] Take default weather parameters.")
            return DataWeatherParameters.from_weather(WeatherParameters.Default, DataWeatherParametersType.Default)
        # Load weather data from file
        weather_data = JSONHelper.load_weather_from_scenic(weather_file)
        return weather_data

    def monitor_simulation_run(self, file_path: str, weather_file_path: str, result_file_path: str) -> None:
        """
        Monitor the simulation run of the given file
        @param file_path: The file name of the simulation run to monitor
        @return: None
        """
        print(f">> [IO] Evaluate recorder data at path: '{file_path}'")
        log_data_path = file_path

        try:
            # 1) Parse full recorder info (tick-level details incl. collisions per frame)
            info = self.client.show_recorder_file_info(log_data_path, True)
            if info == "File is not a CARLA recorder/n":
                print(">> [CARLA] The file at path", file_path, "is not a CARLA recorder")
                return
            if "not found" in info:
                print(">> [IO] The file at path", file_path, "cannot be found.")
                return

            recording_index = _parse_recorder_info(info)
            recording_frequency = recording_index.delta_seconds
            replay_tick_count = recording_index.frames

            print(f">> [CARLA] Recording frequency: {recording_frequency:.3f}s")
            print(f">> [CARLA] Replay Tick Count: {replay_tick_count}")

            print(f">> [Data-AV Transformer] Create dynamic information for the recording file: '{log_data_path}'")

            weather_parameters: DataWeatherParameters = CarlaMonitor.get_simulation_run_weather(
                weather_file=weather_file_path)

            # Get map name of recording
            map_name = info.split("Map: ")[1].split("\nDate")[0]
            print(f">> [CARLA] Load map: '{map_name}'")

            # Load map from recording
            self.client.load_world(map_name)

            # Get world for later use
            world: World = self.client.get_world()

            # Initialize necessary helper classes
            rasterizer = MapRasterizer(world)
            api_helper = CarlaAPIHelper(self.client, world, rasterizer)

            print(">> [Data-AV Transformer] Load or calculate map data.")
            # Calculate the static data for the current map
            blocks = rasterizer.load_or_calculate_data_map(log_file_path=result_file_path, map_name=map_name)

            traffic_lights = rasterizer.get_all_traffic_lights()

            # Set synchronous mode settings
            new_settings = world.get_settings()
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = recording_frequency
            world.apply_settings(new_settings)

            # Start replay of simulation
            api_helper.start_replaying(log_data_path)
            # A tick is necessary for the server to process the replay_file command
            world.tick()

            # Prepare mapping from recorder IDs to runtime IDs
            recorder_id_mapper = _IdMapper(recording_index)

            # Get the current tick and save it
            snapshot: WorldSnapshot = world.get_snapshot()
            first_tick_timestamp = snapshot.timestamp.elapsed_seconds
            start_time = datetime.now()
            ticks = []

            print(">> [Data-AV Transformer] Start with simulation replay")

            # Tick the world for each frame in the replay
            for step in range(1, replay_tick_count):
                # Advance simulation by one tick
                world.tick()

                # Update current time duration
                snapshot: WorldSnapshot = world.get_snapshot()
                now = snapshot.timestamp.elapsed_seconds
                current_time = (now - first_tick_timestamp)

                # If ONLY_TRACK_AT_SPECIFIC_INTERVAL flag is set: Monitor only every SPECIFIC_TRACK_INTERVAL seconds
                if CarlaMonitor.ONLY_TRACK_AT_SPECIFIC_INTERVAL and math.fmod(round(current_time, 2),
                                                                              CarlaMonitor.SPECIFIC_TRACK_INTERVAL) != 0:
                    continue

                # Compute absolute frame index from elapsed time to avoid off-by-one
                # Frame 1 at t=0, Frame 2 at t=dt, ...
                frame_idx = step

                elapsed_time = (datetime.now() - start_time).total_seconds()
                print(
                    f">> [CARLA] Simulation step: {step:05d}/{replay_tick_count:05d}; "
                    f"Result t={current_time:.3f}s (frame {frame_idx}); Elapsed: {elapsed_time:.3f}s")

                # Keep the ID mapping up-to-date for anything created up to this frame
                recorder_id_mapper.update_until_frame(world, frame_idx)
                if step <= 3:
                    recorder_id_mapper.update_until_frame(world, frame_idx)
                    recorder_id_mapper.update_until_frame(world, frame_idx)

                # Get all vehicles, skip if none
                vehicles = api_helper.get_vehicles()
                if len(vehicles) == 0:
                    print(">> [CARLA] There are no vehicles at the current tick. Skip")
                    continue

                # Dynamic collisions for this *frame*
                per_actor_collisions = _collisions_for_frame(frame_idx, world, recorder_id_mapper, recording_index)

                # Get all actors that are in the block of the ego vehicle
                actors = api_helper.get_actors()
                # Previous code tried to filter out class object; keep only real actors
                actors = [a for a in actors if not isinstance(a, TrafficLight)]

                actor_positions: List[DataActorPosition] = []
                data_actors: List[DataActor] = []

                # Calculate the actor position for each actor (Vehicle, Pedestrian, TrafficSign, TrafficLight)
                for actor in actors:
                    if isinstance(actor, Vehicle):
                        role = actor.attributes.get("role_name", None)
                        is_ego = (role == "hero")
                    else:
                        is_ego = False

                    # Transform the carla.Actor into a DataActor
                    data_actor = api_helper.get_data_actor_from_actor(actor, is_ego)
                    if data_actor is None:
                        continue

                    # ---- NEW: attach collisions for this runtime actor id (if any for this frame) ----
                    if per_actor_collisions.get(data_actor.id):
                        # Replace the empty default [] with the collisions for this tick
                        data_actor.collisions = list(per_actor_collisions[data_actor.id])

                    data_actors.append(data_actor)

                # Also add traffic lights as DataActors (your original code)
                for tl in traffic_lights:
                    dynamic_tl = self.world.get_traffic_light_from_opendrive_id(str(tl.open_drive_id))
                    data_tl = DataTrafficLight.from_traffic_light(dynamic_tl, tl)

                    # Attach collisions if we mapped the recorder TL id to this runtime id
                    if per_actor_collisions.get(data_tl.id):
                        data_tl.collisions = list(per_actor_collisions[data_tl.id])

                    data_actors.append(data_tl)

                # Enrich with lane position for each actor
                for data_actor in data_actors:
                    nearest = rasterizer.get_closest_lane_midpoint(data_actor.location)
                    wp_is_in_blocks = rasterizer.blocks_contain_waypoint(nearest.lane_id, nearest.road_id)
                    if not wp_is_in_blocks:
                        print(
                            ">> [Data-AV Transformer] The waypoint for the current actor is not in the rasterized blocks")
                        JSONHelper.log_invalid_run(log_data_path)
                        raise KeyboardInterrupt

                    actor_position = DataActorPosition(
                        position_on_lane=nearest.distance_to_start,
                        road_id=nearest.road_id,
                        lane_id=nearest.lane_id,
                        actor=data_actor
                    )
                    actor_positions.append(actor_position)

                # Collect all ActorPositions and wrap them in a TickData object
                tick = TickData(
                    current_tick=current_time,
                    actor_positions=actor_positions,
                    weather_parameters=weather_parameters
                )
                ticks.append(tick)

            print(">> [Data-AV Transformer] Calculate velocity and acceleration for actors")
            compute_vel_acc_for_ticks(ticks)
            # Strip the first ticks so that the velocity and acceleration is correct from the beginning
            ticks = ticks[3:]
            print(">> [Data-AV Transformer] Analysis complete.")
            print(">> [IO] Save data to disk.")
            # Save Dynamic data to disk
            file_name = os.path.basename(log_data_path).split(".")[0]
            save_file_name = os.path.join(result_file_path, f"{JSONHelper.DYNAMIC_FILE_NAME_PREFIX}_{file_name}.json")
            saved_dynamic_data = api_helper.save_dynamic_data(ticks=ticks, file_path=save_file_name)
            JSONHelper.zip_and_delete_file(save_file_name)

        except RuntimeError as err:
            print(">> [Error] Logged failed Carla run")
            print(f">> [Error] Unexpected {err}, {type(err)}")
            JSONHelper.log_error("failed_run", name=log_data_path, error_message=f"{err}")
        finally:
            settings = self.world.get_settings()
            # Reset world setting to default values
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            # Destroy all actors for the current simulation
            actors = self.world.get_actors()
            print(f">> [CARLA] Destroying {len(actors)} actors")
            self.client.apply_batch([carla.command.DestroyActor(x) for x in actors])


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-f', '--folder',
        metavar='F',
        type=str,
        default=CarlaMonitor.DEFAULT_LOG_FOLDER,
        help='Set explicit recording folder path')
    args = argparser.parse_args()
    folder_path = os.path.abspath(args.folder)
    print("Analyze folder at:", folder_path)

    # Initialize variables
    log_file = None
    scenic_file = None

    # Search for the files
    for file in os.listdir(folder_path):
        if file.endswith(".log"):
            log_file = os.path.join(folder_path, file)
        elif file.endswith(".scenic"):
            scenic_file = os.path.join(folder_path, file)

    print(f"Got simulation file: {log_file}")
    print(f"Got scenic file: {scenic_file}")
    print("Connect to Carla")

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(60.0)
        client.get_world().get_actors()
        monitor = CarlaMonitor(carla_client=client)
        print("Connected to carla")
        print("Analyze recording", log_file)
        # NOTE: your GUI wrapper passes result_file_path; add it here if you run as script.
        # monitor.monitor_simulation_run(file_path=log_file, weather_file_path=scenic_file, result_file_path="<out dir>")
        print("Done with monitoring the recording")
    except RuntimeError as err:
        print("Logged failed Carla run in main")
        print(f"Unexpected {err}, {type(err)}")
        JSONHelper.log_error("failed_run", name=folder_path, error_message=f"{err}")
