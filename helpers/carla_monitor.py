import os
import re
from datetime import datetime
from typing import List

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
from helpers.collisions import RecorderIndex, IdMapper, collisions_for_time_window
from helpers.json_helper import JSONHelper
from helpers.kinematics import compute_vel_acc_for_ticks


class CarlaMonitor:
    def __init__(self, carla_client: Client):
        self.ego_vehicle = None
        self.client = carla_client
        self.world: World = carla_client.get_world()
        self.map = self.world.get_map()

    @staticmethod
    def get_simulation_run_weather(weather_file: str) -> DataWeatherParameters:
        """
        Read Weather data from given json data into DataWeatherParameters data class
        @return: DataWeatherParameters from given json file, or default if file does not exist
        """
        print(f">> [IO] Evaluating weather data at path: '{weather_file}'")
        if not os.path.exists(weather_file):
            print(f">> [IO] There is no weather data for the recording file: '{weather_file}'")
            print(">> [CARLA] Take default weather parameters.")
            return DataWeatherParameters.from_weather(WeatherParameters.Default, DataWeatherParametersType.Default)
        return JSONHelper.load_weather(weather_file)

    # -------- tiny helpers -----------------------------------------------------

    @staticmethod
    def _parse_map_and_duration(info: str) -> tuple[str, float, int]:
        """
        Extract map name and duration (seconds) + frames from the info string.
        """
        try:
            map_name = info.split("Map: ")[1].split("\nDate")[0]
        except Exception:
            map_name = ""

        m_dur = re.search(r"^Duration:\s+([0-9.]+)\s+seconds", info, re.MULTILINE)
        duration = float(m_dur.group(1)) if m_dur else 0.0

        m_frames = re.search(r"^Frames:\s+(\d+)", info, re.MULTILINE)
        frames = int(m_frames.group(1)) if m_frames else 0

        return map_name, duration, frames

    # -------- main entry -------------------------------------------------------

    def monitor_simulation_run(
            self,
            file_path: str,
            weather_file_path: str,
            result_file_path: str,
            only_track_at_specific_interval: bool = False,
            specific_track_interval: float = 0.5
    ) -> None:
        """
        Replays the given .log and records dynamic data until the replay finishes.
        Collisions are taken from the recorder info and aligned by simulation time.
        """
        print(f">> [IO] Evaluate recorder data at path: '{file_path}'")
        log_data_path = file_path

        try:
            # Get info for map + duration
            info = self.client.show_recorder_file_info(log_data_path, True)
            if info == "File is not a CARLA recorder/n":
                print(">> [CARLA] The file at path", file_path, "is not a CARLA recorder")
                return
            if "not found" in info:
                print(">> [IO] The file at path", file_path, "cannot be found.")
                return

            rec_idx = RecorderIndex.parse(info)
            map_name, replay_duration, replay_frames = self._parse_map_and_duration(info)

            print(f">> [CARLA] Recording duration: {replay_duration:.3f}s")
            print(f">> [CARLA] Replay Tick Count: {replay_frames}")

            print(f">> [Data-AV Transformer] Create dynamic information for the recording file: '{log_data_path}'")

            weather_parameters: DataWeatherParameters = CarlaMonitor.get_simulation_run_weather(
                weather_file=weather_file_path)

            # Map name as in info
            map_name = info.split("Map: ")[1].split("\nDate")[0]
            print(f">> [CARLA] Load map: '{map_name}'")

            # Load map from recording
            self.client.load_world(map_name)
            world: World = self.client.get_world()

            # Initialize helpers
            rasterizer = MapRasterizer(world)
            api_helper = CarlaAPIHelper(self.client, world, rasterizer)

            print(">> [Data-AV Transformer] Load or calculate map data.")
            rasterizer.load_or_calculate_data_world(log_file_path=result_file_path, map_name=map_name)
            traffic_lights = rasterizer.get_all_traffic_lights()

            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
            dt_nominal = settings.fixed_delta_seconds or 0.05

            vehicles = []

            # Start replay of simulation and step once to let actors spawn
            api_helper.start_replaying(log_data_path)
            world.tick()

            while len(vehicles) == 0:
                vehicles = api_helper.get_vehicles()
                world.tick()

            vehicle_id_mapping = CarlaAPIHelper.create_recorder_to_sim_id_map(world, info,
                                                                              actor_filters=("vehicle.*",),
                                                                              position_tolerance_m=1)
            reverse_vehicle_id_mapping = {v: k for k, v in vehicle_id_mapping.items()}
            if len(vehicle_id_mapping) != len(vehicles):
                print(">> [CARLA] The vehicle id mapping is not equal to the vehicle id")
                return

            snapshot: WorldSnapshot = world.get_snapshot()
            base_sim_time = snapshot.timestamp.elapsed_seconds

            start_wall = datetime.now()
            ticks: List[TickData] = []

            mapper = IdMapper(rec_idx, debug=False)
            mapper.bootstrap_statics_once(world)

            print(">> [Data-AV Transformer] Start simulation replay (log-driven collisions)")

            tick_count = 0
            # time window for matching collisions to this tick (± half a tick by default)
            half_window = 0.5 * dt_nominal

            current_tick = 0.0
            actual_tick = 0
            tick_step = specific_track_interval / settings.fixed_delta_seconds

            while tick_count < replay_frames:
                if only_track_at_specific_interval and (tick_count % tick_step != 0.0):
                    world.tick()
                    tick_count += 1
                    current_tick += settings.fixed_delta_seconds
                    continue

                actual_tick += 1
                vehicles = api_helper.get_vehicles()
                if len(vehicles) == 0:
                    # Skip monitoring, as there are no vehicles to monitor
                    print("[CARLA] There are no vehicles at the current tick. Skip")
                    world.tick()
                    continue

                elapsed_time = (datetime.now() - start_wall).total_seconds()  # wall clock (print only)
                print(
                    f">> [CARLA] Simulation tick {actual_tick} at {current_tick:05f}s of {replay_duration}s; Elapsed time: {elapsed_time:3f}s")

                # Collisions for this tick: take all recorder frames within ± half_window around current_time
                per_actor_collisions = collisions_for_time_window(
                    rec_idx=rec_idx,
                    mapper=mapper,
                    world=world,
                    sim_time_rel=current_tick,
                    half_window=half_window
                )

                # Build DataActors from live world
                actors = api_helper.get_actors()
                actors = [a for a in actors if not isinstance(a, TrafficLight)]
                data_actors: List[DataActor] = []
                actor_positions: List[DataActorPosition] = []

                for actor in actors:
                    is_ego = isinstance(actor, Vehicle) and (actor.attributes.get("role_name") == "hero")
                    data_actor = api_helper.get_data_actor_from_actor(actor, is_ego)
                    if data_actor is None:
                        continue

                    mapped_id = reverse_vehicle_id_mapping.get(data_actor.id)
                    if mapped_id is not None:
                        data_actor.id = mapped_id

                    # Attach collisions for this runtime actor id (if any for this frame)
                    data_actor.collisions = per_actor_collisions.get(data_actor.id, [])

                    data_actors.append(data_actor)

                # Add traffic lights as DataActors
                for traffic_light in traffic_lights:
                    dynamic_traffic_light = world.get_traffic_light_from_opendrive_id(str(traffic_light.open_drive_id))
                    data_traffic_light = DataTrafficLight.from_traffic_light(dynamic_traffic_light, traffic_light)
                    if per_actor_collisions.get(data_traffic_light.id):
                        data_traffic_light.collisions = list(per_actor_collisions[data_traffic_light.id])
                    data_actors.append(data_traffic_light)

                # Lane positions
                for data_actor in data_actors:
                    nearest = rasterizer.get_closest_lane_midpoint(data_actor.location)
                    if not rasterizer.blocks_contain_waypoint(nearest.lane_id, nearest.road_id):
                        print(">> [Data-AV Transformer] Waypoint for current actor not in loaded world")
                        JSONHelper.log_invalid_run(log_data_path)
                        raise KeyboardInterrupt

                    actor_positions.append(DataActorPosition(
                        position_on_lane=nearest.distance_to_start,
                        road_id=nearest.road_id,
                        lane_id=nearest.lane_id,
                        actor=data_actor
                    ))

                ticks.append(TickData(
                    current_tick=current_tick,
                    actor_positions=actor_positions,
                    weather_parameters=weather_parameters
                ))

                world.tick()
                tick_count += 1
                current_tick += settings.fixed_delta_seconds

            print(">> [Data-AV Transformer] Calculate velocity and acceleration for actors")
            compute_vel_acc_for_ticks(ticks)

            print(">> [Data-AV Transformer] Analysis complete.")
            print(">> [IO] Save data to disk.")
            file_name = os.path.basename(log_data_path).split(".")[0]
            save_file_name = os.path.join(result_file_path, f"{JSONHelper.DYNAMIC_FILE_NAME_PREFIX}_{file_name}.json")
            saved_dynamic_data = api_helper.save_dynamic_data(ticks=ticks, file_path=save_file_name)
            JSONHelper.zip_and_delete_file(save_file_name)

        except RuntimeError as err:
            print(">> [Error] Logged failed Carla run")
            print(f">> [Error] Unexpected {err}, {type(err)}")
            JSONHelper.log_error("failed_run", name=log_data_path, error_message=f"{err}")
