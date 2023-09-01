import argparse
import logging
import math
import random
import time
from datetime import datetime
from typing import List

import carla
from carla import VehicleLightState as vLS, WeatherParameters, Vehicle, Actor, Map
from carla import World, Client

from carla_data_classes import DataWeatherParameters
from carla_data_classes.data_enums import DataWeatherParametersType
from helpers.carla_api_helper import CarlaAPIHelper
from helpers.json_helper import JSONHelper


# ==============================================================================
# -- start_recording() ---------------------------------------------------------
# ==============================================================================


class CarlaDataGenerator:
    SIMULATOR_FIXED_TICK_DELTA = 0.05

    def __init__(self, carla_client: Client):
        self.ego_vehicle = None
        self.client = carla_client
        self.world: World = carla_client.get_world()
        self.map = self.world.get_map()

    @staticmethod
    def start_recording(client: Client, file_name: str, map_name: str, additional_infos=False) -> str:
        recording_dir = JSONHelper.get_file_path_for_name(name=file_name, map_name=map_name,
                                                          folder=JSONHelper.RECORDINGS_RUNS_FOLDER, file_ending="log")
        print(f"Recording location: {recording_dir}")
        client.start_recorder(recording_dir, additional_infos)
        return recording_dir

    def get_actor_blueprints(self, world, filter, generation):
        bps = world.get_blueprint_library().filter(filter)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        try:
            int_generation = int(generation)
            # Check if generation is in available generations
            if int_generation in [1, 2]:
                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                return bps
            else:
                print("   Warning! Actor Generation is not valid. No actor will be spawned.")
                return []
        except:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []

    def generate_traffic(self, args) -> List[int]:
        """
        This is a copy of the code in the shipped generate_traffic.py file of Carla
        """
        vehicles_list = []
        walkers_list = []
        all_id = []
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        synchronous_master = False
        random.seed(args.seed if args.seed is not None else int(time.time()))

        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
                you could experience some issues. If it's not working correctly, switch to synchronous \
                mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        blueprints = self.get_actor_blueprints(world, args.filterv, args.generationv)
        blueprintsWalkers = self.get_actor_blueprints(world, args.filterw, args.generationw)


        blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                         .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if args.car_lights_on:
            all_vehicle_actors = world.get_actors(vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0  # how many pedestrians will run
        percentagePedestriansCrossing = 0.0  # how many pedestrians will walk through the road
        if args.seed:
            world.set_pedestrians_seed(args.seed)
            random.seed(args.seed)
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        # Example of how to use Traffic Manager parameters
        traffic_manager.global_percentage_speed_difference(30.0)

        world.tick()

        return vehicles_list

    @staticmethod
    def change_map(client: Client) -> str:
        maps = CarlaAPIHelper.get_usable_maps(client)
        map = random.choice(maps)
        if map == "/Game/Carla/Maps/Town10HD_Opt":
            print("Map", map, "is already loaded.")
            return map
        print("Load map", map)
        client.load_world(map)
        return map

    @staticmethod
    def change_weather(world: World) -> DataWeatherParameters:
        # List of all possible WeatherParametersType instances
        weather_params = [DataWeatherParametersType.ClearNoon, DataWeatherParametersType.CloudyNoon,
                          DataWeatherParametersType.ClearSunset, DataWeatherParametersType.WetNoon,
                          DataWeatherParametersType.WetCloudyNoon, DataWeatherParametersType.SoftRainNoon,
                          DataWeatherParametersType.MidRainyNoon, DataWeatherParametersType.HardRainNoon,
                          DataWeatherParametersType.CloudySunset, DataWeatherParametersType.WetSunset,
                          DataWeatherParametersType.WetCloudySunset, DataWeatherParametersType.SoftRainSunset,
                          DataWeatherParametersType.MidRainSunset, DataWeatherParametersType.HardRainSunset]
        # Choose on weather enum
        new_weather_enum = random.choice(weather_params)
        new_weather: WeatherParameters = WeatherParameters.Default
        # Set the actual weather according to the enum
        if new_weather_enum == DataWeatherParametersType.ClearNoon:
            new_weather = WeatherParameters.ClearNoon
        elif new_weather_enum == DataWeatherParametersType.CloudyNoon:
            new_weather = WeatherParameters.CloudyNoon
        elif new_weather_enum == DataWeatherParametersType.ClearSunset:
            new_weather = WeatherParameters.ClearSunset
        elif new_weather_enum == DataWeatherParametersType.WetNoon:
            new_weather = WeatherParameters.WetNoon
        elif new_weather_enum == DataWeatherParametersType.WetCloudyNoon:
            new_weather = WeatherParameters.WetCloudyNoon
        elif new_weather_enum == DataWeatherParametersType.SoftRainNoon:
            new_weather = WeatherParameters.SoftRainNoon
        elif new_weather_enum == DataWeatherParametersType.MidRainyNoon:
            new_weather = WeatherParameters.MidRainyNoon
        elif new_weather_enum == DataWeatherParametersType.HardRainNoon:
            new_weather = WeatherParameters.HardRainNoon
        elif new_weather_enum == DataWeatherParametersType.CloudySunset:
            new_weather = WeatherParameters.CloudySunset
        elif new_weather_enum == DataWeatherParametersType.WetSunset:
            new_weather = WeatherParameters.WetSunset
        elif new_weather_enum == DataWeatherParametersType.WetCloudySunset:
            new_weather = WeatherParameters.WetCloudySunset
        elif new_weather_enum == DataWeatherParametersType.SoftRainSunset:
            new_weather = WeatherParameters.SoftRainSunset
        elif new_weather_enum == DataWeatherParametersType.MidRainSunset:
            new_weather = WeatherParameters.MidRainSunset
        elif new_weather_enum == DataWeatherParametersType.HardRainSunset:
            new_weather = WeatherParameters.HardRainSunset
        print("Changing the weather to", new_weather_enum)
        # Set the weather to the world
        world.set_weather(new_weather)
        return DataWeatherParameters.from_weather(new_weather, new_weather_enum)


if __name__ == '__main__':
    abort = False
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=200,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=30,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        default=0,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')
    argparser.add_argument(
        '-l', '--length-of-run',
        metavar='L',
        default=5,
        type=float,
        help='Length of the run in minutes (default: 5')

    args = argparser.parse_args()
    print("Seed:", args.seed)
    random.seed(args.seed)
    print("Connect to carla simulator")
    # Find carla simulator at localhost on port 2000
    client = carla.Client('localhost', 2000)
    # Try to connect for 10 seconds. Fail if not successful
    client.set_timeout(10.0)
    world: World = client.get_world()
    print("Connected to Carla")
    data_generator = CarlaDataGenerator(client)
    print("Generate traffic")
    # Change weather to one of the predefined ones
    data_weather = data_generator.change_weather(world=world)
    map_name = data_generator.change_map(client=client)
    time.sleep(5)

    file_name = map_name + "_seed" + str(args.seed)
    recording_dir = data_generator.start_recording(client=client, file_name=file_name, map_name=map_name)

    spawned_vehicle_ids = data_generator.generate_traffic(args)

    try:
        target_length_of_run_in_minutes = args.length_of_run
        print("Record", target_length_of_run_in_minutes, "minutes.")
        target_length_of_run = target_length_of_run_in_minutes * 60
        first_tick_timestamp = datetime.now()
        print("Current time:", first_tick_timestamp)
        length_of_current_run = 0.0

        actors: List[Actor] = list(world.get_actors())
        # Decide which vehicle should be used as ego for later reference
        ego_id: int = spawned_vehicle_ids[0]
        # Get the actual actor based on the ego_id
        actor = list(filter(lambda ac: ac.id == ego_id, actors))[0]
        # Save position data to later check if the vehicles have moved
        # Sometimes CARLA does not compute the movement of vehicles such that
        # the simulation data is unusable
        actor_x = actor.get_location().x
        actor_y = actor.get_location().y
        checked_for_moving_vehicles = False

        # Loop until the simulation time as reached the desired length of run time
        while length_of_current_run < target_length_of_run:
            # Calculate next simulation step
            world.tick()
            # Calculate new time stamps and length of current run
            current_tick_timestamp = datetime.now()
            length_of_current_run += CarlaDataGenerator.SIMULATOR_FIXED_TICK_DELTA

            # Sometimes the vehicles do not start moving in the simulation. Check and abort if so
            if length_of_current_run >= 10 and not checked_for_moving_vehicles:
                print("Check for non-moving vehicles")
                # Get the current actors
                new_actors: List[Actor] = list(world.get_actors())
                new_actor = list(filter(lambda ac: ac.id == ego_id, new_actors))[0]
                # Get the current position of the current ego vehicle
                new_actor_x = new_actor.get_location().x
                new_actor_y = new_actor.get_location().y
                # Calculate distance for each axis
                dist_x = pow(new_actor_x - actor_x, 2)
                dist_y = pow(new_actor_y - actor_y, 2)
                # Combine distance of both axes
                dist = math.sqrt(dist_x + dist_y)
                print(f"Distance: {dist}")
                checked_for_moving_vehicles = True

                # Check if the vehicles have moved
                if dist < 0.5:
                    abort = True
                    print("The vehicles have not moved. Abort")
                    # Something in the simulation has gone wrong. Log for later analysis
                    JSONHelper.log_aborted_run(file_name)
                    raise KeyboardInterrupt

        print("Total elapsed seconds:", (datetime.now() - first_tick_timestamp).total_seconds(), "s")
        print("Total elapsed simulation seconds:", length_of_current_run, "s")
    except KeyboardInterrupt:
        pass
    finally:
        if not abort:
            # Stop the recorder
            client.stop_recorder()
            # Zip and delete log file
            JSONHelper.zip_and_delete_file(recording_dir)
            # Log the collected data into a json file
            file_path = JSONHelper.get_file_path_for_name(name=file_name, map_name=map_name, file_ending="json",
                                                          folder=JSONHelper.RECORDINGS_RUNS_FOLDER,
                                                          prefix=JSONHelper.WEATHER_FILE_NAME_PREFIX)
            print("Save weather information to file", file_path)
            JSONHelper.log_weather(data_weather, file_path)
            # Zip and delete weather file
            JSONHelper.zip_and_delete_file(file_path)

        # Reset the world settings to default values
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        # Remove all remaining actors from the simulation
        actors = world.get_actors()

        print(f"Destroy {len(actors)} actors")
        client.apply_batch([carla.command.DestroyActor(x) for x in actors])

        time.sleep(0.5)
        print(f"Generation of recording with seed {args.seed} complete")
