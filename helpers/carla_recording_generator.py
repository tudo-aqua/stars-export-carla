import argparse
import logging
import random
import time
from types import SimpleNamespace
from typing import List, Optional

import carla
from carla import World, Client, WeatherParameters
from more_itertools import flatten

from carla_data_classes.dynamic import DataWeatherParameters
from carla_data_classes.enums.DataWeatherParametersType import DataWeatherParametersType
from data_av_static.lane_utils import _LaneUtils
from data_av_static.world_builder import _BlockBuilder
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

    def generate_traffic(self, args, client, world) -> List[int]:
        """
        This is a copy of the code in the shipped generate_traffic.py file of Carla
        """
        vehicles_list = []
        walkers_list = []
        all_id = []
        client.set_timeout(10.0)
        synchronous_master = False
        random.seed(args.seed if args.seed is not None else int(time.time()))

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

    def _spawn_parked_vehicles(
            self,
            world: World,
            count: int,
            rng: random.Random,
            *,
            filterv: str = "vehicle.*",
    ) -> list[carla.Actor]:
        """
        Spawn `count` parked vehicles on shoulder lanes.
        Strategy:
          - sample shoulder waypoints,
          - for each, try a small search of (lateral, forward, z) offsets to find a collision-free pose,
          - use compact vehicles,
          - tag via role_name='parked', disable movement.
        """
        if count <= 0:
            return []

        # Shoulder candidates (centerline transforms)
        transforms = self._find_shoulder_spawn_transforms(world, min_width_m=1.8)
        if not transforms:
            print("[CARLA] No shoulder transforms found for parked vehicles.")
            return []

        rng.shuffle(transforms)

        # Compact vehicles only → much higher success rate on ~2 m shoulders
        bps = list(world.get_blueprint_library().filter(filterv or "vehicle.*"))
        small: list[carla.ActorBlueprint] = []
        for bp in bps:
            bid = bp.id.lower()
            if any(k in bid for k in
                   ("bus", "truck", "firetruck", "ambulance", "garbage", "sprinter", "van", "carlacola", "semi",
                    "trailer")):
                continue
            # prefer 4-wheelers
            if bp.has_attribute("number_of_wheels") and bp.get_attribute("number_of_wheels").as_int() < 4:
                continue
            small.append(bp)
        candidates = small or bps  # fallback to any if filter empties

        spawned: list[carla.Actor] = []
        placed: list[carla.Location] = []

        # Tuning knobs
        min_spacing_m = 2.0  # allow close spacing; lower to 0.6 if you want even denser
        lateral_margin_m = 0.5  # margin from outer shoulder edge
        z_lift = 0.35  # spawn slightly above ground to avoid ground collision
        fwd_nudge_vals = (0.0, 0.6, -0.6)  # try in-place, then forward/back
        # try positions across the shoulder from near edge inward
        lateral_fractions = (0.9, 0.7, 0.5, 0.3)  # relative to (lane_width/2)

        def _too_close(loc: carla.Location) -> bool:
            for p in placed:
                dx = loc.x - p.x
                dy = loc.y - p.y
                dz = loc.z - p.z
                if (dx * dx + dy * dy + dz * dz) < (min_spacing_m * min_spacing_m):
                    return True
            return False

        amap = world.get_map()

        for base_tf in transforms:
            if len(spawned) >= count:
                break

            # verify shoulder at this transform
            wp = amap.get_waypoint(base_tf.location, project_to_road=True, lane_type=carla.LaneType.Any)
            if not wp or wp.lane_type != carla.LaneType.Shoulder:
                continue

            fwd = wp.transform.get_forward_vector()
            right = wp.transform.get_right_vector()
            half_w = max(0.0, wp.lane_width * 0.5)

            # search small set of offsets (lateral across shoulder; fwd nudge ±)
            placed_here = False
            for frac in lateral_fractions:
                if placed_here:
                    break
                lateral = max(0.0, half_w * frac - lateral_margin_m)

                for fn in fwd_nudge_vals:
                    if placed_here:
                        break

                    # compute candidate transform
                    loc = carla.Location(
                        x=base_tf.location.x + right.x * lateral + fwd.x * fn,
                        y=base_tf.location.y + right.y * lateral + fwd.y * fn,
                        z=base_tf.location.z + z_lift,
                    )
                    rot = carla.Rotation(
                        pitch=base_tf.rotation.pitch,
                        yaw=base_tf.rotation.yaw,
                        roll=base_tf.rotation.roll,
                    )
                    tf = carla.Transform(loc, rot)

                    # crowding check
                    if _too_close(loc):
                        continue

                    # get a fresh blueprint by id (no .clone() in CARLA)
                    base_bp = rng.choice(candidates)
                    bp = world.get_blueprint_library().find(base_bp.id)
                    if bp.has_attribute("role_name"):
                        bp.set_attribute("role_name", "parked")

                    actor = world.try_spawn_actor(bp, tf)
                    if not actor:
                        # final micro-nudge forward if needed
                        loc2 = carla.Location(loc.x + fwd.x * 0.3, loc.y + fwd.y * 0.3, loc.z)
                        tf2 = carla.Transform(loc2, rot)
                        actor = world.try_spawn_actor(bp, tf2)

                    if not actor:
                        continue

                    # pin it in place
                    try:
                        actor.set_autopilot(False)
                    except Exception:
                        pass
                    try:
                        actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                    except Exception:
                        pass

                    spawned.append(actor)
                    placed.append(actor.get_transform().location)
                    placed_here = True

                    if len(spawned) >= count:
                        break

        print(f"Spawned {len(spawned)} parked vehicles (requested {count}).")
        return spawned

    def _to_asset_path(self, client: Client, name: str) -> Optional[str]:
        """Accepts 'Town05' or a full asset path and returns the asset path, or None if not installed."""
        availableMaps = client.get_available_maps()
        name = (name or "").strip()
        if not name:
            return None
        return next((map for map in availableMaps if name in map), None)

    def _load_map_by_seed(self, client: Client, candidates: Optional[list[str]], seed: int) -> str:
        """Pick a map deterministically from candidates using the seed; fall back to usable maps."""
        if candidates:
            pool_set = set()
            for name in candidates:
                if not name:
                    continue
                asset = self._to_asset_path(client, name)
                if asset is None:
                    print(f"!! Map '{name}' is not installed on this CARLA server; skipping.")
                    continue
                pool_set.add(asset)
            pool = sorted(pool_set)
        else:
            pool = sorted({m for m in CarlaAPIHelper.get_usable_maps(client)})
        if not pool:
            raise RuntimeError("No candidate maps available to choose from.")

        rng = random.Random(seed)
        chosen = rng.choice(pool)

        current = client.get_world().get_map().name
        if chosen in current:
            print(f"Map '{chosen}' is already loaded.")
            return chosen

        print(f"Load map '{chosen}'")
        client.load_world(chosen)
        return chosen

    def run_recording_generation(
            self,
            client: Client,
            *,
            seed: int,
            length_minutes: float,
            number_of_vehicles: int,
            number_of_walkers: int,
            filterv: str = "vehicle.*",
            generationv: str = "All",
            filterw: str = "walker.pedestrian.*",
            generationw: str = "2",
            candidate_maps: Optional[list[str]] = None,
            output_dir: Optional[str] = None,
            no_rendering: bool = False,
            number_of_parked: int = 0,
    ) -> None:

        """
        Perform a single recording run in the already-connected CARLA server.
        - Deterministically selects a map from candidate_maps using 'seed'
        - Changes weather
        - Spawns traffic
        - Records for 'length_minutes'
        - Stores outputs under 'output_dir' (if provided)
        """
        # Allow overriding output directory used by JSONHelper
        if output_dir:
            import os
            os.makedirs(output_dir, exist_ok=True)
            JSONHelper.RECORDINGS_RUNS_FOLDER = output_dir  # redirect all outputs

        # Build an 'args' namespace expected by the existing helper methods in this module
        args = SimpleNamespace(
            # seeded determinism
            seed=int(seed),

            # sim/traffic manager settings expected by generate_traffic()
            tm_port=8000,  # CARLA default TM port
            respawn=False,  # only if you want dormant respawn
            hybrid=False,  # TrafficManager hybrid physics
            asynch=False,  # we run in synchronous mode
            hero=False,  # no hero vehicle
            car_lights_on=False,  # leave lights off globally

            # rendering toggle
            no_rendering=bool(no_rendering),

            # actor counts and filters
            number_of_vehicles=int(number_of_vehicles),
            number_of_walkers=int(number_of_walkers),
            filterv=str(filterv or "vehicle.*"),
            generationv=str(generationv or "All"),
            filterw=str(filterw or "walker.pedestrian.*"),
            generationw=str(generationw or "2"),
            number_of_parked=number_of_parked,

            # duration (minutes -> used later)
            length_of_run=float(length_minutes),
        )

        # Deterministic RNG for this run
        print("Seed:", args.seed)
        random.seed(args.seed)

        print("Connect to carla simulator (reusing existing client)")
        world: World = client.get_world()
        print("Connected to Carla")

        data_generator = CarlaDataGenerator(client)

        # Weather first (weather gets logged later)
        data_weather = data_generator.change_weather(world=world)

        # Choose and load a map deterministically by seed
        map_name = self._load_map_by_seed(client=client, candidates=candidate_maps, seed=args.seed)
        time.sleep(5)  # give CARLA some breaths after map load

        # Build the recording log file path
        file_name = f"seed_{args.seed}"
        recording_dir = JSONHelper.get_file_path_for_name(
            name=file_name,
            map_name=map_name,
            file_ending="log",
            folder=JSONHelper.RECORDINGS_RUNS_FOLDER,
            prefix=getattr(JSONHelper, "RECORDING_FILE_NAME_PREFIX", "recording"),
        )
        try:
            # Spawn traffic (your existing function configures TM/sync etc.)
            data_generator.generate_traffic(args, client, world)

            # Parked vehicles (if requested)
            if number_of_parked and number_of_parked > 0:
                print(f"[CARLA] Generate {number_of_parked} parked vehicles")
                self._spawn_parked_vehicles(
                    world,
                    count=int(number_of_parked),
                    rng=random.Random(seed + 13),  # independent but deterministic stream
                    filterv=filterv,
                )

            # Switch world settings (sync/no_rendering) inside your generate_traffic() already,
            # but we still start the recorder here before spawning traffic (like your main).
            client.start_recorder(recording_dir, False)

            # Record for the requested duration
            snapshot = world.get_snapshot()
            start_timestamp = snapshot.timestamp.elapsed_seconds
            end_timestamp = start_timestamp + (args.length_of_run * 60)
            current_timestamp = start_timestamp
            while current_timestamp < end_timestamp:
                world.tick()
                current_timestamp = world.get_snapshot().timestamp.elapsed_seconds

        finally:
            # Stop and zip recorder log
            try:
                client.stop_recorder()
            except Exception:
                pass

        # Zip and remove raw recorder log (matches your pattern at the bottom of file)
        try:
            JSONHelper.zip_and_delete_file(recording_dir)
        except Exception as e:
            print("Warning: failed to zip recording:", e)

        # Save weather json next to recording
        try:
            weather_path = JSONHelper.get_file_path_for_name(
                name=file_name,
                map_name=map_name,
                file_ending="json",
                folder=JSONHelper.RECORDINGS_RUNS_FOLDER,
                prefix=JSONHelper.WEATHER_FILE_NAME_PREFIX,
            )
            print("Save weather information to file", weather_path)
            JSONHelper.log_weather(data_weather, weather_path)
            JSONHelper.zip_and_delete_file(weather_path)
        except Exception as e:
            print("Warning: failed to save weather info:", e)

        # Reset world and clean up actors (same as your main tail)
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        actors = world.get_actors()
        print(f"Destroy {len(actors)} actors")
        client.apply_batch([carla.command.DestroyActor(x) for x in actors])

        time.sleep(0.5)
        print(f"Generation of recording with seed {args.seed} complete")

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

    def _find_shoulder_spawn_transforms(self, world: World, *, min_width_m: float = 1.8) -> list[carla.Transform]:
        """
        Return transforms along Shoulder lanes with approx given width.
        Uses waypoints to align vehicles in driving direction.
        """
        amap = world.get_map()
        # generate shoulder waypoints roughly every 2.5m (fine-grained)
        waypoints = amap.generate_waypoints(2.5)
        lane_utils = _LaneUtils(amap)
        all_lanes = _BlockBuilder.collect_all_lanes_waypoints(waypoints)
        shoulder_lanes = list(
            filter(lambda l: (
                    not l.is_junction and l.lane_type == carla.LaneType.Shoulder and l.lane_width >= min_width_m),
                   all_lanes))
        all_shoulder_lane_waypoints = list(
            flatten(map(lambda l: map(lambda tupl: tupl[1], lane_utils.get_all_waypoints_for_lane(l)), shoulder_lanes)))
        return list(map(lambda l: l.transform, all_shoulder_lane_waypoints))


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--seed', type=int, default=0)
    argparser.add_argument('--length-of-run', type=float, default=5.0)
    argparser.add_argument('--number-of-vehicles', type=int, default=200)
    argparser.add_argument('--number-of-walkers', type=int, default=30)
    argparser.add_argument('--filterv', default="vehicle.*")
    argparser.add_argument('--generationv', default="All")
    argparser.add_argument('--filterw', default="walker.pedestrian.*")
    argparser.add_argument('--generationw', default="2")
    argparser.add_argument('--no-rendering', action='store_true')

    # Repeatable map candidates; the chosen map is seed-deterministic
    argparser.add_argument('--map', dest='maps', action='append', default=None,
                           help='Candidate map (repeatable). If omitted, server-usable maps are used.')

    # Optional output folder override
    argparser.add_argument('--output-dir', default='', help='Override JSONHelper.RECORDINGS_RUNS_FOLDER')

    args = argparser.parse_args()

    print("Connect to carla simulator")
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    print("Connected to Carla")

    generator = CarlaDataGenerator(client)

    generator.run_recording_generation(
        client,
        seed=args.seed,
        length_minutes=args.length_of_run,
        number_of_vehicles=args.number_of_vehicles,
        number_of_walkers=args.number_of_walkers,
        filterv=args.filterv,
        generationv=args.generationv,
        filterw=args.filterw,
        generationw=args.generationw,
        candidate_maps=args.maps,
        output_dir=(args.output_dir or None),
        no_rendering=bool(args.no_rendering),
    )
