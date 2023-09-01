from datetime import datetime

import carla
from carla import World

from carla_data_classes import DataActorPosition, TickData, DataLandmarkType
from helpers.carla_api_helper import CarlaAPIHelper

from helpers.map_rasterizer import MapRasterizer

if __name__ == '__main__':
    print("Connect to carla simulator")
    # Find carla simulator at localhost on port 2000
    client = carla.Client('localhost', 2000)
    # Try to connect for 10 seconds. Fail if not successful
    client.set_timeout(10.0)
    world: World = client.get_world()
    print("Connected to carla simulator")
    # Initialize helpers
    _rasterizer = MapRasterizer(carla_world=world)
    api_helper = CarlaAPIHelper(client, world, _rasterizer)
    map_name = world.get_map().name

    # Calculate the static data for the current map
    data_blocks = _rasterizer.load_or_calculate_data_blocks(map_name, map_name, False)

    actors = world.get_actors()
    landmarks = world.get_map().get_all_landmarks()
    valid_landmarks = []
    for landmark in landmarks:
        data_road = _rasterizer.get_data_road(landmark.road_id)
        if data_road.is_junction and DataLandmarkType(int(landmark.type)) == DataLandmarkType.StopSign or DataLandmarkType(int(landmark.type)) == DataLandmarkType.YieldSign:
            valid_landmarks.append(landmark)
    landmark = list(filter(lambda l: l.id == '947', landmarks))[0]
    data_landmark = _rasterizer.get_data_landmark_for_landmark(landmark)
    # Cycle through the landmarks and get their corresponding lane objects
    landmark_road_id = landmark.road_id
    lane_validities = landmark.get_lane_validities()

    # Get world settings for later execution settings
    settings = world.get_settings()
    # settings.synchronous_mode = True
    world.apply_settings(settings)

    # Get the current tick and save it
    first_tick_time = datetime.now()
    last_tick_time = first_tick_time
    ticks = []

    print("Start with tick data")
    # Loop over 1 second and track the first vehicle that is found
    while (datetime.now() - first_tick_time).total_seconds() < 1:
        # Advance simulation by one tick
        world.tick()
        # Update current time duration
        now = datetime.now()
        current_tick = (now - first_tick_time).total_seconds()
        print("Current tick: ", current_tick)
        # Get all vehicles
        vehicles = api_helper.get_vehicles()
        # Get the ego vehicle
        ego_vehicle = vehicles[0]
        # Get the waypoint (and therefore road and lane) for the ego vehicle
        ego_waypoint = _rasterizer.get_waypoint_for_actor(ego_vehicle)
        # Get the block the ego vehicle is in
        ego_block = _rasterizer.get_best_block_for_actor(ego_vehicle)
        # Get all actors that are in the block of the ego vehicle
        actors = api_helper.get_actors_in_block(ego_block)

        actor_positions = []
        # Calculate the actor position for each actor (Vehicle, Pedestrian, TrafficSign, TrafficLight)
        for actor in actors:
            # Transform the carla.Actor into a DataActor
            data_actor = api_helper.get_data_actor_from_actor(actor)
            # Get road and lane information for the current actor
            data_actor_wp = _rasterizer.get_waypoint_for_actor(actor)
            distance_to_start_of_lane = _rasterizer.get_distance_to_start_of_lane_from_waypoint(data_actor_wp)
            # Create ActorPosition object
            actor_position = DataActorPosition(position_on_lane=distance_to_start_of_lane,
                                               road_id=data_actor_wp.road_id, lane_id=data_actor_wp.lane_id,
                                               actor=data_actor)
            # Save the ActorPosition object in the list
            actor_positions.append(actor_position)
        # Collect all ActorPositions and wrap them in a TickData object
        tick = TickData(current_tick=current_tick, actor_positions=actor_positions)
        # Save the TickData object in the list
        ticks.append(tick)
        last_tick_time = now
    api_helper.save_dynamic_data(ticks)
