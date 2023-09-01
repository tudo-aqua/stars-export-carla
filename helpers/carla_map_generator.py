import argparse

import carla
from carla import Client
from carla import World

from helpers.carla_api_helper import CarlaAPIHelper
from helpers.carla_monitor import CarlaMonitor
from helpers.json_helper import JSONHelper
from helpers.map_rasterizer import MapRasterizer


def generate_static_map_data(client: Client, update_existing: bool) -> None:
    """
    Monitor the simulation run of the given file
    @param client: The Carla client
    @param update_existing: Decide whether existing monitor results should be overwritten
    @return: None
    """

    map_names = CarlaAPIHelper.get_usable_maps(client)

    for map_name in map_names:
        print("Analyze map", map_name)
        # Load map from recording
        world: World = client.load_world(map_name)
        client.reload_world()

        print("Current map: ", world.get_map().name)

        # Initialize necessary helper classes
        rasterizer = MapRasterizer(world)

        # Calculate the static data for the current map
        rasterizer.load_or_calculate_data_blocks(map_name=map_name, file_name=map_name, update_existing=update_existing)


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-u', '--update-existing-files',
        metavar='U',
        type=bool,
        default=False,
        help='Decide whether existing files should be updated.')
    args = argparser.parse_args()

    update_existing = args.update_existing_files or CarlaMonitor.FORCE_JSON_FILE_UPDATES
    print("Update existing files:", update_existing)
    print("Connect to Carla")

    try:
        # Find carla simulator at localhost on port 2000
        client = carla.Client('localhost', 2000)

        # Try to connect for 20 seconds. Fail if not successful
        client.set_timeout(60.0)
        print("Connected to carla")

        generate_static_map_data(client=client, update_existing=update_existing)
        print("Done with generating static map data")
    except RuntimeError as err:
        print("Logged failed Carla run in main")
        JSONHelper.log_failed_carla_run(err)
