from typing import Tuple

import carla


def connect() -> carla.Client:
    # Connect to Carla
    print("Connect to Carla")
    client = carla.Client('localhost', 2000)

    # Try to connect for 10 seconds. Fail if not successful
    client.set_timeout(60.0)
    print("Connected to Carla")
    return client

def load_world(client: carla.Client, begin_at: float, end_at: float, logfile: str) -> (carla.World, Tuple[int, float, float, float]):
    # Check data from carla recording for validity
    info = client.show_recorder_file_info(logfile, True)
    if info == "File is not a CARLA recorder\n":
        print("The file at path", logfile, "is not a CARLA recorder")
        raise RuntimeError()

    # Get recording frequency in the recorded file using the recorder_file_info and split
    recording_frequency = float(info.split("Frame 2 at ")[1].split(" seconds")[0])

    # Get count of all ticks in the recorded file using the recorder_file_info and split
    tick_count = int(info.split("Frames: ")[1].split("Duration")[0])
    end_at = min(end_at, (tick_count - 1) * recording_frequency)
    begin_at = min(begin_at, end_at)

    # Get map name of recording
    map_name = info.split("Map: ")[1].split("\nDate")[0]

    # Load map from recording
    client.load_world(map_name)

    # Get world for later use
    # noinspection PyArgumentList
    world: carla.World = client.get_world()

    # Set synchronous mode settings
    # noinspection PyArgumentList
    new_settings = world.get_settings()
    new_settings.synchronous_mode = True
    new_settings.fixed_delta_seconds = recording_frequency
    world.apply_settings(new_settings)

    return world, (tick_count, begin_at, end_at, recording_frequency)