import argparse
import os
import sys

import carla
import cv2
from carla import Vehicle
from carla import World, Client

from helpers.carla_api_helper import CarlaAPIHelper
from helpers.json_helper import JSONHelper
from helpers.map_rasterizer import MapRasterizer


class CarlaCameraRecorder:

    def __init__(self, carla_client: Client):
        self.ego_vehicle = None
        self.client = carla_client
        self.world: World = carla_client.get_world()
        self.map = self.world.get_map()

    def record_camera_in_simulation_run(self, recording_folder: str, path: str, vehicle_id: int, width: int,
                                        height: int, begin_at: float,
                                        end_at: float) -> None:
        """
        Record the camera in the given simulation run for the given vehicle_id and save it as an mp4 file
        @param path: The path of the recording that should be recorded
        @param vehicle_id: The id of the vehicle that should be recorded
        """
        log_data_path = os.path.abspath(path)
        print("Evaluate recorder data at path", log_data_path)
        filename = os.path.basename(path)

        # Extract log file when path is zipped
        if filename.endswith(".zip"):
            # Unzip recorder file at path
            JSONHelper.extract_from_zip(log_data_path)
            # Log file path
            log_data_path = str(log_data_path).replace(".zip", ".log")
        # Check data from carla recording for validity
        info = self.client.show_recorder_file_info(log_data_path, True)
        if info == "File is not a CARLA recorder\n":
            print("The file at path", log_data_path, "is not a CARLA recorder")
            return

        if info.__contains__("not found"):
            print("The file at path", log_data_path, "cannot be found.")
            return

        # Get recording frequency in the recorded file using the recorder_file_info and split
        recording_frequency = float(info.split("Frame 2 at ")[1].split(" seconds")[0])

        # Get count of all ticks in the recorded file using the recorder_file_info and split
        replay_tick_count = int(info.split("Frames: ")[1].split("Duration")[0])

        if end_at == sys.maxsize:
            end_at = (replay_tick_count - 1) * recording_frequency
        CarlaCameraRecorder.END_AT = end_at

        filename_without_extension = os.path.splitext(filename)[0]

        image_save_folder = CarlaCameraRecorder.get_image_save_folder(recording_folder=recording_folder,
                                                                      filename_without_extension=filename_without_extension,
                                                                      vehicle_id=vehicle_id, begin_at=begin_at,
                                                                      end_at=CarlaCameraRecorder.END_AT)
        if os.path.exists(image_save_folder):
            print(f"The files were already recorded at {image_save_folder}")
            return

        # Get map name of recording
        map_name = info.split("Map: ")[1].split("\nDate")[0]

        # Load map from recording
        self.client.load_world(map_name)

        # Get world for later use
        world: World = self.client.get_world()

        # Set synchronous mode settings
        new_settings = world.get_settings()
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = recording_frequency
        world.apply_settings(new_settings)

        # Initialize necessary helper classes
        rasterizer = MapRasterizer(world)
        api_helper = CarlaAPIHelper(client, world, rasterizer)

        # Start replay of simulation
        api_helper.start_replaying(log_data_path)
        # A tick is necessary for the server to process the replay_file command
        world.tick()

        print("Start with simulation replay")

        vehicles = []

        while len(vehicles) == 0:
            vehicles = api_helper.get_vehicles()
            world.tick()

        # Get the ego vehicle from the given vehicle id
        if vehicle_id == -1:
            ego_vehicle: Vehicle = list(filter(lambda v: 'ego' in v.attributes['role_name'], vehicles))[0]
        else:
            ego_vehicle: Vehicle = list(filter(lambda v: v.id == vehicle_id, vehicles))[0]

        # --------------
        # Spawn attached RGB camera
        # --------------
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(width))
        cam_bp.set_attribute("image_size_y", str(height))
        cam_bp.set_attribute("fov", str(105))
        cam_location = carla.Location(-2, 0, 3)
        cam_rotation = carla.Rotation(0, 0, 0)
        cam_transform = carla.Transform(cam_location, cam_rotation)
        ego_cam = world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle,
                                    attachment_type=carla.AttachmentType.Rigid)
        ego_cam.listen(
            lambda image: CarlaCameraRecorder.save_image_data(image=image, recording_folder=recording_folder,
                                                              filename_without_extension=filename_without_extension,
                                                              vehicle_id=vehicle_id, begin_at=begin_at,
                                                              end_at=end_at, recording_frequency=recording_frequency))
        spectator = world.get_spectator()

        # Tick the world for each frame in the replay
        for tick in range(1, replay_tick_count):
            # Advance simulation by one tick
            world.tick()
            current_tick = recording_frequency * tick
            if current_tick < begin_at:
                print(f"Current tick {current_tick} is not within [{begin_at}, {end_at}]")
                continue
            if current_tick > end_at:
                print(f"Current tick {current_tick} is not within [{begin_at}, {end_at}]")
                break
            transform = ego_cam.get_transform()
            spectator.set_transform(carla.Transform(transform.location, transform.rotation))
            print(f"Tick {tick} of {replay_tick_count}. Simulation Tick: {current_tick}")
            while CarlaCameraRecorder.COUNTER < tick:
                x = ""

        client.reload_world()

        if filename.endswith(".zip"):
            JSONHelper.delete_file(log_data_path)

    COUNTER: int = 0
    CURRENTLY_SAVING_IMAGE = False
    END_AT = 0.0

    @staticmethod
    def get_video_prefix(filename_without_extension: str, vehicle_id: int, begin_at: float, end_at: float) -> os.path:
        return f"{filename_without_extension}-vehicle_{vehicle_id}_range[{begin_at}, {end_at}]"

    @staticmethod
    def get_image_save_folder(recording_folder: str, filename_without_extension: str, vehicle_id: int, begin_at: float,
                              end_at: float) -> os.path:
        folder_path = CarlaCameraRecorder.get_video_prefix(filename_without_extension, vehicle_id, begin_at=begin_at,
                                                           end_at=end_at)
        return os.path.join(recording_folder, JSONHelper.VIDEO_IMAGE_FOLDER, folder_path)

    @staticmethod
    def get_video_save_folder(recording_folder: str) -> os.path:
        return os.path.join(recording_folder, JSONHelper.VIDEO_FOLDER)

    @staticmethod
    def save_image_data(image, recording_folder: str, filename_without_extension: str, vehicle_id: int, begin_at: float,
                        end_at: float, recording_frequency: float):
        CarlaCameraRecorder.COUNTER += 1
        current_tick = recording_frequency * CarlaCameraRecorder.COUNTER
        if begin_at <= current_tick <= end_at:
            image_name = "%.6d.jpg" % CarlaCameraRecorder.COUNTER
            image_save_folder = CarlaCameraRecorder.get_image_save_folder(recording_folder=recording_folder,
                                                                          filename_without_extension=filename_without_extension,
                                                                          vehicle_id=vehicle_id,
                                                                          begin_at=begin_at,
                                                                          end_at=end_at)
            recording_path = os.path.join(image_save_folder, image_name)
            print(f"Save image {image_name}")
            image.save_to_disk(recording_path)

    @staticmethod
    def save_video(recording_folder: str, filename_without_extension: str, vehicle_id: int, begin_at: float,
                   end_at: float):
        image_folder = CarlaCameraRecorder.get_image_save_folder(recording_folder, filename_without_extension,
                                                                 vehicle_id,
                                                                 begin_at=begin_at, end_at=end_at)

        video_folder = CarlaCameraRecorder.get_video_save_folder(recording_folder)
        if not os.path.exists(video_folder):
            os.makedirs(video_folder)

        video_name = f"{CarlaCameraRecorder.get_video_prefix(filename_without_extension, vehicle_id, begin_at, end_at)}.mp4"
        video_path = os.path.join(video_folder, video_name)

        if os.path.exists(video_path):
            print(f"The video was already produced at {video_path}")
            return

        images_in_folder = os.listdir(image_folder)
        if images_in_folder.__sizeof__() == 0:
            print("There are no images to save as video")
            return

        images = [img for img in images_in_folder if img.endswith(".jpg")]

        images = images[0:-1]

        frame = cv2.imread(os.path.join(image_folder, images[0]))
        height, width, layers = frame.shape

        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        video = cv2.VideoWriter(video_path, fourcc, 20, (width, height))
        print(f"Save video to {video_path}")

        for image in images:
            video.write(cv2.imread(os.path.join(image_folder, image)))

        cv2.destroyAllWindows()
        video.release()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-p', '--path',
        metavar='P',
        type=str,
        default='.\..\scenarios\scenario_1\scenario_2024_30_05_15_48_41.log',
        help='Set path of the recording file that should be recorded')
    argparser.add_argument(
        '-v', '--vehicle-id',
        metavar='V',
        type=int,
        default=-1,
        help='For which vehicle id should the camera be recorded?')
    argparser.add_argument(
        '-x', '--width',
        metavar='V',
        type=int,
        default=640,
        help='Width of the resulting video')
    argparser.add_argument(
        '-y', '--height',
        metavar='V',
        type=int,
        default=480,
        help='Height of the resulting video')
    argparser.add_argument(
        '-b', '--begin_at',
        metavar='B',
        type=float,
        default=0.0,
        help='Tick at which the video should start')
    argparser.add_argument(
        '-e', '--end_at',
        metavar='E',
        type=float,
        default=sys.maxsize,
        help='Tick at which the video should end')
    argparser.add_argument(
        '-d', '--destination',
        metavar='D',
        type=str,
        default="",
        help='The destination folder where the video should be saved')
    args = argparser.parse_args()

    path = args.path
    destination = args.destination

    vehicle_id = args.vehicle_id
    begin_at = args.begin_at
    end_at = args.end_at

    video_width = args.width
    video_height = args.height

    print("Proceed with the following arguments:")
    print(f"Path: {path}, Vehicle Id: {vehicle_id}, Tick Range: [{begin_at}, {end_at}] ")
    print(f"Video Width: {video_width}, Video Height: {video_height}")

    print("Connect to Carla")

    # Find carla simulator at localhost on port 2000
    client = carla.Client('localhost', 2000)

    # Try to connect for 10 seconds. Fail if not successful
    client.set_timeout(60.0)
    recorder = CarlaCameraRecorder(carla_client=client)
    print("Connected to carla")
    try:
        recorder.record_camera_in_simulation_run(recording_folder=destination, path=path, vehicle_id=vehicle_id,
                                                 width=video_width,
                                                 height=video_height,
                                                 begin_at=begin_at, end_at=end_at)
        print("Done with monitoring the recording")
    finally:
        print("Convert images to video")
        filename_without_extension = os.path.splitext(os.path.basename(path))[0]
        CarlaCameraRecorder.save_video(destination, filename_without_extension=filename_without_extension,
                                       vehicle_id=vehicle_id,
                                       begin_at=begin_at,
                                       end_at=CarlaCameraRecorder.END_AT)
