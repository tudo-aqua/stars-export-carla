import argparse
import math
import os
import queue
import sys

import carla
import cv2
import numpy as np
from carla import Vehicle
from carla import World, Client

from data_av_static import MapRasterizer
from helpers.carla_api_helper import CarlaAPIHelper
from helpers.json_helper import JSONHelper


class CarlaCameraRecorder:
    TICK_SECONDS: float = 0.05

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
        print(f">> [IO] Evaluate recorder data at path: '{log_data_path}'")
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
            print(">> [CARLA] The file at path", log_data_path, "is not a CARLA recorder")
            return

        if info.__contains__("not found"):
            print(">> [IO] The file at path", log_data_path, "cannot be found.")
            return

        # ----------
        # Parse recording meta
        # ----------
        # Frequency present in recorder info (not used to drive sim anymore, but kept for reference)
        try:
            recording_frequency = float(info.split('Frame 2 at ')[1].split(' seconds')[0])
        except Exception:
            recording_frequency = 0.0

        # Frames count
        try:
            replay_tick_count = int(info.split('Frames: ')[1].split('Duration')[0])
        except Exception:
            replay_tick_count = 0

        # Duration in seconds (primary driver for how long we run the loop)
        duration_seconds = None
        try:
            duration_seconds = float(info.split('Duration: ')[1].split(' seconds')[0])
        except Exception:
            # Fallback: derive from frames & recording_frequency if duration is not explicitly present
            if replay_tick_count > 0 and recording_frequency > 0.0:
                duration_seconds = (replay_tick_count - 1) * recording_frequency
            else:
                duration_seconds = 0.0

        # If no end is provided, use the full recording duration from the recorder info
        if end_at == sys.maxsize:
            end_at = duration_seconds
        CarlaCameraRecorder.END_AT = end_at

        filename_without_extension = os.path.splitext(filename)[0]

        image_save_folder = CarlaCameraRecorder.get_image_save_folder(recording_folder=recording_folder,
                                                                      filename_without_extension=filename_without_extension,
                                                                      vehicle_id=vehicle_id, begin_at=begin_at,
                                                                      end_at=CarlaCameraRecorder.END_AT,
                                                                      bounding_boxes=True)
        if os.path.exists(image_save_folder):
            print(f">> [Recorder] The files were already recorded at {image_save_folder}")
            return
        else:
            os.makedirs(image_save_folder)

        # video_path = os.path.join(destination, "videos")
        #
        # if not os.path.exists(video_path):
        #     os.makedirs(video_path)
        #
        # video_file = os.path.join(video_path, f"{CarlaCameraRecorder.get_video_prefix(filename_without_extension, vehicle_id, begin_at, end_at)}.mp4")
        # print(f"Save video to {video_file}")
        # video = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 20, (width, height))

        # Get map name of recording
        map_name = info.split("Map: ")[1].split("\nDate")[0]

        # Load map from recording
        self.client.load_world(map_name)

        # Get world for later use
        world: World = self.client.get_world()

        # Set synchronous mode settings
        new_settings = world.get_settings()
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = CarlaCameraRecorder.TICK_SECONDS
        world.apply_settings(new_settings)

        image_queue = queue.Queue()

        # Initialize necessary helper classes
        rasterizer = MapRasterizer(world)
        api_helper = CarlaAPIHelper(self.client, world, rasterizer)

        print(">> [Recorder] Start with simulation replay")

        # Start replay of simulation
        api_helper.start_replaying(log_data_path)

        # A tick is necessary for the server to process the replay_file command
        world.tick()

        vehicles = []

        while len(vehicles) == 0:
            vehicles = api_helper.get_vehicles()
            world.tick()

        vehicle_id_mapping = CarlaAPIHelper.create_recorder_to_sim_id_map(world, info, position_tolerance_m=1)
        if len(vehicle_id_mapping) != len(vehicles):
            mapped_sim_ids = set(vehicle_id_mapping.values())
            unmapped_vehicles = [v.id for v in vehicles if v.id not in mapped_sim_ids]
            print(">> [CARLA] The vehicle id mapping is not equal to the vehicle id")
            print(f">> [CARLA] Mapped {len(vehicle_id_mapping)} recorder id(s) to {len(vehicles)} live vehicle(s); "
                 f"unmapped live vehicle id(s): {unmapped_vehicles}")
            return

        # Get the ego vehicle from the given vehicle id
        if vehicle_id == -1:
            ego_vehicle: Vehicle = list(filter(lambda v: 'hero' in v.attributes['role_name'], vehicles))[0]
        else:
            ego_vehicle: Vehicle = list(filter(lambda v: v.id == vehicle_id_mapping[vehicle_id], vehicles))[0]

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

        ego_cam.listen(lambda t: CarlaCameraRecorder.tmp(image=t, image_queue=image_queue,
                                                         recording_folder=recording_folder,
                                                         filename_without_extension=filename_without_extension,
                                                         vehicle_id=vehicle_id, begin_at=begin_at,
                                                         end_at=end_at,
                                                         recording_frequency=CarlaCameraRecorder.TICK_SECONDS))

        spectator = world.get_spectator()

        # ----------
        # Tick the world for the entire length of the recording (derived from Duration in recorder info)
        # ----------
        total_ticks = int(math.ceil(duration_seconds / CarlaCameraRecorder.TICK_SECONDS))
        # Ensure we run at least one tick if duration rounds to 0
        total_ticks = max(total_ticks, 1)

        for tick in range(1, total_ticks + 1):
            world.tick()
            current_time = CarlaCameraRecorder.TICK_SECONDS * tick
            if current_time < begin_at:
                print(f">> [Recorder] Current time {current_time:.3f}s is not within [{begin_at}, {end_at}]")
                continue
            if current_time > end_at:
                print(f">> [Recorder] Current time {current_time:.3f}s is not within [{begin_at}, {end_at}]")
                break

            image = image_queue.get()

            img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

            # Get the camera matrix
            world_2_camera = np.array(ego_cam.get_transform().get_inverse_matrix())

            for npc in world.get_actors().filter('*vehicle*'):
                bb = npc.bounding_box

                # Calculate the dot product between the forward vector
                # of the vehicle and the vector between the vehicle
                # and the other vehicle. We threshold this dot product
                # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                forward_vec = ego_vehicle.get_transform().get_forward_vector()
                ray = npc.get_transform().location - ego_vehicle.get_transform().location

                if forward_vec.dot(ray) > 0:
                    verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                    edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2],
                             [7, 3]]
                    for edge in edges:
                        K = CarlaCameraRecorder.build_projection_matrix(width, height, 105)
                        K_b = CarlaCameraRecorder.build_projection_matrix(width, height, 105, is_behind_camera=True)

                        p1 = CarlaCameraRecorder.get_image_point(verts[edge[0]], K, world_2_camera)
                        p2 = CarlaCameraRecorder.get_image_point(verts[edge[1]], K, world_2_camera)

                        p1_in_canvas = CarlaCameraRecorder.point_in_canvas(p1, height, width)
                        p2_in_canvas = CarlaCameraRecorder.point_in_canvas(p2, height, width)

                        if not p1_in_canvas and not p2_in_canvas:
                            continue

                        ray0 = verts[edge[0]] - ego_cam.get_transform().location
                        ray1 = verts[edge[1]] - ego_cam.get_transform().location
                        cam_forward_vec = ego_cam.get_transform().get_forward_vector()

                        # One of the vertex is behind the camera
                        if not (cam_forward_vec.dot(ray0) > 0):
                            p1 = CarlaCameraRecorder.get_image_point(verts[edge[0]], K_b, world_2_camera)
                        if not (cam_forward_vec.dot(ray1) > 0):
                            p2 = CarlaCameraRecorder.get_image_point(verts[edge[1]], K_b, world_2_camera)

                        cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (255, 0, 0, 255), 1)

            cv2.imshow('ImageWindowName', img)

            image_name = "%.6d.jpg" % CarlaCameraRecorder.COUNTER
            cv2.imwrite(os.path.join(image_save_folder, image_name), img)

            CarlaCameraRecorder.COUNTER += 1

            current_tick = recording_frequency * tick
            if current_tick < begin_at:
                print(f">> [Recorder] Current tick {current_tick} is not within [{begin_at}, {end_at}]")
                continue
            if current_tick > end_at:
                print(f">> [Recorder] Current tick {current_tick} is not within [{begin_at}, {end_at}]")
                break
            transform = ego_cam.get_transform()
            spectator.set_transform(carla.Transform(transform.location, transform.rotation))
            print(f">> [CARLA] Tick {tick:05d} of {total_ticks:05d}. Simulation Time: {current_time:.3f}s")

        self.client.reload_world()

        if filename.endswith(".zip"):
            JSONHelper.delete_file(log_data_path)

    COUNTER: int = 0
    CURRENTLY_SAVING_IMAGE = False
    END_AT = 0.0

    @staticmethod
    def tmp(image, image_queue, recording_folder,
            filename_without_extension,
            vehicle_id, begin_at,
            end_at, recording_frequency):
        image_queue.put(image)
        # CarlaCameraRecorder.save_image_data(image=image, recording_folder=recording_folder,
        #                                     filename_without_extension=filename_without_extension,
        #                                     vehicle_id=vehicle_id, begin_at=begin_at,
        #                                     end_at=end_at, recording_frequency=recording_frequency)


    @staticmethod
    def build_projection_matrix(w, h, fov, is_behind_camera=False):
        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        K = np.identity(3)

        if is_behind_camera:
            K[0, 0] = K[1, 1] = -focal
        else:
            K[0, 0] = K[1, 1] = focal

        K[0, 2] = w / 2.0
        K[1, 2] = h / 2.0
        return K

    @staticmethod
    def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

    @staticmethod
    def point_in_canvas(pos, img_h, img_w):
        """Return true if point is in canvas"""
        if (pos[0] >= 0) and (pos[0] < img_w) and (pos[1] >= 0) and (pos[1] < img_h):
            return True
        return False

    @staticmethod
    def get_video_prefix(filename_without_extension: str, vehicle_id: int, begin_at: float, end_at: float,
                         bounding_boxes: bool = False) -> os.path:
        return f"{filename_without_extension}-vehicle_{vehicle_id}_range[{begin_at}, {end_at}]{'_bounding_boxes' if bounding_boxes else ''}"

    @staticmethod
    def get_image_save_folder(recording_folder: str, filename_without_extension: str, vehicle_id: int, begin_at: float,
                              end_at: float, bounding_boxes: bool = False) -> os.path:
        folder_path = CarlaCameraRecorder.get_video_prefix(filename_without_extension, vehicle_id, begin_at=begin_at,
                                                           end_at=end_at, bounding_boxes=bounding_boxes)
        return os.path.join(recording_folder, JSONHelper.VIDEO_IMAGE_FOLDER, folder_path)

    @staticmethod
    def get_video_save_folder(recording_folder: str) -> os.path:
        return os.path.join(recording_folder, JSONHelper.VIDEO_FOLDER)

    @staticmethod
    def save_image_data(image, recording_folder: str, filename_without_extension: str, vehicle_id: int, begin_at: float,
                        end_at: float, recording_frequency: float):
        CarlaCameraRecorder.COUNTER += 1
        current_tick_time = recording_frequency * CarlaCameraRecorder.COUNTER
        if begin_at <= current_tick_time <= end_at:
            image_name = "%.6d.jpg" % CarlaCameraRecorder.COUNTER
            image_save_folder = CarlaCameraRecorder.get_image_save_folder(
                recording_folder=recording_folder,
                                                                          filename_without_extension=filename_without_extension,
                                                                          vehicle_id=vehicle_id,
                                                                          begin_at=begin_at,
                end_at=end_at
            )
            recording_path = os.path.join(image_save_folder, image_name)
            # Ensure directory exists
            os.makedirs(image_save_folder, exist_ok=True)
            print(f">> [IO] Save image {image_name}")
            image.save_to_disk(recording_path)

    @staticmethod
    def save_video(recording_folder: str, filename_without_extension: str, vehicle_id: int, begin_at: float,
                   end_at: float, bounding_boxes: bool = False):
        image_folder = CarlaCameraRecorder.get_image_save_folder(recording_folder, filename_without_extension,
                                                                 vehicle_id,
                                                                 begin_at=begin_at, end_at=end_at, bounding_boxes=bounding_boxes)

        video_folder = CarlaCameraRecorder.get_video_save_folder(recording_folder)
        if not os.path.exists(video_folder):
            os.makedirs(video_folder)

        video_name = f"{CarlaCameraRecorder.get_video_prefix(filename_without_extension, vehicle_id, begin_at, end_at, bounding_boxes)}.mp4"
        video_path = os.path.join(video_folder, video_name)

        if os.path.exists(video_path):
            print(f">> [Recorder] The video was already produced at {video_path}")
            return

        images_in_folder = os.listdir(image_folder) if os.path.exists(image_folder) else []
        if len(images_in_folder) == 0:
            print(">> [IO] There are no images to save as video")
            return

        images = [img for img in images_in_folder if img.endswith(".jpg")]

        images = images[0:-1]

        frame = cv2.imread(os.path.join(image_folder, images[0]))
        height, width, layers = frame.shape

        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        video = cv2.VideoWriter(video_path, fourcc, 20, (width, height))
        print(f">> [IO] Save video to {video_path}")

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
        help='Time (seconds) at which the video should start')
    argparser.add_argument(
        '-e', '--end_at',
        metavar='E',
        type=float,
        default=sys.maxsize,
        help='Time (seconds) at which the video should end')
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

    print('Proceed with the following arguments:')
    print(f'Path: {path}, Vehicle Id: {vehicle_id}, Time Range: [{begin_at}, {end_at}] ')
    print(f'Video Width: {video_width}, Video Height: {video_height}')
    print(f'Fixed tick: {CarlaCameraRecorder.TICK_SECONDS}s (synchronous mode)')

    print("Connect to Carla")

    # Find carla simulator at localhost on port 2000
    client = carla.Client('localhost', 2000)

    # Try to connect for 60 seconds. Fail if not successful
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
                                       end_at=CarlaCameraRecorder.END_AT, bounding_boxes=True)
