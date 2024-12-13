import os
import queue
import sys
from typing import Any, Tuple

import carla
import cv2
import numpy as np
from carla import World, Vehicle

from helpers.carla_api_helper import CarlaAPIHelper
from helpers.map_rasterizer import MapRasterizer
from helper import build_projection_matrix, get_image_point, point_in_canvas
from python.camera_recorder.CameraPosition import CameraPosition


class CarlaCameraRecorder:
    def __init__(self,
                 output_dir: str,
                 img_width: int = 1920,
                 img_height: int = 1080,
                 fov: int = 105,
                 vehicle_id: int = -1,
                 begin_at: float = 0,
                 end_at: float = sys.maxsize,
                 camera_position: CameraPosition = None,
                 render_bounding_boxes: bool = False,
                 bounding_box_color: Tuple[int, int, int, int] = (0, 255, 0, 255),
                 render_safety_boxes: bool = False,
                 safety_bounding_box_color: Tuple[int, int, int, int] = (255, 0, 0, 255),
                 show_preview: bool = False):
        self.output_dir = output_dir
        self.img_height = img_height
        self.img_width = img_width
        self.fov = fov
        self.vehicle_id = vehicle_id
        self.begin_at = begin_at
        self.end_at = end_at
        self.tick_count = 0
        self.camera_position = camera_position
        self.render_bounding_boxes = render_bounding_boxes
        self.bounding_box_color = bounding_box_color
        self.render_safety_boxes = render_safety_boxes
        self.safety_bounding_box_color = safety_bounding_box_color
        self.show_preview = show_preview

    @staticmethod
    def __connect__() -> carla.Client:
        # Connect to Carla
        print("Connect to Carla")
        client = carla.Client('localhost', 2000)

        # Try to connect for 10 seconds. Fail if not successful
        client.set_timeout(60.0)
        print("Connected to Carla")
        return client

    def __load_world__(self, client: carla.Client, logfile: str) -> (World, float):
        # Check data from carla recording for validity
        info = client.show_recorder_file_info(logfile, True)
        if info == "File is not a CARLA recorder\n":
            print("The file at path", logfile, "is not a CARLA recorder")
            raise RuntimeError()

        # Get recording frequency in the recorded file using the recorder_file_info and split
        recording_frequency = float(info.split("Frame 2 at ")[1].split(" seconds")[0])

        # Get count of all ticks in the recorded file using the recorder_file_info and split
        self.tick_count = int(info.split("Frames: ")[1].split("Duration")[0])
        self.end_at = min(self.end_at, (self.tick_count - 1) * recording_frequency)
        self.begin_at = min(self.begin_at, self.end_at)

        # Get map name of recording
        map_name = info.split("Map: ")[1].split("\nDate")[0]

        # Load map from recording
        client.load_world(map_name)

        # Get world for later use
        # noinspection PyArgumentList
        world: World = client.get_world()

        # Set synchronous mode settings
        # noinspection PyArgumentList
        new_settings = world.get_settings()
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = recording_frequency
        world.apply_settings(new_settings)

        return world, recording_frequency

    def __start_replay__(self, logfile: str, api_helper: CarlaAPIHelper, world: World, img_queue: queue.Queue):
        print("Start with simulation replay")
        api_helper.start_replaying(logfile)

        # Tick until vehicles are spawned
        vehicles = []
        while len(vehicles) == 0:
            # noinspection PyArgumentList
            world.tick()
            vehicles = api_helper.get_vehicles()

        # Get the ego vehicle from the given vehicle id
        if self.vehicle_id == -1:
            ego_vehicle: Vehicle = list(filter(lambda v: 'ego' in v.attributes['role_name'], vehicles))[0]
        else:
            ego_vehicle: Vehicle = list(filter(lambda v: v.id == self.vehicle_id, vehicles))[0]

        # Spawn attached RGB camera
        # noinspection PyArgumentList
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(self.img_width))
        cam_bp.set_attribute("image_size_y", str(self.img_height))
        cam_bp.set_attribute("fov", str(self.fov))

        # Set camera position
        cam_location = self.camera_position.value[0]
        cam_rotation = self.camera_position.value[1]
        cam_transform = carla.Transform(cam_location, cam_rotation)

        # Spawn camera
        ego_cam = world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle,
                                    attachment_type=carla.AttachmentType.Rigid)

        ego_cam.listen(img_queue.put)

        # noinspection PyArgumentList
        return ego_cam

    def __render_bounding_boxes__(self, world: World, camera: Any, image: np.ndarray) -> None:
        world_to_camera = np.array(camera.get_transform().get_inverse_matrix())

        # noinspection PyArgumentList
        for vehicle in world.get_actors().filter('*vehicle*'):
            bounding_box = vehicle.bounding_box

            # Get vertices of bounding box
            vertices = [v for v in bounding_box.get_world_vertices(vehicle.get_transform())]

            k = build_projection_matrix(width=self.img_width, height=self.img_height, fov=self.fov)
            k_behind = build_projection_matrix(width=self.img_width, height=self.img_height, fov=self.fov, is_behind_camera=True)

            edge_connections = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]
            edges = list(map(lambda ec: (vertices[ec[0]], vertices[ec[1]], self.bounding_box_color), edge_connections))

            # Calculate edges to draw
            for (loc1, loc2, color) in edges:
                # Get points of edge
                p1 = get_image_point(loc=loc1, k=k, world_to_camera=world_to_camera)
                p2 = get_image_point(loc=loc2,  k=k, world_to_camera=world_to_camera)

                # Skip invisible edges
                if (not point_in_canvas(pos=p1, img_width=self.img_width, img_height=self.img_height)
                        and not point_in_canvas(pos=p2, img_width=self.img_width, img_height=self.img_height)):
                    continue

                ray0 = loc1 - camera.get_transform().location
                ray1 = loc2 - camera.get_transform().location
                cam_forward_vec = camera.get_transform().get_forward_vector()

                # One of the vertex is behind the camera
                if not (cam_forward_vec.dot(ray0) > 0):
                    p1 = get_image_point(loc1, k_behind, world_to_camera)
                if not (cam_forward_vec.dot(ray1) > 0):
                    p2 = get_image_point(loc2, k_behind, world_to_camera)

                # Draw edge
                cv2.line(image, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), color, 1)

    def __record_ticks__(self, world: World, camera: Any, image_queue: queue.Queue, recording_frequency: float, output: str):
        # Get the spectator
        # noinspection PyArgumentList
        spectator = world.get_spectator()

        # Tick the world for each frame in the replay
        for tick in range(1, self.tick_count):
            # Advance simulation by one tick
            # noinspection PyArgumentList
            world.tick()

            # Check if the current tick is within the specified range
            current_tick = recording_frequency * tick
            print(f"\rTick {tick} of {self.tick_count}. Simulation Tick: {float(current_tick):.2f}")
            if current_tick < self.begin_at:
                print(f"Current tick {current_tick} is not within [{self.begin_at}, {self.end_at}]")
                continue
            if current_tick > self.end_at:
                print(f"Current tick {current_tick} is not within [{self.begin_at}, {self.end_at}]")
                break

            # Create the image
            image = image_queue.get()
            image = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

            # Add bounding boxes to the image
            if self.render_bounding_boxes:
                self.__render_bounding_boxes__(world=world, camera=camera, image=image)

            if self.show_preview:
                cv2.imshow('Carla image preview', image)
                cv2.waitKey(1)

            # Save the image
            image_name = "%.6d.jpg" % tick
            cv2.imwrite(os.path.join(output, image_name), image)

            # Set the spectator to the current vehicle
            transform = camera.get_transform()
            spectator.set_transform(carla.Transform(transform.location, transform.rotation))

        cv2.destroyAllWindows()

    def record_images(self, logfile: str) -> str:
        # Create output directory
        output = os.path.join(self.output_dir, "_images\\" + logfile.split('\\')[-1].split('.')[
            0] + f"-vehicle_{self.vehicle_id}_range[{self.begin_at}, {self.end_at}]")
        if not os.path.exists(output):
            os.makedirs(output)
        else:
            print(f"Warning: The output directory {output} already exists. Skipping.", file=sys.stderr)
            raise RuntimeError()

        # Connect to Carla
        client = self.__connect__()

        # Load world and recording frequency
        world, recording_frequency = self.__load_world__(client=client, logfile=logfile)

        # Initialize necessary helper classes
        rasterizer = MapRasterizer(carla_world=world)
        api_helper = CarlaAPIHelper(client=client, world=world, rasterizer=rasterizer)

        # Declare image queue
        image_queue = queue.Queue()

        # Start replay of simulation
        camera = self.__start_replay__(logfile=logfile, api_helper=api_helper, world=world, img_queue=image_queue)

        # Record all ticks
        self.__record_ticks__(world=world, camera=camera, image_queue=image_queue,
                              recording_frequency=recording_frequency, output=output)

        return output

    def record_video(self, images_directory: str) -> None:
        output = os.path.join(self.output_dir, "_videos\\")
        if not os.path.exists(output):
            os.makedirs(output)

        output = os.path.join(output, images_directory.split('\\')[-1].split('.')[0] + ".mp4")
        if os.path.exists(output):
            print(f"Warning: The output file {output} already exists. Skipping.", file=sys.stderr)
            return

        images = [img for img in os.listdir(images_directory) if img.endswith(".jpg")][0:-1]
        if len(images) == 0:
            print("There are no images to save as video")
            return

        frame = cv2.imread(os.path.join(images_directory, images[0]))
        height, width, layers = frame.shape

        # noinspection PyUnresolvedReferences
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        video = cv2.VideoWriter(output, fourcc, 20, (width, height))
        print(f"Saving video to {output}")

        last = -1
        for idx, image in enumerate(images):
            progress = int(100 * idx / len(images))
            if progress > last:
                print(f"\rSaving video {progress}%")
                last = progress

            img = cv2.imread(os.path.join(images_directory, image))
            if self.show_preview:
                cv2.imshow('Carla video preview', img)
                cv2.waitKey(1)
            video.write(img)

        cv2.destroyAllWindows()
        video.release()
