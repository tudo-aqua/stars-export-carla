import os
import queue
import sys
from typing import Any, Tuple, List

import carla
import cv2
import numpy as np
from carla import World, Vehicle, Vector3D

from helpers.carla_api_helper import CarlaAPIHelper
from helpers.map_rasterizer import MapRasterizer
from helper import build_projection_matrix, get_image_point, point_in_canvas
from python.camera_recorder.CameraPosition import CameraPosition
from python.camera_recorder.SafetyBoxStyle import SafetyBoxStyle


# noinspection PyDefaultArgument
class CarlaCameraRecorder:
    def __init__(self,
                 output_dir: str,
                 img_width: int = 1920,
                 img_height: int = 1080,
                 fov: int = 105,
                 vehicle_id: int = -1,
                 begin_at: float = 0,
                 end_at: float = sys.maxsize,
                 camera_positions: List[Tuple[CameraPosition, bool]] = [],
                 bounding_box_color: Tuple[int, int, int, int] = (0, 255, 0, 255),
                 render_safety_boxes: bool = False,
                 safety_box_style: SafetyBoxStyle = None,
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
        self.camera_positions = camera_positions
        self.bounding_box_color = bounding_box_color
        self.render_safety_boxes = render_safety_boxes
        self.safety_box_style = safety_box_style
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

    def __start_replay__(self, logfile: str, api_helper: CarlaAPIHelper, world: World) -> List[Tuple[Any, bool, queue.Queue]]:
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
        cameras: List[Tuple[Any, bool, queue.Queue]] = []
        for (camera, show_bounding_box) in self.camera_positions:
            # noinspection PyArgumentList
            cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
            cam_bp.set_attribute("image_size_x", str(self.img_width))
            cam_bp.set_attribute("image_size_y", str(self.img_height))
            cam_bp.set_attribute("fov", str(self.fov))

            # Set camera position
            cam_location = camera.value[0]
            cam_rotation = camera.value[1]
            cam_transform = carla.Transform(cam_location, cam_rotation)

            # Spawn camera
            ego_cam = world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle,
                                        attachment_type=carla.AttachmentType.Rigid)

            # Create queue for image
            image_queue = queue.Queue()

            ego_cam.listen(image_queue.put)
            cameras.append((ego_cam, show_bounding_box, image_queue))

        # noinspection PyArgumentList
        return cameras

    def __create_safety_box(self, vertices: List[Vector3D]) ->  List[Tuple[list, list, tuple[int, int, int, int]]]:
        bottom_back: Vector3D = vertices[2]  # Bottom Right Back
        top_back: Vector3D = vertices[3]  # Top Right Back
        bottom_front: Vector3D = vertices[6]  # Bottom Right Front
        top_front: Vector3D = vertices[7]  # Top Right Front

        # vec(Bottom Right Front <- Bottom Left Front)
        vec: Vector3D = (bottom_front - vertices[4]).make_unit_vector()

        shifted_bottom_back: Vector3D = bottom_back + 1.5 * vec
        shifted_top_back: Vector3D = top_back + 1.5 * vec
        shifted_bottom_front: Vector3D = bottom_front + 1.5 * vec
        shifted_top_front: Vector3D = top_front + 1.5 * vec

        new_edge_connections = [(bottom_back, shifted_bottom_back),
                                (bottom_front, shifted_bottom_front),
                                (shifted_bottom_front, shifted_bottom_back),]
        match self.safety_box_style:
            case SafetyBoxStyle.BOX:
                new_edge_connections.extend([
                    (top_back, shifted_top_back),
                    (top_front, shifted_top_front),
                    (shifted_bottom_front, shifted_top_front),
                    (shifted_top_front, shifted_top_back),
                    (shifted_top_back, shifted_bottom_back),
                ])

            case SafetyBoxStyle.X:
                new_edge_connections.extend([
                    (bottom_back, shifted_bottom_front),
                    (shifted_bottom_back, bottom_front),
                ])

            case SafetyBoxStyle.HATCHING:
                for i in np.arange(0, 1.25, 0.25):
                    new_edge_connections.extend([
                        (bottom_back + i*vec, bottom_front + (i+0.25)*vec),
                    ])

        return list(map(lambda ec: (ec[0], ec[1], self.safety_bounding_box_color), new_edge_connections))

    def __render_bounding_boxes__(self, world: World, camera: Any, image: np.ndarray) -> None:
        world_to_camera = np.array(camera.get_transform().get_inverse_matrix())

        # noinspection PyArgumentList
        for vehicle in world.get_actors().filter('*vehicle*'):
            bounding_box = vehicle.bounding_box

            camera_forward_vec = camera.get_transform().get_forward_vector()
            ray = vehicle.get_transform().location - camera.get_transform().location

            if camera_forward_vec.dot(ray) > 0:
                # Get vertices of bounding box
                vertices = [v for v in bounding_box.get_world_vertices(vehicle.get_transform())]

                k = build_projection_matrix(width=self.img_width, height=self.img_height, fov=self.fov)
                k_behind = build_projection_matrix(width=self.img_width, height=self.img_height, fov=self.fov,
                                                   is_behind_camera=True)

                edge_connections = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

                edges = list(
                    map(lambda ec: (vertices[ec[0]], vertices[ec[1]], self.bounding_box_color), edge_connections))

                if self.render_safety_boxes:
                    edges.extend(self.__create_safety_box(vertices))

                # Calculate edges to draw
                for (loc1, loc2, color) in edges:
                    # Get points of edge
                    p1 = get_image_point(loc=loc1, k=k, world_to_camera=world_to_camera)
                    p2 = get_image_point(loc=loc2, k=k, world_to_camera=world_to_camera)

                    # # Skip invisible edges
                    if (not point_in_canvas(pos=p1, img_width=self.img_width, img_height=self.img_height)
                            and not point_in_canvas(pos=p2, img_width=self.img_width, img_height=self.img_height)):
                        continue

                    ray0 = loc1 - camera.get_transform().location
                    ray1 = loc2 - camera.get_transform().location

                    # One of the vertex is behind the camera
                    if not (camera_forward_vec.dot(ray0) > 0):
                        p1 = get_image_point(loc1, k_behind, world_to_camera)
                    if not (camera_forward_vec.dot(ray1) > 0):
                        p2 = get_image_point(loc2, k_behind, world_to_camera)

                    # Draw edge
                    cv2.line(image, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 1)

    def __record_ticks__(self, world: World, cameras: List[Tuple[Any, bool, queue.Queue]], recording_frequency: float,
                         output: str):
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
            for idx, (camera, show_bounding_boxes, image_queue) in enumerate(cameras):
                image = image_queue.get()
                image = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

                # Add bounding boxes to the image
                if show_bounding_boxes:
                    self.__render_bounding_boxes__(world=world, camera=camera, image=image)

                if self.show_preview:
                    cv2.imshow(f'Carla image preview CAM {idx}', image)
                    cv2.waitKey(1)

                # Save the image
                image_name = "%.6d.jpg" % tick
                cv2.imwrite(os.path.join(os.path.join(output, f"CAM_{idx}"), image_name), image)

                # Set the spectator to the current vehicle
                transform = camera.get_transform()
                spectator.set_transform(carla.Transform(transform.location, transform.rotation))

        cv2.destroyAllWindows()

    def record_images(self, logfile: str) -> List[str]:
        # Create output directory
        output = os.path.join(self.output_dir, "_images\\" + logfile.split('\\')[-1].split('.')[
            0] + f"-vehicle_{self.vehicle_id}_range[{self.begin_at}, {self.end_at}]")
        outputs = []
        if not os.path.exists(output):
            os.makedirs(output)

            for i in range(len(self.camera_positions)):
                out = os.path.join(output, f"CAM_{i}")
                outputs.append(out)
                os.makedirs(out)
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

        # Start replay of simulation
        cameras = self.__start_replay__(logfile=logfile, api_helper=api_helper, world=world)

        # Record all ticks
        self.__record_ticks__(world=world, cameras=cameras, recording_frequency=recording_frequency, output=output)

        return outputs

    def record_videos(self, images_directory: str) -> None:
        output = os.path.join(self.output_dir, "_videos\\")
        if not os.path.exists(output):
            os.makedirs(output)

        scenario = images_directory.split('\\')[-1].split('.')[0]
        output = os.path.join(output, scenario)
        if os.path.exists(output):
            print(f"Warning: The output directory {output} already exists. Skipping.", file=sys.stderr)
            return
        else:
            os.makedirs(output)

        cameras = os.listdir(images_directory)
        images = []
        videos = []
        for camera in cameras:
            images.append([img for img in os.listdir(os.path.join(images_directory, camera)) if img.endswith(".jpg")][0:-1])
            # noinspection PyUnresolvedReferences
            videos.append(cv2.VideoWriter(f"{output}\\{camera}", cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 20, (1920, 1080)))
        # noinspection PyUnresolvedReferences
        videos.append(cv2.VideoWriter(f"{output}\\ALL", cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 20, (1920, 1080)))

        for tick in range(len(images[0])):
            img = []
            for idx, camera in enumerate(cameras):
                img.append(cv2.imread(os.path.join(images_directory, camera, images[idx][tick])))
                videos[idx].write(img[-1])

            img_all = None
            match len(img):
                case 1:
                    break
                case 2:
                    img_all = np.concatenate((img[0], img[1]), axis=1)
                case 3:
                    img_all = np.concatenate((
                        np.concatenate((img[0], img[1]), axis=1),
                        np.concatenate((img[2], np.zeros_like(img[0])), axis=1)
                    ), axis=0)
                case 4:
                    img_all = np.concatenate((
                        np.concatenate((img[0], img[1]), axis=1),
                        np.concatenate((img[2], img[3]), axis=1)
                    ), axis=0)
                case 5:
                    img_all = np.concatenate((
                        np.concatenate((img[0], img[1], img[2]), axis=1),
                        np.concatenate((img[3], img[4], np.zeros_like(img[0])), axis=1)
                    ), axis=0)
                case 6:
                    img_all = np.concatenate((
                        np.concatenate((img[0], img[1], img[2]), axis=1),
                        np.concatenate((img[3], img[4], img[5]), axis=1)
                    ), axis=0)
                case 7:
                    img_all = np.concatenate((
                        np.concatenate((img[0], img[1], img[2]), axis=1),
                        np.concatenate((img[3], img[4], img[5]), axis=1),
                        np.concatenate((img[6], np.zeros_like(img[0]), np.zeros_like(img[0])), axis=1)
                    ), axis=0)
                case 8:
                    img_all = np.concatenate((
                        np.concatenate((img[0], img[1], img[2]), axis=1),
                        np.concatenate((img[3], img[4], img[5]), axis=1),
                        np.concatenate((img[6], img[7], np.zeros_like(img[0])), axis=1)
                    ), axis=0)
                case 9:
                    img_all = np.concatenate((
                        np.concatenate((img[0], img[1], img[2]), axis=1),
                        np.concatenate((img[3], img[4], img[5]), axis=1),
                        np.concatenate((img[6], img[7], img[8]), axis=1)
                    ), axis=0)
                case _:
                    print("Too many cameras for multi-view")
                    break


            if img_all is not None:
                videos[-1].write(img_all)
                if self.show_preview:
                    cv2.imshow('Carla video preview', img_all)
                    cv2.waitKey(1)

        cv2.destroyAllWindows()

        for video in videos:
            video.release()