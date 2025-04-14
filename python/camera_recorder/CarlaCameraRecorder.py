import os
import queue
import sys
from typing import Any, Tuple, List

import carla
import cv2
import numpy as np

from helpers.carla_api_helper import CarlaAPIHelper
from helpers.map_rasterizer import MapRasterizer
from helper import build_projection_matrix, get_image_point, point_in_canvas, min_distance_to_front
from python.camera_recorder.CameraPosition import CameraPosition
from python.camera_recorder.CarlaConnector import connect, load_world
from python.camera_recorder.SafetyBoxStyle import SafetyBoxStyle


# noinspection PyDefaultArgument,PyUnresolvedReferences
class CarlaCameraRecorder:
    def __init__(self,
                 output_dir: str,
                 img_width: int = 1920,
                 img_height: int = 1080,
                 fov: int = 105,
                 vehicle_id: int = -1,
                 begin_at: float = 0,
                 end_at: float = sys.maxsize,
                 camera_positions: List[Tuple[CameraPosition, bool, bool]] = [],
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

    def __start_replay__(self, logfile: str, output:str, api_helper: CarlaAPIHelper, world: carla.World) -> Tuple[List[Tuple[Any, str, bool, bool, queue.Queue]], carla.Vehicle]:
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
            ego_vehicle: carla.Vehicle = vehicles[1]
        else:
            ego_vehicle: carla.Vehicle = list(filter(lambda v: v.id == self.vehicle_id, vehicles))[0]

        # Spawn attached RGB camera
        # noinspection PyArgumentList
        cameras: List[Tuple[Any, str, bool, bool, queue.Queue]] = []
        for (camera, show_metadata, show_bounding_box) in self.camera_positions:
            out = os.path.join(output, f"CAM_{camera.name + ('_META' if show_metadata else '') + ('_BB' if show_bounding_box else '')}")
            os.makedirs(out)

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
            cameras.append((ego_cam, out, show_metadata, show_bounding_box, image_queue))

        # noinspection PyArgumentList
        return cameras, ego_vehicle

    def __create_safety_box_right(self, vertices: List[carla.Vector3D]) ->  List[Tuple[list, list, tuple[int, int, int, int]]]:
        bottom_back = vertices[2]  # Bottom Right Back
        top_back = vertices[3]  # Top Right Back
        bottom_front = vertices[6]  # Bottom Right Front
        top_front = vertices[7]  # Top Right Front

        # vec(Bottom Right Front <- Bottom Left Front)
        vec = (bottom_front - vertices[4]).make_unit_vector()

        shifted_bottom_back = bottom_back + 1.5 * vec
        shifted_top_back = top_back + 1.5 * vec
        shifted_bottom_front = bottom_front + 1.5 * vec
        shifted_top_front = top_front + 1.5 * vec

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

    def __create_safety_box_front(self, vertices: List[carla.Vector3D], safety_distance: float) ->  List[Tuple[list, list, tuple[int, int, int, int]]]:
        bottom_right_front = vertices[6]  # Bottom Right Front
        top_right_front = vertices[7]  # Top Right Front
        top_left_front = vertices[5]  # Top Left Front
        bottom_left_front = vertices[4]  # Bottom Left Front

        # vec(Bottom Right Front <- Bottom Right Back)
        vec = (bottom_right_front - vertices[2]).make_unit_vector() * safety_distance

        shifted_bottom_right_front = bottom_right_front + vec
        shifted_top_right_front = top_right_front + vec
        shifted_top_left_front = top_left_front + vec
        shifted_bottom_left_front = bottom_left_front + vec

        new_edge_connections = [(bottom_left_front, shifted_bottom_left_front),
                                (shifted_bottom_left_front, shifted_bottom_right_front),
                                (shifted_bottom_right_front, bottom_right_front)]

        match self.safety_box_style:
            case SafetyBoxStyle.BOX:
                new_edge_connections.extend([
                    (top_left_front, shifted_top_left_front),
                    (shifted_top_left_front, shifted_top_right_front),
                    (shifted_top_right_front, top_right_front),
                    (shifted_bottom_right_front, shifted_top_right_front),
                    (shifted_bottom_left_front, shifted_top_left_front),
                ])

            case SafetyBoxStyle.X:
                new_edge_connections.extend([
                    (bottom_left_front, shifted_bottom_right_front),
                    (bottom_right_front, shifted_bottom_left_front),
                ])

            case SafetyBoxStyle.HATCHING:
                for i in np.arange(0, 1, 0.10):
                    new_edge_connections.extend([
                        (bottom_left_front + i*vec, bottom_right_front + (i+0.10)*vec),
                    ])

        return list(map(lambda ec: (ec[0], ec[1], self.safety_bounding_box_color), new_edge_connections))

    def __render_bounding_boxes__(self, world: carla.World, camera: Any, safety_distance: float, image: np.ndarray) -> None:
        world_to_camera = np.array(camera.get_transform().get_inverse_matrix())

        # noinspection PyArgumentList
        b = True
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

                if self.render_safety_boxes and not b:
                    edges.extend(self.__create_safety_box_right(vertices))
                    edges.extend(self.__create_safety_box_front(vertices=vertices, safety_distance=safety_distance))

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
                    cv2.line(image, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 2)
                    #cv2.putText(image, text=f"{loc1}, {loc2}", bottomLeftOrigin=(int(p1[0]), int(p1[1])))
            b = False

    @staticmethod
    def __render_metadata__(tick: int, time: float, velocity: float, safety_distance: float, image: np.ndarray) -> None:
        cv2.putText(image, f"Tick: {tick}", (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, f"Time: {time:.2f} s", (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, f"Velocity: {velocity:.2f} m/s", (10, 70), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, f"Safety distance: {safety_distance:.2f} m", (10, 90), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)

    # noinspection PyArgumentList
    def __record_ticks__(self, world: carla.World, cameras: List[Tuple[Any, str, bool, bool, queue.Queue]], ego_vehicle: carla.Vehicle, recording_frequency: float):
        # Get the spectator
        spectator = world.get_spectator()

        last_location = ego_vehicle.get_location()

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

            new_location = ego_vehicle.get_location()
            velocity = new_location.distance(last_location) / recording_frequency
            safety_distance = min_distance_to_front(velocity)
            last_location = new_location

            # Create the image
            for idx, (camera, directory, render_metadata, render_bounding_box, image_queue) in enumerate(cameras):
                image = image_queue.get()
                image = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

                if render_metadata:
                    self.__render_metadata__(tick=tick, time=current_tick, velocity=velocity, safety_distance=safety_distance, image=image)
                if render_bounding_box:
                    self.__render_bounding_boxes__(world=world, camera=camera, safety_distance=safety_distance, image=image)

                if self.show_preview:
                    cv2.imshow(f'Carla image preview CAM {idx}', image)
                    cv2.waitKey(1)

                # Save the image
                image_name = "%.6d.jpg" % tick
                cv2.imwrite(os.path.join(directory, image_name), image)

                # Set the spectator to the current vehicle
                transform = camera.get_transform()
                spectator.set_transform(carla.Transform(transform.location, transform.rotation))

        cv2.destroyAllWindows()

    def record_images(self, logfile: str) -> None:
        # Create output directory
        output = os.path.join(self.output_dir, "_images\\" + logfile.split('\\')[-1].split('.')[
            0] + f"-vehicle_{self.vehicle_id}_range[{self.begin_at}, {self.end_at}]")

        if not os.path.exists(output):
            os.makedirs(output)
        else:
            print(f"Warning: The output directory {output} already exists. Skipping.", file=sys.stderr)
            raise RuntimeError()

        # Connect to Carla
        client = connect()

        # Load world and recording frequency
        world, (tick_count, begin_at, end_at, recording_frequency) = load_world(client=client, begin_at=self.begin_at, end_at=self.end_at, logfile=logfile)
        self.tick_count = tick_count
        self.begin_at = begin_at
        self.end_at = end_at

        # Initialize necessary helper classes
        rasterizer = MapRasterizer(carla_world=world)
        api_helper = CarlaAPIHelper(client=client, world=world, rasterizer=rasterizer)

        # Start replay of simulation
        cameras, ego_vehicle = self.__start_replay__(logfile=logfile, output=output, api_helper=api_helper, world=world)

        # Record all ticks
        self.__record_ticks__(world=world, cameras=cameras, ego_vehicle=ego_vehicle, recording_frequency=recording_frequency)
