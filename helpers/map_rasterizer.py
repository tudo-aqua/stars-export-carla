import os

import numpy as np
from carla import World, Map, Junction, Landmark, LaneType
from scipy.spatial import KDTree

from carla_data_classes import *
from helpers.json_helper import JSONHelper


class MapRasterizer:
    """
    This class rasterizes the current map loaded in carla into
    DataBlocks. Each block spans either a junction or
    a multi-lane road.
    """

    def __init__(self, carla_world: World):
        self._world = carla_world
        self._map: Map = carla_world.get_map()
        self._blocks = []
        self._debug_helper = self._world.debug
        self._kd_tree = None
        self._lane_midpoints = []

    def debug_road(self, data_road: DataRoad) -> None:
        for data_lane in data_road.lanes:
            for midpoint in data_lane.lane_midpoints:
                self._debug_helper.draw_string(midpoint.location.to_location(),
                                               f"{data_lane.road_id} - {data_lane.lane_id}", life_time=600000000)

    def save_data_blocks(self, file_path: str) -> None:
        """
        Saves the blocks created for the current map to disk.
        """
        if self._blocks.__len__() == 0:
            raise RuntimeError("The blocks have not yet been calculated. Use method 'get_data_blocks'")
        JSONHelper.log_data_blocks(self._blocks, file_path)

    def load_or_calculate_data_blocks(self, file_name: str, map_name: str, update_existing: bool) -> List[DataBlock]:
        """
        Loads the DataBlocks for the current map if existing. Otherwise, they are calculated and
        saved to disk.
        """
        if self._blocks.__len__() > 0:
            print("Blocks are already calculated. Nothing new to be done.")
            return self._blocks
        # Load the collected static data from the json file
        logfile_name = JSONHelper.get_file_path_for_name(name=file_name, prefix=JSONHelper.STATIC_FILE_NAME_PREFIX,
                                                         folder=JSONHelper.SIMULATION_RUNS_FOLDER, map_name=map_name,
                                                         file_ending="zip")
        data_log_file = logfile_name.replace(".zip", ".json")
        if not os.path.exists(logfile_name) or update_existing:
            if update_existing:
                print("Overwrite existing map data")
            else:
                print("The static information was not yet calculated. Calculate now.")
            self._blocks = self.get_data_blocks()
            self.save_data_blocks(file_path=data_log_file)
            JSONHelper.zip_and_delete_file(data_log_file)
        else:
            print("The static information for the file", logfile_name, "has already been calculated. Load from file.")
            JSONHelper.extract_from_zip(logfile_name)
            data_blocks = JSONHelper.load_data_blocks(data_log_file)
            self._blocks = data_blocks
            JSONHelper.delete_file(data_log_file)

        self._lane_midpoints = self.getLaneMidpointsArray()
        lane_midpoint_locations = list(
            map(lambda l: (l.location.x, l.location.y, l.location.z), self._lane_midpoints))
        self._kd_tree = KDTree(lane_midpoint_locations)
        return self._blocks

    @staticmethod
    def flatten(l):
        return [item for sublist in l for item in sublist]

    def getLaneMidpointsArray(self) -> List[DataLaneMidpoint]:
        roads = MapRasterizer.flatten(list(map(lambda b: b.roads, self._blocks)))
        lanes = MapRasterizer.flatten(list(map(lambda r: r.lanes, roads)))
        lane_midpoints = MapRasterizer.flatten(list(map(lambda l: l.lane_midpoints, lanes)))
        return lane_midpoints

    def get_closest_lane_midpoint(self, location: DataLocation) -> DataLaneMidpoint:
        d, i = self._kd_tree.query((location.x, location.y, location.z))
        return self._lane_midpoints[i]

    def get_data_blocks(self, distance: float = 2.0) -> List[DataBlock]:
        """
        Returns a list of fully filled DataBlock objects of the whole map
        @param distance: The distance for which the waypoints should be created
        @return: A list of filled DataBlocks
        """
        landmarks = self._map.get_all_landmarks()
        data_blocks: List[DataBlock] = []
        waypoint_list: List[Waypoint] = self._map.generate_waypoints(distance)
        for waypoint in waypoint_list:
            already_processed = False
            for data_block in data_blocks:
                if MapRasterizer.block_contains_waypoint(data_block, waypoint):
                    already_processed = True
            if already_processed:
                continue
            if waypoint.is_junction:
                # The waypoint belongs to a junction: calculate all possible waypoints representing the roads
                junction = waypoint.get_junction()
                data_roads: List[DataRoad] = self.get_data_roads_for_junction(junction, landmarks)
                road_ids = list(map(lambda d: f"{d.road_id}", data_roads))
                block_id: str = "-".join(road_ids)
                data_block: DataBlock = DataBlock(block_id, data_roads)
                data_blocks.append(data_block)
            else:
                # The waypoint belongs to a multi-lane road: calculate all possible waypoints
                data_road = self.get_data_road_for_waypoints(waypoint, waypoint_list, landmarks)
                data_block = DataBlock(str(data_road.road_id), [data_road])
                data_blocks.append(data_block)
        self._blocks = data_blocks
        self.add_landmarks_to_lanes(data_blocks)
        return data_blocks

    def get_all_traffic_lights(self) -> List[DataStaticTrafficLight]:
        roads = self.flatten(list(map(lambda b: b.roads, self._blocks)))
        lanes = self.flatten(list(map(lambda r: r.lanes, roads)))
        return self.flatten(list(map(lambda l: l.traffic_lights, lanes)))

    def add_landmarks_to_lanes(self, data_blocks: List[DataBlock]) -> None:
        """
        This method adds all available landmarks of the map to the corresponding lanes
        """
        landmarks: List[Landmark] = self._map.get_all_landmarks()
        for landmark in landmarks:
            data_landmark = self.get_data_landmark_for_landmark(landmark)
            # Cycle through the landmarks and get their corresponding lane objects
            landmark_road_id = landmark.road_id
            data_road = self.get_specific_road_from_blocks(data_blocks, landmark_road_id)
            if not data_road:
                return
            # Check for each lane if it is valid for the given landmark
            for lane in data_road.lanes:
                if MapRasterizer.is_lane_valid_for_landmark(landmark, lane):
                    # The landmark is valid for the current lane. Append to list of landmarks
                    lane.landmarks.append(data_landmark)
                    if lane.lane_id > 0:
                        data_landmark.s = lane.lane_length - data_landmark.s

    @staticmethod
    def is_lane_valid_for_landmark(landmark: Landmark, data_lane: DataLane) -> bool:
        """
        Returns whether the given lane is valid for the given landmark
        @param landmark: The landmark for which the lane should be checked
        @param data_lane: The lane that should be checked
        @return: True, if the lane is valid for the given Landmark. False, otherwise
        """
        lane_validities: List[Tuple[int]] = landmark.get_lane_validities()
        for tupl in lane_validities:
            # As the lane validity is given as a lane_id interval we have to check
            # whether the lane_id is inside the current interval
            if tupl[0] <= data_lane.lane_id <= tupl[1]:
                return True
        return False

    @staticmethod
    def get_specific_road_from_blocks(blocks: List[DataBlock], road_id: int) -> Optional[DataRoad]:
        """
        Returns the corresponding DataRoad to the given road_id from the lust of DataBlocks
        @param blocks: The DataBlocks in which should be searched for the given road_id
        @param road_id: The id of the DataRoad which should be retrieved
        @return: The DataRoad corresponding to the given road_id
        """
        for block in blocks:
            for road in block.roads:
                if road.road_id == road_id:
                    return road
        return None

    @staticmethod
    def block_contains_waypoint(data_block: DataBlock, waypoint: Waypoint) -> bool:
        """
        Returns whether the given waypoint lays within the given data block
        @param data_block: That block that is taken as a reference point
        @param waypoint: The waypoint that should be checked
        @return: True, if the waypoint is within the given block. False, otherwise.
        """
        for data_road in data_block.roads:
            if data_road.road_id == waypoint.road_id:
                return True
        return False

    def get_data_road_for_waypoints(self, waypoint: Waypoint, waypoints: List[Waypoint],
                                    landmarks: List[Landmark]) -> DataRoad:
        """
        This method returns a filled DataRoad based on the given waypoint. The waypoints list is for
        the calculation of the underlying lanes of the road.
        @param waypoint: The lane of which the road should be constructed of
        @param waypoints: All waypoint of the map
        @return: A filled DataRoad object based on the given waypoint
        """
        road_id = waypoint.road_id
        # Extract all waypoints belonging to the road of the waypoint
        lane_waypoints = list(filter(lambda wp: wp.road_id == road_id, waypoints))
        relevant_waypoints = []
        for lane_waypoint in lane_waypoints:
            already_in_list = False
            # Check if the waypoint is also included in the road
            for relevant_waypoint in relevant_waypoints:
                if relevant_waypoint.lane_id == lane_waypoint.lane_id:
                    already_in_list = True
            # Only process lanes that are not yet tracked in the road
            if not already_in_list:
                relevant_waypoints.append(lane_waypoint)
        # Transform all relevant waypoints to DataLanes
        data_lanes = list(map(lambda wp: self.get_data_lane_for_waypoint(wp, landmarks), relevant_waypoints))
        return DataRoad(road_id, lanes=data_lanes, is_junction=waypoint.is_junction)

    def get_data_roads_for_junction(self, junction: Junction, landmarks: List[Landmark]) -> List[DataRoad]:
        """
        Returns a list of filled DataRoads. Also gathers the necessary information about included DataLanes.
        @param junction: The Junction from which the road information is gathered
        @return: List of filled DataRoads
        """
        road_ids: List[int] = []
        roads = dict()
        roads_list = []
        road_is_junction = dict()
        block_lanes: List[DataLane] = []
        # Loop over each waypoint in junction
        for waypoint_tuple in junction.get_waypoints(LaneType.Driving):
            waypoint = waypoint_tuple[0]
            road_id = waypoint.road_id
            road_is_junction[road_id] = waypoint.is_junction
            already_appended = False
            for road_it_id in road_ids:
                if waypoint.road_id == road_it_id:
                    already_appended = True
            if not already_appended:
                # A new road was found
                road_ids.append(waypoint.road_id)
            # Get data lane from waypoint
            data_lane = self.get_data_lane_for_waypoint(waypoint, landmarks)
            if road_id not in roads:
                roads[road_id] = []
            roads[road_id].append(data_lane)
            block_lanes.append(data_lane)
        for road_id in road_ids:
            road_lanes = roads[road_id]
            is_junction = road_is_junction[road_id]
            # Build DataRoad from collected data
            data_road: DataRoad = DataRoad(road_id=road_id, is_junction=is_junction, lanes=road_lanes)
            # Add to roads list
            roads_list.append(data_road)
        # If currently a junction is observed: calculate intersection points of the junctions' lanes
        for lane in block_lanes:
            for intersection_check_lane in block_lanes:
                # Do not calculate the intersection of the same lane
                if lane == intersection_check_lane:
                    continue
                if lane.lane_id == intersection_check_lane.lane_id and lane.road_id == intersection_check_lane.road_id:
                    continue
                # Get intersection point, if existing
                intersection_point = MapRasterizer.get_intersection_of_lanes(lane, intersection_check_lane)
                # Check if there is an intersection
                if intersection_point:
                    # Create DataContactLaneInfo for intersection
                    data_contact_lane_info = DataContactLaneInfo(road_id=intersection_check_lane.road_id,
                                                                 lane_id=intersection_check_lane.lane_id)
                    lane.intersecting_lanes.append(data_contact_lane_info)
                    # Calculate the distance from the start of the lane to the intersection point
                    distance_lane = MapRasterizer.get_distance_to_start_of_lane(lane, intersection_point)
                    distance_intersection_check_lane = MapRasterizer.get_distance_to_start_of_lane(
                        intersection_check_lane,
                        intersection_point)
                    contact_area = DataContactArea.from_lanes(contact_location=intersection_point, lane_1=lane,
                                                              start_pos_lane_1=distance_lane,
                                                              lane_2=intersection_check_lane,
                                                              start_pos_lane_2=distance_intersection_check_lane)
                    lane.contact_areas.append(contact_area)
        return roads_list

    def get_data_lane_for_waypoint(self, waypoint: Waypoint, landmarks: List[Landmark]) -> DataLane:
        """
        Returns the filled DataLane. Collects information about pre-/successor lanes, intersections, etc.
        @param waypoint: The waypoint from which the information should be collected
        @return: The filled DataLane
        """
        print("Converting road", waypoint.road_id, "with lane", waypoint.lane_id)
        # Get length of the lane
        lane_length: float = self.get_length_of_lane(waypoint)
        # Get list of leading lanes
        predecessor_lanes: List[DataContactLaneInfo] = self.get_predecessor_contact_infos(waypoint)
        # Get list of follow-up lanes
        successor_lanes: List[DataContactLaneInfo] = self.get_successor_contact_infos(waypoint)
        # Get all waypoints for the current lane
        all_waypoints: List[Tuple[float, Waypoint]] = self.get_all_waypoints_for_lane(waypoint, 0.1)
        # Map waypoints to their respective coordinates
        lane_midpoints: List[DataLaneMidpoint] = list(
            map(lambda tuple_distance_waypoint: DataLaneMidpoint(distance_to_start=tuple_distance_waypoint[0],
                                                                 location=DataLocation.from_waypoint(
                                                                     tuple_distance_waypoint[1]),
                                                                 rotation=DataRotation.from_waypoint(
                                                                     tuple_distance_waypoint[1]),
                                                                 lane_id=waypoint.lane_id,
                                                                 road_id=waypoint.road_id),
                all_waypoints))

        traffic_light_posts = list(filter(lambda l: l.type == '1000001', landmarks))
        traffic_lights = []
        for traffic_light_post in traffic_light_posts:
            if traffic_light_post.road_id == waypoint.road_id:
                for lane_validity in traffic_light_post.get_lane_validities():
                    if lane_validity[0] <= waypoint.lane_id <= lane_validity[1]:
                        dynamic_traffic_light = self._world.get_traffic_light_from_opendrive_id(traffic_light_post.id)
                        static_traffic_light = MapRasterizer.get_data_static_traffic_light_for_traffic_light(
                            traffic_light_post, dynamic_traffic_light)
                        traffic_lights.append(static_traffic_light)

        # Build and return DataLane
        return DataLane(road_id=waypoint.road_id, lane_id=waypoint.lane_id,
                        lane_type=DataLaneType(int(waypoint.lane_type)), lane_width=waypoint.lane_width,
                        lane_length=lane_length, s=waypoint.s, predecessor_lanes=predecessor_lanes,
                        successor_lanes=successor_lanes, intersecting_lanes=[], lane_midpoints=lane_midpoints,
                        speed_limits=[], landmarks=[], contact_areas=[], traffic_lights=traffic_lights)

    @staticmethod
    def get_intersection_of_lanes(lane_1: DataLane, lane_2: DataLane) -> Optional[DataLocation]:
        """
        Returns the crossing point of the two given lanes, if one exists
        @param lane_1: Lane 1
        @param lane_2: Lane 2
        @return: The crossing point as a DataLocation, if existing. Otherwise None
        """
        if lane_1.road_id == lane_2.road_id and lane_1.road_id == lane_2.lane_id:
            return None
        lane_1_midpoints = list(filter(lambda l: l.distance_to_start % 1.0 == 0, lane_1.lane_midpoints))
        lane_2_midpoints = list(filter(lambda l: l.distance_to_start % 1.0 == 0, lane_2.lane_midpoints))
        for index_lane_1, _ in enumerate(lane_1_midpoints):
            for index_lane_2, _ in enumerate(lane_2_midpoints):
                # Check if there is a crossing point between the two midpoints
                if not index_lane_1 == lane_1.lane_midpoints.__len__() - 1 and \
                        not index_lane_2 == lane_2.lane_midpoints.__len__() - 1:
                    midpoint_lane_1_start = lane_1.lane_midpoints[index_lane_1].location
                    midpoint_lane_1_end = lane_1.lane_midpoints[index_lane_1 + 1].location
                    midpoint_lane_2_start = lane_2.lane_midpoints[index_lane_2].location
                    midpoint_lane_2_end = lane_2.lane_midpoints[index_lane_2 + 1].location
                    crossing_point = MapRasterizer.get_intersect(midpoint_lane_1_start, midpoint_lane_1_end,
                                                                 midpoint_lane_2_start, midpoint_lane_2_end)
                    if not crossing_point:
                        continue
                    # There is a midpoint. Now check if it is between the given points, or somewhere else
                    if MapRasterizer.is_between(midpoint_lane_1_start, midpoint_lane_1_end,
                                                crossing_point) and MapRasterizer.is_between(midpoint_lane_2_start,
                                                                                             midpoint_lane_2_end,
                                                                                             crossing_point):
                        # Only the crossing point that lays between the given points should be returned
                        return crossing_point
        return None

    @staticmethod
    def get_intersect(lane_1_start: DataLocation, lane_1_end: DataLocation, lane_2_start: DataLocation,
                      lane_2_end: DataLocation) -> Optional[DataLocation]:
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        Solution from https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        """
        stacked = np.vstack([lane_1_start.to_tuple(), lane_1_end.to_tuple(), lane_2_start.to_tuple(),
                             lane_2_end.to_tuple()])
        h = np.hstack((stacked, np.ones((4, 1))))  # h for homogeneous
        l1 = np.cross(h[0], h[1])  # get first line
        l2 = np.cross(h[2], h[3])  # get second line
        x, y, z = np.cross(l1, l2)  # point of intersection
        if z == 0:  # lines are parallel
            return None
        return DataLocation(x=x / z, y=y / z, z=0)

    @staticmethod
    def is_between(from_point: DataLocation, to_point: DataLocation, point_to_check: DataLocation) -> bool:
        """
        Returns whether @param point_to_check is between @param from_point and @param to_point
        This method is used to check whether the crossing point c of
        the line (a,b) with another line lies between a and b.
        Otherwise, the crossing point is outside and not relevant
        Solution from:
        https://stackoverflow.com/questions/328107/how-can-you-determine-a-point-is-between-two-other-points-on-a-line-segment
        @param from_point: Starting point
        @param to_point: Ending point
        @param point_to_check: Point to check of being between a and b
        @return: True, if c is between a and b. False, otherwise.
        """
        cross_product = (point_to_check.y - from_point.y) * (to_point.x - from_point.x) - (
                point_to_check.x - from_point.x) * (to_point.y - from_point.y)

        # compare versus epsilon for floating point values, or != 0 if using integers
        if abs(cross_product) > 0.1:
            return False

        dot_product = (point_to_check.x - from_point.x) * (to_point.x - from_point.x) + (
                point_to_check.y - from_point.y) * (to_point.y - from_point.y)
        if dot_product < 0:
            return False

        squared_length_ba = (to_point.x - from_point.x) * (to_point.x - from_point.x) + (to_point.y - from_point.y) * (
                to_point.y - from_point.y)
        if dot_product > squared_length_ba:
            return False

        return True

    @staticmethod
    def distance_between(from_point: DataLocation, to_point: DataLocation) -> float:
        """
        Returns the Euclidean Distance between the two given points
        Solution from:
        https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
        @param from_point: First point
        @param to_point: Second point
        @return: The distance as a float value between the two given points
        """
        a = np.array((from_point.x, from_point.y, from_point.z))
        b = np.array((to_point.x, to_point.y, to_point.z))
        return np.linalg.norm(a - b)

    @staticmethod
    def get_distance_to_start_of_lane(lane: DataLane, point: DataLocation) -> float:
        """
        Returns the distance of the given point to the start of the given lane
        @param lane: The lane which is considered for the distance
        @param point: The point for which the distance should be calculated
        @return: The distance as a float value of the given point the start of the given lane
        """
        minimum_distance = None
        min_distance = float("inf")
        # Check for each midpoint if it closer to the given point then the ones before
        for lane_midpoint in lane.lane_midpoints:
            midpoint = lane_midpoint.location
            distance = lane_midpoint.distance_to_start
            relative_distance = MapRasterizer.distance_between(midpoint, point)
            # Check if closer than the previous midpoints
            if relative_distance < min_distance:
                # Save minimal distance to any midpoint
                min_distance = relative_distance
                # Save distance to start for current midpoint
                minimum_distance = distance
        return minimum_distance

    @staticmethod
    def get_data_static_traffic_light_for_traffic_light(static_traffic_light: Landmark,
                                                        traffic_light: TrafficLight) -> DataStaticTrafficLight:
        """
        Returns the DataStaticTrafficLight object based on the given traffic_light
        @param traffic_light: The traffic_light that should be converted into a DataStaticTrafficLight
        @return: The filled DataStaticTrafficLight
        """
        location = DataLocation.from_location(static_traffic_light.transform.location)
        rotation = DataRotation.from_rotation(static_traffic_light.transform.rotation)
        if traffic_light is not None:
            stop_locations = list(
                map(lambda waypoint: DataLocation.from_waypoint(waypoint), traffic_light.get_stop_waypoints()))
        else:
            stop_locations = []
        return DataStaticTrafficLight(open_drive_id=static_traffic_light.id, location=location, rotation=rotation,
                                      stop_locations=stop_locations, position_distance=static_traffic_light.s)

    def get_length_of_lane(self, lane: Waypoint, precision: float = 2.0) -> float:
        """
        Returns the length of the lane in meters
        @param lane: The lane for which the length should be calculated
        @param precision: The precision for which the waypoint should be evaluated
        @return: The length of the given lane in meters
        """
        all_waypoints = self.get_all_waypoints_for_lane(lane, precision)
        return all_waypoints[all_waypoints.__len__() - 1][0] + precision

    def get_predecessor_contact_infos(self, lane: Waypoint) -> List[DataContactLaneInfo]:
        """
        Returns a list of all predecessor lanes of the given lane
        @param lane: The lane for which the predecessor lanes should be returned
        @return: List of all predecessor lanes
        """
        predecessor_lanes = self.get_predecessor_lanes(lane)
        data_contact_lane_infos = []
        for pre_lane in predecessor_lanes:
            data_contact_lane_info = DataContactLaneInfo(lane_id=pre_lane.lane_id, road_id=pre_lane.road_id)
            data_contact_lane_infos.append(data_contact_lane_info)
        return data_contact_lane_infos

    def get_successor_contact_infos(self, lane: Waypoint) -> List[DataContactLaneInfo]:
        """
        Returns a list of all successor lanes of the given lane
        @param lane: The lane for which the successor lanes should be returned
        @return: List of all successor lanes
        """
        successor_lanes = self.get_successor_lanes(lane)
        data_contact_lane_infos = []
        for pre_lane in successor_lanes:
            data_contact_lane_info = DataContactLaneInfo(lane_id=pre_lane.lane_id, road_id=pre_lane.road_id)
            data_contact_lane_infos.append(data_contact_lane_info)
        return data_contact_lane_infos

    def get_waypoint_for_actor(self, actor: Actor) -> Waypoint:
        """
        Returns the Waypoint for the given actor
        :param actor: The actor from which the waypoint should be calculated
        :return: Waypoint closest to the given actor
        """
        location = actor.get_transform().location
        return self.get_waypoint_for_location(location)

    def get_waypoint_for_location(self, location: Location) -> Waypoint:
        """
        Returns the nearest Waypoint for the given location
        :param location: The location from which the waypoint should be calculated
        :return: Waypoint closest to the given location
        """
        waypoint = self._map.get_waypoint(location)
        return waypoint

    def get_all_waypoints_for_lane(self, lane: Waypoint, precision: float = 2.0) -> List[Tuple[float, Waypoint]]:
        """
        Returns a list of all waypoints with distance from the start for the current lane
        :param lane: The lane for which the waypoints should be calculated
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of {distance, Waypoint} for the lane
        """
        all_waypoints = []
        # Get all waypoints until the lane starts
        waypoints_until_start = self.get_all_waypoints_until_start_of_lane(lane, precision)
        # Reverse the list to start with the waypoint that is at the beginning of the lane
        waypoints_until_start.reverse()
        # Add waypoints until start of lane to waypoint list
        all_waypoints.extend(waypoints_until_start)
        # Add current waypoint as it is not included in the previous list
        all_waypoints.append(lane)
        # Get all waypoints until the lane end
        waypoints_until_end = self.get_all_waypoints_until_end_of_lane(lane, precision)
        # Add waypoints until end of lane to waypoint list
        all_waypoints.extend(waypoints_until_end)
        # Remove duplicate entries with preserving of order
        unique_waypoints = []
        for wp in all_waypoints:
            if wp not in unique_waypoints:
                unique_waypoints.append(wp)
        waypoint_distance_list = []
        waypoint_counter = 0
        # Calculate distance to start of lane for each waypoint
        for waypoint in unique_waypoints:
            # The distance is based on the given precision (in m)
            distance = precision * waypoint_counter
            waypoint_distance_tuple: Tuple[float, Waypoint] = (distance, waypoint)
            waypoint_distance_list.append(waypoint_distance_tuple)
            waypoint_counter += 1
        return waypoint_distance_list

    def get_last_waypoint_of_lane(self, lane: Waypoint, precision: float = 2.0) -> Waypoint:
        """
        Returns the last waypoint of the given lane with the given precision
        :param lane: The lane of which the last waypoint should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: Last Waypoint of the given lane
        """
        waypoints_until_end_of_lane = self.get_all_waypoints_until_end_of_lane(lane, precision)
        if waypoints_until_end_of_lane.__len__() == 0:
            return lane
        last_waypoint = waypoints_until_end_of_lane[len(waypoints_until_end_of_lane) - 1]
        return last_waypoint

    def get_first_waypoint_of_lane(self, lane: Waypoint, precision: float = 2.0) -> Waypoint:
        """
        Returns the first waypoint of the given lane with the given precision
        :param lane: The lane of which the last waypoint should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: First Waypoint of the given lane
        """
        waypoints_until_start_of_lane = self.get_all_waypoints_until_start_of_lane(lane, precision)
        if waypoints_until_start_of_lane.__len__() == 0:
            return lane
        first_waypoint = waypoints_until_start_of_lane[len(waypoints_until_start_of_lane) - 1]
        return first_waypoint

    def get_successor_lanes(self, lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of waypoints representing each follow-up lane for the given lane
        :param lane: Given lane from which should be looked ahead
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints representing follow-up lanes
        """
        last_waypoint = self.get_last_waypoint_of_lane(lane, precision)
        return last_waypoint.next(float(precision))

    def get_predecessor_lanes(self, lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of waypoints representing each leading lane for the given lane
        :param lane: Given lane from which should be looked at
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints representing leading lanes
        """
        first_waypoint = self.get_first_waypoint_of_lane(lane, precision)
        return first_waypoint.previous(float(precision))

    def is_actor_in_block(self, actor: Actor, block: DataBlock) -> bool:
        """
        Returns whether the given actor is inside the given block
        :param actor: Actor which should be checked to be inside the given block
        :param block: The block for which the location of the actor should be checked
        :return: True if the actor is inside the given block, False otherwise
        """
        # Get Waypoint for the given actor
        actor_lane: Waypoint = self.get_waypoint_for_actor(actor)
        # Check each road of the block for a matching lane/road
        road: DataRoad
        for road in block.roads:
            # Check if the road is included in the block
            if road.road_id == actor_lane.road_id:
                # The road is inside the block. Check for matching lane
                for lane in road.lanes:
                    # The road and lane matches
                    if lane.lane_id == actor_lane.lane_id:
                        return True
        # No matching Road/Lane was found
        return False

    def get_best_block_for_actor(self, actor: Actor) -> Optional[DataBlock]:
        """
        Return DataBlock for current actor
        :param actor: Actor for which the DataBlock should be calculated
        :return: The DataBlock for the given Actor
        """
        if self._blocks.__len__() == 0:
            raise RuntimeError(
                "The blocks for the current map have to be calculated first! Use the method 'get_data_blocks()'")
        # Check each block if it is the correct one
        for block in self._blocks:
            if self.is_actor_in_block(actor, block):
                return block
        # No matching block was found.
        return None

    @staticmethod
    def get_all_waypoints_until_end_of_lane(lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of all waypoints until the end of the current lane
        :param lane: The lane from which the waypoints should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints until the lane ends
        """
        # A manual cycle is necessary as sometime the previous_until_lane_start will result in a SIGSEGV error
        # besides the waypoint having a fitting previous waypoint
        next_waypoints = []
        has_next = True
        while has_next:
            # Move one waypoint with the given precision
            next_lanes: List[Waypoint] = lane.next(precision)
            # Check if there is another lane available
            if next_lanes.__len__() == 0:
                has_next = False
            # Cycle through all attached lanes
            for nxt in next_lanes:
                # Only append lanes on the same road
                if nxt.road_id == lane.road_id and nxt.lane_id == lane.lane_id:
                    next_waypoints.append(nxt)
                    lane = nxt
                else:
                    has_next = False
        return next_waypoints

    @staticmethod
    def get_all_waypoints_until_start_of_lane(lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of all waypoints until the start of the current lane
        :param lane: The lane from which the waypoints should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints until the lane start
        """
        # A manual cycle is necessary as sometime the previous_until_lane_start will result in a SIGSEGV error
        # besides the waypoint having a fitting previous waypoint
        previous_waypoints = []
        has_previous = True
        while has_previous:
            # Move one waypoint with the given precision
            prev_lanes: List[Waypoint] = lane.previous(precision)
            # Check if there is another lane available
            if prev_lanes.__len__() == 0:
                has_previous = False
            # Cycle through all attached lanes
            for pre in prev_lanes:
                # Only append lanes on the same road
                if pre.road_id == lane.road_id and pre.lane_id == lane.lane_id:
                    previous_waypoints.append(pre)
                    lane = pre
                else:
                    has_previous = False
        return previous_waypoints

    @staticmethod
    def get_data_landmark_for_landmark(landmark: Landmark) -> DataLandmark:
        """
        Returns the DataLandmark object based on the given landmark
        @param landmark: The landmark that should be converted into a DataLandmark
        @return: The filled DataLandmark object
        """
        orientation = DataLandmarkOrientation(int(landmark.orientation))
        landmark_type = DataLandmarkType(int(landmark.type))
        location = DataLocation.from_location(landmark.transform.location)
        rotation = DataRotation.from_rotation(landmark.transform.rotation)
        return DataLandmark(id=landmark.id, road_id=landmark.road_id, name=landmark.name, distance=landmark.distance,
                            s=landmark.s, is_dynamic=landmark.is_dynamic, orientation=orientation,
                            z_offset=landmark.z_offset, country=landmark.country, type=landmark_type,
                            sub_type=landmark.sub_type, value=landmark.value, unit=landmark.unit,
                            height=landmark.height, width=landmark.width, text=landmark.text,
                            h_offset=landmark.h_offset, pitch=landmark.pitch, roll=landmark.roll, location=location,
                            rotation=rotation)

    def blocks_contain_waypoint(self, lane_id: int, road_id: int) -> bool:
        for block in self._blocks:
            for road in block.roads:
                for lane in road.lanes:
                    if lane.lane_id == lane_id and lane.road_id == road_id:
                        return True
        return False

    def get_data_road(self, road_id: int) -> DataRoad:
        roads = list(map(lambda b: b.roads, self._blocks))
        flat_list = [item for sublist in roads for item in sublist]
        for road in flat_list:
            if road.road_id == road_id:
                return road
