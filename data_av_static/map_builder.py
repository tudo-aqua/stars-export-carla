from collections import deque
from typing import List, Optional, Set, Tuple, Dict, TYPE_CHECKING

from carla import Junction, Landmark, Waypoint, LaneType
from shapely import Point, LineString, STRtree
from shapely.ops import nearest_points

from carla_data_classes.static import DataRoad, DataLandmark, DataLane, DataLocation, DataContactArea, \
    DataContactLaneInfo
from carla_data_classes.static.DataBlock import DataBlock
from carla_data_classes.static.DataJunction import DataJunction
from carla_data_classes.static.DataMap import DataMap

if TYPE_CHECKING:
    pass


class _BlockBuilder:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    @staticmethod
    def _build_data_map(blocks: List[DataBlock]) -> DataMap:
        """
        Flatten the DataBlock list into the two collections expected by
        DataMap (straights & junctions).
        """
        straights: List[DataRoad] = []
        junctions_by_id: Dict[int, DataJunction] = {}

        for blk in blocks:
            for rd in blk.roads:
                if rd.is_junction:
                    # group all junction roads under their CARLA-junction id
                    j_id = rd.junction_id
                    if j_id is None:
                        continue  # fallback – shouldn’t happen
                    junc = junctions_by_id.setdefault(j_id, DataJunction(j_id, []))
                    junc.roads.append(rd)
                else:
                    straights.append(rd)

        return DataMap(straights=straights, junctions=list(junctions_by_id.values()))

    def get_data_map(self, distance: float = 0.1) -> DataMap:
        """
        Retrieves a list of data blocks based on map waypoints and landmarks.

        Arguments:
            distance: A float specifying the spacing between generated waypoints in
                      meters. Defaults to 0.1.

        Returns:
            A list of DataBlock objects representing the structured information
            derived from the map's waypoints and landmarks.
        """
        landmarks = self.ctx.map.get_all_landmarks()
        data_blocks: List[DataBlock] = []
        waypoint_list: List[Waypoint] = self.ctx.map.generate_waypoints(distance)
        for waypoint in waypoint_list:
            already_processed = False
            for data_block in data_blocks:
                if self.block_contains_waypoint(data_block, waypoint):
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
                data_road = self.get_data_road_for_waypoints(waypoint, landmarks)
                data_block = DataBlock(str(data_road.road_id), [data_road])
                data_blocks.append(data_block)
        self.ctx.blocks = data_blocks
        self.add_landmarks_to_lanes(data_blocks)
        self.ctx.update_static_traffic_lights_from_landmarks(data_blocks)
        self.ctx.close_speed_limit_gaps(data_blocks, default_speed_kmh=30.0)
        data_map = self._build_data_map(blocks=data_blocks)
        self.ctx.data_map = data_map
        return data_map

    def get_data_road_for_waypoints(self, waypoint: Waypoint, landmarks: List[Landmark]) -> DataRoad:
        """
        This method returns a filled DataRoad based on the given waypoint. Using map topology and lane 
        information, all lanes of the specified road are identified and included in the result.

        Args:
            waypoint (Waypoint): A representative waypoint for the road to be constructed
            landmarks (List[Landmark]): List of landmarks that could be relevant for this road

        Returns:
            DataRoad: A filled DataRoad object containing all lanes of the specified road
        """
        target_road = waypoint.road_id

        topo = self.ctx.map.get_topology()

        lane_to_wp = {}
        for wp, _ in topo:
            if wp.road_id == target_road and wp.lane_id not in lane_to_wp:
                lane_to_wp[wp.lane_id] = wp

        road_lanes = list(lane_to_wp.values())
        lanes = self.collect_all_lanes_waypoints(road_lanes)
        data_lanes = [
            self.ctx.get_data_lane_for_waypoint(wp, landmarks)
            for wp in lanes
        ]

        return DataRoad(
            road_id=target_road,
            is_junction=waypoint.is_junction,
            lanes=data_lanes
        )

    def get_data_roads_for_junction(self, junction: Junction, landmarks: List[Landmark]) -> List[DataRoad]:
        """
        Returns a list of filled DataRoads and gathers information about included DataLanes from the provided junction.

        Args:
            junction (Junction): The Junction from which to gather road information
            landmarks (List[Landmark]): List of landmarks that could be relevant for these roads

        Returns:
            List[DataRoad]: A list of filled DataRoad objects containing all lanes within the junction
        """
        # ---- collect roads and lanes in this junction -------------------------
        road_ids: List[int] = []
        roads: Dict[int, List[DataLane]] = {}
        roads_list: List[DataRoad] = []
        road_is_junction: Dict[int, bool] = {}
        block_lanes: List[DataLane] = []

        junction_waypoints = junction.get_waypoints(LaneType.Any)
        for wp_tuple in junction_waypoints:
            waypoint = wp_tuple[0]
            road_id = waypoint.road_id
            road_is_junction[road_id] = waypoint.is_junction

            if road_id not in road_ids:
                road_ids.append(road_id)

            data_lane = self.ctx.get_data_lane_for_waypoint(waypoint, landmarks)
            roads.setdefault(road_id, []).append(data_lane)
            block_lanes.append(data_lane)

        for road_id in road_ids:
            lanes = roads.get(road_id, [])
            is_junction = road_is_junction.get(road_id, True)
            roads_list.append(DataRoad(road_id=road_id, is_junction=is_junction, lanes=lanes, junction_id=junction.id))

        # ---- compute contact areas / intersections within the junction --------
        # (Only if we actually collected lanes)
        if not block_lanes:
            return roads_list

        geoms: List[LineString] = [lane.get_linestring() for lane in block_lanes]
        strtree = STRtree(geoms)
        geom2lane: Dict[LineString, DataLane] = {g: ln for g, ln in zip(geoms, block_lanes)}

        # Keep track of processed geometry pairs (avoid double work)
        processed: Dict[frozenset, Optional[DataContactArea]] = {}
        tol = 0.30  # 30 cm near-miss tolerance

        for i, geom_a in enumerate(geoms):
            lane_a = geom2lane[geom_a]

            # Query candidates near geom_a (buffered by tol)
            for idx in strtree.query(geom_a.buffer(tol)):
                j = int(idx)
                geom_b = geoms[j]
                if geom_a is geom_b:
                    continue

                key = frozenset((geom_a, geom_b))
                if key in processed:
                    continue

                lane_b = geom2lane[geom_b]

                # ---- find contact point between the two lane center-lines ----
                inter_geom = geom_a.intersection(geom_b)

                point: Optional[Point] = None
                if inter_geom.is_empty:
                    # Near miss? If farther than tol, skip; else use mid of closest points
                    if geom_a.distance(geom_b) > tol:
                        processed[key] = None
                        continue
                    p, q = nearest_points(geom_a, geom_b)
                    rough = Point((p.x + q.x) * 0.5, (p.y + q.y) * 0.5)
                    point = self.ctx.nose_point(rough, geom_a, geom_b, True)
                else:
                    # Overlap / intersection – delegate to nose_point
                    point = self.ctx.nose_point(inter_geom, geom_a, geom_b)

                # If we still have no usable point, skip this pair safely
                if point is None:
                    processed[key] = None
                    continue

                contact_loc = DataLocation(point.x, point.y, 0.0)

                # Distances along each lane to the contact location
                d_a = self.ctx.get_distance_to_start_of_lane(lane_a, contact_loc)
                d_b = self.ctx.get_distance_to_start_of_lane(lane_b, contact_loc)

                # Build the contact area exactly once (no UnboundLocalError possible)
                area = DataContactArea.from_lanes(
                    contact_location=contact_loc,
                    lane_1=lane_a, start_pos_lane_1=d_a,
                    lane_2=lane_b, start_pos_lane_2=d_b
                )

                # Wire up relationships
                lane_a.contact_areas.append(area)
                lane_b.contact_areas.append(area)
                lane_a.intersecting_lanes.append(
                    DataContactLaneInfo(lane_b.lane_id, lane_b.road_id)
                )
                lane_b.intersecting_lanes.append(
                    DataContactLaneInfo(lane_a.lane_id, lane_a.road_id)
                )

                processed[key] = area

        return roads_list

    def add_landmarks_to_lanes(self, data_blocks: List[DataBlock]) -> None:
        """
        Adds landmarks to appropriate lanes in non-junction roads or maps them to
        specific approach/exit lanes in junction roads. This method processes a list
        of landmarks obtained from the map, identifies the corresponding road and
        lanes in the given data blocks, and attaches the landmarks to compatible lanes.

        Parameters:
            data_blocks (List[DataBlock]): A list of DataBlock objects containing road
            and lane information.
        """
        landmarks: List[Landmark] = self.ctx.map.get_all_landmarks()

        for landmark in landmarks:
            print(f"Converting Landmark {landmark.id}")
            data_landmark = DataLandmark.from_landmark(landmark)  # builds DataLandmark from CARLA Landmark
            road = self.get_specific_road_from_blocks(data_blocks, landmark.road_id)
            if not road:
                continue

            # --- non-junction roads: unchanged ------------------------------------
            if not road.is_junction:
                for lane in road.lanes:
                    if self.is_lane_valid_for_landmark(landmark, lane):
                        lane.landmarks.append(data_landmark)
                continue

            # --- junction roads: map to approaches (predecessors) / exits (successors)
            junction_lanes = [lane for lane in road.lanes if self.is_lane_valid_for_landmark(landmark, lane)]

            seen = set()

            def attach(road_id: int, lane_id: int):
                key = (road_id, lane_id, landmark.id)
                if key in seen:
                    return
                seen.add(key)
                src_road = self.get_specific_road_from_blocks(data_blocks, road_id)
                if not src_road or src_road.is_junction:
                    return  # only attach to approach/exit lanes outside the junction
                target = next((x for x in src_road.lanes if x.lane_id == lane_id), None)
                if target is not None:
                    target.landmarks.append(data_landmark)

            for junction_lane in junction_lanes:
                for pred in junction_lane.predecessor_lanes:
                    attach(pred.road_id, pred.lane_id)

    @staticmethod
    def collect_all_lanes_waypoints(starting_waypoints: List[Waypoint]) -> List[Waypoint]:
        """
        Given one or more seed waypoints on the same road, traverse left/right
        across all lanes without cycling, and return one representative
        Waypoint per lane.
        """
        visited: Set[Tuple[int, int]] = set()  # (road_id, lane_id)
        queue = deque(starting_waypoints)
        result: List[Waypoint] = []

        while queue:
            wp = queue.popleft()
            key = (wp.road_id, wp.lane_id)
            if key in visited:
                continue
            visited.add(key)
            result.append(wp)

            # Enqueue neighbors
            for neighbor in (wp.get_left_lane(), wp.get_right_lane()):
                if neighbor is not None:
                    queue.append(neighbor)

        return result

    @staticmethod
    def block_contains_waypoint(data_block: DataBlock, waypoint: Waypoint) -> bool:
        """
        Returns whether the given waypoint is located within the given data block.
        Args:
            data_block (DataBlock): The block used as reference point for the check
            waypoint (Waypoint): The waypoint to check for containment
        Returns:
            bool: True if the waypoint is within the given block, False otherwise
        """
        for data_road in data_block.roads:
            if data_road.road_id == waypoint.road_id:
                return True
        return False

    @staticmethod
    def get_specific_road_from_blocks(blocks: List[DataBlock], road_id: int) -> Optional[DataRoad]:
        """
        Returns the corresponding DataRoad to the given road_id from the list of DataBlocks.

        Args:
            blocks (List[DataBlock]): The DataBlocks in which should be searched for the given road_id
            road_id (int): The id of the DataRoad which should be retrieved

        Returns:
            Optional[DataRoad]: The DataRoad corresponding to the given road_id or None if not found
        """
        for block in blocks:
            for road in block.roads:
                if road.road_id == road_id:
                    return road
        return None

    @staticmethod
    def is_lane_valid_for_landmark(landmark: Landmark, data_lane: DataLane) -> bool:
        """
        Returns whether the given lane is valid for the given landmark based on lane validity information.

        Args:
            landmark (Landmark): The landmark for which the lane should be checked
            data_lane (DataLane): The lane that should be checked

        Returns:
            bool: True if the lane is valid for the given Landmark (i.e., its lane_id falls within one
                 of the landmark's validity intervals), False otherwise
        """
        lane_validities: List[Tuple[int, int]] = landmark.get_lane_validities()
        for tupl in lane_validities:
            # As the lane validity is given as a lane_id interval, we have to check
            # whether the lane_id is inside the current interval
            if tupl[0] <= data_lane.lane_id <= tupl[1]:
                return True
        return False
