import math
from collections import deque
from typing import List, Optional, Set, Tuple, Dict, TYPE_CHECKING

from carla import Junction, Landmark, Waypoint, LaneType
from shapely import Point, LineString, STRtree
from shapely.ops import nearest_points

from carla_data_classes.enums.DataLandmarkType import DataLandmarkType
from carla_data_classes.static import DataRoad, DataLandmark, DataLane, DataLocation, DataContactArea, \
    DataContactLaneInfo
from carla_data_classes.static.DataBlock import DataBlock
from carla_data_classes.static.DataCrosswalk import DataCrosswalk
from carla_data_classes.static.DataJunction import DataJunction
from carla_data_classes.static.DataWorld import DataWorld
from data_av_static.traffic_light_utils import _TrafficLightUtils

if TYPE_CHECKING:
    pass


class _BlockBuilder:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    @staticmethod
    def _build_data_world(blocks: List[DataBlock], crosswalks: List[DataCrosswalk]) -> DataWorld:
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

        return DataWorld(straights=straights, junctions=list(junctions_by_id.values()), crosswalks=crosswalks)

    def get_data_world(self, distance: float = 0.1) -> DataWorld:
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
        self.ctx.compute_speed_limits(data_blocks)
        self.ctx.close_speed_limit_gaps(data_blocks, default_speed_kmh=30.0)

        crosswalks = self._collect_crosswalks()
        data_world = self._build_data_world(blocks=data_blocks, crosswalks=crosswalks)

        self.ctx.data_world = data_world
        return data_world

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
                    DataContactLaneInfo(road_id=lane_b.road_id, lane_id=lane_b.lane_id)
                )
                lane_b.intersecting_lanes.append(
                    DataContactLaneInfo(road_id=lane_a.road_id, lane_id=lane_a.lane_id)
                )

                processed[key] = area

        return roads_list

    def _is_speed_limit_landmark(self, landmark) -> bool:
        """True for (end of) maximum/minimum speed signs; never map these to predecessors."""
        t = getattr(landmark, "type", None)
        # Prefer enum equality when available
        try:
            if t in (
                    DataLandmarkType.MaximumSpeed,
                    getattr(DataLandmarkType, "EndMaximumSpeed", None),
                    getattr(DataLandmarkType, "MinimumSpeed", None),
                    getattr(DataLandmarkType, "EndMinimumSpeed", None),
            ):
                return True
        except Exception:
            pass
        # Fallbacks: OpenDRIVE numeric codes + name contains “speed”
        code = getattr(t, "value", None)
        if isinstance(code, (int, float)) and int(code) in (274, 278, 275,
                                                            279):  # 274 max, 278 end max, 275 min, 279 end min
            return True
        name = (str(t) if t is not None else "").lower()
        if "maximumspeed" in name or "endspeed" in name or "speedlimit" in name:
            # avoid false positives like "speedbump"
            return not ("bump" in name or "hump" in name)
        return False

    def _is_control_landmark(self, landmark) -> bool:
        """True for TL/Stop/Yield/etc.—these get mapped to predecessor approach lanes."""
        t = getattr(landmark, "type", None)
        name = (str(t) if t is not None else "").lower()
        # Common control types; extend as needed for your enum set
        keywords = ("stop", "allwaystop", "yield", "giveway", "priority")
        return _TrafficLightUtils.is_light_landmark(landmark) or any(k in name for k in keywords)

    def add_landmarks_to_lanes(self, data_blocks: List[DataBlock]) -> None:
        landmarks: List[Landmark] = self.ctx.map.get_all_landmarks()

        EPS_AT_END = 0.05  # keep s inside [0, lane_length] to avoid rendering/logic drops

        for landmark in landmarks:
            print(f">> [Data-AV Transformer] Converting Landmark {landmark.id}")
            road = self.get_specific_road_from_blocks(data_blocks, landmark.road_id)
            if not road:
                continue

            # -- Non-junction roads: attach directly to valid lanes (unchanged) ----
            if not road.is_junction:
                for lane in road.lanes:
                    if self.is_lane_valid_for_landmark(landmark, lane):
                        dl = DataLandmark.from_landmark(landmark)  # clone per lane
                        if lane.lane_id > 0:
                            dl.s = lane.lane_length - landmark.s  # keep your positive-lane flip on same road
                        lane.landmarks.append(dl)
                continue

            # -- Junction roads: split by type -------------------------------------
            if self._is_speed_limit_landmark(landmark):
                # SPEED LIMITS: keep on the specified (junction) road’s valid lanes
                for lane in road.lanes:
                    if self.is_lane_valid_for_landmark(landmark, lane):
                        dl = DataLandmark.from_landmark(landmark)
                        if lane.lane_id > 0:
                            dl.s = lane.lane_length - landmark.s
                        lane.landmarks.append(dl)
                continue  # do not fall through to predecessor mapping

            if self._is_control_landmark(landmark):
                # CONTROL LANDMARKS (TL/Stop/Yield...):
                # Map to predecessor approach lanes outside the junction.
                junction_lanes = [lane for lane in road.lanes if self.is_lane_valid_for_landmark(landmark, lane)]
                seen = set()  # (road_id, lane_id, landmark_id)

                def attach_to_predecessor(road_id: int, lane_id: int):
                    key = (int(road_id), int(lane_id), int(landmark.id))
                    if key in seen:
                        return
                    seen.add(key)

                    src_road = self.get_specific_road_from_blocks(data_blocks, road_id)
                    if not src_road or src_road.is_junction:
                        return  # only approach lanes outside the junction

                    target = next((x for x in src_road.lanes if x.lane_id == lane_id), None)
                    if target is None:
                        return

                    # Avoid duplicates by landmark id
                    if any(getattr(lm, "id", None) == landmark.id for lm in (target.landmarks or [])):
                        return

                    dl = DataLandmark.from_landmark(landmark)
                    # Put control exactly at the end of the approach lane (inside bounds)
                    L = float(getattr(target, "lane_length", 0.0) or 0.0)
                    dl.s = max(0.0, L - EPS_AT_END)
                    target.landmarks.append(dl)

                for jl in junction_lanes:
                    for pred in (getattr(jl, "predecessor_lanes", None) or []):
                        attach_to_predecessor(pred.road_id, pred.lane_id)

                continue

            # Other landmark types on junction roads: keep only on valid junction lanes (no predecessor mapping)
            for lane in road.lanes:
                if self.is_lane_valid_for_landmark(landmark, lane):
                    dl = DataLandmark.from_landmark(landmark)
                    if lane.lane_id > 0:
                        dl.s = lane.lane_length - landmark.s
                    lane.landmarks.append(dl)

    def _collect_crosswalks(self) -> List[DataCrosswalk]:
        """
        Parse CARLA's flat list of Location into multiple crosswalk polygons.
        The list encodes polygons as sequences where the first point is repeated
        at the end to mark closure: A, B, C, A, D, E, F, D, ...
        """
        try:
            locs = self.ctx.map.get_crosswalks()  # list[carla.Location]
        except Exception:
            return []

        if not locs:
            return []

        def same(a, b, eps=1e-3) -> bool:
            # tolerant equality for float coords (meters)
            return (math.isclose(a.x, b.x, abs_tol=eps) and
                    math.isclose(a.y, b.y, abs_tol=eps) and
                    math.isclose(getattr(a, "z", 0.0), getattr(b, "z", 0.0), abs_tol=eps))

        result: List[DataCrosswalk] = []
        i, n, cw_id = 0, len(locs), 0

        while i < n:
            start = locs[i]
            # find the next index j > i where locs[j] == start (polygon closure)
            j = i + 1
            while j < n and not same(locs[j], start):
                j += 1

            if j >= n:
                # incomplete tail (no closing repeat) -> ignore the remainder
                break

            # vertices are from i .. j-1 (exclude the repeated closing point at j)
            poly = locs[i:j]
            if len(poly) >= 3:  # at least a triangle
                vertices = [DataLocation(p.x, p.y, getattr(p, "z", 0.0)) for p in poly]
                result.append(DataCrosswalk(crosswalk_id=cw_id, vertices=vertices))
                cw_id += 1

            # continue after the closing repeated point
            i = j + 1

        return result

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
