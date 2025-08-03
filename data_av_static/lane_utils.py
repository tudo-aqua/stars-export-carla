from typing import List, Tuple, TYPE_CHECKING

from carla import Waypoint, Actor, Location, Landmark
from shapely import LineString, Point

from carla_data_classes import (
    DataLane, DataLaneMidpoint, DataLocation, DataRotation, DataLaneType,
    DataContactLaneInfo, DataSpeedLimit
)

if TYPE_CHECKING:
    from .rasterizer import MapRasterizer


class _LaneUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    def get_lane_midpoints_array(self) -> List[DataLaneMidpoint]:
        """
        Return a flat list of all-lane midpoints from computed blocks.
        """
        roads = self.ctx.flatten(list(map(lambda b: b.roads, self.ctx.blocks)))
        lanes = self.ctx.flatten(list(map(lambda r: r.lanes, roads)))
        lane_midpoints = self.ctx.flatten(list(map(lambda l: l.lane_midpoints, lanes)))
        return lane_midpoints

    def get_closest_lane_midpoint(self, location: DataLocation) -> DataLaneMidpoint:
        """
        Return the nearest midpoint using the KDTree built in IO ops.
        """
        distance, index = self.ctx.kd_tree.query((location.x, location.y, location.z))
        return self.ctx.lane_midpoints[index]

    def get_data_lane_for_waypoint(self, waypoint: Waypoint, landmarks: List[Landmark]) -> DataLane:
        """
        Returns the filled DataLane. Collects information about pre-/successor lanes, intersections, etc.
        
        Args:
            waypoint: The waypoint from which the information should be collected
            landmarks: List of landmarks used to populate speed limits and traffic lights
            
        Returns:
            DataLane: A DataLane object containing lane information including
                - Basic properties (road_id, lane_id, type, width, length)
                - Predecessor and successor lanes
                - Lane midpoints with locations
                - Speed limits
                - Traffic lights
                - Contact areas and landmarks
        """
        print(f"Converting road {waypoint.road_id} with lane {waypoint.lane_id}")
        # Get the length of the lane
        lane_length: float = self.get_length_of_lane(waypoint)
        # Get a list of leading lanes
        predecessor_lanes: List[DataContactLaneInfo] = self.get_predecessor_contact_infos(waypoint)
        # Get a list of follow-up lanes
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

        # Build and return DataLane
        data_lane = DataLane(road_id=waypoint.road_id, lane_id=waypoint.lane_id,
                             lane_type=DataLaneType(int(waypoint.lane_type)), lane_width=waypoint.lane_width,
                             lane_length=lane_length, s=waypoint.s, predecessor_lanes=predecessor_lanes,
                             successor_lanes=successor_lanes, intersecting_lanes=[], lane_midpoints=lane_midpoints,
                             speed_limits=[], landmarks=[], contact_areas=[], traffic_lights=[])

        geom = LineString([(m.location.x, m.location.y) for m in lane_midpoints])
        data_lane._geom = geom
        data_lane.speed_limits = self._compute_speed_limits_for_lane(
            geom=geom,
            lane_id=waypoint.lane_id,
            road_id=waypoint.road_id,
            landmarks=landmarks,
            lane_length=lane_length
        )
        return data_lane

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
        Returns a list of all predecessor lanes for the given lane
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
        Returns a list of all successor lanes for the given lane
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
        waypoint = self.ctx.map.get_waypoint(location)
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
        # Add waypoints until the start of the lane-to-waypoint list
        all_waypoints.extend(waypoints_until_start)
        # Add the current waypoint as it is not included in the previous list
        all_waypoints.append(lane)
        # Get all waypoints until the lane end
        waypoints_until_end = self.get_all_waypoints_until_end_of_lane(lane, precision)
        # Add waypoints until the end of the lane to a waypoint list
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

    def _compute_speed_limits_for_lane(self, geom: LineString, lane_id: int, road_id: int,
                                       landmarks: List[Landmark], lane_length: float) -> List[DataSpeedLimit]:
        """
        Build [start_s, end_s) speed-limit segments for this lane.
        """
        # OpenDRIVE/German codes commonly used by CARLA maps
        begin_codes = {"274", "274.1", "275"}  # max speed, zone begin, min speed begin
        end_codes = {"278", "274.2", "279"}  # end of max speed, zone end, min speed end

        # Filter landmarks affecting this road and lane
        def affects_lane(landmark):
            if landmark.road_id != road_id:
                return False
            for a, b in landmark.get_lane_validities():
                if a <= lane_id <= b:
                    return True
            return False

        speed_events = []
        for lm in landmarks:
            t = str(lm.type)
            if t not in begin_codes and t not in end_codes:
                continue
            if not affects_lane(lm):
                continue

            # project landmark XY onto lane center-line to get s
            loc = lm.transform.location
            s = geom.project(Point(loc.x, loc.y))
            s = max(0.0, min(float(s), float(lane_length)))

            if t in begin_codes:
                # lm.value is km/h in CARLA’s Landmark; convert to m/s
                val_mps = self._kmh_to_mps(float(lm.value)) if getattr(lm, "value", None) is not None else None
                speed_events.append(("begin", s, val_mps))
            else:
                speed_events.append(("end", s, None))

        # No signs → leave empty
        if not speed_events:
            return []

        # sort by distance along lane
        speed_events.sort(key=lambda e: (e[1], 0 if e[0] == "end" else 1))
        segments = []
        curr_v = None
        seg_start = 0.0

        for kind, s, v in speed_events:
            s = float(s)
            if kind == "begin":
                # close a previous segment if any
                if curr_v is not None and s > seg_start:
                    segments.append(DataSpeedLimit(from_distance=seg_start, to_distance=s, speed_limit=curr_v))
                curr_v = v
                seg_start = s
            else:  # "end"
                if curr_v is not None and s > seg_start:
                    segments.append(DataSpeedLimit(from_distance=seg_start, to_distance=s, speed_limit=curr_v))
                curr_v = None
                seg_start = s

        # tail segment to the lane end if a limit is still active
        if curr_v is not None and lane_length > seg_start:
            segments.append(DataSpeedLimit(from_distance=seg_start, to_distance=lane_length, speed_limit=curr_v))

        return segments

    @staticmethod
    def get_all_waypoints_until_start_of_lane(lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of all waypoints until the start of the current lane
        :param lane: The lane from which the waypoints should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints until the lane starts
        """
        # A manual cycle is necessary as sometimes the previous_until_lane_start will result in a SIGSEGV error
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
    def get_all_waypoints_until_end_of_lane(lane: Waypoint, precision: float = 2.0) -> List[Waypoint]:
        """
        Returns a list of all waypoints until the end of the current lane
        :param lane: The lane from which the waypoints should be returned
        :param precision: Default: 2.0. Sets the search distance for the next waypoints
        :return: List of Waypoints until the lane ends
        """
        # A manual cycle is necessary as sometimes the previous_until_lane_start will result in a SIGSEGV error
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
    def _kmh_to_mps(v):
        """Convert km/h to m/s (helper for speed limits)."""
        return v / 3.6 if v is not None else None
