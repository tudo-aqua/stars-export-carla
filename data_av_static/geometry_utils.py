from typing import List, Any, TYPE_CHECKING

import numpy as np
from shapely import LineString, Point, MultiLineString, MultiPoint, GeometryCollection, Polygon
from shapely.ops import nearest_points

from carla_data_classes.static import DataLocation, DataLane

if TYPE_CHECKING:
    from .rasterizer import MapRasterizer


class _GeometryUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    @staticmethod
    def flatten(list_to_flatten: List[List[Any]]) -> List[Any]:
        """
        Flatten a list of lists one level.
        """
        return [item for sublist in list_to_flatten for item in sublist]

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

    def nose_point(self, overlap, geom_a: LineString, geom_b: LineString, point_is_seed: bool = False) -> Point:
        """
        Return the first shared point (nose) of two overlapping lanes.
        Guaranteed to return a shapely Point.
        """
        cand: List[Point] = []

        def add_endpoints(linestring: LineString):
            c = list(linestring.coords)
            cand.append(Point(c[0]))
            cand.append(Point(c[-1]))

        if isinstance(overlap, Point) and point_is_seed:
            return self._scan_to_nose(geom_a, geom_b, overlap, tol=0.01, step=0.01)

        # collect candidates
        if isinstance(overlap, Point):
            return overlap

        if isinstance(overlap, LineString):
            add_endpoints(overlap)

        elif isinstance(overlap, MultiLineString):
            for ls in overlap.geoms:
                add_endpoints(ls)

        elif isinstance(overlap, MultiPoint):
            cand.extend(overlap.geoms)

        elif isinstance(overlap, GeometryCollection):
            for g in overlap.geoms:
                if isinstance(g, Point):
                    cand.append(g)
                elif isinstance(g, (LineString, MultiLineString)):
                    if isinstance(g, LineString):
                        add_endpoints(g)
                    else:
                        for ls in g.geoms:
                            add_endpoints(ls)
                # Polygons can appear in weird OpenDRIVE edge cases
                elif isinstance(g, Polygon):
                    cand.extend([Point(c) for c in g.exterior.coords[:2]])

        if cand:
            rough_pt = min(cand, key=lambda pt: min(geom_a.project(pt),
                                                    geom_b.project(pt)))
            point = self._scan_to_nose(geom_a, geom_b, rough_pt, tol=0.01, step=0.01)
            return point

        # ---------- fall-back if still no candidate ----------
        # 1) overlap centroid (always a Point)
        centroid = overlap.centroid
        if not centroid.is_empty:
            return centroid

        # 2) nearest points between the centre-lines (never fails)
        p, _ = nearest_points(geom_a, geom_b)
        point = self._scan_to_nose(geom_a, geom_b, p, tol=0.01, step=0.01)
        return point

    @staticmethod
    def _scan_to_nose(geom_a: LineString,
                      geom_b: LineString,
                      start_pt: Point,
                      tol: float = 0.30,
                      step: float = 0.10) -> Point:
        """
        Return the earliest point where the two lanes come within `tol`
        even when the initial rough point is at s=0 on either lane.
        """

        def _forward(line_from, line_other):
            s, length = 0.0, line_from.length
            while s < length:
                pt = line_from.interpolate(s)
                if pt.distance(line_other) >= tol:
                    return pt, min(line_from.project(pt), line_other.project(pt))
                s += step
            return None, float('inf')

        def _backward(line_from, line_other, s_start):
            s = s_start
            while s > 0.0:
                s_prev = max(0.0, s - step)
                pt_prev = line_from.interpolate(s_prev)
                if pt_prev.distance(line_other) > tol:
                    return line_from.interpolate(s), min(
                        line_from.project(line_from.interpolate(s)),
                        line_other.project(line_from.interpolate(s)))
                s = s_prev
            # reached the beginning
            pt0 = line_from.interpolate(0.0)
            return pt0, min(line_from.project(pt0), line_other.project(pt0))

        # --- run both directions on both lanes -------------------------
        best_pt, best_score = None, float(0)

        # forward scan from start of each lane
        for g_from, g_other in ((geom_a, geom_b), (geom_b, geom_a)):
            pt, sc = _forward(g_from, g_other)
            if sc > best_score:
                best_pt, best_score = pt, sc

        # backward scan from rough point on each lane
        s_a = geom_a.project(start_pt)
        s_b = geom_b.project(start_pt)
        for g_from, g_other, s_start in ((geom_a, geom_b, s_a),
                                         (geom_b, geom_a, s_b)):
            pt, sc = _backward(g_from, g_other, s_start)
            if sc > best_score:
                best_pt, best_score = pt, sc

        return best_pt

    def get_distance_to_start_of_lane(self, lane: DataLane, point: DataLocation) -> float:
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
            relative_distance = self.distance_between(midpoint, point)
            # Check if closer than the previous midpoints
            if relative_distance < min_distance:
                # Save minimal distance to any midpoint
                min_distance = relative_distance
                # Save distance to start for current midpoint
                minimum_distance = distance
        return minimum_distance

    def blocks_contain_waypoint(self, lane_id: int, road_id: int) -> bool:
        for block in self.ctx.blocks:
            for road in block.roads:
                for lane in road.lanes:
                    if lane.lane_id == lane_id and lane.road_id == road_id:
                        return True
        return False
