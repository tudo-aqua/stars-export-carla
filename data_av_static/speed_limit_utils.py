from typing import List, Optional, TYPE_CHECKING, Dict, Tuple

from carla_data_classes.static import DataSpeedLimit, DataRoad
from carla_data_classes.static.DataBlock import DataBlock

if TYPE_CHECKING:
    # Only for type hints; avoid hard imports at runtime
    pass


class _SpeedLimitUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx
        # road_id -> DataRoad (populated in close_speed_limit_gaps / compute indices)
        self._road_index: Dict[int, DataRoad] = {}

    # -------------------------------------------------------------------------
    # Index helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def build_lane_index(blocks: List[DataBlock]):
        """Index lanes by (road_id, lane_id)."""
        return {(r.road_id, ln.lane_id): ln
                for b in blocks for r in b.roads for ln in r.lanes}

    @staticmethod
    def build_road_index(blocks: List[DataBlock]) -> Dict[int, DataRoad]:
        """Index roads by road_id."""
        out: Dict[int, DataRoad] = {}
        for b in blocks:
            for r in b.roads:
                out[r.road_id] = r
        return out

    # -------------------------------------------------------------------------
    # Junction awareness
    # -------------------------------------------------------------------------

    def _is_lane_in_junction(self, lane: "DataLane") -> bool:
        """True iff the lane's parent road is a junction road."""
        r: Optional[DataRoad] = self._road_index.get(lane.road_id)
        if r is None:
            return False
        if getattr(r, "is_junction", False):
            return True
        jn = getattr(r, "junction_id", None)
        return jn not in (None, 0, -1)

    def _blocks_propagation(self, lane: "DataLane") -> bool:
        """
        Returns True if propagation should STOP at this lane:
        - lane is in a junction AND
        - there is evidence of *joining* traffic: intersecting lanes or contact areas.
        Otherwise (not in junction, or isolated connector inside junction), allow crossing.
        """
        if not self._is_lane_in_junction(lane):
            return False
        # If our data model has per-lane intersections/contact areas (world_builder populates these)
        inter = getattr(lane, "intersecting_lanes", None) or []
        contacts = getattr(lane, "contact_areas", None) or []
        return len(inter) > 0 or len(contacts) > 0

    # -------------------------------------------------------------------------
    # Upstream inheritance (junctions allowed only when *no* road joins)
    # -------------------------------------------------------------------------

    def find_upstream_speed_mps(self, lane: "DataLane", lane_index, visited=None) -> Optional[float]:
        """
        Find a speed to inherit from upstream lanes only (for head-gap filling).
        - Does NOT consider the current lane’s own limits.
        - May traverse through junctions *only when no road joins into the lane path*,
          i.e. across junction lanes that have no intersecting lanes/contact areas.
        Returns m/s or None (caller will use default).
        """
        from collections import deque
        if visited is None:
            visited = set()

        # Seed with direct predecessors (skip current lane)
        q = deque()
        for pre in (getattr(lane, "predecessor_lanes", []) or []):
            q.append((pre.road_id, pre.lane_id))

        while q:
            key = q.popleft()
            if key in visited:
                continue
            visited.add(key)

            ln = lane_index.get(key)
            if ln is None:
                continue

            # If this lane blocks propagation (junction with joining roads), stop this branch
            if self._blocks_propagation(ln):
                continue

            # If this upstream lane has explicit limits, inherit its latest segment
            if ln.speed_limits:
                last = max(ln.speed_limits, key=lambda s: s.to_distance)
                return float(last.speed_limit)

            # Otherwise, continue walking upstream
            for pre in (getattr(ln, "predecessor_lanes", []) or []):
                pre_key = (pre.road_id, pre.lane_id)
                if pre_key not in visited:
                    q.append(pre_key)

        # No upstream limit found → caller should use default
        return None

    # -------------------------------------------------------------------------
    # Build segments from per-lane landmarks (run BEFORE closing gaps)
    # -------------------------------------------------------------------------

    def compute_speed_limits(self, blocks: List[DataBlock]) -> None:
        """
        For every lane in DataBlocks, read lane.landmarks (already filtered to that lane)
        and produce piecewise DataSpeedLimit segments (without filling gaps).
        Run this BEFORE close_speed_limit_gaps(...).
        """
        EPS = 1e-6
        # index roads for later checks if needed
        self._road_index = self.build_road_index(blocks)

        for block in blocks:
            for road in block.roads:
                for lane in road.lanes:
                    L = float(getattr(lane, "lane_length", 0.0) or 0.0)
                    if L <= 0.0:
                        lane.speed_limits = []
                        continue

                    lmarks = list(getattr(lane, "landmarks", []) or [])
                    if not lmarks:
                        lane.speed_limits = []
                        continue

                    # Build (kind, s, value_mps) events
                    events: List[Tuple[str, float, Optional[float]]] = []
                    for lm in lmarks:
                        tstr = self._landmark_type_str(lm)
                        is_begin = self._is_begin_type(tstr)
                        is_end = self._is_end_type(tstr)
                        if not (is_begin or is_end):
                            continue

                        s_attr = getattr(lm, "s", None)
                        if isinstance(s_attr, (int, float)):
                            s = max(0.0, min(float(s_attr), L))
                        else:
                            # project its (x,y) onto this lane if needed
                            loc = getattr(lm, "location", None)
                            if loc is not None:
                                s = self._project_s_on_lane(lane, float(loc.x), float(loc.y))
                            else:
                                tr = getattr(lm, "transform", None)
                                if tr is not None and getattr(tr, "location", None) is not None:
                                    s = self._project_s_on_lane(lane, float(tr.location.x), float(tr.location.y))
                                else:
                                    s = 0.0

                        if is_begin:
                            v_mps = self._value_to_mps(lm)
                            events.append(("begin", s, v_mps))
                        else:
                            events.append(("end", s, None))

                    if not events:
                        lane.speed_limits = []
                        continue

                    # Sort by s; process 'end' before 'begin' if identical s
                    events.sort(key=lambda e: (e[1], 0 if e[0] == "end" else 1))

                    segments: List[DataSpeedLimit] = []
                    curr_v: Optional[float] = None
                    seg_start: float = 0.0

                    for kind, s, v in events:
                        s = float(s)
                        if kind == "begin":
                            if curr_v is not None and s > seg_start + EPS:
                                segments.append(DataSpeedLimit(
                                    speed_limit=curr_v, from_distance=seg_start, to_distance=s
                                ))
                            curr_v = v
                            seg_start = s
                        else:  # "end"
                            if curr_v is not None and s > seg_start + EPS:
                                segments.append(DataSpeedLimit(
                                    speed_limit=curr_v, from_distance=seg_start, to_distance=s
                                ))
                            curr_v = None
                            seg_start = s

                    if curr_v is not None and L > seg_start + EPS:
                        segments.append(DataSpeedLimit(
                            speed_limit=curr_v, from_distance=seg_start, to_distance=L
                        ))

                    lane.speed_limits = segments

    # -------------------------------------------------------------------------
    # Gap closing (head/tail) – propagation per new rule
    # -------------------------------------------------------------------------

    def close_speed_limit_gaps(self, blocks, default_speed_kmh: float = 30.0) -> None:
        """
        Ensure continuous [0, L] coverage of speed limits (m/s) across all lanes.

        Two input styles are supported per lane:
          • Marker mode: one or more zero-length entries (to == from) meaning "speed becomes X at d".
          • Interval mode: proper segments [from, to] with explicit speeds.

        Behavior:
          • Marker mode → build piecewise-constant segments between markers; head uses default or
            upstream (via find_upstream_speed_mps) which *may* cross junctions only through lanes
            that have no joining roads.
          • Interval mode → honor intervals; fill only gaps with default/upstream.
        """
        default_mps = float(default_speed_kmh) / 3.6
        # Precompute indices once
        lane_index = self.build_lane_index(blocks)
        self._road_index = self.build_road_index(blocks)

        EPS = 1e-6

        for block in blocks:
            for road in block.roads:
                for lane in road.lanes:
                    lane_length = float(lane.lane_length or 0.0)
                    if lane_length <= 0.0:
                        lane.speed_limits = []
                        continue

                    sl = list(lane.speed_limits or [])
                    if not sl:
                        inherit = self.find_upstream_speed_mps(lane, lane_index)
                        spd = inherit if inherit is not None else default_mps
                        lane.speed_limits = [
                            DataSpeedLimit(speed_limit=spd, from_distance=0.0, to_distance=lane_length)
                        ]
                        continue

                    any_marker = any(abs(float(s.to_distance) - float(s.from_distance)) < EPS for s in sl)
                    out: List[DataSpeedLimit] = []

                    if any_marker:
                        # ---- MARKER MODE ------------------------------------------------------
                        markers: Dict[float, float] = {}
                        for s in sl:
                            d = float(s.from_distance)
                            if d < 0.0: d = 0.0
                            if d > lane_length: d = lane_length
                            markers[d] = float(s.speed_limit)

                        if not markers:
                            inherit = self.find_upstream_speed_mps(lane, lane_index)
                            spd = inherit if inherit is not None else default_mps
                            lane.speed_limits = [
                                DataSpeedLimit(speed_limit=spd, from_distance=0.0, to_distance=lane_length)
                            ]
                            continue

                        xs = sorted(markers.keys())

                        # Head [0, first)
                        head_end = xs[0]
                        if head_end > 0.0 + EPS:
                            inherit = self.find_upstream_speed_mps(lane, lane_index)
                            head_spd = inherit if inherit is not None else default_mps
                            out.append(DataSpeedLimit(
                                speed_limit=head_spd, from_distance=0.0, to_distance=head_end
                            ))

                        # Middle [xi, x(i+1))
                        for i in range(len(xs) - 1):
                            d0, d1 = xs[i], xs[i + 1]
                            if d1 > d0 + EPS:
                                out.append(DataSpeedLimit(
                                    speed_limit=markers[d0], from_distance=d0, to_distance=d1
                                ))

                        # Tail [last, lane_length]
                        last = xs[-1]
                        if lane_length > last + EPS:
                            out.append(DataSpeedLimit(
                                speed_limit=markers[last], from_distance=last, to_distance=lane_length
                            ))
                        elif lane_length > last:
                            out.append(DataSpeedLimit(
                                speed_limit=markers[last], from_distance=last, to_distance=lane_length
                            ))

                    else:
                        # ---- INTERVAL MODE ----------------------------------------------------
                        norm: List[DataSpeedLimit] = []
                        for s in sl:
                            s0 = max(0.0, min(float(s.from_distance), lane_length))
                            s1 = max(0.0, min(float(s.to_distance), lane_length))
                            if s1 > s0 + EPS:
                                norm.append(DataSpeedLimit(
                                    speed_limit=float(s.speed_limit), from_distance=s0, to_distance=s1
                                ))
                        norm.sort(key=lambda seg: (seg.from_distance, seg.to_distance))

                        out = []
                        cursor = 0.0
                        carried = None  # last known m/s

                        # Head gap [0, first.from)
                        if norm and norm[0].from_distance > 0.0 + EPS:
                            inherit = self.find_upstream_speed_mps(lane, lane_index)
                            head_spd = carried if carried is not None else (
                                inherit if inherit is not None else default_mps)
                            out.append(DataSpeedLimit(
                                speed_limit=head_spd, from_distance=0.0,
                                to_distance=min(norm[0].from_distance, lane_length)
                            ))
                            cursor = float(norm[0].from_distance)
                            carried = head_spd

                        # Segments & internal gaps
                        for seg in norm:
                            s0, s1, spd = float(seg.from_distance), float(seg.to_distance), float(seg.speed_limit)
                            if s1 <= cursor + EPS:
                                continue
                            if s0 > cursor + EPS:
                                gap_spd = carried if carried is not None else default_mps
                                out.append(DataSpeedLimit(speed_limit=gap_spd, from_distance=cursor, to_distance=s0))
                                cursor = s0
                            out.append(DataSpeedLimit(speed_limit=spd, from_distance=cursor, to_distance=s1))
                            cursor = s1
                            carried = spd
                            if cursor >= lane_length - EPS:
                                break

                        # Tail gap
                        if cursor < lane_length - EPS:
                            tail_spd = carried if carried is not None else (
                                    self.find_upstream_speed_mps(lane, lane_index) or default_mps
                            )
                            out.append(
                                DataSpeedLimit(speed_limit=tail_spd, from_distance=cursor, to_distance=lane_length))

                    lane.speed_limits = self.merge_adjacent_equal(out)

    # -------------------------------------------------------------------------
    # Landmark parsing helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def _landmark_type_str(lm) -> str:
        """Return a normalized type string/code for begin/end detection."""
        t = getattr(lm, "type", None)
        v = getattr(t, "value", None)
        if isinstance(v, (int, float)):
            return str(int(v))
        if isinstance(t, (int, float)):
            return str(int(t))
        return str(t).split(".")[-1].strip().lower() if t is not None else ""

    @staticmethod
    def _is_begin_type(tstr: str) -> bool:
        # OpenDRIVE codes: 274 MaximumSpeed, 275 MinSpeed start; some maps use 274.1
        begin_codes = {"274", "274.1", "275"}
        begin_names = {"maximumspeed", "speedlimit", "speed_limit", "speed"}
        if tstr in begin_codes:
            return True
        return tstr in begin_names

    @staticmethod
    def _is_end_type(tstr: str) -> bool:
        # OpenDRIVE codes: 278 end of maximum speed, 279 end of min speed; some maps use 274.2
        end_codes = {"278", "279", "274.2"}
        end_names = {"endspeedlimit", "endofspeed", "speed_limit_end", "endspeed", "endmaximumspeed"}
        if tstr in end_codes:
            return True
        return tstr in end_names

    @staticmethod
    def _value_to_mps(lm) -> Optional[float]:
        """
        Interpret landmark.value as km/h by default; accept m/s explicitly.
        """
        val = getattr(lm, "value", None)
        if val is None:
            return None
        try:
            v = float(val)
        except Exception:
            return None
        u = (getattr(lm, "unit", "") or "").strip().lower()
        if u in ("m/s", "mps", "meter_per_second", "meters_per_second"):
            return v
        return v / 3.6  # default: km/h → m/s

    @staticmethod
    def _project_s_on_lane(lane, x: float, y: float) -> float:
        """
        Project a (x,y) world point onto the lane center polyline using
        lane.lane_midpoints[*].location{.x,.y} and .distance_to_start.
        Returns s in meters along the lane, clamped to [0, lane_length].
        """
        mps = getattr(lane, "lane_midpoints", None) or []
        if not mps:
            return 0.0
        best_d2 = float("inf")
        best_s = 0.0
        for i in range(len(mps) - 1):
            p0 = mps[i].location
            p1 = mps[i + 1].location
            s0 = float(mps[i].distance_to_start)
            s1 = float(mps[i + 1].distance_to_start)
            vx, vy = (p1.x - p0.x), (p1.y - p0.y)
            wx, wy = (x - p0.x), (y - p0.y)
            seg_len2 = vx * vx + vy * vy
            if seg_len2 <= 1e-9:
                t = 0.0
            else:
                t = (vx * wx + vy * wy) / seg_len2
                if t < 0.0: t = 0.0
                if t > 1.0: t = 1.0
            px = p0.x + t * vx
            py = p0.y + t * vy
            dx, dy = (x - px), (y - py)
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_s = s0 + t * (s1 - s0)
        L = float(getattr(lane, "lane_length", 0.0) or 0.0)
        if L <= 0.0:
            return max(0.0, best_s)
        return max(0.0, min(best_s, L))

    # -------------------------------------------------------------------------
    # Segment merge utility
    # -------------------------------------------------------------------------

    @staticmethod
    def merge_adjacent_equal(speed_limits: List[DataSpeedLimit], eps: float = 1e-6) -> List[DataSpeedLimit]:
        """
        Merge touching segments with identical speeds.
        """
        if not speed_limits:
            return speed_limits
        speed_limits.sort(key=lambda limit: (limit.from_distance, limit.to_distance))
        out = [speed_limits[0]]
        for speed_limit in speed_limits[1:]:
            last = out[-1]
            touches = abs(float(last.to_distance) - float(speed_limit.from_distance)) < eps
            same = abs(float(last.speed_limit) - float(speed_limit.speed_limit)) < 1e-6
            if touches and same:
                last.to_distance = max(float(last.to_distance), float(speed_limit.to_distance))
            else:
                out.append(speed_limit)
        return out
