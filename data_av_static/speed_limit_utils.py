from typing import List, Optional, TYPE_CHECKING, Dict, Tuple

from carla_data_classes.static import DataSpeedLimit, DataRoad
from carla_data_classes.static.DataBlock import DataBlock

if TYPE_CHECKING:
    # Only for type hints; avoid hard imports at runtime
    pass


class _SpeedLimitUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx
        # road_id -> DataRoad (populated in close_speed_limit_gaps)
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
    # Junction detection (by parent road)
    # -------------------------------------------------------------------------

    def _is_junction_lane(self, lane: "DataLane") -> bool:
        """
        True iff the lane's parent road is a junction road.
        Uses DataRoad.is_junction and (optionally) junction_id.
        """
        r: Optional[DataRoad] = self._road_index.get(lane.road_id)
        if r is None:
            return False
        if getattr(r, "is_junction", False):
            return True
        jn = getattr(r, "junction_id", None)
        # treat any non-None/non-zero junction_id as junction
        return jn not in (None, 0, -1)

    # -------------------------------------------------------------------------
    # Upstream inheritance (blocked by junction roads)
    # -------------------------------------------------------------------------

    def find_upstream_speed_mps(self, lane: "DataLane", lane_index, visited=None) -> Optional[float]:
        """
        Find a speed to inherit from upstream *straight* lanes only.
        - Do NOT consider the current lane’s own speed limits.
        - Never traverse through junction lanes.
        Returns m/s or None (caller will use default).
        """
        # If the target lane itself is in a junction, we don't inherit through it
        if self._is_junction_lane(lane):
            return None

        from collections import deque
        if visited is None:
            visited = set()

        q = deque()

        # Seed search with direct predecessors (skip the current lane)
        for pre in (getattr(lane, "predecessor_lanes", []) or []):
            q.append((pre.road_id, pre.lane_id))

        while q:
            key = q.popleft()
            if key in visited:
                continue
            visited.add(key)

            ln = lane_index.get(key)
            if not ln:
                continue

            # Stop traversing when encountering a junction lane
            if self._is_junction_lane(ln):
                continue

            # If this straight lane has explicit limits, use its latest segment
            if ln.speed_limits:
                last = max(ln.speed_limits, key=lambda s: s.to_distance)
                return float(last.speed_limit)

            # Otherwise, keep walking strictly through straight predecessors
            for pre in (getattr(ln, "predecessor_lanes", []) or []):
                pre_key = (pre.road_id, pre.lane_id)
                if pre_key not in visited:
                    q.append(pre_key)

        # No upstream limit found → caller should use default
        return None

    def compute_speed_limits(self, blocks: List[DataBlock]) -> None:
        """
        For every lane in DataBlocks, read lane.landmarks (already filtered to that lane)
        and produce piecewise DataSpeedLimit segments (without filling gaps).
        Run this BEFORE close_speed_limit_gaps(...).
        """
        EPS = 1e-6

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

                        # Only accept speed-related landmarks
                        if not (is_begin or is_end):
                            continue

                        # Prefer explicit s on the landmark if present, else project from XY
                        s_attr = getattr(lm, "s", None)
                        if isinstance(s_attr, (int, float)):
                            s = float(s_attr)
                            # clamp
                            s = max(0.0, min(s, L))
                        else:
                            # project its (x,y) onto this lane
                            loc = getattr(lm, "location", None)
                            if loc is not None:
                                s = self._project_s_on_lane(lane, float(loc.x), float(loc.y))
                            else:
                                # try CARLA transform.location
                                tr = getattr(lm, "transform", None)
                                if tr is not None and getattr(tr, "location", None) is not None:
                                    s = self._project_s_on_lane(lane, float(tr.location.x), float(tr.location.y))
                                else:
                                    s = 0.0

                        if is_begin:
                            v_mps = self._value_to_mps(lm)
                            events.append(("begin", s, v_mps))
                        elif is_end:
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

                    # Tail segment if still active
                    if curr_v is not None and L > seg_start + EPS:
                        segments.append(DataSpeedLimit(
                            speed_limit=curr_v, from_distance=seg_start, to_distance=L
                        ))

                    lane.speed_limits = segments

    def close_speed_limit_gaps(self, blocks, default_speed_kmh: float = 30.0) -> None:
        """
        Ensure continuous [0, L] coverage of speed limits (m/s) across all lanes.

        Two input styles are supported per lane:
          • Marker mode: one or more zero-length entries (to == from) meaning "speed becomes X at d".
          • Interval mode: proper segments [from, to] with explicit speeds.

        Behavior:
          • Marker mode → build piecewise-constant segments between markers; head uses default/upstream; tail uses last marker.
          • Interval mode → honor intervals; fill only gaps with default/upstream.
        In both modes, upstream inheritance NEVER crosses junction roads.
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
                        # No info at all → inherit (no junction) or default
                        inherit = self.find_upstream_speed_mps(lane, lane_index)
                        spd = inherit if inherit is not None else default_mps
                        lane.speed_limits = [
                            DataSpeedLimit(speed_limit=spd, from_distance=0.0, to_distance=lane_length)
                        ]
                        continue

                    # Determine mode: marker vs interval
                    any_marker = any(abs(float(s.to_distance) - float(s.from_distance)) < EPS for s in sl)

                    out: List[DataSpeedLimit] = []

                    if any_marker:
                        # ---- MARKER MODE ------------------------------------------------------
                        # Build sorted unique markers (d -> speed)
                        markers: Dict[float, float] = {}
                        for s in sl:
                            d = float(s.from_distance)
                            if d < 0.0:
                                d = 0.0
                            if d > lane_length:
                                d = lane_length
                            # if multiple markers at the same 'd', keep the last one in input order
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
                            # tiny sliver
                            out.append(DataSpeedLimit(
                                speed_limit=markers[last], from_distance=last, to_distance=lane_length
                            ))

                    else:
                        # ---- INTERVAL MODE ----------------------------------------------------
                        # Clip, sort, and coalesce exact duplicates
                        norm: List[DataSpeedLimit] = []
                        for s in sl:
                            s0 = max(0.0, min(float(s.from_distance), lane_length))
                            s1 = max(0.0, min(float(s.to_distance), lane_length))
                            if s1 > s0 + EPS:
                                norm.append(DataSpeedLimit(
                                    speed_limit=float(s.speed_limit), from_distance=s0, to_distance=s1
                                ))
                        norm.sort(key=lambda seg: (seg.from_distance, seg.to_distance))

                        # Now fill gaps without crossing junctions
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

                    # Final tidy-up
                    lane.speed_limits = self.merge_adjacent_equal(out)

    @staticmethod
    def _landmark_type_str(lm) -> str:
        """Return a normalized type string/code for begin/end detection."""
        t = getattr(lm, "type", None)
        # Enum with .value code (e.g., 274, 278)
        v = getattr(t, "value", None)
        if isinstance(v, (int, float)):
            return str(int(v))
        # Direct numeric
        if isinstance(t, (int, float)):
            return str(int(t))
        # Enum/name path → last token, lowercased
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
        (Your data says km/h is the correct interpretation.)
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
        # Find closest segment and compute projected s
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
                # degenerate segment; just use s0
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
                # interpolate s along this segment
                best_s = s0 + t * (s1 - s0)
        # clamp to lane length
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
