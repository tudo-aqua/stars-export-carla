from collections import deque
from typing import List, Optional, TYPE_CHECKING

from carla_data_classes import DataBlock, DataLane, DataSpeedLimit

if TYPE_CHECKING:
    from .rasterizer import MapRasterizer

class _SpeedLimitUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    @staticmethod
    def build_lane_index(blocks: List[DataBlock]):
        """Index lanes by (road_id, lane_id)."""
        return {(r.road_id, ln.lane_id): ln
                for b in blocks for r in b.roads for ln in r.lanes}

    @staticmethod
    def find_upstream_speed_mps(lane: DataLane, lane_index, visited=None) -> Optional[float]:
        """
        Walk predecessor graph to find a speed to inherit.
        Assumes DataSpeedLimit.speed_limit is stored in m/s.
        Returns m/s or None.
        """
        if visited is None:
            visited = set()
        q = deque([(lane.road_id, lane.lane_id)])
        while q:
            key = q.popleft()
            if key in visited:
                continue
            visited.add(key)

            ln = lane_index.get(key)
            if not ln:
                continue

            if ln.speed_limits:
                last = max(ln.speed_limits, key=lambda s: s.to_distance)
                return float(last.speed_limit)  # m/s
            for pre in getattr(ln, "predecessor_lanes", []) or []:
                q.append((pre.road_id, pre.lane_id))
        return None

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
            touches = abs(last.to_distance - speed_limit.from_distance) < eps
            same = abs(float(last.speed_limit) - float(speed_limit.speed_limit)) < 1e-6
            if touches and same:
                last.to_distance = max(last.to_distance, speed_limit.to_distance)
            else:
                out.append(speed_limit)
        return out

    def close_speed_limit_gaps(self, blocks: List[DataBlock], default_speed_kmh: float = 30.0) -> None:
        """
        Ensure continuous [0, L] coverage of speed limits in m/s across all lanes.
        """
        default_mps = float(default_speed_kmh) / 3.6
        lane_index = self.build_lane_index(blocks)

        for block in blocks:
            for road in block.roads:
                for lane in road.lanes:
                    lane_length = lane.lane_length
                    speed_limits = lane.speed_limits

                    # No segments → inherit or default across the full lane
                    if not speed_limits:
                        inherit = self.find_upstream_speed_mps(lane, lane_index)
                        spd = inherit if inherit is not None else default_mps
                        lane.speed_limits = [
                            DataSpeedLimit(speed_limit=spd, from_distance=0.0, to_distance=lane_length)]
                        continue

                    # Normalize and clamp to [0, lane_length]
                    speed_limits.sort(key=lambda limit: limit.from_distance)
                    norm = []
                    for s in speed_limits:
                        s0 = max(0.0, min(float(s.from_distance), lane_length))
                        s1 = max(0.0, min(float(s.to_distance), lane_length))
                        if s1 > s0:
                            # keep speed_limit in m/s
                            norm.append(DataSpeedLimit(speed_limit=float(s.speed_limit),
                                                       from_distance=s0, to_distance=s1))
                    speed_limits = norm

                    out = []
                    cursor = 0.0
                    carried = None  # last known m/s

                    # Head gap
                    if speed_limits and speed_limits[0].from_distance > 0.0:
                        inherit = self.find_upstream_speed_mps(lane, lane_index)
                        head_spd = carried if carried is not None else (inherit if inherit is not None else default_mps)
                        out.append(DataSpeedLimit(speed_limit=head_spd, from_distance=0.0,
                                                  to_distance=min(speed_limits[0].from_distance, lane_length)))
                        cursor = speed_limits[0].from_distance
                        carried = head_spd

                    # Segments & internal gaps
                    for s in speed_limits:
                        s0, s1, spd = float(s.from_distance), float(s.to_distance), float(s.speed_limit)  # m/s
                        if s1 <= cursor:
                            continue
                        if s0 > cursor:
                            gap_spd = carried if carried is not None else default_mps
                            out.append(DataSpeedLimit(speed_limit=gap_spd, from_distance=cursor, to_distance=s0))
                            cursor = s0
                        out.append(DataSpeedLimit(speed_limit=spd, from_distance=cursor, to_distance=s1))
                        cursor = s1
                        carried = spd
                        if cursor >= lane_length:
                            break

                    # Tail gap
                    if cursor < lane_length:
                        tail_spd = carried if carried is not None else (
                                self.find_upstream_speed_mps(lane, lane_index) or default_mps)
                        out.append(DataSpeedLimit(speed_limit=tail_spd, from_distance=cursor, to_distance=lane_length))

                    lane.speed_limits = self.merge_adjacent_equal(out)
