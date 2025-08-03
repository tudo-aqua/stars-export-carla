from typing import TYPE_CHECKING

from carla_data_classes import DataRoad

if TYPE_CHECKING:
    from .rasterizer import MapRasterizer


class _DebugUtils:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    def debug_road(self, data_road: DataRoad) -> None:
        """
        Draw per-lane labels at midpoints for debugging.
        """
        for data_lane in data_road.lanes:
            for midpoint in data_lane.lane_midpoints:
                self.ctx.debug_helper.draw_string(midpoint.location.to_location(),
                                                  f"{data_lane.road_id} - {data_lane.lane_id}", life_time=600000000)
