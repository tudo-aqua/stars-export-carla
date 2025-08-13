from typing import List

from carla import World, Map

from carla_data_classes.static import DataBlock
from carla_data_classes.static.DataWorld import DataWorld


class _Base:
    """
    Holds shared state for MapRasterizer and provides typed attributes.
    """
    def __init__(self, carla_world: World):
        self.world: World = carla_world
        self.map: Map = carla_world.get_map()
        self.blocks: List[DataBlock] = []
        self.data_world: DataWorld = None
        self.debug_helper = self.world.debug
        self.kd_tree = None
        self.lane_midpoints = []
