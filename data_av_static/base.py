from typing import List
from carla import World, Map
from carla_data_classes import DataBlock

class _Base:
    """
    Holds shared state for MapRasterizer and provides typed attributes.
    """
    def __init__(self, carla_world: World):
        self.world: World = carla_world
        self.map: Map = carla_world.get_map()
        self.blocks: List[DataBlock] = []
        self.debug_helper = self.world.debug
        self.kd_tree = None
        self.lane_midpoints = []
