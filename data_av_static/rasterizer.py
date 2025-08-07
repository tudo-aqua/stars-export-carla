from .base import _Base
from .debug_utils import _DebugUtils
from .geometry_utils import _GeometryUtils
from .io_ops import _IOOps
from .lane_utils import _LaneUtils
from .map_builder import _BlockBuilder
from .speed_limit_utils import _SpeedLimitUtils
from .traffic_light_utils import _TrafficLightUtils


class MapRasterizer(_Base, _BlockBuilder, _DebugUtils, _GeometryUtils, _IOOps, _LaneUtils, _SpeedLimitUtils, _TrafficLightUtils):
    """
    Thin facade that combines all mixins.
    Construction and attributes live in _Base via _IOOps.
    """

    def __init__(self, carla_world):
        _Base.__init__(self, carla_world)
        self.io = _IOOps(self)

    def __getattr__(self, name: str):
        # forward missing attributes/methods to services
        for comp in (self.io,):
            attr = getattr(comp, name, None)
            if attr is not None:
                return attr
        raise AttributeError(name)
