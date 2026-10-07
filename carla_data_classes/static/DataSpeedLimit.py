from dataclasses import dataclass


@dataclass
class DataSpeedLimit:
    """
    DataClass to wrap a speed limit section for a lane
    """
    speed_limit: float
    from_distance: float
    to_distance: float
