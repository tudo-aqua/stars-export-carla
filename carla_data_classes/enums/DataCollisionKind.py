from enum import Enum


class DataCollisionKind(str, Enum):
    """
    Kind codes from CARLA recorder's 'Types' column.
    The recorder uses single letters; we normalize to readable enums.
    """
    VEHICLE = "VEHICLE"  # 'v'
    WALKER = "WALKER"  # 'w'
    TRAFFIC = "TRAFFIC"  # 't' (traffic light/sign/prop)
    STATIC = "STATIC"  # 's' (if present in your build)
    OTHER = "OTHER"  # 'o' / anything else
    UNKNOWN = "UNKNOWN"

    @staticmethod
    def from_token(tok: str) -> "DataCollisionKind":
        if not tok:
            return DataCollisionKind.UNKNOWN
        t = tok.strip().lower()
        if t == "v":
            return DataCollisionKind.VEHICLE
        if t == "w":
            return DataCollisionKind.WALKER
        if t == "t":
            return DataCollisionKind.TRAFFIC
        if t == "s":
            return DataCollisionKind.STATIC
        if t == "o":
            return DataCollisionKind.OTHER
        return DataCollisionKind.UNKNOWN
