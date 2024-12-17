from enum import Enum

class SafetyBoxStyle(Enum):
    """Style of the safety box"""
    # Standard 3D box
    BOX = 0

    # 2D X marker
    X = 1

    # Hatching
    HATCHING = 2