from __future__ import annotations

from dataclasses import dataclass

from dataclass_wizard import JSONWizard

from carla_data_classes.enums.DataCollisionKind import DataCollisionKind


@dataclass
class DataCollision(JSONWizard):
    """
    One collision event
    """
    actor1_kind: DataCollisionKind
    actor2_kind: DataCollisionKind
    actor1_id: int
    actor1_type_id: str  # e.g., 'vehicle.lincoln.mkz_2020'
    actor2_id: int
    actor2_type_id: str  # e.g., 'traffic.traffic_light'
