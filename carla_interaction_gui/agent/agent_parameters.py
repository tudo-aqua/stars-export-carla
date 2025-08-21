from dataclasses import dataclass, field
from typing import Dict


@dataclass
class AgentParameters:
    # probabilities in [0,1], TM compares against random sample
    percentage_running_light: float = 0.0  # 0 => always stop at red/yellow
    percentage_running_sign: float = 0.0  # 0 => stop at non-signalized junction
    percentage_ignore_vehicles: float = 0.0
    percentage_ignore_walkers: float = 0.0

    # distance to leading vehicle override (meters); if 0, dynamic
    distance_to_leading_vehicle: float = 0.0

    # lane offset (meters, + right, - left), used in motion plan
    lane_offset: float = 0.0

    # lane-change toggles (simplified placeholders)
    auto_lane_change: bool = True
    keep_slow_lane_percentage: float = 0.0
    random_left_lane_change_percentage: float = 0.0
    random_right_lane_change_percentage: float = 0.0

    # physics mode (we assume physics enabled)
    synchronous_mode: bool = True

    # per-actor overrides (ID->value)
    per_actor_running_light: Dict[int, float] = field(default_factory=dict)
    per_actor_running_sign: Dict[int, float] = field(default_factory=dict)
    per_actor_ignore_vehicles: Dict[int, float] = field(default_factory=dict)
    per_actor_ignore_walkers: Dict[int, float] = field(default_factory=dict)
    per_actor_distance_to_leading: Dict[int, float] = field(default_factory=dict)

    # ---- convenience getters ----
    def get_running_light(self, actor_id: int) -> float:
        return self.per_actor_running_light.get(actor_id, self.percentage_running_light)

    def get_running_sign(self, actor_id: int) -> float:
        return self.per_actor_running_sign.get(actor_id, self.percentage_running_sign)

    def get_ignore_vehicles(self, actor_id: int) -> float:
        return self.per_actor_ignore_vehicles.get(actor_id, self.percentage_ignore_vehicles)

    def get_ignore_walkers(self, actor_id: int) -> float:
        return self.per_actor_ignore_walkers.get(actor_id, self.percentage_ignore_walkers)

    def get_distance_to_leading(self, actor_id: int) -> float:
        return self.per_actor_distance_to_leading.get(
            actor_id,
            self.distance_to_leading_vehicle or 0.0
        )
