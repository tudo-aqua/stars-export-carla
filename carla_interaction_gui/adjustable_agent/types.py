from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import carla


@dataclass
class SensedState:
    wp: carla.Waypoint
    speed_mps: float
    speed_limit_mps: float
    in_junction: bool
    next_options: List[carla.Waypoint]
    traffic_light_state: Optional[carla.TrafficLightState]
    at_traffic_light: bool
    stop_or_yield_ahead: Optional[Tuple[str, float]]  # ("STOP"/"YIELD", distance)
    sun_altitude_angle: float
    lead_vehicle: Optional[carla.Actor]
    lead_distance: float = math.inf
    lead_rel_speed: float = 0.0
    curvature: float = 0.0  # local curvature estimate (1/m)


@dataclass
class Plan:
    target_speed_mps: float
    target_wp: carla.Waypoint
    blink_left: bool = False
    blink_right: bool = False
    headlights_on: bool = False
    stop_now: bool = False  # used to force immediate stop (red light at line, etc.)
    stop_distance: float = 0.0  # meters to stop line
