from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class AgentConfig:
    # Control loop
    dt: float = 0.05  # seconds; sync world recommended

    # Speed control (PID-ish)
    v_kp: float = 0.8
    v_ki: float = 0.05
    v_kd: float = 0.1

    # Lateral control (pure-pursuit-like with heading + cross-track terms)
    lookahead_min: float = 4.0  # m
    lookahead_max: float = 14.0  # m
    lookahead_speed_gain: float = 0.4  # m per (m/s)
    lat_k_heading: float = 1.8
    lat_k_cte: float = 0.2

    # Comfort limits
    max_throttle: float = 0.6
    max_brake: float = 0.8
    max_throttle_rate: float = 0.12  # per step
    max_brake_rate: float = 0.15  # per step
    max_steer_rate: float = 0.10  # per step
    max_steer: float = 0.9  # absolute

    # Desired headway & emergency TTC thresholds
    time_headway: float = 1.5  # seconds
    min_gap: float = 2.0  # meters
    emergency_ttc: float = 2.0  # seconds

    # Red/yellow light behavior
    stop_buffer: float = 1.3  # meters before the line/landmark

    # Stop/Yield landmark lookahead distance
    landmark_lookahead: float = 35.0  # meters

    # Treat YIELD as STOP (per your requirement)
    treat_yield_as_stop: bool = True

    # Lights
    sun_angle_headlights_deg: float = 5.0  # turn on if sun altitude below this

    # Curve handling
    a_lat_comf: float = 1.8  # m/s^2 comfortable lateral acceleration

    # Launch smoothing after full stop
    launch_ramp_sec: float = 1.5  # seconds to ramp throttle after standstill

    # Random seed for route choices (makes tests deterministic if set)
    seed: Optional[int] = None
