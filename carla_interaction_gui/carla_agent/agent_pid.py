# agent_pid.py
# Exact translation of traffic_manager::PID::RunStep from PIDController.h.
from dataclasses import dataclass
from typing import Tuple

from .agent_constants import constants as C


@dataclass
class StateEntry:
    angular_deviation: float
    velocity_deviation: float
    steer: float = 0.0  # previous steer for slew clamp (TM stores steer in state)


def run_step(present: StateEntry,
             previous: StateEntry,
             longitudinal_parameters: Tuple[float, float, float],
             lateral_parameters: Tuple[float, float, float]):
    # Longitudinal PID (no accumulated I; uses present+previous per step)
    expr_v = (
            longitudinal_parameters[0] * present.velocity_deviation +
            longitudinal_parameters[1] * (present.velocity_deviation + previous.velocity_deviation) * C.PID.DT +
            longitudinal_parameters[2] * (present.velocity_deviation - previous.velocity_deviation) * C.PID.INV_DT
    )

    if expr_v > 0.0:
        throttle = min(expr_v, C.PID.MAX_THROTTLE);
        brake = 0.0
    else:
        throttle = 0.0;
        brake = min(abs(expr_v), C.PID.MAX_BRAKE)

    # Lateral PID
    steer = (
            lateral_parameters[0] * present.angular_deviation +
            lateral_parameters[1] * (present.angular_deviation + previous.angular_deviation) * C.PID.DT +
            lateral_parameters[2] * (present.angular_deviation - previous.angular_deviation) * C.PID.INV_DT
    )

    # Slew + clamp (exact order)
    steer = max(previous.steer - C.PID.MAX_STEERING_DIFF, min(steer, previous.steer + C.PID.MAX_STEERING_DIFF))
    steer = max(-C.PID.MAX_STEERING, min(steer, C.PID.MAX_STEERING))

    return throttle, brake, steer
