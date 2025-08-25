from __future__ import annotations

from typing import Optional

import carla

from .agent_collision import CollisionLogic
from .agent_localization import LocalizationBuffer
from .agent_motion_plan import compute_control
from .agent_parameters import AgentParameters
from .agent_pid import StateEntry, run_step as pid_run
from .agent_traffic_lights import TrafficLightLogic


class SimpleAgent:
    """
    Usage:
        carla_agent = SimpleAgent(vehicle)
        ctrl = carla_agent.run_step(dt=0.05)
        vehicle.apply_control(ctrl)

    This mirrors CARLA TM behavior at a high level while keeping the logic
    split into feature-specific modules for easy debugging & tuning.
    """

    def __init__(
            self,
            vehicle: carla.Vehicle,
            params: Optional[AgentParameters] = None,
            *,
            lane_offset: Optional[float] = None,
    ):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.params = params or AgentParameters()

        if lane_offset is not None:
            self.params.lane_offset = float(lane_offset)

        # Stages
        self._loc = LocalizationBuffer(self.world, vehicle)
        self._collision = CollisionLogic(vehicle, self.params)
        self._tl = TrafficLightLogic(vehicle, self.params)

        # PID memory (steer kept for slew-rate limiting)
        self._prev_state = StateEntry(angular_deviation=0.0, velocity_deviation=0.0, steer=0.0)

    # --- optional: update parameters at runtime --------------------------------
    def set_parameters(self, **kwargs) -> None:
        """
        Update TMParameters on the fly, e.g.:
            carla_agent.set_parameters(lane_offset=0.2, percentage_running_light=0.1)
        """
        for k, v in kwargs.items():
            if hasattr(self.params, k):
                setattr(self.params, k, v)

    # --- main control tick -----------------------------------------------------
    def run_step(self, dt: float = 0.05) -> carla.VehicleControl:
        """
        Single control step:
          1) update localization buffer & junction state
          2) evaluate traffic lights / unsignalized junction stop
          3) compute deviations & PID params via motion plan (incl. collision policy)
          4) PID to (throttle, brake, steer)
          5) emergency stop override
        """
        # 1) Localization
        self._loc.update()

        # 2) Traffic lights / junction logic
        tl_hazard = self._tl.update(self._loc)

        # 3) Motion plan (includes collision policy & target speed shaping)
        ang_dev, vel_dev, emergency_stop, long_params, lat_params = compute_control(
            self.world, self.vehicle, self.params, self._loc, self._collision, tl_hazard
        )

        # 4) PID (TM-like longitudinal & lateral with steering slew limit)
        present = StateEntry(angular_deviation=ang_dev, velocity_deviation=vel_dev, steer=0.0)
        throttle, brake, steer = pid_run(present, self._prev_state, long_params, lat_params)

        # 5) Emergency stop → full brake
        if emergency_stop:
            throttle, brake = 0.0, 1.0

        control = carla.VehicleControl(throttle=float(throttle), brake=float(brake), steer=float(steer))

        # keep previous state for next tick (TM stores steer in the state for slew-rate limiting)
        self._prev_state = StateEntry(
            angular_deviation=ang_dev,
            velocity_deviation=vel_dev,
            steer=steer,
        )
        return control
