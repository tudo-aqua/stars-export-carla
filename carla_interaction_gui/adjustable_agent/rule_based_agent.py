"""
Modular Rule-based CARLA Agent

- sensing/plan/act are split into separate modules in rule_agent/
- This file wires them together.
"""
from __future__ import annotations

import random
from typing import Optional, Dict, Tuple

import carla

from .acting import Acting
from .config import AgentConfig
from .planning import Planning
from .sensing import Sensing
from .types import SensedState, Plan


class RuleBasedAgent:
    def __init__(self, ego: carla.Vehicle, client: carla.Client, config: Optional[AgentConfig] = None):
        self.ego = ego
        self.client = client
        self.world = client.get_world()
        self.map = self.world.get_map()
        self.cfg = config or AgentConfig()

        # Random for route choices
        self.rng = random.Random(self.cfg.seed)

        # Controller accumulators / memory
        self.throttle_i = 0.0
        self.last_speed_err = 0.0
        self.last_control = carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0)

        # Misc memory
        self.base_light_state = carla.VehicleLightState.NONE
        self.was_stopped = True
        self.launch_t = 0.0

        # Ensure physics
        physics_control = self.ego.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        self.ego.apply_physics_control(physics_control)

        # Subsystems
        self.sensing = Sensing(self)
        self.planning = Planning(self)
        self.acting = Acting(self)

    def run_step(self) -> Tuple[carla.VehicleControl, Dict]:
        state: SensedState = self.sensing.sense()
        plan: Plan = self.planning.plan(state)
        control: carla.VehicleControl = self.acting.act(state, plan)

        debug = {
            "speed_mps": state.speed_mps,
            "speed_limit_mps": state.speed_limit_mps,
            "target_speed_mps": plan.target_speed_mps,
            "in_junction": state.in_junction,
            "next_options": len(state.next_options),
            "tl_state": str(state.traffic_light_state) if state.traffic_light_state else None,
            "stop_or_yield": state.stop_or_yield_ahead,
            "blink_left": plan.blink_left,
            "blink_right": plan.blink_right,
            "headlights_on": plan.headlights_on,
            "steer": control.steer,
            "throttle": control.throttle,
            "brake": control.brake,
        }
        return control, debug


def drive_ego_hero(client: carla.Client, ego: carla.Vehicle, seed: Optional[int] = 0) -> RuleBasedAgent:
    cfg = AgentConfig(seed=seed)
    return RuleBasedAgent(ego, client, cfg)
