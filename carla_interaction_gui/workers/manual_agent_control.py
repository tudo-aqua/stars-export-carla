#!/usr/bin/env python3
"""
manual_agent_control.py

Run CARLA's original PythonAPI 'manual_control.py' verbatim, but replace the
built-in autopilot with your Python SimpleAgent when autopilot is toggled ON
(key 'P' in the viewer). Everything else in manual_control behaves the same.

Public entry (used by carla_task_runner):
    launch_from_runner(host="127.0.0.1", port=2000, res="1280x720", sync=True, carla_exe=None)
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path
from typing import Any


# --------------------- dynamic loader for CARLA manual_control ----------------
def _load_manual_control_from_carla(carla_exe: str | os.PathLike) -> Any:
    exe = Path(carla_exe).resolve()
    mc_path = exe.parent / "PythonAPI" / "examples" / "manual_control.py"
    if not mc_path.exists():
        raise FileNotFoundError(f"manual_control.py not found next to CARLA install: {mc_path}")
    spec = importlib.util.spec_from_file_location("carla_manual_control", str(mc_path))
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not build spec for manual_control.py at {mc_path}")
    mc = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mc
    spec.loader.exec_module(mc)  # type: ignore[attr-defined]
    return mc


# ------------------------------ SimpleAgent import ----------------------------
def _import_simple_agent():
    """
    Try several stable import paths so this file works inside your package
    or when run as a standalone module.
    """
    try:
        # Preferred: project package path
        from carla_interaction_gui.agent.agent import SimpleAgent  # type: ignore
        return SimpleAgent
    except Exception:
        pass
    try:
        # Same folder
        from SimpleAgent import SimpleAgent  # type: ignore
        return SimpleAgent
    except Exception:
        pass
    raise ImportError(
        "Could not import SimpleAgent. Ensure 'SimpleAgent.py' is available and importable "
        "(e.g., carla_interaction_gui/workers/SimpleAgent.py)."
    )


# -------------------------- patch manual_control behavior ---------------------
def _install_agent_patch(mc):
    """
    Patch KeyboardControl.parse_events so that when '_autopilot_enabled' is ON,
    we disable TM autopilot and instead run SimpleAgent each tick. This keeps
    the rest of manual_control.py untouched.
    """
    SimpleAgent = _import_simple_agent()
    orig_parse = mc.KeyboardControl.parse_events

    def parse_events_with_agent(self, client, world, clock, sync_mode):
        # Let the original handler process keys, HUD updates, recording toggles, etc.
        ret = orig_parse(self, client, world, clock, sync_mode)

        # When the viewer's autopilot is ON, run our Python agent instead of TM
        try:
            import carla  # available after CARLA egg is loaded by manual_control
            if isinstance(world.player, carla.Vehicle) and getattr(self, "_autopilot_enabled", False):
                # Make sure Traffic Manager isn't touching the car
                world.player.set_autopilot(False)

                # (Re)bind agent if the ego changed or agent not present
                agent = getattr(self, "_agent", None)
                if not agent or getattr(agent, "vehicle", None) is None or agent.vehicle.id != world.player.id:
                    # Optional: read some knobs from environment if you like
                    lane_offset = float(os.getenv("TM_LANE_OFFSET", "0.0"))
                    agent = SimpleAgent(world.player, params=None if lane_offset == 0.0 else None)
                    # If your SimpleAgent accepts lane_offset directly:
                    try:
                        agent.set_parameters(lane_offset=lane_offset)  # no-op if method not present
                    except Exception:
                        pass
                    self._agent = agent

                # Compute dt: use fixed_delta_seconds in sync mode, otherwise clock time
                dt = 0.0
                try:
                    settings = world.world.get_settings() if hasattr(world, "world") else None
                    if settings and settings.fixed_delta_seconds:
                        dt = settings.fixed_delta_seconds
                except Exception:
                    pass
                if not dt or dt <= 0:
                    dt = max(clock.get_time() / 1000.0, 1.0 / 60.0)

                control = self._agent.run_step(dt=dt)
                world.player.apply_control(control)
        except Exception as e:
            # Soft-fail: show in HUD, keep the viewer alive
            try:
                world.hud.notification(f"Agent error: {e}")
            except Exception:
                pass

        return ret

    mc.KeyboardControl.parse_events = parse_events_with_agent


# --------------------------------- public entry --------------------------------
def launch_from_runner(host="127.0.0.1", port=2000, res="1280x720", sync=True, carla_exe=None):
    """
    Used by carla_task_runner.py. Loads CARLA's manual_control.py and installs the agent patch.
    """
    if not carla_exe:
        raise ValueError("launch_from_runner requires 'carla_exe' to locate manual_control.py")

    mc = _load_manual_control_from_carla(carla_exe)
    _install_agent_patch(mc)

    # Build args namespace expected by manual_control.game_loop
    args = argparse.Namespace()
    args.debug = False
    args.host = host
    args.port = port
    args.autopilot = False
    args.res = res
    args.width, args.height = [int(x) for x in args.res.split("x")]
    args.sync = bool(sync)
    args.filter = os.getenv("AGENT_VEHICLE_FILTER", "vehicle.*")
    args.generation = "2"
    args.rolename = "hero"
    args.gamma = 2.2

    return mc.game_loop(args)


def main():
    parser = argparse.ArgumentParser("manual_agent_control (dynamic)")
    parser.add_argument("--carla-exe", required=True, help="Path to CARLA executable (CarlaUE4.exe)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--res", default="1280x720")
    parser.add_argument("--sync", action="store_true")
    args = parser.parse_args()

    mc = _load_manual_control_from_carla(args.carla_exe)
    _install_agent_patch(mc)
    return mc.main()


if __name__ == "__main__":
    sys.exit(main() or 0)
