#!/usr/bin/env python3
"""
manual_agent_control.py

Self-contained viewer that behaves like CARLA's manual_control.py, but when you press 'P'
it toggles a Python Agent (not Traffic Manager). This module dynamically loads CARLA's
original manual_control.py from the CARLA install, using the path to CarlaUE4.exe.

Public entry point for runners:
    launch_from_runner(host="127.0.0.1", port=2000, res="1280x720", sync=True, carla_exe=None)

If you run this file directly, pass --carla-exe to point at your CARLA binary.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path
from typing import Any

from carla_interaction_gui.Agent.SimpleAgent import _patch_keyboard_for_agent


def _load_manual_control_from_carla(carla_exe: str | os.PathLike) -> Any:
    """
    Import CARLA's manual_control.py by absolute path derived from the CARLA executable.
    Returns the imported module object (aliased as 'mc' elsewhere).
    """
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


def launch_from_runner(host="127.0.0.1", port=2000, res="1280x720", sync=True, carla_exe=None):
    """
    Used by carla_task_runner.py.
    Dynamically imports CARLA's manual_control from the CARLA install derived from carla_exe.
    """
    if not carla_exe:
        raise ValueError("launch_from_runner requires 'carla_exe' to locate manual_control.py")

    mc = _load_manual_control_from_carla(carla_exe)
    _patch_keyboard_for_agent(mc)

    # Build args namespace expected by manual_control.game_loop
    args = argparse.Namespace()
    args.debug = False
    args.host = host
    args.port = port
    args.autopilot = False
    args.res = res
    args.width, args.height = [int(x) for x in args.res.split('x')]
    args.sync = bool(sync)
    args.filter = 'vehicle.*'
    args.generation = '2'
    args.rolename = 'hero'
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
    _patch_keyboard_for_agent(mc)
    return mc.main()


if __name__ == "__main__":
    sys.exit(main() or 0)
