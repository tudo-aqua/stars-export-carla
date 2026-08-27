from __future__ import annotations

import sys
from pathlib import Path

from carla_interaction_gui.carla_launcher import restart_carla, kill_carla
from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker

SCRIPT_PATH = Path(__file__).resolve().parents[2] / "manual_control_steering_wheel" / "manual_control_steeringwheel.py"


class SteeringWheelControlWorker(ThreadWorker):
    """
    Launches manual_control_steeringwheel.py, a pygame viewer that behaves like
    CARLA's own manual_control.py but reads input from a steering wheel/joystick
    instead of the keyboard. The script takes no CLI arguments of its own (it
    always connects to 127.0.0.1:2000 and spawns its own 'hero' vehicle), so this
    is a primary/exclusive drive method like the main "Start manual driving"
    button, not an additional controlled actor.
    """

    def __init__(
        self,
        cfg: Config,
        log_cb,
        *,
        restart_before: bool = True,
        kill_server_after: bool = True,
        exclusive: bool = True,
    ):
        super().__init__(cfg, log_cb)
        self.restart_before = restart_before
        self.kill_server_after = kill_server_after
        self.exclusive = exclusive

    def run(self):
        if self.restart_before:
            self.log(">> [CARLA] Rebooting CARLA")
            restart_carla(
                self.cfg.carla_executable,
                log=self.log,
                render_quality_low=self.cfg.render_quality_low,
                render_off_screen=self.cfg.render_off_screen,
                map_name=self.cfg.selected_map,
            )
            if self.cancelled:
                return

        if not SCRIPT_PATH.exists():
            self.log(f"!! manual_control_steeringwheel.py missing @ {SCRIPT_PATH}")
            return

        cmd = [sys.executable, str(SCRIPT_PATH)]
        try:
            # wheel_config.ini is loaded via a path relative to the working directory,
            # so this must run with the script's own folder as cwd.
            self._start_and_stream(cmd, cwd=str(SCRIPT_PATH.parent))
        finally:
            if self.kill_server_after:
                kill_carla()
            self.log(">> [CARLA] manual_control_steeringwheel.py is shut down")

    def cancel(self):
        """Called by the GUI when the user presses *Stop* or on app close."""
        super().cancel()  # also force-kills the process tree, see ThreadWorker.cancel
        if self.kill_server_after:
            kill_carla()
