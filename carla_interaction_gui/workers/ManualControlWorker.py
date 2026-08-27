from __future__ import annotations

import sys
from pathlib import Path

from carla_interaction_gui.carla_launcher import restart_carla, kill_carla
from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker

SCRIPT_PATH = Path(__file__).resolve().parents[2] / "manual_control_steering_wheel" / "manual_control_keyboard.py"

class ManualControlWorker(ThreadWorker):
    """
    Represents a worker responsible for manually driving in the CARLA simulator.
    Can be used to launch multiple manual_control_keyboard.py instances concurrently.
    """

    def __init__(
        self,
        cfg: Config,
        log_cb,
        *,
        vehicle_filter: str | None = None,
        role_name: str | None = None,
        restart_before: bool = True,
        kill_server_after: bool = True,
        exclusive: bool = True,
    ):
        super().__init__(cfg, log_cb)
        self.vehicle_filter = vehicle_filter
        self.role_name = role_name
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
            self.log(f"!! manual_control_keyboard.py missing @ {SCRIPT_PATH}")
            return

        cmd = [sys.executable, str(SCRIPT_PATH)]
        if self.vehicle_filter:
            cmd += ["--filter", self.vehicle_filter]
        if self.role_name:
            cmd += ["--rolename", self.role_name]
        try:
            # wheel_config.ini is loaded via a path relative to the working directory,
            # so this must run with the script's own folder as cwd.
            self._start_and_stream(cmd, cwd=str(SCRIPT_PATH.parent))
        finally:
            if self.kill_server_after:
                kill_carla()
            self.log(">> [CARLA] manual_control_keyboard.py is shut down")

    def cancel(self):
        """Called by the GUI when the user presses *Stop* or on app close."""
        super().cancel()  # also force-kills the process tree, see ThreadWorker.cancel
        if self.kill_server_after:
            kill_carla()
