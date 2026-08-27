from __future__ import annotations

import sys
from pathlib import Path

from carla_interaction_gui.carla_launcher import restart_carla, kill_carla
from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class ManualControlWorker(ThreadWorker):
    """
    Represents a worker responsible for manually driving in the CARLA simulator.
    Can be used to launch multiple manual_control.py instances concurrently.
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
        exclusive: bool = False,
    ):
        super().__init__(cfg, log_cb)

        # params for this instance
        self.vehicle_filter = vehicle_filter          # e.g. "vehicle.lincoln.mkz_2020"
        self.role_name = role_name                    # e.g. "manual_control"
        self.restart_before = restart_before          # reboot CARLA before launching?
        self.kill_server_after = kill_server_after    # kill CARLA server after exit?
        self.exclusive = exclusive                    # let GUI gate this instance

    def run(self):
        # optional reboot (only for the primary/manual one)
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

        # locate script
        mc_py = (Path(self.cfg.carla_executable).parent /
                 "PythonAPI" / "examples" / "manual_control.py")
        if not mc_py.exists():
            self.log(f"!! manual_control.py missing @ {mc_py}")
            return

        # build command
        cmd = [sys.executable, str(mc_py)]
        if self.role_name:
            cmd += ["--rolename", self.role_name]
        if self.vehicle_filter:
            cmd += ["--filter", self.vehicle_filter]

        cmd += ["--sync"]

        try:
            self._start_and_stream(cmd)
        finally:
            if self.kill_server_after:
                kill_carla()
            self.log(">> [CARLA] manual_control.py is shut down")

    def cancel(self):
        """Called by the GUI when the user presses *Stop* or on app close."""
        super().cancel()  # also force-kills the process tree, see ThreadWorker.cancel
        if self.kill_server_after:
            kill_carla()
