from __future__ import annotations

import sys
import threading
from pathlib import Path

from carla_interaction_gui.carla_launcher import kill_carla
from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker

SCRIPT_PATH = Path(__file__).resolve().parents[2] / "manual_control_steering_wheel" / "manual_control_steeringwheel.py"


class SteeringWheelControlWorker(ThreadWorker):
    """
    Launches manual_control_steeringwheel.py, a pygame viewer that behaves like
    CARLA's own manual_control_keyboard.py but reads input from a steering wheel/joystick
    instead of the keyboard. It always connects to 127.0.0.1:2000 and spawns its
    own 'hero' vehicle, so this is a primary/exclusive drive method like the main
    "Start manual driving" button, not an additional controlled actor.
    """

    def __init__(
        self,
        cfg: Config,
        log_cb,
        *,
        vehicle_filter: str | None = None,
        role_name: str | None = None,
        camera_elevation: float | None = None,
        camera_tilt: float | None = None,
        restart_before: bool = True,
        kill_server_after: bool = True,
        exclusive: bool = True,
    ):
        super().__init__(cfg, log_cb)
        self.vehicle_filter = vehicle_filter
        self.role_name = role_name
        self.camera_elevation = camera_elevation
        self.camera_tilt = camera_tilt
        self.restart_before = restart_before
        self.kill_server_after = kill_server_after
        self.exclusive = exclusive
        # Set once the driving client is launched into the (re)started server;
        # the GUI waits for it before spawning traffic.
        self.client_started = threading.Event()

    def run(self):
        if self.restart_before:
            self.log(">> [CARLA] Rebooting CARLA")
            if not self._restart_carla():
                if self.kill_server_after:
                    kill_carla(log=self.log)
                return
            if self.cancelled:
                return

        if not SCRIPT_PATH.exists():
            self.log(f"!! manual_control_steeringwheel.py missing @ {SCRIPT_PATH}")
            return

        cmd = [sys.executable, str(SCRIPT_PATH)]
        if self.vehicle_filter:
            cmd += ["--filter", self.vehicle_filter]
        if self.role_name:
            cmd += ["--rolename", self.role_name]
        if self.camera_elevation is not None:
            cmd += ["--camera-elevation", str(self.camera_elevation)]
        if self.camera_tilt is not None:
            cmd += ["--camera-tilt", str(self.camera_tilt)]
        self.client_started.set()
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
