from __future__ import annotations

import sys

from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class SpawnTrafficWorker(ThreadWorker):
    """
    Connects to the CARLA server that is already running (e.g. from a manual
    driving session) and spawns a batch of autopilot-driven vehicles into it,
    without restarting or killing the server. No walkers are spawned.
    """

    exclusive = False

    def __init__(
        self,
        cfg: Config,
        log_cb,
        *,
        number_of_vehicles: int = 30,
        filter_vehicles: str | None = None,
    ):
        super().__init__(cfg, log_cb)
        self.number_of_vehicles = number_of_vehicles
        self.filter_vehicles = filter_vehicles

    def run(self):
        runner = self._resolve_runner()
        if not runner:
            self.log(f"!! Could not locate {self.RUNNER}")
            return

        cmd = [
            sys.executable or "python",
            runner,
            "spawn_traffic",
            "--number-of-vehicles", str(self.number_of_vehicles),
            "--number-of-walkers", "0",
        ]
        if self.filter_vehicles:
            cmd += ["--filterv", self.filter_vehicles]
        self._start_and_stream(cmd)
