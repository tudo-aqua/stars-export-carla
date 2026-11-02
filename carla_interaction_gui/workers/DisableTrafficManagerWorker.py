from __future__ import annotations

import sys

from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class DisableTrafficManagerWorker(ThreadWorker):
    """
    Connects to the CARLA server that is already running (e.g. from a manual
    driving session) and takes every non-hero, non-manually-controlled
    vehicle off Traffic Manager control and brakes it to a stop, so the rest
    of the traffic stops moving.
    """

    exclusive = False

    def __init__(self, cfg: Config, log_cb):
        super().__init__(cfg, log_cb)

    def run(self):
        runner = self._resolve_runner()
        if not runner:
            self.log(f"!! Could not locate {self.RUNNER}")
            return

        cmd = [sys.executable or "python", runner, "disable_traffic_manager"]
        self._start_and_stream(cmd)
