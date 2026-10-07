# workers/GenerateMapsWorker.py
import sys

from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class GenerateMapsWorker(ThreadWorker):
    """
    Starts a separate Python process that generates map files for the allowed maps,
    then kills the entire process tree on cancel/finish.
    """

    def __init__(self, cfg: Config, log_cb, allowed_maps: list[str]):
        super().__init__(cfg, log_cb)
        self._allowed_maps = allowed_maps

    def run(self):
        runner = self._resolve_runner()
        if not runner:
            return self.log(f"!! Could not locate {self.RUNNER}")

        cmd = [
            sys.executable, runner, "gen_maps",
            "--carla-exe", self.cfg.carla_executable,
            "--output", self.cfg.transformer_output_path,
        ]
        # pass rendering options / session params
        if getattr(self.cfg, "render_off_screen", False):
            cmd.append("--offscreen")
        if getattr(self.cfg, "render_quality_low", False):
            cmd.append("--quality-low")

        # limit to provided allowed maps
        for m in self._allowed_maps:
            cmd += ["--map", m]

        self._start_and_stream(cmd)
        self.log(">> [GenerateMaps] All maps processed.")
