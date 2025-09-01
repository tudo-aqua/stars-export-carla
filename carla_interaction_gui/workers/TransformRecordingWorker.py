import subprocess
import sys

from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class TransformRecordingWorker(ThreadWorker):
    """
    Launches a separate Python process that runs CarlaMonitor.monitor_simulation_run
    and kills the entire process tree on cancel/finish.
    """

    def __init__(self, cfg: Config, log_cb):
        super().__init__(cfg, log_cb)
        self._proc: subprocess.Popen | None = None

    def run(self):
        exe = self.cfg.carla_executable
        runner = self._resolve_runner()
        if not runner:
            return self.log(f"!! Could not locate {self.RUNNER}")

        cmd = [
            sys.executable, runner, "transform",
            "--carla-exe", exe,
            "--input", self.cfg.transform_input_file,
            "--output", self.cfg.transformer_output_path,
        ]
        if getattr(self.cfg, "render_off_screen", False):
            cmd.append("--offscreen")
        if getattr(self.cfg, "render_quality_low", False):
            cmd.append("--quality-low")

        if getattr(self.cfg, "only_track_at_specific_interval", False):
            cmd += [
                "--only-track-at-specific-interval",
                "--specific-track-interval",
                str(getattr(self.cfg, "specific_track_interval", 0.5))
            ]

        self._start_and_stream(cmd)
        self.log(">> [Data-AV Transformer] Done.")
        return None

    def cancel(self):
        super().cancel()
        self._kill_tree()
