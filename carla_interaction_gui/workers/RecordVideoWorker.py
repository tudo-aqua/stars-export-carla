import sys

from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class RecordVideoWorker(ThreadWorker):
    """
    Launches an isolated process that performs the replay + camera render to mp4,
    then kills the entire process tree on cancel/finish.
    """

    def run(self):
        runner = self._resolve_runner()
        if not runner:
            return self.log(f"!! Could not locate {self.RUNNER}")

        # derive end_at: float('inf') in GUI => file end in runner (pass negative)
        end_at = self.cfg.end_at
        if end_at == float("inf"):
            end_arg = "-1"
        else:
            end_arg = str(end_at)

        cmd = [
            sys.executable, runner, "record_video",
            "--carla-exe", self.cfg.carla_executable,
            "--input", self.cfg.video_input_file,
            "--output", self.cfg.video_output_path,
            "--width", str(self.cfg.video_width),
            "--height", str(self.cfg.video_height),
            "--vehicle-id", str(self.cfg.vehicle_id),
            "--begin-at", str(max(0.0, float(self.cfg.begin_at)) if self.cfg.begin_at is not None else 0.0),
            "--end-at", end_arg,
        ]
        if self.cfg.with_bboxes:
            cmd.append("--with-bboxes")
        if getattr(self.cfg, "render_quality_low", False):
            cmd.append("--quality-low")
        if getattr(self.cfg, "render_off_screen", False):
            cmd.append("--offscreen")
        if getattr(self.cfg, "selected_map", ""):
            cmd += ["--map-name", self.cfg.selected_map]

        self._start_and_stream(cmd)
        self.log(">> [Recorder] Finished video export")
        return None
