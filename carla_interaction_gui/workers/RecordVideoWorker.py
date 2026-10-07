import json
import sys

from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class RecordVideoWorker(ThreadWorker):
    """
    Launches an isolated process that performs the replay + multi-camera
    render to mp4, then kills the entire process tree on cancel/finish.
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
            "--fov", str(getattr(self.cfg, "video_fov", 105)),
            "--vehicle-id", str(self.cfg.vehicle_id),
            "--begin-at", str(max(0.0, float(self.cfg.begin_at)) if self.cfg.begin_at is not None else 0.0),
            "--end-at", end_arg,
            "--camera-positions", json.dumps(getattr(self.cfg, "video_camera_positions", None) or []),
        ]
        if getattr(self.cfg, "video_render_safety_boxes", False):
            cmd.append("--render-safety-boxes")
            cmd += ["--safety-box-style", getattr(self.cfg, "video_safety_box_style", "HATCHING")]
        docker_mount_path = getattr(self.cfg, "video_docker_mount_path", "") or ""
        if docker_mount_path:
            cmd += ["--docker-mount-path", docker_mount_path]
        if getattr(self.cfg, "render_quality_low", False):
            cmd.append("--quality-low")
        if getattr(self.cfg, "render_off_screen", False):
            cmd.append("--offscreen")
        # Deliberately no --map-name: the map to load comes from the
        # recording itself (see CarlaCameraRecorder's _load_world_for_replay,
        # matching how the Transform tab's monitor_simulation_run works).
        # Forcing a --map-name here would make restart_and_connect() load a
        # map during boot, only for _load_world_for_replay to immediately
        # load a(nother) map again from the recording - a redundant, racy
        # double map-load right after a fresh CARLA boot.

        self._start_and_stream(cmd)
        self.log(">> [Recorder] Finished video export")
        return None
