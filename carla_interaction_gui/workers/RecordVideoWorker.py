import sys
from pathlib import Path

from carla_interaction_gui.carla_launcher import restart_and_connect, kill_carla
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker
from helpers.carla_camera_recorder import CarlaCameraRecorder as RawRec
from helpers.carla_camera_recorder_with_bboxes import CarlaCameraRecorder as BoxRec


class RecordVideoWorker(ThreadWorker):
    """
    Handles video recording tasks using CARLA simulation and exports the video.
    """

    def run(self):
        """
        Executes the video recording process with optional bounding box annotations, connects to the CARLA simulator, and
        handles the exporting of the final encoded video.
        """
        RecCls = BoxRec if self.cfg.with_bboxes else RawRec

        self.log(">> [CARLA] Rebooting CARLA & connecting")
        client = restart_and_connect(self.cfg.carla_executable, log=self.log)
        if self.cancelled:
            return

        rec = RecCls(client)
        args = dict(
            recording_folder=self.cfg.video_output_path or ".",
            path=self.cfg.video_input_file,
            vehicle_id=self.cfg.vehicle_id,
            width=self.cfg.video_width,
            height=self.cfg.video_height,
            begin_at=self.cfg.begin_at,
            end_at=self.cfg.end_at if self.cfg.end_at != float("inf") else sys.maxsize
        )
        self.log(f">> [Recorder] Recording video (bboxes={self.cfg.with_bboxes})")
        rec.record_camera_in_simulation_run(**args)

        self.log(">> [Recorder] Encoding mp4")
        stem = Path(self.cfg.video_input_file).stem
        if self.cfg.with_bboxes:
            rec.save_video(self.cfg.video_output_path, stem, self.cfg.vehicle_id, self.cfg.begin_at, rec.END_AT,
                           self.cfg.with_bboxes)
        else:
            rec.save_video(self.cfg.video_output_path, stem, self.cfg.vehicle_id, self.cfg.begin_at, rec.END_AT)

        self.log(">> [Recorder] Finished video export")
        kill_carla(log=self.log)

    def cancel(self):
        """
        Cancels the operation or process and ensures the appropriate shutdown logic for Carla software.
        """
        super().cancel()
        kill_carla(log=self.log)
