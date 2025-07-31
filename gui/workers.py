#!/usr/bin/env python3
"""
Background threads for the GUI.

Re-uses restart_carla() / restart_and_connect() from carla_launcher
and respects new config keys: transformer_output_path & video_output_path
"""
from __future__ import annotations
import shutil, threading, subprocess, sys, time
from pathlib import Path
import psutil, carla  # type: ignore

from .config_data import Config
from .carla_launcher import (
    kill_carla, start_carla,
    restart_carla, restart_and_connect
)


# ────────────────────────────────────────────────────────────────────────────
class _ThreadWorker(threading.Thread):
    exclusive = True

    def __init__(self, cfg: Config, log_cb):
        super().__init__(daemon=True)
        self.cfg = cfg;
        self._log = log_cb
        self._cancel = threading.Event()

    def cancel(self): self._cancel.set()

    @property
    def cancelled(self): return self._cancel.is_set()

    def log(self, txt: str): self._log(txt)


# ────────────────────────────────────────────────────────────────────────────
class CarlaServerWorker(_ThreadWorker):
    def run(self):
        restart_carla(self.cfg.carla_executable, log=self.log)
        try:
            while not self.cancelled: time.sleep(1)
        finally:
            kill_carla(log=self.log)


# ────────────────────────────────────────────────────────────────────────────
class ManualControlWorker(_ThreadWorker):
    def run(self):
        self.log(">> Rebooting CARLA …")
        restart_carla(self.cfg.carla_executable)
        if self.cancelled: return

        mc_py = (Path(self.cfg.carla_executable).parent /
                 "PythonAPI" / "examples" / "manual_control.py")
        if not mc_py.exists():
            return self.log(f"!! manual_control.py missing @ {mc_py}")

        self.log(">> Launching manual_control.py")
        proc = subprocess.Popen([sys.executable, str(mc_py)],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, start_new_session=True)
        try:
            for line in proc.stdout:
                if self.cancelled: break
                self.log(line.rstrip())
        finally:
            try:
                psutil.Process(proc.pid).kill()
            except Exception:
                pass
            kill_carla(log=self.log)
            self.log(">> manual_control.py exited")


# ────────────────────────────────────────────────────────────────────────────
class MoveLatestRecWorker(_ThreadWorker):
    exclusive = False

    def __init__(self, cfg: Config, new_file_name: str, log_cb):
        super().__init__(cfg, log_cb)
        self._dst_name = new_file_name

    def run(self):
        ext = self.cfg.recording_extension
        src = Path(self.cfg.manual_output_dir) / f"manual_recording{ext}"
        if not src.exists():
            return self.log(f"!! '{src.name}' not found in {src.parent}")

        ts = time.strftime("%Y_%m_%d-%H_%M_%S")
        dst_dir = Path(self.cfg.default_recordings_folder) or src.parent
        dst = dst_dir / f"{self._dst_name}_{ts}{ext}"

        self.log(f">> Moving {src.name} → {dst}")
        try:
            dst_dir.mkdir(parents=True, exist_ok=True)
            shutil.move(src, dst)
            self.log(">> Move successful.")
        except Exception as e:
            self.log(f"!! {e}")


# ────────────────────────────────────────────────────────────────────────────
class TransformWorker(_ThreadWorker):
    def run(self):
        import traceback
        try:
            from helpers.carla_monitor import CarlaMonitor  # type: ignore
            self.log(">> Rebooting CARLA & connecting …")
            client = restart_and_connect(self.cfg.carla_executable, log=self.log)
            if self.cancelled: return

            monitor = CarlaMonitor(carla_client=client)
            self.log(f">> Transforming {self.cfg.transform_input_file}")
            monitor.monitor_simulation_run(
                file_path=self.cfg.transform_input_file,
                weather_file_path="",
                result_file_path=self.cfg.transformer_output_path
            )
        except Exception:
            self.log(traceback.format_exc())
        finally:
            kill_carla(log=self.log)
            self.log(">> Done.")

    def cancel(self):
        super().cancel(); kill_carla(log=self.log)


# ────────────────────────────────────────────────────────────────────────────
class RecordVideoWorker(_ThreadWorker):
    def run(self):
        from helpers.carla_camera_recorder import CarlaCameraRecorder as RawRec  # type: ignore
        from helpers.carla_camera_recorder_with_bboxes import CarlaCameraRecorder as BoxRec  # type: ignore
        RecCls = BoxRec if self.cfg.with_bboxes else RawRec

        self.log(">> Rebooting CARLA & connecting …")
        client = restart_and_connect(self.cfg.carla_executable, log=self.log)
        if self.cancelled: return

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
        self.log(f">> Recording video (bboxes={self.cfg.with_bboxes}) …")
        rec.record_camera_in_simulation_run(**args)

        self.log(">> Encoding mp4 …")
        stem = Path(self.cfg.video_input_file).stem
        rec.save_video(self.cfg.video_output_path, stem,
                       self.cfg.vehicle_id, self.cfg.begin_at, rec.END_AT, self.cfg.with_bboxes)
        self.log(">> Finished video export")
        kill_carla(log=self.log)

    def cancel(self): super().cancel(); kill_carla(log=self.log)
