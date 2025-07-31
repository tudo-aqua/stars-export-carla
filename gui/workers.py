from __future__ import annotations
import shutil, time, threading, subprocess, sys
from pathlib import Path
import carla
import psutil

from .config_data import Config
from .carla_launcher import kill_carla, start_carla


# --- shared helper ----------------------------------------------------------
class _ThreadWorker(threading.Thread):
    """
    Base class.
    • override `exclusive = False` if this job may run parallel
    • call self.cancel() from GUI stop-button
    """
    exclusive = True

    def __init__(self, cfg: Config, log_cb):
        super().__init__(daemon=True)
        self.cfg, self._log = cfg, log_cb
        self._cancel = threading.Event()

    def cancel(self): self._cancel.set()

    @property
    def cancelled(self): return self._cancel.is_set()

    def log(self, txt): self._log(txt)


# ---------------------------------------------------------------------------#
# Stand-alone CARLA server worker                                            #
# ---------------------------------------------------------------------------#
class CarlaServerWorker(_ThreadWorker):
    """Launches the UE4 server only; loops until .cancel() is called."""

    def run(self):
        self.log(">> Starting CARLA server …")
        kill_carla()  # be sure we’re the only one
        start_carla(self.cfg.carla_executable)
        try:
            while not self.cancelled:
                time.sleep(1)
        finally:
            self.log(">> Stopping CARLA server")
            kill_carla()


# ---------------------------------------------------------------------------#
# Manual-driving workflow (kill → start CARLA → run manual_control.py)       #
# ---------------------------------------------------------------------------#
class ManualControlWorker(_ThreadWorker):
    def run(self):
        self.log(">> Killing CARLA …")
        kill_carla();
        time.sleep(1)
        if self.cancelled: return

        self.log(">> Starting CARLA …")
        start_carla(self.cfg.carla_executable)
        time.sleep(20)  # let server boot
        if self.cancelled: return

        mc_py = Path(self.cfg.carla_executable).parent / "PythonAPI" / "examples" / "manual_control.py"
        if not mc_py.exists():
            self.log(f"!! manual_control.py missing @ {mc_py}")
            return

        self.log(">> Launching manual_control.py")
        self._proc = subprocess.Popen(
            [sys.executable, str(mc_py)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True
        )

        for line in self._proc.stdout:
            if self.cancelled: break
            self.log(line.rstrip())

        self.log(">> manual_control.py exited")

    def cancel(self):
        super().cancel()
        try:
            psutil.Process(self._proc.pid).kill()
        except Exception:
            pass
        kill_carla()


# ---------------------------------------------------------------------------#
# “Move latest .rec” quick utility (former *Transform & Move Latest*)        #
# ---------------------------------------------------------------------------#
class MoveLatestRecWorker(_ThreadWorker):
    exclusive = False
    def __init__(self, cfg, new_file_name, log_cb):
        super().__init__(cfg, log_cb)
        self._dst_name = new_file_name

    def run(self):
        recs = list(Path(self.cfg.input_path).glob("*.rec"))
        if not recs: return self.log("!! No .rec files found.")
        src = max(recs, key=lambda p: p.stat().st_mtime)
        ts = time.strftime("%Y_%m_%d_%H_%M")
        dst = Path(self.cfg.output_path) / f"{self._dst_name}_{ts}.rec"
        self.log(f">> Moving {src.name} → {dst.name}")
        try:
            shutil.move(src, dst);
            self.log(">> Move successful.")
        except Exception as e:
            self.log(f"!! {e}")


# ---------------------------------------------------------------------------#
# Full recording transformation (uses helpers.CarlaMonitor exactly as before)#
# ---------------------------------------------------------------------------#
class TransformWorker(_ThreadWorker):
    def run(self):
        import traceback
        try:
            from helpers.carla_monitor import CarlaMonitor
            self.log(">> Booting CARLA …");
            kill_carla();
            time.sleep(2)
            start_carla(self.cfg.carla_executable);
            time.sleep(20)
            if self.cancelled: return

            self.log(">> Connecting to CARLA")
            client = carla.Client("localhost", 2000);
            client.set_timeout(60)
            monitor = CarlaMonitor(carla_client=client)

            self.log(f">> Transforming {self.cfg.input_path}")
            monitor.monitor_simulation_run(
                file_path=self.cfg.input_path,
                weather_file_path="",
                result_file_path=self.cfg.output_path
            )
        except Exception:
            self.log(traceback.format_exc())
        finally:
            kill_carla();
            self.log(">> Done.")


# ---------------------------------------------------------------------------#
# Video recorder (wraps both original recorder classes)                      #
# ---------------------------------------------------------------------------#
class RecordVideoWorker(_ThreadWorker):
    def run(self):
        from helpers.carla_camera_recorder import CarlaCameraRecorder as RawRec
        from helpers.carla_camera_recorder_with_bboxes import CarlaCameraRecorder as BoxRec

        RecCls = BoxRec if self.cfg.with_bboxes else RawRec

        self.log(">> Spawning CARLA client")
        client = carla.Client('localhost', 2000);
        client.set_timeout(60)
        rec = RecCls(client)

        args = dict(
            recording_folder=self.cfg.output_path or ".",
            path=self.cfg.input_path,
            vehicle_id=self.cfg.vehicle_id,
            width=self.cfg.video_width,
            height=self.cfg.video_height,
            begin_at=self.cfg.begin_at,
            end_at=self.cfg.end_at if self.cfg.end_at != float("inf") else sys.maxsize
        )
        self.log(f">> Recording video (bboxes={self.cfg.with_bboxes}) …")
        rec.record_camera_in_simulation_run(**args)
        self.log(">> Encoding mp4 …")
        filename_wo_ext = Path(self.cfg.input_path).stem
        rec.save_video(self.cfg.output_path, filename_wo_ext,
                       self.cfg.vehicle_id, self.cfg.begin_at, rec.END_AT)
        self.log(">> Finished video export")
