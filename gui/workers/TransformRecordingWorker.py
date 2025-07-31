from gui.carla_launcher import restart_and_connect, kill_carla
from gui.workers.ThreadWorker import ThreadWorker
import traceback


class TransformRecordingWorker(ThreadWorker):
    def run(self):
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
        super().cancel();
        kill_carla(log=self.log)
