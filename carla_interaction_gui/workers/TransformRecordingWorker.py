import traceback

from carla_interaction_gui.carla_launcher import restart_and_connect, kill_carla
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class TransformRecordingWorker(ThreadWorker):
    """
    Handles transformation of recording data in CARLA simulation.
    """

    def run(self):
        """
        Executes the main process involving connecting to CARLA Simulator, monitoring its
        simulation run, and handling transformation tasks.
        """
        try:
            from helpers.carla_monitor import CarlaMonitor  # type: ignore
            self.log(">> [CARLA] Rebooting CARLA & connecting")
            client = restart_and_connect(self.cfg.carla_executable, log=self.log,
                                         render_off_screen=self.cfg.render_off_screen,
                                         render_quality_low=self.cfg.render_quality_low)
            if self.cancelled: return

            monitor = CarlaMonitor(carla_client=client)
            self.log(f">> [Data-AV Transformer] Transforming {self.cfg.transform_input_file}")
            monitor.monitor_simulation_run(
                file_path=self.cfg.transform_input_file,
                weather_file_path="",
                result_file_path=self.cfg.transformer_output_path
            )
        except Exception:
            self.log(traceback.format_exc())
        finally:
            kill_carla(log=self.log)
            self.log(">> [Data-AV Transformer] Done.")

    def cancel(self):
        """
        Cancels the current operation and terminates the Carla simulation environment
        associated with it.
        """
        super().cancel()
        kill_carla(log=self.log)
