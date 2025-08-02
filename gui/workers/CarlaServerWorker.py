import time

from gui.carla_launcher import restart_carla, kill_carla
from gui.workers.ThreadWorker import ThreadWorker


class CarlaServerWorker(ThreadWorker):
    """
    Manages a server worker for running the CARLA simulation.
    """
    def run(self):
        """
        Manages the execution of a CARLA simulation runtime environment, allowing for
        restarting and terminating the CARLA executable.
        """
        restart_carla(exe = self.cfg.carla_executable, log=self.log, render_off_screen=self.cfg.render_off_screen, render_quality_low=self.cfg.render_quality_low)
        try:
            while not self.cancelled: time.sleep(1)
        finally:
            kill_carla(log=self.log)
