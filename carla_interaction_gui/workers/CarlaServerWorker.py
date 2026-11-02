import time

from carla_interaction_gui.carla_launcher import kill_carla
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class CarlaServerWorker(ThreadWorker):
    """
    Manages a server worker for running the CARLA simulation.
    """

    def run(self):
        """
        Manages the execution of a CARLA simulation runtime environment, allowing for
        restarting and terminating the CARLA executable.
        """
        try:
            if self._restart_carla():
                while not self.cancelled: time.sleep(1)
        finally:
            kill_carla(log=self.log)
