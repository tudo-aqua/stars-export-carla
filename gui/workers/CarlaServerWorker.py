import time

from gui.carla_launcher import restart_carla, kill_carla
from gui.workers.ThreadWorker import ThreadWorker


class CarlaServerWorker(ThreadWorker):
    def run(self):
        restart_carla(self.cfg.carla_executable, log=self.log)
        try:
            while not self.cancelled: time.sleep(1)
        finally:
            kill_carla(log=self.log)
