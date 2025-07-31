import subprocess
import sys
from pathlib import Path

import psutil

from gui.carla_launcher import restart_carla, kill_carla
from gui.workers.ThreadWorker import ThreadWorker


class ManualControlWorker(ThreadWorker):
    """
    Represents a worker responsible for manually driving in the CARLA simulator.
    """

    def run(self):
        """
        Runs the process to reboot the CARLA simulator and launch the `manual_control.py` script.
        """
        self.log(">> Rebooting CARLA …")
        restart_carla(self.cfg.carla_executable)
        if self.cancelled:
            return None

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
                if self.cancelled:
                    break
                self.log(line.rstrip())
        finally:
            try:
                psutil.Process(proc.pid).kill()
            except Exception:
                pass
            kill_carla(log=self.log)
            self.log(">> manual_control.py exited")
