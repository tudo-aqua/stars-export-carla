from __future__ import annotations

import os, sys, time, subprocess, signal
from pathlib import Path

import psutil

from gui.carla_launcher import restart_carla, kill_carla
from gui.workers.ThreadWorker import ThreadWorker
from gui.config_data import Config


class ManualControlWorker(ThreadWorker):
    """
    Represents a worker responsible for manually driving in the CARLA simulator.
    """

    def __init__(self, cfg: Config, log_cb):
        super().__init__(cfg, log_cb)
        self._proc: subprocess.Popen | None = None

    def run(self):
        self.log(">> Rebooting CARLA …")
        restart_carla(self.cfg.carla_executable, log=self.log)
        if self.cancelled:  # stop pressed during boot wait
            return

        # locate script
        mc_py = (Path(self.cfg.carla_executable).parent /
                 "PythonAPI" / "examples" / "manual_control.py")
        if not mc_py.exists():
            self.log(f"!! manual_control.py missing @ {mc_py}")
            return

        # launch in separate process group so we can later kill the group
        self.log(">> Launching manual_control.py")
        creation: dict[str, int | None] = {}
        if sys.platform.startswith("win"):
            creation["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            creation["preexec_fn"] = os.setsid

        self._proc = subprocess.Popen(
            [sys.executable, str(mc_py)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            **creation
        )

        # stream stdout to GUI log until we’re canceled or script ends
        try:
            for line in self._proc.stdout:
                if self.cancelled:
                    break
                self.log(line.rstrip())
        finally:
            self._terminate_manual_control()
            kill_carla()
            self.log(">> manual_control.py & CARLA shut down")

    def cancel(self):
        """Called by the GUI when the user presses *Stop*."""
        super().cancel()
        self._terminate_manual_control()
        kill_carla()

    def _terminate_manual_control(self):
        """
        Force-kill *manual_control.py* **and every child process**.

        Uses psutil so it works the same on Windows, macOS, and Linux.
        """
        if not self._proc:
            return

        try:
            parent = psutil.Process(self._proc.pid)
        except psutil.NoSuchProcess:
            self._proc = None
            return

        # Gather full tree: parent + recursive children
        procs = [parent] + parent.children(recursive=True)

        # Hard-kill everything
        for p in procs:
            try:
                p.kill()          # unconditional SIGKILL / TerminateProcess
            except psutil.NoSuchProcess:
                pass

        # Wait a moment; if anything survives, kill again
        gone, alive = psutil.wait_procs(procs, timeout=3)
        for p in alive:
            try:
                p.kill()
            except Exception:
                pass

        self._proc = None
