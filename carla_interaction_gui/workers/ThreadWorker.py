import os
import subprocess
import sys
import threading
from pathlib import Path

import psutil

from carla_interaction_gui.config_data import Config


class ThreadWorker(threading.Thread):
    """
    Represents a thread worker used to execute tasks on a separate thread.
    """
    exclusive = True
    RUNNER = "carla_task_runner.py"

    def __init__(self, cfg: Config, log_cb):
        super().__init__(daemon=True)
        self.cfg = cfg
        self._log = log_cb
        self._cancel = threading.Event()
        self._proc: subprocess.Popen | None = None

    def cancel(self):
        """
        Cancels the current operation by setting the internal cancellation flag.
        """
        self._cancel.set()

    @property
    def cancelled(self):
        """
        Gets the cancellation status.
        """
        return self._cancel.is_set()

    def log(self, txt: str):
        """
        Logs a given text message.

        Parameters:
            txt (str): The text message that needs to be logged.
        """
        self._log(txt)

    def _resolve_runner(self) -> str | None:
        here = Path(__file__).resolve().parent
        candidates = [
            here / self.RUNNER,
            Path(os.getcwd()) / self.RUNNER,
        ]
        for c in candidates:
            if c.exists():
                return str(c)
        return None

    def _start_and_stream(self, cmd: list[str]):
        creation = {}
        if sys.platform.startswith("win"):
            creation["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            creation["preexec_fn"] = os.setsid

        self.log(">> [Runner] " + " ".join(cmd))
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            **creation
        )
        try:
            assert self._proc.stdout is not None
            for line in self._proc.stdout:
                if self.cancelled:
                    break
                clean = line.rstrip()
                self.log(clean)
        finally:
            self._kill_tree()

    def _kill_tree(self):
        if not self._proc:
            return
        try:
            parent = psutil.Process(self._proc.pid)
        except psutil.NoSuchProcess:
            self._proc = None
            return
        procs = [parent] + parent.children(recursive=True)
        for p in procs:
            try:
                p.kill()
            except psutil.NoSuchProcess:
                pass
        psutil.wait_procs(procs, timeout=5)
        self._proc = None

    def send_command(self, text: str):
        """
        Send a single-line command to the child runner. No-op if process ended.
        """
        try:
            if self._proc and self._proc.stdin:
                self._proc.stdin.write(text.strip() + "\n")
                self._proc.stdin.flush()
        except Exception:
            # don't crash UI if sending fails
            pass
