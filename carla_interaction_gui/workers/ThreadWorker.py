import os
import subprocess
import sys
import threading
from pathlib import Path

import psutil

from carla_interaction_gui.carla_launcher import restart_carla
from carla_interaction_gui.carla_versions import python_api_env
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
        Cancels the current operation and force-kills the child process tree, if any,
        so a blocked read on its stdout (which only checks `cancelled` between lines)
        can't stall the stop.
        """
        self._cancel.set()
        self._kill_tree()

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

    def _restart_carla(self) -> bool:
        """
        restart_carla() for workers running in the GUI process: the map is
        loaded from a runner subprocess afterwards, since only subprocesses
        may import the CARLA Python API (see _start_and_stream). Returns
        False (after logging why) if the server didn't come up.
        """
        try:
            restart_carla(
                self.cfg.carla_executable,
                log=self.log,
                render_quality_low=self.cfg.render_quality_low,
                render_off_screen=self.cfg.render_off_screen,
                python_api=self.cfg.carla_python_api,
            )
        except Exception as e:
            self.log(f"!! [CARLA] Server did not start: {e}")
            return False
        map_name = self.cfg.selected_map
        if map_name and not self.cancelled:
            runner = self._resolve_runner()
            if not runner:
                self.log(f"!! Could not locate {self.RUNNER}")
                return False
            self._start_and_stream([sys.executable, runner, "set_map", "--map", map_name])
        return True

    def _start_and_stream(self, cmd: list[str], *, cwd: str | None = None):
        # The child imports the CARLA Python API selected in the GUI, matching
        # the server version; it is copied out of its docker image on first use.
        try:
            env = python_api_env(self.cfg.carla_python_api, self.log)
        except Exception as e:
            self.log(f"!! [CARLA] Could not provide CARLA Python API '{self.cfg.carla_python_api}': {e}")
            return

        # Python block-buffers stdout by default when it isn't a real
        # terminal (as here, piped for streaming into the GUI log) - so
        # print() output can sit in the child's buffer, unflushed, for a
        # while. A hard crash (e.g. a native segfault) bypasses normal
        # cleanup entirely and never flushes it, silently losing whatever
        # was printed most recently - making the last *visible* line look
        # like where the crash happened when it may actually be much
        # further along. Force the child to flush every line immediately
        # so the log always reflects its true, most recent state.
        env["PYTHONUNBUFFERED"] = "1"

        self.log(">> [Runner] " + " ".join(cmd))
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=cwd,
            env=env,
            preexec_fn=os.setsid,
        )
        try:
            assert self._proc.stdout is not None
            for line in self._proc.stdout:
                if self.cancelled:
                    break
                clean = line.rstrip()
                self.log(clean)
            if not self.cancelled:
                try:
                    rc = self._proc.wait(timeout=10)
                except Exception:
                    rc = None
                if rc not in (0, None):
                    self.log(f"!! [Runner] Process exited abnormally with code {rc} "
                             f"- the task did NOT finish successfully.")
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
