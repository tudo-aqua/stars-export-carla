import threading

from gui.config_data import Config


class ThreadWorker(threading.Thread):
    exclusive = True

    def __init__(self, cfg: Config, log_cb):
        super().__init__(daemon=True)
        self.cfg = cfg
        self._log = log_cb
        self._cancel = threading.Event()

    def cancel(self):
        self._cancel.set()

    @property
    def cancelled(self):
        return self._cancel.is_set()

    def log(self, txt: str):
        self._log(txt)
