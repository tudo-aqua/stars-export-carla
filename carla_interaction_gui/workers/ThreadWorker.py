import threading

from carla_interaction_gui.config_data import Config


class ThreadWorker(threading.Thread):
    """
    Represents a thread worker used to execute tasks on a separate thread.
    """
    exclusive = True

    def __init__(self, cfg: Config, log_cb):
        super().__init__(daemon=True)
        self.cfg = cfg
        self._log = log_cb
        self._cancel = threading.Event()

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
