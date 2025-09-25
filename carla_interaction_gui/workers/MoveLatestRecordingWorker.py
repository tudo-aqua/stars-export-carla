import shutil
import time
from pathlib import Path

from carla_interaction_gui.config_data import Config
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class MoveLatestRecordingWorker(ThreadWorker):
    """
    Worker class for moving the latest recording file with a specific naming format.
    """
    exclusive = False

    def __init__(self, cfg: Config, new_file_name: str, log_cb):
        super().__init__(cfg, log_cb)
        self._dst_name = new_file_name

    def run(self):
        """
        Moves a manually recorded file to a specified destination directory with a timestamped name.
        """
        ext = self.cfg.recording_extension
        src = Path(self.cfg.manual_output_dir) / f"manual_recording.rec"
        if not src.exists():
            return self.log(f"!! '{src.name}' not found in {src.parent}")

        ts = time.strftime("%Y_%m_%d-%H_%M_%S")
        dst_dir = Path(self.cfg.default_recordings_folder) or src.parent
        dst = dst_dir / f"{self._dst_name}_{ts}{ext}"

        self.log(f">> Moving {src.name} → {dst}")
        try:
            dst_dir.mkdir(parents=True, exist_ok=True)
            shutil.move(src, dst)
            self.log(">> Move successful.")
        except Exception as e:
            self.log(f"!! {e}")
