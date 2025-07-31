from __future__ import annotations
import json
from pathlib import Path
from dataclasses import dataclass, asdict, fields

_CONFIG_PATH = Path.home() / ".carla_gui_config.json"


@dataclass
class Config:
    carla_executable: str = ""
    recording_extension: str = ".rec"

    manual_output_dir: str = ""
    default_recordings_folder: str = ""
    new_file_name: str = ""

    transform_input_file: str = ""
    video_input_file: str = ""
    transformer_output_path: str = ""
    video_output_path: str = ""

    video_width: int = 640
    video_height: int = 480
    vehicle_id: int = -1
    begin_at: float = 0.0
    end_at: float = float("inf")
    with_bboxes: bool = False


def load() -> Config:
    """
    Loads configuration from a file if it exists, otherwise provides a default
    configuration. The function ensures only predefined fields are accepted
    from the loaded file content. Any unexpected data or errors during the
    loading process result in returning a default configuration.

    Returns:
        Config: The loaded Config object or a default object if no file
        exists or an error occurs during loading.
    """
    if not _CONFIG_PATH.exists():
        return Config()
    try:
        raw = json.loads(_CONFIG_PATH.read_text())
        allowed_fields = {field.name for field in fields(Config)}
        filtered = {field: value for field, value in raw.items() if field in allowed_fields}
        return Config(**filtered)
    except Exception:
        return Config()


def save(cfg: Config) -> None:
    """
    Saves the configuration to a file in JSON format.

    This function serializes the given configuration object into
    JSON format and writes it to the predefined configuration path.
    The output is formatted with an indentation level of 2 for
    readability. It overwrites the existing content of the file
    if it already exists.

    Parameters:
        cfg (Config): The configuration object to be saved.

    Returns:
        None
    """
    _CONFIG_PATH.write_text(json.dumps(asdict(cfg), indent=2))
