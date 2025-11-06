from __future__ import annotations

import json
from dataclasses import dataclass, asdict, fields
from pathlib import Path

_CONFIG_PATH = Path.home() / ".carla_gui_config.json"


@dataclass
class Config:
    carla_executable: str = ""
    render_off_screen: bool = False
    render_quality_low: bool = False
    recording_extension: str = ".rec"
    selected_map: str = ""

    manual_output_dir: str = ""
    default_recordings_folder: str = ""
    new_file_name: str = ""

    transform_input_file: str = ""
    transformer_output_path: str = ""
    only_track_at_specific_interval: bool = False
    specific_track_interval: float = 0.5

    video_input_file: str = ""
    video_output_path: str = ""
    video_width: int = 640
    video_height: int = 480
    vehicle_id: int = -1
    begin_at: float = 0.0
    end_at: float = float("inf")
    with_bboxes: bool = False

    agent_vehicle_filter: str = "vehicle.tesla.model3"
    agent_target_speed_kph: float = 35.0

    recgen_seed_start: int = 0
    recgen_num_scenarios: int = 1
    recgen_selected_maps: list[str] | None = None  # persisted as JSON list
    recgen_num_vehicles: int = 200
    recgen_num_walkers: int = 30
    recgen_filter_vehicles: str = "vehicle.*"
    recgen_generation_vehicles: str = "All"  # "1" | "2" | "All"
    recgen_filter_walkers: str = "walker.pedestrian.*"
    recgen_generation_walkers: str = "2"  # "1" | "2" | "All"
    recgen_length_minutes: float = 5.0
    recgen_output_dir: str = ""
    recgen_num_parked: int = 0


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
    try:
        raw = json.loads(_CONFIG_PATH.read_text())
        allowed_fields = {field.name for field in fields(Config)}
        filtered = {field: value for field, value in raw.items() if field in allowed_fields}
        cfg = Config(**filtered)
        return cfg
    except Exception:
        # On any error, return defaults
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
