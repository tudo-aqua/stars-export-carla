# gui/ConfigData.py
#!/usr/bin/env python3
import json
from pathlib import Path
from dataclasses import dataclass

CONFIG_PATH = Path.home() / ".carla_gui_config.json"

@dataclass
class ConfigData:
    # From your original app:
    carla_executable:       str
    recording_extension:    str
    input_path:             str
    output_path:            str

    # New fields for manual control GUI:
    default_recordings_folder: str
    new_file_name:             str

def load_saved_config() -> ConfigData | None:
    """
    Loads all six fields from CONFIG_PATH, or returns None if file missing/invalid.
    """
    if not CONFIG_PATH.exists():
        return None
    try:
        raw = json.loads(CONFIG_PATH.read_text())
        return ConfigData(
            carla_executable         = raw.get("carla_executable", ""),
            recording_extension      = raw.get("recording_extension", ""),
            input_path               = raw.get("input_path", ""),
            output_path              = raw.get("output_path", ""),
            default_recordings_folder= raw.get("default_recordings_folder", ""),
            new_file_name            = raw.get("new_file_name", "")
        )
    except Exception:
        return None

def save_config_to_disk(cfg: ConfigData):
    """
    Persists all six fields to CONFIG_PATH as JSON.
    """
    CONFIG_PATH.write_text(json.dumps({
        "carla_executable":          cfg.carla_executable,
        "recording_extension":       cfg.recording_extension,
        "input_path":                cfg.input_path,
        "output_path":               cfg.output_path,
        "default_recordings_folder": cfg.default_recordings_folder,
        "new_file_name":             cfg.new_file_name
    }, indent=2))
