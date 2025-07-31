#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path
from dataclasses import dataclass, asdict

_CFG = Path.home() / ".carla_gui_config.json"


@dataclass
class Config:
    # --- generic ---
    carla_executable: str = ""
    input_path: str = ""  # .rec / .log or a folder
    output_path: str = ""  # generic destination
    recording_extension: str = ".rec"

    # --- manual-driving assistant ---
    default_recordings_folder: str = ""
    new_file_name: str = ""  # prefix for “move latest …”

    # --- video recorder ---
    video_width: int = 640
    video_height: int = 480
    vehicle_id: int = -1
    begin_at: float = 0.0
    end_at: float = float("inf")
    with_bboxes: bool = False  # choose which recorder class


def load() -> Config:
    if not _CFG.exists():
        return Config()
    try:
        return Config(**json.loads(_CFG.read_text()))
    except Exception:
        return Config()


def save(cfg: Config) -> None:
    _CFG.write_text(json.dumps(asdict(cfg), indent=2))
