#!/usr/bin/env python3
from __future__ import annotations
import json
from pathlib import Path
from dataclasses import dataclass, asdict, fields

_CFG = Path.home() / ".carla_gui_config.json"

@dataclass
class Config:
    # ─ generic ─
    carla_executable: str = ""
    recording_extension: str = ".rec"

    # ─ manual drive ─
    manual_output_dir: str = ""
    default_recordings_folder: str = ""
    new_file_name: str = ""

    # ─ downstream tasks ─
    transform_input_file: str = ""
    video_input_file: str = ""
    transformer_output_path: str = ""
    video_output_path: str = ""

    # ─ video export params ─
    video_width: int = 640
    video_height: int = 480
    vehicle_id: int = -1
    begin_at: float = 0.0
    end_at: float = float("inf")
    with_bboxes: bool = False


def load() -> Config:
    if not _CFG.exists():
        return Config()
    try:
        raw = json.loads(_CFG.read_text())
        allowed = {f.name for f in fields(Config)}
        filtered = {k: v for k, v in raw.items() if k in allowed}
        return Config(**filtered)
    except Exception:
        return Config()


def save(cfg: Config) -> None:
    _CFG.write_text(json.dumps(asdict(cfg), indent=2))
