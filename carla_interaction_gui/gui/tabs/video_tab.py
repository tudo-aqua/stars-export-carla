from __future__ import annotations

import math
import os

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox, QLabel, QSizePolicy, QWidget

from carla_interaction_gui.gui.widgets import (
    Tab, check, combo, double_spin, hbox, line_edit, path_field, primary_button, spin,
)
from carla_interaction_gui.workers.RecordVideoWorker import RecordVideoWorker
from helpers.camera_recorder.CameraPosition import CameraPosition
from helpers.camera_recorder.SafetyBoxStyle import SafetyBoxStyle


class VideoTab(Tab):
    """The "Record ➜ MP4" tab: export a recording directly to an mp4 file."""

    def __init__(self, app):
        super().__init__(app, "Export a recording, or every recording in a folder, to mp4 from any number of "
                              "camera angles, optionally with bounding boxes/safety boxes.")
        settings = app.settings

        form = self.section("Input / output")
        form.addRow("Recording extension:", line_edit(settings["recording_extension"], ".rec"))
        # A folder is batch mode - see _start_video, which marks a folder path
        # with a trailing slash so the runner records every recording file
        # directly inside it, mirroring the Transform tab's file-or-folder input.
        form.addRow("Input recording / folder:",
                    path_field(app, settings["video_input_file"], recording=True, folder=True))
        # Container-side equivalent of the input, if CARLA runs in docker -
        # same idea as on the Transform tab.
        mount = line_edit(settings["video_docker_mount_path"], "e.g. /workspace/recordings")
        mount.setToolTip("Container-side path of the input above, if CARLA runs in docker.")
        form.addRow("Docker mount folder (container-side):", mount)
        form.addRow("Output folder:", path_field(app, settings["video_output_path"], folder=True))

        form = self.section("Video parameters")
        form.addRow("Resolution:", hbox(spin(settings["video_width"], 16, 8192, " px"), QLabel("×"),
                                        spin(settings["video_height"], 16, 8192, " px"), QWidget(),
                                        stretch_last=True))
        form.addRow("Field of view:", spin(settings["video_fov"], 1, 179, "°"))
        vehicle = spin(settings["vehicle_id"], -1, 2_000_000_000)
        vehicle.setSpecialValueText("ego vehicle")
        form.addRow("Vehicle ID:", vehicle)
        form.addRow("Start at:", double_spin(settings["begin_at"], 0, 1e7, step=1, suffix=" s"))
        form.addRow("End at:", self._end_at_spin())

        self.body.addWidget(self._camera_angles())

        form = self.section("Safety boxes")
        safety_enabled = check("Render safety boxes", settings["video_render_safety_boxes"])
        safety_enabled.setToolTip("Extra projection in front of/beside the ego vehicle, shown on angles "
                                  "with bounding boxes enabled.")
        form.addRow(safety_enabled)
        style = combo(settings["video_safety_box_style"], [s.name for s in SafetyBoxStyle])
        style.setEnabled(safety_enabled.isChecked())
        safety_enabled.toggled.connect(style.setEnabled)
        form.addRow("Style:", style)

        self.stop_btn = self.actions(primary_button("Start recording", self._start_video))

    def _end_at_spin(self) -> QDoubleSpinBox:
        """End time; -1 (shown as "file end") is stored as infinity, i.e. record to the end."""
        value = self.app.settings["end_at"]
        end = QDoubleSpinBox()
        end.setRange(-1, 1e7)
        end.setSingleStep(1)
        end.setSuffix(" s")
        end.setSpecialValueText("file end")
        end.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        end.setMinimumWidth(140)

        def to_spin(v):
            end.setValue(-1 if math.isinf(v) or v < 0 else v)

        to_spin(value.get())
        end.valueChanged.connect(lambda v: value.set(float("inf") if v < 0 else v))
        value.changed.connect(to_spin)
        return end

    def _camera_angles(self) -> QGroupBox:
        """
        One row per CameraPosition, each choosing whether to render that angle
        at all, and if so whether to overlay metadata text and/or 3-D vehicle
        bounding boxes.
        """
        value = self.app.settings["video_camera_positions"]
        saved = {c.get("name"): c for c in value.get() or []}
        group = QGroupBox("Camera angles")
        grid = QGridLayout(group)
        grid.setColumnStretch(3, 1)
        grid.setHorizontalSpacing(24)
        center = Qt.AlignmentFlag.AlignHCenter
        for column, title in enumerate(("Angle", "Metadata", "Bounding box")):
            grid.addWidget(QLabel(f"<b>{title}</b>"), 0, column, center if column else Qt.AlignmentFlag.AlignLeft)

        self._camera_checks: dict[str, tuple[QCheckBox, QCheckBox, QCheckBox]] = {}

        def store():
            value.set([{"name": name, "metadata": metadata.isChecked(), "bbox": bbox.isChecked()}
                       for name, (selected, metadata, bbox) in self._camera_checks.items() if selected.isChecked()])

        for row, pos in enumerate(CameraPosition, start=1):
            entry = saved.get(pos.name)
            selected = QCheckBox(pos.name)
            selected.setChecked(entry is not None)
            metadata = QCheckBox()
            metadata.setChecked(bool(entry and entry.get("metadata")))
            bbox = QCheckBox()
            bbox.setChecked(bool(entry and entry.get("bbox")))
            for options in (metadata, bbox):
                options.setEnabled(selected.isChecked())
                selected.toggled.connect(options.setEnabled)
            for c in (selected, metadata, bbox):
                c.toggled.connect(store)
            grid.addWidget(selected, row, 0)
            grid.addWidget(metadata, row, 1, center)
            grid.addWidget(bbox, row, 2, center)
            self._camera_checks[pos.name] = (selected, metadata, bbox)
        return group

    def _start_video(self):
        """
        Starts the video recording process by validating input parameters, collecting
        configuration data, and attaching the RecordVideoWorker for the task.
        """
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", "carla_executable", "carla"),
            ("Input recording", "video_input_file", ("dir", "file")),
            ("Output folder", "video_output_path", "dir"),
        ]):
            return

        if not app.settings["video_camera_positions"].get():
            return app.show_error("No camera angle selected", "Select at least one camera angle to record.")

        app.clear_log()
        config = app.collect_cfg()

        # A folder path needs a trailing slash for the runner to treat it as
        # batch mode (record every recording inside it) rather than a single
        # file - add one if it's missing (e.g. typed by hand, or picked with
        # the "Folder…" button, whose dialog doesn't add one itself).
        if os.path.isdir(config.video_input_file) and not config.video_input_file.endswith("/"):
            config.video_input_file += "/"

        app.attach_worker(RecordVideoWorker(config, app.log), stop_button=self.stop_btn)
        return None
