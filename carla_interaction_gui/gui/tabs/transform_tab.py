from __future__ import annotations

from carla_interaction_gui.gui.widgets import Tab, check, double_spin, line_edit, path_field, primary_button
from carla_interaction_gui.workers.TransformRecordingWorker import TransformRecordingWorker


class TransformTab(Tab):
    """The "Transform" tab: replay a recording and dump processed data."""

    def __init__(self, app):
        super().__init__(app, "Replay a recording, or every recording below a root folder, and dump the "
                              "processed data.")
        settings = app.settings

        form = self.section("Input / output")
        form.addRow("Recording extension:", line_edit(settings["recording_extension"], ".rec"))
        form.addRow("Input recording / root folder:",
                    path_field(app, settings["transform_input_file"], recording=True, folder=True))
        # If CARLA runs in a docker container whose mounted recordings folder
        # (see CARLA_DOCKER_RECORDINGS_* in gui/constants.py) differs from the
        # path this process sees the folder/file above at, give the
        # container-side equivalent here so the actual replay/transform calls
        # (which run server-side) use the right path while the picker above
        # still works against the real path on this machine.
        mount = line_edit(settings["transform_docker_mount_path"], "e.g. /workspace/recordings")
        mount.setToolTip("Container-side path of the input above, if CARLA runs in docker.")
        form.addRow("Docker mount folder (container-side):", mount)
        form.addRow("Output folder:", path_field(app, settings["transformer_output_path"], folder=True))

        form = self.section("Tracking interval")
        interval_enabled = check("Only track at specific interval", settings["only_track_at_specific_interval"])
        form.addRow(interval_enabled)
        interval = double_spin(settings["specific_track_interval"], 0.01, 3600, step=0.1, suffix=" s")
        interval.setEnabled(interval_enabled.isChecked())
        interval_enabled.toggled.connect(interval.setEnabled)
        form.addRow("Interval:", interval)

        self.stop_btn = self.actions(primary_button("Start transform", self._start_transform))

    def _start_transform(self):
        """
        Starts the recording transformation process by validating input parameters,
        collecting configuration data, and attaching the TransformRecordingWorker.
        """
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", "carla_executable", "carla"),
            ("Input recording", "transform_input_file", ("dir", "file")),
            ("Output folder", "transformer_output_path", "dir"),
        ]):
            return

        app.clear_log()
        app.attach_worker(TransformRecordingWorker(app.collect_cfg(), app.log), stop_button=self.stop_btn)
