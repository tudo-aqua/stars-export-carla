from __future__ import annotations

from PySide6.QtWidgets import QLabel

from carla_interaction_gui.gui.widgets import Tab, path_field, primary_button
from carla_interaction_gui.workers.GenerateMapsWorker import GenerateMapsWorker


class MapsTab(Tab):
    """The "Generate Maps" tab: precompute map data for all allowed maps."""

    def __init__(self, app):
        super().__init__(app, "Generate map data for all maps of the selected CARLA version.")

        form = self.section("Output")
        form.addRow("Maps output folder:", path_field(app, app.settings["transformer_output_path"], folder=True))
        maps_label = QLabel()
        maps_label.setWordWrap(True)
        app.register_version_listener(lambda: maps_label.setText(", ".join(app.available_maps())))
        form.addRow("Maps:", maps_label)

        self.stop_btn = self.actions(primary_button("Generate all maps", self._start_generate_maps))

    def _start_generate_maps(self):
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", "carla_executable", "carla"),
            ("Maps output folder", "transformer_output_path", "dir"),
        ]):
            return

        app.clear_log()
        cfg = app.collect_cfg()
        app.attach_worker(GenerateMapsWorker(cfg, app.log, app.available_maps()), stop_button=self.stop_btn)
