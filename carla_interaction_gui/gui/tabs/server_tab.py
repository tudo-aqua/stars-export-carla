from __future__ import annotations

from carla_interaction_gui.gui.binding import set_combo_items
from carla_interaction_gui.gui.widgets import Tab, combo, primary_button
from carla_interaction_gui.workers.CarlaServerWorker import CarlaServerWorker


class ServerTab(Tab):
    """The "CARLA Server" tab: start or stop a head-less CARLA instance."""

    def __init__(self, app):
        super().__init__(app, "Start or stop a CARLA server with the CARLA settings above.")
        settings = app.settings

        form = self.section("Server")
        map_combo = combo(settings["selected_map"])
        app.register_version_listener(
            lambda: set_combo_items(map_combo, app.available_maps(), settings["selected_map"]))
        form.addRow("Map:", map_combo)

        self.server_btn = primary_button("Start CARLA server", self._toggle_carla)
        self.stop_btn = self.actions(self.server_btn)

    def _toggle_carla(self):
        """
        Toggles the CARLA server process between starting and stopping states. If the
        CARLA server is running, it cancels the server process. If the server is not
        running, it validates the configuration and starts a new CARLA server process.
        """
        app = self.app
        if app.carla_worker and app.carla_worker.is_alive():
            app.carla_worker.cancel()
            self.reset_server_button()
        else:
            if not app.validate_paths([("CARLA executable", "carla_executable", "carla")]):
                return
            app.clear_log()
            app.start_carla_worker(CarlaServerWorker(app.collect_cfg(), app.log))
            self.server_btn.setText("Stop CARLA server")
            self.stop_btn.setEnabled(True)

    def reset_server_button(self):
        self.server_btn.setText("Start CARLA server")
        self.stop_btn.setEnabled(False)
