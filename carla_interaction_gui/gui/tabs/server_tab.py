from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from carla_interaction_gui.gui.constants import ALLOWED_CARLA_MAPS
from carla_interaction_gui.gui.widgets import entry_row
from carla_interaction_gui.workers.CarlaServerWorker import CarlaServerWorker


class ServerTab(ttk.Frame):
    """The "CARLA Server" tab: start or stop a head-less CARLA instance."""

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="CARLA Server")
        tk.Label(self, text="Start or stop a head-less CARLA instance.").pack(pady=5)

        entry_row(self, "CARLA executable:", app.carla_executable_variable,
                 lambda: app.open_file_dialog(app.carla_executable_variable))

        row = tk.Frame(self)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Map:", width=26, anchor="w").pack(side="left")
        ttk.Combobox(
            row,
            textvariable=app.selected_map_variable,
            state="readonly",
            values=ALLOWED_CARLA_MAPS,
            width=42
        ).pack(side="left", fill="x", expand=True)

        rendering_options = ttk.LabelFrame(self, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(rendering_options, text="Render off screen",
                       variable=app.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(rendering_options, text="Render quality low",
                       variable=app.render_quality_low_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        self.server_btn = tk.Button(self, text="Start CARLA server", width=25, command=self._toggle_carla)
        self.server_btn.pack(pady=10)

        self.stop_btn = tk.Button(self, text="Stop", command=app.stop_worker, state="disabled")
        self.stop_btn.pack(pady=2)
        app.register_stop_button(self.stop_btn)

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
            if not app.validate_paths([("CARLA executable", app.carla_executable_variable, "file")]):
                return
            app.clear_log()
            app.carla_worker = CarlaServerWorker(app.collect_cfg(), app.log)
            app.carla_worker.start()
            self.server_btn.config(text="Stop CARLA server")
            self.stop_btn.config(state="normal")

    def reset_server_button(self):
        self.server_btn.config(text="Start CARLA server")
        self.stop_btn.config(state="disabled")
