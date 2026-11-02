from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from carla_interaction_gui.gui.widgets import carla_target_rows, entry_row
from carla_interaction_gui.workers.GenerateMapsWorker import GenerateMapsWorker


class MapsTab(ttk.Frame):
    """The "Generate Maps" tab: precompute map data for all allowed maps."""

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="Generate Maps")
        tk.Label(self, text="Generate map data for all maps of the selected CARLA version.").pack(pady=5)

        carla_target_rows(self, app)
        entry_row(self, "Maps output folder:", app.transformer_output_path_variable,
                 lambda: app.open_directory_dialog(app.transformer_output_path_variable))

        rendering_options = ttk.LabelFrame(self, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(rendering_options, text="Render off screen",
                       variable=app.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        tk.Button(self, text="Generate all maps", command=self._start_generate_maps).pack(pady=10)
        self.stop_btn = tk.Button(self, text="Stop", command=app.stop_worker, state="disabled")
        self.stop_btn.pack(pady=8)
        app.register_stop_button(self.stop_btn)

    def _start_generate_maps(self):
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", app.carla_executable_variable, "carla"),
            ("Maps output folder", app.transformer_output_path_variable, "dir"),
        ]):
            return

        app.clear_log()
        cfg = app.collect_cfg()
        app.attach_worker(
            GenerateMapsWorker(cfg, app.log, app.available_maps()),
            stop_button=self.stop_btn
        )
