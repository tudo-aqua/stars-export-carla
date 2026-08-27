from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from carla_interaction_gui.gui.widgets import entry_row, validate_number
from carla_interaction_gui.workers.RecordVideoWorker import RecordVideoWorker


class VideoTab(ttk.Frame):
    """The "Record ➜ MP4" tab: export a recording directly to an mp4 file."""

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="Record ➜ MP4")
        tk.Label(self, text="Export a recording directly to mp4.").pack(pady=5)

        entry_row(self, "Recording extension:", app.recording_extension_variable)
        entry_row(self, "Input recording:", app.video_input_path_variable,
                 lambda: app.open_file_selection_with_specified_extension(app.video_input_path_variable))
        entry_row(self, "Output folder:", app.video_output_path_variable,
                 lambda: app.open_directory_dialog(app.video_output_path_variable))

        video_parameters = ttk.LabelFrame(self, text="Video parameters")
        video_parameters.pack(fill="x", padx=4, pady=6)
        entry_row(video_parameters, "Width:", app.video_width_variable, width=8)
        entry_row(video_parameters, "Height:", app.video_height_variable, width=8)
        entry_row(video_parameters, "Vehicle ID (-1 = ego):", app.vehicle_id_variable, width=8)

        vcmd = (self.register(validate_number), "%P")

        row = tk.Frame(video_parameters)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Start at (s):", width=26, anchor="w").pack(side="left")
        tk.Entry(row, textvariable=app.begin_at_variable, width=8,
                 validate="key", validatecommand=vcmd).pack(side="left", fill="x", expand=True)

        row = tk.Frame(video_parameters)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="End at (s, -1 = file end):", width=26, anchor="w").pack(side="left")
        tk.Entry(row, textvariable=app.end_at_variable, width=8,
                 validate="key", validatecommand=vcmd).pack(side="left", fill="x", expand=True)

        tk.Checkbutton(video_parameters, text="Draw 3-D bounding boxes",
                       variable=app.with_bboxes_variable).pack(anchor="w", padx=4, pady=4)

        rendering_options = ttk.LabelFrame(self, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(
            rendering_options, text="Render quality low", variable=app.render_quality_low_variable
        ).pack(anchor="w", padx=4, pady=4)

        tk.Button(self, text="Start recording", command=self._start_video).pack(pady=10)
        self.stop_btn = tk.Button(self, text="Stop", command=app.stop_worker, state="disabled")
        self.stop_btn.pack(pady=8)
        app.register_stop_button(self.stop_btn)

    def _start_video(self):
        """
        Starts the video recording process by validating input parameters, collecting
        configuration data, and attaching the RecordVideoWorker for the task.
        """
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", app.carla_executable_variable, "file"),
            ("Input recording", app.video_input_path_variable, "file"),
            ("Output folder", app.video_output_path_variable, "dir"),
        ]):
            return

        app.clear_log()
        config = app.collect_cfg()
        config.video_input_file = app.video_input_path_variable.get().strip()
        config.video_output_path = app.video_output_path_variable.get().strip()
        config.video_width = app.video_width_variable.get()
        config.video_height = app.video_height_variable.get()
        config.vehicle_id = app.vehicle_id_variable.get()
        config.with_bboxes = app.with_bboxes_variable.get()

        def _to_float(s: str, default: float) -> float:
            s = (s or "").strip()
            if not s:
                return default
            return float(s)

        config.begin_at = max(0.0, _to_float(app.begin_at_variable.get(), 0.0))

        end_str = (app.end_at_variable.get() or "").strip()
        if not end_str:
            config.end_at = float("inf")
        else:
            end_val = float(end_str)
            config.end_at = float("inf") if end_val < 0 else end_val

        app.attach_worker(RecordVideoWorker(config, app.log), stop_button=self.stop_btn)
