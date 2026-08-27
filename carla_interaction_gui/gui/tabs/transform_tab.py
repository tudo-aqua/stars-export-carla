from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from carla_interaction_gui.gui.widgets import entry_row, validate_number
from carla_interaction_gui.workers.TransformRecordingWorker import TransformRecordingWorker


class TransformTab(ttk.Frame):
    """The "Transform" tab: replay a recording and dump processed data."""

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="Transform")
        tk.Label(self, text="Replay a recording and dump processed data.").pack(pady=5)

        entry_row(self, "Recording extension:", app.recording_extension_variable)

        # Input: allow either a single file or a root folder
        row = tk.Frame(self)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Input recording / root folder:", width=26, anchor="w").pack(side="left")
        tk.Entry(row, textvariable=app.transform_input_file_variable, width=45).pack(
            side="left", fill="x", expand=True)
        tk.Button(
            row, text="File...",
            command=lambda: app.open_file_selection_with_specified_extension(app.transform_input_file_variable)
        ).pack(side="left", padx=2)
        tk.Button(
            row, text="Folder...",
            command=lambda: app.open_directory_dialog(app.transform_input_file_variable)
        ).pack(side="left", padx=2)

        entry_row(self, "Output folder:", app.transformer_output_path_variable,
                 lambda: app.open_directory_dialog(app.transformer_output_path_variable))

        rendering_options = ttk.LabelFrame(self, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(
            rendering_options, text="Render off screen",
            variable=app.render_off_screen_variable, anchor="w",
        ).pack(fill="x", padx=6, pady=2)

        tracking_frame = ttk.LabelFrame(self, text="Tracking interval")
        tracking_frame.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(
            tracking_frame,
            text="Only track at specific interval",
            variable=app.only_track_at_specific_interval_variable,
            anchor="w",
            command=self._update_interval_entry_state
        ).pack(fill="x", padx=6, pady=2)

        row = tk.Frame(tracking_frame)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Interval (s):", width=26, anchor="w").pack(side="left")
        vcmd = (self.register(validate_number), "%P")
        self.interval_entry = tk.Entry(
            row,
            textvariable=app.specific_track_interval_variable,
            width=10,
            validate="key",
            validatecommand=vcmd
        )
        self.interval_entry.pack(side="left", fill="x", expand=False)
        self._update_interval_entry_state()

        tk.Button(self, text="Start transform", command=self._start_transform).pack(pady=10)
        self.stop_btn = tk.Button(self, text="Stop", command=app.stop_worker, state="disabled")
        self.stop_btn.pack(pady=8)
        app.register_stop_button(self.stop_btn)

    def _update_interval_entry_state(self):
        if self.app.only_track_at_specific_interval_variable.get():
            self.interval_entry.config(state="normal")
        else:
            self.interval_entry.config(state="disabled")

    def _start_transform(self):
        """
        Starts the recording transformation process by validating input parameters,
        collecting configuration data, and attaching the TransformRecordingWorker.
        """
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", app.carla_executable_variable, "file"),
            ("Input recording", app.transform_input_file_variable, ("dir", "file")),
            ("Output folder", app.transformer_output_path_variable, "dir"),
        ]):
            return

        app.clear_log()
        config = app.collect_cfg()
        config.transform_input_file = app.transform_input_file_variable.get().strip()
        config.transformer_output_path = app.transformer_output_path_variable.get().strip()
        app.attach_worker(TransformRecordingWorker(config, app.log), stop_button=self.stop_btn)
