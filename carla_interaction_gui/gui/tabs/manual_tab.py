from __future__ import annotations

import tkinter as tk
from tkinter import ttk, messagebox

from carla_interaction_gui.gui.constants import ALLOWED_CARLA_MAPS
from carla_interaction_gui.gui.widgets import entry_row
from carla_interaction_gui.workers.ManualControlWorker import ManualControlWorker
from carla_interaction_gui.workers.MoveLatestRecordingWorker import MoveLatestRecordingWorker


class ManualTab(ttk.Frame):
    """
    The "Manual Drive" tab: drive around in CARLA manually, spawn extra
    manually-controlled actors, and archive the resulting recording.
    """

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="Manual Drive")
        tk.Label(self, text="Let's you manually drive around in CARLA.").pack(pady=5)

        entry_row(self, "CARLA executable:", app.carla_executable_variable,
                 lambda: app.open_file_dialog(app.carla_executable_variable))
        entry_row(self, "Recording extension:", app.recording_extension_variable)
        entry_row(self, "CARLA output folder:", app.manual_output_dir_variable,
                 lambda: app.open_directory_dialog(app.manual_output_dir_variable))
        entry_row(self, "Archive recordings folder:", app.default_recordings_folder_variable,
                 lambda: app.open_directory_dialog(app.default_recordings_folder_variable))
        entry_row(self, "New file-name prefix:", app.new_file_name_variable)

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

        tk.Button(self, text="Start manual driving", width=25, command=self._start_manual).pack(pady=8)

        row2 = tk.Frame(self)
        row2.pack(fill="x", pady=4)
        tk.Label(row2, text="Add controlled actor:", width=26, anchor="w").pack(side="left")
        tk.Button(row2, text="Cyclist",
                  command=lambda: self._spawn_manual_extra(filter_str="vehicle.bh.crossbike")
                  ).pack(side="left", padx=2)
        tk.Button(row2, text="Walker",
                  command=lambda: self._spawn_manual_extra(filter_str="walker.pedestrian.*")
                  ).pack(side="left", padx=2)
        tk.Button(row2, text="Small car",
                  command=lambda: self._spawn_manual_extra(filter_str="vehicle.mini.cooper_s_2021")
                  ).pack(side="left", padx=2)
        tk.Button(row2, text="Truck",
                  command=lambda: self._spawn_manual_extra(filter_str="vehicle.carlamotors.carlacola")
                  ).pack(side="left", padx=2)

        tk.Button(self, text="Move 'manual_recording'", command=self._move_latest, state="active").pack(pady=2)

        self.stop_btn = tk.Button(self, text="Stop", command=app.stop_worker, state="disabled")
        self.stop_btn.pack(pady=8)
        app.register_stop_button(self.stop_btn)

    def _start_manual(self):
        """Start the primary manual driving (exclusive) as Lincoln MKZ 2020."""
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", app.carla_executable_variable, "file"),
            ("CARLA output folder", app.manual_output_dir_variable, "dir"),
        ]):
            return
        app.clear_log()

        w = ManualControlWorker(
            app.collect_cfg(),
            app.log,
            vehicle_filter="vehicle.lincoln.mkz_2020",
            role_name=None,
            restart_before=True,
            kill_server_after=True,
            exclusive=True,
        )
        app.manual_workers.append(w)
        app.attach_worker(w, stop_button=self.stop_btn)

    def _spawn_manual_extra(self, *, filter_str: str):
        """
        Launch another manual_control.py instance with --rolename=manual_control and
        the provided --filter, without rebooting/killing the CARLA server.
        """
        app = self.app
        if not app.validate_paths([("CARLA executable", app.carla_executable_variable, "file")]):
            return

        w = ManualControlWorker(
            app.collect_cfg(),
            app.log,
            vehicle_filter=filter_str,
            role_name="manual_control",
            restart_before=False,
            kill_server_after=False,
            exclusive=False,
        )
        app.manual_workers.append(w)
        app.attach_worker(w)

    def _move_latest(self):
        """Moves the latest recording with the specified prefix."""
        app = self.app
        if not app.new_file_name_variable.get().strip():
            return messagebox.showerror("Missing", "File-name prefix required.")
        if not app.validate_paths([
            ("Archive recordings folder", app.default_recordings_folder_variable, "dir"),
        ]):
            return
        app.attach_worker(MoveLatestRecordingWorker(app.collect_cfg(), app.new_file_name_variable.get(), app.log))
