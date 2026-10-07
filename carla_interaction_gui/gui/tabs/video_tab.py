from __future__ import annotations

import os
import tkinter as tk
from tkinter import ttk, messagebox

from carla_interaction_gui.gui.widgets import entry_row, validate_number
from carla_interaction_gui.workers.RecordVideoWorker import RecordVideoWorker
from helpers.camera_recorder.CameraPosition import CameraPosition
from helpers.camera_recorder.SafetyBoxStyle import SafetyBoxStyle


class VideoTab(ttk.Frame):
    """The "Record ➜ MP4" tab: export a recording directly to an mp4 file."""

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="Record ➜ MP4")
        tk.Label(self, text="Let's you export a recording to mp4, from any number of camera "
                            "angles, optionally with bounding boxes/safety boxes.").pack(pady=5)

        entry_row(self, "Recording extension:", app.recording_extension_variable)

        # Input: allow either a single file or a folder (batch mode - see
        # _start_video, which marks a folder path with a trailing slash so
        # the runner knows to record every recording file directly inside
        # it, mirroring the Transform tab's file-or-folder input).
        row = tk.Frame(self)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Input recording / folder:", width=26, anchor="w").pack(side="left")
        tk.Entry(row, textvariable=app.video_input_path_variable, width=45).pack(
            side="left", fill="x", expand=True)
        tk.Button(
            row, text="File...",
            command=lambda: app.open_file_selection_with_specified_extension(app.video_input_path_variable)
        ).pack(side="left", padx=2)
        tk.Button(
            row, text="Folder...",
            command=lambda: app.open_directory_dialog(app.video_input_path_variable)
        ).pack(side="left", padx=2)
        # If CARLA runs in a docker container (see carla_run.sh) whose
        # mounted recordings folder differs from the path this process sees
        # the file above at, give the container-side equivalent here so the
        # replay (which runs server-side) uses the right path - same idea as
        # on the Transform tab.
        entry_row(self, "Docker mount folder (container-side):", app.video_docker_mount_path_variable)
        entry_row(self, "Output folder:", app.video_output_path_variable,
                 lambda: app.open_directory_dialog(app.video_output_path_variable))

        video_parameters = ttk.LabelFrame(self, text="Video parameters")
        video_parameters.pack(fill="x", padx=4, pady=6)
        entry_row(video_parameters, "Width:", app.video_width_variable, width=8)
        entry_row(video_parameters, "Height:", app.video_height_variable, width=8)
        entry_row(video_parameters, "FOV:", app.video_fov_variable, width=8)
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

        # ── Camera angles: one row per CameraPosition, each choosing whether
        # to render that angle at all, and if so whether to overlay metadata
        # text and/or 3-D vehicle bounding boxes. ──────────────────────────
        camera_frame = ttk.LabelFrame(self, text="Camera angles")
        camera_frame.pack(fill="x", padx=4, pady=6)

        header = tk.Frame(camera_frame)
        header.pack(fill="x", padx=6, pady=(4, 0))
        tk.Label(header, text="Angle", width=22, anchor="w").pack(side="left")
        tk.Label(header, text="Metadata", width=10, anchor="w").pack(side="left")
        tk.Label(header, text="Bounding box", width=12, anchor="w").pack(side="left")

        for pos in CameraPosition:
            vars_for_pos = app.video_camera_position_vars[pos.name]
            row = tk.Frame(camera_frame)
            row.pack(fill="x", padx=6, pady=1)
            tk.Checkbutton(row, text=pos.name, variable=vars_for_pos["selected"], width=20,
                          anchor="w").pack(side="left")
            tk.Checkbutton(row, variable=vars_for_pos["metadata"], width=8).pack(side="left")
            tk.Checkbutton(row, variable=vars_for_pos["bbox"], width=10).pack(side="left")

        # ── Safety boxes: an optional extra projection (in front of/beside
        # the ego vehicle) shown only on angles with bounding boxes enabled. ─
        safety_frame = ttk.LabelFrame(self, text="Safety boxes")
        safety_frame.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(safety_frame, text="Render safety boxes",
                       variable=app.video_render_safety_boxes_variable,
                       command=self._update_safety_style_state).pack(anchor="w", padx=6, pady=2)

        style_row = tk.Frame(safety_frame)
        style_row.pack(fill="x", padx=6, pady=2)
        tk.Label(style_row, text="Style:", width=26, anchor="w").pack(side="left")
        self.safety_style_combo = ttk.Combobox(
            style_row,
            textvariable=app.video_safety_box_style_variable,
            state="readonly",
            values=[s.name for s in SafetyBoxStyle],
            width=15
        )
        self.safety_style_combo.pack(side="left")
        self._update_safety_style_state()

        rendering_options = ttk.LabelFrame(self, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(
            rendering_options, text="Render off screen", variable=app.render_off_screen_variable
        ).pack(anchor="w", padx=4, pady=4)
        tk.Checkbutton(
            rendering_options, text="Render quality low", variable=app.render_quality_low_variable
        ).pack(anchor="w", padx=4, pady=4)

        tk.Button(self, text="Start recording", command=self._start_video).pack(pady=10)
        self.stop_btn = tk.Button(self, text="Stop", command=app.stop_worker, state="disabled")
        self.stop_btn.pack(pady=8)
        app.register_stop_button(self.stop_btn)

    def _update_safety_style_state(self):
        state = "readonly" if self.app.video_render_safety_boxes_variable.get() else "disabled"
        self.safety_style_combo.config(state=state)

    def _start_video(self):
        """
        Starts the video recording process by validating input parameters, collecting
        configuration data, and attaching the RecordVideoWorker for the task.
        """
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", app.carla_executable_variable, "file"),
            ("Input recording", app.video_input_path_variable, ("dir", "file")),
            ("Output folder", app.video_output_path_variable, "dir"),
        ]):
            return

        if not any(v["selected"].get() for v in app.video_camera_position_vars.values()):
            return messagebox.showerror("No camera angle selected",
                                        "Select at least one camera angle to record.")

        app.clear_log()
        config = app.collect_cfg()

        # A folder path needs a trailing slash for the runner to treat it as
        # batch mode (record every recording inside it) rather than a single
        # file - add one if it's missing (e.g. typed by hand, or picked with
        # the "Folder..." button, whose dialog doesn't add one itself).
        video_input = app.video_input_path_variable.get().strip()
        if os.path.isdir(video_input) and not video_input.endswith(("/", "\\")):
            video_input += "/"
        config.video_input_file = video_input
        config.video_output_path = app.video_output_path_variable.get().strip()
        config.video_width = app.video_width_variable.get()
        config.video_height = app.video_height_variable.get()
        config.video_fov = app.video_fov_variable.get()
        config.vehicle_id = app.vehicle_id_variable.get()

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
