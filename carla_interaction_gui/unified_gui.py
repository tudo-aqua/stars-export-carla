from __future__ import annotations

import os
import sys
import tkinter as tk
from datetime import datetime
from tkinter import ttk, filedialog, messagebox, scrolledtext
from typing import Callable

from carla_interaction_gui.carla_launcher import kill_carla
from carla_interaction_gui.config_data import Config, load, save
from carla_interaction_gui.workers.CarlaServerWorker import CarlaServerWorker
from carla_interaction_gui.workers.ManualControlWorker import ManualControlWorker
from carla_interaction_gui.workers.MoveLatestRecordingWorker import MoveLatestRecordingWorker
from carla_interaction_gui.workers.RecordVideoWorker import RecordVideoWorker
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker
from carla_interaction_gui.workers.TransformRecordingWorker import TransformRecordingWorker

# Official CARLA maps from the docs (non-layered + layered "_Opt")
ALLOWED_NON_LAYERED_MAPS = [
    "Town01",
    "Town02",
    "Town03",
    # "Town04",
    "Town05",
    "Town10HD_Opt",
]
ALLOWED_CARLA_MAPS = ALLOWED_NON_LAYERED_MAPS


class UnifiedCarlaGUI(tk.Tk):
    """
    Represents the main GUI application window for interaction with the CARLA Simulator.
    """

    def __init__(self):
        super().__init__()
        self.title("CARLA interaction GUI")
        self.geometry("950x800")
        self.resizable(True, True)

        self._current_stop_button: tk.Button | None = None

        self.config: Config = load()

        self.carla_executable_variable = tk.StringVar(value=self.config.carla_executable)
        self.recording_extension_variable = tk.StringVar(value=self.config.recording_extension)

        self.manual_output_dir_variable = tk.StringVar(value=self.config.manual_output_dir)
        self.default_recordings_folder_variable = tk.StringVar(value=self.config.default_recordings_folder)
        self.new_file_name_variable = tk.StringVar(value=self.config.new_file_name)

        self.transform_input_file_variable = tk.StringVar(value=self.config.transform_input_file)
        self.transformer_output_path_variable = tk.StringVar(value=self.config.transformer_output_path)
        self.video_input_path_variable = tk.StringVar(value=self.config.video_input_file)
        self.video_output_path_variable = tk.StringVar(value=self.config.video_output_path)

        self.video_width_variable = tk.IntVar(value=self.config.video_width)
        self.video_height_variable = tk.IntVar(value=self.config.video_height)
        self.vehicle_id_variable = tk.IntVar(value=self.config.vehicle_id)
        self.with_bboxes_variable = tk.BooleanVar(value=self.config.with_bboxes)
        self.begin_at_variable = tk.StringVar(value=str(self.config.begin_at))
        end_at_default = -1 if self.config.end_at == float("inf") else self.config.end_at
        self.end_at_variable = tk.StringVar(value=str(end_at_default))

        self.agent_vehicle_filter_variable = tk.StringVar(
            value=getattr(self.config, "agent_vehicle_filter", "vehicle.tesla.model3"))
        self.agent_target_speed_variable = tk.DoubleVar(
            value=getattr(self.config, "agent_target_speed_kph", 35.0))

        self.render_off_screen_variable = tk.BooleanVar(
            value=getattr(self.config, "render_off_screen", False)
        )
        self.render_quality_low_variable = tk.BooleanVar(
            value=getattr(self.config, "render_quality_low", False)
        )

        if getattr(self.config, "selected_map", "") in ALLOWED_CARLA_MAPS:
            default_map = self.config.selected_map
        else:
            default_map = ALLOWED_CARLA_MAPS[0]
        self.selected_map_variable = tk.StringVar(value=default_map)
        self._map_combos: list[ttk.Combobox] = []

        self._active_worker = None
        self._carla_worker = None

        self._manual_workers: list[ManualControlWorker] = []
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True)
        self._tab_server(notebook)
        self._tab_manual(notebook)
        self._tab_transform(notebook)
        self._tab_video(notebook)
        self._tab_generate_maps(notebook)
        self._tab_agent(notebook)

        # log pane
        self.log = scrolledtext.ScrolledText(self, height=30, state="disabled")
        self.log.pack(fill="both", expand=False, padx=4, pady=4)
        self._init_log_file()

        self._redirect_console()
        self._setup_autosave()

    def _entry_row(self, parent: tk.Widget, label: str, var: tk.Variable, browse: Callable = None, width: int = 45):
        """
        Creates a single row layout comprising a label, entry field, and optionally
        a browse button within the specified parent widget.

        Parameters:
        parent : tk.Widget
            The parent widget in which the entry row will be created.
        label : str
            The text to be displayed as the label in the row.
        var : tk.Variable
            The tkinter variable associated with the entry widget.
        browse : Callable, optional
            A callback function to be executed when the browse button is clicked.
        width : int
            The width of the entry field.
        """
        row = tk.Frame(parent)
        row.pack(fill="x", pady=2)
        tk.Label(row, text=label, width=26, anchor="w").pack(side="left")
        tk.Entry(row, textvariable=var, width=width) \
            .pack(side="left", fill="x", expand=True)
        if browse:
            tk.Button(row, text="...", command=browse) \
                .pack(side="left", padx=2)

    def _tab_server(self, notebook: ttk.Notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="CARLA Server")
        tk.Label(frame, text="Start or stop a head-less CARLA instance.").pack(pady=5)

        self._entry_row(frame, "CARLA executable:", self.carla_executable_variable,
                        lambda: self._open_file_dialog(self.carla_executable_variable))

        # Map selection (allowed maps only)
        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Map:", width=26, anchor="w").pack(side="left")
        ttk.Combobox(
            row,
            textvariable=self.selected_map_variable,
            state="readonly",
            values=ALLOWED_CARLA_MAPS,
            width=42
        ).pack(side="left", fill="x", expand=True)

        # Rendering options (unchanged)
        rendering_options = ttk.LabelFrame(frame, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(rendering_options, text="Render off screen",
                       variable=self.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(rendering_options, text="Render quality low",
                       variable=self.render_quality_low_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        self.server_btn = tk.Button(frame, text="Start CARLA server",
                                    width=25, command=self._toggle_carla)
        self.server_btn.pack(pady=10)

        self.stop_btn_server = tk.Button(frame, text="Stop",
                                         command=self._stop_worker, state="disabled")
        self.stop_btn_server.pack(pady=2)

    def _tab_manual(self, notebook: ttk.Notebook):
        """
        Initializes and organizes elements in the "Manual Drive" tab. Provides controls and input
        fields to facilitate interaction with the CARLA Simulator for manual driving operations.
        It also includes functionality to set paths and trigger starting and stopping of the manual
        driving process.

        Parameters
        ----------
        notebook : ttk.Notebook
            The notebook widget to which the "Manual Drive" tab is added. This parameter is used
            to integrate the frame containing all related elements within the notebook.
        """
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Manual Drive")
        tk.Label(frame, text="Let's you manually drive around in CARLA.").pack(pady=5)

        self._entry_row(frame, "CARLA executable:", self.carla_executable_variable,
                        lambda: self._open_file_dialog(self.carla_executable_variable))
        self._entry_row(frame, "Recording extension:", self.recording_extension_variable)
        self._entry_row(frame, "CARLA output folder:", self.manual_output_dir_variable,
                        lambda: self._open_directory_dialog(self.manual_output_dir_variable))
        self._entry_row(frame, "Archive recordings folder:", self.default_recordings_folder_variable,
                        lambda: self._open_directory_dialog(self.default_recordings_folder_variable))
        self._entry_row(frame, "New file-name prefix:", self.new_file_name_variable)

        # Map selection (allowed maps only)
        row = tk.Frame(frame)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Map:", width=26, anchor="w").pack(side="left")
        ttk.Combobox(
            row,
            textvariable=self.selected_map_variable,
            state="readonly",
            values=ALLOWED_CARLA_MAPS,
            width=42
        ).pack(side="left", fill="x", expand=True)

        # Rendering options (unchanged)
        rendering_options = ttk.LabelFrame(frame, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(rendering_options, text="Render off screen",
                       variable=self.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(rendering_options, text="Render quality low",
                       variable=self.render_quality_low_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        self.start_btn = tk.Button(frame, text="Start manual driving",
                                   width=25, command=self._start_manual)
        self.start_btn.pack(pady=8)

        row2 = tk.Frame(frame)
        row2.pack(fill="x", pady=4)

        tk.Label(row2, text="Add controlled actor:", width=26, anchor="w").pack(side="left")

        tk.Button(row2, text="Cyclist",
                  command=lambda: self._spawn_manual_extra(filter_str="vehicle.bh.crossbike")).pack(side="left", padx=2)

        tk.Button(row2, text="Walker",
                  command=lambda: self._spawn_manual_extra(filter_str="walker.pedestrian.*")).pack(side="left", padx=2)

        tk.Button(row2, text="Small car",
                  command=lambda: self._spawn_manual_extra(filter_str="vehicle.mini.cooper_s_2021")).pack(side="left",
                                                                                                          padx=2)

        tk.Button(row2, text="Truck",
                  command=lambda: self._spawn_manual_extra(filter_str="vehicle.carlamotors.carlacola")).pack(
            side="left", padx=2)

        self.move_btn = tk.Button(frame, text="Move 'manual_recording'",
                                  command=self._move_latest, state="active")
        self.move_btn.pack(pady=2)

        self.stop_btn_manual = tk.Button(frame, text="Stop",
                                         command=self._stop_worker, state="disabled")
        self.stop_btn_manual.pack(pady=8)

    def _tab_transform(self, notebook: ttk.Notebook):
        """
        Creates and configures the "Transform" tab in the provided notebook widget. This tab allows
        users to replay a recording and dump processed data. It provides fields for entering the
        recording extension, input recording path, and output folder, along with a button to
        initiate the transformation process.
        """
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Transform")
        tk.Label(frame, text="Replay a recording and dump processed data.").pack(pady=5)

        self._entry_row(frame, "Recording extension:", self.recording_extension_variable)
        self._entry_row(frame, "Input recording:", self.transform_input_file_variable,
                        lambda: self._open_file_selection_with_specified_extension(self.transform_input_file_variable))
        self._entry_row(frame, "Output folder:", self.transformer_output_path_variable,
                        lambda: self._open_directory_dialog(self.transformer_output_path_variable))

        # ── NEW: Rendering options ─────────────────────────────────────────────────
        rendering_options = ttk.LabelFrame(frame, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)

        tk.Checkbutton(
            rendering_options,
            text="Render off screen",
            variable=self.render_off_screen_variable,
            anchor="w",
        ).pack(fill="x", padx=6, pady=2)

        tk.Checkbutton(
            rendering_options,
            text="Render quality low",
            variable=self.render_quality_low_variable,
            anchor="w",
        ).pack(fill="x", padx=6, pady=2)
        # ──────────────────────────────────────────────────────────────────────────

        tk.Button(frame, text="Start transform",
                  command=self._start_transform).pack(pady=10)
        self.stop_btn_transform = tk.Button(frame, text="Stop",
                                            command=self._stop_worker, state="disabled")
        self.stop_btn_transform.pack(pady=8)

    def _tab_video(self, notebook: ttk.Notebook):
        """
        Creates a tab in the provided notebook that allows users to configure and export
        a recording directly to an MP4 format with custom video parameters and options.

        Parameters
        ----------
        notebook : ttk.Notebook
            The notebook widget where a new tab will be added for video recording.
        """
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Record ➜ MP4")
        tk.Label(frame, text="Export a recording directly to mp4.").pack(pady=5)

        self._entry_row(frame, "Recording extension:", self.recording_extension_variable)
        self._entry_row(frame, "Input recording:", self.video_input_path_variable,
                        lambda: self._open_file_selection_with_specified_extension(self.video_input_path_variable))
        self._entry_row(frame, "Output folder:", self.video_output_path_variable,
                        lambda: self._open_directory_dialog(self.video_output_path_variable))

        video_parameters_label_frame = ttk.LabelFrame(frame, text="Video parameters")
        video_parameters_label_frame.pack(fill="x", padx=4, pady=6)
        self._entry_row(video_parameters_label_frame, "Width:", self.video_width_variable, width=8)
        self._entry_row(video_parameters_label_frame, "Height:", self.video_height_variable, width=8)
        self._entry_row(video_parameters_label_frame, "Vehicle ID (-1 = ego):", self.vehicle_id_variable, width=8)

        vcmd = (self.register(self._validate_number), "%P")  # %P = proposed value

        # Start at (s)
        row = tk.Frame(video_parameters_label_frame)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Start at (s):", width=26, anchor="w").pack(side="left")
        tk.Entry(
            row,
            textvariable=self.begin_at_variable,  # StringVar (see __init__ note)
            width=8,
            validate="key",
            validatecommand=vcmd
        ).pack(side="left", fill="x", expand=True)

        # End at (s, -1 = file end)
        row = tk.Frame(video_parameters_label_frame)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="End at (s, -1 = file end):", width=26, anchor="w").pack(side="left")
        tk.Entry(
            row,
            textvariable=self.end_at_variable,  # StringVar (see __init__ note)
            width=8,
            validate="key",
            validatecommand=vcmd
        ).pack(side="left", fill="x", expand=True)
        # --------------------------------------------------------------

        tk.Checkbutton(video_parameters_label_frame, text="Draw 3-D bounding boxes",
                       variable=self.with_bboxes_variable).pack(anchor="w", padx=4, pady=4)

        # ── Rendering options for video export ─────────────────────────
        rendering_options = ttk.LabelFrame(frame, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(
            rendering_options,
            text="Render quality low",
            variable=self.render_quality_low_variable
        ).pack(anchor="w", padx=4, pady=4)
        # ───────────────────────────────────────────────────────────────

        tk.Button(frame, text="Start recording",
                  command=self._start_video).pack(pady=10)

        self.stop_btn_video = tk.Button(frame, text="Stop",
                                        command=self._stop_worker, state="disabled")
        self.stop_btn_video.pack(pady=8)

    def _tab_generate_maps(self, notebook: ttk.Notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Generate Maps")
        tk.Label(frame, text="Generate map data for all allowed maps.").pack(pady=5)

        # Reuse CARLA exe and output folder
        self._entry_row(frame, "CARLA executable:", self.carla_executable_variable,
                        lambda: self._open_file_dialog(self.carla_executable_variable))
        self._entry_row(frame, "Maps output folder:", self.transformer_output_path_variable,
                        lambda: self._open_directory_dialog(self.transformer_output_path_variable))

        # Rendering options (same as other tabs)
        rendering_options = ttk.LabelFrame(frame, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(rendering_options, text="Render off screen",
                       variable=self.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(rendering_options, text="Render quality low",
                       variable=self.render_quality_low_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        tk.Button(frame, text="Generate all maps", command=self._start_generate_maps).pack(pady=10)
        self.stop_btn_generate = tk.Button(frame, text="Stop",
                                           command=self._stop_worker, state="disabled")
        self.stop_btn_generate.pack(pady=8)

    def _tab_agent(self, notebook: ttk.Notebook):
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="Agent Drive")
        tk.Label(frame, text="Start a Python Agent controlling a single ego car.").pack(pady=5)

        self._entry_row(frame, "CARLA executable:", self.carla_executable_variable,
                        lambda: self._open_file_dialog(self.carla_executable_variable))
        # Map select
        row = tk.Frame(frame);
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Map:", width=26, anchor="w").pack(side="left")
        ttk.Combobox(row, textvariable=self.selected_map_variable,
                     state="readonly", values=ALLOWED_CARLA_MAPS, width=42).pack(side="left", fill="x", expand=True)

        # Agent settings
        agent_opts = ttk.LabelFrame(frame, text="Agent parameters")
        agent_opts.pack(fill="x", padx=4, pady=6)
        self._entry_row(agent_opts, "Vehicle filter:", self.agent_vehicle_filter_variable)
        self._entry_row(agent_opts, "Target speed (kph):", self.agent_target_speed_variable, width=10)

        # Rendering options
        rendering_options = ttk.LabelFrame(frame, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(rendering_options, text="Render off screen",
                       variable=self.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(rendering_options, text="Render quality low",
                       variable=self.render_quality_low_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        # Controls
        row_btns = tk.Frame(frame);
        row_btns.pack(fill="x", pady=6)
        tk.Button(row_btns, text="Start Agent", width=20, command=self._start_manual_agent).pack(side="left", padx=2)

        self.stop_btn_agent = tk.Button(frame, text="Stop",
                                        command=self._stop_worker, state="disabled")
        self.stop_btn_agent.pack(pady=8)

    def _start_manual_agent(self):
        if not self._validate_paths([("CARLA executable", self.carla_executable_variable, "file")]):
            return
        self._clear_log()
        cfg = self._collect_cfg()

        class _Runner(ThreadWorker):
            RUNNER = "carla_task_runner.py"

            def run(self_inner):
                runner = self_inner._resolve_runner()
                if not runner:
                    self_inner.log("!! Could not locate carla_task_runner.py")
                    return
                cmd = [sys.executable or "python", runner, "manual_agent",
                       "--carla-exe", cfg.carla_executable]
                m = (getattr(cfg, "selected_map", "") or "").strip()
                if m:
                    cmd += ["--map-name", m]
                if getattr(cfg, "render_quality_low", False):
                    cmd.append("--quality-low")
                # viewer needs a window; do NOT pass --offscreen
                cmd += ["--res", "1280x720", "--sync"]
                self_inner._start_and_stream(cmd)

        w = _Runner(cfg, self._log)
        self._attach_worker(w, stop_button=self.stop_btn_agent)

    def _validate_number(self, proposed: str) -> bool:
        """
        Entry validator: allow empty, integers, or floats (with optional leading '-').
        This lets users type partial values like '-', '.', '-.' while editing.
        """
        if proposed in ("", "-", ".", "-."):
            return True
        try:
            float(proposed)
            return True
        except ValueError:
            return False

    def _attach_worker(self, worker: ThreadWorker, *, stop_button: tk.Button | None = None, enable_move: bool = False):
        """
        Attaches a worker to the system and manages its execution, UI updates, and
        state tracking.

        Parameters:
            worker (ThreadWorker): The worker instance to be attached.
            enable_move (bool, optional): Flag to enable or disable "move" functionality in the UI. Default is False.

        Returns:
            None: This method has no return value.
        """
        if worker.exclusive and self._active_worker:
            return messagebox.showwarning("Busy", "Another exclusive task is running.")

        worker.start()

        if worker.exclusive:
            self._active_worker = worker
            # enable the specific stop button for this task
            if stop_button is not None:
                stop_button.config(state="normal")
                self._current_stop_button = stop_button

        def poll():
            if worker.is_alive():
                self.after(500, poll)
            else:
                if worker is self._active_worker:
                    self._active_worker = None
                    # disable whichever stop button was tied to this worker
                    if self._current_stop_button is not None:
                        self._current_stop_button.config(state="disabled")
                        self._current_stop_button = None

        poll()
        return None

    def _toggle_carla(self):
        """
        Toggles the CARLA server process between starting and stopping states. If the CARLA server is running,
        it cancels the server process. If the server is not running, it validates the configuration and starts a
        new CARLA server process.
        """
        if self._carla_worker and self._carla_worker.is_alive():
            self._carla_worker.cancel()
            self.server_btn.config(text="Start CARLA server")
            if hasattr(self, "stop_btn_server"):
                self.stop_btn_server.config(state="disabled")
        else:
            if not self._validate_paths([
                ("CARLA executable", self.carla_executable_variable, "file"),
            ]):
                return
            self._clear_log()
            self._carla_worker = CarlaServerWorker(self._collect_cfg(), self._log)
            self._carla_worker.start()
            self.server_btn.config(text="Stop CARLA server")
            if hasattr(self, "stop_btn_server"):
                self.stop_btn_server.config(state="normal")

    def _start_manual(self):
        """
        Start the primary manual driving (exclusive) as Lincoln MKZ 2020.
        """
        if not self._validate_paths([
            ("CARLA executable", self.carla_executable_variable, "file"),
            ("CARLA output folder", self.manual_output_dir_variable, "dir"),
        ]):
            return
        self._clear_log()

        w = ManualControlWorker(
            self._collect_cfg(),
            self._log,
            vehicle_filter="vehicle.lincoln.mkz_2020",  # requested default
            role_name=None,  # manual_control.py default -> "hero"
            restart_before=True,
            kill_server_after=True,
            exclusive=True,
        )
        self._manual_workers.append(w)
        self._attach_worker(w, stop_button=self.stop_btn_manual, enable_move=True)

    def _spawn_manual_extra(self, *, filter_str: str):
        """
        Launch another manual_control.py instance with --rolename=manual_control and
        the provided --filter, without rebooting/killing the CARLA server.
        """
        if not self._validate_paths([
            ("CARLA executable", self.carla_executable_variable, "file"),
        ]):
            return

        w = ManualControlWorker(
            self._collect_cfg(),
            self._log,
            vehicle_filter=filter_str,
            role_name="manual_control",
            restart_before=False,  # do NOT reboot the server
            kill_server_after=False,  # do NOT kill the server when this instance ends
            exclusive=False,
        )
        self._manual_workers.append(w)
        # non-exclusive: do not pass a stop_button tied to exclusivity
        self._attach_worker(w, stop_button=None, enable_move=False)

    def _move_latest(self):
        """
        Moves the latest recording with the specified prefix.
        """
        if not self.new_file_name_variable.get().strip():
            return messagebox.showerror("Missing", "File-name prefix required.")
        if not self._validate_paths([
            ("Archive recordings folder", self.default_recordings_folder_variable, "dir"),
        ]):
            return
        self._attach_worker(
            MoveLatestRecordingWorker(self._collect_cfg(), self.new_file_name_variable.get(), self._log))
        return None

    def _start_transform(self):
        """
        Starts the recording transformation process by validating input parameters, collecting configuration data,
        and attaching the TransformRecordingWorker for the task.
        """
        if not self._validate_paths([
            ("CARLA executable", self.carla_executable_variable, "file"),
            ("Input recording", self.transform_input_file_variable, "file"),
            ("Output folder", self.transformer_output_path_variable, "dir"),
        ]):
            return

        self._clear_log()
        config = self._collect_cfg()
        config.transform_input_file = self.transform_input_file_variable.get().strip()
        config.transformer_output_path = self.transformer_output_path_variable.get().strip()
        self._attach_worker(TransformRecordingWorker(config, self._log),
                            stop_button=self.stop_btn_transform)

    def _start_video(self):
        """
        Starts the video recording process by validating input parameters, collecting configuration data,
        and attaching the RecordVideoWorker for the task.
        """
        if not self._validate_paths([
            ("CARLA executable", self.carla_executable_variable, "file"),
            ("Input recording", self.video_input_path_variable, "file"),
            ("Output folder", self.video_output_path_variable, "dir"),
        ]):
            return

        self._clear_log()
        config = self._collect_cfg()
        config.video_input_file = self.video_input_path_variable.get().strip()
        config.video_output_path = self.video_output_path_variable.get().strip()

        config.video_width = self.video_width_variable.get()
        config.video_height = self.video_height_variable.get()
        config.vehicle_id = self.vehicle_id_variable.get()
        config.with_bboxes = self.with_bboxes_variable.get()

        def _to_float(s: str, default: float) -> float:
            s = (s or "").strip()
            if not s:
                return default
            return float(s)

        config.begin_at = max(0.0, _to_float(self.begin_at_variable.get(), 0.0))

        end_str = (self.end_at_variable.get() or "").strip()
        if not end_str:
            # empty -> use file end
            config.end_at = float("inf")
        else:
            end_val = float(end_str)
            config.end_at = float("inf") if end_val < 0 else end_val

        self._attach_worker(RecordVideoWorker(config, self._log),
                            stop_button=self.stop_btn_video)

    def _start_generate_maps(self):
        if not self._validate_paths([
            ("CARLA executable", self.carla_executable_variable, "file"),
            ("Maps output folder", self.transformer_output_path_variable, "dir"),
        ]):
            return

        self._clear_log()
        cfg = self._collect_cfg()
        cfg.maps_output_path = self.transformer_output_path_variable.get().strip()

        # Only use the allowed maps constant here (as requested)
        allowed_maps = list(ALLOWED_NON_LAYERED_MAPS)

        from carla_interaction_gui.workers.GenerateMapsWorker import GenerateMapsWorker
        self._attach_worker(
            GenerateMapsWorker(cfg, self._log, allowed_maps),
            stop_button=self.stop_btn_generate
        )

    def _stop_worker(self):
        """
        Stops any active exclusive worker, stops all manual_control workers,
        and stops the CARLA server worker as before.
        """
        # stop ALL manual_control workers
        for w in list(self._manual_workers):
            try:
                if w.is_alive():
                    w.cancel()
            except Exception:
                pass
        self._manual_workers.clear()

        # existing exclusive worker stop (kept)
        w = getattr(self, "_active_worker", None)
        if w:
            try:
                w.cancel()
            except Exception:
                pass
            try:
                w.join(timeout=5.0)
            except Exception:
                pass
            self._active_worker = None
            try:
                if self._current_stop_button is not None:
                    self._current_stop_button.config(state="disabled")
                    self._current_stop_button = None
            except Exception:
                pass

            # Defensive: also disable any per-tab stop buttons if they exist
            for btn_name in ("stop_btn_manual", "stop_btn_server", "stop_btn_transform", "stop_btn_video",
                             "stop_btn_agent", "btn_agent_toggle_ap", "btn_agent_toggle_rec"):
                btn = getattr(self, btn_name, None)
                if isinstance(btn, tk.Button):
                    try:
                        btn.config(state="disabled")
                    except Exception:
                        pass

        # stop CARLA server worker if running
        if getattr(self, "_carla_worker", None) and self._carla_worker.is_alive():
            try:
                self._carla_worker.cancel()
            except Exception:
                pass
            try:
                self._carla_worker.join(timeout=5.0)
            except Exception:
                pass
            try:
                self.server_btn.config(text="Start CARLA server")
            except Exception:
                pass

        kill_carla()

    def _setup_autosave(self):
        """
        Sets up automatic saving for specified variables.
        """
        for variable in (
                self.carla_executable_variable,
                self.recording_extension_variable,
                self.manual_output_dir_variable,
                self.default_recordings_folder_variable,
                self.new_file_name_variable,
                self.transform_input_file_variable,
                self.transformer_output_path_variable,
                self.video_input_path_variable,
                self.video_output_path_variable,
                self.video_width_variable,
                self.video_height_variable,
                self.vehicle_id_variable,
                self.with_bboxes_variable,
                self.begin_at_variable,
                self.end_at_variable,
                self.render_off_screen_variable,
                self.render_quality_low_variable,
                self.agent_vehicle_filter_variable,
                self.agent_target_speed_variable
        ):
            variable.trace_add("write", self._auto_save)

    def _auto_save(self, *_):
        """
        Handles automatic saving of current configurations.
        Skips saving while numeric fields are in an in-progress state.
        """
        # Skip when user is mid-typing a number
        transient = {"", "-", ".", "-."}
        if (self.begin_at_variable.get() in transient or
                self.end_at_variable.get() in transient):
            return
        try:
            self._collect_cfg()
        except Exception:
            # Ignore transient parsing glitches while editing
            pass

    def _on_close(self):
        try:
            self._stop_worker()
        finally:
            try:
                self.destroy()
            except Exception:
                pass

    def _collect_cfg(self) -> Config:
        """
        Collects and normalizes configuration data from various sources, updates the
        config object, and saves it.
        """
        config = self.config
        config.carla_executable = self.carla_executable_variable.get().strip()
        config.recording_extension = self._normalize_extension(self.recording_extension_variable.get())
        config.manual_output_dir = self.manual_output_dir_variable.get().strip()
        config.default_recordings_folder = self.default_recordings_folder_variable.get().strip()
        config.new_file_name = self.new_file_name_variable.get().strip()
        config.transform_input_file = self.transform_input_file_variable.get().strip()
        config.transformer_output_path = self.transformer_output_path_variable.get().strip()
        config.video_input_file = self.video_input_path_variable.get().strip()
        config.video_output_path = self.video_output_path_variable.get().strip()
        config.with_bboxes = self.with_bboxes_variable.get()
        config.video_width = self.video_width_variable.get()
        config.video_height = self.video_height_variable.get()
        config.vehicle_id = self.vehicle_id_variable.get()
        config.agent_vehicle_filter = self.agent_vehicle_filter_variable.get().strip()
        try:
            config.agent_target_speed_kph = float(self.agent_target_speed_variable.get())
        except Exception:
            config.agent_target_speed_kph = 35.0

        # ---- tolerant numeric parsing (works with StringVar or DoubleVar) ----
        def _parse_var_as_float(var, default: float) -> float:
            try:
                val = var.get()
            except Exception:
                return default
            # normalize to string for consistent handling
            s = str(val).strip()
            if s in ("", "-", ".", "-."):
                return default
            try:
                return float(s)
            except ValueError:
                return default

        # begin_at: clamp to >= 0
        begin = _parse_var_as_float(self.begin_at_variable, 0.0)
        config.begin_at = max(0.0, begin)

        # end_at: empty -> file end; any negative -> file end
        end_val = _parse_var_as_float(self.end_at_variable, float("inf"))
        config.end_at = float("inf") if end_val < 0 else end_val
        # ---------------------------------------------------------------------

        # rendering + selected map (if you added these previously)
        if hasattr(self, "render_off_screen_variable"):
            config.render_off_screen = self.render_off_screen_variable.get()
        if hasattr(self, "render_quality_low_variable"):
            config.render_quality_low = self.render_quality_low_variable.get()
        if hasattr(self, "selected_map_variable"):
            config.selected_map = self.selected_map_variable.get().strip()

        save(config)
        return config

    def _open_file_dialog(self, variable: tk.StringVar):
        """
        Selects a file using a file dialog and sets it to the provided variable.

        Parameters:
            variable (StringVar): A Tkinter StringVar instance to update with the
            selected file path.
        """
        selected_file = filedialog.askopenfilename()
        variable.set(selected_file or variable.get())

    def _open_file_selection_with_specified_extension(self, variable: tk.StringVar):
        """
        Helper method for file selection with the specified file type and extension.

        Parameters:
            variable: A tkinter `StringVar` instance that holds the file path to be
                updated.

        Raises:
            No explicit error is raised, but runtime errors may occur if `var` is not
            a tkinter-compatible variable.
        """
        extension = self._normalize_extension(self.recording_extension_variable.get())
        selected_file = filedialog.askopenfilename(filetypes=[(f"{extension} files", f"*{extension}"),
                                                              ("All files", "*.*")])
        variable.set(selected_file or variable.get())

    def _open_directory_dialog(self, variable: tk.StringVar):
        """
        Handles a directory selection dialog and assigns the selected path.

        Parameters:
            variable (tkinter.StringVar): A Tkinter StringVar object that holds the current
            directory path to be updated with the selected path.
        """
        selected_directory = filedialog.askdirectory()
        variable.set(selected_directory or variable.get())

    @staticmethod
    def _normalize_extension(ext: str) -> str:
        """
        Normalizes a given file extension by ensuring it starts with a period.

        Parameters:
            ext: The file extension to normalize.

        Returns:
            The normalized file extension string.
        """
        return ext if ext.startswith(".") else f".{ext}"

    def _redirect_console(self):
        """
        Redirects console output to the GUI log.
        """

        class _TextOutputHandler:
            def __init__(self, gui): self.gui = gui

            def write(self, txt):
                for lines in txt.rstrip().splitlines():
                    self.gui._log(lines)

            def flush(self): pass

        sys.stdout = sys.stderr = _TextOutputHandler(self)

    def _log(self, txt: str):
        """
        Logs a given text message to the GUI text widget and also appends it to a file.
        """
        # 1) GUI pane
        self.log.configure(state="normal")
        self.log.insert("end", txt + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

        # 2) File (append). Open per write to avoid cross-thread handle issues.
        #    If _init_log_file didn't run yet for some reason, skip quietly.
        log_path = getattr(self, "_log_file_path", None)
        if log_path:
            try:
                with open(log_path, "a", encoding="utf-8") as f:
                    f.write(txt + "\n")
            except Exception:
                # never let file I/O break the GUI
                pass

    def _clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _init_log_file(self):
        """
        Decide where to write the session log and create the folder if needed.
        """
        # put logs into a 'logs' folder next to this script
        logs_dir = os.path.join(self.config.transformer_output_path, "logs")
        try:
            os.makedirs(logs_dir, exist_ok=True)
        except Exception:
            # fallback to current working directory if making 'logs' failed
            logs_dir = os.getcwd()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_file_path = os.path.join(logs_dir, f"carla_gui_{timestamp}.log")

        # write a small header
        try:
            with open(self._log_file_path, "a", encoding="utf-8") as f:
                f.write(f"=== CARLA GUI log started {datetime.now().isoformat()} ===\n")
        except Exception:
            # don't crash the GUI if file logging can't be set up
            pass

    def _validate_paths(self, specs: list[tuple[str, tk.Variable, str]]) -> bool:
        """
        Validate that the given variables point to existing paths.

        specs: list of (label, variable, kind) where kind in {"file","dir","any"}.
        Shows a messagebox and returns False on the first invalid item.
        """
        for label, var, kind in specs:
            try:
                value = var.get().strip()
            except Exception:
                value = str(var).strip()

            if not value:
                messagebox.showerror("Missing", f"{label} required.")
                return False

            if kind == "file":
                if not os.path.isfile(value):
                    messagebox.showerror("Invalid path", f"{label} does not exist as a file:\n{value}")
                    return False
            elif kind == "dir":
                if not os.path.isdir(value):
                    messagebox.showerror("Invalid path", f"{label} does not exist as a folder:\n{value}")
                    return False
            else:  # "any"
                if not os.path.exists(value):
                    messagebox.showerror("Invalid path", f"{label} path does not exist:\n{value}")
                    return False
        return True


if __name__ == "__main__":
    UnifiedCarlaGUI().mainloop()
