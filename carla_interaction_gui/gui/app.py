from __future__ import annotations

import os
import queue
import sys
import tkinter as tk
from datetime import datetime
from tkinter import ttk, filedialog, messagebox, scrolledtext

from carla_interaction_gui.carla_launcher import kill_carla
from carla_interaction_gui.config_data import Config, load, save
from carla_interaction_gui.gui.constants import ALLOWED_CARLA_MAPS, ALLOWED_EGO_VEHICLES
from helpers.camera_recorder.CameraPosition import CameraPosition
from carla_interaction_gui.gui.tabs.manual_tab import ManualTab
from carla_interaction_gui.gui.tabs.maps_tab import MapsTab
from carla_interaction_gui.gui.tabs.recgen_tab import RecGenTab
from carla_interaction_gui.gui.tabs.server_tab import ServerTab
from carla_interaction_gui.gui.tabs.transform_tab import TransformTab
from carla_interaction_gui.gui.tabs.video_tab import VideoTab
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class CarlaInteractionGUI(tk.Tk):
    """
    Main GUI application window for interaction with the CARLA Simulator.

    Owns the persisted configuration, the tk.Variables bound to it, and the
    cross-cutting services (background-worker lifecycle, logging, path
    validation, file dialogs) that every tab relies on. Each tab's own UI and
    click handlers live in carla_interaction_gui.gui.tabs and are handed a
    reference to this app to reach those shared services.
    """

    def __init__(self):
        super().__init__()
        self.title("CARLA interaction GUI")
        self.geometry("3000x2000")
        self.resizable(True, True)

        self._current_stop_button: tk.Button | None = None
        self._stop_buttons: list[tk.Button] = []

        self.config: Config = load()

        self.carla_executable_variable = tk.StringVar(value=self.config.carla_executable)
        self.recording_extension_variable = tk.StringVar(value=self.config.recording_extension)

        self.manual_output_dir_variable = tk.StringVar(value=self.config.manual_output_dir)
        self.default_recordings_folder_variable = tk.StringVar(value=self.config.default_recordings_folder)
        self.new_file_name_variable = tk.StringVar(value=self.config.new_file_name)
        self.manual_spawn_traffic_num_vehicles_variable = tk.IntVar(
            value=getattr(self.config, "manual_spawn_traffic_num_vehicles", 30))
        self.manual_spawn_traffic_enabled_variable = tk.BooleanVar(
            value=getattr(self.config, "manual_spawn_traffic_enabled", False))

        self.transform_input_file_variable = tk.StringVar(value=self.config.transform_input_file)
        self.transform_docker_mount_path_variable = tk.StringVar(
            value=getattr(self.config, "transform_docker_mount_path", ""))
        self.transformer_output_path_variable = tk.StringVar(value=self.config.transformer_output_path)
        self.video_input_path_variable = tk.StringVar(value=self.config.video_input_file)
        self.video_docker_mount_path_variable = tk.StringVar(
            value=getattr(self.config, "video_docker_mount_path", ""))
        self.video_output_path_variable = tk.StringVar(value=self.config.video_output_path)

        self.video_width_variable = tk.IntVar(value=self.config.video_width)
        self.video_height_variable = tk.IntVar(value=self.config.video_height)
        self.video_fov_variable = tk.IntVar(value=getattr(self.config, "video_fov", 105))
        self.vehicle_id_variable = tk.IntVar(value=self.config.vehicle_id)
        self.begin_at_variable = tk.StringVar(value=str(self.config.begin_at))
        end_at_default = -1 if self.config.end_at == float("inf") else self.config.end_at
        self.end_at_variable = tk.StringVar(value=str(end_at_default))

        # One (selected, metadata, bbox) BooleanVar triple per CameraPosition,
        # so the Record->MP4 tab can let the user pick which camera angles to
        # render and whether each shows metadata text/bounding boxes.
        self.video_camera_position_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        saved_positions = {
            c.get("name"): c for c in (getattr(self.config, "video_camera_positions", None) or [])
        }
        for pos in CameraPosition:
            saved = saved_positions.get(pos.name)
            self.video_camera_position_vars[pos.name] = {
                "selected": tk.BooleanVar(value=saved is not None),
                "metadata": tk.BooleanVar(value=bool(saved.get("metadata")) if saved else False),
                "bbox": tk.BooleanVar(value=bool(saved.get("bbox")) if saved else False),
            }
        self.video_render_safety_boxes_variable = tk.BooleanVar(
            value=getattr(self.config, "video_render_safety_boxes", False))
        self.video_safety_box_style_variable = tk.StringVar(
            value=getattr(self.config, "video_safety_box_style", "HATCHING"))

        self.render_off_screen_variable = tk.BooleanVar(value=getattr(self.config, "render_off_screen", False))
        self.render_quality_low_variable = tk.BooleanVar(value=getattr(self.config, "render_quality_low", False))

        self.only_track_at_specific_interval_variable = tk.BooleanVar(
            value=getattr(self.config, "only_track_at_specific_interval", False))
        self.specific_track_interval_variable = tk.DoubleVar(
            value=getattr(self.config, "specific_track_interval", 0.5))

        if getattr(self.config, "selected_map", "") in ALLOWED_CARLA_MAPS:
            default_map = self.config.selected_map
        else:
            default_map = ALLOWED_CARLA_MAPS[0]
        self.selected_map_variable = tk.StringVar(value=default_map)

        if getattr(self.config, "selected_ego_vehicle", "") in ALLOWED_EGO_VEHICLES:
            default_ego_vehicle = self.config.selected_ego_vehicle
        else:
            default_ego_vehicle = ALLOWED_EGO_VEHICLES[0]
        self.selected_ego_vehicle_variable = tk.StringVar(value=default_ego_vehicle)

        self.recgen_seed_start_var = tk.IntVar(value=getattr(self.config, "recgen_seed_start", 0))
        self.recgen_num_scenarios_var = tk.IntVar(value=getattr(self.config, "recgen_num_scenarios", 1))
        self.recgen_num_vehicles_var = tk.IntVar(value=getattr(self.config, "recgen_num_vehicles", 200))
        self.recgen_num_walkers_var = tk.IntVar(value=getattr(self.config, "recgen_num_walkers", 30))
        self.recgen_filter_vehicles_var = tk.StringVar(
            value=getattr(self.config, "recgen_filter_vehicles", "vehicle.*"))
        self.recgen_generation_vehicles_var = tk.StringVar(
            value=getattr(self.config, "recgen_generation_vehicles", "All"))
        self.recgen_filter_walkers_var = tk.StringVar(
            value=getattr(self.config, "recgen_filter_walkers", "walker.pedestrian.*"))
        self.recgen_generation_walkers_var = tk.StringVar(
            value=getattr(self.config, "recgen_generation_walkers", "2"))
        self.recgen_length_minutes_var = tk.DoubleVar(value=getattr(self.config, "recgen_length_minutes", 5.0))
        self.recgen_output_dir_variable = tk.StringVar(value=getattr(self.config, "recgen_output_dir", ""))
        self.recgen_num_parked_var = tk.IntVar(value=getattr(self.config, "recgen_num_parked", 0))

        self._active_worker = None
        self.carla_worker = None
        self.manual_workers: list[ThreadWorker] = []
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True)
        self.server_tab = ServerTab(notebook, self)
        self.manual_tab = ManualTab(notebook, self)
        self.transform_tab = TransformTab(notebook, self)
        self.video_tab = VideoTab(notebook, self)
        self.maps_tab = MapsTab(notebook, self)
        self.recgen_tab = RecGenTab(notebook, self)

        # log pane
        self.log_widget = scrolledtext.ScrolledText(self, height=30, state="disabled")
        self.log_widget.pack(fill="both", expand=False, padx=4, pady=4)
        self._init_log_file()

        # Every ThreadWorker subprocess (record_video, transform, recgen, ...)
        # streams its output through log() from its own background thread,
        # and sys.stdout/sys.stderr are globally redirected into it too (see
        # _redirect_console below). Tcl/Tk is not thread-safe: mutating
        # self.log_widget from any thread other than the one running
        # mainloop() races with Tk's own event processing and corrupts the
        # interpreter's internal state, which segfaults the whole process
        # with no Python traceback - exactly the native crashes seen when
        # driving these tasks from the GUI (they never reproduce running the
        # same command from a plain shell, since there's no Tk widget to
        # race against there). log() only ever enqueues; _drain_log_queue,
        # scheduled here via after() and therefore always running on the
        # main thread, is the only thing allowed to touch the widget.
        self._log_queue: queue.Queue[str] = queue.Queue()
        self.after(50, self._drain_log_queue)

        self._redirect_console()
        self._setup_autosave()

    # ------------------------------------------------------------------
    # Worker lifecycle
    # ------------------------------------------------------------------

    def register_stop_button(self, button: tk.Button) -> None:
        """Track a tab's stop button so a global stop can disable it."""
        self._stop_buttons.append(button)

    def attach_worker(self, worker: ThreadWorker, *, stop_button: tk.Button | None = None):
        """
        Starts a worker and manages its lifecycle in the UI: guards against
        starting a second exclusive worker while one is running, and
        re-enables the stop button once the worker finishes.
        """
        if worker.exclusive and self._active_worker:
            return messagebox.showwarning("Busy", "Another exclusive task is running.")

        worker.start()

        if worker.exclusive:
            self._active_worker = worker
            if stop_button is not None:
                stop_button.config(state="normal")
                self._current_stop_button = stop_button

        def poll():
            if worker.is_alive():
                self.after(500, poll)
            else:
                if worker is self._active_worker:
                    self._active_worker = None
                    if self._current_stop_button is not None:
                        self._current_stop_button.config(state="disabled")
                        self._current_stop_button = None

        poll()
        return None

    def stop_worker(self):
        """
        Stops any active exclusive worker, stops all manual_control workers,
        and stops the CARLA server worker.
        """
        for w in list(self.manual_workers):
            try:
                if w.is_alive():
                    w.cancel()
            except Exception:
                pass
        self.manual_workers.clear()

        w = self._active_worker
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

            for btn in self._stop_buttons:
                try:
                    btn.config(state="disabled")
                except Exception:
                    pass

        if self.carla_worker and self.carla_worker.is_alive():
            try:
                self.carla_worker.cancel()
            except Exception:
                pass
            try:
                self.carla_worker.join(timeout=5.0)
            except Exception:
                pass
            self.server_tab.reset_server_button()

        kill_carla()

    # ------------------------------------------------------------------
    # Config persistence
    # ------------------------------------------------------------------

    def collect_cfg(self) -> Config:
        """
        Collects and normalizes configuration data from every tab's variables,
        updates the persisted config object, and saves it.
        """
        config = self.config
        config.carla_executable = self.carla_executable_variable.get().strip()
        config.recording_extension = self.normalize_extension(self.recording_extension_variable.get())
        config.manual_output_dir = self.manual_output_dir_variable.get().strip()
        config.default_recordings_folder = self.default_recordings_folder_variable.get().strip()
        config.new_file_name = self.new_file_name_variable.get().strip()
        config.manual_spawn_traffic_num_vehicles = max(1, int(self.manual_spawn_traffic_num_vehicles_variable.get()))
        config.manual_spawn_traffic_enabled = bool(self.manual_spawn_traffic_enabled_variable.get())
        config.transform_input_file = self.transform_input_file_variable.get().strip()
        config.transform_docker_mount_path = self.transform_docker_mount_path_variable.get().strip()
        config.transformer_output_path = self.transformer_output_path_variable.get().strip()
        config.video_input_file = self.video_input_path_variable.get().strip()
        config.video_docker_mount_path = self.video_docker_mount_path_variable.get().strip()
        config.video_output_path = self.video_output_path_variable.get().strip()
        config.video_width = self.video_width_variable.get()
        config.video_height = self.video_height_variable.get()
        config.video_fov = self.video_fov_variable.get()
        config.vehicle_id = self.vehicle_id_variable.get()
        config.video_camera_positions = [
            {"name": name, "metadata": v["metadata"].get(), "bbox": v["bbox"].get()}
            for name, v in self.video_camera_position_vars.items() if v["selected"].get()
        ]
        config.video_render_safety_boxes = bool(self.video_render_safety_boxes_variable.get())
        config.video_safety_box_style = self.video_safety_box_style_variable.get()

        config.recgen_seed_start = int(self.recgen_seed_start_var.get())
        config.recgen_num_scenarios = max(1, int(self.recgen_num_scenarios_var.get()))
        config.recgen_selected_maps = [m for m, v in self.recgen_tab.map_vars.items() if v.get()]
        config.recgen_num_vehicles = int(self.recgen_num_vehicles_var.get())
        config.recgen_num_walkers = int(self.recgen_num_walkers_var.get())
        config.recgen_filter_vehicles = (self.recgen_filter_vehicles_var.get() or "vehicle.*").strip()
        config.recgen_generation_vehicles = (self.recgen_generation_vehicles_var.get() or "All").strip()
        config.recgen_filter_walkers = (self.recgen_filter_walkers_var.get() or "walker.pedestrian.*").strip()
        config.recgen_generation_walkers = (self.recgen_generation_walkers_var.get() or "2").strip()
        try:
            config.recgen_length_minutes = float(self.recgen_length_minutes_var.get())
        except Exception:
            config.recgen_length_minutes = 5.0
        config.recgen_output_dir = self.recgen_output_dir_variable.get().strip()
        config.recgen_num_parked = max(0, int(self.recgen_num_parked_var.get()))

        def _parse_var_as_float(var, default: float) -> float:
            try:
                val = var.get()
            except Exception:
                return default
            s = str(val).strip()
            if s in ("", "-", ".", "-."):
                return default
            try:
                return float(s)
            except ValueError:
                return default

        begin = _parse_var_as_float(self.begin_at_variable, 0.0)
        config.begin_at = max(0.0, begin)

        end_val = _parse_var_as_float(self.end_at_variable, float("inf"))
        config.end_at = float("inf") if end_val < 0 else end_val

        config.render_off_screen = self.render_off_screen_variable.get()
        config.render_quality_low = self.render_quality_low_variable.get()
        config.selected_map = self.selected_map_variable.get().strip()
        config.selected_ego_vehicle = self.selected_ego_vehicle_variable.get().strip()

        config.only_track_at_specific_interval = bool(self.only_track_at_specific_interval_variable.get())
        config.specific_track_interval = _parse_var_as_float(self.specific_track_interval_variable, 0.5)

        save(config)
        return config

    def _setup_autosave(self):
        """Persist the config whenever any bound variable changes."""
        for variable in (
                self.carla_executable_variable,
                self.recording_extension_variable,
                self.manual_output_dir_variable,
                self.default_recordings_folder_variable,
                self.new_file_name_variable,
                self.manual_spawn_traffic_num_vehicles_variable,
                self.manual_spawn_traffic_enabled_variable,
                self.transform_input_file_variable,
                self.transform_docker_mount_path_variable,
                self.transformer_output_path_variable,
                self.video_input_path_variable,
                self.video_docker_mount_path_variable,
                self.video_output_path_variable,
                self.video_width_variable,
                self.video_height_variable,
                self.video_fov_variable,
                self.vehicle_id_variable,
                self.video_render_safety_boxes_variable,
                self.video_safety_box_style_variable,
                *[v for group in self.video_camera_position_vars.values() for v in group.values()],
                self.begin_at_variable,
                self.end_at_variable,
                self.render_off_screen_variable,
                self.render_quality_low_variable,
                self.selected_ego_vehicle_variable,
                self.only_track_at_specific_interval_variable,
                self.specific_track_interval_variable,
                self.recgen_seed_start_var,
                self.recgen_num_scenarios_var,
                self.recgen_num_vehicles_var,
                self.recgen_num_walkers_var,
                self.recgen_filter_vehicles_var,
                self.recgen_generation_vehicles_var,
                self.recgen_filter_walkers_var,
                self.recgen_generation_walkers_var,
                self.recgen_length_minutes_var,
                self.recgen_output_dir_variable,
                self.recgen_num_parked_var,
        ):
            variable.trace_add("write", self._auto_save)

        for var in self.recgen_tab.map_vars.values():
            var.trace_add("write", self._auto_save)

        self.recgen_num_parked_var.trace_add("write", lambda *_: self.recgen_tab.refresh_map_filters())

    def _auto_save(self, *_):
        """Skips saving while numeric fields are in an in-progress state."""
        transient = {"", "-", ".", "-."}
        if (self.begin_at_variable.get() in transient or
                self.end_at_variable.get() in transient):
            return
        try:
            self.collect_cfg()
        except Exception:
            pass

    def _on_close(self):
        try:
            self.stop_worker()
        finally:
            try:
                self.destroy()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Path validation / dialogs
    # ------------------------------------------------------------------

    @staticmethod
    def normalize_extension(ext: str) -> str:
        """Normalizes a given file extension by ensuring it starts with a period."""
        return ext if ext.startswith(".") else f".{ext}"

    def validate_paths(self, specs: list[tuple[str, tk.Variable, str]]) -> bool:
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

            if "file" in kind and "dir" in kind:
                if not (os.path.isfile(value) or os.path.isdir(value)):
                    messagebox.showerror("Invalid path", f"{label} does not exist as a file/folder:\n{value}")
                    return False

            if kind == "file":
                if not os.path.isfile(value):
                    messagebox.showerror("Invalid path", f"{label} does not exist as a file:\n{value}")
                    return False
            elif kind == "dir":
                if not os.path.isdir(value):
                    messagebox.showerror("Invalid path", f"{label} does not exist as a folder:\n{value}")
                    return False
            else:
                if not os.path.exists(value):
                    messagebox.showerror("Invalid path", f"{label} path does not exist:\n{value}")
                    return False
        return True

    def open_file_dialog(self, variable: tk.StringVar):
        selected_file = filedialog.askopenfilename()
        variable.set(selected_file or variable.get())

    def open_file_selection_with_specified_extension(self, variable: tk.StringVar):
        extension = self.normalize_extension(self.recording_extension_variable.get())
        selected_file = filedialog.askopenfilename(filetypes=[(f"{extension} files", f"*{extension}"),
                                                              ("All files", "*.*")])
        variable.set(selected_file or variable.get())

    def open_directory_dialog(self, variable: tk.StringVar):
        selected_directory = filedialog.askdirectory()
        variable.set(selected_directory or variable.get())

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _redirect_console(self):
        class _TextOutputHandler:
            def __init__(self, app): self.app = app

            def write(self, txt):
                for lines in txt.rstrip().splitlines():
                    self.app.log(lines)

            def flush(self): pass

        sys.stdout = sys.stderr = _TextOutputHandler(self)

    def log(self, txt: str):
        """
        Logs a given text message to the GUI text widget, the log file, and the real
        terminal. Called from arbitrary threads (every worker's subprocess-output
        loop, plus anything using the redirected sys.stdout/stderr), so it must
        never touch self.log_widget directly - see the comment on self._log_queue
        in __init__ for why. The actual widget update happens later, on the main
        thread, via _drain_log_queue.
        """
        # sys.__stdout__ is the original stdout Python captured at startup and
        # never reassigned by anything - unlike sys.stdout, which
        # _redirect_console() points at the GUI widget instead. Writing here
        # lets output (including from worker subprocesses, which all funnel
        # through this method as their log callback) survive in the terminal
        # even if the GUI window closes right after a crash.
        try:
            print(txt, file=sys.__stdout__, flush=True)
        except Exception:
            pass

        self._log_queue.put(txt)

        log_path = getattr(self, "_log_file_path", None)
        if log_path:
            try:
                with open(log_path, "a", encoding="utf-8") as f:
                    f.write(txt + "\n")
            except Exception:
                pass

    def _drain_log_queue(self):
        """
        Runs on the main thread only (scheduled via after()) and is the sole
        place allowed to touch self.log_widget, so background threads calling
        log() never race Tk's own event processing.
        """
        pending = []
        try:
            while True:
                pending.append(self._log_queue.get_nowait())
        except queue.Empty:
            pass

        if pending:
            self.log_widget.configure(state="normal")
            for txt in pending:
                self.log_widget.insert("end", txt + "\n")
            self.log_widget.see("end")
            self.log_widget.configure(state="disabled")

        self.after(50, self._drain_log_queue)

    def clear_log(self):
        self.log_widget.configure(state="normal")
        self.log_widget.delete("1.0", "end")
        self.log_widget.configure(state="disabled")

    def _init_log_file(self):
        """Decide where to write the session log and create the folder if needed."""
        logs_dir = os.path.join(self.config.transformer_output_path, "logs")
        try:
            os.makedirs(logs_dir, exist_ok=True)
        except Exception:
            logs_dir = os.getcwd()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_file_path = os.path.join(logs_dir, f"carla_gui_{timestamp}.log")

        try:
            with open(self._log_file_path, "a", encoding="utf-8") as f:
                f.write(f"=== CARLA GUI log started {datetime.now().isoformat()} ===\n")
        except Exception:
            pass


if __name__ == "__main__":
    CarlaInteractionGUI().mainloop()
