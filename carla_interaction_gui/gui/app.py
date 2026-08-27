from __future__ import annotations

import os
import sys
import tkinter as tk
from datetime import datetime
from tkinter import ttk, filedialog, messagebox, scrolledtext

from carla_interaction_gui.carla_launcher import kill_carla
from carla_interaction_gui.config_data import Config, load, save
from carla_interaction_gui.gui.constants import ALLOWED_CARLA_MAPS
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
        self.geometry("950x800")
        self.resizable(True, True)

        self._current_stop_button: tk.Button | None = None
        self._stop_buttons: list[tk.Button] = []

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
        config.transform_input_file = self.transform_input_file_variable.get().strip()
        config.transformer_output_path = self.transformer_output_path_variable.get().strip()
        config.video_input_file = self.video_input_path_variable.get().strip()
        config.video_output_path = self.video_output_path_variable.get().strip()
        config.with_bboxes = self.with_bboxes_variable.get()
        config.video_width = self.video_width_variable.get()
        config.video_height = self.video_height_variable.get()
        config.vehicle_id = self.vehicle_id_variable.get()

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
        """Logs a given text message to the GUI text widget and also appends it to a file."""
        self.log_widget.configure(state="normal")
        self.log_widget.insert("end", txt + "\n")
        self.log_widget.see("end")
        self.log_widget.configure(state="disabled")

        log_path = getattr(self, "_log_file_path", None)
        if log_path:
            try:
                with open(log_path, "a", encoding="utf-8") as f:
                    f.write(txt + "\n")
            except Exception:
                pass

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
