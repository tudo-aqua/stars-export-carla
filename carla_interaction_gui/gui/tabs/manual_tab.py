from __future__ import annotations

import tkinter as tk
from tkinter import ttk, messagebox

from carla_interaction_gui.gui.constants import ALLOWED_CARLA_MAPS
from carla_interaction_gui.gui.widgets import entry_row
from carla_interaction_gui.workers.ManualControlWorker import ManualControlWorker
from carla_interaction_gui.workers.MoveLatestRecordingWorker import MoveLatestRecordingWorker
from carla_interaction_gui.workers.SpawnTrafficWorker import SpawnTrafficWorker
from carla_interaction_gui.workers.SteeringWheelControlWorker import SteeringWheelControlWorker


class ManualTab(ttk.Frame):
    """
    The "Manual Drive" tab: drive around in CARLA manually, spawn extra
    manually-controlled actors, and archive the resulting recording.
    """

    # "Start manual driving" restarts the CARLA server before it even
    # launches the pygame client (see ManualControlWorker -> restart_carla in
    # carla_launcher.py): kill_carla, a 5s cooldown, then the new process
    # gets a 20s boot wait before its map gets (re)loaded - all of that runs
    # before pygame is even started, let alone connected and the ego vehicle
    # spawned. Spawning traffic any earlier than that just has it wiped out
    # again the moment the map load finishes. Keep this in sync with
    # restart_carla's cooldown/boot defaults if those ever change.
    _CARLA_COOLDOWN_S = 5
    _CARLA_BOOT_S = 20
    _MAP_LOAD_AND_EGO_SPAWN_BUFFER_S = 20
    _SPAWN_TRAFFIC_DELAY_MS = (_CARLA_COOLDOWN_S + _CARLA_BOOT_S + _MAP_LOAD_AND_EGO_SPAWN_BUFFER_S) * 1000

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

        options = ttk.LabelFrame(self, text="Options")
        options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(options, text="Render off screen",
                       variable=app.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(options, text="Render quality low",
                       variable=app.render_quality_low_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        spawn_traffic_row = tk.Frame(options)
        spawn_traffic_row.pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(spawn_traffic_row, text="Spawn traffic",
                       variable=app.manual_spawn_traffic_enabled_variable, anchor="w").pack(side="left")
        tk.Label(spawn_traffic_row, text="Number of vehicles:").pack(side="left", padx=(12, 4))
        tk.Entry(spawn_traffic_row, textvariable=app.manual_spawn_traffic_num_vehicles_variable,
                 width=8).pack(side="left")

        tk.Button(self, text="Start manual driving", width=25, command=self._start_manual).pack(pady=8)
        tk.Button(self, text="Start manual driving (Steering Wheel)",
                  command=self._start_manual_steering_wheel).pack(pady=2)

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
            # ("CARLA output folder", app.manual_output_dir_variable, "dir"),
        ]):
            return
        number_of_vehicles = self._resolve_spawn_traffic_vehicle_count()
        if number_of_vehicles is None:
            return
        app.clear_log()

        w = ManualControlWorker(
            app.collect_cfg(),
            app.log,
            restart_before=True,
            kill_server_after=True,
            exclusive=True,
        )
        app.manual_workers.append(w)
        app.attach_worker(w, stop_button=self.stop_btn)
        self._schedule_spawn_traffic(w, number_of_vehicles)

    def _start_manual_steering_wheel(self):
        """Start the primary manual driving (exclusive), reading input from a steering wheel/joystick."""
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", app.carla_executable_variable, "file"),
        ]):
            return
        number_of_vehicles = self._resolve_spawn_traffic_vehicle_count()
        if number_of_vehicles is None:
            return
        app.clear_log()

        w = SteeringWheelControlWorker(
            app.collect_cfg(),
            app.log,
            restart_before=True,
            kill_server_after=True,
            exclusive=True,
        )
        app.manual_workers.append(w)
        app.attach_worker(w, stop_button=self.stop_btn)
        self._schedule_spawn_traffic(w, number_of_vehicles)

    def _spawn_manual_extra(self, *, filter_str: str):
        """
        Launch another manual_control_keyboard.py instance with --rolename=manual_control and
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

    def _resolve_spawn_traffic_vehicle_count(self) -> int | None:
        """
        Returns the configured vehicle count if the "Spawn traffic" checkbox
        is enabled, 0 if it's disabled, or None (after showing an error) if
        the configured count is invalid.
        """
        if not self.app.manual_spawn_traffic_enabled_variable.get():
            return 0
        try:
            return max(1, int(self.app.manual_spawn_traffic_num_vehicles_variable.get()))
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid value", "Number of vehicles to spawn must be a whole number.")
            return None

    def _schedule_spawn_traffic(self, manual_worker, number_of_vehicles: int):
        """
        Spawns a batch of random autopilot vehicles into the just-started
        CARLA session after a delay, giving it time to restart, load the
        map, and connect before traffic gets injected. Skipped if the
        session was stopped again before the delay elapsed.
        """
        if number_of_vehicles <= 0:
            return

        def fire():
            if manual_worker.is_alive() and manual_worker in self.app.manual_workers:
                self._spawn_traffic_now(number_of_vehicles)

        self.app.after(self._SPAWN_TRAFFIC_DELAY_MS, fire)

    def _spawn_traffic_now(self, number_of_vehicles: int):
        app = self.app
        w = SpawnTrafficWorker(app.collect_cfg(), app.log, number_of_vehicles=number_of_vehicles)
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
