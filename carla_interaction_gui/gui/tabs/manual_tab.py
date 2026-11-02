from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QHBoxLayout, QWidget

from carla_interaction_gui.gui.binding import set_combo_items
from carla_interaction_gui.gui.widgets import (
    Tab, button, check, combo, double_spin, hbox, line_edit, path_field, primary_button, spin,
)
from carla_interaction_gui.workers.DisableTrafficManagerWorker import DisableTrafficManagerWorker
from carla_interaction_gui.workers.ManualControlWorker import ManualControlWorker
from carla_interaction_gui.workers.MoveLatestRecordingWorker import MoveLatestRecordingWorker
from carla_interaction_gui.workers.SpawnTrafficWorker import SpawnTrafficWorker
from carla_interaction_gui.workers.SteeringWheelControlWorker import SteeringWheelControlWorker


class ManualTab(Tab):
    """
    The "Manual Drive" tab: drive around in CARLA manually, spawn extra
    manually-controlled actors, and archive the resulting recording.
    """

    # "Start manual driving" restarts the CARLA server and loads the map
    # before it launches the pygame client, which then spawns the ego vehicle.
    # Traffic spawned any earlier would be wiped out by that restart/map load,
    # so it starts once the client is launched and then waits (up to this
    # long) for the ego vehicle to appear.
    _WAIT_FOR_EGO_VEHICLE_S = 120

    def __init__(self, app):
        super().__init__(app, "Drive around in CARLA manually, add further manually controlled actors and "
                              "archive the resulting recording.")
        settings = app.settings

        form = self.section("Driving")
        map_combo = combo(settings["selected_map"])
        ego_combo = combo(settings["selected_ego_vehicle"])
        traffic_type_combo = combo(settings["manual_spawn_traffic_vehicle_type"])

        def follow_version():
            set_combo_items(map_combo, app.available_maps(), settings["selected_map"])
            for c, name in ((ego_combo, "selected_ego_vehicle"),
                            (traffic_type_combo, "manual_spawn_traffic_vehicle_type")):
                set_combo_items(c, app.available_vehicles(), settings[name])

        app.register_version_listener(follow_version)
        form.addRow("Map:", map_combo)
        form.addRow("Ego vehicle:", ego_combo)
        form.addRow("Camera elevation:", double_spin(settings["camera_elevation"], -50, 50, step=0.1, suffix=" m"))
        form.addRow("Camera tilt:", double_spin(settings["camera_tilt"], -90, 90, step=1, suffix="°"))

        form = self.section("Traffic")
        spawn_enabled = check("Spawn traffic", settings["manual_spawn_traffic_enabled"])
        count = spin(settings["manual_spawn_traffic_num_vehicles"], 1, 10_000, " vehicles")
        form.addRow(hbox(spawn_enabled, count, QWidget(), stretch_last=True))
        single_type = check("Spawn only one vehicle type:", settings["manual_spawn_traffic_single_type_enabled"])
        form.addRow(hbox(single_type, traffic_type_combo, stretch_last=True))

        def update_traffic_enabled():
            count.setEnabled(spawn_enabled.isChecked())
            single_type.setEnabled(spawn_enabled.isChecked())
            traffic_type_combo.setEnabled(spawn_enabled.isChecked() and single_type.isChecked())

        update_traffic_enabled()
        spawn_enabled.toggled.connect(update_traffic_enabled)
        single_type.toggled.connect(update_traffic_enabled)
        form.addRow(check("Disable traffic manager for other vehicles (they will not move)",
                          settings["manual_disable_traffic_manager_enabled"]))

        form = self.section("Recording")
        form.addRow("Recording extension:", line_edit(settings["recording_extension"], ".rec"))
        form.addRow("CARLA output folder:", path_field(app, settings["manual_output_dir"], folder=True))
        form.addRow("Archive recordings folder:", path_field(app, settings["default_recordings_folder"], folder=True))
        form.addRow("New file-name prefix:", hbox(line_edit(settings["new_file_name"]),
                                                  button("Move 'manual_recording'", self._move_latest)))

        form = self.section("Add controlled actor")
        self._actor_buttons = QWidget()
        actor_row = QHBoxLayout(self._actor_buttons)
        actor_row.setContentsMargins(0, 0, 0, 0)

        def show_actor_buttons():
            while actor_row.count():
                old = actor_row.takeAt(0).widget()
                old.setParent(None)  # off screen right away, not only once deleteLater runs
                old.deleteLater()
            for label, filter_str in app.available_controlled_actors().items():
                actor_row.addWidget(button(label, lambda _=False, f=filter_str: self._spawn_manual_extra(
                    filter_str=f)))

        app.register_version_listener(show_actor_buttons)
        form.addRow("Into the running session:", hbox(self._actor_buttons, QWidget(), stretch_last=True))

        self.stop_btn = self.actions(
            primary_button("Start manual driving", self._start_manual),
            button("Start manual driving (steering wheel)", self._start_manual_steering_wheel),
        )

    def _start_primary(self, worker_type):
        """Start the primary manual driving (exclusive) as the selected ego vehicle."""
        app = self.app
        if not app.validate_paths([("CARLA executable", "carla_executable", "carla")]):
            return
        app.clear_log()

        cfg = app.collect_cfg()
        w = worker_type(
            cfg,
            app.log,
            vehicle_filter=cfg.selected_ego_vehicle,
            camera_elevation=cfg.camera_elevation,
            camera_tilt=cfg.camera_tilt,
            restart_before=True,
            kill_server_after=True,
            exclusive=True,
        )
        app.manual_workers.append(w)
        app.attach_worker(w, stop_button=self.stop_btn)
        self._schedule_post_start_actions(w, cfg.manual_spawn_traffic_num_vehicles
                                          if cfg.manual_spawn_traffic_enabled else 0)

    def _start_manual(self):
        self._start_primary(ManualControlWorker)

    def _start_manual_steering_wheel(self):
        """Like _start_manual, reading input from a steering wheel/joystick."""
        self._start_primary(SteeringWheelControlWorker)

    def _spawn_manual_extra(self, *, filter_str: str):
        """
        Launch another manual_control_keyboard.py instance with --rolename=manual_control and
        the provided --filter, without rebooting/killing the CARLA server.
        """
        app = self.app
        if not app.validate_paths([("CARLA executable", "carla_executable", "carla")]):
            return

        cfg = app.collect_cfg()
        w = ManualControlWorker(
            cfg,
            app.log,
            vehicle_filter=filter_str,
            role_name="manual_control",
            camera_elevation=cfg.camera_elevation,
            camera_tilt=cfg.camera_tilt,
            restart_before=False,
            kill_server_after=False,
            exclusive=False,
        )
        app.manual_workers.append(w)
        app.attach_worker(w)

    def _schedule_post_start_actions(self, manual_worker, number_of_vehicles: int):
        """
        Once the just-(re)started CARLA session has launched its driving
        client, optionally spawns a batch of random autopilot vehicles (after
        the ego vehicle appeared) and/or disables the Traffic Manager for all
        non-ego vehicles so they sit still. Skipped if there's nothing to do,
        or if the session is stopped before its client launched.
        """
        disable_traffic_manager = self.app.settings["manual_disable_traffic_manager_enabled"].get()
        if number_of_vehicles <= 0 and not disable_traffic_manager:
            return

        def fire():
            if not (manual_worker.is_alive() and manual_worker in self.app.manual_workers):
                return
            if not manual_worker.client_started.is_set():
                QTimer.singleShot(500, fire)
                return
            if number_of_vehicles > 0:
                spawn_worker = self._spawn_traffic_now(number_of_vehicles)
                if disable_traffic_manager:
                    self._wait_then_disable_traffic_manager(spawn_worker)
            elif disable_traffic_manager:
                self._disable_traffic_manager_now()

        fire()

    def _spawn_traffic_now(self, number_of_vehicles: int) -> SpawnTrafficWorker:
        app = self.app
        cfg = app.collect_cfg()
        filter_vehicles = cfg.manual_spawn_traffic_vehicle_type if cfg.manual_spawn_traffic_single_type_enabled \
            else None
        w = SpawnTrafficWorker(cfg, app.log, number_of_vehicles=number_of_vehicles, filter_vehicles=filter_vehicles,
                               wait_for_hero=self._WAIT_FOR_EGO_VEHICLE_S)
        app.manual_workers.append(w)
        app.attach_worker(w)
        return w

    def _wait_then_disable_traffic_manager(self, spawn_worker: SpawnTrafficWorker):
        """Waits for the just-launched spawn-traffic job to finish before disabling the Traffic Manager."""

        def poll():
            if spawn_worker.is_alive():
                QTimer.singleShot(500, poll)
            else:
                self._disable_traffic_manager_now()

        poll()

    def _disable_traffic_manager_now(self):
        app = self.app
        w = DisableTrafficManagerWorker(app.collect_cfg(), app.log)
        app.manual_workers.append(w)
        app.attach_worker(w)

    def _move_latest(self):
        """Moves the latest recording with the specified prefix."""
        app = self.app
        if not app.settings["new_file_name"].get().strip():
            return app.show_error("Missing", "File-name prefix required.")
        if not app.validate_paths([("Archive recordings folder", "default_recordings_folder", "dir")]):
            return None
        cfg = app.collect_cfg()
        app.attach_worker(MoveLatestRecordingWorker(cfg, cfg.new_file_name, app.log))
        return None
