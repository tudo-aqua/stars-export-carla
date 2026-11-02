from __future__ import annotations

import gc
import os
import sys
from datetime import datetime
from typing import Callable

from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QFontDatabase, QTextCursor
from PySide6.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
    QSplitter, QTabWidget, QVBoxLayout, QWidget,
)

from carla_interaction_gui.carla_launcher import kill_carla
from carla_interaction_gui.carla_versions import (
    controlled_actors_for, is_start_script, maps_for, python_api_for, python_api_versions, validate_python_api,
    validate_target, vehicles_for,
)
from carla_interaction_gui.config_data import Config, load, save
from carla_interaction_gui.gui.binding import Settings, Value
from carla_interaction_gui.gui.constants import ALLOWED_NON_LAYERED_MAPS, CARLA_DEFAULT_DOCKER_IMAGE
from carla_interaction_gui.gui.tabs.manual_tab import ManualTab
from carla_interaction_gui.gui.tabs.maps_tab import MapsTab
from carla_interaction_gui.gui.tabs.recgen_tab import RecGenTab
from carla_interaction_gui.gui.tabs.server_tab import ServerTab
from carla_interaction_gui.gui.tabs.transform_tab import TransformTab
from carla_interaction_gui.gui.tabs.video_tab import VideoTab
from carla_interaction_gui.gui.widgets import carla_panel
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker

STYLESHEET = """
QLabel#tabDescription { color: palette(placeholder-text); }
QGroupBox { font-weight: 600; margin-top: 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QGroupBox > QWidget { font-weight: normal; }
QPushButton#primaryButton { font-weight: 600; padding: 6px 14px; }
"""


class _LogSignal(QObject):
    """Carries log lines from any thread to the GUI thread (queued by Qt)."""
    line = Signal(str)


class CarlaInteractionGUI(QMainWindow):
    """
    Main GUI application window for interaction with the CARLA Simulator.

    Owns the persisted configuration (as observable Settings that the tabs'
    widgets bind to) and the cross-cutting services (background-worker
    lifecycle, logging, path validation, file dialogs) that every tab relies
    on. Each tab's own UI and click handlers live in
    carla_interaction_gui.gui.tabs and are handed a reference to this app to
    reach those shared services.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("CARLA interaction GUI")
        self.resize(1200, 950)

        self._stop_buttons: list[QPushButton] = []
        self._current_stop_button: QPushButton | None = None
        self._active_worker: ThreadWorker | None = None
        self.carla_worker: ThreadWorker | None = None
        self.manual_workers: list[ThreadWorker] = []
        # Every worker ever started, so closing can wait for all of them.
        self._all_workers: list[ThreadWorker] = []
        self._closing = False
        self._version_listeners: list[Callable[[], None]] = []

        self.settings = Settings(self._normalized(load()), self)
        self.settings["carla_executable"].changed.connect(self._on_carla_executable_changed)

        # Debounced so typing into a field doesn't rewrite the config file per keystroke.
        self._save_timer = QTimer(self, singleShot=True, interval=400)
        self._save_timer.timeout.connect(lambda: save(self.collect_cfg()))
        self.settings.changed.connect(self._save_timer.start)

        self._log_signal = _LogSignal(self)
        self._log_signal.line.connect(self._append_log, Qt.ConnectionType.QueuedConnection)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(carla_panel(self))

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.server_tab = ServerTab(self)
        self.manual_tab = ManualTab(self)
        self.transform_tab = TransformTab(self)
        self.video_tab = VideoTab(self)
        self.maps_tab = MapsTab(self)
        self.recgen_tab = RecGenTab(self)
        for tab, title in ((self.server_tab, "CARLA Server"), (self.manual_tab, "Manual Drive"),
                           (self.transform_tab, "Transform"), (self.video_tab, "Record ➜ MP4"),
                           (self.maps_tab, "Generate Maps"), (self.recgen_tab, "Recording Generator")):
            self.tabs.addTab(tab, title)

        self.log_widget = QPlainTextEdit(readOnly=True)
        self.log_widget.setMaximumBlockCount(20_000)
        self.log_widget.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self.log_widget.setPlaceholderText("Task output appears here.")

        log_pane = QWidget()
        log_layout = QVBoxLayout(log_pane)
        log_layout.setContentsMargins(0, 4, 0, 0)
        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("<b>Log</b>"))
        log_header.addStretch(1)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.clear_log)
        log_header.addWidget(clear)
        log_layout.addLayout(log_header)
        log_layout.addWidget(self.log_widget)

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        splitter.addWidget(self.tabs)
        splitter.addWidget(log_pane)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter, 1)
        self.setCentralWidget(central)

        self._init_log_file()
        self._redirect_console()

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    @staticmethod
    def _normalized(config: Config) -> Config:
        """Replace saved choices that don't fit the configured CARLA version with its defaults."""
        # A saved start script that's gone (e.g. the former carla_run.sh) falls
        # back to the default docker image.
        if not config.carla_executable or (
                is_start_script(config.carla_executable) and not os.path.isfile(config.carla_executable)):
            config.carla_executable = CARLA_DEFAULT_DOCKER_IMAGE
        config.carla_python_api = python_api_for(config.carla_executable) or config.carla_python_api
        if config.carla_python_api not in python_api_versions():
            config.carla_python_api = python_api_versions()[0]

        maps = maps_for(config.carla_executable)
        if config.selected_map not in maps:
            config.selected_map = maps[0]
        vehicles = vehicles_for(config.carla_executable)
        if config.selected_ego_vehicle not in vehicles:
            config.selected_ego_vehicle = vehicles[0]
        if config.manual_spawn_traffic_vehicle_type not in vehicles:
            config.manual_spawn_traffic_vehicle_type = vehicles[0]
        if config.recgen_selected_maps is None:
            config.recgen_selected_maps = list(ALLOWED_NON_LAYERED_MAPS)
        if config.video_camera_positions is None:
            config.video_camera_positions = []
        return config

    def collect_cfg(self) -> Config:
        """Snapshot of the current settings, also written to disk."""
        config = self.settings.to_config()
        config.recording_extension = self.normalize_extension(config.recording_extension.strip())
        for field in ("carla_executable", "manual_output_dir", "default_recordings_folder", "new_file_name",
                      "transform_input_file", "transform_docker_mount_path", "transformer_output_path",
                      "video_input_file", "video_docker_mount_path", "video_output_path", "recgen_output_dir"):
            setattr(config, field, getattr(config, field).strip())
        save(config)
        return config

    @staticmethod
    def normalize_extension(ext: str) -> str:
        """Normalizes a given file extension by ensuring it starts with a period."""
        return ext if ext.startswith(".") else f".{ext}"

    # ------------------------------------------------------------------
    # CARLA version selection
    # ------------------------------------------------------------------

    def available_maps(self) -> list[str]:
        """Maps offered for the current CARLA executable (all known ones for free text)."""
        return maps_for(self.settings["carla_executable"].get())

    def available_vehicles(self) -> list[str]:
        """Vehicle blueprints offered for the current CARLA executable (all known ones for free text)."""
        return vehicles_for(self.settings["carla_executable"].get())

    def available_controlled_actors(self) -> dict[str, str]:
        """'Add controlled actor' buttons (label -> blueprint filter) for the current CARLA executable."""
        return controlled_actors_for(self.settings["carla_executable"].get())

    def register_version_listener(self, listener: Callable[[], None]) -> None:
        """Call `listener` now and whenever the CARLA executable changes (e.g. to refresh map choices)."""
        self._version_listeners.append(listener)
        listener()

    def _on_carla_executable_changed(self, executable: str):
        """Follow a catalogued image with its Python API, maps and vehicles."""
        python_api = python_api_for(executable)
        if python_api:
            self.settings["carla_python_api"].set(python_api)

        maps = self.available_maps()
        if self.settings["selected_map"].get() not in maps:
            self.settings["selected_map"].set(maps[0])
        vehicles = self.available_vehicles()
        for name in ("selected_ego_vehicle", "manual_spawn_traffic_vehicle_type"):
            if self.settings[name].get() not in vehicles:
                self.settings[name].set(vehicles[0])
        for listener in self._version_listeners:
            listener()

    # ------------------------------------------------------------------
    # Worker lifecycle
    # ------------------------------------------------------------------

    def register_stop_button(self, button: QPushButton) -> None:
        """Track a tab's stop button so a global stop can disable it."""
        self._stop_buttons.append(button)

    def attach_worker(self, worker: ThreadWorker, *, stop_button: QPushButton | None = None):
        """
        Starts a worker and manages its lifecycle in the UI: guards against
        starting a second exclusive worker while one is running, and
        re-enables the stop button once the worker finishes.
        """
        if worker.exclusive and self._active_worker:
            QMessageBox.warning(self, "Busy", "Another exclusive task is running.")
            return

        worker.start()
        self._all_workers.append(worker)

        if worker.exclusive:
            self._active_worker = worker
            if stop_button is not None:
                stop_button.setEnabled(True)
                self._current_stop_button = stop_button

        timer = QTimer(self, interval=500)

        def poll():
            if worker.is_alive():
                return
            timer.stop()
            timer.deleteLater()
            if worker is self._active_worker:
                self._active_worker = None
                if self._current_stop_button is not None:
                    self._current_stop_button.setEnabled(False)
                    self._current_stop_button = None

        timer.timeout.connect(poll)
        timer.start()

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
                w.join(timeout=5.0)
            except Exception:
                pass
            self._active_worker = None
            self._current_stop_button = None
            for button in self._stop_buttons:
                button.setEnabled(False)

        if self.carla_worker and self.carla_worker.is_alive():
            try:
                self.carla_worker.cancel()
                self.carla_worker.join(timeout=5.0)
            except Exception:
                pass
            self.server_tab.reset_server_button()

        kill_carla()

    def start_carla_worker(self, worker: ThreadWorker) -> None:
        """Start the CARLA server worker (not exclusive: other tasks may use its server)."""
        self.carla_worker = worker
        self._all_workers.append(worker)
        worker.start()

    def closeEvent(self, event):
        try:
            self._save_timer.stop()
            self.collect_cfg()
            self.stop_worker()
            # Cancelled workers still log while winding down (e.g. from their
            # finally blocks); let them finish before Qt objects go away, and
            # then stop routing output into the window, or a late log line
            # from a worker thread can crash the interpreter on exit.
            for worker in self._all_workers:
                worker.join(timeout=10)
            self._closing = True
            sys.stdout, sys.stderr = sys.__stdout__, sys.__stderr__
        finally:
            super().closeEvent(event)

    # ------------------------------------------------------------------
    # Validation / dialogs
    # ------------------------------------------------------------------

    def show_error(self, title: str, message: str) -> None:
        QMessageBox.critical(self, title, message)

    def validate_paths(self, specs: list[tuple[str, str, str | tuple[str, ...]]]) -> bool:
        """
        Validate that the given settings point to existing paths.

        specs: list of (label, setting name, kind) where kind in {"file","dir","any","carla"}
        or a tuple of "file"/"dir" accepting either. "carla" checks a CARLA
        executable (start script or docker image, see carla_versions) together
        with the selected CARLA Python API.
        Shows an error dialog and returns False on the first invalid item.
        """
        for label, name, kind in specs:
            value = str(self.settings[name].get()).strip()

            if kind == "carla":
                error = validate_target(value) or validate_python_api(self.settings["carla_python_api"].get())
                if error:
                    self.show_error("Invalid CARLA executable", error)
                    return False
                continue

            if not value:
                self.show_error("Missing", f"{label} required.")
                return False
            if "file" in kind and "dir" in kind:
                ok, what = os.path.isfile(value) or os.path.isdir(value), "file/folder"
            elif kind == "file":
                ok, what = os.path.isfile(value), "file"
            elif kind == "dir":
                ok, what = os.path.isdir(value), "folder"
            else:
                ok, what = os.path.exists(value), "path"
            if not ok:
                self.show_error("Invalid path", f"{label} does not exist as a {what}:\n{value}")
                return False
        return True

    def choose_file(self, value: Value, *, recording: bool = False):
        name_filter = ""
        if recording:
            extension = self.normalize_extension(self.settings["recording_extension"].get())
            name_filter = f"{extension} files (*{extension});;All files (*)"
        selected, _ = QFileDialog.getOpenFileName(self, "Select file", self._dialog_start(value), name_filter)
        if selected:
            value.set(selected)

    def choose_folder(self, value: Value):
        selected = QFileDialog.getExistingDirectory(self, "Select folder", self._dialog_start(value))
        if selected:
            value.set(selected)

    @staticmethod
    def _dialog_start(value: Value) -> str:
        current = str(value.get()).strip()
        return current if os.path.exists(current) else os.path.dirname(current)

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _redirect_console(self):
        class _TextOutputHandler:
            def __init__(self, app): self.app = app

            def write(self, txt):
                for line in txt.rstrip().splitlines():
                    self.app.log(line)

            def flush(self): pass

        sys.stdout = sys.stderr = _TextOutputHandler(self)

    def log(self, txt: str):
        """
        Logs a given text message to the log pane, the log file, and the real
        terminal. Safe to call from any thread (every worker's subprocess-output
        loop calls it): the pane is only updated on the GUI thread, via a
        queued signal.
        """
        # sys.__stdout__ is the original stdout, never redirected by
        # _redirect_console(), so output survives in the terminal even if the
        # GUI window closes right after a crash.
        try:
            print(txt, file=sys.__stdout__, flush=True)
        except Exception:
            pass

        if not self._closing:
            self._log_signal.line.emit(txt)

        log_path = getattr(self, "_log_file_path", None)
        if log_path:
            try:
                with open(log_path, "a", encoding="utf-8") as f:
                    f.write(txt + "\n")
            except Exception:
                pass

    def _append_log(self, txt: str):
        at_bottom = self.log_widget.verticalScrollBar().value() == self.log_widget.verticalScrollBar().maximum()
        self.log_widget.appendPlainText(txt)
        if at_bottom:
            self.log_widget.moveCursor(QTextCursor.MoveOperation.End)

    def clear_log(self):
        self.log_widget.clear()

    def _init_log_file(self):
        """Decide where to write the session log and create the folder if needed."""
        logs_dir = os.path.join(self.settings["transformer_output_path"].get(), "logs")
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


def run() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("CARLA interaction GUI")
    app.setStyle("Fusion")
    app.setStyleSheet(STYLESHEET)
    window = CarlaInteractionGUI()
    window.show()
    exit_code = app.exec()
    # Tear Qt objects down while the QApplication still exists; leaving them
    # to Python's final garbage collection crashed on exit ("munmap_chunk():
    # invalid pointer") once workers had run.
    window.deleteLater()
    app.processEvents()
    del window
    gc.collect()
    return exit_code


if __name__ == "__main__":
    sys.exit(run())
