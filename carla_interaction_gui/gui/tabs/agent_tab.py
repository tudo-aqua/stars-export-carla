from __future__ import annotations

import os
import sys
import tkinter as tk
from tkinter import ttk

from carla_interaction_gui.gui.constants import ALLOWED_CARLA_MAPS
from carla_interaction_gui.gui.widgets import entry_row
from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class _AgentRunner(ThreadWorker):
    RUNNER = "carla_task_runner.py"

    def run(self):
        runner = self._resolve_runner()
        if not runner:
            self.log("!! Could not locate carla_task_runner.py")
            return

        cfg = self.cfg
        cmd = [sys.executable or "python", runner, "manual_agent", "--carla-exe", cfg.carla_executable]

        m = (getattr(cfg, "selected_map", "") or "").strip()
        if m:
            cmd += ["--map-name", m]
        if getattr(cfg, "render_quality_low", False):
            cmd.append("--quality-low")
        if getattr(cfg, "render_off_screen", False):
            cmd.append("--offscreen")

        cmd += ["--res", "1280x720", "--sync"]

        os.environ["AGENT_TARGET_KPH"] = str(getattr(cfg, "agent_target_speed_kph", 35.0))
        os.environ["AGENT_VEHICLE_FILTER"] = getattr(cfg, "agent_vehicle_filter", "vehicle.*")

        self._start_and_stream(cmd)


class AgentTab(ttk.Frame):
    """The "Agent Drive" tab: run a Python agent controlling a single ego car."""

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="Agent Drive")
        tk.Label(self,
                 text="Start a Python Agent controlling a single ego car (press 'P' in the viewer to toggle).").pack(
            pady=5)

        entry_row(self, "CARLA executable:", app.carla_executable_variable,
                 lambda: app.open_file_dialog(app.carla_executable_variable))

        row = tk.Frame(self)
        row.pack(fill="x", pady=2)
        tk.Label(row, text="Map:", width=26, anchor="w").pack(side="left")
        ttk.Combobox(row, textvariable=app.selected_map_variable,
                     state="readonly", values=ALLOWED_CARLA_MAPS, width=42).pack(side="left", fill="x", expand=True)

        agent_opts = ttk.LabelFrame(self, text="Agent parameters")
        agent_opts.pack(fill="x", padx=4, pady=6)
        entry_row(agent_opts, "Vehicle filter:", app.agent_vehicle_filter_variable)
        entry_row(agent_opts, "Target speed (kph):", app.agent_target_speed_variable, width=10)

        rendering_options = ttk.LabelFrame(self, text="Rendering options")
        rendering_options.pack(fill="x", padx=4, pady=6)
        tk.Checkbutton(rendering_options, text="Render off screen",
                       variable=app.render_off_screen_variable, anchor="w").pack(fill="x", padx=6, pady=2)
        tk.Checkbutton(rendering_options, text="Render quality low",
                       variable=app.render_quality_low_variable, anchor="w").pack(fill="x", padx=6, pady=2)

        row_btns = tk.Frame(self)
        row_btns.pack(fill="x", pady=6)
        tk.Button(row_btns, text="Start Agent", width=20, command=self._start_manual_agent).pack(side="left", padx=2)

        self.stop_btn = tk.Button(self, text="Stop", command=app.stop_worker, state="disabled")
        self.stop_btn.pack(pady=8)
        app.register_stop_button(self.stop_btn)

    def _start_manual_agent(self):
        app = self.app
        if not app.validate_paths([("CARLA executable", app.carla_executable_variable, "file")]):
            return
        app.clear_log()
        cfg = app.collect_cfg()
        app.attach_worker(_AgentRunner(cfg, app.log), stop_button=self.stop_btn)
