from __future__ import annotations

import tkinter as tk
from tkinter import ttk, messagebox

from carla_interaction_gui.gui.constants import ALL_CARLA_MAPS, ALLOWED_NON_LAYERED_MAPS, ALLOWED_PARKED_MAPS
from carla_interaction_gui.gui.widgets import carla_target_rows, entry_row
from carla_interaction_gui.workers.RecGenRunner import RecGenRunner


class RecGenTab(ttk.Frame):
    """
    The "Recording Generator" tab: generate CARLA recordings across seed
    ranges and selected maps.
    """

    def __init__(self, notebook: ttk.Notebook, app):
        super().__init__(notebook)
        self.app = app
        notebook.add(self, text="Recording Generator")

        tk.Label(self, text="Generate CARLA recordings across seed ranges and selected maps.").pack(pady=5)

        carla_target_rows(self, app)

        render_row = tk.Frame(self)
        render_row.pack(fill="x", padx=4, pady=(2, 8))
        tk.Checkbutton(
            render_row, text="Render off screen", variable=app.render_off_screen_variable
        ).pack(side="left", padx=4)

        seed_frame = ttk.LabelFrame(self, text="Seeds")
        seed_frame.pack(fill="x", padx=4, pady=6)
        entry_row(seed_frame, "Seed start:", app.recgen_seed_start_var, width=12)
        entry_row(seed_frame, "# scenarios:", app.recgen_num_scenarios_var, width=12)

        maps_frame = ttk.LabelFrame(self, text="Maps (select one or more)")
        maps_frame.pack(fill="x", padx=4, pady=6)

        # One var per known map, so selections survive switching CARLA
        # versions; only the maps of the current version get a checkbox.
        saved_maps = getattr(app.config, "recgen_selected_maps", None)
        previously = set(saved_maps) if saved_maps else set(ALLOWED_NON_LAYERED_MAPS)
        self.map_vars: dict[str, tk.BooleanVar] = {
            m: tk.BooleanVar(value=m in previously) for m in ALL_CARLA_MAPS
        }
        self._maps_frame = maps_frame
        app.register_version_listener(self._show_maps)

        tp = ttk.LabelFrame(self, text="Traffic parameters")
        tp.pack(fill="x", padx=4, pady=6)
        entry_row(tp, "Vehicles (N):", app.recgen_num_vehicles_var, width=12)
        entry_row(tp, "Walkers (W):", app.recgen_num_walkers_var, width=12)
        entry_row(tp, "Vehicle filter:", app.recgen_filter_vehicles_var)
        entry_row(tp, "Vehicle generation:", app.recgen_generation_vehicles_var, width=12)
        entry_row(tp, "Walker filter:", app.recgen_filter_walkers_var)
        entry_row(tp, "Walker generation:", app.recgen_generation_walkers_var, width=12)
        entry_row(tp, "Parked vehicles (N):", app.recgen_num_parked_var, width=12)

        rs = ttk.LabelFrame(self, text="Run settings")
        rs.pack(fill="x", padx=4, pady=6)
        entry_row(rs, "Length per run (min):", app.recgen_length_minutes_var, width=12)

        entry_row(
            self, "Output directory:", app.recgen_output_dir_variable,
            lambda: app.open_directory_dialog(app.recgen_output_dir_variable)
        )

        row_btns = tk.Frame(self)
        row_btns.pack(fill="x", pady=8)
        tk.Button(row_btns, text="Run generator", width=18, command=self._start_recgen).pack(side="left", padx=2)
        self.stop_btn = tk.Button(row_btns, text="Stop", width=8, command=app.stop_worker, state="disabled")
        self.stop_btn.pack(side="left", padx=4)
        app.register_stop_button(self.stop_btn)

    def _show_maps(self):
        """(Re)build the map checkboxes for the maps of the current CARLA version."""
        maps = self._available_maps = self.app.available_maps()
        for child in self._maps_frame.winfo_children():
            child.destroy()
        for i, m in enumerate(maps):
            if i % 4 == 0:
                row = tk.Frame(self._maps_frame)
                row.pack(fill="x", pady=2)
            tk.Checkbutton(row, text=m, variable=self.map_vars[m], anchor="w").pack(side="left", padx=6)

    def _start_recgen(self):
        """
        Start the recording generator across a seed range.
        The generator receives all selected maps and picks one deterministically using the seed.
        """
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", app.carla_executable_variable, "carla"),
            ("Output directory", app.recgen_output_dir_variable, "dir"),
        ]):
            return

        cfg = app.collect_cfg()

        selected_maps = [m for m in self._available_maps if self.map_vars[m].get()]
        if not selected_maps:
            return messagebox.showerror("Missing", "Select at least one map.")

        parked = max(0, int(app.recgen_num_parked_var.get()))
        if parked > 0:
            selected_maps = [m for m in selected_maps if m in ALLOWED_PARKED_MAPS]
            if not selected_maps:
                return messagebox.showerror("No valid maps",
                                            "With parked vehicles > 0, select maps from ALLOWED_PARKED_MAPS only.")

        app.clear_log()
        runner = RecGenRunner(cfg, app.log, selected_maps=selected_maps)
        app.attach_worker(runner, stop_button=self.stop_btn)
        self.refresh_map_filters()

    def refresh_map_filters(self):
        """Disable/uncheck maps that can't spawn parked vehicles if parked>0."""
        try:
            parked = int(self.app.recgen_num_parked_var.get())
        except Exception:
            parked = 0

        for m, var in self.map_vars.items():
            allowed = (parked <= 0) or (m in ALLOWED_PARKED_MAPS)
            if not allowed and var.get():
                var.set(False)
