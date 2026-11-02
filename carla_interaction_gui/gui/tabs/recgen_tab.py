from __future__ import annotations

from PySide6.QtWidgets import QCheckBox, QGridLayout, QGroupBox

from carla_interaction_gui.gui.constants import ALLOWED_PARKED_MAPS
from carla_interaction_gui.gui.widgets import Tab, combo, double_spin, line_edit, path_field, primary_button, spin
from carla_interaction_gui.workers.RecGenRunner import RecGenRunner

_GENERATIONS = ["1", "2", "All"]


class RecGenTab(Tab):
    """
    The "Recording Generator" tab: generate CARLA recordings across seed
    ranges and selected maps.
    """

    _MAP_COLUMNS = 4

    def __init__(self, app):
        super().__init__(app, "Generate CARLA recordings across a seed range; each seed picks one of the "
                              "selected maps.")
        settings = app.settings

        form = self.section("Seeds")
        form.addRow("Seed start:", spin(settings["recgen_seed_start"], 0, 2_000_000_000))
        form.addRow("Number of scenarios:", spin(settings["recgen_num_scenarios"], 1, 1_000_000))

        maps_group = QGroupBox("Maps")
        self._maps_grid = QGridLayout(maps_group)
        self._map_checks: dict[str, QCheckBox] = {}
        self.body.addWidget(maps_group)

        form = self.section("Traffic")
        form.addRow("Vehicles:", spin(settings["recgen_num_vehicles"], 0, 10_000))
        form.addRow("Walkers:", spin(settings["recgen_num_walkers"], 0, 10_000))
        parked = spin(settings["recgen_num_parked"], 0, 10_000)
        parked.setToolTip(f"Only possible on: {', '.join(ALLOWED_PARKED_MAPS)}")
        form.addRow("Parked vehicles:", parked)
        form.addRow("Vehicle filter:", line_edit(settings["recgen_filter_vehicles"], "vehicle.*"))
        form.addRow("Vehicle generation:", combo(settings["recgen_generation_vehicles"], _GENERATIONS, editable=True))
        form.addRow("Walker filter:", line_edit(settings["recgen_filter_walkers"], "walker.pedestrian.*"))
        form.addRow("Walker generation:", combo(settings["recgen_generation_walkers"], _GENERATIONS, editable=True))

        form = self.section("Run")
        form.addRow("Length per run:", double_spin(settings["recgen_length_minutes"], 0.1, 10_000, step=0.5,
                                                   suffix=" min"))
        form.addRow("Output folder:", path_field(app, settings["recgen_output_dir"], folder=True))

        app.register_version_listener(self._show_maps)
        settings["recgen_num_parked"].changed.connect(lambda _: self.refresh_map_filters())
        self.stop_btn = self.actions(primary_button("Run generator", self._start_recgen))

    def _show_maps(self):
        """(Re)build the map checkboxes for the maps of the current CARLA version."""
        for check in self._map_checks.values():
            check.setParent(None)  # off screen right away, not only once deleteLater runs
            check.deleteLater()
        self._map_checks = {}
        # Selections of maps the current version lacks are kept for switching back.
        selected = self.app.settings["recgen_selected_maps"]
        for i, name in enumerate(self.app.available_maps()):
            check = QCheckBox(name)
            check.setChecked(name in selected.get())
            check.toggled.connect(lambda on, n=name: selected.set(
                [m for m in selected.get() if m != n] + ([n] if on else [])))
            self._maps_grid.addWidget(check, i // self._MAP_COLUMNS, i % self._MAP_COLUMNS)
            self._map_checks[name] = check
        self.refresh_map_filters()

    def refresh_map_filters(self):
        """Disable (and untick) maps that can't spawn parked vehicles while parked > 0."""
        parked = self.app.settings["recgen_num_parked"].get() > 0
        for name, check in self._map_checks.items():
            allowed = not parked or name in ALLOWED_PARKED_MAPS
            if not allowed:
                check.setChecked(False)
            check.setEnabled(allowed)

    def _start_recgen(self):
        """
        Start the recording generator across a seed range.
        The generator receives all selected maps and picks one deterministically using the seed.
        """
        app = self.app
        if not app.validate_paths([
            ("CARLA executable", "carla_executable", "carla"),
            ("Output folder", "recgen_output_dir", "dir"),
        ]):
            return None

        selected_maps = [name for name, check in self._map_checks.items() if check.isChecked()]
        if not selected_maps:
            message = "Select at least one map."
            if app.settings["recgen_num_parked"].get() > 0:
                message += f"\nWith parked vehicles, only these maps are possible: {', '.join(ALLOWED_PARKED_MAPS)}"
            return app.show_error("Missing", message)

        app.clear_log()
        runner = RecGenRunner(app.collect_cfg(), app.log, selected_maps=selected_maps)
        app.attach_worker(runner, stop_button=self.stop_btn)
        return None
