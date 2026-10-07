# carla_interaction_gui/workers/RecGenRunner.py
from __future__ import annotations

import sys
from typing import Callable

from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class RecGenRunner(ThreadWorker):
    """
    Runs carla_recording_generator.py repeatedly for a seed range.
    The seed range and candidate maps are forwarded to the 'recgen' subcommand
    of carla_task_runner.py, which loops over the seeds itself (see
    carla_task_runner.run_recgen). The generator chooses ONE map deterministically
    per seed from the candidate list (seeded choice).
    """

    def __init__(
            self,
            cfg,
            log: Callable[[str], None],
            *,
            selected_maps: list[str],
    ):
        super().__init__(cfg, log)
        self._selected_maps = list(selected_maps)

    def run(self):
        # Always use the standard runner (carla_task_runner.py) like other workers
        runner = self._resolve_runner()
        if not runner:
            self.log("!! Could not locate carla_task_runner.py")
            return

        cfg = self.cfg

        # Build a SINGLE command that delegates looping to the 'recgen' subcommand
        cmd = [
            sys.executable or "python",
            runner,
            "recgen",
            "--carla-exe", cfg.carla_executable,
        ]

        # Render flags consistent with other tasks
        if getattr(cfg, "render_off_screen", False):
            cmd.append("--offscreen")
        if getattr(cfg, "render_quality_low", False):
            cmd.append("--quality-low")

        # Output directory
        cmd += ["--output", cfg.recgen_output_dir]

        # Seed range
        cmd += ["--seed-start", str(getattr(cfg, "recgen_seed_start", 0))]
        cmd += ["--num-scenarios", str(max(1, int(getattr(cfg, "recgen_num_scenarios", 1))))]

        # Candidate maps (repeatable)
        for m in self._selected_maps:
            cmd += ["--map", m]

        # Traffic & filters
        cmd += [
            "--number-of-vehicles", str(getattr(cfg, "recgen_num_vehicles", 200)),
            "--number-of-walkers", str(getattr(cfg, "recgen_num_walkers", 30)),
            "--filterv", getattr(cfg, "recgen_filter_vehicles", "vehicle.*"),
            "--generationv", getattr(cfg, "recgen_generation_vehicles", "All"),
            "--filterw", getattr(cfg, "recgen_filter_walkers", "walker.pedestrian.*"),
            "--generationw", getattr(cfg, "recgen_generation_walkers", "2"),
            "--length-of-run", str(getattr(cfg, "recgen_length_minutes", 5.0)),
            "--number-of-parked", str(getattr(cfg, "recgen_num_parked", 0))
        ]

        # Stream it like the other workers
        self._start_and_stream(cmd)

        self.log(">> [RecGen] Done.")
