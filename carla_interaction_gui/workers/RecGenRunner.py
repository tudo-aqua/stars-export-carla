# carla_interaction_gui/workers/RecGenRunner.py
from __future__ import annotations

import os
import sys
from typing import Callable, Iterable

from carla_interaction_gui.workers.ThreadWorker import ThreadWorker


class RecGenRunner(ThreadWorker):
    """
    Runs carla_recording_generator.py repeatedly for a list of seeds.
    The generator itself chooses ONE map deterministically from the
    candidate list passed with repeated --map flags (seeded choice).
    """

    def __init__(
            self,
            cfg,
            log: Callable[[str], None],
            *,
            selected_maps: list[str],
            seeds: Iterable[int],
    ):
        super().__init__(cfg, log)  # no 'exclusive' kwarg in your ThreadWorker
        self.exclusive = True  # keep the intent for any code that checks it
        self._selected_maps = list(selected_maps)
        self._seeds = list(seeds)

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
        ]

        # Stream it like the other workers
        self._start_and_stream(cmd)

        self.log(">> [RecGen] Done.")

    def _resolve_script(self, name: str) -> str | None:
        """
        Try hard to find a top-level runner like 'carla_recording_generator.py'
        regardless of current working directory.
        """
        here = os.path.abspath(os.path.dirname(__file__))

        # 1) Common absolute guesses
        candidates = [
            os.path.join(os.getcwd(), name),  # current working dir
            os.path.join(here, name),  # workers/<name>
            os.path.join(os.path.dirname(here), name),  # carla_interaction_gui/<name>
            os.path.join(os.path.dirname(os.path.dirname(here)), name)  # <repo-root>/<name>  ← important
        ]

        # 2) Walk up to 5 parents from the worker folder and look for the file
        parent = here
        for _ in range(5):
            parent = os.path.dirname(parent)
            candidates.append(os.path.join(parent, name))

        for c in candidates:
            if os.path.isfile(c):
                return os.path.abspath(c)
        return None
