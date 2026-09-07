from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
import traceback
import zipfile
from typing import List

from carla_interaction_gui.carla_launcher import restart_and_connect, kill_carla
from data_av_static import MapRasterizer
from helpers.carla_monitor import CarlaMonitor
from helpers.json_helper import JSONHelper


def run_transform(args):
    """
    Transform a single recording, or – if --input is a directory – transform
    all recording/weather pairs found in Town* subfolders.

    Each recording is transformed in its own fresh "transform_one" subprocess
    (see run_transform_one), reusing this same already-running CARLA server
    rather than restarting it per file. CARLA's native client can be unstable
    across repeated replay_file()/show_recorder_file_info() calls within one
    process (up to and including segfaults with no Python traceback), so
    isolating each recording into its own process means a crash on one
    recording doesn't take the rest of the batch down with it, while still
    avoiding the cost of a full server restart per file.

    Folder mode:
      <input_root>/
        Town01_.../
          recording_seed_0.zip
          weather_data_seed_0.zip
          recording_seed_1.zip
          weather_data_seed_1.zip
          ...
        Town02_.../
          ...
    """
    try:
        restart_and_connect(
            exe=args.carla_exe,
            render_off_screen=args.offscreen,
            render_quality_low=args.quality_low,
            map_name=args.map_name or None,
            log=print,
        )

        # ── Check if we are in "single-file" or "folder" mode ──────────────
        in_path = args.input
        docker_mount_path = (getattr(args, "docker_mount_path", "") or "").strip()

        def _to_docker_path(recording_path: str) -> str:
            """
            Recording paths are read by the CARLA *server*, which - when it
            runs inside a docker container (see carla_run.sh) - can only see
            them under the container's own mount path, not the host path
            this process (and its os.path.isdir/os.listdir folder-mode
            discovery above) actually sees them at. If the user has given
            the container-side equivalent of --input via --docker-mount-path,
            rewrite host-discovered paths to that container-side path,
            preserving their position relative to --input.
            """
            if not docker_mount_path:
                return recording_path
            if os.path.isfile(in_path):
                # --input itself is the single file being transformed, so
                # the given mount path is already its exact container-side
                # path - there's no relative offset to preserve.
                return docker_mount_path
            rel = os.path.relpath(recording_path, in_path)
            return recording_path if rel == "." else f"{docker_mount_path.rstrip('/')}/{rel.replace(os.sep, '/')}"

        def _output_zip_for(recording_path: str) -> str:
            """The output .zip monitor_simulation_run() will produce for this recording."""
            file_name = os.path.splitext(os.path.basename(recording_path))[0]
            return os.path.join(args.output, f"{JSONHelper.DYNAMIC_FILE_NAME_PREFIX}_{file_name}.zip")

        succeeded: List[str] = []
        failed: List[str] = []
        skipped: List[str] = []

        # Helper to run a single transform, isolated in its own subprocess.
        def _run_single(recording_path: str, weather_path: str = ""):
            # weather_path is read directly by this (host) process via plain
            # file I/O, so it must stay a host path - only the recording
            # itself needs the container-side translation.
            docker_recording_path = _to_docker_path(recording_path)
            if docker_recording_path != recording_path:
                print(f">> [Runner] Mapped '{recording_path}' -> docker path '{docker_recording_path}'")
            print(f">> [Runner] Transform '{docker_recording_path}' -> '{args.output}'")

            cmd = [
                sys.executable, os.path.abspath(__file__), "transform_one",
                "--input", docker_recording_path,
                "--weather", weather_path,
                "--output", args.output,
            ]
            if args.only_track_at_specific_interval:
                cmd += ["--only-track-at-specific-interval",
                        "--specific-track-interval", str(args.specific_track_interval)]

            # Inherits this process's stdout/stderr, so its output streams
            # straight through - no manual relaying needed.
            result = subprocess.run(cmd)
            if result.returncode == 0:
                succeeded.append(recording_path)
            else:
                failed.append(recording_path)
                print(f">> [Runner] FAILED (exit code {result.returncode}): '{recording_path}' "
                      f"- continuing with the next recording.")

        if os.path.isdir(in_path):
            print(f">> [Runner] Batch transform from root folder: {in_path}")
            processed = 0

            recording_exts = {".log", ".rec", ".zip"}
            cfg_ext = (getattr(args, "recording_ext", "") or "").strip().lower()
            if cfg_ext:
                if not cfg_ext.startswith("."):
                    cfg_ext = "." + cfg_ext
                recording_exts.add(cfg_ext)

            # ── Structured mode: Town* subfolders with recording/weather pairs ──
            for entry in sorted(os.listdir(in_path)):
                sub = os.path.join(in_path, entry)
                if not os.path.isdir(sub):
                    continue
                if "Town" not in entry:
                    continue

                print(f">> [Runner] Searching in subfolder: {sub}")

                # Build pairs of recording_seed_n.zip <-> weather_data_seed_n.zip
                names = set(os.listdir(sub))
                for fname in sorted(names):
                    m = re.match(r"recording_seed_(\d+)\.zip$", fname)
                    if not m:
                        continue
                    seed = m.group(1)

                    recording_path = os.path.join(sub, fname)
                    weather_name = f"weather_data_seed_{seed}.zip"
                    weather_path = os.path.join(sub, weather_name)

                    if not os.path.exists(weather_path):
                        print(f">> [Runner] WARNING: No weather file for seed {seed} in '{sub}' "
                              f"(expected '{weather_name}'). Skipping this seed.")
                        continue

                    expected_zip = _output_zip_for(recording_path)
                    if os.path.exists(expected_zip):
                        print(f">> [Runner] Skipping seed {seed}: already transformed -> '{expected_zip}'")
                        skipped.append(recording_path)
                        processed += 1
                        continue

                    print(f">> [Runner] Found pair for seed {seed}:")
                    print(f"              recording = {recording_path}")
                    print(f"              weather   = {weather_path}")

                    # Unzip files
                    with zipfile.ZipFile(recording_path, 'r') as zip_ref:
                        zip_ref.extractall(os.path.dirname(recording_path))
                    with zipfile.ZipFile(weather_path, 'r') as zip_ref:
                        zip_ref.extractall(os.path.dirname(weather_path))

                    unzipped_recording_path = recording_path.replace(".zip", ".log")
                    unzipped_weather_path = weather_path.replace(".zip", ".json")

                    _run_single(unzipped_recording_path, unzipped_weather_path)
                    processed += 1

            # Flat mode: recordings sitting directly in the root folder
            for fname in sorted(os.listdir(in_path)):
                fpath = os.path.join(in_path, fname)
                if not os.path.isfile(fpath):
                    continue
                # Skip files that belong to the structured mode handled above.
                if re.match(r"recording_seed_\d+\.zip$", fname):
                    continue
                if fname.startswith("weather_data_seed_"):
                    continue
                # Skip this tool's own prior output, in case --output is the
                # same folder as --input (otherwise a previous run's output
                # gets rediscovered and fed back in as if it were a fresh
                # recording, which it isn't - it's a processed JSON/zip dump).
                if fname.startswith((
                        JSONHelper.DYNAMIC_FILE_NAME_PREFIX,
                        JSONHelper.STATIC_FILE_NAME_PREFIX,
                        JSONHelper.WEATHER_FILE_NAME_PREFIX,
                )):
                    continue

                ext = os.path.splitext(fname)[1].lower()
                if ext not in recording_exts:
                    continue

                expected_zip = _output_zip_for(fpath)
                if os.path.exists(expected_zip):
                    print(f">> [Runner] Skipping '{fpath}': already transformed -> '{expected_zip}'")
                    skipped.append(fpath)
                    processed += 1
                    continue

                recording_path = fpath
                if ext == ".zip":
                    with zipfile.ZipFile(fpath, 'r') as zip_ref:
                        zip_ref.extractall(os.path.dirname(fpath))
                    recording_path = fpath[:-4] + ".log"

                print(f">> [Runner] Found flat recording: {recording_path}")
                _run_single(recording_path, "")
                processed += 1

            if processed == 0:
                print(f">> [Runner] WARNING: No transformable recordings found in '{in_path}'. "
                      f"Expected either Town*/recording_seed_N.zip (+ weather_data_seed_N.zip) "
                      f"pairs, or recording files ({', '.join(sorted(recording_exts))}) "
                      f"directly in the folder.")

            print(f">> [Runner] Batch transform finished: {len(succeeded)} succeeded, "
                  f"{len(failed)} failed, {len(skipped)} skipped, out of {processed} recording(s) processed.")
            if failed:
                print(">> [Runner] Failed recordings:")
                for f in failed:
                    print(f"    - {f}")

        else:
            # A directory-shaped path can end up here if it only exists on
            # the CARLA *server's* side (e.g. a docker container's mounted
            # volume, such as the manual-driving recordings under
            # '/workspace/recordings/' from carla_run.sh) - this host process
            # can't see it with os.path.isdir, so it falls through to here
            # instead of the folder/batch branch above. CARLA's native
            # client doesn't validate this and can segfault the whole
            # process if handed a directory instead of an actual recorder
            # file, so reject anything that doesn't look like one up front.
            recording_exts = (".log", ".rec", ".zip")
            if in_path.endswith(("/", "\\")) or not in_path.lower().endswith(recording_exts):
                print(f">> [Runner] ERROR: '{in_path}' doesn't look like a single recording "
                      f"file ({', '.join(recording_exts)}). If this is meant to be a folder, "
                      f"give the path as seen from THIS machine (e.g. the host side of a "
                      f"docker volume mount) and use --docker-mount-path for the container-side "
                      f"equivalent, rather than typing the container path directly here.")
                return

            expected_zip = _output_zip_for(in_path)
            if os.path.exists(expected_zip):
                print(f">> [Runner] Skipping '{in_path}': already transformed -> '{expected_zip}'")
                skipped.append(in_path)
            else:
                # Original single-file behaviour (still supported)
                print(f">> [Runner] Transform single file: '{in_path}' -> '{args.output}'")
                # We keep weather_file_path empty here to retain existing behaviour
                _run_single(in_path, "")

        print(">> [Runner] Transform finished.")
        # A per-file failure is now handled (logged, batch continues) rather
        # than raised, so surface it as a non-zero exit code instead - the
        # GUI worker reports abnormal exits, and callers/scripts checking
        # this process's return code still see that something failed.
        return 1 if failed else 0
    except Exception:
        traceback.print_exc()
        raise
    finally:
        try:
            kill_carla(log=print)
        except Exception:
            pass


def run_transform_one(args):
    """
    Transform exactly one already-resolved recording against an already-
    running CARLA server (does NOT restart it - see run_transform, which is
    the caller for batch/folder mode and owns the server's lifecycle).

    Kept as its own process/subcommand so that if this recording's replay
    crashes CARLA's native client, it only takes down this one process - the
    caller just sees a non-zero exit code for this one file and moves on to
    the next, instead of the whole batch dying with it.
    """
    import carla

    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)
    mon = CarlaMonitor(carla_client=client)

    if args.only_track_at_specific_interval:
        mon.monitor_simulation_run(
            file_path=args.input,
            weather_file_path=args.weather,
            result_file_path=args.output,
            only_track_at_specific_interval=True,
            specific_track_interval=args.specific_track_interval,
        )
    else:
        mon.monitor_simulation_run(
            file_path=args.input,
            weather_file_path=args.weather,
            result_file_path=args.output,
        )


def run_record_video(args):
    # Choose correct recorder class based on --with-bboxes
    if args.with_bboxes:
        from helpers.carla_camera_recorder_with_bboxes import CarlaCameraRecorder as Recorder
    else:
        from helpers.carla_camera_recorder import CarlaCameraRecorder as Recorder

    client = None
    try:
        client = restart_and_connect(
            exe=args.carla_exe,
            render_off_screen=args.offscreen,
            render_quality_low=args.quality_low,
            map_name=args.map_name or None,
            log=print,
        )
        rec = Recorder(client)

        end_at = sys.maxsize if args.end_at is None or args.end_at < 0 else args.end_at

        print(f">> [Runner] Record video (bboxes={args.with_bboxes}) from '{args.input}'")
        rec.record_camera_in_simulation_run(
            recording_folder=args.output,
            path=args.input,
            vehicle_id=args.vehicle_id,
            width=args.width,
            height=args.height,
            begin_at=max(0.0, args.begin_at or 0.0),
            end_at=end_at,
        )
        print(">> [Runner] Video export finished.")
    except Exception:
        traceback.print_exc()
        raise
    finally:
        try:
            kill_carla(log=print)
        except Exception:
            pass


def run_gen_maps(args):
    """
    Start CARLA, then for each provided map:
      - client.load_world(map_name)
      - world = client.get_world()
      - MapRasterizer(world).load_or_calculate_data_world(log_file_path=args.output, map_name=map_name)
    """
    client = None
    try:
        client = restart_and_connect(
            exe=args.carla_exe,
            render_off_screen=args.offscreen,
            render_quality_low=False,
            map_name=None,
            log=print,
        )

        maps: List[str] = args.map or []
        if not maps:
            print("!! No maps provided to gen_maps; nothing to do.")
            return

        # Skip maps that aren't installed on this CARLA server (e.g. Town06/Town07 ship
        # separately in CARLA's Additional Maps package) instead of crashing the whole batch.
        available = client.get_available_maps()
        resolved_maps = []
        for map_name in maps:
            if any(map_name in available_map for available_map in available):
                resolved_maps.append(map_name)
            else:
                print(f"!! [GenerateMaps] Map '{map_name}' is not installed on this CARLA server; skipping.")
        if not resolved_maps:
            print("!! [GenerateMaps] None of the requested maps are available on this server; nothing to do.")
            return
        maps = resolved_maps

        # Ensure output folder exists
        os.makedirs(args.output, exist_ok=True)

        for map_name in maps:
            print(f">> [GenerateMaps] Loading map: {map_name}")
            try:
                client.load_world(map_name)
            except RuntimeError as err:
                print(f">> [GenerateMaps] Failed to load map '{map_name}': {err}. Skipping.")
                continue
            time.sleep(3)
            world = client.get_world()
            current_map_name = world.get_map().name
            if map_name not in current_map_name:
                print(f">> [GenerateMaps] Warning: map name mismatch: {current_map_name} != {map_name}")
                print(">> [GenerateMaps] Wait 10 seconds and retry.")
                time.sleep(10)
                world = client.get_world()
                current_map_name = world.get_map().name
                if map_name not in current_map_name:
                    print(f">> [GenerateMaps] Failed to load map: {map_name}")
                    continue

            rasterizer = MapRasterizer(world)
            print(">> [Data-AV Transformer] Load or calculate map data.")
            rasterizer.load_or_calculate_data_world(
                log_file_path=args.output,
                map_name=map_name
            )
            print(f">> [GenerateMaps] Finished map: {map_name}")

        print(">> [GenerateMaps] All maps done.")

    finally:
        try:
            kill_carla(log=print)
        except Exception:
            pass


def run_recgen_once(args):
    """
    Connect to an already-running CARLA server and run one recording
    via CarlaDataGenerator.run_recording_generation(...). This is intended
    to be launched by the parent 'recgen' task as a child process (per seed).
    """
    import time
    import carla
    from helpers.carla_recording_generator import CarlaDataGenerator  # keep your existing import layout

    # Connect to the existing server the parent has started
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)

    # Make sure the world is ticking (server may need a breath)
    try:
        world = client.get_world()
        _ = world.wait_for_tick(10.0)
    except Exception:
        time.sleep(1.0)

    generator = CarlaDataGenerator(client)
    candidate_maps = list(args.map) if args.map else None

    print(f">> [RecGen-Once] seed {args.seed} start")
    generator.run_recording_generation(
        client,
        seed=int(args.seed),
        length_minutes=args.length_of_run,
        number_of_vehicles=args.number_of_vehicles,
        number_of_walkers=args.number_of_walkers,
        filterv=args.filterv,
        generationv=args.generationv,
        filterw=args.filterw,
        generationw=args.generationw,
        candidate_maps=candidate_maps,
        output_dir=args.output,
        no_rendering=args.offscreen,
        number_of_parked=args.number_of_parked,
    )
    print(f">> [RecGen-Once] seed {args.seed} finished")


def run_spawn_traffic(args):
    """
    Connect to an already-running CARLA server and spawn a batch of
    autopilot-driven vehicles and walkers into the current world, using the
    same routine as the recording generator's traffic setup.
    """
    import time
    import carla
    from types import SimpleNamespace
    from helpers.carla_recording_generator import CarlaDataGenerator

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)

    try:
        world = client.get_world()
        _ = world.wait_for_tick(10.0)
    except Exception:
        time.sleep(1.0)

    generator = CarlaDataGenerator(client)
    world = generator.world

    tm_args = SimpleNamespace(
        seed=None,
        tm_port=8000,
        respawn=False,
        hybrid=True,
        # We are injecting traffic into an already-running session (e.g. a
        # manual-driving session) rather than driving the world ourselves.
        # Whatever tick mode that session is already using (synchronous or
        # not), generate_traffic() leaves synchronous_mode/fixed_delta_seconds
        # alone and just matches the Traffic Manager's own mode to it -
        # setting our own assumption here would either hijack that session's
        # world settings (freezing it once this process exits and nobody is
        # left to tick) or desync the Traffic Manager from the world's
        # actual tick mode (causing autopilot vehicles to go unstable).
        asynch=True,
        no_rendering=False,
        hero=False,
        car_lights_on=False,
        number_of_vehicles=args.number_of_vehicles,
        number_of_walkers=args.number_of_walkers,
        filterv=args.filterv,
        generationv=args.generationv,
        filterw=args.filterw,
        generationw=args.generationw,
    )

    print(f">> [SpawnTraffic] spawning {args.number_of_vehicles} vehicles "
          f"and {args.number_of_walkers} walkers")
    generator.generate_traffic(tm_args, client, world)
    print(">> [SpawnTraffic] done")


def run_recgen(args):
    """
    For each seed:
      - start fresh CARLA,
      - spawn a child process: `python carla_task_runner.py recgen-once --seed <s> ...`
      - stream its output,
      - kill CARLA,
      - continue to next seed.
    """
    import os
    import sys
    import time
    import subprocess
    import traceback

    # Build the static part of the child command (everything except --seed)
    # We call the same script (this file) with subcommand 'recgen-once'.
    runner_path = os.path.abspath(__file__)
    base_cmd = [
        sys.executable or "python",
        runner_path,
        "recgen-once",
        "--output", args.output,
        "--length-of-run", str(args.length_of_run),
        "--number-of-vehicles", str(args.number_of_vehicles),
        "--number-of-walkers", str(args.number_of_walkers),
        "--filterv", args.filterv,
        "--generationv", args.generationv,
        "--filterw", args.filterw,
        "--generationw", args.generationw,
        "--number-of-parked", str(args.number_of_parked),
    ]
    if args.offscreen:
        base_cmd.append("--offscreen")
    for m in (args.map or []):
        base_cmd += ["--map", m]

    seed_start = int(args.seed_start)
    num_scenarios = max(1, int(args.num_scenarios))
    last_error = None

    for s in range(seed_start, seed_start + num_scenarios):
        print(f">> [RecGen] seed {s}")
        try:
            # Start a fresh server
            _client = restart_and_connect(
                exe=args.carla_exe,
                render_off_screen=args.offscreen,
                render_quality_low=args.quality_low,
                map_name=None,  # child will load maps as needed
                log=print,
            )
            # Spawn the child that does ONE generation, streaming output
            cmd = base_cmd + ["--seed", str(s)]
            print(">> [Runner-Child] " + " ".join(cmd))
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line.rstrip())
            proc.wait()
            if proc.returncode != 0:
                raise RuntimeError(f"recgen-once child exited with code {proc.returncode}")
            print(f">> [RecGen] seed {s} finished")
        except Exception as e:
            last_error = e
            print(f">> [RecGen] seed {s} FAILED:")
            traceback.print_exc()
        finally:
            # Always stop CARLA before the next seed
            try:
                kill_carla(log=print)
            except Exception:
                pass
            time.sleep(1.0)  # small cool-down

    print(">> [RecGen] All scenarios finished.")
    if last_error:
        raise last_error


def main():
    p = argparse.ArgumentParser("carla_task_runner")
    sub = p.add_subparsers(dest="task", required=True)

    def add_common(sp):
        sp.add_argument("--carla-exe", required=True, help="Path to CARLA executable")
        sp.add_argument("--offscreen", action="store_true", default=False)
        sp.add_argument("--quality-low", action="store_true", default=False)
        sp.add_argument("--map-name", default="", help="Optional map to load")

    # transform
    pt = sub.add_parser("transform", help="Replay a recording and dump processed data")
    add_common(pt)
    pt.add_argument("--input", required=True, help="Input recording file (.log/.zip/etc.)")
    pt.add_argument("--output", required=True, help="Output folder for JSON/zip")
    pt.add_argument("--docker-mount-path", dest="docker_mount_path", default="",
                     help="Container-side path corresponding to --input, if CARLA runs in a "
                          "docker container whose mounted recordings folder differs from the "
                          "path this process sees --input at (see carla_run.sh)")
    # NEW: CLI to control sampling
    pt.add_argument("--recording-ext", dest="recording_ext", default="",
                    help="Recording file extension to pick up in flat-folder batch mode "
                         "(in addition to .log/.rec/.zip)")
    pt.add_argument("--only-track-at-specific-interval", action="store_true", default=False)
    pt.add_argument("--specific-track-interval", type=float, default=0.5)
    pt.set_defaults(_fn=run_transform)

    # transform_one (internal: one recording, against an already-running server - see run_transform)
    pto = sub.add_parser("transform_one",
                          help="Transform exactly one recording against an already-running CARLA server")
    pto.add_argument("--input", required=True, help="Recording file path, as seen by the CARLA server")
    pto.add_argument("--weather", default="", help="Weather JSON file path, as seen by this (host) process")
    pto.add_argument("--output", required=True, help="Output folder for JSON/zip")
    pto.add_argument("--only-track-at-specific-interval", action="store_true", default=False)
    pto.add_argument("--specific-track-interval", type=float, default=0.5)
    pto.set_defaults(_fn=run_transform_one)

    # record_video
    pv = sub.add_parser("record_video", help="Render a recording directly to mp4")
    add_common(pv)
    pv.add_argument("--input", required=True, help="Input recording file")
    pv.add_argument("--output", required=True, help="Output folder (images/mp4)")
    pv.add_argument("--width", type=int, required=True)
    pv.add_argument("--height", type=int, required=True)
    pv.add_argument("--vehicle-id", type=int, default=-1)
    pv.add_argument("--begin-at", dest="begin_at", type=float, default=0.0)
    pv.add_argument("--end-at", dest="end_at", type=float, default=None)
    pv.add_argument("--with-bboxes", action="store_true", default=False)
    pv.set_defaults(_fn=run_record_video)

    # generate maps
    pg = sub.add_parser("gen_maps", help="Generate map files for a list of maps")
    add_common(pg)
    pg.add_argument("--output", required=True, help="Output folder for generated map data")
    pg.add_argument("--map", action="append", help="Map name to generate (repeatable)")
    pg.set_defaults(_fn=run_gen_maps)

    # recording generator (recgen)
    pr = sub.add_parser("recgen", help="Generate recordings over a seed range (deterministic map per seed)")
    add_common(pr)
    pr.add_argument("--output", required=True, help="Output folder for recordings")

    # Map candidates (repeatable). If omitted, the generator will use server-usable maps.
    pr.add_argument("--map", action="append", help="Candidate map name (repeatable), e.g. Town01")

    # Seed range
    pr.add_argument("--seed-start", type=int, default=0, help="First seed (inclusive)")
    pr.add_argument("--num-scenarios", type=int, default=1, help="Number of seeds to run")
    pr.set_defaults(_fn=run_recgen)

    # Traffic parameters & filters (names match generator CLI)
    pr.add_argument("--number-of-vehicles", type=int, default=200)
    pr.add_argument("--number-of-walkers", type=int, default=30)
    pr.add_argument("--filterv", default="vehicle.*")
    pr.add_argument("--generationv", default="All")
    pr.add_argument("--filterw", default="walker.pedestrian.*")
    pr.add_argument("--generationw", default="2")
    pr.add_argument("--number-of-parked", type=int, default=0,
                    help="Number of parked vehicles to spawn on shoulder lanes")

    pr1 = sub.add_parser("recgen-once", help="Run a single recording generation (expects server to be running)")
    # NOTE: do NOT call add_common(pr1) here; child must not require --carla-exe
    pr1.add_argument("--offscreen", action="store_true", default=False)  # we keep this for parity
    pr1.add_argument("--quality-low", action="store_true", default=False)  # not used, but harmless if passed
    pr1.add_argument("--output", required=True, help="Output folder for recordings")
    pr1.add_argument("--seed", type=int, required=True, help="Seed for this single run")
    pr1.add_argument("--map", action="append", help="Candidate map name (repeatable), e.g. Town01")
    pr1.add_argument("--number-of-vehicles", type=int, default=200)
    pr1.add_argument("--number-of-walkers", type=int, default=30)
    pr1.add_argument("--filterv", default="vehicle.*")
    pr1.add_argument("--generationv", default="All")
    pr1.add_argument("--filterw", default="walker.pedestrian.*")
    pr1.add_argument("--generationw", default="2")
    pr1.add_argument("--length-of-run", type=float, default=5.0)
    pr1.add_argument("--number-of-parked", type=int, default=0)

    pr1.set_defaults(_fn=run_recgen_once)

    # spawn_traffic (expects server to be running, e.g. from a manual driving session)
    pst = sub.add_parser("spawn_traffic", help="Spawn random autopilot traffic into the running world")
    pst.add_argument("--number-of-vehicles", type=int, default=30)
    pst.add_argument("--number-of-walkers", type=int, default=10)
    pst.add_argument("--filterv", default="vehicle.*")
    pst.add_argument("--generationv", default="All")
    pst.add_argument("--filterw", default="walker.pedestrian.*")
    pst.add_argument("--generationw", default="2")
    pst.set_defaults(_fn=run_spawn_traffic)

    # Duration (minutes)
    pr.add_argument("--length-of-run", type=float, default=5.0)

    args = p.parse_args()
    return args._fn(args)


if __name__ == "__main__":
    # A crash inside CARLA's native client (e.g. a segfault) kills the
    # process directly, bypassing Python's exception handling entirely - no
    # `except Exception` ever runs, and there is no Python stack trace to
    # print for it. faulthandler at least dumps the low-level C stack for
    # fatal signals to stderr, which the GUI streams into its persisted log
    # file, so something survives after the window closes.
    import faulthandler
    faulthandler.enable()
    sys.exit(main() or 0)
