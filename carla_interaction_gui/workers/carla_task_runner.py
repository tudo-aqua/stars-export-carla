from __future__ import annotations

import argparse
import os
import sys
import time
import traceback
from typing import List

from carla_interaction_gui.carla_launcher import restart_and_connect, kill_carla
from carla_interaction_gui.workers import manual_agent_control
from data_av_static import MapRasterizer
from helpers.carla_monitor import CarlaMonitor


def run_transform(args):
    client = None
    try:
        client = restart_and_connect(
            exe=args.carla_exe,
            render_off_screen=args.offscreen,
            render_quality_low=args.quality_low,
            map_name=args.map_name or None,
            log=print,
        )
        mon = CarlaMonitor(carla_client=client)
        print(f">> [Runner] Transform '{args.input}' -> '{args.output}'")

        if args.only_track_at_specific_interval:
            # Pass kwargs ONLY when the toggle is enabled
            mon.monitor_simulation_run(
                file_path=args.input,
                weather_file_path="",
                result_file_path=args.output,
                only_track_at_specific_interval=True,
                specific_track_interval=args.specific_track_interval,
            )
        else:
            # Do not pass the parameters at all
            mon.monitor_simulation_run(
                file_path=args.input,
                weather_file_path="",
                result_file_path=args.output,
            )

        print(">> [Runner] Transform finished.")
    except Exception:
        traceback.print_exc()
        raise
    finally:
        try:
            kill_carla(log=print)
        except Exception:
            pass


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

        print(">> [Runner] Encoding mp4")
        stemless = __import__("os").path.splitext(__import__("os").path.basename(args.input))[0]
        if args.with_bboxes:
            rec.save_video(args.output, stemless, args.vehicle_id, max(0.0, args.begin_at or 0.0), rec.END_AT, True)
        else:
            rec.save_video(args.output, stemless, args.vehicle_id, max(0.0, args.begin_at or 0.0), rec.END_AT)

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

        # Ensure output folder exists
        os.makedirs(args.output, exist_ok=True)

        for map_name in maps:
            print(f">> [GenerateMaps] Loading map: {map_name}")
            client.load_world(map_name)
            time.sleep(3)
            world = client.get_world()
            current_map_name = world.get_map().name
            if map_name not in current_map_name:
                print(f">> [GenerateMaps] Warning: map name mismatch: {current_map_name} != {map_name}")
                print(f">> [GenerateMaps] Wait 10 seconds and retry.")
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


def run_manual_agent(args):
    """
    Start CARLA (if needed), then launch a pygame viewer that behaves like manual_control.py,
    except that pressing 'P' toggles our Python carla_agent instead of Traffic Manager.
    """
    client = None
    try:
        client = restart_and_connect(
            exe=args.carla_exe,
            render_off_screen=args.offscreen,
            render_quality_low=args.quality_low,
            map_name=args.map_name or None,
            log=print,
        )
        manual_agent_control.launch_from_runner(
            host="127.0.0.1",
            port=2000,
            res=args.res,
            sync=args.sync,
            carla_exe=args.carla_exe
        )
    except Exception:
        traceback.print_exc()
        raise
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
        no_rendering=args.offscreen,  # align with parent setting
    )
    print(f">> [RecGen-Once] seed {args.seed} finished")


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
    # NEW: CLI to control sampling
    pt.add_argument("--only-track-at-specific-interval", action="store_true", default=False)
    pt.add_argument("--specific-track-interval", type=float, default=0.5)
    pt.set_defaults(_fn=run_transform)

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

    # carla_agent drive
    pm = sub.add_parser("manual_agent", help="Manual drive viewer where 'P' toggles Python carla_agent")
    add_common(pm)
    pm.add_argument("--res", default="1280x720", help="Window resolution, e.g. 1280x720")
    pm.add_argument("--sync", action="store_true", help="Run viewer in synchronous mode")
    pm.set_defaults(_fn=run_manual_agent)

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
    pr1.set_defaults(_fn=run_recgen_once)

    # Duration (minutes)
    pr.add_argument("--length-of-run", type=float, default=5.0)

    args = p.parse_args()
    return args._fn(args)

if __name__ == "__main__":
    sys.exit(main() or 0)
