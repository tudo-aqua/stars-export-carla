from __future__ import annotations

import argparse
import os
import sys
import traceback
from typing import List

from carla_interaction_gui import carla_launcher
from carla_interaction_gui.carla_launcher import restart_and_connect, kill_carla
from data_av_static import MapRasterizer


def run_transform(args):
    from helpers.carla_monitor import CarlaMonitor  # lazy import inside child
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

        # begin/end semantics: align with your GUI (end < 0 => file end)
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
        # Start CARLA once; we’ll swap maps via load_world per your snippet
        client = restart_and_connect(
            exe=args.carla_exe,
            render_off_screen=args.offscreen,
            render_quality_low=args.quality_low,
            # initial map doesn’t matter; we will load per-map below
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
            carla_launcher.set_map_via_config_py(exe=args.carla_exe, map_name=map_name)
            world = client.get_world()

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


def main():
    p = argparse.ArgumentParser("carla_task_runner")
    sub = p.add_subparsers(dest="task", required=True)

    # Shared base options
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

    args = p.parse_args()
    args._fn(args)


if __name__ == "__main__":
    main()
