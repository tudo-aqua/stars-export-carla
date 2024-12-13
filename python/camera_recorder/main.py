import os
import sys
from time import sleep
from typing import List

from CarlaCameraRecorder import CarlaCameraRecorder
from python.camera_recorder.CameraPosition import CameraPosition

INPUT_DIR: str = "C:\\Users\\Dominik\\Desktop\\scenarios"
OUTPUT_DIR: str = "C:\\Users\\Dominik\\Desktop\\scenarios\\_output"
DELETE_OUTPUT_DIR: bool = True

CARLA_HOME: str = "C:\\Users\\Dominik\\workspace\\Carla0.9.15\\CarlaUE4.exe"
CARLA_STARTUP_DELAY: int = 10
RENDER_OFFSCREEN: bool = True
SHOW_PREVIEW: bool = True

CAMERA_POSITION: CameraPosition = CameraPosition.BACK_ABOVE
RENDER_BOUNDING_BOXES: bool = True
RENDER_SAFETY_BOXES: bool = True

BOUNDING_BOX_COLOR = (0, 255, 0, 255)
SAFETY_BOUNDING_BOX_COLOR = (255, 0, 255, 255)

def kill_carla():
    os.system('taskkill /f /im CarlaUE4-Win64-Shipping.exe')
    os.system('taskkill /f /im CarlaUE4.exe')

def start_carla():
    os.system(f"start {CARLA_HOME} {'-RenderOffScreen' if RENDER_OFFSCREEN else ''}")
    sleep(CARLA_STARTUP_DELAY)

if __name__ == '__main__':
    # Check if input directory exists
    if not os.path.exists(INPUT_DIR):
        print(f"Error: The input directory {INPUT_DIR} does not exist.", file=sys.stderr)
        exit(1)

    # Delete output directory
    if DELETE_OUTPUT_DIR and os.path.exists(OUTPUT_DIR):
        os.system(f"rmdir /s /q {OUTPUT_DIR}")

    # Create output directory if it does not exist
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    # Create Carla recorder
    recorder = CarlaCameraRecorder(
        output_dir=OUTPUT_DIR,
        camera_position=CAMERA_POSITION,
        render_bounding_boxes=RENDER_BOUNDING_BOXES,
        render_safety_boxes=RENDER_SAFETY_BOXES,
        show_preview=SHOW_PREVIEW
    )

    # Record all logs
    recordings: List[str] = []
    for scenario in [os.path.join(INPUT_DIR, file) for file in os.listdir(INPUT_DIR)]:
        for logfile in [os.path.join(scenario, file) for file in os.listdir(scenario) if file.endswith(".log")]:
            try:
                # Restart Carla
                kill_carla()
                start_carla()
                recordings.append(recorder.record_images(logfile=logfile))
            except RuntimeError:
                print(f"Error: Could not record images for {logfile}", file=sys.stderr)

    kill_carla()

    # Render videos
    for images_directory in recordings:
        recorder.record_video(images_directory=images_directory)

    # Close Carla
    kill_carla()