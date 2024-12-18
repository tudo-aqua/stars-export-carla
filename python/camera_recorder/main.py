import os
import sys
from time import sleep
from typing import List, Tuple

from CarlaCameraRecorder import CarlaCameraRecorder
from python.camera_recorder.SafetyBoxStyle import SafetyBoxStyle
from python.camera_recorder.CameraPosition import CameraPosition

INPUT_DIR: str = "C:\\Users\\Dominik\\Desktop\\scenarios"
OUTPUT_DIR: str = "C:\\Users\\Dominik\\Desktop\\scenarios\\_output"
DELETE_OUTPUT_DIR: bool = True

CARLA_HOME: str = "C:\\Users\\Dominik\\workspace\\Carla0.9.15\\CarlaUE4.exe"
CARLA_STARTUP_DELAY: int = 10
RENDER_OFFSCREEN: bool = True
SHOW_PREVIEW: bool = True

CAMERA_POSITIONS: List[Tuple[CameraPosition, bool]] = [
    (CameraPosition.BACK_ABOVE, False),
    (CameraPosition.TOP_DOWN_FAR, False),
    (CameraPosition.REAR, False),

    (CameraPosition.TOP_DOWN_NEAR, True),
    (CameraPosition.TOP_DOWN_FAR, True),
    (CameraPosition.TOP_DOWN_VERY_FAR, True)
]

RENDER_BOUNDING_BOXES: bool = True
RENDER_SAFETY_BOXES: bool = True
RENDER_METADATA: bool = True

SAFETY_BOX_STYLE: SafetyBoxStyle = SafetyBoxStyle.HATCHING

BOUNDING_BOX_COLOR = (0, 255, 0, 255)
SAFETY_BOUNDING_BOX_COLOR = (255, 0, 255, 255)

def kill_carla():
    os.system('taskkill /f /im CarlaUE4-Win64-Shipping.exe')
    os.system('taskkill /f /im CarlaUE4.exe')

def start_carla():
    os.system(f"start {CARLA_HOME} {'-RenderOffScreen' if RENDER_OFFSCREEN else ''}")
    for i in range(CARLA_STARTUP_DELAY):
        print(f"Waiting for {CARLA_STARTUP_DELAY - i}")
        sleep(1)

def __create_images(limit: int):
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

    # Record all logs
    count = 0
    for scenario in [os.path.join(INPUT_DIR, file) for file in os.listdir(INPUT_DIR)]:
        if count == limit:
            break

        for logfile in [os.path.join(scenario, file) for file in os.listdir(scenario) if file.endswith(".log")]:
            try:
                # Restart Carla
                kill_carla()
                start_carla()
                recorder.record_images(logfile=logfile)
            except RuntimeError:
                print(f"Error: Could not record images for {logfile}", file=sys.stderr)

        count += 1

    # Close Carla
    kill_carla()

def __render_videos():
    input_dir = os.path.join(OUTPUT_DIR, "_images")
    for images_directory in os.listdir(input_dir):
        recorder.record_videos(images_directory=os.path.join(input_dir, images_directory))


if __name__ == '__main__':
    # Create Carla recorder
    recorder = CarlaCameraRecorder(
        output_dir=OUTPUT_DIR,
        camera_positions=CAMERA_POSITIONS,
        render_bounding_boxes=RENDER_BOUNDING_BOXES,
        bounding_box_color=BOUNDING_BOX_COLOR,
        render_safety_boxes=RENDER_SAFETY_BOXES,
        safety_box_style=SAFETY_BOX_STYLE,
        safety_bounding_box_color=SAFETY_BOUNDING_BOX_COLOR,
        render_metadata=RENDER_METADATA,
        show_preview=SHOW_PREVIEW
    )

    #__create_images(limit=1)
    __render_videos()