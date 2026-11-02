import os
import sys

import cv2
import numpy as np

from helpers.camera_recorder.DownscalingMethod import DownscalingMethod


def record_videos(images_directory: str, output_directory: str, scaling_method: DownscalingMethod,
                   img_width: int = 1920, img_height: int = 1080, fps: int = 20,
                   show_preview: bool = False) -> None:
    """
    Turns each CAM_* subfolder of `images_directory` (as produced by
    CarlaCameraRecorder.record_images) into its own mp4, plus a combined
    multi-view "ALL.mp4" grid of all cameras at once.
    """
    output = os.path.join(output_directory, "_videos")
    os.makedirs(output, exist_ok=True)

    scenario = os.path.basename(os.path.normpath(images_directory))
    output = os.path.join(output, scenario)
    if os.path.exists(output) and os.listdir(output):
        print(f">> [Video] Warning: output directory '{output}' already contains videos. Skipping.",
              file=sys.stderr)
        return
    os.makedirs(output, exist_ok=True)

    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')

    cameras = sorted(os.listdir(images_directory))
    images = []
    videos = []
    for camera in cameras:
        images.append(
            sorted(img for img in os.listdir(os.path.join(images_directory, camera)) if img.endswith(".jpg"))[0:-1])
        video_path = os.path.join(output, f"{camera}.mp4")
        writer = cv2.VideoWriter(video_path, fourcc, fps, (img_width, img_height))
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video writer for {video_path} - check that the mp4v/FFMPEG "
                              f"codec is available in this OpenCV install.")
        videos.append(writer)

    if not images or not images[0]:
        print(f">> [Video] Warning: no images found to render for '{images_directory}'. Skipping.", file=sys.stderr)
        for video in videos:
            video.release()
        return

    # Combined "ALL.mp4" grid height, scaled down proportionally to the number
    # of camera rows so the overall canvas width stays at img_width.
    grid_height = img_height
    match len(cameras):
        case 2:
            grid_height = int(img_height / 2)
        case 5 | 6:
            grid_height = int(img_height * 2 / 3)

    all_video_path = os.path.join(output, "ALL.mp4")
    all_writer = cv2.VideoWriter(all_video_path, fourcc, fps, (img_width, grid_height))
    if not all_writer.isOpened():
        raise RuntimeError(f"Could not open video writer for {all_video_path} - check that the mp4v/FFMPEG "
                          f"codec is available in this OpenCV install.")
    videos.append(all_writer)

    total_ticks = len(images[0]) - 1
    for tick in range(total_ticks):
        print(f"\rRendering video frame {tick + 1} of {total_ticks}")
        img = []
        for idx, camera in enumerate(cameras):
            img.append(cv2.imread(os.path.join(images_directory, camera, images[idx][tick])))
            videos[idx].write(img[-1])

        match len(img):
            case 1:
                break
            case 2:
                img_all = np.concatenate((img[0], img[1]), axis=1)
            case 3:
                img_all = np.concatenate((
                    np.concatenate((img[0], img[1]), axis=1),
                    np.concatenate((img[2], np.zeros_like(img[0])), axis=1)
                ), axis=0)
            case 4:
                img_all = np.concatenate((
                    np.concatenate((img[0], img[1]), axis=1),
                    np.concatenate((img[2], img[3]), axis=1)
                ), axis=0)
            case 5:
                img_all = np.concatenate((
                    np.concatenate((img[0], img[1], img[2]), axis=1),
                    np.concatenate((img[3], img[4], np.zeros_like(img[0])), axis=1)
                ), axis=0)
            case 6:
                img_all = np.concatenate((
                    np.concatenate((img[0], img[1], img[2]), axis=1),
                    np.concatenate((img[3], img[4], img[5]), axis=1)
                ), axis=0)
            case 7:
                img_all = np.concatenate((
                    np.concatenate((img[0], img[1], img[2]), axis=1),
                    np.concatenate((img[3], img[4], img[5]), axis=1),
                    np.concatenate((img[6], np.zeros_like(img[0]), np.zeros_like(img[0])), axis=1)
                ), axis=0)
            case 8:
                img_all = np.concatenate((
                    np.concatenate((img[0], img[1], img[2]), axis=1),
                    np.concatenate((img[3], img[4], img[5]), axis=1),
                    np.concatenate((img[6], img[7], np.zeros_like(img[0])), axis=1)
                ), axis=0)
            case 9:
                img_all = np.concatenate((
                    np.concatenate((img[0], img[1], img[2]), axis=1),
                    np.concatenate((img[3], img[4], img[5]), axis=1),
                    np.concatenate((img[6], img[7], img[8]), axis=1)
                ), axis=0)
            case _:
                print(">> [Video] Too many cameras for the multi-view grid (max 9); skipping ALL.mp4 frames.")
                break

        img_all = cv2.resize(img_all, (img_width, grid_height), interpolation=scaling_method.value)
        videos[-1].write(img_all)
        if show_preview:
            cv2.imshow('Carla video preview', img_all)
            cv2.waitKey(1)

    cv2.destroyAllWindows()

    for video in videos:
        video.release()
