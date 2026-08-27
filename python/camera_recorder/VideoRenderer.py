import os
import sys

import cv2
import numpy as np

from python.camera_recorder import DownscalingMethod


def record_videos(images_directory: str, output_directory: str, scaling_method: DownscalingMethod, show_preview: bool) -> None:
    output = os.path.join(output_directory, "_videos\\")
    if not os.path.exists(output):
        os.makedirs(output)

    scenario = images_directory.split('\\')[-1].split('.')[0]
    output = os.path.join(output, scenario)
    if os.path.exists(output) and os.listdir(output):
        print(f"Warning: The output directory {output} already contains videos. Skipping.", file=sys.stderr)
        return
    os.makedirs(output, exist_ok=True)

    cameras = sorted(os.listdir(images_directory))
    images = []
    videos = []
    for camera in cameras:
        images.append(sorted(img for img in os.listdir(os.path.join(images_directory, camera)) if img.endswith(".jpg"))[0:-1])
        # noinspection PyUnresolvedReferences
        writer = cv2.VideoWriter(f"{output}\\{camera}.mp4", cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 20, (1920, 1080))
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video writer for {output}\\{camera}.mp4 - check that the mp4v/FFMPEG codec is available in this OpenCV install.")
        videos.append(writer)

    if not images or not images[0]:
        print(f"Warning: No images found to render for {images_directory}. Skipping.", file=sys.stderr)
        for video in videos:
            video.release()
        return

    # Default for cases 1, 3, 4, 7, 8, 9
    width:int = 1080
    match len(cameras):
        case 2:
            width = int(1080/2)
        case 5 | 6:
            width = int(1080*2/3)
        # case 10 | 11 | 12:
        #     width = 1080*3/4

    # noinspection PyUnresolvedReferences
    all_writer = cv2.VideoWriter(f"{output}\\ALL.mp4", cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 20, (1920, width))
    if not all_writer.isOpened():
        raise RuntimeError(f"Could not open video writer for {output}\\ALL.mp4 - check that the mp4v/FFMPEG codec is available in this OpenCV install.")
    videos.append(all_writer)

    total_ticks = len(images[0])-1
    for tick in range(total_ticks):
        print(f"\rRendering video frame {tick+1} of {total_ticks}")
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
                print("Too many cameras for multi-view")
                break


        img_all = cv2.resize(img_all, (1920, width), interpolation=scaling_method.value)
        videos[-1].write(img_all)
        if show_preview:
            cv2.imshow('Carla video preview', img_all)
            cv2.waitKey(1)

    cv2.destroyAllWindows()

    for video in videos:
        video.release()