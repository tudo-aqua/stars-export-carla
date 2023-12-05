import json
import os
from datetime import datetime
from os.path import dirname
from typing import List
import zipfile
from pathlib import Path

from carla_data_classes import DataBlock, TickData, DataWeatherParameters


class JSONHelper:
    SIMULATION_RUNS_FOLDER = "simulation_runs"
    RECORDINGS_RUNS_FOLDER = "recordings"
    VIDEO_IMAGE_FOLDER = "video_images"
    VIDEO_FOLDER = "videos"
    ERROR_FOLDER = "errors"

    DYNAMIC_FILE_NAME_PREFIX = "dynamic_data"
    STATIC_FILE_NAME_PREFIX = "static_data"
    WEATHER_FILE_NAME_PREFIX = "weather_data"

    @staticmethod
    def get_experiment_data_folder() -> os.path:
        stars_main_dir = Path(os.path.dirname(os.path.abspath(__file__))).parent
        recording_dir = os.path.join(stars_main_dir, "stars-experiments-data")
        return recording_dir

    @staticmethod
    def get_path_from_seed(seed: int, recording: bool) -> os.path:
        recording_dir = JSONHelper.get_experiment_data_folder()
        if recording:
            recording_dir = os.path.join(recording_dir, JSONHelper.RECORDINGS_RUNS_FOLDER)
        else:
            recording_dir = os.path.join(recording_dir, JSONHelper.SIMULATION_RUNS_FOLDER)
        path_list = Path(recording_dir).glob('**/*.zip')
        file_path = ""
        for path in path_list:
            # because path is object not string
            if JSONHelper.WEATHER_FILE_NAME_PREFIX not in str(path):
                file_seed = str(path).split("_seed")[1].split(".")[0]
                if file_seed == str(seed):
                    file_path = path
                    break
        return file_path

    @staticmethod
    def get_file_path_folder(folder: str = "") -> os.path:
        # Get parent directory of this file
        project_root = dirname(dirname(os.path.abspath(__file__)))
        # Move to experiments repository
        log_directory = os.path.join(project_root, "generated-data")
        if folder:
            # Path where logs folder should go
            log_directory = os.path.join(log_directory, folder)
        return log_directory

    @staticmethod
    def get_file_path_for_name(name: str, map_name: str = "", folder: str = "", log_directory: str = "",
                               file_ending: str = "json", prefix: str = "", add_time: bool = False,
                               add_date: bool = False) -> os.path:
        """
        Return the path for the log file with the given name
        :param name: The name of the log file
        :param prefix: The prefix that should be attached to the file name
        :param add_time: Decides whether the current time stamp should be appended to the file's name
        :return: path for the log file
        """
        if log_directory == "":
            log_directory = JSONHelper.get_file_path_folder(folder)
        if map_name:
            log_directory = os.path.join(log_directory, JSONHelper.clean_string(map_name))
        # Create folder if it does not exist
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        name_string = name
        if prefix != "":
            name_string = f"{prefix}_{name}"
        # Add time stamp to name if necessary
        if add_time:
            name_string += f"_{datetime.now()}"
        # Add date to name if necessary
        if add_date:
            name_string += f"_{datetime.now().date()}"
        name_string = JSONHelper.clean_string(name_string)
        # Create path for calculated name
        return os.path.join(log_directory, f"{name_string}.{file_ending}")

    @staticmethod
    def clean_string(string: str) -> str:
        return string.replace(":", "-").replace(" ", "_").replace(".", "_").replace("/", "_")

    @staticmethod
    def log_error(file_name: str, name: str, error_message: str) -> None:
        path = JSONHelper.get_file_path_for_name(name=file_name, folder=JSONHelper.ERROR_FOLDER, file_ending="txt",
                                                 add_date=True)
        print(f"Log {file_name} to", path)
        with open(path, "a") as aborted_runs:
            aborted_runs.write(f"{datetime.now()}: {name}\n")
            aborted_runs.write(f"\t\t {error_message}\n")

    @staticmethod
    def log_aborted_run(name) -> None:
        path = JSONHelper.get_file_path_for_name(name="aborted_runs", folder=JSONHelper.ERROR_FOLDER, file_ending="txt",
                                                 add_date=True)
        print("Log aborted run to", path)
        with open(path, "a") as aborted_runs:
            aborted_runs.write(f"{datetime.now()}: {name}\n")

    @staticmethod
    def log_invalid_run(name) -> None:
        path = JSONHelper.get_file_path_for_name(name="invalid_runs", folder=JSONHelper.ERROR_FOLDER, file_ending="txt",
                                                 add_date=True)
        print("Log invalid run to", path)
        with open(path, "a") as aborted_runs:
            aborted_runs.write(f"{datetime.now()}: {name}\n")

    @staticmethod
    def log_failed_carla_run(name) -> None:
        path = JSONHelper.get_file_path_for_name(name="failed_carla_runs", folder=JSONHelper.ERROR_FOLDER,
                                                 file_ending="txt",
                                                 add_date=True)
        print("Log aborted run to", path)
        with open(path, "a") as aborted_runs:
            aborted_runs.write(f"{datetime.now()}: {name}\n")

    @staticmethod
    def log_tick_data(ticks: List[TickData], path: os.path) -> None:
        # Override existing files
        with open(path, "w") as logfile:
            json_string = TickData.list_to_json(ticks)
            logfile.write(json_string)

    @staticmethod
    def log_data_blocks(blocks: List[DataBlock], path: os.path) -> None:
        # Override existing files
        with open(path, "w") as logfile:
            json_string = DataBlock.list_to_json(blocks)
            logfile.write(json_string)

    @staticmethod
    def log_weather(weather_params: DataWeatherParameters, path: os.path) -> None:
        # Override existing files
        with open(path, "w") as logfile:
            json_string = DataBlock.to_json(weather_params)
            logfile.write(json_string)

    @staticmethod
    def load_data_blocks(path: os.path) -> [DataBlock]:
        with open(path) as logfile:
            data = json.loads(logfile.read())
            # TODO Check for invalid json files
            return DataBlock.from_list(data)

    @staticmethod
    def load_tick_data(path: os.path) -> List[TickData]:
        with open(path) as logfile:
            data = json.loads(logfile.read())
            return TickData.from_list(data)

    @staticmethod
    def load_weather(path: os.path) -> DataWeatherParameters:
        with open(path, encoding="utf8") as logfile:
            data = json.loads(logfile.read())
            return DataWeatherParameters.from_dict(data)

    @staticmethod
    def zip_and_delete_file(path: os.path) -> None:
        JSONHelper.zip_file(path)
        JSONHelper.delete_file(path)

    @staticmethod
    def zip_file(path: os.path) -> None:
        try:
            import zlib
            compression = zipfile.ZIP_DEFLATED
        except:
            compression = zipfile.ZIP_STORED

        modes = {zipfile.ZIP_DEFLATED: 'deflated',
                 zipfile.ZIP_STORED: 'stored',
                 }
        print('creating archive')
        zip_file_path = os.path.splitext(path)[0] + ".zip"
        archive_name = os.path.basename(path)
        zf = zipfile.ZipFile(zip_file_path, mode='w')
        try:
            print(f'adding {path}', modes[compression])
            zf.write(path, arcname=archive_name, compress_type=compression)
        finally:
            print('closing')
            zf.close()

    @staticmethod
    def delete_file(path: os.path) -> None:
        os.remove(path)

    @staticmethod
    def extract_from_zip(path: os.path):
        zf = zipfile.ZipFile(path)
        dir_name = os.path.dirname(path)
        zf.extractall(dir_name)
