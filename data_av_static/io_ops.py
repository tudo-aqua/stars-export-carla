import os
import zipfile
from pathlib import Path
from typing import TYPE_CHECKING

from scipy.spatial import KDTree

from carla_data_classes.static.DataMap import DataMap
from helpers.json_helper import JSONHelper

if TYPE_CHECKING:
    pass


class _IOOps:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    def save_data_map(self, file_path: str) -> None:
        """
        Saves the blocks created for the current map to disk.
        """
        if not self.ctx.data_map:
            raise RuntimeError("The map has not yet been calculated. Use method 'get_data_map'")
        JSONHelper.log_data_map(self.ctx.data_map, file_path)

    def load_data_map(self, file_path: Path) -> DataMap:
        """
        — Checks that exactly one static_data_*.zip exists in `log_file_path`.
        — Extracts and loads all JSON files inside into a dict.
        Returns:
            Dict[str, Any]: mapping from JSON filename → parsed JSON content.
        Raises:
            FileNotFoundError: if no matching zip is found.
            ValueError: if more than one matching zip is found,
                        or if the zip contains no .json files.
        """

        data_map = None
        with zipfile.ZipFile(file_path, 'r') as zf:
            # find all JSON entries
            json_files = [n for n in zf.namelist() if n.lower().endswith('.json')]
            if not json_files:
                raise ValueError(f"No .json files inside {file_path.name}")

            for name in json_files:
                with zf.open(name) as f:
                    data_map = JSONHelper.load_data_map(f)

        return data_map

    def load_or_calculate_data_map(self, log_file_path: str, map_name: str) -> DataMap:
        """
        Loads the DataMap for the current map if existing. Otherwise, they are calculated and
        saved to disk.
        """
        if self.ctx.data_map:
            print(">> [Data-AV Transformer] Map is already calculated. Nothing new to be done.")
            return self.ctx.data_map
        log_dir = Path(log_file_path)
        # Look for any file named static_data_*.zip
        matches = list(log_dir.glob(f"static_data_{map_name}.zip"))
        # Optionally, ensure they’re actual files
        if any(p.is_file() for p in matches):
            # Load the collected static data from the json file
            print(f">> [Data-AV Transformer] The map data was already calculated.")
            print(">> [IO] Load map data from file: '{matches[0]}'")
            self.ctx.data_map = self.load_data_map(matches[0])
        else:
            self.ctx.data_map = self.ctx.get_data_map()
            save_file_name = os.path.join(log_file_path, f"{JSONHelper.STATIC_FILE_NAME_PREFIX}_{map_name}.json")
            self.save_data_map(file_path=save_file_name)
            JSONHelper.zip_and_delete_file(save_file_name)

        self.ctx.lane_midpoints = self.ctx.get_lane_midpoints_array()
        lane_midpoint_locations = list(
            map(lambda l: (l.location.x, l.location.y, l.location.z), self.ctx.lane_midpoints))
        self.ctx.kd_tree = KDTree(lane_midpoint_locations)
        return self.ctx.data_map
