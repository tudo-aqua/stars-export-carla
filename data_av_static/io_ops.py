import os
import zipfile
from pathlib import Path
from typing import List, TYPE_CHECKING

from scipy.spatial import KDTree

from carla_data_classes.static.DataBlock import DataBlock
from helpers.json_helper import JSONHelper

if TYPE_CHECKING:
    from .rasterizer import MapRasterizer

class _IOOps:
    def __init__(self, ctx: "MapRasterizer"):
        self.ctx = ctx

    def save_data_blocks(self, file_path: str) -> None:
        """
        Saves the blocks created for the current map to disk.
        """
        if len(self.ctx.blocks) == 0:
            raise RuntimeError("The blocks have not yet been calculated. Use method 'get_data_blocks'")
        JSONHelper.log_data_blocks(self.ctx.blocks, file_path)

    def load_data_blocks(self, file_path: Path) -> List[DataBlock]:
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

        blocks = None
        with zipfile.ZipFile(file_path, 'r') as zf:
            # find all JSON entries
            json_files = [n for n in zf.namelist() if n.lower().endswith('.json')]
            if not json_files:
                raise ValueError(f"No .json files inside {file_path.name}")

            for name in json_files:
                with zf.open(name) as f:
                    blocks = JSONHelper.load_data_blocks(f)

        return blocks

    def load_or_calculate_data_blocks(self, log_file_path: str, map_name: str) -> List[DataBlock]:
        """
        Loads the DataBlocks for the current map if existing. Otherwise, they are calculated and
        saved to disk.
        """
        if self.ctx.blocks.__len__() > 0:
            print("Blocks are already calculated. Nothing new to be done.")
            return self.ctx.blocks
        log_dir = Path(log_file_path)
        # Look for any file named static_data_*.zip
        matches = list(log_dir.glob(f"static_data_{map_name}.zip"))
        # Optionally, ensure they’re actual files
        if any(p.is_file() for p in matches):
            # Load the collected static data from the json file
            print(f"Static data was already calculated. Load data from file: '{matches[0]}'")
            self.ctx.blocks = self.load_data_blocks(matches[0])
        else:
            self.ctx.blocks = self.ctx.get_data_blocks()
            save_file_name = os.path.join(log_file_path, f"{JSONHelper.STATIC_FILE_NAME_PREFIX}_{map_name}.json")
            self.save_data_blocks(file_path=save_file_name)
            JSONHelper.zip_and_delete_file(save_file_name)

        self.ctx.lane_midpoints = self.ctx.get_lane_midpoints_array()
        lane_midpoint_locations = list(
            map(lambda l: (l.location.x, l.location.y, l.location.z), self.ctx.lane_midpoints))
        self.ctx.kd_tree = KDTree(lane_midpoint_locations)
        return self.ctx.blocks
