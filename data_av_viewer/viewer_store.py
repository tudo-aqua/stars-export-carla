# viewer_store.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

import orjson
import pandas as pd

from layers.base_layer import LAYER_REGISTRY


@dataclass
class ViewerStore:
    dfs: Dict[str, pd.DataFrame] = field(default_factory=dict)

    # ---------- construction from raw data ---------------------------
    @classmethod
    def from_source(cls, data_map, tick=None):
        dfs={}
        for name, layer_cls in LAYER_REGISTRY.items():
            df = layer_cls.build_df(data_map, tick)
            dfs[name] = df
        return cls(dfs)

    # ---------- (de)serialisation ------------------------------------
    def to_json(self) -> str:
        payload = {k: df.to_dict(orient="split") for k, df in self.dfs.items()}
        return orjson.dumps(payload, option=orjson.OPT_SERIALIZE_NUMPY).decode()

    @classmethod
    def from_json(cls, js: str) -> "ViewerStore":
        data = orjson.loads(js)
        dfs = {k: pd.DataFrame(**v) for k, v in data.items()}
        return cls(dfs)
