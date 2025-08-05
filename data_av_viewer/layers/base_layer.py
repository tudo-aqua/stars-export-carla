# layers/base.py
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Dict, Type, TYPE_CHECKING, Tuple

import pandas as pd
import plotly.graph_objects as go

if TYPE_CHECKING:
    from data_av_viewer.viewer_store import ViewerStore

LAYER_REGISTRY: Dict[str, "Type[BaseLayer]"] = {}


def register(name: str):
    """Decorator to register layer classes."""

    def deco(cls):
        LAYER_REGISTRY[name] = cls
        cls.layer_name = name  # convenience
        return cls

    return deco


def build_all_traces(store, visible_layers: List[str], size_cfg: Dict[str, int]) -> Tuple[
    List[go.BaseTraceType], Dict[str, List[int]], List[dict]]:
    """
    Iterate over every registered layer, let it build its traces (and shapes),
    and return (list_of_traces, layer_map, list_of_shapes).

    layer_map maps:
      - layer_name -> list[trace_index]
      - layer_name + "_shapes_range" -> [start_index, end_index] in the global shapes list
      - layer_name + "_shape_xy" -> [[x...], [y...]]  (used for resizing shapes)
    """
    traces: List[go.BaseTraceType] = []
    layer_map: Dict[str, List[int]] = {}
    shapes: List[dict] = []

    for name, LayerCls in LAYER_REGISTRY.items():
        layer_obj = LayerCls(store, size_cfg)

        # ---- traces ---------------------------------------------------
        layer_traces = layer_obj.traces()
        for tr in layer_traces:
            tr.visible = name in visible_layers
        start_idx = len(traces)
        traces.extend(layer_traces)
        layer_map[name] = list(range(start_idx, len(traces)))

        # ---- shapes (optional) ---------------------------------------
        sh_fn = getattr(layer_obj, "shapes", None)
        if callable(sh_fn):
            layer_shapes = sh_fn() or []
            if layer_shapes:
                init_vis = name in visible_layers  # ← NEW
                for s in layer_shapes:  # ← NEW
                    s["visible"] = init_vis
                    s0 = len(shapes)
                shapes.extend(layer_shapes)
                s1 = len(shapes) - 1
                layer_map[name + "_shapes_range"] = [s0, s1]

                # store the anchor points used to compute/resize the shapes
                try:
                    df = layer_obj.get_df(getattr(layer_obj, "df_key", ""))
                    if df is not None and not df.empty:
                        layer_map[name + "_shape_xy"] = [df["x"].tolist(), df["y"].tolist()]
                    else:
                        layer_map[name + "_shape_xy"] = [[], []]
                except Exception:
                    layer_map[name + "_shape_xy"] = [[], []]

    return traces, layer_map, shapes


class BaseLayer(ABC):
    """Every layer subclass gets the ViewerStore and returns traces."""
    layer_name: str  # set by decorator
    slider_key: str | None = None  # set if layer has a size/width slider
    default_size: int = 6  # set default value of the size/width slider
    df_key: str  # same as layer_name by default

    def __init__(self, store: ViewerStore, size_cfg: Dict[str, int]):
        self.viewer_store = store
        self.size = size_cfg

    # ------------- traces -------------------------------------------
    @abstractmethod
    def traces(self) -> List[go.BaseTraceType]:
        ...

    # ------------- dataframe builder --------------------------------
    @classmethod
    def build_df(cls, blocks, tick) -> pd.DataFrame:  # default = empty
        return pd.DataFrame()

    # Allow override
    def get_df(self, key: str):
        return self.viewer_store.dfs.get(key, pd.DataFrame())
