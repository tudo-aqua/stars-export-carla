# layers/base.py
from __future__ import annotations
from abc import ABC, abstractmethod
from typing import List, Dict, Type, TYPE_CHECKING
import plotly.graph_objects as go
import pandas as pd

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


def build_all_traces(store, visible_layers: List[str], size_cfg: Dict[str, int]):
    """
    Iterate over every registered layer, let it build its traces, and
    return (list_of_traces, layer_map).

    layer_map maps layer_name -> list[trace_index] so GUI callbacks can
    toggle visibility, patch sizes, etc.
    """
    traces: List[go.BaseTraceType] = []
    layer_map: Dict[str, List[int]] = {}

    for name, LayerCls in LAYER_REGISTRY.items():
        layer_obj = LayerCls(store, size_cfg)
        layer_traces = layer_obj.traces()

        # set initial visibility according to `visible_layers`
        for tr in layer_traces:
            tr.visible = name in visible_layers

        start_idx = len(traces)
        traces.extend(layer_traces)
        layer_map[name] = list(range(start_idx, len(traces)))

    return traces, layer_map


class BaseLayer(ABC):
    """Every layer subclass gets the ViewerStore and returns traces."""
    layer_name: str  # set by decorator
    slider_key: str | None = None  # set if layer has a size/width slider
    default_size: int = 6 # set default value of the size/width slider
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
