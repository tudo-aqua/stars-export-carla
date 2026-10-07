# layers/crosswalks.py
from __future__ import annotations

from typing import Iterable, List, Tuple

import pandas as pd
import plotly.graph_objects as go

from carla_data_classes.static.DataWorld import DataWorld
from .base_layer import register, BaseLayer


def _ensure_closed(x, y, eps=1e-8):
    if not x:
        return x, y
    if abs(x[0] - x[-1]) > eps or abs(y[0] - y[-1]) > eps:
        x = list(x) + [x[0]]
        y = list(y) + [y[0]]
    return x, y


def _iter_crosswalk_polys(data_world: DataWorld) -> Iterable[Tuple[int, List[Tuple[float, float]]]]:
    """Yield (id, polygon_xy) for each crosswalk."""
    if hasattr(data_world, "crosswalks") and data_world.crosswalks is not None:
        cws = data_world.crosswalks
    elif hasattr(data_world, "get_crosswalks"):
        cws = data_world.get_crosswalks()
    else:
        cws = []

    for i, cw in enumerate(cws):
        if hasattr(cw, "vertices"):  # DataCrosswalk(id, vertices)
            idx = getattr(cw, "id", i)
            xy = [(v.x, v.y) for v in cw.vertices]
        else:  # list[DataLocation]
            idx = i
            xy = [(v.x, v.y) for v in (cw or [])]
        if len(xy) >= 3:
            yield idx, xy


@register("crosswalks")
class CrosswalkLayer(BaseLayer):
    slider_key = "crosswalks"
    df_key = "crosswalks"

    @classmethod
    def build_df(cls, data_world, tick) -> pd.DataFrame:
        rows = []
        for cw_id, poly_xy in _iter_crosswalk_polys(data_world):
            rows.append(dict(
                id=int(cw_id),
                poly_x=[p[0] for p in poly_xy],
                poly_y=[p[1] for p in poly_xy],
            ))
        return pd.DataFrame(rows)

    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        traces = []
        fillcolor = "rgba(255,255,255,0.95)"
        border = dict(width=1, color="rgba(0,0,0,0.55)")

        for _, row in df.iterrows():
            x, y = list(row.poly_x), list(row.poly_y)
            x, y = _ensure_closed(x, y)  # <-- close the loop

            traces.append(go.Scattergl(
                x=x, y=y, mode="lines",
                line=border, fill="toself", fillcolor=fillcolor,
                name=f"Crosswalk {row.id}",
                hovertemplate="Crosswalk %{customdata}<extra></extra>",
                customdata=[row.id] * len(x),
                showlegend=False,
            ))
        return traces
