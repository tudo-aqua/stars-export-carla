# layers/roads.py
import numpy as np
import plotly.graph_objects as go
from .base_layer import register, BaseLayer
from .utils import color_for_road, rgba


@register("roads")
class RoadLayer(BaseLayer):
    """Uses the lanes DataFrame already built by LaneLayer."""
    df_key = "lanes"

    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty: return []
        max_abs_lane = df.lane_id.abs().max() or 1
        traces = []
        for _, row in df.iterrows():
            base = color_for_road(row.road_id)
            opacity = max(0.15, 1 - abs(row.lane_id) / max_abs_lane)
            color = rgba(base, opacity)
            poly = np.asarray(row.poly)
            xs, ys = poly[:, 0], poly[:, 1]
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name=f"Road {row.road_id} lane {row.lane_id}",
                line=dict(width=2, color=color), hovertemplate=(f"Road: {row.road_id} Lane: {row.lane_id}<br>"
                                                                "x:%{x:.2f} y:%{y:.2f}<extra></extra>"),
                hoverlabel=dict(bgcolor=color),
                showlegend=False
            ))
        return traces
