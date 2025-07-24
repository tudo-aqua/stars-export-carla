import numpy as np
import plotly.graph_objects as go

from .base_layer import register, BaseLayer
from shapely.geometry import LineString

from .utils import rgba, color_for_road


@register("roads")
class RoadLayer(BaseLayer):
    """
    For every lane centre‑line, create a polygon corridor with width = lane_width,
    then draw it with fill="toself". Hover works on the filled area.
    Uses the lane DataFrame prepared by LaneLayer (df_key='lanes').
    """
    df_key = "lanes"

    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        max_abs_lane = df.lane_id.abs().max() or 1
        traces = []
        for _, row in df.iterrows():
            base_color = color_for_road(row.road_id)
            opacity = max(0.15, 1 - abs(row.lane_id) / max_abs_lane)
            fill_color = rgba(base_color, opacity)

            poly = np.asarray(row.poly)  # (N,2) array
            line = LineString(poly)
            radius = row.width / 2.0
            corridor = line.buffer(radius, cap_style=2, join_style=2)
            xs, ys = map(np.asarray, corridor.exterior.xy)

            traces.append(
                go.Scatter(
                    x=xs,
                    y=ys,
                    mode="lines",
                    name=f"Road: {row.road_id} Lane: {row.lane_id}",
                    text=f"Road: {row.road_id}<br>"
                         f"Lane: {row.lane_id}<br>"
                         "───────────────<br>"
                         f"Type: {row.lane_type}<br>"
                         f"Width: {row.width:.2f} m<br>"
                         f"Length: {row.length:.2f} m<br>",
                    line=dict(width=1.5, color=base_color),
                    fill="toself",
                    fillcolor=fill_color,
                    hoveron="fills",
                    hoverlabel=dict(bgcolor=base_color,namelength=0),
                )
            )

        return traces
