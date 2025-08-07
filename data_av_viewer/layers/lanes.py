# layers/lanes.py  (only the traces() method changes)

import numpy as np
import pandas as pd
import plotly.graph_objects as go

from carla_data_classes.static.DataMap import DataMap
from .base_layer import register, BaseLayer
from .utils import color_for_road, rgba


@register("lanes")
class LaneLayer(BaseLayer):
    df_key = "lanes"
    slider_key = "lanes"
    default_size = 2

    @classmethod
    def build_df(cls, data_map: DataMap, tick):
        rows = []
        for r in data_map.get_all_roads():
            for ln in r.lanes:
                if not ln.lane_midpoints:
                    continue
                poly = np.column_stack([[mp.location.x for mp in ln.lane_midpoints],
                                        [mp.location.y for mp in ln.lane_midpoints]])
                distance_to_start = np.column_stack([mp.distance_to_start for mp in ln.lane_midpoints])
                rows.append(dict(poly=poly,
                                 distance_to_start=distance_to_start,
                                 lane_type=ln.lane_type.name,
                                 road_id=ln.road_id,
                                 lane_id=ln.lane_id,
                                 width=ln.lane_width,
                                 length=ln.lane_length))
        return pd.DataFrame(rows)

    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        max_abs_lane = df.lane_id.abs().max() or 1
        traces = []
        for _, row in df.iterrows():
            base = color_for_road(row.road_id)
            opacity = max(0.15, 1 - abs(row.lane_id) / max_abs_lane)
            color = rgba(base, opacity)

            poly = np.asarray(row.poly)
            distances = np.asarray(row.distance_to_start)
            xs, ys = poly[:, 0], poly[:, 1]
            custom = distances.reshape(-1, 1)  # shape (N,1)

            hover_tpl = (f"Road: {row.road_id}<br>"
                         f"Lane: {row.lane_id}<br>"
                         "───────────────<br>"
                         f"Type: {row.lane_type}<br>"
                         f"Width: {row.width:.2f} m<br>"
                         f"Length: {row.length:.2f} m<br>"
                         "Distance: %{customdata[0]:.2f} m<br>"
                         "X:%{x:.2f} Y:%{y:.2f}<extra></extra>")

            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name=f"Lane {row.lane_id} on road {row.road_id}",
                line=dict(width=self.size["lanes"], color=color),
                customdata=custom,
                hovertemplate=hover_tpl,
                hoverlabel=dict(bgcolor=color)
            ))

        return traces
