# layers/contact_areas.py
import pandas as pd
import plotly.graph_objects as go

from carla_data_classes.static.DataWorld import DataWorld
from .base_layer import register, BaseLayer
from .utils import color_for_road


@register("contact_areas")
class ContactAreaLayer(BaseLayer):
    slider_key = "contact_areas"
    df_key = "contact_areas"

    # -------- build DataFrame -------------------------------------------------
    @classmethod
    def build_df(cls, data_map: DataWorld, tick):
        rows = []
        for ln in (l for b in data_map.junctions for r in b.roads for l in r.lanes):
            for c in ln.contact_areas or []:
                rows.append(dict(
                    x=c.contact_location.x,
                    y=c.contact_location.y,
                    id=c.id,
                    lane_1_road_id=c.lane_1_road_id,
                    lane_1_lane_id=c.lane_1_id,
                    lane_2_road_id=c.lane_2_road_id,
                    lane_2_lane_id=c.lane_2_id,
                ))

        # one row per unique contact‑area ID
        return pd.DataFrame(rows).drop_duplicates("id")

    # -------- build Plotly traces --------------------------------------------
    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        traces = []
        for _, row in df.iterrows():
            color = color_for_road(row.lane_1_road_id)
            hover = (f"ID: {row.id}<br>"
                     f"Lane 1: Road {row.lane_1_road_id} Lane {row.lane_1_lane_id}<br>"
                     f"Lane 2: Road {row.lane_2_road_id} Lane {row.lane_2_lane_id}<br>"
                     f"X: {row.x:.2f} Y: {row.y:.2f}<extra></extra>")

            traces.append(
                go.Scattergl(
                    x=[row.x], y=[row.y],  # one point
                    mode="markers",
                    name=str(row.id),  # shows in legend
                    marker=dict(size=self.size["contact_areas"], symbol="x", color=color),
                    hovertemplate=hover,
                    hoverlabel=dict(bgcolor=color),
                    showlegend=True
                )
            )
        return traces
