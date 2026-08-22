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
    def build_df(cls, data_world: DataWorld, tick):
        rows = []
        for ln in (l for b in data_world.junctions for r in b.roads for l in r.lanes):
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

        # Merge all points sharing the same color into one marker trace instead
        # of one trace per point (Plotly.js carries a meaningful fixed cost per
        # trace on every pan/zoom, so trace count dominates over point count).
        by_color: dict = {}
        for _, row in df.iterrows():
            color = color_for_road(row.lane_1_road_id)
            hover = (f"ID: {row.id}<br>"
                     f"Lane 1: Road {row.lane_1_road_id} Lane {row.lane_1_lane_id}<br>"
                     f"Lane 2: Road {row.lane_2_road_id} Lane {row.lane_2_lane_id}<br>"
                     f"X: {row.x:.2f} Y: {row.y:.2f}<extra></extra>")
            bucket = by_color.setdefault(color, dict(x=[], y=[], hover=[]))
            bucket["x"].append(row.x)
            bucket["y"].append(row.y)
            bucket["hover"].append(hover)

        traces = []
        for color, b in by_color.items():
            traces.append(
                go.Scattergl(
                    x=b["x"], y=b["y"],
                    mode="markers",
                    name="Contact Areas",
                    marker=dict(size=self.size["contact_areas"], symbol="x", color=color),
                    hovertemplate=b["hover"],
                    hoverlabel=dict(bgcolor=color),
                    showlegend=False,
                )
            )
        return traces
