# layers/landmarks.py
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("landmarks")
class LandmarkLayer(BaseLayer):
    slider_key = "landmarks"  # gives a size slider
    df_key = "landmarks"

    # ---------------------------------------------------------------- build df
    @classmethod
    def build_df(cls, blocks, tick):
        by_id = {}
        for block in blocks:
            for road in block.roads:
                for lane in road.lanes:
                    for lm in lane.landmarks or []:
                        rec = by_id.get(lm.id)
                        if rec is None:
                            rec = {
                                "x": lm.location.x,
                                "y": lm.location.y,
                                "id": lm.id,
                                "name": lm.name,
                                "orientation": lm.orientation.name,
                                "country": lm.country,
                                "text": lm.text,
                                "value": lm.value,
                                "sub_type": lm.sub_type,
                                "type": lm.type.name,
                                "lane_pairs_set": set(),  # collect (road_id, lane_id)
                            }
                            by_id[lm.id] = rec
                        # Add the lane pair that references this landmark
                        rid = getattr(lane, "road_id", getattr(road, "road_id", None))
                        rec["lane_pairs_set"].add((rid, lane.lane_id))

        rows = []
        for rec in by_id.values():
            pairs = sorted(rec.pop("lane_pairs_set"))
            # Indented, multi-line HTML for Plotly hover
            lines = ["&nbsp;&nbsp;&nbsp;&nbsp;(Road {}, Lane {})".format(r, l) for r, l in pairs]
            rec["lane_pairs_html"] = "<br>" + "<br>".join(lines) if lines else ""
            rows.append(rec)

        return pd.DataFrame(rows)

    # ---------------------------------------------------------------- traces
    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        custom = df[[
            "id", "name", "orientation",
            "country", "text", "value", "sub_type", "type",
            "lane_pairs_html"
        ]].to_numpy()

        hover_tpl = (
            "ID: %{customdata[0]}<br>"
            "Name: %{customdata[1]}<br>"
            "Orientation: %{customdata[2]}<br>"
            "Country: %{customdata[3]}<br>"
            "Text: %{customdata[4]}<br>"
            "Value: %{customdata[5]}<br>"
            "Sub-type: %{customdata[6]}<br>"
            "Type: %{customdata[7]}<br>"
            "Lanes:%{customdata[8]}<br>"
            "X:%{x:.2f} Y:%{y:.2f}<extra></extra>"
        )

        return [go.Scattergl(
            x=df.x, y=df.y, mode="markers",
            marker=dict(size=self.size["landmarks"], symbol="circle"),
            customdata=custom,
            hovertemplate=hover_tpl,
            hoverlabel=dict(bgcolor="#d62728"),
            name="Landmarks",
            showlegend=True
        )]
