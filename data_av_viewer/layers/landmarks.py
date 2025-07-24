# layers/landmarks.py
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("landmarks")
class LandmarkLayer(BaseLayer):
    slider_key = "landmarks"   # gives a size slider
    df_key     = "landmarks"

    # ---------------------------------------------------------------- build df
    @classmethod
    def build_df(cls, blocks, tick):
        rows = []
        for ln in (l for b in blocks for r in b.roads for l in r.lanes):
            for lm in ln.landmarks or []:
                rows.append(dict(
                    x          = lm.location.x,
                    y          = lm.location.y,
                    id         = lm.id,
                    road_id    = lm.road_id,
                    name       = lm.name,
                    orientation= lm.orientation.name,   # enum → text
                    country    = lm.country,
                    text       = lm.text,
                    value      = lm.value,
                    sub_type   = lm.sub_type,
                    type       = lm.type.name,
                    yaw        = lm.rotation.yaw        # for arrow
                ))
        return pd.DataFrame(rows)

    # ---------------------------------------------------------------- traces
    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        # -------- marker trace (one trace for all landmarks) ---------
        custom = df[[
            "id", "road_id", "name", "orientation",
            "country", "text", "value", "sub_type", "type"
        ]].to_numpy()

        hover_tpl = (
            "ID: %{customdata[0]}<br>"
            "Road: %{customdata[1]}<br>"
            "Name: %{customdata[2]}<br>"
            "Orientation: %{customdata[3]}<br>"
            "Country: %{customdata[4]}<br>"
            "Text: %{customdata[5]}<br>"
            "Value: %{customdata[6]}<br>"
            "Sub‑type: %{customdata[7]}<br>"
            "Type: %{customdata[8]}<br>"
            "x:%{x:.2f} y:%{y:.2f}<extra></extra>"
        )

        marker_trace = go.Scattergl(
            x=df.x, y=df.y, mode="markers",
            marker=dict(size=self.size["landmarks"], symbol="circle"),
            customdata=custom,
            hovertemplate=hover_tpl,
            hoverlabel=dict(bgcolor="#d62728"),
            name="Landmarks",
            showlegend=True
        )

        # -------- orientation arrows --------------------------------
        arrow_len = 3.0  # metres
        yaw_rad = np.deg2rad(df.yaw.to_numpy())
        x2 = df.x + np.cos(yaw_rad) * arrow_len
        y2 = df.y + np.sin(yaw_rad) * arrow_len

        seg_x = np.column_stack([df.x, x2, np.full(len(df), np.nan)]).ravel()
        seg_y = np.column_stack([df.y, y2, np.full(len(df), np.nan)]).ravel()

        arrow_trace = go.Scattergl(
            x=seg_x, y=seg_y, mode="lines",
            line=dict(width=1.5, color="#d62728"),
            hoverinfo="skip",               # hover on marker is enough
            showlegend=False
        )

        return [marker_trace, arrow_trace]
