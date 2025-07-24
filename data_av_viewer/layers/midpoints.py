# layers/midpoints.py
import numpy as np, pandas as pd, plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("midpoints")
class MidpointLayer(BaseLayer):
    slider_key = "midpoints"   # will get a size slider
    df_key = "midpoints"

    # ---------------------------------------------------------- build df
    @classmethod
    def build_df(cls, blocks, tick) -> pd.DataFrame:
        rows=[]
        for b in blocks:
            for r in b.roads:
                for ln in r.lanes:
                    for mp in ln.lane_midpoints or []:
                        rows.append(dict(
                            x=mp.location.x, y=mp.location.y, yaw=mp.rotation.yaw,
                            road_id=mp.road_id, lane_id=mp.lane_id,
                            dist=mp.distance_to_start))
        return pd.DataFrame(rows)

    # ---------------------------------------------------------- traces
    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        tpl = ("Road:%{customdata[0]} Lane:%{customdata[1]}<br>"
               "Dist:%{customdata[2]:.2f} m<br>"
               "Yaw:%{customdata[3]:.1f}°<br>"
               "x:%{x:.2f} y:%{y:.2f}<extra></extra>")

        pt = go.Scattergl(
            x=df.x, y=df.y, mode="markers",
            marker=dict(size=self.size["midpoints"]),
            customdata=df[["road_id","lane_id","dist","yaw"]].to_numpy(),
            hovertemplate=tpl, hoverlabel=dict(bgcolor="#1f77b4")
        )

        # arrows
        yaw_rad = np.deg2rad(df.yaw)
        x2 = df.x + np.cos(yaw_rad)
        y2 = df.y + np.sin(yaw_rad)
        seg_x = np.column_stack([df.x, x2, np.full(len(df), np.nan)]).ravel()
        seg_y = np.column_stack([df.y, y2, np.full(len(df), np.nan)]).ravel()
        arrow = go.Scattergl(
            x=seg_x, y=seg_y, mode="lines",
            line=dict(width=1, color="#1f77b4"),
            customdata=np.repeat(pt.customdata, 3, axis=0),
            hovertemplate=tpl, hoverlabel=dict(bgcolor="#1f77b4")
        )
        return [pt, arrow]
