# layers/midpoints.py
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from .base_layer import register, BaseLayer
from .utils import color_for_road    # pick up the same road colors as lanes :contentReference[oaicite:3]{index=3}

@register("midpoints")
class MidpointLayer(BaseLayer):
    slider_key = "midpoints"
    df_key     = "midpoints"

    @classmethod
    def build_df(cls, blocks, tick) -> pd.DataFrame:
        rows=[]
        for b in blocks:
            for r in b.roads:
                for ln in r.lanes:
                    for mp in ln.lane_midpoints or []:
                        rows.append(dict(
                            x=mp.location.x,
                            y=mp.location.y,
                            yaw=mp.rotation.yaw,
                            road_id=mp.road_id,
                            lane_id=mp.lane_id,
                            dist=mp.distance_to_start
                        ))
        return pd.DataFrame(rows)

    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        tpl = (
            "Road:%{customdata[0]}  Lane:%{customdata[1]}<br>"
            "Dist:%{customdata[2]:.2f} m<br>"
            "Yaw:%{customdata[3]:.1f}°<br>"
            "X:%{x:.2f} Y:%{y:.2f}<extra></extra>"
        )

        traces = []
        # group by lane so each gets its own legend entry & color
        for (road_id, lane_id), grp in df.groupby(["road_id","lane_id"], sort=False):
            color = color_for_road(road_id)
            legend_name = f"Lane {lane_id} of Road {road_id}"

            # -- markers (visible in legend) -----------------------
            pt = go.Scattergl(
                x=grp.x, y=grp.y, mode="markers",
                marker=dict(size=self.size["midpoints"], color=color),
                customdata=grp[["road_id","lane_id","dist","yaw"]].to_numpy(),
                hovertemplate=tpl,
                hoverlabel=dict(bgcolor=color),
                name=legend_name,
                legendgroup=legend_name,
                showlegend=True
            )

            # -- arrows (same group, hidden in legend) -------------
            yaw_rad = np.deg2rad(grp.yaw.to_numpy())
            x2 = grp.x.to_numpy() + np.cos(yaw_rad)
            y2 = grp.y.to_numpy() + np.sin(yaw_rad)

            seg_x = np.column_stack([grp.x, x2, np.full(len(grp), np.nan)]).ravel()
            seg_y = np.column_stack([grp.y, y2, np.full(len(grp), np.nan)]).ravel()

            arr = go.Scattergl(
                x=seg_x, y=seg_y, mode="lines",
                line=dict(width=1, color=color),
                customdata=np.repeat(
                    grp[["road_id","lane_id","dist","yaw"]].to_numpy(), 3, axis=0
                ),
                hovertemplate=tpl,
                hoverlabel=dict(bgcolor=color),
                legendgroup=legend_name,
                showlegend=False
            )

            traces.extend([pt, arr])

        return traces
