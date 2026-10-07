# layers/midpoints.py
import numpy as np
import pandas as pd
import plotly.graph_objects as go

from carla_data_classes.static.DataWorld import DataWorld
from .base_layer import register, BaseLayer
from .utils import color_for_road  # pick up the same road colors as lanes :contentReference[oaicite:3]{index=3}


@register("midpoints")
class MidpointLayer(BaseLayer):
    slider_key = "midpoints"
    df_key = "midpoints"

    @classmethod
    def build_df(cls, data_world: DataWorld, tick) -> pd.DataFrame:
        rows = []
        for r in data_world.get_all_roads():
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

        # Merge across all lanes sharing the same color into one marker trace and
        # one arrow trace per color, instead of one lane -> two traces (which was
        # thousands of traces on a large map): Plotly.js carries a meaningful
        # fixed cost per trace on every pan/zoom regardless of trace content, so
        # trace *count* is what makes interaction slow, not point count. Hover
        # still works per-point via customdata, which already varies per point.
        by_color = {}  # color -> dict(pt_x, pt_y, pt_cd, arr_x, arr_y, arr_cd)
        for (road_id, lane_id), grp in df.groupby(["road_id", "lane_id"], sort=False):
            color = color_for_road(road_id)
            bucket = by_color.setdefault(color, dict(pt_x=[], pt_y=[], pt_cd=[], arr_x=[], arr_y=[], arr_cd=[]))

            cd = grp[["road_id", "lane_id", "dist", "yaw"]].to_numpy()
            bucket["pt_x"].append(grp.x.to_numpy())
            bucket["pt_y"].append(grp.y.to_numpy())
            bucket["pt_cd"].append(cd)

            yaw_rad = np.deg2rad(grp.yaw.to_numpy())
            x2 = grp.x.to_numpy() + np.cos(yaw_rad)
            y2 = grp.y.to_numpy() + np.sin(yaw_rad)
            bucket["arr_x"].append(np.column_stack([grp.x, x2, np.full(len(grp), np.nan)]).ravel())
            bucket["arr_y"].append(np.column_stack([grp.y, y2, np.full(len(grp), np.nan)]).ravel())
            bucket["arr_cd"].append(np.repeat(cd, 3, axis=0))

        traces = []
        for color, b in by_color.items():
            traces.append(go.Scattergl(
                x=np.concatenate(b["pt_x"]), y=np.concatenate(b["pt_y"]), mode="markers",
                marker=dict(size=self.size["midpoints"], color=color),
                customdata=np.concatenate(b["pt_cd"]),
                hovertemplate=tpl,
                hoverlabel=dict(bgcolor=color),
                name="Midpoints",
                showlegend=False,
            ))
            traces.append(go.Scattergl(
                x=np.concatenate(b["arr_x"]), y=np.concatenate(b["arr_y"]), mode="lines",
                line=dict(width=1, color=color),
                customdata=np.concatenate(b["arr_cd"]),
                hovertemplate=tpl,
                hoverlabel=dict(bgcolor=color),
                showlegend=False,
            ))

        return traces
