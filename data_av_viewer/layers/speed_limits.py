# layers/speed_limits.py
import numpy as np, pandas as pd, plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("speed_limits")
class SpeedLimitLayer(BaseLayer):
    df_key="speed_segs"

    @classmethod
    def build_df(cls, blocks, tick):
        sx,sy=[],[]
        for ln in (l for b in blocks for r in b.roads for l in r.lanes):
            for sl in ln.speed_limits or []:
                pts=[mp for mp in ln.lane_midpoints or []
                     if sl.from_distance<=mp.distance_to_start<=sl.to_distance]
                if len(pts)<2: continue
                sx.extend([p.location.x for p in pts]+[np.nan])
                sy.extend([p.location.y for p in pts]+[np.nan])
        return pd.DataFrame(dict(x=sx, y=sy))

    def traces(self):
        df=self.get_df(self.df_key)
        if df.empty: return []
        tr=go.Scattergl(
            x=df.x, y=df.y, mode="lines",
            name="Speed limits", line=dict(width=3,dash="dot"),
            hoverinfo="skip"
        )
        return [tr]
