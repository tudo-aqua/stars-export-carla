# layers/traffic_lights.py
import pandas as pd, plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("traffic_lights")
class TrafficLightLayer(BaseLayer):
    slider_key= "traffic_lights"
    df_key="tlights"

    @classmethod
    def build_df(cls, blocks, tick):
        rows=[]
        for ln in (l for b in blocks for r in b.roads for l in r.lanes):
            for t in ln.traffic_lights or []:
                rows.append(dict(x=t.location.x, y=t.location.y,
                                 od=t.open_drive_id, dist=t.position_distance))
        return pd.DataFrame(rows)

    def traces(self):
        df=self.get_df(self.df_key)
        if df.empty: return []
        tpl=("OD:%{customdata[0]}<br>Pos:%{customdata[1]:.2f} m<br>"
             "X:%{x:.2f} Y:%{y:.2f}<extra></extra>")
        tr=go.Scattergl(
            x=df.x,y=df.y,mode="markers",
            marker=dict(size=self.size["traffic_lights"],symbol="triangle-up"),
            customdata=df[["od","dist"]].to_numpy(),
            hovertemplate=tpl, hoverlabel=dict(bgcolor="#2ca02c")
        )
        return [tr]
