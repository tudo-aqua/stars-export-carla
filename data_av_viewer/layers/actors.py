# layers/actors.py
import pandas as pd, plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("actors")
class ActorLayer(BaseLayer):
    slider_key= "actors"
    df_key="actors"

    @classmethod
    def build_df(cls, blocks, tick):
        if tick is None:                   # no dynamic data
            return pd.DataFrame()
        rows=[]
        for ap in tick.actor_positions or []:
            a=ap.actor
            rows.append(dict(
                x=a.location.x, y=a.location.y,
                id=a.id, tid=a.type_id,
                road=ap.road_id, lane=ap.lane_id,
                pos=ap.position_on_lane
            ))
        return pd.DataFrame(rows)

    def traces(self):
        df=self.get_df(self.df_key)
        if df.empty: return []
        tpl=("Actor:%{customdata[0]} (%{customdata[1]})<br>"
             "Road/Lane:%{customdata[2]}/%{customdata[3]}<br>"
             "Pos:%{customdata[4]:.2f} m<br>x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        tr=go.Scattergl(
            x=df.x,y=df.y,mode="markers",
            marker=dict(size=self.size["actors"],symbol="circle-open"),
            customdata=df[["id","tid","road","lane","pos"]].to_numpy(),
            hovertemplate=tpl, hoverlabel=dict(bgcolor="#8c564b")
        )
        return [tr]
