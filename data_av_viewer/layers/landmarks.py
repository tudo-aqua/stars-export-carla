# layers/landmarks.py
import pandas as pd, plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("landmarks")
class LandmarkLayer(BaseLayer):
    slider_key= "landmarks"
    df_key="landmarks"

    @classmethod
    def build_df(cls, blocks, tick):
        rows=[]
        for ln in (l for b in blocks for r in b.roads for l in r.lanes):
            for lm in ln.landmarks or []:
                rows.append(dict(
                    x=lm.location.x, y=lm.location.y,
                    id=lm.id, typ=lm.type.name, sub=lm.sub_type,
                    val=lm.value, unit=lm.unit))
        return pd.DataFrame(rows)

    def traces(self):
        df=self.get_df(self.df_key)
        if df.empty: return []
        tpl=("ID:%{customdata[0]}<br>%{customdata[1]}/%{customdata[2]}<br>"
             "Value:%{customdata[3]} %{customdata[4]}<br>x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        tr=go.Scattergl(
            x=df.x,y=df.y,mode="markers",
            marker=dict(size=self.size["landmarks"],symbol="star"),
            customdata=df[["id","typ","sub","val","unit"]].to_numpy(),
            hovertemplate=tpl, hoverlabel=dict(bgcolor="#d62728")
        )
        return [tr]
