# layers/contact_areas.py
import pandas as pd, plotly.graph_objects as go
from .base_layer import register, BaseLayer

@register("contact_areas")
class ContactAreaLayer(BaseLayer):
    slider_key= "contact_areas"
    df_key="contact_areas"

    @classmethod
    def build_df(cls, blocks, tick):
        rows=[]
        for ln in (l for b in blocks for r in b.roads for l in r.lanes):
            for c in ln.contact_areas or []:
                rows.append(dict(
                    x=c.contact_location.x, y=c.contact_location.y,
                    id=c.id, l1=f"{c.lane_1_road_id}/{c.lane_1_id}",
                    l2=f"{c.lane_2_road_id}/{c.lane_2_id}"
                ))
        return pd.DataFrame(rows)

    def traces(self):
        df=self.get_df(self.df_key)
        if df.empty: return []
        tpl=("ID:%{customdata[0]}<br>%{customdata[1]}<br>%{customdata[2]}<br>"
             "x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        trace=go.Scattergl(
            x=df.x, y=df.y, mode="markers",
            marker=dict(size=self.size["contact_areas"], symbol="x"),
            customdata=df[["id","l1","l2"]].to_numpy(),
            hovertemplate=tpl, hoverlabel=dict(bgcolor="#9467bd")
        )
        return [trace]
