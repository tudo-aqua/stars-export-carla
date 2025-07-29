# dynamic/renderers/traffic_light.py
from __future__ import annotations
import numpy as np
import plotly.graph_objects as go
from carla_data_classes import DataActor, DataTrafficLight
from . import register, BaseRenderer

# Carla TL state → colour map
_TL_COLOUR = {0:"#f00", 1:"#ff0", 2:"#0f0", 3:"#0f0", 4:"#777"}   # Fallback/Off = grey

@register("traffic_light")
class TrafficLightRenderer(BaseRenderer):

    @classmethod
    def matches(cls, actor: DataActor) -> bool:
        return actor.type == "TrafficLight"

    # ------------------------------------------------------------------
    @classmethod
    def make_template(cls, actor: DataTrafficLight, base_color) -> go.Scatter:
        # template = a single marker whose *colour will be patched* each frame
        return go.Scatter(
            x=[actor.location.x],
            y=[actor.location.y],
            mode="markers",
            marker=dict(size=10, color=_TL_COLOUR.get(actor.state, "#777")),
            name=f"TL {actor.id}",
            hoverinfo="text"
        )

    # ------------------------------------------------------------------
    @classmethod
    def frame_payload(cls, actor: DataTrafficLight):
        xs = np.asarray([actor.location.x])
        ys = np.asarray([actor.location.y])
        txt = f"Traffic‑Light {actor.id}<br>State: {actor.state}"
        style = {"marker.color": _TL_COLOUR.get(actor.state, "#777")}
        return xs, ys, txt, style
