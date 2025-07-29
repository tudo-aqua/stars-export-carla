# dynamic/renderers/vehicle.py
from __future__ import annotations
import numpy as np
from shapely.geometry import box as shp_box
import plotly.graph_objects as go
from layers.utils import rgba
from carla_data_classes import DataActor
from . import register, BaseRenderer

# both Vehicles **and** Pedestrians → square bounding box with opaque fill
@register("square_bbox")
class VehicleRenderer(BaseRenderer):

    @classmethod
    def matches(cls, actor: DataActor) -> bool:
        return actor.type in {"Vehicle", "Pedestrian"}

    # ------------------------------------------------------------------
    @staticmethod
    def _square_xy(actor: DataActor):
        bb = actor.bounding_box
        if bb:                       # use real bb
            xs = [v.x for v in bb.vertices] + [bb.vertices[0].x]
            ys = [v.y for v in bb.vertices] + [bb.vertices[0].y]
        else:                        # tiny 1×1 m square around point
            x, y = actor.location.x, actor.location.y
            half = 0.5
            xs, ys = zip(*shp_box(x-half, y-half, x+half, y+half).exterior.coords)
        return np.asarray(xs), np.asarray(ys)

    # ------------------------------------------------------------------
    @classmethod
    def make_template(cls, actor: DataActor, base_color: str) -> go.Scatter:
        xs, ys = cls._square_xy(actor)
        return go.Scatter(
            x=xs, y=ys,
            mode="lines",
            fill="toself",
            line=dict(width=1.0, color=base_color),
            fillcolor=rgba(base_color, .45),
            name=f"{actor.type.lower()} {actor.id}",
            hoverinfo="text"
        )

    # ------------------------------------------------------------------
    @classmethod
    def frame_payload(cls, actor: DataActor):
        xs, ys = cls._square_xy(actor)
        txt = (f"ID {actor.id}<br>"
               f"{actor.type}: {actor.type_id}<br>"
               f"Speed: {getattr(actor,'velocity',None) and round(actor.velocity.x,1)} m/s")
        return xs, ys, txt, {}
