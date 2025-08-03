# dynamic/renderers/traffic_light.py

from __future__ import annotations
import numpy as np
import math
import plotly.graph_objects as go
from carla_data_classes import DataActor, DataTrafficLight
from . import register, BaseRenderer


@register("traffic_light")
class TrafficLightRenderer(BaseRenderer):
    """
    Draws each TrafficLight actor as:
      - A filled housing rectangle
      - Three filled circles (one per lamp position),
        with only the active lamp coloured, the others grey.
    All in data coordinates so they scale together on zoom.
    """

    # CARLA state → (active lamp index, hex colour)
    _STATE_INFO = {
        0: (0, "#e74c3c"),  # Red
        1: (1, "#f1c40f"),  # Amber
        2: (2, "#2ecc71"),  # Green
    }

    @classmethod
    def matches(cls, actor: DataActor) -> bool:
        return isinstance(actor, DataTrafficLight)

    @staticmethod
    def _circle_coords(cx: float, cy: float, r: float, segments: int = 16):
        """Return closed arrays for a filled-circle polygon centered at (cx,cy)."""
        thetas = np.linspace(0, 2 * math.pi, segments, endpoint=False)
        xs = cx + r * np.cos(thetas)
        ys = cy + r * np.sin(thetas)
        return np.concatenate([xs, xs[:1]]), np.concatenate([ys, ys[:1]])

    @classmethod
    def make_templates(cls, actor: DataTrafficLight):
        # Base coordinates
        x0, y0 = actor.location.x, actor.location.y

        # Geometry parameters in *data* units
        lamp_r = 0.6  # lamp radius
        gap = 2.0 * lamp_r  # center-to-center vertical
        pad = 1.0  # housing padding
        total_w = 2 * lamp_r + 2 * pad
        total_h = 3 * (2 * lamp_r) + 2 * (gap - 2 * lamp_r) + 2 * pad

        # Housing rectangle (closed polygon)
        rect_x = [x0 - total_w / 2, x0 + total_w / 2,
                  x0 + total_w / 2, x0 - total_w / 2,
                  x0 - total_w / 2]
        rect_y = [y0 - total_h / 2, y0 - total_h / 2,
                  y0 + total_h / 2, y0 + total_h / 2,
                  y0 - total_h / 2]

        housing = go.Scatter(
            x=rect_x, y=rect_y,
            mode="lines",
            fill="toself",
            hoveron="fills",
            line=dict(color="black", width=1),
            fillcolor="#222222",
            name=f"Traffic Light {actor.id}",
            hoverinfo="text", hovertext="",
            showlegend=True
        )

        # Three grey lamp placeholders
        offsets = [+gap, 0.0, -gap]
        lamp_traces = []
        for _ in offsets:
            # compute a grey circle at that offset
            cx, cy = x0, y0  # actual positions will be set in frame_payload
            xs, ys = cls._circle_coords(cx, cy, lamp_r)
            lamp_traces.append(go.Scattergl(
                x=xs, y=ys,
                mode="lines", fill="toself",
                line=dict(color="#555555", width=0),
                fillcolor="#555555",
                hoverinfo="skip",
                showlegend=False
            ))

        return [housing, *lamp_traces]

    @classmethod
    def frame_payload(cls, actor: DataTrafficLight, ap=None):
        x0, y0 = actor.location.x, actor.location.y

        lamp_r = 0.6
        gap = 2.0 * lamp_r
        pad = 1.0
        total_w = 2 * lamp_r + 2 * pad
        total_h = 3 * (2 * lamp_r) + 2 * (gap - 2 * lamp_r) + 2 * pad

        # Recompute housing coords
        rect_x = [x0 - total_w / 2, x0 + total_w / 2,
                  x0 + total_w / 2, x0 - total_w / 2,
                  x0 - total_w / 2]
        rect_y = [y0 - total_h / 2, y0 - total_h / 2,
                  y0 + total_h / 2, y0 + total_h / 2,
                  y0 - total_h / 2]

        # Build hover text on housing
        st = actor.state
        idx_active, color_active = cls._STATE_INFO.get(st, (None, "#555555"))
        st_name = {0: "Red", 1: "Amber", 2: "Green"}.get(st, "Off")
        hover = (
            f"Traffic Light {actor.id}<br>"
            f"State: {st_name} ({st})<br>"
            f"X: {x0:.2f} Y: {y0:.2f}"
        )
        housing_payload = (rect_x, rect_y, hover, {})

        # Three lamp payloads
        lamp_payloads = []
        offsets = [+gap, 0.0, -gap]
        for idx, off in enumerate(offsets):
            cx, cy = x0, y0 + off
            xs, ys = cls._circle_coords(cx, cy, lamp_r)

            if idx == idx_active:
                fill = color_active
                line_color = color_active
            else:
                fill = "#555555"
                line_color = "#555555"

            lamp_payloads.append((
                xs, ys,  # polygon
                "",  # no hover on lamp
                {
                    "fillcolor": fill,
                    "line.color": line_color,
                    "line.width": 0
                }
            ))

        # Return housing + 3 lamps
        return [housing_payload, *lamp_payloads]
