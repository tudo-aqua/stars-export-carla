# dynamic/renderers/vehicle.py

from __future__ import annotations
import numpy as np
import plotly.graph_objects as go
from carla_data_classes import DataActor, DataVehicle
from helpers.kinematics import actor_speed_kmh, actor_accel_mps2
from . import register, BaseRenderer


def _footprint_xy_from_vertices(verts):
    """Return a single XY loop for the bottom face (4 lowest-z verts)."""
    if not verts or len(verts) < 4:
        return None

    pts = np.array([(v.x, v.y, v.z) for v in verts], dtype=float)
    # take the 4 lowest z as the bottom face (works on slopes / pitches)
    idx = np.argsort(pts[:, 2])[:4]
    bottom = pts[idx, :2]

    # order polygon vertices consistently (ccw) and close the loop
    c = bottom.mean(axis=0)
    ang = np.arctan2(bottom[:, 1] - c[1], bottom[:, 0] - c[0])
    order = np.argsort(ang)
    poly = bottom[order]
    poly = np.vstack([poly, poly[0]])  # close

    return poly[:, 0], poly[:, 1]


@register("square_bbox")
class VehicleRenderer(BaseRenderer):
    """Render Vehicles as filled bounding‐boxes in black on top."""

    @classmethod
    def matches(cls, actor: DataActor) -> bool:
        return isinstance(actor, DataVehicle)

    @classmethod
    def make_template(cls, actor: DataVehicle, base_color: str):
        xs, ys = _footprint_xy_from_vertices(getattr(actor.bounding_box, "vertices", []))
        if xs is None:
            # small fallback square if bbox missing
            x, y = actor.location.x, actor.location.y
            xs = np.array([x - 0.5, x + 0.5, x + 0.5, x - 0.5, x - 0.5])
            ys = np.array([y - 0.5, y - 0.5, y + 0.5, y + 0.5, y - 0.5])

        return go.Scatter(
            x=xs,
            y=ys,
            mode="lines",
            fill="toself",
            hoveron="fills",
            hoverinfo="text",
            line=dict(width=1.0, color="black"),
            fillcolor="black",
            name=f"Vehicle {actor.id}",
        )

    @classmethod
    def frame_payload(cls, actor: DataActor):
        """
        Return the per‐frame payload for hover/text and style.
        Displays ID, type, attributes (one per line), velocity, and acceleration.
        """
        # Compute the 2D footprint (bottom 4 verts or fallback square)
        xs, ys = _footprint_xy_from_vertices(getattr(actor.bounding_box, "vertices", []))
        if xs is None:
            # fallback: 1×1 m square around the actor’s location
            x, y = actor.location.x, actor.location.y
            half = 0.5
            xs = np.array([x - half, x + half, x + half, x - half, x - half])
            ys = np.array([y - half, y - half, y + half, y + half, y - half])

        # Build hover‐text lines
        lines = [
            f"Vehicle Id: {actor.id}",
            f"{actor.type}: {actor.type_id}",
        ]

        # If attributes exist, list each on its own indented line
        if getattr(actor, "attributes", None):
            lines.append("Attributes:")
            # assume actor.attributes is a dict; adjust if it's a list
            for name, val in actor.attributes.items():
                lines.append(f"&nbsp;&nbsp;{name}: {val!r}")

        # Append velocity and acceleration
        lines.append(f"Velocity: {actor_speed_kmh(actor):.2f} km/h")
        lines.append(f"Acceleration: {actor_accel_mps2(actor):.2f} m/s²")
        lines.append(f"X: {actor.location.x:.2f} Y: {actor.location.y:.2f} Z: {actor.location.z:.2f}")

        # Join with HTML line breaks
        txt = "<br>".join(lines) + "<br>"

        # Return the XY, the hover‐text, and no per‐frame style changes
        return xs, ys, txt, {}

