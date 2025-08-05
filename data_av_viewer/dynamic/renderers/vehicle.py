# dynamic/renderers/vehicle.py

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from carla_data_classes.dynamic import DataActor, DataVehicle
from helpers.kinematics import actor_speed_kmh, actor_accel_mps2
from . import register, BaseRenderer


def _footprint_xy_from_vertices(verts):
    """Return a single XY loop for the bottom face (4 lowest-z verts)."""
    if not verts or len(verts) < 4:
        return None
    pts = np.array([(v.x, v.y, v.z) for v in verts], dtype=float)
    idx = np.argsort(pts[:, 2])[:4]
    bottom = pts[idx, :2]
    c = bottom.mean(axis=0)
    ang = np.arctan2(bottom[:, 1] - c[1], bottom[:, 0] - c[0])
    order = np.argsort(ang)
    poly = bottom[order]
    poly = np.vstack([poly, poly[0]])  # close
    return poly[:, 0], poly[:, 1]

def _extract_road_lane(actor_position) -> tuple[str, str]:
    """
    Best-effort extraction of (road_id, lane_id) from the ActorPosition.
    Returns ('n/a','n/a') if not available.
    """
    rid = getattr(actor_position, "road_id", None)
    lid = getattr(actor_position, "lane_id", None)

    # Try nested objects some pipelines use
    if rid is None:
        rid = getattr(getattr(actor_position, "lane_position", None), "road_id", None)
    if lid is None:
        lid = getattr(getattr(actor_position, "lane_position", None), "lane_id", None)

    if rid is None:
        rid = getattr(getattr(actor_position, "lane", None), "road_id", None)
    if lid is None:
        lid = getattr(getattr(actor_position, "lane", None), "lane_id", None)

    # Stringify for hover
    return (str(rid) if rid is not None else "n/a",
            str(lid) if lid is not None else "n/a")

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
    def frame_payload(cls, actor: DataActor, actor_position=None):
        """
        Return per-frame payload: polygon (xs, ys), hover text (with road/lane),
        and an empty style dict (always black).
        """
        xs, ys = _footprint_xy_from_vertices(getattr(actor.bounding_box, "vertices", []))
        if xs is None:
            x, y = actor.location.x, actor.location.y
            half = 0.5
            xs = np.array([x - half, x + half, x + half, x - half, x - half])
            ys = np.array([y - half, y - half, y + half, y + half, y - half])

        # Road/Lane from the ActorPosition if provided
        if actor_position is not None:
            road_str, lane_str = _extract_road_lane(actor_position)
        else:
            road_str, lane_str = "n/a", "n/a"

        # Build hover text
        lines = [
            f"Vehicle Id: {actor.id}",
            f"{actor.type}: {actor.type_id}",
            f"Road: {road_str}  Lane: {lane_str}",
        ]

        if getattr(actor, "attributes", None):
            lines.append("Attributes:")
            for name, val in actor.attributes.items():
                lines.append(f"&nbsp;&nbsp;{name}: {val!r}")

        lines.append(f"Velocity: {actor_speed_kmh(actor):.2f} km/h")
        lines.append(f"Acceleration: {actor_accel_mps2(actor):.2f} m/s²")
        lines.append(f"X: {actor.location.x:.2f} Y: {actor.location.y:.2f} Z: {actor.location.z:.2f}")

        txt = "<br>".join(lines) + "<br>"
        return xs, ys, txt, {}
