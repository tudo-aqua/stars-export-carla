import math
from typing import Iterable, List, Optional, Tuple

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from shapely.geometry import LineString, Polygon, MultiPolygon

from .base_layer import register, BaseLayer
from .utils import rgba, color_for_road, topology_highlight_color, LineTraceMerger


def _safe_linestring_from_coords(
        coords: Iterable[Tuple[float, float]],
        fallback_xy: Optional[Tuple[float, float]] = None,
        eps: float = 1e-3,
) -> LineString:
    """
    Build a LineString robustly:
      - filters non-finite coords
      - removes consecutive duplicates
      - if <2 points remain, synthesizes a tiny segment at fallback_xy (or first point)
    """
    pts: List[Tuple[float, float]] = []
    last: Optional[Tuple[float, float]] = None
    for xy in coords:
        if xy is None:
            continue
        x, y = xy
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        p = (float(x), float(y))
        if last is None or p != last:
            pts.append(p)
            last = p

    if len(pts) >= 2:
        return LineString(pts)

    # Not enough points → synthesize a tiny segment (or empty if truly nothing)
    if pts:
        x, y = pts[0]
        return LineString([(x, y), (x + eps, y + eps)])
    if fallback_xy is not None:
        x, y = float(fallback_xy[0]), float(fallback_xy[1])
        return LineString([(x, y), (x + eps, y + eps)])
    return LineString()  # empty is allowed; caller may skip


@register("roads")
class RoadLayer(BaseLayer):
    """
    For every lane centre-line, create a polygon corridor with width = lane_width,
    then draw it with fill="toself". Hover works on the filled area.
    Uses the lane DataFrame prepared by LaneLayer (df_key='lanes').
    """

    def traces(self):
        df_junctions = self.get_df("junctions")
        df_straights = self.get_df("straights")
        df = pd.concat([df_junctions, df_straights])

        max_abs_lane = df.lane_id.abs().max() or 1
        filled = LineTraceMerger()
        fallback_lines = LineTraceMerger()
        highlighted = LineTraceMerger()
        highlighted_fallback = LineTraceMerger()

        for _, row in df.iterrows():
            base_color = color_for_road(row.road_id)
            opacity = max(0.15, 1 - abs(row.lane_id) / max_abs_lane)
            fill_color = rgba(base_color, opacity)

            topology = row.get("lane_topology", "")
            highlight_color = topology_highlight_color(topology)
            outline_color = highlight_color or base_color
            topology_line = f"⚠ {topology} Lane<br>" if topology else ""

            if row.get("intersection_lanes_count", 0) and row.get("intersection_lanes_html"):
                intersections_block = (
                    f"Intersections ({int(row.intersection_lanes_count)}):"
                    f"{row.intersection_lanes_html}<br>"
                )
            else:
                intersections_block = ""

            hover_text = (
                f"Road: {row.road_id}<br>"
                f"Lane: {row.lane_id}<br>"
                "───────────────<br>"
                f"Type: {row.lane_type}<br>"
                f"{topology_line}"
                f"Width: {row.width:.2f} m<br>"
                f"Length: {row.length:.2f} m<br>"
                f"Left Lane: {row.get('left_neighbor', '—')}<br>"
                f"Right Lane: {row.get('right_neighbor', '—')}<br>"
                f"Preceding Lane(s): {row.get('predecessor_lanes', '—')}<br>"
                f"Following Lane(s): {row.get('successor_lanes', '—')}<br>"
                f"{'Overlapping Lane(s): ' + row.get('overlapping_lanes', '—') + '<br>' if topology else ''}"
                f"{intersections_block}"
            )

            # row.poly -> robust centerline
            poly = np.asarray(row.poly)  # (N,2)
            fallback = (float(poly[0, 0]), float(poly[0, 1])) if poly.size >= 2 else None
            line = _safe_linestring_from_coords((tuple(p) for p in poly), fallback_xy=fallback)

            # Skip if we still have nothing useful
            if line.is_empty or line.length == 0.0:
                continue

            # Compute corridor; guard against non-positive widths
            width = float(row.width) if row.width is not None else 0.0
            radius = max(width / 2.0, 1e-3)  # tiny radius avoids degenerate buffers

            corridor = line.buffer(radius, cap_style=2, join_style=2)
            if corridor.is_empty:
                # As a fallback, draw just the centerline
                xs, ys = map(np.asarray, line.xy)
                (highlighted_fallback if highlight_color else fallback_lines).add(outline_color, xs, ys, hover_text)
                continue

            # Corridor can be Polygon or MultiPolygon
            polys: List[Polygon] = []
            if isinstance(corridor, Polygon):
                polys = [corridor]
            elif isinstance(corridor, MultiPolygon):
                polys = [g for g in corridor.geoms if not g.is_empty and g.exterior is not None]
            else:
                # Unknown geometry type; skip gracefully
                continue

            for geom in polys:
                if geom.is_empty or geom.exterior is None or geom.exterior.is_empty:
                    continue
                xs, ys = map(np.asarray, geom.exterior.xy)
                (highlighted if highlight_color else filled).add((outline_color, fill_color), xs, ys, hover_text)

        traces: List[go.Scattergl] = []
        for base_color, xs, ys, text in fallback_lines.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Roads",
                text=text,
                hoverinfo="text",
                line=dict(width=1.5, color=base_color),
                showlegend=False,
            ))
        for (base_color, fill_color), xs, ys, text in filled.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Roads",
                text=text,
                hoverinfo="text",
                line=dict(width=1.5, color=base_color),
                fill="toself",
                fillcolor=fill_color,
                hoverlabel=dict(bgcolor=base_color, namelength=0),
                showlegend=False,
            ))
        for base_color, xs, ys, text in highlighted_fallback.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Roads (merge/diverge)",
                text=text,
                hoverinfo="text",
                line=dict(width=3.5, color=base_color),
                showlegend=False,
            ))
        for (base_color, fill_color), xs, ys, text in highlighted.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Roads (merge/diverge)",
                text=text,
                hoverinfo="text",
                line=dict(width=3.5, color=base_color),
                fill="toself",
                fillcolor=fill_color,
                hoverlabel=dict(bgcolor=base_color, namelength=0),
                showlegend=False,
            ))

        return traces
