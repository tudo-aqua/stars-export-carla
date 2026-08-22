import math
from typing import Iterable, List, Optional, Tuple

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from shapely.geometry import LineString, Polygon, MultiPolygon

from carla_data_classes.static.DataWorld import DataWorld
from .base_layer import register, BaseLayer
from .utils import rgba, color_for_road, marking_type_name, marking_color_name, neighbor_label, LineTraceMerger


def _safe_linestring_from_coords(
        coords: Iterable[Tuple[float, float]],
        fallback_xy: Optional[Tuple[float, float]] = None,
        eps: float = 1e-3,
) -> LineString:
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

    if pts:
        x, y = pts[0]
        return LineString([(x, y), (x + eps, y + eps)])
    if fallback_xy is not None:
        x, y = float(fallback_xy[0]), float(fallback_xy[1])
        return LineString([(x, y), (x + eps, y + eps)])
    return LineString()


@register("junctions")
class JunctionLayer(BaseLayer):
    """
    Junction lanes drawn as buffered corridors. Hover lists intersecting lanes.
    """
    df_key = "junctions"
    slider_key = "junctions"
    default_size = 2

    @classmethod
    def build_df(cls, data_world: DataWorld, tick):
        rows = []
        every_n = 25
        for junction in data_world.junctions:
            for road in junction.roads:
                for ln in road.lanes:
                    if not ln.lane_midpoints:
                        continue

                    pts = ln.lane_midpoints[::every_n]
                    if pts and pts[-1] is not ln.lane_midpoints[-1]:
                        pts = pts + [ln.lane_midpoints[-1]]

                    poly = np.column_stack([[mp.location.x for mp in pts],
                                            [mp.location.y for mp in pts]])
                    distance_to_start = np.array([mp.distance_to_start for mp in pts], dtype=float)

                    pairs = sorted({(ili.road_id, ili.lane_id) for ili in (ln.intersecting_lanes or [])})
                    if pairs:
                        lines = [f"&nbsp;&nbsp;&nbsp;&nbsp;(Road {rd}, Lane {lid})" for rd, lid in pairs]
                        intersections_html = "<br>" + "<br>".join(lines)
                    else:
                        intersections_html = "<br>&nbsp;&nbsp;&nbsp;&nbsp;—"

                    rows.append(dict(
                        poly=poly,
                        junction_id=junction.junction_id,
                        distance_to_start=distance_to_start,
                        lane_type=ln.lane_type.name,
                        road_id=ln.road_id,
                        lane_id=ln.lane_id,
                        width=ln.lane_width,
                        length=ln.lane_length,
                        intersection_lanes_html=intersections_html,
                        intersection_lanes_count=len(pairs),
                        left_marking_type=marking_type_name(ln.left_lane_marking),
                        left_marking_color=marking_color_name(ln.left_lane_marking),
                        right_marking_type=marking_type_name(ln.right_lane_marking),
                        right_marking_color=marking_color_name(ln.right_lane_marking),
                        left_neighbor=neighbor_label(ln.left_lane),
                        right_neighbor=neighbor_label(ln.right_lane),
                    ))
        return pd.DataFrame(rows)

    def traces(self):
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        max_abs_lane = df.lane_id.abs().max() or 1
        filled = LineTraceMerger()
        fallback_lines = LineTraceMerger()

        for _, row in df.iterrows():
            base_color = color_for_road(row.road_id)
            opacity = max(0.15, 1 - abs(row.lane_id) / max_abs_lane)
            fill_color = rgba(base_color, opacity)

            poly = np.asarray(row.poly)
            fallback = (float(poly[0, 0]), float(poly[0, 1])) if poly.size >= 2 else None
            line = _safe_linestring_from_coords((tuple(p) for p in poly), fallback_xy=fallback)
            if line.is_empty or line.length == 0.0:
                # fallback: draw centerline
                xs, ys = map(np.asarray, line.xy)
                fallback_lines.add(base_color, xs, ys, (
                    f"Junction: {row.junction_id}<br>"
                    f"Road: {row.road_id}<br>"
                    f"Lane: {row.lane_id}<br>"
                    "───────────────<br>"
                    f"Type: {row.lane_type}<br>"
                    f"Width: {row.width:.2f} m<br>"
                    f"Length: {row.length:.2f} m<br>"
                    f"Left Lane: {row.left_neighbor}<br>"
                    f"Right Lane: {row.right_neighbor}<br>"
                    f"Intersections:{row.intersection_lanes_html}<br>"
                ))
                continue

            corridor = line.buffer(max(float(row.width) / 2.0, 1e-3), cap_style=2, join_style=2)
            polys: List[Polygon] = []
            if isinstance(corridor, Polygon):
                polys = [corridor]
            elif isinstance(corridor, MultiPolygon):
                polys = [g for g in corridor.geoms if not g.is_empty and g.exterior is not None]

            for geom in polys:
                if geom.is_empty or geom.exterior is None or geom.exterior.is_empty:
                    continue
                xs, ys = map(np.asarray, geom.exterior.xy)
                filled.add((base_color, fill_color), xs, ys, (
                    f"Road: {row.road_id}<br>"
                    f"Lane: {row.lane_id}<br>"
                    "───────────────<br>"
                    f"Type: {row.lane_type}<br>"
                    f"Width: {row.width:.2f} m<br>"
                    f"Length: {row.length:.2f} m<br>"
                    f"Left Lane: {row.left_neighbor}<br>"
                    f"Right Lane: {row.right_neighbor}<br>"
                    f"Intersections:{row.intersection_lanes_html}<br>"
                ))

        traces: List[go.Scattergl] = []
        for base_color, xs, ys, text in fallback_lines.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Junctions",
                text=text,
                hoverinfo="text",
                line=dict(width=1.5, color=base_color),
                showlegend=False,
            ))
        for (base_color, fill_color), xs, ys, text in filled.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Junctions",
                text=text,
                hoverinfo="text",
                line=dict(width=1.5, color=base_color),
                fill="toself",
                fillcolor=fill_color,
                hoverlabel=dict(bgcolor=base_color, namelength=0),
                showlegend=False,
            ))

        return traces
