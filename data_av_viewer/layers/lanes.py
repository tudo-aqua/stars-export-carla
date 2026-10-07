# layers/lanes.py  (only the traces() method changes)

from .base_layer import register, BaseLayer
from .utils import color_for_road, rgba, topology_highlight_color, LineTraceMerger


@register("lanes")
class LaneLayer(BaseLayer):
    slider_key = "lanes"
    default_size = 2

    def traces(self):
        import numpy as np
        import pandas as pd
        import plotly.graph_objects as go

        df_junctions = self.get_df("junctions")
        df_straights = self.get_df("straights")

        if df_junctions is None:
            df_junctions = pd.DataFrame()
        if df_straights is None:
            df_straights = pd.DataFrame()
        if df_junctions.empty and df_straights.empty:
            return []

        df = pd.concat([df_junctions, df_straights], ignore_index=True)

        max_abs_lane = df.lane_id.abs().max() or 1
        merger = LineTraceMerger()
        highlighted = LineTraceMerger()

        for _, row in df.iterrows():
            base = color_for_road(row.road_id)
            opacity = max(0.15, 1 - abs(row.lane_id) / max_abs_lane)
            color = rgba(base, opacity)

            topology = row.get("lane_topology", "")
            highlight_color = topology_highlight_color(topology)
            topology_line = f"⚠ {topology} Lane<br>" if topology else ""

            poly = np.asarray(row.poly, dtype=float)
            if poly.ndim != 2 or poly.shape[0] < 2:
                continue
            xs, ys = poly[:, 0], poly[:, 1]

            distances = np.asarray(row.distance_to_start, dtype=float)

            # --- intersections: prefer new DF fields; fallback to legacy computation if missing ---
            if "intersection_lanes_html" in row:
                intersections_html = row["intersection_lanes_html"]
                try:
                    intersections_count = int(row.get("intersection_lanes_count", 0))
                except Exception:
                    intersections_count = 0
            else:
                # Legacy fallback: compute from 'intersecting_lanes' if present on the row
                def _as_pair(item):
                    if hasattr(item, "road_id") and hasattr(item, "lane_id"):
                        return int(item.road_id), int(item.lane_id)
                    if isinstance(item, dict) and "road_id" in item and "lane_id" in item:
                        return int(item["road_id"]), int(item["lane_id"])
                    if isinstance(item, (tuple, list)) and len(item) >= 2:
                        return int(item[0]), int(item[1])
                    return None

                raw_il = row.get("intersecting_lanes", None)
                pairs = []
                if isinstance(raw_il, list):
                    for it in raw_il:
                        p = _as_pair(it)
                        if p is not None:
                            pairs.append(p)
                pairs = sorted(set(pairs))
                intersections_count = len(pairs)
                if pairs:
                    ilines = [f"&nbsp;&nbsp;&nbsp;&nbsp;(Road {rd}, Lane {ln})" for rd, ln in pairs]
                    intersections_html = "<br>" + "<br>".join(ilines)
                else:
                    intersections_html = "<br>&nbsp;&nbsp;&nbsp;&nbsp;—"

            if isinstance(intersections_count, (int, float)) and intersections_count > 0 and intersections_html:
                intersections_block = f"Intersections ({int(intersections_count)}):{intersections_html}<br>"
            else:
                intersections_block = ""

            header = (
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
            # per-vertex text: distance-along-lane and coordinates differ at each point
            vertex_text = [
                f"{header}Distance: {d:.2f} m<br>X:{x:.2f} Y:{y:.2f}"
                for d, x, y in zip(distances, xs, ys)
            ]

            (highlighted if highlight_color else merger).add(
                highlight_color or color, xs, ys, vertex_text
            )

        traces = []
        for color, xs, ys, text in merger.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Lanes",
                line=dict(width=self.size["lanes"], color=color),
                text=text,
                hoverinfo="text",
                hoverlabel=dict(bgcolor=color),
                showlegend=False,
            ))
        for color, xs, ys, text in highlighted.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Lanes (merge/diverge)",
                line=dict(width=max(self.size["lanes"], 4), color=color),
                text=text,
                hoverinfo="text",
                hoverlabel=dict(bgcolor=color),
                showlegend=False,
            ))

        return traces
