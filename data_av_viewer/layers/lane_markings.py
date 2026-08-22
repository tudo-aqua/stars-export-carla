# layers/lane_markings.py
from .base_layer import register, BaseLayer
from .utils import color_for_marking, dash_for_marking, LineTraceMerger


def _line_xy(geom):
    """
    Flatten a LineString or MultiLineString into (xs, ys) numpy arrays, with a
    NaN gap between disconnected parts so Plotly draws them as separate segments
    within a single trace. offset_curve() commonly splits into multiple parts on
    curved/kinked centerlines (e.g. highway curves), where it can no longer
    produce one continuous line. Returns (None, None) for unsupported geometry
    (e.g. an empty GeometryCollection).
    """
    import numpy as np

    if geom.geom_type == "LineString":
        parts = [geom]
    elif geom.geom_type == "MultiLineString":
        parts = list(geom.geoms)
    else:
        return None, None

    xs_parts, ys_parts = [], []
    for i, part in enumerate(parts):
        if part.is_empty:
            continue
        if i > 0 and xs_parts:
            xs_parts.append(np.array([np.nan]))
            ys_parts.append(np.array([np.nan]))
        px, py = part.xy
        xs_parts.append(np.asarray(px))
        ys_parts.append(np.asarray(py))

    if not xs_parts:
        return None, None
    return np.concatenate(xs_parts), np.concatenate(ys_parts)


@register("lane_markings")
class LaneMarkingLayer(BaseLayer):
    """
    Draws the left/right lane-boundary markings (solid/broken, white/yellow, ...)
    offset from each lane's centerline by half the lane width. Uses the same
    'junctions'/'straights' DataFrames as LaneLayer/RoadLayer.
    """
    slider_key = "lane_markings"
    default_size = 2

    def traces(self):
        import numpy as np
        import pandas as pd
        import plotly.graph_objects as go
        from shapely import LineString

        df_junctions = self.get_df("junctions")
        df_straights = self.get_df("straights")

        if df_junctions is None:
            df_junctions = pd.DataFrame()
        if df_straights is None:
            df_straights = pd.DataFrame()
        if df_junctions.empty and df_straights.empty:
            return []

        df = pd.concat([df_junctions, df_straights], ignore_index=True)
        merger = LineTraceMerger()

        for _, row in df.iterrows():
            poly = np.asarray(row.poly, dtype=float)
            if poly.ndim != 2 or poly.shape[0] < 2:
                continue

            centerline = LineString(poly)
            if centerline.length == 0.0:
                continue

            half_width = max(float(row.width) / 2.0, 1e-3)

            # sign follows GEOS offset_curve convention: positive = left of the
            # line's direction of travel, negative = right
            for side, sign, type_col, color_col in (
                    ("Left", 1.0, "left_marking_type", "left_marking_color"),
                    ("Right", -1.0, "right_marking_type", "right_marking_color"),
            ):
                marking_type = row[type_col]
                if marking_type == "NONE":
                    continue

                try:
                    boundary = centerline.offset_curve(sign * half_width)
                except Exception:
                    continue
                if boundary.is_empty:
                    continue

                xs, ys = _line_xy(boundary)
                if xs is None:
                    continue
                color = color_for_marking(row[color_col])
                dash = dash_for_marking(marking_type)

                hover_text = (
                    f"Road: {row.road_id}<br>"
                    f"Lane: {row.lane_id}<br>"
                    f"{side} marking<br>"
                    "───────────────<br>"
                    f"Type: {marking_type}<br>"
                    f"Color: {row[color_col]}"
                )
                merger.add((color, dash), xs, ys, hover_text)

        traces = []
        for (color, dash), xs, ys, text in merger.items():
            traces.append(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name="Lane Markings",
                line=dict(width=self.size["lane_markings"], color=color, dash=dash),
                text=text,
                hoverinfo="text",
                hoverlabel=dict(bgcolor=color),
                showlegend=False,
            ))

        return traces
