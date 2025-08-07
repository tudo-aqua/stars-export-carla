# layers/traffic_lights.py
from __future__ import annotations

import pandas as pd
import plotly.graph_objects as go

from carla_data_classes.static.DataMap import DataMap
from .base_layer import register, BaseLayer


@register("traffic_lights")
class TrafficLightLayer(BaseLayer):
    """
    Renders each traffic light as a rectangle housing + three circular lamps
    using Plotly layout shapes (one rect + 3 circles per light) and an
    invisible scatter trace to provide hover + legend.

    The DataFrame aggregates by open_drive_id and collects all (road, lane)
    pairs that reference each light, so hover can list the affected lanes.
    """
    slider_key = "traffic_lights"
    df_key = "traffic_lights"
    default_size = 12  # slider default

    # ------------------------------- map slider size -> data-unit geometry
    @staticmethod
    def _geometry_from_size(size: int | float) -> dict:
        """
        Map the UI 'size' (≈1..20) to data-unit geometry.
        Tuned to be clearly visible and non-overlapping.
        """
        s = float(size or 0)
        lamp_r = 0.05 * s  # meters (or your map units)
        gap = 0.60 * lamp_r
        pad = 0.80 * lamp_r
        total_w = 2 * lamp_r + 2 * pad
        total_h = 3 * (2 * lamp_r) + 2 * gap + 2 * pad
        border_w = max(1, int(s / 3))  # px
        return dict(lamp_r=lamp_r, pad=pad, gap=gap,
                    total_w=total_w, total_h=total_h, border_w=border_w)

    # -------------------------------------------------------------- build df
    @classmethod
    def build_df(cls, data_map: DataMap, tick):
        """
        Aggregate by open_drive_id so one marker represents one post, and
        collect all lanes that reference it for hover.
        """
        by_od = {}
        for r in data_map.get_all_roads():
            for l in r.lanes:
                for tl in (l.traffic_lights or []):
                    od = getattr(tl, "open_drive_id", getattr(tl, "opendrive_id", None))
                    if od is None:
                        continue
                    rec = by_od.get(od)
                    if rec is None:
                        rec = {
                            "x": getattr(tl.location, "x", None),
                            "y": getattr(tl.location, "y", None),
                            "od": od,
                            "dist": getattr(tl, "position_distance", None),
                            "lane_pairs_set": set(),  # (road_id, lane_id)
                        }
                        by_od[od] = rec
                    rid = getattr(l, "road_id", getattr(r, "road_id", None))
                    rec["lane_pairs_set"].add((rid, l.lane_id))

        rows = []
        for rec in by_od.values():
            pairs = sorted(rec.pop("lane_pairs_set"))
            # same pattern as landmarks layer: multi-line, indented HTML
            lines = [f"&nbsp;&nbsp;&nbsp;&nbsp;(Road {rd}, Lane {ln})" for rd, ln in pairs]
            rec["lane_pairs_html"] = "<br>" + "<br>".join(lines) if lines else ""
            rec["lane_pairs_count"] = len(pairs)
            rows.append(rec)

        df = pd.DataFrame(rows)
        if not df.empty:
            df["x"] = pd.to_numeric(df["x"], errors="coerce")
            df["y"] = pd.to_numeric(df["y"], errors="coerce")
            df = df.dropna(subset=["x", "y"])
        return df

    # ---------------------------------------------------------------- traces
    def traces(self):
        """
        Provide an invisible scatter for hover/legend.
        Icons themselves are drawn via layout shapes (see shapes()).
        """
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        custom = df[["od", "lane_pairs_html"]].to_numpy()
        hover_tpl = (
            "Static ID: %{customdata[0]}<br>"
            "Lanes:%{customdata[1]}<br>"
            "X:%{x:.2f} Y:%{y:.2f}<extra></extra>"
        )

        # Invisible anchor for hover/legend
        anchor = go.Scatter(
            x=df["x"], y=df["y"], mode="markers",
            marker=dict(size=6, opacity=0),  # invisible but hoverable
            customdata=custom,
            hovertemplate=hover_tpl,
            hoverlabel=dict(bgcolor="#2ca02c"),
            name="Traffic Lights",
            showlegend=True,
        )
        return [anchor]

    # --------------------------------------------------------------- shapes
    def shapes(self):
        """
        Return a list of layout shapes (rect + 3 circles per light).
        The viewer appends these to `fig.layout.shapes` and resizes them in
        the size-slider callback using the same formulas.
        """
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        s = self.size.get("traffic_lights", self.default_size) or self.default_size
        g = self._geometry_from_size(s)
        lamp_r, pad, gap = g["lamp_r"], g["pad"], g["gap"]
        total_w, total_h, border_w = g["total_w"], g["total_h"], g["border_w"]

        shapes = []
        # Colors
        housing_fill = "#222"
        red, amber, green = "#e74c3c", "#f1c40f", "#2ecc71"

        # vertical spacing between lamp centers (no overlap)
        D = 2.0 * lamp_r + gap
        offsets = (+D, 0.0, -D)

        for x, y in zip(df["x"], df["y"]):
            # Housing rectangle centered at (x, y)
            x0 = x - total_w / 2.0
            x1 = x + total_w / 2.0
            y0 = y - total_h / 2.0
            y1 = y + total_h / 2.0

            shapes.append(dict(
                type="rect", xref="x", yref="y",
                x0=x0, x1=x1, y0=y0, y1=y1,
                line=dict(width=border_w, color="#444"),
                fillcolor=housing_fill, opacity=0.95,
                layer="above",
            ))

            # Three circular lamps
            for off, color in zip(offsets, (red, amber, green)):
                cy = y + off
                shapes.append(dict(
                    type="circle", xref="x", yref="y",
                    x0=x - lamp_r, x1=x + lamp_r,
                    y0=cy - lamp_r, y1=cy + lamp_r,
                    line=dict(width=0),
                    fillcolor=color, opacity=1.0,
                    layer="above",
                ))

        return shapes
