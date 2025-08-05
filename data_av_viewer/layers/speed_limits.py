from __future__ import annotations

from typing import List

import pandas as pd
import plotly.graph_objects as go

from .base_layer import BaseLayer, register


def _speed_to_color(kmh: float) -> str:
    """
    Simple, readable palette by limit bands (km/h).
    Tweak as you prefer.
    """
    # blues (slow) → yellows → oranges → red (fast)
    if kmh <= 20:  return "#2c7bb6"
    if kmh <= 30:  return "#74add1"
    if kmh <= 40:  return "#abd9e9"
    if kmh <= 50:  return "#ffffbf"
    if kmh <= 60:  return "#fee090"
    if kmh <= 70:  return "#fdae61"
    if kmh <= 80:  return "#f46d43"
    if kmh <= 100: return "#d73027"
    return "#a50026"


@register("speed_limits")
class SpeedLimitsLayer(BaseLayer):
    """
    Draw speed-limit sections along each lane as colored polylines.

    DataFrame schema produced by build_df():
        road_id      : int
        lane_id      : int
        speed        : float  (km/h)
        start        : float  (from_distance, m)
        end          : float  (to_distance, m)
        xs           : list[float]
        ys           : list[float]
    """
    slider_key   = "line_width"   # makes it appear under "Marker sizes" in the GUI
    default_size = 6
    df_key       = "speed_limits"

    # ---------- build the per-layer dataframe from DataBlocks ----------
    @classmethod
    def build_df(cls, blocks, tick) -> pd.DataFrame:
        rows: List[dict] = []

        # blocks can be a single DataBlock or a list
        if blocks is None:
            return pd.DataFrame(rows)
        block_list = blocks if isinstance(blocks, list) else [blocks]

        for blk in block_list:
            for road in getattr(blk, "roads", []):
                for lane in getattr(road, "lanes", []):
                    speed_sections = getattr(lane, "speed_limits", None)
                    midpoints      = getattr(lane, "lane_midpoints", None)
                    if not speed_sections or not midpoints:
                        continue

                    # Pre-collect (distance, (x,y)) tuples for fast slicing
                    mid_items = []
                    for mp in midpoints:
                        # DataLaneMidpoint has distance_to_start and DataLocation with .to_tuple()
                        try:
                            x, y = mp.location.to_tuple()
                        except Exception:
                            # fallback if DataLocation lacks to_tuple(): use attributes
                            x, y = float(mp.location.x), float(mp.location.y)
                        mid_items.append((float(mp.distance_to_start), (x, y)))

                    # Build a small line for each speed-limit segment on this lane
                    for seg in speed_sections:
                        start = float(seg.from_distance)
                        end   = float(seg.to_distance)
                        xs, ys = [], []
                        for d, (x, y) in mid_items:
                            if start <= d <= end:
                                xs.append(x)
                                ys.append(y)

                        # need at least 2 points to draw a line
                        if len(xs) >= 2:
                            rows.append({
                                "road_id": road.road_id,
                                "lane_id": lane.lane_id,
                                "speed": float(seg.speed_limit),
                                "start": start,
                                "end": end,
                                "xs": xs,
                                "ys": ys,
                            })

        return pd.DataFrame(rows)

    # ---------- turn the dataframe into Plotly traces ------------------
    def traces(self) -> List[go.BaseTraceType]:
        df = self.get_df(self.df_key)
        if df.empty:
            return []

        width = self.size.get(self.layer_name, self.default_size)
        traces: List[go.Scattergl] = []

        # One trace per segment keeps colors discrete and hover simple
        for _, row in df.iterrows():
            speed_ms = float(row["speed"])              # <-- stored in m/s
            kmh      = _ms_to_kmh(speed_ms)
            mph      = _ms_to_mph(speed_ms)

            road_id = int(row["road_id"])
            lane_id = int(row["lane_id"])
            start   = float(row["start"])
            end     = float(row["end"])

            xs = row["xs"]
            ys = row["ys"]
            n  = len(xs)

            # one customdata row per plotted point:
            # [m/s, km/h, mph, road_id, lane_id, start, end]
            customdata = [[speed_ms, kmh, mph, road_id, lane_id, start, end] for _ in range(n)]

            traces.append(
                go.Scattergl(
                    x=xs,
                    y=ys,
                    mode="lines",
                    line=dict(
                        width=width,
                        color=_speed_to_color(kmh)  # keep your color bands in km/h
                    ),
                    name="Speed limits",
                    showlegend=False,
                    hovertemplate=(
                        "Speed: %{customdata[0]:.1f} m/s"
                        "<br>%{customdata[1]:.0f} km/h (%{customdata[2]:.0f} mph)"
                        "<br>Road: %{customdata[3]}  Lane: %{customdata[4]}"
                        "<br>From: %{customdata[5]:.0f} m  To: %{customdata[6]:.0f} m"
                        "<extra></extra>"
                    ),
                    customdata=customdata,
                )
            )
        return traces

def _ms_to_kmh(ms: float) -> float:
    return ms * 3.6

def _ms_to_mph(ms: float) -> float:
    return ms * 2.2369362921  # exact factor