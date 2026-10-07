# layers/utils.py
from typing import Dict, List, Optional, Tuple


class LineTraceMerger:
    """
    Accumulates many small shapes (closed polygons or polylines) that share the
    same visual style into a handful of merged Scattergl traces, joining shapes
    with a NaN coordinate so they stay visually separate within one trace.

    Plotly.js carries a meaningful fixed per-trace cost on every pan/zoom
    (measured at roughly 1.5-2ms/trace in this app, regardless of trace content
    or fill) — so a layer with hundreds/thousands of one-shape-per-trace objects
    is what makes interaction feel slow, independent of WebGL vs SVG. Grouping by
    the exact style key (e.g. (line_color, fill_color)) collapses that trace count
    with zero visual change, since every shape in a bucket already looked identical.

    Hover trades whole-shape-area hit-testing for nearest-vertex hit-testing (one
    `text` entry per vertex) — the same trade-off already made when this app's
    filled layers moved from SVG (`hoveron="fills"`) to Scattergl.
    """

    def __init__(self):
        self._buckets: Dict[Tuple, Tuple[List[float], List[float], List]] = {}

    def add(self, style_key: Tuple, xs, ys, hover_text) -> None:
        """
        Append one shape's coordinates (a closed ring or a polyline) to its style
        bucket. hover_text is either a single string, shown for every vertex of
        this shape, or a sequence the same length as xs/ys for per-vertex text
        (e.g. a running distance-along-lane that differs at each point).
        """
        bx, by, bt = self._buckets.setdefault(style_key, ([], [], []))
        if bx:
            bx.append(float("nan"))
            by.append(float("nan"))
            bt.append(None)
        n = len(xs)
        texts = hover_text if isinstance(hover_text, (list, tuple)) else [hover_text] * n
        for x, y, t in zip(xs, ys, texts):
            bx.append(float(x))
            by.append(float(y))
            bt.append(t)

    def items(self):
        """Yield (style_key, xs, ys, text) once per merged bucket."""
        for style_key, (xs, ys, text) in self._buckets.items():
            yield style_key, xs, ys, text

PALETTE = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd',
           '#8c564b','#e377c2','#7f7f7f','#bcbd22','#17becf']

def color_for_road(road_id: int, palette: List[str] = PALETTE) -> str:
    return palette[road_id % len(palette)]

def rgba(hex_color: str, alpha: float) -> str:
    if hex_color.startswith("#"):
        r = int(hex_color[1:3],16); g = int(hex_color[3:5],16); b = int(hex_color[5:7],16)
        return f"rgba({r},{g},{b},{alpha:.3f})"
    return hex_color


# Matches carla.LaneMarkingColor names (see DataLaneMarkingColor)
MARKING_COLORS = {
    "Standard": "#f5f5f5",
    "White": "#f5f5f5",
    "Blue": "#3366ff",
    "Green": "#33cc66",
    "Red": "#ff3333",
    "Yellow": "#ffcc00",
    "Other": "#999999",
}


def color_for_marking(color_name: str) -> str:
    return MARKING_COLORS.get(color_name, "#999999")


# Matches carla.LaneMarkingType names (see DataLaneMarkingType); NONE/Curb/Grass are not drawn
MARKING_DASH = {
    "Solid": "solid",
    "SolidSolid": "solid",
    "Broken": "dash",
    "BottsDots": "dot",
    "SolidBroken": "dashdot",
    "BrokenSolid": "dashdot",
    "BrokenBroken": "dash",
    "Other": "solid",
}


def dash_for_marking(type_name: str) -> str:
    return MARKING_DASH.get(type_name, "solid")


def marking_type_name(marking) -> str:
    """marking is an Optional[DataLaneMarking]; returns 'NONE' when absent."""
    return marking.marking_type.name if marking is not None else "NONE"


def marking_color_name(marking) -> str:
    """marking is an Optional[DataLaneMarking]; returns 'NONE' when absent."""
    return marking.color.name if marking is not None else "NONE"


def neighbor_label(info) -> str:
    """info is an Optional[DataContactLaneInfo]; returns '—' when there is no neighbor lane."""
    if info is None:
        return "—"
    return f"Road {info.road_id}, Lane {info.lane_id}"


def lane_list_label(infos) -> str:
    """infos is a List[DataContactLaneInfo]; returns '—' when the list is empty."""
    if not infos:
        return "—"
    return ", ".join(f"Road {info.road_id}, Lane {info.lane_id}" for info in infos)


MERGE_COLOR = "#ff9800"  # orange
DIVERGE_COLOR = "#e91e63"  # magenta
MERGE_DIVERGE_COLOR = "#d50000"  # red
OVERLAP_COLOR = "#00bcd4"  # teal


def topology_highlight_color(label: str) -> Optional[str]:
    """
    Distinct outline color for a lane's `lane_topology` value (computed at export time by
    MapRasterizer.compute_lane_overlaps from actual overlapping lane geometry — see DataLane),
    or None for a lane with no physical overlap.
    """
    if label == "Merging":
        return MERGE_COLOR
    if label == "Diverging":
        return DIVERGE_COLOR
    if label == "Merging & Diverging":
        return MERGE_DIVERGE_COLOR
    if label == "Overlapping":
        return OVERLAP_COLOR
    return None
