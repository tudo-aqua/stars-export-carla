#!/usr/bin/env python3
import base64
import re
from typing import Iterable, Dict, List, Tuple, Any

import numpy as np
import orjson
import plotly.graph_objects as go
from dash import Patch
from dash_extensions.enrich import (
    Dash, dcc, html, Input, Output, State, no_update
)

from dataclass_wizard import JSONWizard
# Adjust import to your project
from carla_data_classes import TickData, DataBlock

# -------------------- Helpers --------------------
def iter_lanes(blocks: Iterable[Any]) -> Iterable[Any]:
    for b in blocks:
        for r in getattr(b, "roads", []):
            for l in getattr(r, "lanes", []):
                yield l

def safe_attr(obj, name, default=None):
    return getattr(obj, name, default)

def decode_upload(contents: str) -> bytes:
    return base64.b64decode(contents.split(",")[1])

def load_json(raw: bytes):
    """Your modified loader: TickData, single DataBlock, list[DataBlock]."""
    try:
        td = TickData.from_json(raw)
        return td, []
    except Exception:
        pass
    try:
        db = DataBlock.from_json(raw)
        return None, db
    except Exception:
        pass
    data = orjson.loads(raw)
    if isinstance(data, list):
        return None, [DataBlock.from_dict(d) for d in data]
    raise ValueError("JSON did not match TickData or DataBlock schema.")

def build_midpoint_arrays(midpoints: List[Any], arrow_len=1.0) -> Dict[str, list]:
    n = len(midpoints)
    xs = np.empty(n, np.float32); ys = np.empty(n, np.float32)
    yaw = np.empty(n, np.float32)
    road = np.empty(n, np.int32); lane = np.empty(n, np.int32)
    dist = np.empty(n, np.float32)
    for i, mp in enumerate(midpoints):
        xs[i], ys[i] = mp.location.x, mp.location.y
        yaw[i] = mp.rotation.yaw
        road[i] = mp.road_id; lane[i] = mp.lane_id
        dist[i] = mp.distance_to_start

    yaw_rad = np.deg2rad(yaw)
    x2 = xs + np.cos(yaw_rad) * arrow_len
    y2 = ys + np.sin(yaw_rad) * arrow_len

    seg_x = np.empty(n * 3, np.float32); seg_y = np.empty(n * 3, np.float32)
    seg_x[0::3] = xs; seg_y[0::3] = ys
    seg_x[1::3] = x2; seg_y[1::3] = y2
    seg_x[2::3] = np.nan; seg_y[2::3] = np.nan

    cd = np.stack([road, lane, dist, yaw], axis=1)
    cd_rep = np.repeat(cd, 3, axis=0)
    cd_rep[np.isnan(seg_x)] = [np.nan]*4

    return dict(xs=xs.tolist(), ys=ys.tolist(), yaw=yaw.tolist(),
                road=road.tolist(), lane=lane.tolist(), dist=dist.tolist(),
                seg_x=seg_x.tolist(), seg_y=seg_y.tolist(), seg_cd=cd_rep.tolist())

def lanes_to_polys(lanes: List[Any]) -> List[dict]:
    out = []
    for l in lanes:
        mids = l.lane_midpoints or []
        if not mids: continue
        xs = [mp.location.x for mp in mids]
        ys = [mp.location.y for mp in mids]
        out.append(dict(
            x=xs, y=ys,
            road_id=l.road_id,
            lane_id=l.lane_id,
            lane_type=str(l.lane_type),
            width=l.lane_width,
            length=l.lane_length
        ))
    return out

def list_landmarks(blocks):
    res=[]
    for l in iter_lanes(blocks):
        for lm in safe_attr(l, "landmarks", []) or []:
            res.append(dict(x=lm.location.x, y=lm.location.y,
                            id=lm.id, typ=lm.type.name, sub=lm.sub_type,
                            val=lm.value, unit=lm.unit))
    return res

def list_tlights(blocks):
    res=[]
    for l in iter_lanes(blocks):
        for t in safe_attr(l, "traffic_lights", []) or []:
            res.append(dict(x=t.location.x, y=t.location.y,
                            od=t.open_drive_id, dist=t.position_distance))
    return res

def list_contact_areas(blocks):
    res=[]
    for l in iter_lanes(blocks):
        for c in safe_attr(l, "contact_areas", []) or []:
            res.append(dict(x=c.contact_location.x, y=c.contact_location.y,
                            id=c.id,
                            l1=f"{c.lane_1_road_id}/{c.lane_1_id}",
                            l2=f"{c.lane_2_road_id}/{c.lane_2_id}"))
    return res

def list_speed_limit_segments(blocks):
    sx, sy = [], []
    for l in iter_lanes(blocks):
        for sl in safe_attr(l, "speed_limits", []) or []:
            pts = [mp for mp in (l.lane_midpoints or [])
                   if sl.from_distance <= mp.distance_to_start <= sl.to_distance]
            if len(pts) < 2: continue
            xs=[p.location.x for p in pts]; ys=[p.location.y for p in pts]
            sx.extend(xs+[np.nan]); sy.extend(ys+[np.nan])
    return sx, sy

def list_actors(tick):
    if tick is None: return []
    return [dict(x=a.actor.location.x, y=a.actor.location.y,
                 id=a.actor.id, tid=a.actor.type_id,
                 road=a.road_id, lane=a.lane_id, pos=a.position_on_lane)
            for a in getattr(tick, "actor_positions", []) or []]

def color_for_road(road_id: int, palette: List[str]) -> str:
    return palette[road_id % len(palette)]

def rgba(color_hex: str, alpha: float) -> str:
    if color_hex.startswith("#"):
        r = int(color_hex[1:3],16); g=int(color_hex[3:5],16); b=int(color_hex[5:7],16)
        return f"rgba({r},{g},{b},{alpha:.3f})"
    return color_hex

def restack(fig: go.Figure, layer_map: Dict[str, List[int]], order_bottom_to_top: List[str]):
    new_data = []
    new_layer_map = {k: [] for k in layer_map}
    for lname in order_bottom_to_top:
        for old_idx in layer_map.get(lname, []):
            new_idx = len(new_data)
            new_data.append(fig.data[old_idx])
            new_layer_map[lname].append(new_idx)
    fig.data = tuple(new_data)
    return fig, new_layer_map

# -------------------- Build figure --------------------
def build_full_figure(store: Dict[str,Any],
                      lane_filter: str,
                      default_layers: List[str],
                      sizes: Dict[str, int]) -> Tuple[go.Figure, Dict[str,List[int]], Dict[int,str]]:
    fig = go.Figure()
    layer_map: Dict[str,List[int]] = {}
    hover_templates: Dict[int, str] = {}

    if not store:
        return fig, layer_map, hover_templates

    def lane_match(r,l):
        if lane_filter == "ALL": return True
        rs, ls = lane_filter.split("/")
        return int(r)==int(rs) and int(l)==int(ls)

    palette = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd',
               '#8c564b','#e377c2','#7f7f7f','#bcbd22','#17becf']

    # ---- Midpoints
    if store.get("mid"):
        xs=np.array(store["mid"]["xs"]); ys=np.array(store["mid"]["ys"])
        road=np.array(store["mid"]["road"]); lane=np.array(store["mid"]["lane"])
        dist=np.array(store["mid"]["dist"]); yaw=np.array(store["mid"]["yaw"])
        mask=np.array([lane_match(r,l) for r,l in zip(road,lane)])
        cd=np.stack([road[mask], lane[mask], dist[mask], yaw[mask]], axis=1)
        tpl=("Road:%{customdata[0]} Lane:%{customdata[1]}<br>"
             "Dist:%{customdata[2]:.2f} m<br>"
             "Yaw:%{customdata[3]:.1f}°<br>x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        fig.add_trace(go.Scattergl(
            x=xs[mask], y=ys[mask], mode="markers", name="Lane midpoints",
            marker=dict(size=sizes["midpoints"]),
            customdata=cd,
            hovertemplate=tpl,
            hoverlabel=dict(bgcolor="#1f77b4"),
            visible="midpoints" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("midpoints",[]).append(idx)
        hover_templates[idx]=tpl

        # Arrows
        seg_x=np.array(store["mid"]["seg_x"]); seg_y=np.array(store["mid"]["seg_y"])
        seg_cd=np.array(store["mid"]["seg_cd"])
        if lane_filter!="ALL":
            starts=seg_cd[0::3]
            keep=np.repeat([lane_match(r,l) for r,l in zip(starts[:,0], starts[:,1])],3)
            seg_x=seg_x[keep]; seg_y=seg_y[keep]; seg_cd=seg_cd[keep]
        tpl=("Road:%{customdata[0]} Lane:%{customdata[1]}<br>"
             "Dist:%{customdata[2]:.2f} m<br>"
             "Yaw:%{customdata[3]:.1f}°<extra></extra>")
        fig.add_trace(go.Scattergl(
            x=seg_x, y=seg_y, mode="lines", name="Orientation arrows",
            line=dict(width=1, color="#1f77b4"),
            customdata=seg_cd,
            hovertemplate=tpl,
            hoverlabel=dict(bgcolor="#1f77b4"),
            visible="arrows" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("arrows",[]).append(idx)
        hover_templates[idx]=tpl

    # ---- Lane polylines
    lane_color = "#555555"
    for p in store["lanes"]:
        if not lane_match(p["road_id"], p["lane_id"]): continue
        xs = p["x"]; ys = p["y"]
        cd = np.stack([
            np.full(len(xs), p["road_id"]),
            np.full(len(xs), p["lane_id"]),
            np.full(len(xs), p["width"]),
            np.full(len(xs), p["length"])
        ], axis=1)
        tpl=("Road:%{customdata[0]} Lane:%{customdata[1]}<br>"
             "Width:%{customdata[2]:.2f} Len:%{customdata[3]:.2f}<br>"
             "x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        fig.add_trace(go.Scattergl(
            x=xs, y=ys, mode="lines",
            name=f"Lane {p['road_id']}/{p['lane_id']}",
            line=dict(width=2, color=lane_color),
            customdata=cd,
            hovertemplate=tpl,
            hoverlabel=dict(bgcolor=lane_color),
            visible="lanes" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("lanes",[]).append(idx)
        hover_templates[idx]=tpl

    # ---- Roads
    if store["lanes"]:
        max_abs_lane = max(abs(p["lane_id"]) for p in store["lanes"]) or 1
        for p in store["lanes"]:
            if not lane_match(p["road_id"], p["lane_id"]): continue
            xs = p["x"]; ys = p["y"]
            cd = np.stack([
                np.full(len(xs), p["road_id"]),
                np.full(len(xs), p["lane_id"])
            ], axis=1)
            tpl=("Road:%{customdata[0]} Lane:%{customdata[1]}<br>"
                 "x:%{x:.2f} y:%{y:.2f}<extra>Road view</extra>")
            base = color_for_road(p["road_id"], palette)
            opacity = max(0.15, 1 - (abs(p["lane_id"])/max_abs_lane))
            color = rgba(base, opacity)
            fig.add_trace(go.Scattergl(
                x=xs, y=ys, mode="lines",
                name=f"Road {p['road_id']} (lane {p['lane_id']})",
                line=dict(width=2, color=color),
                customdata=cd,
                hovertemplate=tpl,
                hoverlabel=dict(bgcolor=color),
                visible="roads" in default_layers,
                showlegend=False
            ))
            idx=len(fig.data)-1
            layer_map.setdefault("roads",[]).append(idx)
            hover_templates[idx]=tpl

    # ---- Landmarks
    if store["landmarks"]:
        x=[l["x"] for l in store["landmarks"]]; y=[l["y"] for l in store["landmarks"]]
        cd=[(l["id"],l["typ"],l["sub"],l["val"],l["unit"]) for l in store["landmarks"]]
        tpl=("ID:%{customdata[0]}<br>Type:%{customdata[1]}/%{customdata[2]}<br>"
             "Value:%{customdata[3]} %{customdata[4]}<br>"
             "x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        fig.add_trace(go.Scattergl(
            x=x, y=y, mode="markers", marker=dict(size=sizes["landmarks"], symbol="star"),
            name="Landmarks", customdata=cd,
            hovertemplate=tpl,
            hoverlabel=dict(bgcolor="#d62728"),
            visible="landmarks" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("landmarks",[]).append(idx)
        hover_templates[idx]=tpl

    # ---- Traffic lights
    if store["tlights"]:
        x=[t["x"] for t in store["tlights"]]; y=[t["y"] for t in store["tlights"]]
        cd=[(t["od"], t["dist"]) for t in store["tlights"]]
        tpl=("OD:%{customdata[0]}<br>Pos dist:%{customdata[1]:.2f} m<br>"
             "x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        fig.add_trace(go.Scattergl(
            x=x, y=y, mode="markers", marker=dict(size=sizes["traffic_lights"], symbol="triangle-up"),
            name="Traffic lights", customdata=cd,
            hovertemplate=tpl,
            hoverlabel=dict(bgcolor="#2ca02c"),
            visible="traffic_lights" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("traffic_lights",[]).append(idx)
        hover_templates[idx]=tpl

    # ---- Contact areas
    if store["contacts"]:
        x=[c["x"] for c in store["contacts"]]; y=[c["y"] for c in store["contacts"]]
        cd=[(c["id"],c["l1"],c["l2"]) for c in store["contacts"]]
        tpl=("ID:%{customdata[0]}<br>Lane1:%{customdata[1]}<br>"
             "Lane2:%{customdata[2]}<br>x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        fig.add_trace(go.Scattergl(
            x=x, y=y, mode="markers", marker=dict(size=sizes["contact_areas"], symbol="x"),
            name="Contact areas", customdata=cd,
            hovertemplate=tpl,
            hoverlabel=dict(bgcolor="#9467bd"),
            visible="contact_areas" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("contact_areas",[]).append(idx)
        hover_templates[idx]=tpl

    # ---- Speed limits (no hover)
    if store["sl_x"]:
        fig.add_trace(go.Scattergl(
            x=store["sl_x"], y=store["sl_y"], mode="lines",
            name="Speed limits", line=dict(width=3, dash="dot"),
            hovertemplate=None,
            hoverinfo="skip",
            visible="speed_limits" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("speed_limits",[]).append(idx)
        hover_templates[idx]=None

    # ---- Actors
    if store["actors"]:
        x=[a["x"] for a in store["actors"]]; y=[a["y"] for a in store["actors"]]
        cd=[(a["id"],a["tid"],a["road"],a["lane"],a["pos"]) for a in store["actors"]]
        mask=[lane_match(r,l) for _,_,r,l,_ in cd]
        x=np.array(x)[mask]; y=np.array(y)[mask]; cd=list(np.array(cd, dtype=object)[mask])
        tpl=("Actor:%{customdata[0]} (%{customdata[1]})<br>"
             "Road/Lane:%{customdata[2]}/%{customdata[3]}<br>"
             "Pos:%{customdata[4]:.2f} m<br>x:%{x:.2f} y:%{y:.2f}<extra></extra>")
        fig.add_trace(go.Scattergl(
            x=x, y=y, mode="markers", marker=dict(size=sizes["actors"], symbol="circle-open"),
            name="Actors", customdata=cd,
            hovertemplate=tpl,
            hoverlabel=dict(bgcolor="#8c564b"),
            visible="actors" in default_layers
        ))
        idx=len(fig.data)-1
        layer_map.setdefault("actors",[]).append(idx)
        hover_templates[idx]=tpl

    # stack: bottom -> top
    DRAW_ORDER = [
        "roads", "lanes", "speed_limits",
        "midpoints", "arrows", "actors",
        "contact_areas", "traffic_lights", "landmarks"
    ]
    fig, layer_map = restack(fig, layer_map, DRAW_ORDER)

    fig.update_layout(
        xaxis=dict(scaleanchor="y", scaleratio=1, showgrid=False, zeroline=False),
        yaxis=dict(showgrid=False, zeroline=False),
        hovermode="closest",
        hoverdistance=5,
        uirevision="keep",
        legend=dict(itemsizing="constant")
    )
    # adjust hover_templates indices after restack
    new_templates = {}
    # Build mapping old->new indices from layer_map (we restacked; but we already rebuilt layer_map)
    # We can just iterate through traces in new order
    for new_idx, tr in enumerate(fig.data):
        # find original hover template by matching name+customdata length? simpler: copy sequentially
        # easier: during restack we lost original mapping; but we rebuilt layer_map
        # we can just set new_templates[new_idx] = tr.hovertemplate
        new_templates[new_idx] = tr.hovertemplate
    return fig, layer_map, new_templates

# -------------------- Dash App --------------------
LAYER_OPTIONS = [
    {"label":"Lane midpoints",        "value":"midpoints"},
    {"label":"Orientation arrows",    "value":"arrows"},
    {"label":"Lane polylines",        "value":"lanes"},
    {"label":"Roads (colored lanes)", "value":"roads"},
    {"label":"Landmarks",             "value":"landmarks"},
    {"label":"Traffic lights",        "value":"traffic_lights"},
    {"label":"Contact areas",         "value":"contact_areas"},
    {"label":"Speed limit segments",  "value":"speed_limits"},
    {"label":"Actors (tick)",         "value":"actors"},
]
DEFAULT_LAYERS = ["midpoints", "arrows"]
DEFAULT_SIZES = {
    "midpoints": 3,
    "contact_areas": 8,
    "landmarks": 8,
    "traffic_lights": 9,
    "actors": 7
}
HOVER_DEFAULT = [o["value"] for o in LAYER_OPTIONS]  # enable hover for all by default

app = Dash(__name__, suppress_callback_exceptions=True)
app.title = "CARLA Map Viewer"

app.layout = html.Div([
    html.Div([
        html.H3("CARLA JSON Viewer"),
        dcc.Upload(
            id="upload-json",
            children=html.Div(["Drag & Drop or ", html.A("Select JSON")]),
            style={"width":"100%","height":"60px","lineHeight":"60px",
                   "borderWidth":"1px","borderStyle":"dashed","borderRadius":"5px",
                   "textAlign":"center","margin":"10px 0"},
            multiple=False
        ),
        html.Div(id="load-msg", style={"fontSize":"12px","color":"#555"}),

        html.H4("Layers"),
        dcc.Checklist(id="layer-checklist", options=LAYER_OPTIONS,
                      value=DEFAULT_LAYERS,
                      inputStyle={"margin-right":"4px","margin-left":"12px"}),

        html.H4("Lane filter"),
        dcc.Dropdown(id="lane-filter",
                     options=[{"label":"ALL","value":"ALL"}],
                     value="ALL", clearable=False, searchable=True),
        html.Div("Click a lane/midpoint to set the filter.",
                 style={"fontSize":"11px","color":"#777"}),

        html.H4("Hover enabled for"),
        dcc.Checklist(id="hover-checklist", options=LAYER_OPTIONS,
                      value=HOVER_DEFAULT,
                      inputStyle={"margin-right":"4px","margin-left":"12px"}),

        html.H4("Marker sizes"),
        html.Label("Midpoints"),
        dcc.Slider(1, 12, 1, value=DEFAULT_SIZES["midpoints"], id="size-midpoints"),
        html.Label("Contact areas"),
        dcc.Slider(2, 20, 1, value=DEFAULT_SIZES["contact_areas"], id="size-contact"),
        html.Label("Landmarks"),
        dcc.Slider(2, 20, 1, value=DEFAULT_SIZES["landmarks"], id="size-landmark"),
        html.Label("Traffic lights"),
        dcc.Slider(2, 20, 1, value=DEFAULT_SIZES["traffic_lights"], id="size-tl"),
        html.Label("Actors"),
        dcc.Slider(2, 20, 1, value=DEFAULT_SIZES["actors"], id="size-actors"),

    ], style={"width":"300px","float":"left","padding":"10px"}),

    html.Div([dcc.Graph(id="map-fig", style={"height":"100vh"})],
             style={"margin-left":"320px"}),

    dcc.Store(id="raw-store"),
    dcc.Store(id="fig-layer-map"),
    dcc.Store(id="trace-templates")
])

# ---------- Callbacks ----------
@app.callback(
    Output("raw-store","data"),
    Output("lane-filter","options"),
    Output("load-msg","children"),
    Input("upload-json","contents"),
    State("upload-json","filename"),
    prevent_initial_call=True
)
def handle_upload(contents, filename):
    if not contents: return no_update, no_update, no_update
    try:
        raw = decode_upload(contents)
        tick, blocks = load_json(raw)
        if isinstance(blocks, DataBlock):
            blocks = [blocks]
        elif blocks is None:
            blocks = []

        mids=[]
        for l in iter_lanes(blocks):
            mids.extend(safe_attr(l,"lane_midpoints",[]) or [])
        mid = build_midpoint_arrays(mids, 1.0) if mids else {}
        lanes = lanes_to_polys(list(iter_lanes(blocks)))

        lane_opts = [{"label":"ALL","value":"ALL"}] + [
            {"label": f"{p['road_id']}/{p['lane_id']}",
             "value": f"{p['road_id']}/{p['lane_id']}"}
            for p in lanes
        ]

        sx, sy = list_speed_limit_segments(blocks)

        store = dict(
            mid=mid,
            lanes=lanes,
            landmarks=list_landmarks(blocks),
            tlights=list_tlights(blocks),
            contacts=list_contact_areas(blocks),
            sl_x=sx,
            sl_y=sy,
            actors=list_actors(tick)
        )
        msg = f"Loaded '{filename}'. Lanes: {len(lanes)}  Midpoints: {len(mid.get('xs',[]))}"
        return store, lane_opts, msg
    except Exception as e:
        return {}, [{"label":"ALL","value":"ALL"}], f"Error: {e}"

@app.callback(
    Output("map-fig","figure"),
    Output("fig-layer-map","data"),
    Output("trace-templates","data"),
    Input("raw-store","data"),
    Input("lane-filter","value"),
    State("layer-checklist","value"),
    State("size-midpoints","value"),
    State("size-contact","value"),
    State("size-landmark","value"),
    State("size-tl","value"),
    State("size-actors","value"),
    prevent_initial_call=True
)
def rebuild_on_filter(store, lane_filter, selected_layers,
                      s_mid, s_contact, s_landm, s_tl, s_act):
    if not store:
        return go.Figure(), {}, {}
    sizes = dict(
        midpoints=s_mid,
        contact_areas=s_contact,
        landmarks=s_landm,
        traffic_lights=s_tl,
        actors=s_act
    )
    fig, lmap, templates = build_full_figure(store, lane_filter,
                                             selected_layers or DEFAULT_LAYERS,
                                             sizes)
    return fig, lmap, templates

# Fast layer visibility toggle
@app.callback(
    Output("map-fig","figure", allow_duplicate=True),
    Input("layer-checklist","value"),
    State("fig-layer-map","data"),
    prevent_initial_call=True
)
def fast_toggle(layers, layer_map):
    if not layer_map: return no_update
    patch = Patch()
    vis = set(layers)
    for lname, idxs in layer_map.items():
        show = lname in vis
        for i in idxs:
            patch["data"][i]["visible"] = show
    return patch

# Fast marker size patch
@app.callback(
    Output("map-fig", "figure", allow_duplicate=True),
    Input("size-midpoints", "value"),
    Input("size-contact", "value"),
    Input("size-landmark", "value"),
    Input("size-tl", "value"),
    Input("size-actors", "value"),
    State("fig-layer-map", "data"),
    prevent_initial_call=True
)
def patch_sizes(s_mid, s_contact, s_landm, s_tl, s_act, layer_map):
    if not layer_map:
        return no_update

    sizes = {
        "midpoints": s_mid,
        "contact_areas": s_contact,
        "landmarks": s_landm,
        "traffic_lights": s_tl,
        "actors": s_act
    }
    marker_layers = sizes.keys()  # only these have markers

    patch = Patch()
    for lname in marker_layers:
        for idx in layer_map.get(lname, []):
            # Just set the marker dict; Plotly will ignore it for line-only traces anyway.
            patch["data"][idx]["marker"] = {"size": sizes[lname]}
    return patch

# Fast hover enable/disable patch
@app.callback(
    Output("map-fig","figure", allow_duplicate=True),
    Input("hover-checklist","value"),
    State("fig-layer-map","data"),
    State("trace-templates","data"),
    prevent_initial_call=True
)
def patch_hover(enabled_layers, layer_map, templates):
    if not layer_map or not templates:
        return no_update
    enabled = set(enabled_layers or [])
    patch = Patch()
    for lname, idxs in layer_map.items():
        for i in idxs:
            if lname in enabled:
                # restore template
                patch["data"][i]["hovertemplate"] = templates[str(i)] if isinstance(templates, dict) else templates[i]
                # ensure hoverinfo not skip
                patch["data"][i]["hoverinfo"] = "all"
            else:
                patch["data"][i]["hovertemplate"] = None
                patch["data"][i]["hoverinfo"] = "skip"
    return patch

# Click to set lane filter
LANE_RE = re.compile(r"(\d+)\s*/\s*(\d+)")  # matches "123/4"

@app.callback(
    Output("lane-filter", "value"),
    Input("map-fig", "clickData"),
    State("lane-filter", "value"),
    prevent_initial_call=True
)
def click_to_filter(clickData, current):
    if not clickData:
        return no_update

    point = clickData["points"][0]
    cd = point.get("customdata")

    # Helper to try int conversion
    def try_int_pair(a, b):
        try:
            return f"{int(a)}/{int(b)}"
        except Exception:
            return None

    # 1) Best case: first two entries are ints
    if isinstance(cd, (list, tuple)) and len(cd) >= 2:
        lane_str = try_int_pair(cd[0], cd[1])
        if lane_str:
            return lane_str

    # 2) Look for "123/4" pattern anywhere in customdata
    def find_lane_in_iter(items):
        for v in items:
            if isinstance(v, str):
                m = LANE_RE.search(v)
                if m:
                    return f"{m.group(1)}/{m.group(2)}"
        return None

    if isinstance(cd, (list, tuple)):
        lane_str = find_lane_in_iter(cd)
        if lane_str:
            return lane_str

    # 3) Give up (don’t change the filter)
    return no_update

if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)
