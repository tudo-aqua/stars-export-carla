#!/usr/bin/env python3
"""
CARLA viewer – static layers **plus** super‑lightweight dynamic replay.

  • drag‑&‑drop a *.json* with either blocks (static) or tick list (dynamic) or both
  • layer toggles + per‑layer size sliders (unchanged)
  • tick slider + play/pause + speed control for the dynamics
  • only x/y/text of pre‑built actor traces are patched – no lag, even with thousands of frames
"""
from __future__ import annotations
import base64, importlib, pkgutil, pathlib, orjson
from typing import Dict, List, Tuple

import numpy as np
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output, State, Patch, ALL, ctx, no_update

from carla_data_classes import TickData, DataBlock
from viewer_store import ViewerStore
from layers.base_layer import build_all_traces, LAYER_REGISTRY
from dynamic.actor_traces import build_dynamic_templates     # <── our helper

# ----------------------------------------------------------------------
# auto‑import all static layer modules ---------------------------------
_layers_path = pathlib.Path(__file__).parent / "layers"
for m in pkgutil.iter_modules([str(_layers_path)]):
    importlib.import_module(f"layers.{m.name}")

# ----------------------------------------------------------------------
# helpers --------------------------------------------------------------
def _decode_upload(contents: str) -> bytes:
    return base64.b64decode(contents.split(",", 1)[1])


def _load_raw_json(raw: bytes) -> Tuple[List[TickData], List[DataBlock]]:

    # try TickData(s)
    try:
        return TickData.from_json(raw), []
    except Exception:
        pass

    # try DataBlock(s)
    try:
        blk = DataBlock.from_json(raw)
        return [], blk
    except Exception:
        pass

    raise ValueError("unknown JSON schema")

# ----------------------------------------------------------------------
# GUI constants --------------------------------------------------------
LAYER_OPTIONS = [{"label": n.replace("_", " ").title(), "value": n}
                 for n in LAYER_REGISTRY]
DEFAULT_LAYERS = ["roads"]
HOVER_DEFAULT  = ["roads"]
SIZE_LAYERS    = [n for n,c in LAYER_REGISTRY.items() if getattr(c,"slider_key",None)]
DEFAULT_SIZES  = {n: c.default_size for n,c in LAYER_REGISTRY.items()
                  if getattr(c,"default_size",None)}

# ----------------------------------------------------------------------
# small re‑usable widgets ----------------------------------------------
def _size_slider(layer):
    return html.Div([
        html.Label(layer.replace("_", " ").title()),
        dcc.Slider(1, 20, 1, updatemode="drag",
                   value=DEFAULT_SIZES[layer],
                   id={"type":"size","layer":layer})
    ])

# ----------------------------------------------------------------------
# Dash layout ----------------------------------------------------------
app = Dash(__name__, suppress_callback_exceptions=True)
app.layout = html.Div([
    # ---- sidebar -----------------------------------------------------
    html.Div([
        html.H3("CARLA Viewer"),
        dcc.Upload(id="upload",
                   children=html.Div(["Drag & Drop or ", html.A("Select JSON")]),
                   style={"width":"100%","height":"60px","lineHeight":"60px",
                          "border":"1px dashed","borderRadius":"5px",
                          "textAlign":"center","margin":"10px 0"}),
        html.Div(id="msg", style={"fontSize":"12px","color":"#555"}),

        html.H4("Static layers"),
        dcc.Checklist(id="layer-ck", options=LAYER_OPTIONS, value=DEFAULT_LAYERS,
                      inputStyle={"margin-right":"4px","margin-left":"12px"}),

        html.H4("Hover enabled for"),
        dcc.Checklist(id="hover-ck", options=LAYER_OPTIONS, value=HOVER_DEFAULT,
                      inputStyle={"margin-right":"4px","margin-left":"12px"}),

        html.H4("Marker sizes"),
        *(_size_slider(l) for l in SIZE_LAYERS),

        html.Hr(),

        html.H4("Dynamic replay"),
        dcc.Slider(id="tick-sl", min=0, max=0, step=1, value=0, updatemode="drag",
                   tooltip={"placement":"bottom","always_visible":True}),
        html.Div([
            html.Button("▶ Play", id="play-btn", n_clicks=0,
                        style={"width":"60px"}),
            html.Button("⏸ Pause", id="pause-btn", n_clicks=0,
                        style={"width":"60px","marginLeft":"6px"}),
            html.Span("  speed"),
            dcc.Slider(id="speed-sl", min=0.2, max=5, updatemode="drag",
                       step=0.2, value=1.0, tooltip={"placement":"bottom"})
        ], style={"marginTop":"4px"})
    ], style={"width":"300px","float":"left","padding":"10px"}),

    # ---- main figure -------------------------------------------------
    html.Div([dcc.Graph(id="fig", style={"height":"100vh"})],
             style={"margin-left":"320px"}),

    # ---- client‑side stores -----------------------------------------
    dcc.Store(id="store-json"),         # static layers – same as before
    dcc.Store(id="dyn-ticks"),          # list[TickData]     (raw)
    dcc.Store(id="dyn-data"),           # dict {templates, per_tick}
    dcc.Store(id="layer-map"),          # map layer‑name -> trace‑idxs
    dcc.Store(id="hover-tpl")           # original hover templates
])

# An interval that drives playback (disabled by default)
app.layout.children.append(
    dcc.Interval(id="play-ivl", disabled=True, interval=500)
)

# ----------------------------------------------------------------------
# 1)  Parse Upload -----------------------------------------------------
@app.callback(
    Output("store-json", "data"),
    Output("dyn-ticks",   "data"),
    Output("msg",         "children"),
    Input("upload", "contents"),
    State("upload", "filename"),
    prevent_initial_call=True
)
def parse_upload(contents, fname):
    if not contents:
        return {}, [], "No file."
    try:
        ticks, blocks = _load_raw_json(_decode_upload(contents))
        store = ViewerStore.from_source(blocks, ticks[0] if ticks else None)
        msg = f"Loaded '{fname}'  |  ticks:{len(ticks)}  |  blocks:{len(blocks)}"
        return store.to_json(), orjson.dumps([t.to_dict() for t in ticks]).decode(), msg
    except Exception as e:
        return {}, [], f"Error: {e}"

# ----------------------------------------------------------------------
# 2)  Build base figure + dynamic templates ---------------------------
@app.callback(
    Output("fig",       "figure"),
    Output("layer-map", "data"),
    Output("hover-tpl", "data"),
    Output("dyn-data",  "data"),          # ← NEW
    Output("tick-sl",   "max"),           # set slider range
    Input("store-json", "data"),
    State("dyn-ticks",  "data"),
    State("layer-ck",   "value"),
    prevent_initial_call=True
)
def build_fig(json_data, dyn_raw, visible_layers):
    if not json_data:
        return go.Figure(), {}, {}, {}, 0
    # --- static layers ------------------------------------------------
    store = ViewerStore.from_json(json_data)
    static_traces, layer_map = build_all_traces(store, visible_layers, DEFAULT_SIZES)

    # --- dynamic templates & per‑tick payload -------------------------
    dyn_templates = []
    per_tick      = []
    if dyn_raw:
        ticks = [TickData.from_dict(d) for d in orjson.loads(dyn_raw)]
        dyn_templates, per_tick = build_dynamic_templates(ticks)

    # ---- compose figure ---------------------------------------------
    fig = go.Figure(data=static_traces + dyn_templates)
    fig.update_layout(
        dragmode="pan", hovermode="closest", uirevision="keep",
        legend=dict(itemsizing="constant"),
        xaxis=dict(scaleanchor="y", scaleratio=1, showgrid=False),
        yaxis=dict(showgrid=False)
    )
    # remember original hovertpl for static layers
    tmpl = {i: t.hovertemplate for i, t in enumerate(fig.data)}

    # we also keep a pointer to dynamic trace indices (for patching)
    if dyn_templates:
        layer_map["dynamic"] = list(
            range(len(static_traces),
                  len(static_traces) + len(dyn_templates))
        )
    return fig, layer_map, tmpl, {"frames": per_tick}, len(per_tick)-1

# ----------------------------------------------------------------------
# 3)  Static‑layer toggles / sizes / hover   – unchanged callbacks -----
@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input("layer-ck", "value"),
    State("layer-map", "data"),
    prevent_initial_call=True
)
def toggle_layers(sel, layer_map):
    if not layer_map:
        return no_update
    patch = Patch()
    sel = set(sel)
    for lname, idxs in layer_map.items():
        if lname == "dynamic":
            continue
        for i in idxs:
            patch["data"][i]["visible"] = lname in sel
    return patch

@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input({"type":"size","layer":ALL}, "value"),
    State({"type":"size","layer":ALL}, "id"),
    State("layer-map", "data"),
    prevent_initial_call=True
)
def patch_sizes(values, ids, layer_map):
    if not layer_map:
        return no_update
    patch = Patch()
    for val,item in zip(values,ids):
        lname=item["layer"]
        for idx in layer_map.get(lname,[]):
            patch["data"][idx]["marker"]["size"] = val
            patch["data"][idx]["line"]["width"]  = val
    return patch

@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input("hover-ck", "value"),
    State("layer-map", "data"), State("hover-tpl", "data"),
    prevent_initial_call=True
)
def patch_hover(enabled, layer_map, tpl):
    if not layer_map:
        return no_update
    enabled=set(enabled)
    patch=Patch()
    for lname,idxs in layer_map.items():
        if lname=="dynamic":        # dynamic traces always keep their own hover
            continue
        for i in idxs:
            if lname in enabled:
                patch["data"][i]["hovertemplate"]=tpl[str(i)]
                patch["data"][i]["hoverinfo"]="all"
            else:
                patch["data"][i]["hovertemplate"]=None
                patch["data"][i]["hoverinfo"]="skip"
    return patch

# ----------------------------------------------------------------------
# 4)  Tick slider patches the dynamic traces --------------------------
@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input("tick-sl", "value"),
    State("dyn-data",  "data"),
    State("layer-map", "data"),
    prevent_initial_call=True
)
def patch_tick(tick_idx, dyn_data, layer_map):
    if not dyn_data or not layer_map or "dynamic" not in layer_map:
        return no_update
    if tick_idx is None:
        return no_update
    frame = dyn_data["frames"][tick_idx]
    patch = Patch()
    for idx,(xs,ys,txt,style) in zip(layer_map["dynamic"], frame):
        # xs may be empty → actor not present in this tick
        patch["data"][idx]["x"] = xs
        patch["data"][idx]["y"] = ys
        patch["data"][idx]["text"] = txt
        for prop,val in style.items():            # e.g. "marker.color"
            tgt = patch["data"][idx]
            parts = prop.split(".")
            for p in parts[:-1]:
                tgt = tgt[p]
            tgt[parts[-1]] = val
    return patch

# ----------------------------------------------------------------------
# 5)  Play / Pause / Speed --------------------------------------------
@app.callback(
    Output("play-ivl", "disabled"),
    Output("play-ivl", "interval"),
    Input("play-btn",  "n_clicks"),
    Input("pause-btn", "n_clicks"),
    State("speed-sl",  "value"),
    prevent_initial_call=True
)
def control_play(play, pause, speed):
    trig = ctx.triggered_id
    playing = trig == "play-btn"
    period  = int(1000 / max(speed, 0.1))
    return (not playing), period


@app.callback(
    Output("tick-sl", "value"),
    Input("play-ivl", "n_intervals"),
    State("tick-sl", "value"),
    State("tick-sl", "max"),
    prevent_initial_call=True
)
def advance_tick(_, cur, mx):
    if cur is None:
        return no_update
    nxt = (cur + 1) % (mx + 1) if mx >= 0 else 0
    return nxt

# ----------------------------------------------------------------------
if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)
