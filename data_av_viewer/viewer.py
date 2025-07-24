#!/usr/bin/env python3
import base64, importlib, pkgutil, pathlib
from typing import Dict, List

import plotly.graph_objects as go
from dash import Patch, ALL
from dash_extensions.enrich import Dash, dcc, html, Input, Output, State, no_update

from carla_data_classes import TickData, DataBlock
from viewer_store import ViewerStore
from layers.base_layer import build_all_traces, LAYER_REGISTRY

# auto‑import all layer modules
_layers_path = pathlib.Path(__file__).parent / "layers"
for m in pkgutil.iter_modules([str(_layers_path)]):
    importlib.import_module(f"layers.{m.name}")


# helpers --------------------------------------------------------------
def decode_upload(c): return base64.b64decode(c.split(",", 1)[1])


def load_raw_json(raw):
    try:
        return TickData.from_json(raw), []
    except:
        pass
    try:
        return None, DataBlock.from_json(raw)
    except:
        pass
    import orjson
    data = orjson.loads(raw)
    if isinstance(data, list): return None, [DataBlock.from_dict(d) for d in data]
    raise ValueError("unknown schema")


# UI constants ---------------------------------------------------------
LAYER_OPTIONS = [{"label": n.replace("_", " ").title(), "value": n}
                 for n in LAYER_REGISTRY]
DEFAULT_LAYERS = ["roads"]
HOVER_DEFAULT = ["roads"]
SIZE_LAYERS = [name for name, cls in LAYER_REGISTRY.items()
               if getattr(cls, "slider_key", None)]
DEFAULT_SIZES = {name: cls.default_size for name, cls in LAYER_REGISTRY.items()
                 if getattr(cls, "default_size", None)}


def size_slider(layer):
    return html.Div([
        html.Label(layer.replace("_", " ").title()),
        dcc.Slider(1, 20, 1, value=DEFAULT_SIZES[layer],
                   id={"type": "size", "layer": layer})
    ])


# Dash layout ----------------------------------------------------------
app = Dash(__name__, suppress_callback_exceptions=True)
app.layout = html.Div([
    html.Div([
        html.H3("CARLA Viewer"),
        dcc.Upload(id="upload", children=html.Div(["Drag & Drop or ", html.A("Select JSON")]),
                   style={"width": "100%", "height": "60px", "lineHeight": "60px",
                          "border": "1px dashed", "borderRadius": "5px",
                          "textAlign": "center", "margin": "10px 0"}),
        html.Div(id="msg", style={"fontSize": "12px", "color": "#555"}),

        html.H4("Layers"),
        dcc.Checklist(id="layer-ck", options=LAYER_OPTIONS, value=DEFAULT_LAYERS,
                      inputStyle={"margin-right": "4px", "margin-left": "12px"}),

        html.H4("Hover enabled for"),
        dcc.Checklist(id="hover-ck", options=LAYER_OPTIONS, value=HOVER_DEFAULT,
                      inputStyle={"margin-right": "4px", "margin-left": "12px"}),

        html.H4("Marker sizes"),
        *(size_slider(l) for l in SIZE_LAYERS),
    ], style={"width": "300px", "float": "left", "padding": "10px"}),

    html.Div([dcc.Graph(id="fig", style={"height": "100vh"})],
             style={"margin-left": "320px"}),

    dcc.Store(id="store-json"),
    dcc.Store(id="layer-map"),
    dcc.Store(id="hover-tpl")
])


# Upload ----------------------------------------------------------------
@app.callback(
    Output("store-json", "data"), Output("msg", "children"),
    Input("upload", "contents"), State("upload", "filename"),
    prevent_initial_call=True
)
def parse_upload(contents, fname):
    if not contents: return no_update, no_update
    try:
        tick, blocks = load_raw_json(decode_upload(contents))
        if isinstance(blocks, DataBlock):
            blocks = [blocks]
        elif blocks is None:
            blocks = []
        store = ViewerStore.from_source(blocks, tick)
        msg = f"Loaded '{fname}'  | midpoints:{len(store.dfs['midpoints'])}"
        return store.to_json(), msg
    except Exception as e:
        return {}, f"Error: {e}"


# Build figure ----------------------------------------------------------
@app.callback(
    Output("fig", "figure"), Output("layer-map", "data"), Output("hover-tpl", "data"),
    Input("store-json", "data"), State("layer-ck", "value"),
    prevent_initial_call=True
)
def build_fig(json_data, visible_layers):
    if not json_data: return go.Figure(), {}, {}
    store = ViewerStore.from_json(json_data)
    traces, layer_map = build_all_traces(store, visible_layers, DEFAULT_SIZES)
    fig = go.Figure(data=traces)
    fig.update_layout(
        hovermode="closest",
        uirevision="keep",
        legend=dict(itemsizing="constant"),
        xaxis=dict(scaleanchor="y", scaleratio=1, showgrid=False),
        yaxis=dict(showgrid=False)
    )
    tmpl = {i: t.hovertemplate for i, t in enumerate(fig.data)}
    return fig, layer_map, tmpl


# Layer visibility ------------------------------------------------------
@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input("layer-ck", "value"), State("layer-map", "data"),
    prevent_initial_call=True
)
def toggle_layers(sel, layer_map):
    if not layer_map: return no_update
    patch = Patch();
    sel = set(sel)
    for lname, idxs in layer_map.items():
        for i in idxs: patch["data"][i]["visible"] = lname in sel
    return patch


# Marker sizes ----------------------------------------------------------
@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input({"type": "size", "layer": ALL}, "value"),
    State({"type": "size", "layer": ALL}, "id"), State("layer-map", "data"),
    prevent_initial_call=True
)
def patch_sizes(values, ids, layer_map):
    if not layer_map: return no_update
    patch = Patch()
    for val, item in zip(values, ids):
        lname = item["layer"]
        for idx in layer_map.get(lname, []):
            # safe to set both: Plotly ignores irrelevant props
            patch["data"][idx]["marker"]["size"] = val  # point layers
            patch["data"][idx]["line"]["width"] = val  # line layers (lanes, roads)
    return patch


# Hover enable ----------------------------------------------------------
@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input("hover-ck", "value"),
    State("layer-map", "data"), State("hover-tpl", "data"),
    prevent_initial_call=True
)
def patch_hover(enabled, layer_map, tpl):
    if not layer_map: return no_update
    enabled = set(enabled);
    patch = Patch()
    for lname, idxs in layer_map.items():
        for i in idxs:
            if lname in enabled:
                patch["data"][i]["hovertemplate"] = tpl[str(i)] if isinstance(tpl, dict) else tpl[i]
                patch["data"][i]["hoverinfo"] = "all"
            else:
                patch["data"][i]["hovertemplate"] = None
                patch["data"][i]["hoverinfo"] = "skip"
    return patch


if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)
