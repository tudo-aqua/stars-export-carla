from __future__ import annotations
import base64, importlib, pkgutil, pathlib, orjson
from typing import List, Tuple

import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output, State, Patch, ALL, ctx, no_update

from carla_data_classes import TickData, DataBlock
from viewer_store import ViewerStore
from layers.base_layer import build_all_traces, LAYER_REGISTRY
from dynamic.actor_traces import build_dynamic_templates  # <── our helper

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
HOVER_DEFAULT = ["roads"]
SIZE_LAYERS = [n for n, c in LAYER_REGISTRY.items() if getattr(c, "slider_key", None)]
DEFAULT_SIZES = {n: c.default_size for n, c in LAYER_REGISTRY.items()
                 if getattr(c, "default_size", None)}


# ----------------------------------------------------------------------
# small re‑usable widgets ----------------------------------------------
def _size_slider(layer):
    return html.Div([
        html.Label(layer.replace("_", " ").title()),
        dcc.Slider(1, 20, 1, updatemode="drag",
                   value=DEFAULT_SIZES[layer],
                   id={"type": "size", "layer": layer})
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
                   style={"width": "100%", "height": "60px", "lineHeight": "60px",
                          "border": "1px dashed", "borderRadius": "5px",
                          "textAlign": "center", "margin": "10px 0"}),
        html.Div(id="msg", style={"fontSize": "12px", "color": "#555"}),

        html.H4("Static layers"),
        dcc.Checklist(id="layer-ck", options=LAYER_OPTIONS, value=DEFAULT_LAYERS,
                      inputStyle={"margin-right": "4px", "margin-left": "12px"}),

        html.H4("Hover enabled for"),
        dcc.Checklist(id="hover-ck", options=LAYER_OPTIONS, value=HOVER_DEFAULT,
                      inputStyle={"margin-right": "4px", "margin-left": "12px"}),

        html.H4("Marker sizes"),
        *(_size_slider(l) for l in SIZE_LAYERS),

        html.Hr(),

        html.H4("Dynamic replay"),
        dcc.Slider(id="tick-sl", min=0, max=0, step=1, value=0, updatemode="drag",
                   tooltip={"placement": "bottom", "always_visible": True}),
        html.Div([
            html.Button("▶ Play", id="play-btn", n_clicks=0,
                        style={"width": "60px"}),
            html.Button("⏸ Pause", id="pause-btn", n_clicks=0,
                        style={"width": "60px", "marginLeft": "6px"}),
            html.Span("  speed"),
            dcc.Slider(id="speed-sl", min=0.2, max=5, updatemode="drag",
                       step=0.2, value=1.0, tooltip={"placement": "bottom"})
        ], style={"marginTop": "4px"})
    ], style={"width": "300px", "float": "left", "padding": "10px"}),

    # ---- main figure -------------------------------------------------
    html.Div([dcc.Graph(id="fig", style={"height": "100vh"})],
             style={"margin-left": "320px"}),

    # ---- client‑side stores -----------------------------------------
    dcc.Store(id="store-json"),  # static layers – same as before
    dcc.Store(id="dyn-ticks"),  # list[TickData]     (raw)
    dcc.Store(id="dyn-data"),  # dict {templates, per_tick}
    dcc.Store(id="layer-map"),  # map layer‑name -> trace‑idxs
    dcc.Store(id="hover-tpl")  # original hover templates
])

# An interval that drives playback (disabled by default)
app.layout.children.append(
    dcc.Interval(id="play-ivl", disabled=True, interval=500)
)


@app.callback(
    Output("store-json", "data"),
    Output("dyn-ticks", "data"),
    Output("msg", "children"),
    Input("upload", "contents"),
    State("upload", "filename"),
    State("store-json", "data"),  # keep prior static if dynamic upload
    State("dyn-ticks", "data"),  # keep prior dynamic if static upload
    prevent_initial_call=True
)
def parse_upload(contents, fname, prior_store_json, prior_dyn_json):
    if not contents:
        return prior_store_json, prior_dyn_json, "No file."

    try:
        raw = _decode_upload(contents)
        ticks, blocks = _load_raw_json(raw)

        # ---- static upload (replace static, keep dynamic) ----
        if blocks:
            store = ViewerStore.from_source(blocks, ticks[0] if ticks else None)
            msg = f"Loaded static '{fname}' | blocks:{len(blocks)}"
            return store.to_json(), (prior_dyn_json or ""), msg

        # ---- dynamic upload (replace dynamic, keep static) ----
        if ticks:
            dyn_json = orjson.dumps([t.to_dict() for t in ticks]).decode()
            msg = f"Loaded dynamic '{fname}' | ticks:{len(ticks)}"
            return prior_store_json, dyn_json, msg

        # Nothing recognized — keep everything
        return prior_store_json, prior_dyn_json, f"'{fname}': no static or dynamic content found."

    except Exception as e:
        # On error, keep everything as-is
        return prior_store_json, prior_dyn_json, f"Error while loading '{fname}': {e}"


# ----------------------------------------------------------------------
# 2)  Build base figure + dynamic templates ---------------------------
@app.callback(
    Output("fig", "figure"),
    Output("layer-map", "data"),
    Output("hover-tpl", "data"),
    Output("dyn-data", "data"),
    Output("tick-sl", "max"),
    Input("store-json", "data"),
    Input("dyn-ticks", "data"),
    State("layer-ck", "value"),
    prevent_initial_call=True
)
def build_fig(json_data, dyn_raw, visible_layers):
    static_traces, layer_map, shapes = [], {}, []
    if json_data:
        store = ViewerStore.from_json(json_data)
        static_traces, layer_map, shapes = build_all_traces(store, visible_layers, DEFAULT_SIZES)

    # dynamic (optional)
    dyn_templates, per_tick = [], []
    if dyn_raw:
        ticks = [TickData.from_dict(d) for d in orjson.loads(dyn_raw)]
        dyn_templates, per_tick = build_dynamic_templates(ticks)

    fig = go.Figure(data=static_traces + dyn_templates)
    if shapes:
        fig.layout.shapes = tuple(shapes)

    fig.update_layout(
        dragmode="pan", hovermode="closest", uirevision="keep",
        legend=dict(itemsizing="constant"),
        xaxis=dict(scaleanchor="y", scaleratio=1, showgrid=False),
        yaxis=dict(showgrid=False)
    )

    tmpl = {i: t.hovertemplate for i, t in enumerate(fig.data)}
    if dyn_templates:
        layer_map["dynamic"] = list(range(len(static_traces),
                                          len(static_traces) + len(dyn_templates)))

    # ─── PRIME FRAME-0 FOR DYNAMICS ────────────────────────────────────
    # (so hover/text/style are already set before the slider ever moves)
    if per_tick:
        frame0 = per_tick[0]
        # only the dynamic traces (they live after all static_traces)
        for tr, (xs, ys, txt, style) in zip(dyn_templates, frame0):
            # geometry
            tr.x = xs
            tr.y = ys
            # hover-text
            tr.text = txt
            # any per-frame style keys, e.g. "marker.color"
            for prop, val in style.items():
                # split e.g. "marker.color" → target.marker.color = val
                target = tr
                *path, leaf = prop.split(".")
                for p in path:
                    target = getattr(target, p)
                setattr(target, leaf, val)
    # ─────────────────────────────────────────────────────────────────────

    tick_max = (len(per_tick) - 1) if per_tick else 0
    return fig, layer_map, tmpl, {"frames": per_tick}, tick_max


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

    sel = set(sel)
    patch = Patch()

    # 1) traces (as before)
    for lname, idxs in layer_map.items():
        if lname == "dynamic" or lname.endswith("_shapes_range") or lname.endswith("_shape_xy"):
            continue
        for i in idxs:
            patch["data"][i]["visible"] = lname in sel

    # 2) shapes (NEW)
    for lname, idxs in list(layer_map.items()):
        if not lname.endswith("_shapes_range"):
            continue
        base = lname[:-len("_shapes_range")]
        visible = base in sel
        s0, s1 = idxs
        for i in range(s0, s1 + 1):
            patch["layout"]["shapes"][i]["visible"] = visible

    return patch


@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input({"type": "size", "layer": ALL}, "value"),
    State({"type": "size", "layer": ALL}, "id"),
    State("layer-map", "data"),
    prevent_initial_call=True
)
def patch_sizes(values, ids, layer_map):
    if not layer_map:
        return no_update

    patch = Patch()

    # helper: same formulas as layers/traffic_lights._geometry_from_size
    def tl_geom(size):
        s = float(size or 0)
        lamp_r = 0.05 * s
        gap = 0.60 * lamp_r
        pad = 0.80 * lamp_r
        total_w = 2 * lamp_r + 2 * pad
        total_h = 3 * (2 * lamp_r) + 2 * gap + 2 * pad
        border_w = max(1, int(s / 3))
        D = 2.0 * lamp_r + gap  # center-to-center vertical spacing
        return lamp_r, gap, pad, total_w, total_h, border_w, D

    for val, item in zip(values, ids):
        lname = item["layer"]

        # 1) existing trace-size logic
        for idx in layer_map.get(lname, []):
            patch["data"][idx]["marker"]["size"] = val
            patch["data"][idx]["line"]["width"] = val

        # 2) traffic_light shapes (rect + 3 circles per light)
        if lname == "traffic_lights" and (lname + "_shapes_range") in layer_map:
            s0, s1 = layer_map[lname + "_shapes_range"]
            xs, ys = layer_map.get(lname + "_shape_xy", [[], []])
            if xs and ys:
                # NOTE: correct unpack order + variable name
                lamp_r, gap, pad, total_w, total_h, border_w, D = tl_geom(val)
                i = s0
                for x, y in zip(xs, ys):
                    # rectangle
                    x0 = x - total_w / 2.0
                    x1 = x + total_w / 2.0
                    y0 = y - total_h / 2.0
                    y1 = y + total_h / 2.0
                    patch["layout"]["shapes"][i]["x0"] = x0
                    patch["layout"]["shapes"][i]["x1"] = x1
                    patch["layout"]["shapes"][i]["y0"] = y0
                    patch["layout"]["shapes"][i]["y1"] = y1
                    patch["layout"]["shapes"][i]["line"]["width"] = border_w
                    i += 1
                    # three circles (top, mid, bottom)
                    for off in (+D, 0.0, -D):
                        cy = y + off
                        patch["layout"]["shapes"][i]["x0"] = x - lamp_r
                        patch["layout"]["shapes"][i]["x1"] = x + lamp_r
                        patch["layout"]["shapes"][i]["y0"] = cy - lamp_r
                        patch["layout"]["shapes"][i]["y1"] = cy + lamp_r
                        i += 1

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

    enabled = set(enabled)
    patch = Patch()

    for lname, idxs in layer_map.items():
        # Skip non-trace entries
        if lname == "dynamic" or lname.endswith("_shapes_range") or lname.endswith("_shape_xy"):
            continue

        show_hover = lname in enabled
        for i in idxs:
            if show_hover:
                patch["data"][i]["hovertemplate"] = tpl[str(i)]
                patch["data"][i]["hoverinfo"] = "all"
            else:
                patch["data"][i]["hovertemplate"] = None
                patch["data"][i]["hoverinfo"] = "skip"

    return patch


# ----------------------------------------------------------------------
# 4)  Tick slider patches the dynamic traces --------------------------
@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Input("tick-sl", "value"),  # when the slider moves…
    Input("dyn-data", "data"),  # …or when new dynamic data loads ← NEW
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
    for idx, (xs, ys, txt, style) in zip(layer_map["dynamic"], frame):
        # xs may be empty → actor not present in this tick
        patch["data"][idx]["x"] = xs
        patch["data"][idx]["y"] = ys
        patch["data"][idx]["text"] = txt
        for prop, val in style.items():  # e.g. "marker.color"
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
    Input("play-btn", "n_clicks"),
    Input("pause-btn", "n_clicks"),
    State("speed-sl", "value"),
    prevent_initial_call=True
)
def control_play(play, pause, speed):
    trig = ctx.triggered_id
    playing = trig == "play-btn"
    period = int(1000 / max(speed, 0.1))
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
