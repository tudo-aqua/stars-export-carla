from __future__ import annotations

import base64
import importlib
import pathlib
import pkgutil
import traceback
from typing import List, Tuple

import orjson
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output, State, Patch, ALL, ctx, no_update

from carla_data_classes.dynamic import DataVehicle
from carla_data_classes.dynamic.TickData import TickData
from carla_data_classes.static.DataWorld import DataWorld
from dynamic.actor_traces import build_dynamic_templates
from layers.base_layer import build_all_traces, LAYER_REGISTRY
from viewer_store import ViewerStore

# ----------------------------------------------------------------------
# auto‑import all static layer modules ---------------------------------
_layers_path = pathlib.Path(__file__).parent / "layers"
for m in pkgutil.iter_modules([str(_layers_path)]):
    importlib.import_module(f"layers.{m.name}")


# ----------------------------------------------------------------------
# helpers --------------------------------------------------------------
def _decode_upload(contents: str) -> bytes:
    return base64.b64decode(contents.split(",", 1)[1])


def _load_raw_json(raw: bytes) -> Tuple[List[TickData], DataWorld]:
    # try TickData(s)
    try:
        return TickData.from_json(raw), []
    except Exception:
        pass

    # try DataMap
    try:
        blk = DataWorld.from_json(raw)
        return [], blk
    except Exception:
        print(traceback.format_exc())

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

FULL_MENU_STYLE = {
    "position": "absolute",
    "top": "10px",
    "left": "10px",
    "width": "300px",
    "maxHeight": "calc(100vh - 20px)",
    "overflowY": "auto",
    "backgroundColor": "rgba(255,255,255,0.9)",
    "zIndex": 1000,
    "padding": "10px",
    "borderRadius": "5px",
    "boxShadow": "0 2px 4px rgba(0,0,0,0.2)",
}

INITIAL_FIG = go.Figure()
INITIAL_FIG.update_layout(
    autosize=True,
    margin=dict(l=10, r=10, t=10, b=48),
    dragmode="pan",
    hovermode="closest",
    xaxis=dict(scaleanchor="y", scaleratio=1, showgrid=False),
    yaxis=dict(showgrid=False, automargin=True),
)

# Global cache of the per-tick frames,
# so we don’t keep shuttling ~200 MB of JSON back and forth on every slider move.
FRAMES: list = []
TIMES: list[float] = []
# Typical delta-t between ticks (seconds); set after loading dynamic data
BASE_DT_S: float = 0.05
SPEED_MAX = 3.0  # set to 2.0 if you prefer 0 → 2× instead of 0 → 3×


def speed_marks(max_speed: float):
    mid = 1.0
    return {
        0.0: "0×",
        round(max_speed / 4, 2): f"{max_speed / 4:g}×",
        mid: "1×",
        round(max_speed / 2, 2): f"{max_speed / 2:g}×",
        max_speed: f"{max_speed:g}×",
    }


app.layout = html.Div([
    # 2) the toggle button
    html.Button(
        "☰ Menu",
        id="menu-toggle",
        n_clicks=0,
        className="menu-toggle open"
    ),

    # 3) your existing menu contents, wrapped and given the full style
    html.Div([
        html.H3("CARLA Viewer"),
        dcc.Upload(
            id="upload", children=html.Div(["Drag & Drop or ", html.A("Select JSON")]),
            className="upload-area"
        ),
        html.Div(id="msg", style={"fontSize": "12px", "color": "#555"}),

        html.H4("Static layers"),
        dcc.Checklist(
            id="layer-ck",
            options=LAYER_OPTIONS,
            value=DEFAULT_LAYERS,
            inputStyle={"marginRight": "4px", "marginLeft": "12px"}
        ),

        # the rest of your menu: hover-ck, size sliders, slider & buttons
        html.H4("Hover enabled for"),
        dcc.Checklist(
            id="hover-ck",
            options=LAYER_OPTIONS,
            value=HOVER_DEFAULT,
            inputStyle={"marginRight": "4px", "marginLeft": "12px"}
        ),

        html.H4("Marker sizes"),
        *(_size_slider(l) for l in SIZE_LAYERS),

        html.Hr(),
        html.H4("Dynamic replay"),
        dcc.Slider(id="tick-sl", min=0, max=0, step=1, value=0,
                   updatemode="drag", tooltip={"placement": "bottom", "always_visible": True}),
        html.Div([
            html.Button("▶ Play", id="play-btn", n_clicks=0, style={"width": "60px"}),
            html.Button("⏸ Pause", id="pause-btn", n_clicks=0,
                        style={"width": "60px", "marginLeft": "6px"}),
            html.Span(" speed"),
            dcc.Slider(
                id="speed-sl",
                min=0.0,
                max=SPEED_MAX,
                step=0.05,
                value=1.0,  # 1× in the middle
                marks=speed_marks(SPEED_MAX),
                updatemode="drag",
                tooltip={"placement": "bottom"}
            )
        ], style={"marginTop": "4px"}),
    ], id="menu-content", style=FULL_MENU_STYLE),

    # ---- main figure (full-screen) -----------------------------
    html.Div(
        dcc.Graph(
            id="fig",
            figure=INITIAL_FIG,
            config={
                "scrollZoom": True,
                "doubleClick": "reset",
            },
            className="graph-container"
        )
    ),

    # ---- client-side stores -----------------------------------
    dcc.Store(id="store-json"),
    dcc.Store(id="dyn-ticks"),
    dcc.Store(id="layer-map"),
    dcc.Store(id="hover-tpl"),
])

# An interval that drives playback (disabled by default)
app.layout.children.append(
    dcc.Interval(id="play-ivl", disabled=True, interval=500)
)


@app.callback(
    Output("menu-content", "style"),
    Output("menu-toggle", "className"),
    Input("menu-toggle", "n_clicks"),
)
def toggle_menu(n):
    # odd → closed; even → open
    if n and n % 2 == 1:
        return {"display": "none"}, "menu-toggle"
    return FULL_MENU_STYLE, "menu-toggle open"


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
        ticks, data_world = _load_raw_json(raw)

        vehicle_locations = [pos.actor.location for tick in ticks for pos in tick.actor_positions if
                             isinstance(pos.actor, DataVehicle)]

        # ---- static upload (replace static, keep dynamic) ----
        if data_world:
            store = ViewerStore.from_source(data_world, ticks[0] if ticks else None)
            msg = f"Loaded static '{fname}' | Map with {len(data_world.junctions)} junctions and {len(data_world.straights)} roads"
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
    Output("tick-sl", "max"),
    Input("store-json", "data"),
    Input("dyn-ticks", "data"),
    State("layer-ck", "value"),
    prevent_initial_call=True
)
def build_fig(json_data, dyn_raw, visible_layers):
    from statistics import median
    global FRAMES, TIMES, BASE_DT_S

    FRAMES.clear()
    TIMES.clear()

    # --- static layers ---
    static_traces, layer_map, shapes = [], {}, []
    if json_data:
        store = ViewerStore.from_json(json_data)
        static_traces, layer_map, shapes = build_all_traces(
            store, visible_layers, DEFAULT_SIZES
        )

    # --- dynamic: build templates + cache frames & times ---
    dyn_templates, per_tick = [], []
    if dyn_raw:
        ticks = [TickData.from_dict(d) for d in orjson.loads(dyn_raw)]
        dyn_templates, per_tick = build_dynamic_templates(ticks)

        # cache frames (for patch_tick)
        FRAMES.extend(per_tick)

        # extract a time value (seconds) from each TickData
        def tval(t):
            for name in ("timestamp", "time", "sim_time", "elapsed", "current_time", "current_tick"):
                if hasattr(t, name):
                    return float(getattr(t, name))
            raise AttributeError("TickData lacks a usable time attribute")

        TIMES.extend(tval(t) for t in ticks)

        # compute a robust base Δt (median of positive diffs) as fallback
        if len(TIMES) >= 2:
            diffs = [b - a for a, b in zip(TIMES, TIMES[1:]) if (b - a) > 1e-9]
            if diffs:
                BASE_DT_S = max(median(diffs), 1e-3)
            else:
                span = max(TIMES[-1] - TIMES[0], 1e-3)
                BASE_DT_S = span / max(len(TIMES) - 1, 1)

    # --- assemble figure ---
    fig = go.Figure(data=static_traces + dyn_templates)
    if shapes:
        fig.layout.shapes = tuple(shapes)

    fig.update_layout(
        autosize=True,
        margin=dict(l=10, r=10, t=10, b=48),
        dragmode="pan",
        hovermode="closest",
        uirevision="keep",
        legend=dict(itemsizing="constant"),
        xaxis=dict(scaleanchor="y", scaleratio=1, showgrid=False),
        yaxis=dict(showgrid=False, automargin=True),
    )

    # prime tick-0 so hover & style work immediately
    if per_tick:
        frame0 = per_tick[0]
        for tr, (xs, ys, txt, style) in zip(dyn_templates, frame0):
            tr.x = xs
            tr.y = ys
            tr.text = txt
            for prop, val in style.items():
                target = tr
                *path, leaf = prop.split(".")
                for p in path:
                    target = getattr(target, p)
                setattr(target, leaf, val)

    tmpl = {i: t.hovertemplate for i, t in enumerate(fig.data)}
    if dyn_templates:
        layer_map["dynamic"] = list(
            range(len(static_traces), len(static_traces) + len(dyn_templates))
        )

    tick_max = len(per_tick) - 1 if per_tick else 0
    return fig, layer_map, tmpl, tick_max


# ----------------------------------------------------------------------
# 3)  Static‑layer toggles / sizes / hover   – unchanged callbacks -----
@app.callback(
    Output("fig", "figure", allow_duplicate=True),
    Output("layer-map", "data", allow_duplicate=True),
    Output("hover-tpl", "data", allow_duplicate=True),
    Input("layer-ck", "value"),
    State("layer-map", "data"),
    State("store-json", "data"),
    State("fig", "figure"),
    prevent_initial_call=True
)
def toggle_layers(sel, layer_map, store_json, cur_fig):
    """
    Fast path: if all selected layers already exist in the figure, just patch visibility.
    Lazy path: if a selected layer (e.g., 'midpoints') is missing, build it now and append.
    """
    if not layer_map:
        return no_update, layer_map, no_update

    sel = set(sel or [])
    # layers already present as traces (ignore dynamic + shape helpers)
    present = {
        k for k in layer_map
        if k != "dynamic" and not k.endswith("_shapes_range") and not k.endswith("_shape_xy")
    }
    missing = [lname for lname in sel if lname not in present]

    # --- Lazy build for midpoints (and any other future heavy layers) ---
    if store_json and missing:
        # Only bother rebuilding if the missing ones include 'midpoints'
        if "midpoints" in missing:
            # Build only the missing layers as static traces
            store = ViewerStore.from_json(store_json)
            new_traces, new_map, new_shapes = build_all_traces(
                store, missing, DEFAULT_SIZES
            )

            # Start from current figure and append
            fig = go.Figure(cur_fig)  # copy existing data/layout

            # append new static traces
            start_idx = len(fig.data)
            for tr in new_traces:
                fig.add_trace(tr)

            # append shapes, if any (with index offset)
            if new_shapes:
                old_shapes = list(fig.layout.shapes) if fig.layout.shapes else []
                s0 = len(old_shapes)
                old_shapes.extend(new_shapes)
                fig.layout.shapes = tuple(old_shapes)

                # bring across any *_shapes_range and *_shape_xy entries
                for lname, idxs in new_map.items():
                    if lname.endswith("_shapes_range"):
                        # offset range by existing shapes count
                        layer_map[lname] = [s0 + idxs[0], s0 + idxs[1]]
                    elif lname.endswith("_shape_xy"):
                        # these are coords, no offset
                        layer_map[lname] = idxs

            # record absolute trace indices for each newly built layer
            for lname, rel_idxs in new_map.items():
                if lname.endswith("_shapes_range") or lname.endswith("_shape_xy"):
                    continue
                layer_map[lname] = [start_idx + i for i in rel_idxs]

            # set visibility according to current selection
            for lname, idxs in layer_map.items():
                if lname == "dynamic" or lname.endswith("_shapes_range") or lname.endswith("_shape_xy"):
                    continue
                vis = lname in sel
                for i in idxs:
                    fig.data[i].visible = vis

            # refresh hover template store to include appended traces
            tmpl = {i: t.hovertemplate for i, t in enumerate(fig.data)}
            return fig, layer_map, tmpl

    # --- Fast path: just toggle visibility for layers we already have ---
    patch = Patch()

    # traces
    for lname, idxs in layer_map.items():
        if lname == "dynamic" or lname.endswith("_shapes_range") or lname.endswith("_shape_xy"):
            continue
        visible = lname in sel
        for i in idxs:
            patch["data"][i]["visible"] = visible

    # shapes (rects/circles for traffic lights etc.)
    for lname, idxs in list(layer_map.items()):
        if not lname.endswith("_shapes_range"):
            continue
        base = lname[:-len("_shapes_range")]
        visible = base in sel
        s0, s1 = idxs
        for i in range(s0, s1 + 1):
            patch["layout"]["shapes"][i]["visible"] = visible

    return patch, layer_map, no_update


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
        d = 2.0 * lamp_r + gap  # center-to-center vertical spacing
        return lamp_r, gap, pad, total_w, total_h, border_w, d

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
                lamp_r, gap, pad, total_w, total_h, border_w, d = tl_geom(val)
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
                    for off in (+d, 0.0, -d):
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
    Input("tick-sl", "value"),
    State("layer-map", "data"),
    prevent_initial_call=True
)
def patch_tick(tick_idx, layer_map):
    from dash import no_update
    global FRAMES

    if tick_idx is None or not FRAMES or "dynamic" not in layer_map:
        return no_update

    frame = FRAMES[tick_idx]
    patch = Patch()
    for idx, (xs, ys, txt, style) in zip(layer_map["dynamic"], frame):
        patch["data"][idx]["x"] = xs
        patch["data"][idx]["y"] = ys
        patch["data"][idx]["text"] = txt
        for prop, val in style.items():
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
    Output("tick-sl", "value"),
    Input("play-btn", "n_clicks"),
    Input("pause-btn", "n_clicks"),
    Input("play-ivl", "n_intervals"),  # fires each frame while playing
    Input("speed-sl", "value"),  # live speed changes
    State("tick-sl", "value"),
    State("tick-sl", "max"),
    State("play-ivl", "disabled"),
    prevent_initial_call=True
)
def player(play_clicks, pause_clicks, _n_intervals, speed, cur_tick, tick_max, is_disabled):
    """
    Single source of truth for playback:
      - Play: enable interval with dt = actual(current→next)/speed
      - Pause: disable interval
      - Interval: advance tick; retime for next→following / speed
      - Speed change: retime immediately if playing; speed==0 pauses
    """

    # helpers
    def next_idx(i: int | None) -> int:
        if i is None:
            return 0
        return i + 1 if (tick_max is not None and i + 1 <= tick_max) else 0

    def dt_seconds(cur: int | None, nxt: int) -> float:
        # use recorded times if available; else BASE_DT_S
        if TIMES and len(TIMES) >= 2 and cur is not None:
            if nxt > cur:
                dt = TIMES[nxt] - TIMES[cur]
                return dt if dt > 0 else BASE_DT_S
            # wrap around
            return BASE_DT_S
        return BASE_DT_S

    def interval_ms_for(cur: int | None, nxt: int, spd: float) -> int | None:
        if spd <= 0:
            return None  # means "pause"
        dt = dt_seconds(cur, nxt)
        # Convert to ms and scale by speed; clamp to ≥1ms
        return max(int((dt * 1000) / max(spd, 1e-3)), 1)

    trig = ctx.triggered_id

    # --- PAUSE button ---
    if trig == "pause-btn":
        # Disable interval, keep its current ms value and tick
        return True, no_update, no_update

    # --- speed changed (apply immediately) ---
    if trig == "speed-sl":
        # If slider is at 0 → pause
        if speed <= 0:
            return True, no_update, no_update
        # If playing, retime interval right now for current→next
        if not is_disabled and cur_tick is not None:
            nxt = next_idx(cur_tick)
            ms = interval_ms_for(cur_tick, nxt, speed)
            return no_update, ms, no_update
        # If paused, just keep paused; new speed used on next Play
        return no_update, no_update, no_update

    # --- PLAY button ---
    if trig == "play-btn":
        if speed <= 0:
            # do not start when speed==0
            return True, no_update, no_update
        cur = 0 if cur_tick is None else cur_tick
        nxt = next_idx(cur)
        ms = interval_ms_for(cur, nxt, speed)
        return False, ms, no_update  # enable interval and set first ms

    # --- timer fired: advance one frame ---
    if trig == "play-ivl":
        if cur_tick is None or tick_max is None or tick_max < 0:
            return no_update, no_update, no_update
        # if speed was set to 0 while timer running, pause now
        if speed <= 0:
            return True, no_update, no_update
        nxt = next_idx(cur_tick)
        fol = next_idx(nxt)
        ms = interval_ms_for(nxt, fol, speed)
        return no_update, ms, nxt

    # nothing to do
    return no_update, no_update, no_update


# ----------------------------------------------------------------------
if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)
