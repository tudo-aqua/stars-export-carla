# dynamic/actor_traces.py
from __future__ import annotations
from typing import Dict, List, Tuple
import numpy as np
import plotly.graph_objects as go
from carla_data_classes import TickData
from .renderers import RENDERER_REGISTRY, BaseRenderer


def _choose_renderer(actor) -> "Type[BaseRenderer]":
    for cls in RENDERER_REGISTRY.values():
        if cls.matches(actor):
            return cls
    print(f"No renderer for actor type '{actor.type}'")
    return None


def build_dynamic_templates(ticks: List[TickData]):
    """
    Returns
    -------
    templates : list[go.Scatter] (one per actor that has a renderer)
    per_tick  : list[list[tuple]] (xs, ys, text, style_dict) per frame
    """
    ids: Dict[int, int] = {}  # actor_id  -> trace_index
    renderer_for: Dict[int, BaseRenderer] = {}
    templates: List[go.Scatter] = []

    # ---------- 1) one immutable template per actor -------------------
    for tick in ticks:
        for ap in tick.actor_positions:
            aid = ap.actor.id
            if aid in ids:
                continue

            rend_cls = _choose_renderer(ap.actor)
            if rend_cls is None:
                continue

            idx = len(templates)
            ids[aid] = idx
            renderer_for[aid] = rend_cls

            base_col = "#ffffff"
            templates.append(rend_cls.make_template(ap.actor, base_col))

    # ---------- 2) per-tick payload ----------------------------------
    empty = (np.empty(0), np.empty(0), "", {})
    per_tick: List[List[Tuple]] = []

    for tick in ticks:
        frame = [empty] * len(templates)
        for ap in tick.actor_positions:
            aid = ap.actor.id
            if aid not in renderer_for:
                continue
            idx = ids[aid]
            rend_cls = renderer_for[aid]
            # Try new signature first (actor, actor_position), fall back to (actor)
            try:
                payload = rend_cls.frame_payload(ap.actor, ap)
            except TypeError:
                payload = rend_cls.frame_payload(ap.actor)
            frame[idx] = payload
        per_tick.append(frame)

    return templates, per_tick
