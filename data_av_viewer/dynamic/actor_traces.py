# dynamic/actor_traces.py

from __future__ import annotations

from typing import Dict, List, Tuple, Any

import numpy as np
import plotly.graph_objects as go

from carla_data_classes.dynamic import DataActor, TickData
from .renderers import RENDERER_REGISTRY, BaseRenderer


def _choose_renderer(actor: DataActor) -> type[BaseRenderer] | None:
    for cls in RENDERER_REGISTRY.values():
        if cls.matches(actor):
            return cls
    return None

def build_dynamic_templates(
        ticks: List[TickData]
) -> Tuple[List[go.BaseTraceType], List[List[Tuple[Any,Any,Any,Any]]]]:
    """
    Returns:
      templates : list of plotly trace templates (may be multiple per actor)
      per_tick  : for each tick, a frame list of (xs, ys, text, style) tuples
                   aligned 1:1 with templates
    """
    ids: Dict[int, List[int]] = {}        # actor_id -> list of template indices
    renderer_for: Dict[int, Any] = {}     # actor_id -> renderer class
    templates: List[go.BaseTraceType] = []

    # 1) Build templates
    for tick in ticks:
        for ap in tick.actor_positions:
            actor = ap.actor
            aid = actor.id
            if aid in ids:
                continue

            rend_cls = _choose_renderer(actor)
            if rend_cls is None:
                continue

            # If the renderer provides make_templates, use it; else fall back to make_template
            if hasattr(rend_cls, "make_templates"):
                mts = rend_cls.make_templates(actor)
                if not isinstance(mts, list):
                    mts = [mts]
            else:
                # legacy single‐template API, pass a base color (ignored by most)
                base_color = "#ffffff"
                mts = [rend_cls.make_template(actor, base_color)]

            start = len(templates)
            templates.extend(mts)
            ids[aid] = list(range(start, len(templates)))
            renderer_for[aid] = rend_cls

    # 2) Build per‐tick frames
    empty = (np.empty(0), np.empty(0), "", {})
    per_tick: List[List[Tuple[Any,Any,Any,Any]]] = []

    for tick in ticks:
        frame = [empty] * len(templates)
        for ap in tick.actor_positions:
            actor = ap.actor
            aid = actor.id
            if aid not in renderer_for:
                continue
            rend_cls = renderer_for[aid]

            # Try new multi‐payload API first, else fall back
            try:
                # renderer_cls.frame_payload(actor, actor_position) → list or tuple
                fps = rend_cls.frame_payload(actor, ap, tick)
            except TypeError:
                # legacy single‐payload API
                fps = rend_cls.frame_payload(actor)

            if not isinstance(fps, list):
                fps = [fps]

            for idx, payload in zip(ids[aid], fps):
                frame[idx] = payload

        per_tick.append(frame)

    return templates, per_tick
