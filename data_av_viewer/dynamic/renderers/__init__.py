# dynamic/renderers/__init__.py
"""
Pluggable renderers for dynamic Actors.
Just drop a ``*.py`` in this package and decorate the class with
@register("<nice_name>").  See *vehicle.py* and *traffic_light.py*.
"""
from __future__ import annotations

import importlib
import pathlib
import pkgutil
from typing import Dict, Type

import plotly.graph_objects as go

from carla_data_classes.dynamic import DataActor

# ---------------------------------------------------------------------
RENDERER_REGISTRY: Dict[str, "Type[BaseRenderer]"] = {}

def register(name: str):
    def deco(cls):
        RENDERER_REGISTRY[name] = cls
        cls.renderer_name = name  # convenience
        return cls
    return deco

# ---------------------------------------------------------------------
class BaseRenderer:
    """Every renderer must override these three class‑methods."""

    @classmethod
    def matches(cls, actor: DataActor) -> bool:
        "Return True if this renderer handles *actor*."
        raise NotImplementedError

    @classmethod
    def make_template(cls, actor: DataActor, base_color: str) -> go.Scatter:
        "Return a *static* Plotly trace template."
        raise NotImplementedError

    @classmethod
    def frame_payload(cls, actor: DataActor):
        """
        Return a 4‑tuple (xs, ys, hover_text, style_dict) for *one tick*.
        `style_dict` may patch arbitrary nested props, e.g. {'marker.color': '#ff0'}.
        """
        raise NotImplementedError

# ---------------------------------------------------------------------
# auto‑import all modules in this directory
_pkg_path = pathlib.Path(__file__).parent
for m in pkgutil.iter_modules([str(_pkg_path)]):
    importlib.import_module(f"{__name__}.{m.name}")
