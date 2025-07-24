# layers/utils.py
from typing import List

PALETTE = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd',
           '#8c564b','#e377c2','#7f7f7f','#bcbd22','#17becf']

def color_for_road(road_id: int, palette: List[str] = PALETTE) -> str:
    return palette[road_id % len(palette)]

def rgba(hex_color: str, alpha: float) -> str:
    if hex_color.startswith("#"):
        r = int(hex_color[1:3],16); g = int(hex_color[3:5],16); b = int(hex_color[5:7],16)
        return f"rgba({r},{g},{b},{alpha:.3f})"
    return hex_color
