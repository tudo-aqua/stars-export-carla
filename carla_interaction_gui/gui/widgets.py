from __future__ import annotations

import tkinter as tk
from typing import Callable


def entry_row(parent: tk.Widget, label: str, var: tk.Variable, browse: Callable = None,
              width: int = 45) -> tk.Frame:
    """
    Creates a single row layout comprising a label, entry field, and optionally
    a browse button within the specified parent widget.

    Parameters:
    parent : tk.Widget
        The parent widget in which the entry row will be created.
    label : str
        The text to be displayed as the label in the row.
    var : tk.Variable
        The tkinter variable associated with the entry widget.
    browse : Callable, optional
        A callback function to be executed when the browse button is clicked.
    width : int
        The width of the entry field.
    """
    row = tk.Frame(parent)
    row.pack(fill="x", pady=2)
    tk.Label(row, text=label, width=26, anchor="w").pack(side="left")
    tk.Entry(row, textvariable=var, width=width).pack(side="left", fill="x", expand=True)
    if browse:
        tk.Button(row, text="...", command=browse).pack(side="left", padx=2)
    return row


def validate_number(proposed: str) -> bool:
    """
    Entry validator: allow empty, integers, or floats (with optional leading '-').
    This lets users type partial values like '-', '.', '-.' while editing.
    """
    if proposed in ("", "-", ".", "-."):
        return True
    try:
        float(proposed)
        return True
    except ValueError:
        return False
