from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable

from carla_interaction_gui.carla_versions import python_api_versions
from carla_interaction_gui.gui.constants import CARLA_DOCKER_VERSIONS


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


def carla_target_rows(parent: tk.Widget, app) -> None:
    """
    Rows choosing the CARLA server and the Python API to talk to it with: an
    editable dropdown of the catalogued docker images (accepting any other
    image name or a start script path as free text, see carla_versions) and a
    dropdown of Python API versions, which follows the image selection.
    """
    row = tk.Frame(parent)
    row.pack(fill="x", pady=2)
    tk.Label(row, text="CARLA executable:", width=26, anchor="w").pack(side="left")
    ttk.Combobox(row, textvariable=app.carla_executable_variable, values=list(CARLA_DOCKER_VERSIONS),
                 width=42).pack(side="left", fill="x", expand=True)
    tk.Button(row, text="...", command=lambda: app.open_file_dialog(app.carla_executable_variable)
              ).pack(side="left", padx=2)

    row = tk.Frame(parent)
    row.pack(fill="x", pady=2)
    tk.Label(row, text="CARLA Python API:", width=26, anchor="w").pack(side="left")
    ttk.Combobox(row, textvariable=app.carla_python_api_variable, state="readonly",
                 values=python_api_versions(), width=42).pack(side="left", fill="x", expand=True)


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
