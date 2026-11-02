from __future__ import annotations

from typing import Callable

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QFrame, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QScrollArea, QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)

from carla_interaction_gui.carla_versions import python_api_versions
from carla_interaction_gui.gui.binding import Value, bind_check, bind_combo, bind_line_edit, bind_spin
from carla_interaction_gui.gui.constants import CARLA_DOCKER_VERSIONS


class Tab(QScrollArea):
    """
    Base class of the notebook tabs: a scrollable column with a short
    description on top, form sections in the middle and the action buttons
    (always ending in a Stop button) at the bottom.
    """

    # Wider forms only get harder to read on big screens.
    MAX_CONTENT_WIDTH = 1280

    def __init__(self, app, description: str):
        super().__init__()
        self.app = app
        self.setWidgetResizable(True)
        self.setFrameShape(QFrame.Shape.NoFrame)
        body = QWidget()
        body.setMaximumWidth(self.MAX_CONTENT_WIDTH)
        self.body = QVBoxLayout(body)
        self.body.setContentsMargins(12, 12, 12, 12)
        self.body.setSpacing(10)
        self.setWidget(body)

        intro = QLabel(description)
        intro.setWordWrap(True)
        intro.setObjectName("tabDescription")
        self.body.addWidget(intro)

    def section(self, title: str) -> QFormLayout:
        """Add a titled group of label/field rows and return its layout."""
        group = QGroupBox(title)
        form = new_form(group)
        self.body.addWidget(group)
        return form

    def actions(self, *buttons: QPushButton) -> QPushButton:
        """Add the action buttons plus a Stop button (registered with the app); returns the Stop button."""
        self.body.addStretch(1)
        row = QHBoxLayout()
        for button in buttons:
            row.addWidget(button)
        row.addStretch(1)
        stop = QPushButton("Stop")
        stop.setEnabled(False)
        stop.clicked.connect(self.app.stop_worker)
        self.app.register_stop_button(stop)
        row.addWidget(stop)
        self.body.addLayout(row)
        return stop


def new_form(parent: QWidget) -> QFormLayout:
    form = QFormLayout(parent)
    form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
    form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
    return form


def primary_button(text: str, callback: Callable) -> QPushButton:
    button = QPushButton(text)
    button.setDefault(True)
    button.setObjectName("primaryButton")
    button.clicked.connect(callback)
    return button


def button(text: str, callback: Callable) -> QPushButton:
    b = QPushButton(text)
    b.clicked.connect(callback)
    return b


def line_edit(value: Value, placeholder: str = "") -> QLineEdit:
    edit = bind_line_edit(QLineEdit(), value)
    edit.setPlaceholderText(placeholder)
    return edit


def check(text: str, value: Value) -> QCheckBox:
    return bind_check(QCheckBox(text), value)


def _compact(s: QSpinBox | QDoubleSpinBox) -> QSpinBox | QDoubleSpinBox:
    """Numbers don't need the full row width."""
    s.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
    s.setMinimumWidth(max(140, s.sizeHint().width()))
    return s


def spin(value: Value, minimum: int = 0, maximum: int = 1_000_000, suffix: str = "") -> QSpinBox:
    s = QSpinBox()
    s.setRange(minimum, maximum)
    s.setSuffix(suffix)
    return _compact(bind_spin(s, value))


def double_spin(value: Value, minimum: float, maximum: float, *, decimals: int = 2, step: float = 0.1,
                suffix: str = "") -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(minimum, maximum)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setSuffix(suffix)
    return _compact(bind_spin(s, value))


def combo(value: Value, items=(), *, editable: bool = False) -> QComboBox:
    c = QComboBox()
    c.setEditable(editable)
    c.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
    c.setMinimumContentsLength(12)
    return bind_combo(c, value, items)


def browse_button(callback: Callable) -> QPushButton:
    b = button("…", callback)
    b.setToolTip("Browse")
    b.setFixedWidth(b.fontMetrics().horizontalAdvance("…") + 28)
    return b


def hbox(*widgets: QWidget, stretch_last: bool = False) -> QWidget:
    """Lay widgets out side by side; the first one takes the spare width unless stretch_last."""
    container = QWidget()
    row = QHBoxLayout(container)
    row.setContentsMargins(0, 0, 0, 0)
    for i, widget in enumerate(widgets):
        stretch = 1 if (i == len(widgets) - 1 if stretch_last else i == 0) else 0
        row.addWidget(widget, stretch)
    return container


def path_field(app, value: Value, *, file: bool = False, folder: bool = False, recording: bool = False) -> QWidget:
    """
    A path entry with browse buttons: "File…" (filtered by the recording
    extension if `recording`) and/or "Folder…".
    """
    edit = line_edit(value)
    widgets: list[QWidget] = [edit]
    both = (file or recording) and folder
    choose_file = lambda: app.choose_file(value, recording=recording)  # noqa: E731
    choose_folder = lambda: app.choose_folder(value)  # noqa: E731
    if both:
        widgets += [button("File…", choose_file), button("Folder…", choose_folder)]
    elif folder:
        widgets.append(browse_button(choose_folder))
    else:
        widgets.append(browse_button(choose_file))
    return hbox(*widgets)


def carla_panel(app) -> QGroupBox:
    """
    The CARLA server choice shared by every tab: an editable dropdown of the
    catalogued docker images (accepting any other image name or a start script
    path as free text, see carla_versions), the Python API to talk to it with
    (following the image selection), and the rendering options.
    """
    group = QGroupBox("CARLA")
    form = new_form(group)
    settings = app.settings

    executable = combo(settings["carla_executable"], CARLA_DOCKER_VERSIONS, editable=True)
    executable.lineEdit().setPlaceholderText("Docker image (e.g. carlasim/carla:0.9.16) or path to a .sh script")
    executable.setToolTip("A docker image name starts a fresh container; a path ending in .sh runs that script.")
    form.addRow("Executable:", hbox(executable, browse_button(lambda: app.choose_file(settings["carla_executable"]))))

    python_api = combo(settings["carla_python_api"], python_api_versions())
    python_api.setToolTip("CARLA Python API used by all tasks; must match the server version.")
    options = hbox(
        python_api,
        check("Render off screen", settings["render_off_screen"]),
        check("Render quality low", settings["render_quality_low"]),
        stretch_last=False,
    )
    form.addRow("Python API:", options)
    group.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
    return group
