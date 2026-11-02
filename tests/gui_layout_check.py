"""
End-to-end layout check of the CARLA interaction GUI.

Launches the real main window (offscreen by default, against a throwaway
config so the user's ~/.carla_gui_config.json is left alone), shows every tab
at several window sizes and CARLA versions, and reports:

  * clipped text: labels, buttons and checkboxes narrower than their text
  * comboboxes too narrow for their current text
  * overlapping widgets
  * tabs that need horizontal scrolling
  * widgets sticking out of the window

Screenshots of every tab are written to the output folder.

    python tests/gui_layout_check.py [--out DIR] [--visible] [--dark]

Exit code 1 if any issue was found.
"""
from __future__ import annotations

import argparse
import gc
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WINDOW_SIZES = [(1200, 950), (900, 700), (1920, 1080)]
CARLA_EXECUTABLES = ["carlasim/carla:0.9.16", "carlasim/carla:0.10.0", "my/custom-carla:latest"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default=str(Path(tempfile.gettempdir()) / "carla_gui_layout"),
                        help="folder for the screenshots")
    parser.add_argument("--visible", action="store_true", help="use the real display instead of offscreen")
    parser.add_argument("--dark", action="store_true", help="check with a dark palette")
    args = parser.parse_args()

    if not args.visible:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
    # The config path is derived from HOME on import; keep the real one untouched.
    os.environ["HOME"] = tempfile.mkdtemp(prefix="carla_gui_home_")
    os.chdir(os.environ["HOME"])  # the session log lands in the cwd with an empty config
    sys.path.insert(0, str(ROOT))

    from PySide6.QtCore import QRect
    from PySide6.QtGui import QColor, QPalette
    from PySide6.QtWidgets import (
        QAbstractButton, QAbstractSpinBox, QApplication, QComboBox, QLabel, QLineEdit, QPushButton, QWidget,
    )

    from carla_interaction_gui.gui import app as gui

    # Closing the window stops workers and kills CARLA; never touch a real server here.
    gui.kill_carla = lambda *args, **kwargs: None

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    if args.dark:
        palette = QPalette()
        for role, color in ((QPalette.ColorRole.Window, "#2b2b2b"), (QPalette.ColorRole.Base, "#1e1e1e"),
                            (QPalette.ColorRole.Text, "#e0e0e0"), (QPalette.ColorRole.WindowText, "#e0e0e0"),
                            (QPalette.ColorRole.Button, "#3a3a3a"), (QPalette.ColorRole.ButtonText, "#e0e0e0")):
            palette.setColor(role, QColor(color))
        app.setPalette(palette)
    app.setStyleSheet(gui.STYLESHEET)

    window = gui.CarlaInteractionGUI()
    window.show()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    issues: list[str] = []
    leaf_types = (QLabel, QAbstractButton, QComboBox, QLineEdit, QAbstractSpinBox)

    def visible_leaves(root: QWidget) -> list[QWidget]:
        leaves = []
        for w in root.findChildren(QWidget):
            if not isinstance(w, leaf_types) or not w.isVisibleTo(window):
                continue
            # Line edits/buttons inside comboboxes and spin boxes belong to them.
            if isinstance(w.parentWidget(), (QComboBox, QAbstractSpinBox)):
                continue
            if isinstance(w, QAbstractButton) and w.objectName().startswith("qt_"):
                continue
            leaves.append(w)
        return leaves

    def global_rect(w: QWidget) -> QRect:
        return QRect(w.mapTo(window, w.rect().topLeft()), w.size())

    def check(where: str, root: QWidget, scroll_area=None):
        leaves = visible_leaves(root)
        for w in leaves:
            name = f"{type(w).__name__} '{_text(w)}'"
            if isinstance(w, QLabel) and not w.wordWrap() and w.width() < w.minimumSizeHint().width():
                issues.append(f"{where}: clipped {name}: {w.width()} < {w.minimumSizeHint().width()} px")
            if isinstance(w, QPushButton):
                # The style's minimumSizeHint has a fixed 80 px floor; what matters is the text.
                needed = w.fontMetrics().horizontalAdvance(w.text().replace("&", "")) + 12
                if w.width() < needed:
                    issues.append(f"{where}: clipped {name}: {w.width()} < {needed} px")
            elif isinstance(w, QAbstractButton) and w.width() < w.minimumSizeHint().width():
                issues.append(f"{where}: clipped {name}: {w.width()} < {w.minimumSizeHint().width()} px")
            if isinstance(w, QComboBox) and w.currentText():
                needed = w.fontMetrics().horizontalAdvance(w.currentText()) + 34  # frame + arrow
                if w.width() < needed:
                    issues.append(f"{where}: combobox too narrow for {w.currentText()!r}: {w.width()} < {needed} px")
            if scroll_area is None and not window.rect().contains(global_rect(w)):
                issues.append(f"{where}: {name} sticks out of the window: {global_rect(w)}")
        for i, a in enumerate(leaves):
            for b in leaves[i + 1:]:
                if a.isAncestorOf(b) or b.isAncestorOf(a):
                    continue
                overlap = global_rect(a).intersected(global_rect(b))
                if overlap.width() > 1 and overlap.height() > 1:
                    issues.append(f"{where}: {type(a).__name__} '{_text(a)}' overlaps "
                                  f"{type(b).__name__} '{_text(b)}'")
        if scroll_area is not None:
            if scroll_area.horizontalScrollBar().maximum() > 0:
                issues.append(f"{where}: needs horizontal scrolling "
                              f"({scroll_area.horizontalScrollBar().maximum()} px)")

    def _text(w) -> str:
        for getter in ("text", "currentText", "title"):
            if hasattr(w, getter):
                try:
                    return str(getattr(w, getter)())[:40]
                except TypeError:
                    pass
        return w.objectName()

    executable = window.settings["carla_executable"]
    for carla in CARLA_EXECUTABLES:
        executable.set(carla)
        for width, height in WINDOW_SIZES:
            window.resize(width, height)
            for index in range(window.tabs.count()):
                window.tabs.setCurrentIndex(index)
                for _ in range(3):
                    app.processEvents()
                tab = window.tabs.widget(index)
                title = window.tabs.tabText(index)
                where = f"[{carla} {width}x{height}] {title}"
                check(where + " / CARLA panel", window.centralWidget().layout().itemAt(0).widget())
                check(where, tab.widget(), scroll_area=tab)
                slug = "".join(c if c.isalnum() else "_" for c in f"{carla}_{width}x{height}_{title}")
                window.grab().save(str(out / f"{slug}.png"))

    window.close()
    # Same teardown as gui.run(): Qt objects must go before the QApplication.
    window.deleteLater()
    app.processEvents()
    gc.collect()
    for issue in dict.fromkeys(issues):
        print(issue)
    print(f"{len(set(issues))} issue(s); screenshots in {out}")
    return 1 if issues else 0


if __name__ == "__main__":
    sys.exit(main())
