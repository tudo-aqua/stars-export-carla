from __future__ import annotations

import dataclasses
from typing import Any, Iterable

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QCheckBox, QComboBox, QDoubleSpinBox, QLineEdit, QSpinBox

from carla_interaction_gui.config_data import Config


class Value(QObject):
    """
    An observable setting. Several widgets (possibly on different tabs) can be
    bound to the same Value and stay in sync. Only touched from the GUI thread.
    """

    changed = Signal(object)

    def __init__(self, value: Any, parent: QObject | None = None):
        super().__init__(parent)
        self._value = value

    def get(self) -> Any:
        return self._value

    def set(self, value: Any) -> None:
        # Comparing first also stops widget <-> value update loops.
        if value == self._value:
            return
        self._value = value
        self.changed.emit(value)


class Settings(QObject):
    """One Value per Config field, persisted together via to_config()."""

    changed = Signal()

    def __init__(self, config: Config, parent: QObject | None = None):
        # Parented so Qt deletes everything together with the window, instead
        # of Python's final garbage collection after Qt has shut down.
        super().__init__(parent)
        self._values = {f.name: Value(getattr(config, f.name), self) for f in dataclasses.fields(Config)}
        for value in self._values.values():
            value.changed.connect(lambda _: self.changed.emit())

    def __getitem__(self, name: str) -> Value:
        return self._values[name]

    def to_config(self) -> Config:
        return Config(**{name: value.get() for name, value in self._values.items()})


def bind_line_edit(edit: QLineEdit, value: Value) -> QLineEdit:
    edit.setText(str(value.get()))
    edit.textChanged.connect(value.set)
    value.changed.connect(lambda v: edit.setText(str(v)) if edit.text() != str(v) else None)
    return edit


def bind_check(check: QCheckBox, value: Value) -> QCheckBox:
    check.setChecked(bool(value.get()))
    check.toggled.connect(value.set)
    value.changed.connect(lambda v: check.setChecked(bool(v)))
    return check


def bind_spin(spin: QSpinBox | QDoubleSpinBox, value: Value) -> QSpinBox | QDoubleSpinBox:
    spin.setValue(value.get())
    spin.valueChanged.connect(value.set)
    value.changed.connect(spin.setValue)
    return spin


def bind_combo(combo: QComboBox, value: Value, items: Iterable[str] = ()) -> QComboBox:
    set_combo_items(combo, items, value)
    combo.currentTextChanged.connect(value.set)
    value.changed.connect(lambda v: combo.setCurrentText(v) if combo.currentText() != v else None)
    return combo


def set_combo_items(combo: QComboBox, items: Iterable[str], value: Value) -> None:
    """Replace a bound combobox's choices, keeping the bound value selected."""
    combo.blockSignals(True)
    combo.clear()
    combo.addItems(list(items))
    combo.setCurrentText(value.get())
    combo.blockSignals(False)
