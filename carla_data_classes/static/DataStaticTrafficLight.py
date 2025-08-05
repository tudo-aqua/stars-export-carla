from __future__ import annotations

from dataclasses import dataclass
from typing import List

from carla_data_classes.static.DataLocation import DataLocation
from carla_data_classes.static.DataRotation import DataRotation


@dataclass
class DataStaticTrafficLight:
    """
    DataClass mapper to serialize carla.TrafficLight objects
    These are only necessary for the static map information and do not
    contain the state of the TrafficLight! For the dynamic information
    head to DataTrafficLight(DataActor). Traffic Lights are represented
    as Actors in carla and therefore the dynamic information is
    stored in the DataTrafficLight which inherits DataActor
    """
    open_drive_id: int
    position_distance: float
    location: "DataLocation"
    rotation: "DataRotation"
    stop_locations: List["DataLocation"]
