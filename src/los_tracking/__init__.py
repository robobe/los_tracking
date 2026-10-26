"""Pure bbox/altitude/attitude to physical-demand guidance; no vision or device IO."""
from .config import CameraCalibration, NavigatorConfig, Tuning, VehicleConfig
from .navigator import LosNavigator
from .types import (AttitudeSample, BarometerSample, Diagnostics, NavigationInput,
                    NavigationResult, ControlDemand, State, TargetObservation, TrackStage)

__all__ = ["LosNavigator", "NavigatorConfig", "CameraCalibration", "VehicleConfig", "Tuning",
           "TargetObservation", "BarometerSample", "AttitudeSample", "NavigationInput",
           "NavigationResult", "ControlDemand", "Diagnostics", "State", "TrackStage"]

from .hold import AltitudeHeadingHold, ControlHandoff
__all__ += ["AltitudeHeadingHold", "ControlHandoff"]
