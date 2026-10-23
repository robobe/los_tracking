"""Pure bbox/altitude/attitude to Angle-mode RC guidance; no vision or device IO."""
from .config import CameraCalibration, NavigatorConfig, Tuning, VehicleConfig
from .navigator import LosNavigator
from .types import (AttitudeSample, BarometerSample, Diagnostics, NavigationInput,
                    NavigationResult, RcChannels, State, TargetObservation, TrackStage)

__all__ = ["LosNavigator", "NavigatorConfig", "CameraCalibration", "VehicleConfig", "Tuning",
           "TargetObservation", "BarometerSample", "AttitudeSample", "NavigationInput",
           "NavigationResult", "RcChannels", "Diagnostics", "State", "TrackStage"]

from .hold import AltitudeHeadingHold, ControlHandoff
__all__ += ["AltitudeHeadingHold", "ControlHandoff"]
