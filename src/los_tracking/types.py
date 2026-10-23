"""Immutable sensor and command contracts; metres, seconds and radians throughout."""
from dataclasses import dataclass
from enum import StrEnum
from math import isfinite, sqrt


def finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def quaternion(value) -> tuple[float, float, float, float]:
    try:
        q = tuple(finite(v, "quaternion component") for v in value)
    except TypeError as exc:
        raise ValueError("quaternion must contain four components") from exc
    if len(q) != 4:
        raise ValueError("quaternion must contain four components")
    norm = sqrt(sum(v * v for v in q))
    if abs(norm - 1) > 1e-3:
        raise ValueError("quaternion must be unit length (XYZW)")
    return tuple(v / norm for v in q)


def sample_identity(sample) -> None:
    if not isinstance(sample.stream_id, str) or not sample.stream_id:
        raise ValueError("stream_id must be a nonempty string")
    if type(sample.sequence) is not int or sample.sequence < 0:
        raise ValueError("sequence must be a nonnegative integer")
    finite(sample.timestamp_s, "timestamp_s")


@dataclass(frozen=True)
class TargetObservation:
    stream_id: str
    sequence: int
    timestamp_s: float
    bbox: tuple[float, float, float, float] | None

    def __post_init__(self):
        sample_identity(self)
        if self.bbox is not None:
            try:
                box = tuple(finite(v, "bbox component") for v in self.bbox)
            except TypeError as exc:
                raise ValueError("bbox must contain x, y, width, height") from exc
            if len(box) != 4 or min(box[:2]) < 0 or min(box[2:]) <= 0:
                raise ValueError("bbox requires nonnegative x/y and positive width/height")
            object.__setattr__(self, "bbox", box)


@dataclass(frozen=True)
class BarometerSample:
    stream_id: str
    sequence: int
    timestamp_s: float
    altitude_m: float

    def __post_init__(self):
        sample_identity(self)
        finite(self.altitude_m, "altitude_m")


@dataclass(frozen=True)
class AttitudeSample:
    stream_id: str
    sequence: int
    timestamp_s: float
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self):
        sample_identity(self)
        object.__setattr__(self, "orientation_xyzw", quaternion(self.orientation_xyzw))


@dataclass(frozen=True)
class NavigationInput:
    target: TargetObservation | None = None
    barometer: BarometerSample | None = None
    attitude: AttitudeSample | None = None

    def __post_init__(self):
        for name, kind in (("target", TargetObservation), ("barometer", BarometerSample),
                           ("attitude", AttitudeSample)):
            if getattr(self, name) is not None and not isinstance(getattr(self, name), kind):
                raise ValueError(f"{name} must be {kind.__name__} or None")


class State(StrEnum):
    DISABLED = "disabled"
    ACQUIRE = "acquire"
    TRACK = "track"
    RECOVER = "recover"
    FAULT = "fault"


class TrackStage(StrEnum):
    BEARING = "bearing"
    TTC = "ttc"
    TERMINAL = "terminal"


@dataclass(frozen=True)
class RcChannels:
    roll: int
    pitch: int
    throttle: int
    yaw: int

    def __post_init__(self):
        if any(type(v) is not int or not 1000 <= v <= 2000 for v in self.as_tuple()):
            raise ValueError("RC channels must be integers in 1000..2000")

    def as_tuple(self) -> tuple[int, int, int, int]:
        return self.roll, self.pitch, self.throttle, self.yaw


@dataclass(frozen=True)
class Diagnostics:
    elevation_rad: float | None = None
    azimuth_rad: float | None = None
    elevation_rate_rad_s: float | None = None
    azimuth_rate_rad_s: float | None = None
    bearing_valid: bool = False
    rate_observable: bool = False
    reference_elevation_rad: float | None = None
    reference_azimuth_rad: float | None = None
    altitude_m: float | None = None
    vertical_speed_m_s: float | None = None
    vertical_ready: bool = False
    optical_ttc_s: float | None = None
    ttc_reason: str = "not_observed"
    ttc_relative_residual: float | None = None
    descent_feasible: bool | None = None
    image_age_s: float | None = None
    barometer_age_s: float | None = None
    attitude_age_s: float | None = None
    vision_reason: str = "not_observed"
    input_rejection: str = ""
    requested_vertical_speed_m_s: float = 0.0
    vertical_reference_m_s: float = 0.0
    requested_roll_rad: float = 0.0
    reference_roll_rad: float = 0.0
    reference_pitch_rad: float = 0.0
    output_roll_rad: float = 0.0
    output_pitch_rad: float = 0.0
    requested_tilt_rad: float = 0.0
    output_tilt_rad: float = 0.0
    measured_tilt_rad: float = 0.0
    measured_over_tilt: bool = False
    vertical_limited: bool = False
    angle_limited: bool = False
    slew_limited: bool = False
    thrust_n: float = 0.0
    thrust_saturated: bool = False
    vertical_memory_m_s: float = 0.0
    lateral_memory_rad: float = 0.0
    rejected_elevation_rad: float | None = None
    rejected_azimuth_rad: float | None = None
    max_elevation_rad: float | None = None
    max_azimuth_rad: float | None = None


@dataclass(frozen=True)
class NavigationResult:
    sequence: int
    generated_at_s: float
    valid_until_s: float
    state: State
    stage: TrackStage
    reason: str
    command: RcChannels | None
    diagnostics: Diagnostics

    @property
    def usable(self) -> bool:
        return self.command is not None
