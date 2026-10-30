"""Explicit platform configuration and simulation-only tuning defaults."""
from dataclasses import dataclass, field, fields
from math import pi, tan
from pathlib import Path

import yaml

from .types import finite, quaternion


DEG_TO_RAD = pi / 180
IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)

YAML_DEGREE_FIELDS = {
    "approach_pitch_deg": ("approach_pitch_rad", DEG_TO_RAD),
    "roll_limit_deg": ("roll_limit_rad", DEG_TO_RAD),
    "pitch_limit_deg": ("pitch_limit_rad", DEG_TO_RAD),
    "combined_tilt_limit_deg": ("combined_tilt_limit_rad", DEG_TO_RAD),
    "roll_slew_deg_s": ("roll_slew_rad_s", DEG_TO_RAD),
    "pitch_slew_deg_s": ("pitch_slew_rad_s", DEG_TO_RAD),
    "yaw_rate_limit_deg_s": ("yaw_rate_limit_rad_s", DEG_TO_RAD),
    "track_elevation_deg": ("track_elevation_rad", DEG_TO_RAD),
    "elevation_kp_m_s_deg": ("elevation_kp_m_s_rad", 1 / DEG_TO_RAD),
    "elevation_kd_m_deg": ("elevation_kd_m_rad", 1 / DEG_TO_RAD),
    "elevation_ki_m_s2_deg": ("elevation_ki_m_s2_rad", 1 / DEG_TO_RAD),
    "lateral_memory_limit_deg": ("lateral_memory_limit_rad", DEG_TO_RAD),
    "max_elevation_deg": ("max_elevation_rad", DEG_TO_RAD),
    "max_azimuth_deg": ("max_azimuth_rad", DEG_TO_RAD),
    "max_los_deviation_deg": ("max_los_deviation_rad", DEG_TO_RAD),
    "max_association_jump_deg": ("max_association_jump_rad", DEG_TO_RAD),
}


@dataclass(frozen=True)
class CameraCalibration:
    width_px: int
    height_px: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    camera_to_body_xyzw: tuple[float, float, float, float] = IDENTITY_QUATERNION

    def __post_init__(self):
        if any(type(v) is not int or v <= 0 for v in (self.width_px, self.height_px)):
            raise ValueError("camera dimensions must be positive integers")
        for name in ("fx_px", "fy_px", "cx_px", "cy_px"):
            finite(getattr(self, name), name)
        if min(self.fx_px, self.fy_px) <= 0:
            raise ValueError("camera focal lengths must be positive")
        if not 0 <= self.cx_px <= self.width_px or not 0 <= self.cy_px <= self.height_px:
            raise ValueError("optical centre must lie within the calibrated image")
        object.__setattr__(self, "camera_to_body_xyzw", quaternion(self.camera_to_body_xyzw))

    @classmethod
    def from_horizontal_fov(cls, width_px: int, height_px: int, fov_rad: float):
        if not 0 < finite(fov_rad, "fov_rad") < pi:
            raise ValueError("horizontal FOV must lie in (0, pi)")
        if any(type(v) is not int or v <= 0 for v in (width_px, height_px)):
            raise ValueError("camera dimensions must be positive integers")
        focal = width_px / (2 * tan(fov_rad / 2))
        return cls(width_px, height_px, focal, focal, width_px / 2, height_px / 2)


@dataclass(frozen=True)
class VehicleConfig:
    mass_kg: float
    max_thrust_n: float
    gravity_m_s2: float = 9.81

    def __post_init__(self):
        for item in fields(self):
            if finite(getattr(self, item.name), item.name) <= 0:
                raise ValueError(f"{item.name} must be positive")
        if self.max_thrust_n <= finite(self.mass_kg * self.gravity_m_s2, "hover_thrust_n"):
            raise ValueError("maximum thrust must exceed level hover thrust")


@dataclass(frozen=True)
class ControlConnection:
    """Betaflight MSP connection details, independent of MAVLink telemetry."""

    transport: str = "tcp"
    host: str = "127.0.0.1"
    port: int = 5761
    path: str | None = None
    baud_rate: int = 115200

    def __post_init__(self):
        if self.transport not in ("tcp", "serial"):
            raise ValueError("control.connection.transport must be 'tcp' or 'serial'")
        if not isinstance(self.host, str) or not self.host.strip():
            raise ValueError("control.connection.host must be a nonblank string")
        if type(self.port) is not int or not 1 <= self.port <= 65535:
            raise ValueError("control.connection.port must be an integer in [1, 65535]")
        if self.path is not None and (not isinstance(self.path, str) or not self.path.strip()):
            raise ValueError("control.connection.path must be a nonblank string when provided")
        if type(self.baud_rate) is not int or self.baud_rate <= 0:
            raise ValueError("control.connection.baud_rate must be a positive integer")
        if self.transport == "serial" and self.path is None:
            raise ValueError("control.connection.path is required for serial transport")


@dataclass(frozen=True)
class ControlConfig:
    """Select the flight backend and its Betaflight MSP deployment target."""

    backend: str = "internal"
    target: str = "sitl"
    connection: ControlConnection = field(default_factory=ControlConnection)

    def __post_init__(self):
        if self.backend not in ("internal", "betaflight"):
            raise ValueError("control.backend must be 'internal' or 'betaflight'")
        if self.target not in ("sitl", "hardware"):
            raise ValueError("control.target must be 'sitl' or 'hardware'")
        if not isinstance(self.connection, ControlConnection):
            raise ValueError("control.connection must be ControlConnection")


@dataclass(frozen=True)
class Tuning:
    # All defaults are simulation starting points, not validated hardware gains.
    approach_pitch_rad: float = 7 * DEG_TO_RAD
    roll_limit_rad: float = 15 * DEG_TO_RAD
    pitch_limit_rad: float = 30 * DEG_TO_RAD
    combined_tilt_limit_rad: float = 30 * DEG_TO_RAD
    roll_slew_rad_s: float = 15 * DEG_TO_RAD
    pitch_slew_rad_s: float = 5 * DEG_TO_RAD
    yaw_gain_s_inv: float = 4.0
    yaw_rate_limit_rad_s: float = 2.0
    yaw_search_timeout_s: float = 60.0
    track_elevation_rad: float = 0.0
    max_descent_m_s: float = 5.5
    max_climb_m_s: float = 3.0
    vertical_slew_m_s2: float = 1.0
    vertical_brake_slew_m_s2: float = 4.0
    takeoff_kp_s_inv: float = 1.0
    takeoff_ki_s2_inv: float = 0.0
    takeoff_max_climb_m_s: float = 1.0
    takeoff_vertical_slew_m_s2: float = 1.0
    hold_altitude_gain_s_inv: float = 1.0
    # Upstream vertical LOS gains used degrees; convert them to radians here.
    elevation_kp_m_s_rad: float = 0.7 / DEG_TO_RAD
    elevation_kd_m_rad: float = 0.25 / DEG_TO_RAD
    elevation_ki_m_s2_rad: float = 0.25 / DEG_TO_RAD
    vertical_memory_limit_m_s: float = 3.0
    azimuth_kp: float = 0.8
    azimuth_kd_s: float = 4.0
    azimuth_ki_s_inv: float = 0.05
    lateral_memory_limit_rad: float = 5 * DEG_TO_RAD
    vertical_kp_n_s_m: float = 1.0
    vertical_ki_n_m: float = 0.4
    vertical_integral_limit_n: float = 5.0
    betaflight_takeoff_pwm: int = 0
    betaflight_hover_pwm: int = 0
    barometer_alpha: float = 0.35
    barometer_beta: float = 0.05
    vertical_warmup_s: float = 0.1
    bearing_filter_s: float = 0.15
    max_image_age_s: float = 0.1
    max_attitude_age_s: float = 0.05
    max_barometer_age_s: float = 0.2
    attitude_history_s: float = 0.5
    max_alignment_gap_s: float = 0.05
    max_control_gap_s: float = 0.1
    command_validity_s: float = 0.05
    acquisition_dwell_s: float = 0.1
    acquisition_timeout_s: float = 5.0
    recovery_dwell_s: float = 0.1
    recovery_timeout_s: float = 1.0
    limitation_timeout_s: float = 0.75
    max_elevation_rad: float = 60 * DEG_TO_RAD
    max_azimuth_rad: float = 35 * DEG_TO_RAD
    max_los_deviation_rad: float = 12 * DEG_TO_RAD
    max_association_jump_rad: float = 12 * DEG_TO_RAD
    ttc_window_s: float = 0.5
    ttc_min_span_s: float = 0.2
    ttc_min_growth_px: float = 1.0
    ttc_max_relative_residual: float = 0.15
    ttc_enter_s: float = 8.0
    ttc_exit_s: float = 10.0
    ttc_dwell_s: float = 0.3
    terminal_ttc_s: float = 0.8
    ttc_time_margin_s: float = 0.1
    feasibility_height_margin_m: float = 0.25
    commit_coverage_fraction: float = 0.4
    commit_edge_margin_px: float = 0.0

    def __post_init__(self):
        nonnegative = {"elevation_kp_m_s_rad", "elevation_kd_m_rad", "elevation_ki_m_s2_rad",
                       "azimuth_kp", "azimuth_kd_s", "azimuth_ki_s_inv", "vertical_kp_n_s_m",
                       "vertical_ki_n_m", "yaw_gain_s_inv", "ttc_time_margin_s",
                       "feasibility_height_margin_m", "commit_edge_margin_px",
                       "track_elevation_rad",
                       "takeoff_kp_s_inv", "takeoff_ki_s2_inv",
                       "betaflight_takeoff_pwm", "betaflight_hover_pwm"}
        for item in fields(self):
            value = finite(getattr(self, item.name), item.name)
            if value < 0 or (value == 0 and item.name not in nonnegative):
                raise ValueError(f"{item.name} must be {'nonnegative' if item.name in nonnegative else 'positive'}")
        if any(getattr(self, name) >= pi / 2 for name in
               ("roll_limit_rad", "pitch_limit_rad", "combined_tilt_limit_rad", "track_elevation_rad",
                "max_elevation_rad", "max_azimuth_rad")):
            raise ValueError("angular limits must be strictly below pi/2")
        if self.approach_pitch_rad > self.pitch_limit_rad or self.approach_pitch_rad > self.combined_tilt_limit_rad:
            raise ValueError("approach pitch exceeds an angular limit")
        if self.takeoff_max_climb_m_s > self.max_climb_m_s:
            raise ValueError("takeoff climb exceeds maximum climb")
        if not 0 < self.barometer_alpha <= 1 or not 0 < self.barometer_beta <= 1:
            raise ValueError("barometer filter gains must lie in (0,1]")
        if not 0 < self.commit_coverage_fraction <= 1:
            raise ValueError("commit coverage fraction must lie in (0,1]")
        if self.max_image_age_s > self.attitude_history_s:
            raise ValueError("attitude history must cover maximum image age")
        if self.max_alignment_gap_s > self.attitude_history_s:
            raise ValueError("alignment gap exceeds attitude history")
        if self.ttc_min_span_s > self.ttc_window_s:
            raise ValueError("TTC span exceeds its window")
        if not self.terminal_ttc_s < self.ttc_enter_s < self.ttc_exit_s:
            raise ValueError("TTC thresholds must satisfy terminal < enter < exit")
        if self.acquisition_dwell_s >= self.acquisition_timeout_s or self.recovery_dwell_s >= self.recovery_timeout_s:
            raise ValueError("dwell must be shorter than timeout")
        for name in ("betaflight_takeoff_pwm", "betaflight_hover_pwm"):
            value = getattr(self, name)
            if type(value) is not int or value != 0 and not 1050 <= value <= 2000:
                raise ValueError(f"{name} must be 0 or an integer in [1050, 2000]")


@dataclass(frozen=True)
class NavigatorConfig:
    camera: CameraCalibration
    vehicle: VehicleConfig
    target_altitude_m: float
    tuning: Tuning = field(default_factory=Tuning)
    control: ControlConfig = field(default_factory=ControlConfig)

    def __post_init__(self):
        if (not isinstance(self.camera, CameraCalibration) or not isinstance(self.vehicle, VehicleConfig)
                or not isinstance(self.tuning, Tuning) or not isinstance(self.control, ControlConfig)):
            raise ValueError("typed camera, vehicle, tuning and control configuration required")
        finite(self.target_altitude_m, "target_altitude_m")
    @classmethod
    def from_yaml(cls, path: str | Path):
        """Load an explicit profile; unknown fields and malformed data are ValueErrors."""
        try:
            data = yaml.safe_load(Path(path).read_text())
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise ValueError("invalid navigator profile") from exc
        try:
            if not isinstance(data, dict) or set(data) - {"camera", "vehicle", "target_altitude_m", "tuning", "control"}:
                raise ValueError("invalid or unknown navigator profile fields")
            tuning = dict(data.get("tuning", {}))
            legacy = set(tuning) & {runtime for runtime, _ in YAML_DEGREE_FIELDS.values()}
            if legacy:
                raise ValueError("navigator YAML angles must use degree keys")
            for yaml_name, (runtime_name, scale) in YAML_DEGREE_FIELDS.items():
                if yaml_name in tuning:
                    tuning[runtime_name] = finite(tuning.pop(yaml_name), yaml_name) * scale
            control_data = data.get("control", {})
            if not isinstance(control_data, dict) or set(control_data) - {"backend", "target", "connection"}:
                raise ValueError("invalid or unknown control fields")
            connection_data = control_data.get("connection", {})
            if not isinstance(connection_data, dict) or set(connection_data) - {"transport", "host", "port", "path", "baud_rate"}:
                raise ValueError("invalid or unknown control.connection fields")
            control = ControlConfig(
                backend=control_data.get("backend", "internal"),
                target=control_data.get("target", "sitl"),
                connection=ControlConnection(**connection_data),
            )
            return cls(camera=CameraCalibration(**data["camera"]),
                       vehicle=VehicleConfig(**data["vehicle"]),
                       target_altitude_m=data["target_altitude_m"],
                       tuning=Tuning(**tuning), control=control)
        except (TypeError, KeyError) as exc:
            raise ValueError("invalid navigator profile") from exc
