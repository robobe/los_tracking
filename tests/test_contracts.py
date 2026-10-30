from dataclasses import FrozenInstanceError, fields, replace
from math import pi, radians
from pathlib import Path

import pytest
import yaml
from los_tracking import (AttitudeSample, BarometerSample, CameraCalibration, ControlConnection,
                          ControlConfig, LosNavigator, NavigationInput, NavigatorConfig,
                          ControlDemand, TargetObservation, Tuning, VehicleConfig)
from los_tracking.config import YAML_DEGREE_FIELDS


@pytest.mark.parametrize("make", [
    lambda: BarometerSample("baro", 0, 0, float("nan")),
    lambda: BarometerSample("", 0, 0, 10),
    lambda: BarometerSample("baro", True, 0, 10),
    lambda: AttitudeSample("imu", 0, 0, (0, 0, 0, 0)),
    lambda: AttitudeSample("imu", 0, 0, (0, 0, 1)),
    lambda: TargetObservation("camera", 0, 0, (0, 0, -1, 1)),
    lambda: TargetObservation("camera", 0, 0, (0, 0, 1, float("inf"))),
    lambda: TargetObservation("camera", 0, 0, 1),
    lambda: NavigationInput(attitude=(0, 0, 0, 1)),
    lambda: ControlDemand(0, 0, 0, -1),
    lambda: ControlDemand(0, 0, True, 1),
    lambda: CameraCalibration(640, 480, 0, 320, 320, 240),
    lambda: CameraCalibration.from_horizontal_fov(640, 480, pi),
    lambda: CameraCalibration.from_horizontal_fov("640", 480, pi / 2),
    lambda: VehicleConfig(1, 9.81),
    lambda: Tuning(max_control_gap_s=0),
    lambda: Tuning(ttc_enter_s=20),
    lambda: Tuning(barometer_alpha=2),
    lambda: Tuning(combined_tilt_limit_rad=pi / 2),
])
def test_invalid_contracts_raise_value_error(make):
    with pytest.raises(ValueError):
        make()


def test_config_is_frozen_and_cannot_be_hot_replaced(config):
    with pytest.raises(FrozenInstanceError):
        config.target_altitude_m = 1
    nav = LosNavigator(config)
    with pytest.raises(AttributeError):
        nav.config = replace(config, target_altitude_m=2)


def test_profile_rejects_unknown_fields_and_missing_platform_values(tmp_path):
    p = tmp_path / "profile.yaml"
    for data in ["{}", "unexpected: 1", "[]", "null", "camera: ["]:
        p.write_text(data)
        with pytest.raises(ValueError):
            NavigatorConfig.from_yaml(p)


def test_simulation_profile_matches_actual_rc_platform(config):
    profile = NavigatorConfig.from_yaml(Path(__file__).parents[1] / "examples/bullet_profile.yaml")
    expected = replace(config, tuning=replace(config.tuning, max_attitude_age_s=0.2,
                                               max_alignment_gap_s=0.12))
    assert profile.camera == expected.camera and profile.vehicle == expected.vehicle
    for item in fields(Tuning):
        assert getattr(profile.tuning, item.name) == pytest.approx(getattr(expected.tuning, item.name))


@pytest.mark.parametrize('profile', [
    Path(__file__).parents[1] / 'examples/bullet_profile.yaml',
    Path(__file__).parents[2] / 'config/bullet_profile.yaml',
    Path(__file__).parents[2] / 'config/betaflight_los.yaml',
    Path(__file__).parents[2] / 'bullet_app/configs/betaflight_hover_profile.yaml',
])
def test_shipped_profiles_are_complete_yaml(profile):
    data = yaml.safe_load(profile.read_text())
    expected = {item.name for item in fields(Tuning)} - {runtime for runtime, _ in YAML_DEGREE_FIELDS.values()}
    assert set(data['tuning']) == expected | set(YAML_DEGREE_FIELDS)
    NavigatorConfig.from_yaml(profile)


def test_yaml_degrees_convert_to_runtime_radians(tmp_path):
    path = tmp_path / 'profile.yaml'
    path.write_text('''
camera:
  width_px: 640
  height_px: 480
  fx_px: 320
  fy_px: 320
  cx_px: 320
  cy_px: 240
vehicle:
  mass_kg: 1
  max_thrust_n: 33.953391
target_altitude_m: 0
tuning:
  roll_limit_deg: 20
  elevation_kp_m_s_deg: 0.7
''')
    tuning = NavigatorConfig.from_yaml(path).tuning
    assert tuning.roll_limit_rad == pytest.approx(radians(20))
    assert tuning.elevation_kp_m_s_rad == pytest.approx(.7 / (pi / 180))


def test_yaml_parses_betaflight_tcp_connection(tmp_path):
    path = tmp_path / 'profile.yaml'
    path.write_text('''
camera: {width_px: 640, height_px: 480, fx_px: 320, fy_px: 320, cx_px: 320, cy_px: 240}
vehicle: {mass_kg: 1, max_thrust_n: 33.953391}
target_altitude_m: 0
control:
  backend: betaflight
  target: sitl
  connection: {transport: tcp, host: 192.168.1.20, port: 5761}
''')

    control = NavigatorConfig.from_yaml(path).control

    assert control == ControlConfig(
        backend='betaflight', target='sitl',
        connection=ControlConnection(transport='tcp', host='192.168.1.20', port=5761))


def test_yaml_parses_betaflight_serial_connection(tmp_path):
    path = tmp_path / 'profile.yaml'
    path.write_text('''
camera: {width_px: 640, height_px: 480, fx_px: 320, fy_px: 320, cx_px: 320, cy_px: 240}
vehicle: {mass_kg: 1, max_thrust_n: 33.953391}
target_altitude_m: 0
control:
  backend: betaflight
  target: hardware
  connection: {transport: serial, path: /dev/ttyUSB0, baud_rate: 921600}
''')

    connection = NavigatorConfig.from_yaml(path).control.connection

    assert connection.transport == 'serial'
    assert connection.path == '/dev/ttyUSB0'
    assert connection.baud_rate == 921600


@pytest.mark.parametrize('connection', [
    {'transport': 'serial'},
    {'transport': 'tcp', 'port': 0},
    {'transport': 'tcp', 'host': ''},
    {'transport': 'usb'},
])
def test_yaml_rejects_invalid_control_connection(tmp_path, connection):
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump({
        'camera': {'width_px': 640, 'height_px': 480, 'fx_px': 320, 'fy_px': 320, 'cx_px': 320, 'cy_px': 240},
        'vehicle': {'mass_kg': 1, 'max_thrust_n': 33.953391},
        'target_altitude_m': 0,
        'control': {'backend': 'betaflight', 'connection': connection},
    }))

    with pytest.raises(ValueError):
        NavigatorConfig.from_yaml(path)


def test_yaml_rejects_legacy_radian_keys(tmp_path):
    path = tmp_path / 'profile.yaml'
    path.write_text('''
camera:
  width_px: 640
  height_px: 480
  fx_px: 320
  fy_px: 320
  cx_px: 320
  cy_px: 240
vehicle:
  mass_kg: 1
  max_thrust_n: 33.953391
target_altitude_m: 0
tuning:
  roll_limit_rad: 0.2
''')
    with pytest.raises(ValueError):
        NavigatorConfig.from_yaml(path)


def test_oversized_integer_is_reported_as_value_error():
    with pytest.raises(ValueError, match="finite"):
        BarometerSample("baro", 0, 0, 10 ** 10000)
