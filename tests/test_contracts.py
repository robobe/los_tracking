from dataclasses import FrozenInstanceError, fields, replace
from math import pi
from pathlib import Path

import pytest
import yaml
from los_tracking import (AttitudeSample, BarometerSample, CameraCalibration, LosNavigator,
                          NavigationInput, NavigatorConfig, ControlDemand, TargetObservation,
                          Tuning, VehicleConfig)


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
    assert profile == replace(config, tuning=replace(config.tuning, max_attitude_age_s=0.2,
                                                      max_alignment_gap_s=0.12))


@pytest.mark.parametrize('profile', [
    Path(__file__).parents[1] / 'examples/bullet_profile.yaml',
    Path(__file__).parents[2] / 'config/bullet_profile.yaml',
    Path(__file__).parents[2] / 'config/betaflight_los.yaml',
    Path(__file__).parents[2] / 'bullet_app/configs/betaflight_hover_profile.yaml',
])
def test_shipped_profiles_are_complete_yaml(profile):
    data = yaml.safe_load(profile.read_text())
    assert set(data['tuning']) == {item.name for item in fields(Tuning)}
    NavigatorConfig.from_yaml(profile)


def test_oversized_integer_is_reported_as_value_error():
    with pytest.raises(ValueError, match="finite"):
        BarometerSample("baro", 0, 0, 10 ** 10000)
