from dataclasses import replace
from math import atan2, cos, degrees, hypot, pi, radians

import pytest
from bullet_drone.commands import RcInput
from bullet_drone.rc import RcCodec
from los_tracking import AttitudeSample, CameraCalibration, Tuning
from los_tracking.control import LosControl, encode_channels, limit_angles, slew_pair
from los_tracking.estimation import BearingEstimator
from los_tracking.geometry import AttitudeHistory, combined_tilt, rotate, slerp, world_bearing, wrap

from conftest import IDENTITY, from_rpy


def project(world_point, attitude, camera):
    inverse = tuple(-v for v in attitude[:3]) + (attitude[3],)
    x, y, z = rotate(inverse, world_point)
    return camera.cx_px - camera.fx_px * y / x, camera.cy_px - camera.fy_px * z / x


def test_documented_pixel_example():
    camera = CameraCalibration(1280, 720, 640, 640, 640, 360)
    elevation, azimuth = world_bearing((800, 430, 20, 20), camera, IDENTITY, 0)
    assert degrees(elevation) == pytest.approx(6.89, abs=0.01)
    assert degrees(azimuth) == pytest.approx(-14.88, abs=0.01)


@pytest.mark.parametrize("axis", ["roll", "pitch", "yaw", "combined"])
def test_pure_rotation_compensates_bearing_and_rate(config, axis):
    estimator = BearingEstimator(0.15)
    point = (30, 3, -4)
    expected = (atan2(4, hypot(30, 3)), atan2(3, 30))
    for i in range(60):
        value = 0.1 * i / 60
        angles = {"roll": (value, 0, 0), "pitch": (0, value, 0), "yaw": (0, 0, value),
                  "combined": (value, -value, value)}[axis]
        q = from_rpy(*angles)
        u, v = project(point, q, config.camera)
        bearing = world_bearing((u - 5, v - 5, 10, 10), config.camera, q, 0)
        assert bearing == pytest.approx(expected, abs=1e-12)
        estimator.push(i / 30, bearing)
    assert estimator.rates == pytest.approx((0, 0), abs=1e-10)


def test_mount_rotation_and_wrapped_heading(config):
    camera = replace(config.camera, camera_to_body_xyzw=from_rpy(pitch=0.2))
    bearing = world_bearing((310, 230, 20, 20), camera, from_rpy(yaw=-pi + 0.01), pi - 0.01)
    assert bearing == pytest.approx((0.2, 0.02), abs=1e-12)
    assert slerp(IDENTITY, (0, 0, 0, -1), 0.5) == IDENTITY


def test_attitude_history_interpolates_without_extrapolating_or_large_gaps():
    history = AttitudeHistory(0.5, 0.05)
    history.push(AttitudeSample("imu", 0, 0, IDENTITY))
    history.push(AttitudeSample("imu", 1, 0.04, from_rpy(yaw=0.2)))
    assert history.at(0.02) == pytest.approx(from_rpy(yaw=0.1))
    assert history.at(-0.01) is None and history.at(0.05) is None
    history.push(AttitudeSample("imu", 2, 0.2, IDENTITY))
    assert history.at(0.1) is None
    for i in range(3, 1000):
        history.push(AttitudeSample("imu", i, i / 100, IDENTITY))
    assert len(history.samples) <= 52
    assert history.at(0.1) is None


def test_bearing_wrap_does_not_create_a_full_turn_rate():
    estimator = BearingEstimator(0.05)
    for i in range(100):
        estimator.push(i / 100, (0.1, wrap(pi - 0.01 + i * 0.0003)))
    assert estimator.rates[1] == pytest.approx(0.03, abs=0.005)


@pytest.mark.parametrize("roll_sign", [-1, 1])
@pytest.mark.parametrize("pitch_sign", [-1, 1])
def test_exact_combined_tilt_example_and_coordinated_slew(roll_sign, pitch_sign):
    t = replace(Tuning(), roll_limit_rad=radians(30))
    requested = (roll_sign * radians(15), pitch_sign * radians(30))
    limited = limit_angles(*requested, t)
    assert degrees(abs(limited[1])) == pytest.approx(26.288483, abs=1e-6)
    previous = limit_angles(-roll_sign * radians(25), 0, t)
    for _ in range(500):
        current = slew_pair(previous, limited, 0.01, t)
        assert combined_tilt(*current) <= t.combined_tilt_limit_rad + 1e-12
        assert abs(current[0] - previous[0]) <= t.roll_slew_rad_s * 0.01 + 1e-12
        assert abs(current[1] - previous[1]) <= t.pitch_slew_rad_s * 0.01 + 1e-12
        previous = current


def test_quantized_channels_respect_individual_and_total_bounds(config):
    codec = RcCodec(config.vehicle.max_thrust_n)
    for roll in range(-50, 51, 5):
        for pitch in range(-50, 51, 5):
            limited = limit_angles(radians(roll), radians(pitch), config.tuning)
            command, _ = encode_channels(*limited, 9.81, 0.5, config)
            frame = RcInput(command.as_tuple() + (2000, 2000) + (1000,) * 12)
            decoded = codec.decode(frame, "episode", 1)
            assert decoded.mode == "angle" and decoded.armed
            assert abs(decoded.rpy[0]) <= config.tuning.roll_limit_rad + 1e-12
            assert abs(decoded.rpy[1]) <= config.tuning.pitch_limit_rad + 1e-12
            assert combined_tilt(*decoded.rpy[:2]) <= config.tuning.combined_tilt_limit_rad + 1e-12
            assert decoded.thrust_n == pytest.approx(9.81, abs=config.vehicle.max_thrust_n / 1000)
            assert decoded.yaw_rate_rad_s == pytest.approx(0.5)


def test_los_corrections_memory_and_windup(config):
    control = LosControl(config)
    for _ in range(400):
        control.step(0.01, 10, 0, IDENTITY, 0, errors=(0.01, 0.02))
    assert control.vz_reference < 0
    assert control.angles[0] < 0
    learned = (control.vertical_memory, control.lateral_memory)
    assert learned[0] < 0 and learned[1] < 0
    control.step(0.01, 10, 0, IDENTITY, 0, errors=(0, 0))
    assert (control.vertical_memory, control.lateral_memory) == learned
    for _ in range(2000):
        control.step(0.01, 10, 0, IDENTITY, 0, errors=(1, 1))
    assert abs(control.vertical_memory) <= config.tuning.vertical_memory_limit_m_s
    assert abs(control.lateral_memory) <= config.tuning.lateral_memory_limit_rad
    assert control.vz_reference >= -config.tuning.max_descent_m_s
    assert abs(control.thrust_integral) <= config.tuning.vertical_integral_limit_n


def test_heading_rate_and_measured_tilt_compensation(config):
    control = LosControl(config)
    command = control.step(0.01, 10, 0, from_rpy(pitch=0.2, yaw=0.1), 0, hold_altitude=10)
    assert command.yaw < 1500
    assert control.last_diagnostics["thrust_n"] == pytest.approx(9.81 / cos(0.2))
    assert abs(control.last_diagnostics["output_pitch_rad"]) < 1e-12


def test_recovery_reseeds_for_continuous_handoff(config):
    control = LosControl(config)
    for _ in range(100):
        control.step(0.01, 10, 0, IDENTITY, 0, errors=(0.01, 0.01))
    control.recover()
    control.step(0.01, 10, 0, IDENTITY, 0, hold_altitude=10)
    previous = control.angles, control.vz_reference
    control.begin_track((0.02, -0.03), (0.01, 0.01))
    control.step(0.01, 10, 0, IDENTITY, 0, errors=(0.02, -0.03), rates=(0.01, 0.01))
    assert abs(control.vz_reference - previous[1]) <= config.tuning.vertical_slew_m_s2 * 0.01 + 1e-12
    assert all(abs(a - b) <= rate * 0.01 + 1e-12 for a, b, rate in
               zip(control.angles, previous[0], (config.tuning.roll_slew_rad_s, config.tuning.pitch_slew_rad_s)))


def test_known_translation_los_rates_survive_simultaneous_camera_rotation(config):
    estimator = BearingEstimator(0.03)
    for i in range(200):
        now = i / 100
        ray = (40 - 2 * now, -0.5 * now, -10)
        q = from_rpy(0.03 * now, -0.02 * now, 0.04 * now)
        u, v = project(ray, q, config.camera)
        estimator.push(now, world_bearing((u - 5, v - 5, 10, 10), config.camera, q, 0))
    x, y, _ = ray
    horizontal = hypot(x, y)
    horizontal_rate = (-2 * x - 0.5 * y) / horizontal
    expected_elevation_rate = -10 * horizontal_rate / (horizontal ** 2 + 100)
    expected_azimuth_rate = (-0.5 * x + 2 * y) / horizontal ** 2
    assert estimator.rates == pytest.approx((expected_elevation_rate, expected_azimuth_rate), abs=0.0005)
