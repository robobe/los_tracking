"""Sensor-only inputs; hidden plant truth is used solely by fixtures and scoring."""
from math import hypot, sin

import pytest
from bullet_drone.commands import RcInput
from bullet_drone.rc import RcCodec
from los_tracking import (AttitudeSample, BarometerSample, LosNavigator, NavigationInput,
                          State, TargetObservation)
from los_tracking.geometry import rotate

from conftest import from_rpy


DT_S = 0.01
TARGET = (40.0, 0.0, 0.0)


def target_box(position, attitude, camera, target=TARGET):
    vector = tuple(t - p for t, p in zip(target, position))
    inverse = tuple(-v for v in attitude[:3]) + (attitude[3],)
    x, y, z = rotate(inverse, vector)
    if x <= 0:
        return None
    u = camera.cx_px - camera.fx_px * y / x
    v = camera.cy_px - camera.fy_px * z / x
    size = camera.fx_px * 0.5 / x  # Physical size exists only inside the camera fixture.
    if u - size / 2 <= 0 or v - size / 2 <= 0 or u + size / 2 >= camera.width_px or v + size / 2 >= camera.height_px:
        return None
    return (u - size / 2, v - size / 2, size, size)


@pytest.mark.parametrize("crosswind", [0.0, 0.15, -0.15])
def test_sensor_only_closed_loop_reduces_range_and_preserves_los(config, crosswind):
    nav = LosNavigator(config)
    codec = RcCodec(config.vehicle.max_thrust_n)
    position = [0.0, 0.0, 10.0]
    velocity = [0.0, 0.0, 0.0]
    roll = pitch = yaw = 0.0
    ranges, bearing_errors, states = [], [], set()
    for i in range(1500):
        now = i * DT_S
        q = from_rpy(roll, pitch, yaw)
        box = target_box(position, q, config.camera)
        # Camera runs at 25 Hz, barometer at 50 Hz, attitude/control at 100 Hz.
        inputs = NavigationInput(TargetObservation("camera", i, now, box) if i % 4 == 0 else None,
                                 BarometerSample("baro", i, now, position[2]) if i % 2 == 0 else None,
                                 AttitudeSample("attitude", i, now, q))
        result = nav.update(inputs, now_s=now, enabled=True)
        states.add(result.state)
        assert result.state != State.FAULT
        if result.usable:
            # Only the TEST adapter adds arm/mode and the unused auxiliary channels.
            frame = RcInput(result.command.as_tuple() + (2000, 2000) + (1000,) * 12)
            decoded = codec.decode(frame, "test", i + 1)
            roll += (decoded.rpy[0] - roll) * DT_S / 0.12
            pitch += (decoded.rpy[1] - pitch) * DT_S / 0.12
            yaw += decoded.yaw_rate_rad_s * DT_S
            thrust = decoded.thrust_n
        else:
            thrust = 9.81  # Caller-owned stabilization during library warm-up.
        ax, ay, az = rotate(from_rpy(roll, pitch, yaw), (0, 0, thrust))
        acceleration = (ax - 0.2 * velocity[0], ay + crosswind - 0.2 * velocity[1], az - 9.81)
        for axis in range(3):
            velocity[axis] += acceleration[axis] * DT_S
            position[axis] += velocity[axis] * DT_S
        distance = sum((t - p) ** 2 for t, p in zip(TARGET, position)) ** 0.5
        ranges.append(distance)
        if distance < 0.25:
            # Test oracle emulates a future app-owned collision event. The
            # navigator itself never sees distance, velocity or collision truth.
            result = nav.update(NavigationInput(), now_s=now + DT_S / 2, enabled=False)
            break
        if result.state == State.TRACK:
            d = result.diagnostics
            bearing_errors.append(hypot(d.elevation_rad - d.reference_elevation_rad,
                                        d.azimuth_rad - d.reference_azimuth_rad))
    assert State.TRACK in states
    assert ranges[-1] < 0.25
    assert result.state == State.DISABLED and not result.usable
    assert hypot(*velocity[:2]) > 1  # Contact approach deliberately remains moving.
    assert max(bearing_errors) < config.tuning.max_los_deviation_rad
    assert position[2] < 8  # Persistent descending correction, not pixel centering.
    assert abs(position[1]) < 2


def test_pybullet_angle_rc_adapter_and_noisy_barometer_inputs(config):
    from bullet_drone.config import DroneSettings, ForcesSettings, SimulationConfig, SimulationSettings
    from bullet_drone.controllers import InternalControlSettings, PidSettings
    from bullet_drone.simulation import RcSimulation
    from bullet_drone.vehicle import load_racer_parameters

    vehicle = load_racer_parameters()
    assert config.vehicle.max_thrust_n == pytest.approx(4 * vehicle.max_thrust_per_rotor_n)
    sim_config = SimulationConfig(SimulationSettings(1 / 240, 1200, (0, 0, -9.81), gui=False),
                                  DroneSettings(1, (0, 0, 10)), forces=ForcesSettings.ideal())
    settings = InternalControlSettings((4, 4, 4), (2, 2, 2), (PidSettings(0.05, 0, 0, 0.1, 0),) * 3)
    sim = RcSimulation(sim_config, settings, vehicle)
    nav = LosNavigator(config)
    codec = RcCodec(config.vehicle.max_thrust_n)
    tracked = 0
    try:
        observation = sim.reset()
        for i in range(960):
            truth = observation.ground_truth
            now = truth.simulation_time_s
            q = truth.orientation_xyzw  # Ideal attitude observation fixture, not guidance truth access.
            baro = observation.sensors.barometer
            inputs = NavigationInput(
                TargetObservation("camera", i, now, target_box(truth.position_m, q, config.camera)) if i % 8 == 0 else None,
                BarometerSample("baro", i, now, baro.altitude_m + 0.01 * sin(i)) if i % 4 == 0 else None,
                AttitudeSample("attitude", i, now, q))
            result = nav.update(inputs, now_s=now, enabled=True)
            assert result.state != State.FAULT
            tracked += result.state == State.TRACK
            frame = (RcInput(result.command.as_tuple() + (2000, 2000) + (1000,) * 12)
                     if result.usable else codec.encode("angle", (0, 0, 0), 9.81, True))
            observation = sim.step(frame)
        assert tracked > 700
        assert observation.ground_truth.position_m[0] > 2
        assert observation.ground_truth.position_m[2] < 10
    finally:
        sim.close()
