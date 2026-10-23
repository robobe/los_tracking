from math import sin, sqrt
from random import Random

import pytest
from los_tracking import BarometerSample, Tuning
from los_tracking.estimation import TtcEstimator, VerticalEstimator, descent_feasible


@pytest.mark.parametrize("velocity", [-3, 0, 2])
def test_altitude_history_estimates_vertical_speed_with_irregular_times(velocity):
    estimator = VerticalEstimator(Tuning())
    time = 0.0
    for i in range(300):
        time += (0.01, 0.017, 0.023)[i % 3]
        estimator.push(BarometerSample("baro", i, time, 10 + velocity * time))
    assert estimator.ready
    assert estimator.velocity == pytest.approx(velocity, abs=0.01)
    assert estimator.at(time + 0.01)[0] == pytest.approx(10 + velocity * (time + 0.01), abs=0.01)


def test_noisy_barometer_vertical_velocity_is_bounded():
    estimator = VerticalEstimator(Tuning())
    rng = Random(72)
    errors = []
    for i in range(400):
        now = i * 0.02
        estimator.push(BarometerSample("baro", i, now, 10 - now + rng.gauss(0, 0.05)))
        if i > 100:
            errors.append(estimator.velocity + 1)
    assert sqrt(sum(e * e for e in errors) / len(errors)) < 0.5
    assert not VerticalEstimator(Tuning()).ready


def test_optical_ttc_predicts_with_age_then_expires():
    estimator = TtcEstimator(Tuning())
    for i in range(30):
        now = i / 30
        size = 40 / (5 - now)
        estimator.push(now, (100, 100, size, size))
    tau, reason = estimator.at(now)
    assert reason == "qualified"
    assert tau == pytest.approx(5 - now, abs=1e-9)
    assert estimator.at(now + 0.05)[0] == pytest.approx(tau - 0.05, abs=1e-9)
    assert estimator.at(now + 0.101) == (None, "stale")
    assert estimator.relative_residual < 1e-12


@pytest.mark.parametrize("case", ["constant", "receding", "tiny_growth", "noisy"])
def test_unqualified_scale_history_does_not_report_ttc(case):
    estimator = TtcEstimator(Tuning())
    for i in range(30):
        now = i / 30
        size = {"constant": 10, "receding": 20 - now, "tiny_growth": 10 + 0.01 * now,
                "noisy": 15 + 10 * sin(i * 2) + 3 * now}[case]
        estimator.push(now, (100, 100, size, size))
    assert estimator.at(now)[0] is None
    assert estimator.reason != "qualified"


def test_ttc_clipping_invalidates_history_and_bounds_memory():
    estimator = TtcEstimator(Tuning())
    for i in range(2000):
        estimator.push(i / 1000, (100, 100, 20 + i / 100, 20 + i / 100))
    assert len(estimator.samples) <= 256
    estimator.invalidate("clipped")
    assert estimator.at(2) == (None, "clipped")
    assert not estimator.samples


def test_descent_feasibility_includes_acceleration_and_target_height():
    tuning = Tuning()
    assert descent_feasible(5, 0, 10, tuning)
    assert not descent_feasible(5, 0, 0.5, tuning)
    assert not descent_feasible(1, 3, 1, tuning)
    assert descent_feasible(0, 0, 1, tuning)
