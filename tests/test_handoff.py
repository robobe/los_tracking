from dataclasses import replace
from pathlib import Path

import pytest
from los_tracking import (AltitudeHeadingHold, AttitudeSample, BarometerSample, LosNavigator,
                          NavigationInput, NavigatorConfig, State)


PROFILE = Path(__file__).resolve().parents[1] / 'examples/bullet_profile.json'


def warm_hold():
    config = NavigatorConfig.from_json(PROFILE)
    hold = AltitudeHeadingHold(config)
    for i in range(31):
        now = 1 + i * .01
        inputs = NavigationInput(attitude=AttitudeSample('sensor', i, now, (0, 0, 0, 1)),
                                 barometer=BarometerSample('sensor', i, now, 20.))
        hold.observe(inputs)
        if hold.ready(now):
            command = hold.command(now_s=now, target_altitude_m=20.)
    return config, hold, inputs, command


def test_handoff_preserves_filter_history_heading_and_thrust():
    config, hold, inputs, command = warm_hold()
    handoff = hold.handoff(now_s=1.3, target_altitude_m=20.)
    nav = LosNavigator(config)
    nav.reset(handoff=handoff)
    result = nav.update(inputs, now_s=1.31, enabled=True)
    assert result.state == State.ACQUIRE and result.usable
    assert result.command == command
    assert result.diagnostics.vertical_ready
    assert result.diagnostics.altitude_m == 20.
    assert result.diagnostics.vertical_speed_m_s == 0.
    assert len(handoff.attitudes) == 31
    # Snapshot data do not alias the hold's mutable history.
    hold.observe(NavigationInput(attitude=AttitudeSample('sensor', 31, 1.31, (0, 0, 0, 1))))
    assert len(handoff.attitudes) == 31
    with pytest.raises(ValueError, match='profile'):
        LosNavigator(replace(config, target_altitude_m=1.)).reset(handoff=handoff)


def test_invalid_handoff_and_unready_hold():
    config, hold, _, _ = warm_hold()
    handoff = hold.handoff(now_s=1.3, target_altitude_m=20.)
    for changes in (dict(thrust_integral_n=float('nan')), dict(attitudes=()),
                    dict(attitudes=handoff.attitudes[::-1]), dict(thrust_integral_n=1e9)):
        with pytest.raises(ValueError):
            replace(handoff, **changes)
    with pytest.raises(ValueError):
        AltitudeHeadingHold(config).handoff(now_s=0., target_altitude_m=20.)


def test_public_heading_yaw_override_validation_and_filter_continuity():
    from math import radians, pi
    config, hold, _, base = warm_hold()
    prior_vertical = hold.vertical
    prior_integral = hold.control.thrust_integral
    hold.set_heading(2 * pi)
    assert hold.heading_rad == pytest.approx(0.)
    command = hold.command(now_s=1.31, target_altitude_m=20., yaw_rate_rad_s=radians(15))
    assert command.throttle == base.throttle and command.yaw > 1500
    assert hold.vertical is prior_vertical and hold.control.thrust_integral == prior_integral
    assert hold.command(now_s=1.32, target_altitude_m=20.).yaw == 1500
    assert hold.attitude_at(1.305) is None  # No extrapolation.
    assert hold.attitude_at(1.295) == pytest.approx((0, 0, 0, 1))
    for value in (float('nan'), float('inf'), True, config.tuning.yaw_rate_limit_rad_s + .01):
        with pytest.raises(ValueError):
            hold.command(now_s=1.33, target_altitude_m=20., yaw_rate_rad_s=value)
        assert hold.last_now == 1.32
    with pytest.raises(ValueError):
        hold.set_heading(float('nan'))
    assert hold.heading_rad == 0.
