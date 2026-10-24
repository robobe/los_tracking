from dataclasses import replace
from math import radians

import pytest
from los_tracking import (AttitudeSample, BarometerSample, LosNavigator, NavigationInput,
                          State, TargetObservation, TrackStage)

from conftest import DEFAULT_BOX, IDENTITY, acquire, from_rpy, samples


def test_acquisition_needs_vertical_warmup_and_new_visual_samples(config):
    nav = LosNavigator(config)
    early = nav.update(samples(0), now_s=0, enabled=True)
    assert not early.usable and early.reason == "vertical_warmup"
    result = acquire(nav, start=1)
    assert result.state == State.TRACK and result.usable
    assert result.diagnostics.rate_observable
    assert result.diagnostics.reference_elevation_rad > 0
    assert result.generated_at_s < result.valid_until_s <= result.generated_at_s + 0.05
    assert len(result.command.as_tuple()) == 4


def test_duplicate_frame_never_acquires_or_renews_freshness(config):
    nav = LosNavigator(config)
    frame = TargetObservation("camera", 0, 0.1, DEFAULT_BOX)
    for i in range(31):
        inputs = replace(samples(i, camera=False), target=frame if i >= 10 else None)
        result = nav.update(inputs, now_s=i / 100, enabled=True)
    assert result.state == State.ACQUIRE
    assert not result.diagnostics.rate_observable
    assert result.diagnostics.image_age_s == pytest.approx(0.2)
    assert result.diagnostics.vision_reason == "stale_image"


def test_target_loss_recovers_then_reacquires_original_reference(config):
    nav = LosNavigator(config)
    tracked = acquire(nav)
    reference = (tracked.diagnostics.reference_elevation_rad, tracked.diagnostics.reference_azimuth_rad)
    before_pitch = tracked.diagnostics.reference_pitch_rad
    lost = nav.update(samples(35, bbox=None), now_s=0.35, enabled=True)
    assert lost.state == State.RECOVER and lost.usable and lost.reason == "no_target"
    assert lost.diagnostics.reference_pitch_rad < before_pitch
    for i in range(36, 51):
        result = nav.update(samples(i, bbox=(315, 280, 20, 20)), now_s=i / 100, enabled=True)
    assert result.state == State.TRACK
    assert (result.diagnostics.reference_elevation_rad, result.diagnostics.reference_azimuth_rad) == reference


def test_missing_image_is_not_immediate_no_target_and_recovery_has_deadline(config):
    nav = LosNavigator(config)
    acquire(nav)
    for i in range(35, 40):
        result = nav.update(samples(i, camera=False), now_s=i / 100, enabled=True)
    assert result.state == State.TRACK
    for i in range(40, 160):
        result = nav.update(samples(i, camera=False), now_s=i / 100, enabled=True)
    assert result.state == State.FAULT and result.reason == "recovery_timeout"
    assert result.command is None
    assert acquire(nav, start=160).state == State.FAULT
    nav.reset()
    assert acquire(nav).state == State.TRACK


@pytest.mark.parametrize("missing, expected", [("attitude", "attitude_stale"), ("barometer", "barometer_stale")])
def test_stale_required_sensor_latches_fault_until_reset(config, missing, expected):
    nav = LosNavigator(config)
    acquire(nav)
    for i in range(35, 60):
        result = nav.update(replace(samples(i), **{missing: None}), now_s=i / 100, enabled=True)
    assert result.state == State.FAULT and result.reason == expected and not result.usable
    assert nav.update(samples(60), now_s=0.6, enabled=True).reason == expected


def test_control_gap_faults_without_renewing_a_command(config):
    nav = LosNavigator(config)
    old = acquire(nav)
    assert old.valid_until_s < 0.6
    result = nav.update(samples(60), now_s=0.6, enabled=True)
    assert result.state == State.FAULT and result.reason == "control_gap"
    assert not result.usable and result.valid_until_s == 0.6


def test_disable_requires_reset_before_new_mission(config):
    nav = LosNavigator(config)
    acquire(nav)
    disabled = nav.update(NavigationInput(), now_s=0.35, enabled=False)
    assert disabled.state == State.DISABLED and not disabled.usable
    assert nav.update(samples(36), now_s=0.36, enabled=True).reason == "reset_required"
    nav.reset()
    assert acquire(nav).state == State.TRACK


@pytest.mark.parametrize("kind", ["target", "barometer", "attitude"])
def test_stream_restarts_require_reset(config, kind):
    nav = LosNavigator(config)
    acquire(nav)
    inputs = samples(35)
    inputs = replace(inputs, **{kind: replace(getattr(inputs, kind), stream_id="restarted")})
    result = nav.update(inputs, now_s=0.35, enabled=True)
    assert result.reason == f"{kind}_stream_changed" and not result.usable


def test_out_of_order_measurements_do_not_replace_or_refresh_state(config):
    nav = LosNavigator(config)
    acquire(nav)
    old = TargetObservation("camera", 2, 0.02, DEFAULT_BOX)
    result = nav.update(replace(samples(35), target=old), now_s=0.35, enabled=True)
    assert result.state == State.TRACK
    assert result.diagnostics.input_rejection == "target_out_of_order"
    assert result.diagnostics.image_age_s == pytest.approx(0.01)


def test_invalid_tick_is_atomic(config):
    nav = LosNavigator(config)
    acquire(nav)
    good = samples(35)
    bad_cases = [replace(good, attitude=replace(good.attitude, timestamp_s=1)),
                 replace(good, target=replace(good.target, bbox=(630, 100, 20, 20))),
                 replace(good, barometer=BarometerSample("baro", 34, 0.34, 99))]
    for bad in bad_cases:
        with pytest.raises(ValueError):
            nav.update(bad, now_s=0.35, enabled=True)
    result = nav.update(good, now_s=0.35, enabled=True)
    assert result.state == State.TRACK
    with pytest.raises(ValueError, match="strictly increase"):
        nav.update(good, now_s=0.35, enabled=True)
    with pytest.raises(ValueError, match="finite"):
        nav.update(NavigationInput(), now_s=float("nan"), enabled=True)


@pytest.mark.parametrize("bbox,reason", [((0, 100, 20, 20), "clipped_target"),
                                          ((450, 280, 20, 20), "los_deviation")])
def test_clipping_and_association_changes_do_not_become_new_targets(config, bbox, reason):
    nav = LosNavigator(config)
    reference = acquire(nav).diagnostics.reference_elevation_rad
    result = nav.update(samples(35, bbox=bbox), now_s=0.35, enabled=True)
    assert result.state == State.RECOVER and result.reason == reason
    assert result.diagnostics.reference_elevation_rad == reference


def test_measured_over_tilt_recovers_with_frozen_integrator(config):
    nav = LosNavigator(config)
    acquire(nav)
    integral = nav._control.thrust_integral
    result = nav.update(samples(35, attitude=from_rpy(roll=radians(35))), now_s=0.35, enabled=True)
    assert result.state == State.RECOVER and result.reason == "measured_over_tilt"
    assert result.diagnostics.measured_over_tilt
    assert result.diagnostics.output_tilt_rad <= config.tuning.combined_tilt_limit_rad
    assert nav._control.thrust_integral == integral


def test_capture_alignment_uses_history_and_waits_for_right_endpoint(config):
    nav = LosNavigator(config)
    acquire(nav)
    delayed = TargetObservation("camera", 35, 0.345, DEFAULT_BOX)
    # No attitude at or after exposure yet: attitude sample is 0.34.
    inputs = NavigationInput(delayed, BarometerSample("baro", 35, 0.35, 10), None)
    result = nav.update(inputs, now_s=0.35, enabled=True)
    assert result.state == State.RECOVER
    assert result.diagnostics.vision_reason == "attitude_unaligned"
    # Next attitude brackets the pending exposure; do not require a new image.
    inputs = replace(samples(36, camera=False), target=None)
    result = nav.update(inputs, now_s=0.36, enabled=True)
    assert result.diagnostics.vision_reason == "usable"
    assert result.diagnostics.image_age_s == pytest.approx(0.015)


def test_acquisition_times_out_without_inputs(config):
    nav = LosNavigator(config)
    for i in range(501):
        result = nav.update(NavigationInput(), now_s=i / 100, enabled=True)
    assert result.state == State.FAULT and result.reason == "acquisition_timeout"
    assert not result.usable


def test_ttc_stages_and_ttc_loss_keep_same_track_and_original_memory(config):
    config = replace(config, target_altitude_m=10)
    nav = LosNavigator(config)
    stages = set()
    for i in range(231):
        now = i / 100
        size = 40 / (3 - now)
        bbox = (320 - size / 2, 240 - size / 2, size, size)
        result = nav.update(samples(i, bbox=bbox), now_s=now, enabled=True)
        stages.add(result.stage)
    assert result.state == State.TRACK
    assert TrackStage.TTC in stages and result.stage == TrackStage.TERMINAL
    reference = result.diagnostics.reference_elevation_rad
    for i in range(231, 301):
        result = nav.update(samples(i, bbox=bbox), now_s=i / 100, enabled=True)
    assert result.state == State.TRACK and result.stage == TrackStage.TERMINAL
    assert result.diagnostics.optical_ttc_s is None
    assert result.diagnostics.reference_elevation_rad == reference
    assert result.usable  # No collision/completion is inferred from TTC or scale.


def test_ttc_infeasibility_moderates_pitch_then_recovers(config):
    nav = LosNavigator(replace(config, target_altitude_m=-100))
    saw_infeasibility = False
    for i in range(180):
        now = i / 100
        size = 40 / (3 - now)
        bbox = (320 - size / 2, 290 - size / 2, size, size)
        result = nav.update(samples(i, bbox=bbox), now_s=now, enabled=True)
        saw_infeasibility |= result.diagnostics.descent_feasible is False
        if result.state == State.RECOVER:
            break
    assert saw_infeasibility and result.reason == "persistent_limitation"


def test_finite_but_overflowing_vertical_measurement_fails_closed(config):
    nav = LosNavigator(config)
    acquire(nav)
    result = nav.update(samples(35, altitude=-1e308), now_s=0.35, enabled=True)
    assert result.state == State.FAULT and result.reason == "vertical_numeric_failure"
    assert result.command is None
    assert result.diagnostics.vertical_speed_m_s is None


def test_first_frame_zero_rate_is_not_observable(config):
    nav = LosNavigator(config)
    for i in range(11):
        result = nav.update(samples(i, camera=i == 10), now_s=i / 100, enabled=True)
    assert result.diagnostics.bearing_valid
    assert not result.diagnostics.rate_observable
    assert result.diagnostics.elevation_rate_rad_s is None


def test_ttc_stage_entry_requires_continuous_dwell_and_exit_has_hysteresis(config, monkeypatch):
    nav = LosNavigator(config)
    current_ttc = [7.0]
    monkeypatch.setattr(nav._ttc, "at", lambda now: (current_ttc[0], "test_qualified"))
    nav._stage_and_feasibility(1, 10, 0)
    current_ttc[0] = 9
    nav._stage_and_feasibility(1.2, 10, 0)
    current_ttc[0] = 7
    nav._stage_and_feasibility(1.4, 10, 0)
    nav._stage_and_feasibility(1.5, 10, 0)
    assert nav.stage == TrackStage.BEARING
    nav._stage_and_feasibility(1.71, 10, 0)
    assert nav.stage == TrackStage.TTC
    current_ttc[0] = 9
    nav._stage_and_feasibility(1.8, 10, 0)
    assert nav.stage == TrackStage.TTC
    current_ttc[0] = 10.1
    nav._stage_and_feasibility(1.9, 10, 0)
    assert nav.stage == TrackStage.BEARING
    current_ttc[0] = 0.5
    nav._stage_and_feasibility(2, 10, 0)
    current_ttc[0] = None
    nav._stage_and_feasibility(2.1, 10, 0)
    assert nav.stage == TrackStage.TERMINAL


def test_exact_sensor_age_boundary_expires_command_without_early_fault(config):
    nav = LosNavigator(config)
    acquire(nav, count=36)  # Latest attitude timestamp 0.35.
    boundary = nav.update(replace(samples(40, camera=False), attitude=None), now_s=0.4, enabled=True)
    assert boundary.state == State.TRACK  # 0.4 - 0.35 rounds slightly above 0.05.
    assert not boundary.usable and boundary.valid_until_s == pytest.approx(0.4)
    fresh = nav.update(samples(41), now_s=0.41, enabled=True)
    assert fresh.state == State.TRACK and fresh.usable


@pytest.mark.parametrize('azimuth_deg', [-35, -32, 32, 35])
def test_default_azimuth_envelope_accepts_through_35_degrees(config, azimuth_deg):
    from math import tan
    assert config.tuning.max_azimuth_rad == pytest.approx(radians(35))
    camera = config.camera
    box = (camera.cx_px - camera.fx_px * tan(radians(azimuth_deg)) - 5, camera.cy_px - 5, 10, 10)
    result = acquire(LosNavigator(config), bbox=box)
    assert result.state == State.TRACK
    assert result.diagnostics.bearing_valid
    assert result.diagnostics.azimuth_rad == pytest.approx(radians(azimuth_deg))
    assert result.diagnostics.rejected_azimuth_rad is None


@pytest.mark.parametrize('azimuth_deg', [-35.01, 35.01])
def test_rejected_bearing_diagnostics_preserve_cause_through_timeout(config, azimuth_deg):
    from math import tan
    camera = config.camera
    box = (camera.cx_px - camera.fx_px * tan(radians(azimuth_deg)) - 5, camera.cy_px - 5, 10, 10)
    nav = LosNavigator(config)
    for index in range(501):
        result = nav.update(samples(index, bbox=box), now_s=index * .01, enabled=True)
    assert result.state == State.FAULT and result.reason == 'acquisition_timeout'
    d = result.diagnostics
    assert d.vision_reason == 'bearing_outside_envelope'
    assert d.rejected_azimuth_rad == pytest.approx(radians(azimuth_deg))
    assert d.max_azimuth_rad == pytest.approx(radians(35))
    assert not d.bearing_valid and d.azimuth_rad is None
    nav.reset()
    result = acquire(nav)
    assert result.state == State.TRACK and result.diagnostics.rejected_azimuth_rad is None


def test_fresh_envelope_rejection_is_not_hidden_by_stale_previous_good_bearing(config):
    from math import tan
    nav = LosNavigator(config)
    acquire(nav)
    box = (config.camera.cx_px - config.camera.fx_px * tan(radians(36)) - 5,
           config.camera.cy_px - 5, 10, 10)
    for index in range(35, 141):
        result = nav.update(samples(index, bbox=box), now_s=index * .01, enabled=True)
    assert result.state == State.FAULT and result.reason == 'recovery_timeout'
    assert result.diagnostics.vision_reason == 'bearing_outside_envelope'
    assert result.diagnostics.rejected_azimuth_rad == pytest.approx(radians(36))


def test_rejection_context_clears_when_visual_cause_changes_or_target_becomes_usable(config):
    from math import tan
    nav = LosNavigator(config)
    box = (config.camera.cx_px - config.camera.fx_px * tan(radians(36)) - 5,
           config.camera.cy_px - 5, 10, 10)
    result = acquire(nav, bbox=box)
    assert result.diagnostics.rejected_azimuth_rad is not None
    result = nav.update(samples(35, bbox=None), now_s=.35, enabled=True)
    assert result.diagnostics.vision_reason == 'no_target'
    assert result.diagnostics.rejected_azimuth_rad is None
    result = acquire(nav, start=36)
    assert result.state == State.TRACK and result.diagnostics.rejected_azimuth_rad is None


def test_image_deadline_recovers_before_next_command_can_expire(config):
    nav = LosNavigator(config)
    acquire(nav)
    # The image is still fresh, but its deadline cannot cover the caller's next tick and delivery.
    result = nav.update(samples(43, camera=False), now_s=0.43, enabled=True, command_horizon_s=0.02)
    assert result.state == State.RECOVER and result.usable
    assert result.reason == 'image_deadline'
    assert result.valid_until_s >= 0.45


@pytest.mark.parametrize('horizon', [-0.01, float('nan'), float('inf')])
def test_invalid_command_horizon_is_rejected_before_mutation(config, horizon):
    nav = LosNavigator(config)
    with pytest.raises(ValueError):
        nav.update(samples(0), now_s=0.0, enabled=True, command_horizon_s=horizon)
    assert nav.state == State.DISABLED


@pytest.mark.parametrize('bbox,edges', [((0,100,20,20),('left',)),((100,0,20,20),('top',)),
                                      ((620,100,20,20),('right',)),((100,460,20,20),('bottom',)),
                                      ((310,280,20,20),())])
def test_processed_image_identity_and_edge_margins(config,bbox,edges):
    nav=LosNavigator(config)
    acquire(nav)
    result=nav.update(samples(35,bbox=bbox),now_s=.35,enabled=True)
    d=result.diagnostics
    assert d.processed_image_sequence==35 and d.processed_image_stream_id=='camera'
    assert d.processed_image_timestamp_s==pytest.approx(.35)
    assert d.processed_bbox==bbox and d.touched_edges==edges
    x,y,w,h=bbox
    assert d.edge_margins_px==(x,y,640-x-w,480-y-h)
    if edges:
        assert d.vision_reason=='clipped_target' and result.state==State.RECOVER
