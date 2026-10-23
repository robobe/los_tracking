# Failure examples and entry-method review

## Failure and boundary examples

| Situation | Entry-method behavior | Existing check |
| --- | --- | --- |
| Cold first tick | `ACQUIRE`, `vertical_warmup`, no command. | `test_acquisition_needs_vertical_warmup_and_new_visual_samples` |
| Disable active tracking | `DISABLED`, no command; later enable faults with `reset_required` unless reset. | `test_disable_requires_reset_before_new_mission` |
| Control call delayed beyond 0.1 s | Latch `control_gap`, no new command. Previously returned command retains its original expiry. | `test_control_gap_faults_without_renewing_a_command` |
| Required sensor becomes stale after readiness | Latch `attitude_stale` or `barometer_stale`; fresh data alone cannot clear fault. | `test_stale_required_sensor_latches_fault_until_reset` |
| Sensor exactly at freshness deadline | State may remain `TRACK` while expiry clears the command. Freshness comparison uses tolerance; expiry still forbids use at the deadline. | `test_exact_sensor_age_boundary_expires_command_without_early_fault` |
| Explicit no-target observation | Enter recovery and hold captured altitude while references slew toward hold. | `test_target_loss_recovers_then_reacquires_original_reference` |
| Image still fresh but cannot cover positive caller horizon | `image_deadline`; tracking enters recovery before image-limited RC would expire. | `test_image_deadline_recovers_before_next_command_can_expire` |
| Descent infeasible continuously | Reduce requested pitch, then enter recovery with `persistent_limitation`. | `test_ttc_infeasibility_moderates_pitch_then_recovers` |
| Malformed tick or negative/nonfinite horizon | Raise `ValueError` before state mutation. | `test_invalid_tick_is_atomic`, `test_invalid_command_horizon_is_rejected_before_mutation` |

Checks are in [test_navigator.py](../../tests/test_navigator.py). Handoff continuity
is covered by `test_handoff_preserves_filter_history_heading_and_thrust` in
[test_handoff.py](../../tests/test_handoff.py).

## Entry-method review notes

These are observations about the implemented boundaries, separate from the
algorithm explanation. No code changes are proposed by this chapter.

- Safety decisions mix different times deliberately: bearing uses image-time
  attitude and prediction to control time, while tilt and thrust compensation
  use the latest measured attitude. Timestamps must share a clock domain.
- Feasibility and authority limitation are independent of TTC stage reporting.
  Missing TTC alone does not stop the bearing approach; terminal does not declare
  collision or completion.
- Acquisition/recovery holds altitude and heading, not horizontal position. Level
  requested pitch does not guarantee braking or a stationary hover.
- A result containing numeric diagnostics is not proof those estimates are
  currently valid. Inspect validity flags, state, reason and command expiry.

[Back to walkthrough](../approach-walkthrough.md)
