# Update cycle: LosNavigator.update

## Block 1: validate arguments and time

**Idea:** reject malformed input before changing navigation state.

The method validates `command_horizon_s`, then calls `_validate_tick()`. Sample
constructors have already validated their payload shapes and scalar values.
Tick validation checks input/enable types, finite increasing control time,
future measurements, reuse of the latest sample identity with different content,
and bbox containment in the calibrated image.

```text
now > previous_control_time
measurement_time ≤ now + TIME_EPSILON
```

`TIME_EPSILON = 1e-9 s` is a fixed numerical tolerance, not tuning. Invalid data
raises `ValueError`; no `NavigationResult` is returned. Validation also runs when
disabled or already faulted. An out-of-image bbox is invalid input, whereas a
contained bbox touching an edge is later rejected as clipped vision.

**Configuration:** `camera.width_px` and `camera.height_px` define containment;
change them with image resolution/calibration. The horizon has no enforced
upper bound, but a horizon at least as large as image lifetime prevents usable
vision from qualifying. Detailed validation review is pending.

## Block 2: handle faults and authorization

After validation, save the previous time, set `_last_now = now`, and clear the
per-tick input-rejection diagnostic. Then apply these gates in order:

| Condition | State/result | Return behavior |
| --- | --- | --- |
| Already `FAULT` | Keep the original latched fault. | No command; do not ingest samples. |
| `enabled=False` | Set `DISABLED`; mark an active session `_ended`. | No command; do not ingest samples. |
| Enabled after an active session ended | Latch `FAULT`, reason `reset_required`. | No command. |

**Configuration:** none. `enabled` is a runtime boolean. Disabling does not clear
a fault because the fault gate runs first. Disabled calls before the first
enable do not mark the session ended. Call `reset()` to start a new session after
active disable or fault.

## Block 3: enter acquisition and compute timestep

On first enable, change `DISABLED → ACQUIRE` and set `_started_at = now`.
For a cold navigator, discard the preceding control time for this calculation;
the first enabled tick uses zero elapsed time. A handoff preserves its preceding
time because sensors were already ready.

```text
dt = 0                         if there is no applicable previous time
dt = now - previous_time       otherwise
```

The controller uses `dt` for reference slew and integral updates. Each estimator
instead derives measurement intervals from its own accepted sample timestamps.
This distinction prevents repeated control ticks from becoming fake measurements.

**Configuration:** `max_control_gap_s = 0.1 s`, finite and positive. If
`dt > max_control_gap_s + TIME_EPSILON`, latch `control_gap` and return no command
before ingesting new samples. Increasing the setting tolerates longer scheduling
interruptions; decreasing it faults sooner. This check is after the disable gate.

## Block 4: ingest new measurements

Call `_accept()` independently for attitude, barometer and target, in that order:

| Accepted measurement | Action |
| --- | --- |
| Attitude | Append to bounded image-alignment history. |
| Barometer | Update the vertical alpha-beta estimator. |
| Target | Replace `_pending_target` for later visual processing. |

An exact repeat is ignored. A sample that does not advance both sequence and time
is ignored and can set `input_rejection`. A stream change latches a fault. The
fault result is returned after this ingestion block; a vertical-estimator
arithmetic failure latches `vertical_numeric_failure`.

**Configuration used by delegated operations:** history horizon 0.5 s, maximum
interpolation gap 0.05 s, barometer alpha/beta 0.35/0.05. Longer history supports
older image alignment; larger allowed gaps accept coarser interpolation; higher
filter gains react faster to measurements and noise. See the
[filtering reference](../package-parameters.md#5-filtering-and-sensor-alignment)
for bounds. No tuning field changes sample-identity rules.

## Block 5: require ready and fresh sensors

**Idea:** do not generate control until vertical speed is observable, and do not
silently restart warmup after losing previously ready sensors.

```text
sample_age = now - measurement_time
stale = sample missing OR sample_age > configured_max_age + TIME_EPSILON
```

Attitude is checked first, then barometer. Vertical readiness additionally needs
at least three barometer samples spanning `vertical_warmup_s`.

| Configuration | Default and limit | Changing it |
| --- | --- | --- |
| `max_attitude_age_s` | 0.05 s, `> 0` | Higher tolerates older attitude; lower faults sooner after readiness. |
| `max_barometer_age_s` | 0.2 s, `> 0` | Higher permits longer prediction from old altitude data; lower faults sooner. |
| `vertical_warmup_s` | 0.1 s, `> 0` | Higher delays readiness; lower uses less initial history. Three samples remain mandatory. |
| `acquisition_timeout_s` | 5 s, greater than positive acquisition dwell | Higher allows more warmup/acquisition time; lower reaches timeout sooner. |

Before readiness, return no command with the missing/stale-sensor reason or
`vertical_warmup`. At acquisition timeout, latch `acquisition_timeout`. After
readiness, bad sensors instead latch their fault immediately.

With ready sensors, obtain `(altitude, vz)` predicted to `now` from the vertical
estimator. Nonfinite predicted values latch `vertical_numeric_failure`. Detailed
filter math belongs to the pending vertical-estimation chapter.

## Block 6: establish hold and process vision

If no heading was restored, take yaw from the latest attitude and capture the
filtered current altitude as the initial hold target. Then `_process_target()`
attempts image-time attitude alignment, bbox-to-world LOS conversion, acceptance
checks and bearing/TTC updates.

After processing, check the last accepted visual measurement's age. If it has
expired, reject visual usability. If a positive horizon was supplied, also check:

```text
image_deadline = last_usable_image_time + max_image_age_s
remaining_image_budget = image_deadline - now
reject with image_deadline if remaining_image_budget ≤ command_horizon_s + TIME_EPSILON
```

**Configuration:** `max_image_age_s = 0.1 s`, positive and no greater than
`attitude_history_s`. Higher tolerates more visual latency and longer frame gaps;
lower rejects old visual data earlier. A positive horizon reserves caller
scheduling/delivery budget. With horizon zero, this early deadline rejection is
disabled; normal image age and expiry checks still apply.

Camera calibration, angular envelopes, LOS-deviation and association limits,
bearing filtering and TTC-fit settings belong to `_process_target()` and its
delegates. Their full values/limits are in the [parameter reference](../package-parameters.md).
Their math will be reviewed separately rather than duplicated here.

## Block 7: advance navigation state

Calculate measured total tilt from the latest attitude:

```text
measured_tilt = acos(clamp(cos(measured_roll) × cos(measured_pitch), -1, 1))
```

Then apply transitions in this order:

1. Excessive measured tilt enters `RECOVER`, clears visual candidates and prevents
   reacquisition that tick.
2. Otherwise, `TRACK` with unusable vision enters `RECOVER` using the vision reason.
3. Otherwise, `ACQUIRE` or `RECOVER` with usable vision can enter `TRACK` after
   candidate qualification.
4. If still acquiring/recovering, reaching its timeout latches a fault.

| Configuration | Default and limits | Changing it |
| --- | --- | --- |
| `combined_tilt_limit_rad` | 30°, positive and below 90°; must cover approach pitch | Higher permits more tilt before recovery; lower recovers earlier. Also bounds control output. |
| `acquisition_dwell_s` | 0.1 s, positive and below acquisition timeout | Higher requires a longer accepted-frame span before tracking. |
| `recovery_dwell_s` | 0.1 s, positive and below recovery timeout | Higher requires longer reacquisition before resuming tracking. |
| `acquisition_timeout_s` | 5 s, greater than acquisition dwell | Higher waits longer before acquisition fault. |
| `recovery_timeout_s` | 1 s, greater than recovery dwell | Higher permits longer recovery before fault. |

Qualification requires at least three accepted candidate frames spanning the
appropriate dwell, plus at least two bearing-filter observations. Dwell uses
measurement times, not the number of control ticks. On the first transition to
tracking, save the candidate LOS reference; reacquisition preserves that reference.

Recovery captures current altitude for hold and resets bearing/TTC reacquisition
state while preserving heading and acquired LOS. Roll/pitch and vertical-speed
references still obey slew limits, so recovery does not instantly level the drone.
Because qualification is checked before timeout, a successful transition to
`TRACK` on the deadline tick avoids that acquisition/recovery timeout check.

## Block 8: supervise the approach

Initialize `errors=None` and `approach_scale=0`. Only `TRACK` calls
`_stage_and_feasibility()` and then checks authority limitation.

Qualified TTC estimates remaining optical closing time. Feasibility compares
height above `config.target_altitude_m` with descent achievable under configured
speed/reference-acceleration bounds and margins. TTC stage is reporting state;
feasibility can act even while stage is `BEARING`.

Authority limitation uses the **previous control output's** angle/vertical-speed
limitation or thrust saturation, plus the current vertical reference at its
climb/descent bounds. A slew-limit flag alone does not trigger it.

```text
limited = authority_limited OR descent_feasible is False
elapsed = now - first_tick_of_continuous_limitation
approach_scale = max(0, 1 - elapsed / limitation_timeout_s)
```

On the first limited tick elapsed is zero, so requested scale is still one. If
limitation clears, reset its timer and request scale one. If limitation persists
for the timeout, enter recovery with `persistent_limitation`.

**Configuration directly read here:** `limitation_timeout_s = 0.75 s`,
`max_descent_m_s = 5.5 m/s`, `max_climb_m_s = 3 m/s`, all positive. A shorter
limitation timeout reduces pitch and enters recovery faster. Higher speed limits
allow larger references before speed-bound supervision triggers.

Delegated TTC thresholds are terminal/entry/exit = 0.8/8/10 s with 0.3 s entry
dwell. They must satisfy `0 < terminal < entry < exit`; dwell must be positive.
Higher thresholds change stage transition timing, not feedback gains.
Feasibility also uses vertical slew (1 m/s²), time margin (0.1 s) and height
margin (0.25 m). Larger time margin is stricter; **larger height margin is more
permissive**. Full equations and fit qualification remain pending.

If still tracking, compute error from predicted filtered bearing at control time:

```text
wrap(a) = ((a + π) modulo (2π)) - π
elevation_error = wrap(predicted_elevation - reference_elevation)
azimuth_error = wrap(predicted_azimuth - reference_azimuth)
```

Wrapped errors avoid discontinuities at ±π. The saved reference is the acquired
world LOS, not a desired image-centre pixel. If supervision entered recovery,
`errors` remains `None`, selecting hold behavior in the controller that same tick.

## Block 9: control and command expiry

Call `_control.step()` with `dt`, predicted altitude/speed, latest measured
attitude, saved heading, optional LOS errors, filtered LOS rates, held altitude,
approach scale and an over-tilt integral-freeze flag.

| Mode selected by `errors` | Requested behavior |
| --- | --- |
| `None` | Altitude/heading hold, with level roll/pitch references. |
| Elevation/azimuth errors | LOS tracking: elevation → vertical motion, azimuth → roll, scaled approach pitch, heading hold. |

The control method owns feedback gains, integral memory, speed/angle/thrust limits,
slew, tilt compensation, anti-windup and RC encoding. Its configuration is linked
in the [control parameter reference](../package-parameters.md#4-controller-gains-and-integral-limits).
Detailed equations are reserved for the control chapter. An arithmetic failure
latches `control_numeric_failure` and returns no command.

Finally `_result()` collects state/diagnostics and computes expiry:

```text
expiry = min(
    now + command_validity_s,
    latest_attitude_time + max_attitude_age_s,
    latest_barometer_time + max_barometer_age_s,
    applicable_state_deadline,
)
```

The state deadline is acquisition start + acquisition timeout in `ACQUIRE`, last
usable image time + image age limit in `TRACK`, or recovery start + recovery
timeout in `RECOVER`. If expiry is already reached, clear the command. When no
command was generated, expiry is `now`.

**Configuration:** `command_validity_s = 0.05 s`, finite and positive. Increasing
it allows longer reuse only when every other deadline permits it; decreasing it
requires faster replacement. Sensor/state settings can always shorten validity.

Every normal return is a `NavigationResult`; `command` may be `None`.
`command.as_tuple()` is `(roll, pitch, throttle, yaw)`, with integer channels in
1000–2000. `usable` only checks command presence. The caller must separately stop
using previously returned commands at their deadline.

[Back to walkthrough](../approach-walkthrough.md)
