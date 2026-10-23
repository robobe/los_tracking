# LOS tracking parameter reference

This reference accompanies [package control flow](package-control-flow.md). It
explains every configuration parameter and the runtime inputs used in that flow:
their purpose, validation limits, and expected behavior when values change.

## Menu

- [Reading the limits](#reading-the-limits)
- [1. Camera calibration](#1-camera-calibration)
- [2. Vehicle configuration and target altitude](#2-vehicle-configuration-and-target-altitude)
- [3. Approach, angle and speed limits](#3-approach-angle-and-speed-limits)
- [4. Controller gains and integral limits](#4-controller-gains-and-integral-limits)
- [5. Filtering and sensor alignment](#5-filtering-and-sensor-alignment)
- [6. Freshness and command deadlines](#6-freshness-and-command-deadlines)
- [7. Acquisition and recovery timing](#7-acquisition-and-recovery-timing)
- [8. Target acceptance limits](#8-target-acceptance-limits)
- [9. Optical TTC and descent supervision](#9-optical-ttc-and-descent-supervision)
- [10. Runtime inputs and method parameters](#10-runtime-inputs-and-method-parameters)
- [11. Hold handoff parameters](#11-hold-handoff-parameters)
- [12. Returned values and command use](#12-returned-values-and-command-use)
- [Sources and verification](#sources-and-verification)

## Reading the limits

`> 0` means strictly positive; `≥ 0` permits zero. All numeric values must be
finite. These are **code validation limits**, not proven safe operating ranges.
Angles are stored in radians; tables show degrees where easier to read.

The values below match the current `Tuning` defaults and Bullet profile. Camera,
mass, maximum thrust and target altitude are supplied by the profile rather than
having universal defaults. Configuration objects are immutable; configure a new
session rather than treating these fields as live tuning controls.

`NavigatorConfig` groups camera calibration under `camera`, vehicle properties
under `vehicle`, controller settings under `tuning`, and the target altitude in
`target_altitude_m`. The first two groups and target altitude are required;
omitting `tuning` uses `Tuning` defaults. Unknown JSON profile fields are rejected.

## 1. Camera calibration

These describe the actual camera. Change them when calibration, resolution or
mounting changes.

| Parameter | Bullet value | Purpose and allowed values | Effect of changing it |
| --- | --- | --- | --- |
| `width_px` | 640 | Image width; positive integer. Bboxes must fit inside it. | Changes image boundary checks. Changing resolution requires corresponding calibration and detector coordinates. |
| `height_px` | 480 | Image height; positive integer. | Same effect for vertical boundaries. |
| `fx_px` | 320 | Horizontal focal length in pixels; `> 0`. | Larger value interprets the same horizontal pixel offset as a smaller angle; smaller value produces a larger angle. |
| `fy_px` | 320 | Vertical focal length in pixels; `> 0`. | Larger value interprets the same vertical pixel offset as a smaller angle. |
| `cx_px` | 320 | Optical centre X; `0 ≤ cx ≤ width`. | Increasing it shifts the inferred camera ray left for a fixed pixel location. Incorrect values introduce bearing bias. |
| `cy_px` | 240 | Optical centre Y; `0 ≤ cy ≤ height`. | Increasing it shifts the inferred camera ray upward for a fixed pixel location. Incorrect values bias elevation. |
| `camera_to_body_xyzw` | `(0,0,0,1)` | Camera-to-body mounting rotation as a unit XYZW quaternion. Input norm must be within `0.001` of one, then it is normalized. | Rotates every measured LOS. Incorrect mounting rotation can mix or reverse corrections. |

`CameraCalibration.from_horizontal_fov(width_px, height_px, fov_rad)` additionally
accepts **horizontal FOV between 0 and π radians**, exclusive. Dimensions must be
positive integers. Larger FOV produces smaller focal lengths and larger inferred
angles per pixel. This helper sets equal X/Y focal lengths and a centred optical
centre.

## 2. Vehicle configuration and target altitude

| Parameter | Default / Bullet value | Purpose and limits | Effect of changing it |
| --- | --- | --- | --- |
| `mass_kg` | Bullet: 1 kg | Vehicle mass; `> 0`. Must satisfy `max_thrust_n > mass × gravity`. | Higher configured mass raises gravity-compensation thrust. It must match the vehicle. |
| `max_thrust_n` | Bullet: 33.953391 N | Maximum collective thrust; greater than level hover thrust. Also defines throttle encoding. | Higher value allows more calculated thrust, but produces a smaller throttle fraction for the same requested newtons. Must match the receiver's thrust model. |
| `gravity_m_s2` | 9.81 m/s² | Gravity used in hover compensation; `> 0`. | Higher value raises calculated hover thrust. Match the physical/simulated environment. |
| `rc_angle_scale_rad` | 45° | Angle represented by a ±500 PWM offset from 1500. Must be `0 < scale < 90°` and cover roll/pitch limits. | Larger scale produces smaller PWM offsets for the same angle. Must match flight-controller decoding. |
| `rc_yaw_rate_scale_rad_s` | 2 rad/s | Yaw rate represented by a ±500 PWM offset; positive and at least the yaw-rate limit. | Larger scale produces smaller PWM offsets for the same yaw rate. Must match decoding. |
| `target_altitude_m` | Bullet: 0 m | Target altitude used by **descent-feasibility supervision**. Any finite value; same altitude datum as the barometer. | Raising it reduces the descent considered necessary, making feasibility easier. Lowering it makes feasibility harder. **It does not directly command a descent or landing.** |

## 3. Approach, angle and speed limits

| Parameter | Default | Purpose and limits | Increase / decrease |
| --- | --- | --- | --- |
| `approach_pitch_rad` | 7° | Forward pitch during tracking, multiplied by `approach_scale`. Positive; cannot exceed pitch or combined-tilt limits. | Higher requests stronger forward tilt and generally faster closing. Lower gives a gentler approach. Zero is currently rejected. |
| `roll_limit_rad` | 15° | Maximum absolute commanded roll; `0 < limit < 90°`, also within RC scale. | Higher permits stronger lateral correction; lower reaches saturation sooner. |
| `pitch_limit_rad` | 30° | Maximum absolute commanded pitch; `0 < limit < 90°`, also within RC scale. | Higher permits more pitch when other limits allow it; lower constrains forward tilt. |
| `combined_tilt_limit_rad` | 30° | Overall tilt bound; `0 < limit < 90°`. Also triggers recovery when measured tilt exceeds it. | Higher permits steeper flight and delays over-tilt recovery. Lower restricts roll/pitch combinations and triggers recovery earlier. |
| `roll_slew_rad_s` | 15°/s | Maximum roll-reference change rate; `> 0`. | Higher gives faster correction; lower gives smoother, slower response. Roll and pitch slew are coupled, so either can slow both. |
| `pitch_slew_rad_s` | 5°/s | Maximum pitch-reference change rate; `> 0`. | Higher starts/stops the approach faster; lower makes transitions slower, including recovery. |
| `max_descent_m_s` | 5.5 m/s | Maximum downward speed reference; `> 0`. Also used in feasibility prediction. | Higher permits faster descent and makes predicted descent easier to achieve. Lower can cause earlier approach reduction. |
| `max_climb_m_s` | 3 m/s | Maximum upward speed reference; `> 0`. | Higher permits stronger climb correction. Lower clips upward demand sooner. The app's preceding hold separately caps climb at 1 m/s. |
| `vertical_slew_m_s2` | 1 m/s² | Maximum vertical-speed-reference change rate; `> 0`. Also models descent acceleration for feasibility. | Higher responds faster and predicts more descent capability. Lower smooths changes but makes descent feasibility harder. |

The combined-tilt limit is applied geometrically: available pitch decreases as
roll increases. These are reference limits, not guarantees that the vehicle will
achieve the requested angle, velocity or acceleration.

## 4. Controller gains and integral limits

Higher gains generally produce stronger corrections, but the response remains
bounded by speed, angle, slew and thrust limits. Excessive gains can cause
oscillation or persistent saturation.

| Parameter | Default | Purpose and limits | Increase / decrease |
| --- | --- | --- | --- |
| `yaw_gain_s_inv` | 4 s⁻¹ | Held-heading error → yaw-rate request; `≥ 0`. | Higher corrects heading faster; lower more slowly. Zero removes heading-error correction. |
| `yaw_rate_limit_rad_s` | 2 rad/s | Maximum absolute yaw-rate request; positive and within RC yaw scale. | Higher permits faster rotation; lower clips yaw correction sooner. |
| `hold_altitude_gain_s_inv` | 1 s⁻¹ | Hold-altitude error → vertical-speed reference; `> 0`. | Higher returns toward held altitude faster; lower more gently. Used in hold/acquisition/recovery. |
| `elevation_kp_m_s_rad` | 40.107 m/s/rad | Elevation error → vertical-speed correction; `≥ 0`. Equivalent to 0.7 m/s per degree. | Higher reacts more strongly to LOS elevation error; zero removes this term. |
| `elevation_kd_m_rad` | 14.324 m/rad | Elevation angular rate → vertical-speed correction; `≥ 0`. Equivalent to 0.25 m/s per degree/s. | Higher reacts more strongly to LOS motion, including noisy rate estimates; zero removes this term. |
| `elevation_ki_m_s2_rad` | 14.324 m/s²/rad | Accumulates persistent elevation error; `≥ 0`. | Higher removes persistent error faster but can increase overshoot. Zero stops new accumulation. |
| `vertical_memory_limit_m_s` | 3 m/s | Bounds the elevation-loop integral contribution; `> 0`. | Higher permits more sustained vertical correction; lower limits integral influence. |
| `azimuth_kp` | 0.8 | Azimuth error → roll correction; `≥ 0`. | Higher gives stronger lateral correction for angular displacement; zero removes this term. |
| `azimuth_kd_s` | 4 s | Azimuth angular rate → roll correction; `≥ 0`. | Higher reacts more strongly to bearing motion and rate noise; zero removes this term. |
| `azimuth_ki_s_inv` | 0.05 s⁻¹ | Accumulates persistent azimuth error into roll correction; `≥ 0`. | Higher removes persistent error faster but can increase overshoot. Zero stops new accumulation. |
| `lateral_memory_limit_rad` | 5° | Bounds the azimuth-loop integral contribution; `> 0`. | Higher permits more sustained roll correction; lower reduces integral authority. |
| `vertical_kp_n_s_m` | 1 N·s/m | Vertical-speed error → thrust correction; `≥ 0`. | Higher corrects vertical-speed error more strongly; lower responds more weakly. |
| `vertical_ki_n_m` | 0.4 N/m | Accumulates vertical-speed error into thrust correction; `≥ 0`. | Higher compensates sustained vertical error faster; lower more slowly. Zero stops new accumulation. |
| `vertical_integral_limit_n` | 5 N | Maximum absolute thrust-integral contribution; `> 0`. | Higher permits more compensation for persistent thrust mismatch; lower limits that compensation. |

For tracking, positive elevation error contributes a **more downward** speed
request. Positive azimuth error contributes a **negative roll** correction.
Continuity biases are seeded on entry to tracking, so a gain's full effect is not
necessarily visible immediately. Integral updates include anti-windup checks
when commands are limited.

## 5. Filtering and sensor alignment

| Parameter | Default | Purpose and limits | Increase / decrease |
| --- | --- | --- | --- |
| `barometer_alpha` | 0.35 | Altitude residual correction; `0 < α ≤ 1`. | Higher follows altitude measurements more closely, including noise. Lower smooths more but responds slower. |
| `barometer_beta` | 0.05 | Altitude residual correction applied to estimated vertical speed; `0 < β ≤ 1`. | Higher adapts velocity faster but amplifies measurement noise, especially at short sample intervals. |
| `vertical_warmup_s` | 0.1 s | Minimum barometer sample-history span before readiness; `> 0`. At least three samples are also required. | Higher delays command availability; lower starts sooner with less history. |
| `bearing_filter_s` | 0.15 s | Bearing-filter time constant; `> 0`. | Higher smooths bearing/rate estimates more but responds slower. Lower reacts faster with more noise sensitivity. |
| `attitude_history_s` | 0.5 s | Stored attitude-history horizon; positive and at least image-age/alignment-gap settings. | Higher supports older image alignment, subject to the fixed 512-sample capacity. Lower retains less history. |
| `max_alignment_gap_s` | 0.05 s | Maximum gap between attitudes used for image-time interpolation; positive and no greater than history horizon. | Higher accepts wider interpolation gaps with potentially less accurate alignment. Lower rejects more images. |

An exact timestamp match does not need interpolation. The library does not
extrapolate attitude beyond its stored endpoints. A pending image can wait for
its right interpolation endpoint, bounded by image freshness.

## 6. Freshness and command deadlines

| Parameter | Default | Purpose and limits | Increase / decrease |
| --- | --- | --- | --- |
| `max_image_age_s` | 0.1 s | Maximum usable image age; positive and no greater than attitude history. Also bounds tracking command validity and accepted-frame gaps. | Higher tolerates more image delay/loss but uses older visual information. Lower causes earlier visual rejection and recovery. |
| `max_attitude_age_s` | 0.05 s | Maximum latest-attitude age; `> 0`. | Higher tolerates more delay but uses older attitude. Lower causes earlier sensor faults after readiness. |
| `max_barometer_age_s` | 0.2 s | Maximum latest-barometer age; `> 0`. | Higher tolerates more delay but relies longer on prediction. Lower causes earlier sensor faults after readiness. |
| `max_control_gap_s` | 0.1 s | Maximum enabled control-update interval; `> 0`. | Higher tolerates longer scheduling gaps. Lower faults sooner when updates are late. |
| `command_validity_s` | 0.05 s | Maximum lifetime of each generated command; `> 0`. | Higher permits longer reuse **only if other deadlines allow it**. Lower requires faster replacement. |

Before initial sensor readiness, missing/stale sensors cause warmup or acquisition
timeout. **After readiness, stale attitude or barometer data latches `FAULT`.**

A command expires at the earliest applicable deadline:

- `now_s + command_validity_s`;
- latest attitude timestamp plus `max_attitude_age_s`;
- latest barometer timestamp plus `max_barometer_age_s`;
- acquisition deadline during acquisition;
- last usable image timestamp plus `max_image_age_s` during tracking;
- recovery deadline during recovery.

## 7. Acquisition and recovery timing

| Parameter | Default | Purpose and limits | Increase / decrease |
| --- | --- | --- | --- |
| `acquisition_dwell_s` | 0.1 s | Accepted target observations must span this interval before tracking. Positive and shorter than acquisition timeout. At least three frames are required. | Higher delays tracking and requires longer consistency; lower starts sooner. |
| `acquisition_timeout_s` | 5 s | Maximum initial acquisition time, including cold sensor warmup; greater than acquisition dwell. | Higher waits longer before faulting; lower fails sooner. |
| `recovery_dwell_s` | 0.1 s | Accepted observations must span this interval to resume tracking. Positive and shorter than recovery timeout. At least three frames are required. | Higher makes reacquisition slower; lower resumes sooner. |
| `recovery_timeout_s` | 1 s | Maximum recovery duration before fault; greater than recovery dwell. | Higher permits longer altitude/heading hold while reacquiring; lower faults sooner. |
| `limitation_timeout_s` | 0.75 s | Duration of continuous authority limitation/descent infeasibility before recovery; `> 0`. Also sets forward-pitch reduction rate. | Higher reduces pitch more slowly and tolerates limitation longer. Lower reduces pitch faster and enters recovery sooner. |

During continuous limitation:

```text
approach_scale = max(0, 1 - elapsed_limitation / limitation_timeout)
pitch_request = approach_pitch × approach_scale
```

Pitch still obeys slew limits. If limitation clears, the requested scale returns
to one. With default 7° approach pitch, scales of 1, 0.5 and 0 request 7°, 3.5°
and level pitch respectively.

Authority supervision checks prior angle, vertical-speed and thrust limitation,
plus vertical references at speed bounds. A slew-limit flag alone does not
trigger this supervision.

## 8. Target acceptance limits

| Parameter | Default | Purpose and limits | Increase / decrease |
| --- | --- | --- | --- |
| `max_elevation_rad` | 60° | Maximum absolute world LOS elevation; `0 < value < 90°`. | Higher accepts steeper target directions; lower rejects them sooner. |
| `max_azimuth_rad` | 35° | Maximum absolute world LOS azimuth relative to held heading; `0 < value < 90°`. | Higher accepts targets farther sideways; lower narrows acceptance. |
| `max_los_deviation_rad` | 12° | Maximum combined elevation/azimuth deviation from the saved LOS reference; `> 0`. | Higher tolerates more departure from the acquired direction; lower causes earlier rejection/recovery. |
| `max_association_jump_rad` | 12° | Maximum combined angular jump from the previous accepted raw bearing; `> 0`. | Higher tolerates faster movement or detection jumps, but accepts larger identity mistakes. Lower rejects more jumps. |

The last two use the magnitude of wrapped elevation/azimuth differences. They
are **angular limits, not pixel distances**. The association limit compares
consecutive accepted observations without dividing by elapsed time.

Elevation is positive down; azimuth is positive left relative to held heading.
The navigator preserves the acquired world LOS rather than independently
driving target pixels to image centre.

## 9. Optical TTC and descent supervision

| Parameter | Default | Purpose and limits | Increase / decrease |
| --- | --- | --- | --- |
| `ttc_window_s` | 0.5 s | Recent bbox-size history used for fitting; positive and at least minimum span. History holds at most 256 samples. | Higher includes more history, generally smoothing noise but retaining older motion. Lower adapts faster but can lose qualification. |
| `ttc_min_span_s` | 0.2 s | Minimum observation time span for a fit; positive and no greater than window. | Higher delays qualification and requires longer history; lower permits earlier estimates. |
| `ttc_min_growth_px` | 1 px | Minimum increase in `sqrt(width × height)` between first and last samples; `> 0`. | Higher rejects weak growth more aggressively. Lower accepts smaller growth that is easier to confuse with noise. |
| `ttc_max_relative_residual` | 0.15 | Maximum normalized RMS error of reciprocal-size fit; `> 0`, no enforced upper bound. | Higher accepts noisier fits; lower requires closer model agreement and makes TTC unavailable more often. |
| `ttc_enter_s` | 8 s | TTC-stage entry threshold after dwell. Must satisfy `terminal < enter < exit`. | Higher allows entry earlier in the approach; lower delays it. |
| `ttc_exit_s` | 10 s | Returns a nonterminal stage to `BEARING` when TTC reaches/exceeds it. Must exceed entry threshold. | Higher widens hysteresis and retains TTC stage longer; lower exits more readily. |
| `ttc_dwell_s` | 0.3 s | Qualification dwell for entering TTC stage; `> 0`. | Higher delays stage entry; lower responds faster. Terminal entry bypasses this dwell. |
| `terminal_ttc_s` | 0.8 s | Immediate terminal-stage threshold; positive and below entry threshold. | Higher latches terminal earlier; lower later. This does not signal collision or stop commands. |
| `ttc_time_margin_s` | 0.1 s | Subtracted from TTC before calculating available descent time; `≥ 0`. | Higher makes feasibility more conservative, causing earlier slowing/recovery. Lower assumes more time remains. |
| `feasibility_height_margin_m` | 0.25 m | Added to predicted available descent; `≥ 0`. | **Higher makes feasibility more permissive**, allowing a larger predicted descent shortfall. Lower makes it stricter. |

TTC fits reciprocal apparent size rather than directly fitting width or height:

```text
apparent_size = sqrt(width × height)
reciprocal_size = intercept + slope × relative_time
TTC = -remaining_reciprocal_size / slope
```

Important behavior:

- TTC needs **at least eight accepted samples**, regardless of window/span settings.
- A qualified fit also needs sufficient growth, a negative slope and acceptable residual.
- Unavailable TTC does not itself stop bearing tracking.
- **Descent feasibility runs whenever TTC is qualified**, including in `BEARING`; it does not wait for `TTC` stage.
- `BEARING`, `TTC` and `TERMINAL` do not select different controller gains.
- `TERMINAL` remains latched until reset.
- Optical TTC is not measured distance, collision detection or a landing controller.

The height-margin direction follows the actual feasibility comparison:

```text
max(0, altitude - target_altitude) ≤ predicted_max_drop + feasibility_height_margin
```

## 10. Runtime inputs and method parameters

```python
result = navigator.update(
    inputs,
    now_s=now_s,
    enabled=True,
    command_horizon_s=0.0,
)
```

| Parameter / field | Allowed use | Expected behavior when changed |
| --- | --- | --- |
| `inputs.target` | `TargetObservation` or `None`. | `None` supplies no new observation and does not refresh image age. A new observation updates visual processing. |
| `inputs.barometer` | `BarometerSample` or `None`. | A new sample updates vertical estimation. Missing samples eventually reach freshness limits. |
| `inputs.attitude` | `AttitudeSample` or `None`. | A new sample updates current attitude and image-alignment history. Missing samples eventually reach freshness limits. |
| `stream_id` | Nonempty string identifying each sensor stream. | Changing it during a session latches a stream-change fault; reset for a new stream. |
| `sequence` | Nonnegative integer, increasing for new samples. | Exact repeats do not update filters. Older samples are ignored. Reusing the last identity with changed contents raises `ValueError`. |
| `timestamp_s` | Finite measurement time, in the control clock domain; cannot be in the future. New samples must advance measurement time. | Older timestamps increase age and require older attitude alignment. Incorrect timestamps corrupt alignment/freshness behavior. |
| `bbox` | `(x,y,width,height)` in pixels. Nonnegative origin, positive size, inside image. May be `None`. | Centre changes LOS; size changes TTC. `bbox=None` explicitly reports no target. Touching an image edge causes clipped-target rejection. |
| `altitude_m` | Finite altitude in the configured datum. | Updates filtered altitude/speed, hold error and descent feasibility. A datum mismatch gives incorrect supervision. |
| `orientation_xyzw` | Unit body-to-world XYZW quaternion, validated as in camera calibration. | Changes image-to-world LOS conversion, heading error, measured tilt and thrust tilt compensation. |
| `now_s` | Finite control time, strictly increasing. | Larger tick intervals increase filter prediction and controller step duration; excessive gaps fault. Reset before changing clocks. |
| `enabled` | Boolean navigation authorization. | `True` starts/continues navigation. `False` produces no command. Disabling an active session requires reset before re-enabling. It does not arm the drone. |
| `command_horizon_s` | Finite, `≥ 0`; default zero. App supplies 0.02 s. | Higher reserves more image-validity budget and rejects images earlier with `image_deadline`. It never extends freshness or expiry. If too large, tracking may never acquire. |

New samples must advance both sequence and measurement timestamp within their
stream. Disabled updates do not ingest samples. Malformed input and clock
violations raise `ValueError`; operational loss of data is represented through
navigation state and reasons.

## 11. Hold handoff parameters

The caller owns takeoff, hover, yaw search and target centering. These parameters
support the transition from that app-owned hold into LOS navigation.

| Parameter / call | Meaning and effect |
| --- | --- |
| `hold.set_heading(centered_heading_rad)` | Selects the finite heading in radians to hold after app centering. Changing it changes the yaw-hold reference. |
| `hold.handoff(now_s=..., target_altitude_m=hold_goal_m)` | Supplies handoff time and the **hold-altitude goal**. Unlike `NavigatorConfig.target_altitude_m`, this altitude controls the hold reference before/during acquisition. Time and sensor state must satisfy handoff validation. |
| `navigator.reset(handoff=...)` | Restores compatible warmed sensor/controller state. Configuration must match. Without a handoff, reset starts cold. Visual acquisition is still required. |
| `hold.command(..., yaw_rate_rad_s=...)` | Overrides heading-derived yaw rate for app search/centering. A supplied override must be finite and within the configured yaw-rate limit. |

Without a handoff, the first ready attitude establishes held heading and the
filtered altitude establishes initial hold altitude. Recovery captures current
altitude as its hold target while preserving the acquired LOS reference and held
heading.

## 12. Returned values and command use

Returned fields are observations, not tuning parameters.

| Field | Meaning / usage |
| --- | --- |
| `sequence` | Increasing result number, separate from sensor sequences. |
| `generated_at_s` | Control time that generated this result. |
| `valid_until_s` | Command-use deadline. Stop using the command when this time is reached. |
| `state` | `DISABLED`, `ACQUIRE`, `TRACK`, `RECOVER` or `FAULT`. A fault stays latched until reset. |
| `stage` | `BEARING`, `TTC` or `TERMINAL`; describes TTC supervision. |
| `reason` | Machine-readable explanation of current navigation behavior or failure. |
| `command` | Four integer RC channels, or `None` when unavailable. Each channel is in `1000–2000`. |
| `diagnostics` | Estimates, validity flags, rejection reasons, control references and limit indicators. Numeric estimates can remain present after becoming invalid. |
| `usable` | True when `command is not None`; does not check current time against the deadline. |

Check both command availability and expiry:

```python
if result.usable and now_s < result.valid_until_s:
    roll, pitch, throttle, yaw = result.command.as_tuple()
```

Channels use **AETR order**: roll, pitch, throttle, yaw. Roll/pitch/yaw are centred
at 1500. Throttle maps zero-to-maximum thrust onto 1000–2000. Integer rounding
means arbitrary sub-count reference changes cannot be represented exactly.

The caller owns command transport, arming, flight mode and auxiliary channels.
The package does not send packets, infer collision or implement landing.

## Sources and verification

Reviewed against:

- [Package control flow](package-control-flow.md)
- [Configuration and validation](../src/los_tracking/config.py)
- [Bullet profile](../examples/bullet_profile.json)
- [Controller and RC encoding](../src/los_tracking/control.py)
- [Vertical, bearing and TTC estimators](../src/los_tracking/estimation.py)
- [Navigation lifecycle and supervision](../src/los_tracking/navigator.py)
- [Geometry and attitude history](../src/los_tracking/geometry.py)
- [Input/output contracts](../src/los_tracking/types.py)
- [Hold and handoff](../src/los_tracking/hold.py)

This is a source-based explanation. No flight simulations or hardware checks
were performed for this document. Configuration values accepted by validation
are not necessarily stable or physically achievable. Gains are simulation
starting points; calibration, vehicle values and RC scales must match the actual
system.
