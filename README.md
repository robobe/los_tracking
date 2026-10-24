# LOS tracking

`los_tracking` 0.2 is a pure-Python, stateful **bbox + barometric altitude + attitude
→ four Angle-mode RC channels** library. It performs an already-airborne approach
toward one stationary target. It preserves the acquired **world LOS bearing** and
initial heading, correcting elevation with vertical motion and azimuth with roll.
Both bearing error and **rotation-compensated LOS rate** enter the control law.

This project belongs to the [Bullet workspace](../README.md), sharing `../.venv`
and `../uv.lock`. Core runtime dependencies are empty: no OpenCV, NumPy, PyBullet,
Godot, sockets or background threads. The former `detect_red_box` API and its tests
moved to `bullet_app.detector`; the live app's detector behavior is unchanged.
Simulation tests use `bullet-drone` as a development dependency only.

## Calling contract

```python
from los_tracking import (
    AttitudeSample, BarometerSample, LosNavigator, NavigationInput,
    NavigatorConfig, TargetObservation,
)

config = NavigatorConfig.from_json("examples/bullet_profile.json")
navigator = LosNavigator(config)

# Repeated on the application's control tick; values are supplied by its adapters.
# None means no NEW sample, not a renewed measurement of the previous value.
inputs = NavigationInput(
    target=TargetObservation("camera-session", 42, 12.30, (310, 280, 20, 20)),
    barometer=BarometerSample("barometer-session", 615, 12.31, 10.0),
    attitude=AttitudeSample("attitude-session", 1231, 12.31, (0, 0, 0, 1)),
)
now_s = 12.31
result = navigator.update(inputs, now_s=now_s, enabled=True)
if result.usable and now_s < result.valid_until_s:
    roll, pitch, throttle, yaw = result.command.as_tuple()
    # The application may apply these only before result.valid_until_s,
    # in the configured Angle mode. It owns arm/mode/auxiliary channels.

# No target in a NEW image is explicit:
no_target = TargetObservation("camera-session", 43, 12.34, None)

# End this mission (e.g. a future app collision notification), then reset to restart.
result = navigator.update(NavigationInput(), now_s=12.35, enabled=False)
navigator.reset()
```

The example first call normally returns no command: the altitude estimator needs
at least three new samples spanning 0.1 s. The app retains stabilization while
waiting. ACQUIRE may return level/altitude-hold commands once sensor state is ready,
before target acquisition completes. Always check `result.usable` and expiry;
inactive results contain `command=None`, never a neutral-throttle fallback.

Use one owner thread and a common measurement/control clock in seconds. Sample
identities are `(stream_id, sequence)`; stream changes require reset. Duplicate
samples do not renew age, advance filters or satisfy visual dwell. Older sequences
or timestamps are ignored and reported; reusing an ID with different data raises
`ValueError`. All input shapes, numbers, time ordering and image bounds are checked
before an update mutates state. Control time must strictly increase; reset permits
a new time origin. Expected missing/stale operational data return status results.

Attitude is a unit **body-to-world XYZW quaternion**. Body/world axes are
right-handed, Z up; body X forward, Y left. Bbox coordinates use the calibrated
image, x right/y down, top-left plus positive width/height. Supply undistorted,
uncropped coordinates or configure the corresponding intrinsics; this core does
not correct lens distortion or detector letterboxing. The mount quaternion rotates
an aligned FLU camera ray into body FLU: aligned camera forward is body +X, image
right is −Y and image down is −Z. Attitude history interpolates with SLERP; no
extrapolation or additional gyro compensation is applied.

## Estimation and control

Bearing elevation is positive downward; azimuth is positive left relative to the
saved initial heading. Both are radians, rates are rad/s. Each new exposure is
rotated into the same world frame before a time-aware alpha-beta filter estimates
bearing and rate. First-frame zero rate is not observable; diagnostics distinguish
bearing validity from rate availability. Attitude history holds at most 512 samples
and TTC history at most 256, additionally bounded by their configured time windows.

A barometer-only constant-velocity alpha-beta filter estimates altitude and vertical
speed (m/s, positive up). It uses elapsed measurement time and predicts only while
the altitude sample remains fresh. There is no acceleration input, horizontal
velocity estimate, metric range estimate or simulator truth in the public API.

```text
vz_request = vz_bias − Kp_z * elevation_error − Kd_z * elevation_rate + memory_z
roll_request = roll_bias − Kp_r * azimuth_error − Kd_r * azimuth_rate + memory_r
```

Bounded correction memory persists when angular error becomes zero. Anti-windup
accounts for angle/vertical constraints and reference slew. Vertical PI feedback
produces thrust around `mass * gravity`, compensated using measured attitude and
clamped to configured authority. Initial heading error becomes a bounded body-Z
yaw-rate request, using the same upright approximation as Bullet's heading adapter.
Forward pitch is configurable and slewed; it is not a ground-speed command.

For optical TTC, fit reciprocal apparent size `1/sqrt(width*height)` over a short
window: under a local constant-depth-closing model it is linear. Require >=8 samples,
minimum span/growth, positive closing, acceptable relative fit residual and an
unclipped target. Predict the fitted depth proxy to control time and expire it;
do not hold a frozen TTC indefinitely. Optical TTC is not guaranteed collision time,
especially under foreshortening, acceleration or changing silhouette.

TTC supervises BEARING/TTC/TERMINAL timing and descent feasibility against configured
target altitude, vertical speed/slew bounds and margins. Feasibility is a model-based
check, not proof of available physical authority. Qualified TTC entry/exit use
hysteresis; TERMINAL stays latched through TTC loss. The LOS law stays the same;
TTC loss alone neither resets learned correction nor authorizes blind continuation.

## Lifecycle and command meaning

| State | Behavior |
| --- | --- |
| DISABLED | No usable command. First enable starts acquisition; disabling an active mission requires reset before another enable. |
| ACQUIRE | Warm sensors, retain initial heading/altitude, require >=3 usable visual frames and acquisition dwell. Save the first accepted LOS in the completed dwell. |
| TRACK | Follow original LOS; retain reference through TTC stage changes and recovery. Never infer collision/completion from bbox size or altitude. |
| RECOVER | Slew forward pitch/roll toward level, retain heading and request recovery-entry altitude. Only usable sensors permit these bounded commands. Reacquisition needs dwell and preserves the original LOS. |
| FAULT | No command until reset, after required sensor loss, stream restart, excessive control gap, numeric failure or acquisition/recovery timeout. |

Missing/invalid target geometry, clipping, abrupt association shifts, excessive
LOS deviation or measured over-tilt cause recovery. Persistent angle/vertical/thrust
limitations or TTC infeasibility moderate pitch first, then enter recovery.
Reacquisition reseeds biases for a continuous handoff. Commanded combined tilt is
`acos(cos(roll)*cos(pitch))`: retain roll when feasible, reduce pitch first, and
coordinate both slew rates with one interpolation fraction. Measured over-tilt is
reported separately and freezes thrust integral growth during recovery.

Four integer channels, in **roll, pitch, throttle, yaw** order:

- Roll/pitch: 1500 centre; 1000/2000 encode configured negative/positive angle scales.
- Throttle: 1000 is zero thrust; 2000 is configured maximum thrust. Hover is vehicle-specific.
- Yaw: 1500 centre; 1000/2000 encode negative/positive body-Z rate, not heading.

References obey slew bounds exactly. PWM rounding has one-count resolution;
sub-count reference changes cannot be represented on every tick. RC conversion
retracts rounding that would cross individual/combined angle or yaw-rate bounds.
The app must match these scales, add arm/mode/auxiliaries, and reject expired
commands with an independent receiver watchdog. This library opens no transport.

## Configuration and starting profile

`examples/bullet_profile.json` contains every tuning value for reproducibility.
Platform fields are explicit: intrinsics and mount, mass, maximum thrust, RC scales,
and target altitude in the same barometric datum. Barometric altitude is not AGL.
The profile uses a 1 kg vehicle and 33.953391 N maximum thrust from the bundled
Bullet racer, ideal 640×480 / 90° horizontal FOV, and flat target datum 0 m.
These are simulation examples, not inferred calibration or hardware recommendations.

Defaults start at 7° approach pitch, 15° roll and 30° total tilt; roll/pitch slew
15°/s and 5°/s; climb/descent bounds 3/5.5 m/s; vertical-reference slew 1 m/s².
Upstream vertical LOS gains expressed per degree are converted to radians.
Camera age is limited to 0.1 s, attitude 0.05 s, barometer 0.2 s, control gaps 0.1 s,
and command validity 0.05 s. Acquisition/recovery dwell is 0.1 s, with 5 s/1 s
timeouts. Current configuration is frozen: construct a new navigator to change limits.

No part of the profile promises braking, stationary hover, obstacle avoidance,
physical contact or visibility through contact. Level pitch does not stop existing
horizontal motion. Near-contact clipping invokes recovery; collision reporting
and the truck's missing physical collision geometry remain separate work.

## Checks and integration status

```bash
uv sync --locked --all-packages --all-extras --all-groups
uv run --locked --all-packages --all-extras --all-groups pytest -q
uv build --package los-tracking
```

Tests cover pure/combined rotation, wrapped angles, timing, bounded histories,
barometer-only speed estimation/noise, optical expansion qualification, persistent
LOS memory, anti-windup, exact total tilt, RC codec interoperability, lifecycle and
closed-loop approaches. Hidden truth generates observations and scores outcomes
only. The synthetic contact-volume oracle disables guidance externally; it is
not a collision detector in the library. Headless PyBullet tests exercise actual
Angle-mode RC decoding, stabilization and sensor observations.

Live `bullet_app` currently runs image detection/MCAP publication only. It has no
barometer/attitude receiver or RC sender. Current renderer image metadata does not
carry exact source pose/sensor association, and its `capture_ns` is pose-arrival
time rather than a proven exposure timestamp. Do not feed that metadata as exact
capture-aligned sensor data without the deferred integration work.

Reference: [navigation guide](https://github.com/mayaboker/fly_smart/blob/main/docs/los/drone_visual_navigation.md)
and [standalone design](https://github.com/mayaboker/fly_smart/blob/main/docs/los/standalone_los_library_design.md),
source revision `6cce117761b84c78c068185f291527dbb00c5481`. This implementation keeps
the original-LOS contact behavior but adds altitude-history vertical estimation,
explicit RC conversion, combined tilt enforcement, sample identity checks and
qualified reciprocal-size TTC. See [implementation plan](docs/implementation-plan.md).

## Altitude/heading hold and handoff

`AltitudeHeadingHold(config, max_climb_m_s=1.0)` shares the LOS vertical estimator,
bounded attitude history and control law. Feed timestamped `NavigationInput`
sensors through `observe()`, check `ready(now_s)`, then call
`command(now_s=..., target_altitude_m=...)`. It produces four AETR channels;
the caller owns arming, mode, transport and all other channels. It consumes no XY
position or velocity. `estimate(now_s)` reports filtered altitude/vertical speed.

After hold, use `navigator.reset(handoff=hold.handoff(now_s=...,
target_altitude_m=...))`, then advance at a strictly later monotonic time.
`ControlHandoff` is an immutable, validated snapshot tied to the same navigator
profile. It preserves filtered vertical state, recent attitude samples, heading,
hold target, reference roll/pitch/vertical speed and thrust integral. The new
navigator still requires its normal fresh visual acquisition dwell; handoff does
not authorize arming or bypass stale-sensor checks. Plain `reset()` is unchanged.

## Bearing envelope diagnostics

Default azimuth acceptance is ±35 degrees (`max_azimuth_rad`); elevation remains
±60 degrees. The Bullet example profile uses the same limits. A tiny numerical
tolerance preserves inclusive boundary acceptance. Other control limits and
acquisition timeout behavior remain unchanged.

`Diagnostics.rejected_azimuth_rad` / `rejected_elevation_rad` retain the latest
out-of-envelope observation while that rejection is current, alongside
`max_azimuth_rad` / `max_elevation_rad`. These are unfiltered rejected inputs,
separate from valid bearing estimates. They are cleared on a usable observation,
a different visual rejection or reset. Timeout diagnostics retain the rejection
cause so applications can explain the failed acquisition. A stale last accepted
sample does not overwrite a newer rejection with a generic stale-image cause.

For caller-owned target search, `hold.command(..., yaw_rate_rad_s=...)` replaces
heading feedback with an explicit bounded yaw rate while preserving altitude
control. Omitting the keyword restores heading hold. `hold.set_heading(rad)`
changes the hold/handoff heading without resetting filters or thrust integral;
it validates and wraps the supplied angle. `hold.attitude_at(timestamp_s)`
provides interpolation from bounded history with no extrapolation. Overrides
are validated before controller state changes and cannot exceed the profile's
yaw-rate limit.

### Visual decision diagnostics

Navigation results retain the last processed target's stream/sequence/capture time,
bbox, signed left/top/right/bottom edge margins (pixels) and touched edge names.
These fields identify the input associated with rejection/recovery, including
`clipped_target`; they do not change target acceptance or control outputs. App
telemetry archives them for exact MCAP frame matching. `box_edges` is the shared
pure geometry helper used by mission prequalification and navigation diagnostics.
