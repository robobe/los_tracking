# Bbox-to-RC implementation plan

Status: implemented and verified. Commit message: `los flow version1`.

## Agreed goal and boundary
Stateful, independent `LosNavigator(config).update(inputs, now_s=..., enabled=...)`
accepts timestamped bbox, barometric altitude and body-to-world XYZW attitude;
returns four 1000–2000 Angle-mode channels with expiry, state and diagnostics.
One stationary target; already airborne; original world LOS and heading retained.
The app owns arming, takeoff, mode, transport, fallback and future collision reporting.
No inferred horizontal speed/range, acceleration input, live RC transport or automatic contact detection.

## Steps
1. Move red-image detection and its tests into bullet_app; remove vision runtime dependencies from this library.
2. Add immutable validated contracts, configuration and independent camera/quaternion geometry.
3. Add bounded attitude history, bearing/rate and optical TTC estimation, and barometer-only vertical estimation.
4. Add original-LOS feedback, persistent bounded memory, vertical PI, heading-rate conversion,
   combined tilt allocation, coordinated slew, RC conversion and anti-windup.
5. Add acquire/track/recover/fault lifecycle, timing and quality gates, reset/disable semantics.
6. Add a documented simulation starting profile, app calling contract, deterministic closed-loop
   and failure tests, package isolation checks; update the shared lockfile.

## Reference and adaptations
Navigation guide: https://github.com/mayaboker/fly_smart/blob/main/docs/los/drone_visual_navigation.md
Standalone contact design: https://github.com/mayaboker/fly_smart/blob/main/docs/los/standalone_los_library_design.md
Reference source revision: 6cce117761b84c78c068185f291527dbb00c5481.
Port the original-LOS behavior, with radians throughout, altitude-history vertical speed,
explicit yaw-rate/thrust RC conversion, capture-time attitude interpolation and exact total tilt limits.
TTC supervises timing/descent feasibility; it does not replace the LOS law or declare contact.

## Verification
- los_tracking: 79 passing tests, including the final TTC hysteresis and exact-age boundary regressions.
- bullet_app: 23 passing tests after detector relocation, including TCP/SHM detection-to-MCAP readback.
- Full workspace check: bullet_msgs 69, mcap_bagger 65, bullet_drone 594,
  bullet_render 19 passed; LOS/app suites also passed in that run, followed by the final LOS reruns.
- All five Python wheels/source distributions build; the LOS package was rebuilt after final changes.
- Clean wheel installation exercised bbox-to-RC updates with no NumPy, OpenCV, PyBullet or pytest installed.
- Pure-rotation bearing tolerance 1e-12 rad; compensated rate tolerance 1e-10 rad/s.
  Simultaneous known translation/rotation rate tolerance 5e-4 rad/s.
- Constant vertical-speed estimate error <0.01 m/s after 300 irregular samples;
  noisy barometer (0.05 m standard deviation, seed 72) speed RMSE <0.5 m/s.
- Exact 15-degree roll / 30-degree requested pitch / 30-degree total-limit example
  matches 26.288483-degree pitch within 1e-6 degree. Quantized RC decoded bounds
  checked across 441 roll/pitch request combinations.
- Deterministic hidden plant approaches a target 40 m forward / 10 m below within
  a 0.25 m target-volume oracle, with zero and +/-0.15 m/s² lateral disturbances.
  The TEST caller disables on that oracle; no metric range/contact input enters guidance.
  Plant has 0.12 s attitude lag and 0.2/s horizontal damping; this is not a full wind model.
- Separate 4-second PyBullet test verifies real Angle-mode RC/stabilization with noisy
  barometer observations and an ideal attitude fixture. No Godot pixels or live command transport used.
- Whitespace, Python/JSON syntax and locked dependency synchronization checked.

## Limitations
No hardware validation, live sensor/RC wiring, physical target collision or guaranteed
braking/hover. Optical TTC assumes locally consistent projected target scale; the profile
is a simulation starting point. Current live renderer metadata needs an explicit timing
association design before it can meet this library's capture-time input contract.

## Implementation notes
Reciprocal bbox-size regression provides a qualified local optical-depth closing model;
altitude-history alpha-beta estimation replaces upstream acceleration fusion. Combined tilt
is enforced after PWM quantization as well as before coordinated slew. Continuous references
obey slew bounds; PWM has one-count resolution. Image detection moved to bullet_app as agreed.
The public API change is released locally as los-tracking 0.2.0. Changes committed as `los flow version1`; no push requested.
