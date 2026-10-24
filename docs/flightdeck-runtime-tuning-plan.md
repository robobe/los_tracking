# FlightDeck: live MAVLink tuning application

Status: planned, not implemented.

## Summary

Create **FlightDeck**, a separate Qt desktop application for tuning the running
`bullet_app`. Version 1 provides live telemetry, fetching and editing every existing
LOS `Tuning` field, local parameter files, and explicit persistent storage on the
application host.

Use standard MAVLink 2 over configurable UDP. Apply writes parameters individually
and reports partial success; MAVLink does not provide an atomic parameter batch.
Reference: [MAVLink parameter protocol](https://mavlink.io/en/services/parameter.html).

Physics, sensor-noise settings, camera calibration, vehicle properties, mission
constants and direct flight controls remain outside version 1.

## Parameter ownership and runtime behavior

- `bullet_app` owns authoritative active tuning. FlightDeck fetches confirmed
  values rather than assuming its edits succeeded.
- Add a shared parameter catalog in `bullet_msgs`: stable unique MAVLink IDs of
  at most 16 characters, full configuration names, descriptions, units and
  live-update eligibility. Test that every `Tuning` field appears exactly once.
- Keep JSON names and SI units unchanged. Display angles in degrees where helpful,
  converting at the UI boundary.
- Support these changes during an active mission: controller gains, approach
  pitch, speed and attitude limits, slew rates, and controller integral bounds.
- All other tuning fields—including filter settings, history windows, freshness
  checks and state-transition thresholds—are editable only between episodes.
  Reject active writes with an explanation; do not silently queue them.
- Validate each candidate against the complete `NavigatorConfig` and existing
  mission constraints before applying it.
- Reject an active limit reduction when current references or stored controller
  state exceed the proposed bounds. Preserve controller state on accepted
  updates; do not restart tracking or reset integrators.
- Apply accepted changes immediately before a control tick, updating the mission,
  hold controller, navigator and their nested configuration references
  consistently. Acknowledge only after application.
- Disconnecting FlightDeck leaves the controller running with its last confirmed
  tuning.

## MAVLink and persistence

- Add an optional MAVLink interface to `bullet_app`, isolated from control-loop
  networking through bounded queues. Use `pymavlink`; retain existing internal
  ZeroMQ interfaces.
- Implement heartbeat, parameter list/read/set and `PARAM_VALUE` confirmation
  using `REAL32` values. Validate the decoded float32 value, reject non-finite
  values and return the unchanged value on rejected writes.
- Send bounded `STATUSTEXT` explanations for rejected changes. Advertise the
  application as an onboard-controller component, independently addressable from
  a future autopilot.
- FlightDeck sends one write at a time, waits for confirmation, and stops on
  rejection or exhausted retries. Retry after one second, up to three attempts.
  Preserve the visible list of already-applied changes; never claim rollback.
- Validate the final proposed configuration locally first. Send changes in
  catalog order, with an editable review order for coupled constraints. If an
  intermediate configuration is invalid, stop and explain which related value
  must change first.
- Separate **Apply** from **Save on drone**. Apply changes memory only. While
  inactive, `MAV_CMD_PREFLIGHT_STORAGE` saves the complete confirmed navigator
  profile atomically to the explicitly selected `--mission-config` file. A write
  failure returns failure and preserves the previous file.
- Local export saves a versioned JSON parameter snapshot. Import stages edits for
  review and Apply; it does not transmit automatically.
- Begin with localhost UDP, configurable for a trusted network. Actual autopilot
  integration and serial/radio transport are later work.

## Desktop and telemetry

- Create a `flight_deck` workspace project using the existing PySide6 and
  PyQtGraph stack, separate from FlightLens and Godot.
- Provide connection controls, component selection, Fetch, a searchable grouped
  parameter table, Apply, Revert staged edits, Import, Export and Save on drone.
- Show fetched value, staged value, units, eligibility,
  pending/confirmed/rejected status and unsaved changes. Disable edits when
  disconnected or synchronization is incomplete.
- Mark telemetry stale after three seconds without heartbeat and refetch
  parameters after reconnection. Require review before resending staged edits.
- Show mission phase and fault, sensor-derived altitude and vertical speed
  against references, attitude against references, thrust, and controller
  limitation flags.
- Use standard `ATTITUDE`, `NAMED_VALUE_FLOAT` and `STATUSTEXT` messages with
  documented stable channel names. Publish plots at 20 Hz and heartbeat at 1 Hz.
  Do not substitute simulation ground truth for missing flight estimates.
- Keep a two-minute live plot history. All plots share a time axis, with markers
  for confirmed parameter changes, phase transitions and faults.
- Publish parameter application and storage events through existing app telemetry
  for MCAP recording. Include old/new values, result, configuration revision and
  effective control-step timestamp. Update periodic configuration snapshots to
  reflect active tuning.

### Live plots

| Plot | Lines shown | Tuning purpose |
| --- | --- | --- |
| Altitude | Estimated altitude and hold target, when active | Altitude error and overshoot |
| Vertical speed | Estimated speed, requested speed and slewed reference | Acceleration limits and vertical response |
| Attitude | Measured and commanded roll/pitch | Response delay and oscillation |
| Thrust | Commanded thrust and maximum thrust | Saturation and braking authority |
| LOS error | Target elevation and azimuth errors | Tracking convergence and target loss |

The vertical-speed plot is especially useful for the recorded recovery failure:
it distinguishes an upward speed request from a slewed reference that still
commands descent. Ground-truth comparison and camera-image display are outside
version 1.

## Validation and acceptance

- Catalog coverage, unique IDs, units and float32 round-trip tests.
- Runtime tests proving changes reach every active controller and survive
  hold-to-LOS handoff without unintended state resets.
- Rejection tests for invalid values, coupled constraints, unsupported active
  changes and unsafe limit reductions.
- MAVLink integration tests covering fetch completeness, retries, duplicate
  writes, partial success, disconnect/reconnect and wrong target component.
- Persistence tests covering explicit save, restart loading, denied saves during
  flight and failed writes preserving the previous file.
- End-to-end simulation: change `vertical_slew_m_s2` from 1 to 3 during tracking,
  verify the reference ramp changes on the acknowledged tick, and verify MCAP
  timestamps match.
- Desktop checks covering staged edits, failed writes, local import/export and
  telemetry freshness.

No recovery-policy changes or real-hardware flight validation are included. Live
tuning can alter flight behavior immediately; this version proves simulator
integration, not hardware readiness.
