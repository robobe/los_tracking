"""Caller-owned airborne acquire/approach/recover lifecycle; no transports or devices."""
from math import hypot, isfinite

from .config import NavigatorConfig
from .control import LosControl
from .estimation import BearingEstimator, TtcEstimator, VerticalEstimator, descent_feasible
from .geometry import AttitudeHistory, combined_tilt, euler, world_bearing, wrap
from .types import Diagnostics, NavigationInput, NavigationResult, State, TrackStage, finite


MIN_ACQUISITION_FRAMES = 3
TIME_EPSILON = 1e-9
BEARING_EPSILON_RAD = 1e-12


class LosNavigator:
    """Single-threaded, timestamped bbox/altitude/attitude to four Angle RC channels.

    `enabled` is navigation authorization, not arming. A disabled mission or
    latched fault requires reset before another enable. No contact is inferred.
    """
    def __init__(self, config: NavigatorConfig):
        if not isinstance(config, NavigatorConfig):
            raise ValueError("config must be NavigatorConfig")
        self._config = config
        self._sequence = 0
        self.reset()

    @property
    def config(self):
        return self._config

    def reset(self, *, handoff=None):
        t = self.config.tuning
        self.state = State.DISABLED
        self.stage = TrackStage.BEARING
        self._ended = False
        self._last_now = self._started_at = self._recovery_at = None
        self._fault_reason = ""
        self._last_samples = {}
        self._attitudes = AttitudeHistory(t.attitude_history_s, t.max_alignment_gap_s)
        self._vertical = VerticalEstimator(t)
        self._bearing = BearingEstimator(t.bearing_filter_s)
        self._ttc = TtcEstimator(t)
        self._control = LosControl(self.config)
        self._pending_target = None
        self._processed_target = None
        self._heading = self._reference = self._hold_altitude = None
        self._sensors_were_ready = False
        self._visual_usable = False
        self._vision_reason = "not_observed"
        self._rejected_bearing = None
        self._last_visual_time = None
        self._candidate_start = self._candidate_end = None
        self._candidate_reference = None
        self._candidate_count = 0
        self._ttc_entry_at = self._limited_at = None
        self._descent_feasible = None
        self._input_rejection = ""
        self._reason = "disabled"
        if handoff is not None:
            from .hold import restore_handoff
            self._vertical, self._attitudes, self._control = restore_handoff(handoff, self.config)
            self._last_samples = dict(attitude=handoff.attitudes[-1], barometer=handoff.barometer)
            self._heading, self._hold_altitude = handoff.heading_rad, handoff.hold_altitude_m
            self._last_now = handoff.timestamp_s
            self._sensors_were_ready = True

    def _validate_tick(self, inputs, now, enabled):
        if not isinstance(inputs, NavigationInput) or type(enabled) is not bool:
            raise ValueError("NavigationInput and boolean enabled required")
        finite(now, "now_s")
        if self._last_now is not None and now <= self._last_now:
            raise ValueError("control time must strictly increase; reset for a new clock")
        for name in ("attitude", "barometer", "target"):
            sample = getattr(inputs, name)
            if sample is None:
                continue
            if sample.timestamp_s > now + TIME_EPSILON:
                raise ValueError("measurement timestamp is in the future")
            previous = self._last_samples.get(name)
            if (previous is not None and sample.stream_id == previous.stream_id
                    and sample.sequence == previous.sequence and sample != previous):
                raise ValueError("sample identity reused with different data")
        if inputs.target is not None and inputs.target.bbox is not None:
            x, y, width, height = inputs.target.bbox
            camera = self.config.camera
            if x + width > camera.width_px or y + height > camera.height_px:
                raise ValueError("bbox exceeds calibrated image dimensions")

    def _fault(self, reason):
        if self.state != State.FAULT:
            self._fault_reason = reason
        self.state = State.FAULT
        self._reason = self._fault_reason

    def _accept(self, name, sample):
        if sample is None:
            return False
        previous = self._last_samples.get(name)
        if previous is not None:
            if previous.stream_id != sample.stream_id:
                self._fault(f"{name}_stream_changed")
                return False
            if sample.sequence <= previous.sequence or sample.timestamp_s <= previous.timestamp_s:
                if sample != previous:
                    self._input_rejection = f"{name}_out_of_order"
                return False
        self._last_samples[name] = sample
        return True

    def _clear_candidate(self):
        self._candidate_start = self._candidate_end = None
        self._candidate_count = 0
        self._candidate_reference = None

    def _reject_visual(self, reason):
        if reason != "bearing_outside_envelope":
            self._rejected_bearing = None
        self._visual_usable = False
        self._vision_reason = reason
        self._clear_candidate()
        self._ttc.invalidate(reason)

    def _process_target(self, now):
        sample = self._pending_target
        if sample is None:
            return
        self._processed_target = sample
        t = self.config.tuning
        if now - sample.timestamp_s > t.max_image_age_s + TIME_EPSILON:
            self._pending_target = None
            self._reject_visual("stale_image")
            return
        if sample.bbox is None:
            self._pending_target = None
            self._reject_visual("no_target")
            return
        x, y, width, height = sample.bbox
        camera = self.config.camera
        if x <= 0 or y <= 0 or x + width >= camera.width_px or y + height >= camera.height_px:
            self._pending_target = None
            self._reject_visual("clipped_target")
            return
        attitude = self._attitudes.at(sample.timestamp_s)
        if attitude is None:
            # A pending exposure may acquire its right interpolation endpoint on
            # a later tick. An age bound prevents waiting indefinitely.
            self._reject_visual("attitude_unaligned")
            return
        self._pending_target = None
        angles = world_bearing(sample.bbox, camera, attitude, self._heading)
        if not all(isfinite(v) for v in angles):
            self._reject_visual("invalid_geometry")
            return
        if abs(angles[0]) > t.max_elevation_rad + BEARING_EPSILON_RAD or abs(angles[1]) > t.max_azimuth_rad + BEARING_EPSILON_RAD:
            self._rejected_bearing = angles
            self._reject_visual("bearing_outside_envelope")
            return
        if self._reference is not None and hypot(*(wrap(a - b) for a, b in zip(angles, self._reference))) > t.max_los_deviation_rad:
            self._reject_visual("los_deviation")
            return
        if self._bearing.last_raw is not None and hypot(*(wrap(a - b) for a, b in zip(angles, self._bearing.last_raw))) > t.max_association_jump_rad:
            self._reject_visual("association_jump")
            return
        self._bearing.push(sample.timestamp_s, angles)
        self._ttc.push(sample.timestamp_s, sample.bbox)
        if self._candidate_end is None or sample.timestamp_s - self._candidate_end > t.max_image_age_s + TIME_EPSILON:
            self._candidate_start = sample.timestamp_s
            self._candidate_reference = self._bearing.angles
            self._candidate_count = 0
        self._candidate_count += 1
        self._candidate_end = sample.timestamp_s
        self._last_visual_time = sample.timestamp_s
        self._visual_usable = True
        self._vision_reason = "usable"
        self._rejected_bearing = None

    def _recover(self, now, altitude, reason):
        if self.state != State.RECOVER:
            self.state = State.RECOVER
            self._recovery_at = now
            self._hold_altitude = altitude
            self._clear_candidate()
            self._bearing = BearingEstimator(self.config.tuning.bearing_filter_s)
            self._ttc.invalidate("recovery")
            self._ttc_entry_at = None
            self._descent_feasible = None
            self._visual_usable = False
            self._control.recover()
        self._reason = reason

    def _candidate_ready(self, dwell):
        return (self._candidate_count >= MIN_ACQUISITION_FRAMES
                and self._candidate_end - self._candidate_start >= dwell - TIME_EPSILON
                and self._bearing.count >= 2)

    def _track(self, now):
        if self._reference is None:
            self._reference = self._candidate_reference
        errors = tuple(wrap(a - b) for a, b in zip(self._bearing.at(now), self._reference))
        self._control.begin_track(errors, self._bearing.rates)
        self.state = State.TRACK
        self._reason = "tracking"
        self._limited_at = None

    def _stage_and_feasibility(self, now, altitude, vz):
        t = self.config.tuning
        tau, _ = self._ttc.at(now)
        self._descent_feasible = None
        if tau is not None:
            self._descent_feasible = descent_feasible(altitude - self.config.target_altitude_m, vz, tau, t)
        if self.stage == TrackStage.TERMINAL:
            return
        if tau is None or tau >= t.ttc_exit_s:
            self.stage = TrackStage.BEARING
            self._ttc_entry_at = None
        elif tau <= t.terminal_ttc_s:
            self.stage = TrackStage.TERMINAL
        elif tau <= t.ttc_enter_s:
            if self._ttc_entry_at is None:
                self._ttc_entry_at = now
            if now - self._ttc_entry_at >= t.ttc_dwell_s:
                self.stage = TrackStage.TTC
        elif self.stage == TrackStage.BEARING:
            self._ttc_entry_at = None

    def update(self, inputs: NavigationInput, *, now_s: float, enabled: bool,
               command_horizon_s: float = 0.0) -> NavigationResult:
        """Advance once; invalid shapes/clocks raise ValueError before state mutation.

        Missing/stale operational data return state-bearing results. All samples
        use measurement time in the same clock domain as now_s, never arrival time.
        command_horizon_s reserves image validity for caller scheduling and delivery.
        """
        finite(command_horizon_s, "command_horizon_s")
        if command_horizon_s < 0:
            raise ValueError("command_horizon_s must be nonnegative")
        self._validate_tick(inputs, now_s, enabled)
        now = float(now_s)
        previous_time = self._last_now
        self._last_now = now
        self._input_rejection = ""
        if self.state == State.FAULT:
            return self._result(now)
        if not enabled:
            if self.state != State.DISABLED:
                self._ended = True
            self.state = State.DISABLED
            self._reason = "disabled"
            return self._result(now)
        if self._ended:
            self._fault("reset_required")
            return self._result(now)
        if self.state == State.DISABLED:
            self.state = State.ACQUIRE
            self._started_at = now
            if not self._sensors_were_ready:
                previous_time = None
        dt = 0.0 if previous_time is None else now - previous_time
        if dt > self.config.tuning.max_control_gap_s + TIME_EPSILON:
            self._fault("control_gap")
            return self._result(now)
        if self._accept("attitude", inputs.attitude):
            self._attitudes.push(inputs.attitude)
        if self._accept("barometer", inputs.barometer):
            try:
                self._vertical.push(inputs.barometer)
            except ArithmeticError:
                self._fault("vertical_numeric_failure")
        if self._accept("target", inputs.target):
            self._pending_target = inputs.target
        if self.state == State.FAULT:
            return self._result(now)

        t = self.config.tuning
        attitude = self._last_samples.get("attitude")
        barometer = self._last_samples.get("barometer")
        bad_sensor = ("attitude_stale" if attitude is None or now - attitude.timestamp_s > t.max_attitude_age_s + TIME_EPSILON
                      else "barometer_stale" if barometer is None or now - barometer.timestamp_s > t.max_barometer_age_s + TIME_EPSILON
                      else None)
        if bad_sensor or not self._vertical.ready:
            if self._sensors_were_ready:
                self._fault(bad_sensor or "vertical_unready")
            elif now - self._started_at >= t.acquisition_timeout_s:
                self._fault("acquisition_timeout")
            else:
                self._reason = bad_sensor or "vertical_warmup"
            return self._result(now)
        self._sensors_were_ready = True
        altitude, vz = self._vertical.at(now)
        if not isfinite(altitude) or not isfinite(vz):
            self._fault("vertical_numeric_failure")
            return self._result(now)
        if self.state == State.ACQUIRE:
            self._reason = "acquiring"
        if self._heading is None:
            self._heading = euler(attitude.orientation_xyzw)[2]
            self._hold_altitude = altitude
        self._process_target(now)
        if self._last_visual_time is None or now - self._last_visual_time > t.max_image_age_s + TIME_EPSILON:
            self._reject_visual("stale_image" if self._visual_usable else self._vision_reason)
        elif (command_horizon_s > 0 and self._visual_usable
              and self._last_visual_time + t.max_image_age_s - now <= command_horizon_s + TIME_EPSILON):
            # Recover before an image-limited command can expire between caller ticks.
            self._reject_visual("image_deadline")
        measured_roll, measured_pitch, _ = euler(attitude.orientation_xyzw)
        over_tilt = combined_tilt(measured_roll, measured_pitch) > t.combined_tilt_limit_rad + TIME_EPSILON
        if over_tilt:
            self._recover(now, altitude, "measured_over_tilt")
            self._clear_candidate()
            self._visual_usable = False
        elif self.state == State.TRACK and not self._visual_usable:
            self._recover(now, altitude, self._vision_reason)
        elif self.state in (State.ACQUIRE, State.RECOVER) and self._visual_usable:
            dwell = t.acquisition_dwell_s if self.state == State.ACQUIRE else t.recovery_dwell_s
            if self._candidate_ready(dwell):
                self._track(now)

        if self.state == State.ACQUIRE and now - self._started_at >= t.acquisition_timeout_s:
            self._fault("acquisition_timeout")
        if self.state == State.RECOVER and now - self._recovery_at >= t.recovery_timeout_s:
            self._fault("recovery_timeout")
        if self.state == State.FAULT:
            return self._result(now)

        errors = None
        approach_scale = 0.0
        if self.state == State.TRACK:
            self._stage_and_feasibility(now, altitude, vz)
            previous = self._control.last_diagnostics
            authority_limited = (previous.get("angle_limited", False) or previous.get("vertical_limited", False)
                                 or previous.get("thrust_saturated", False)
                                 or abs(self._control.vz_reference) >= t.max_descent_m_s - TIME_EPSILON
                                 or self._control.vz_reference >= t.max_climb_m_s - TIME_EPSILON)
            limited = authority_limited or self._descent_feasible is False
            if limited:
                if self._limited_at is None:
                    self._limited_at = now
                approach_scale = max(0.0, 1 - (now - self._limited_at) / t.limitation_timeout_s)
                self._reason = "descent_infeasible" if self._descent_feasible is False else "control_limited"
                if now - self._limited_at >= t.limitation_timeout_s:
                    self._recover(now, altitude, "persistent_limitation")
            else:
                self._limited_at = None
                approach_scale = 1.0
                self._reason = "tracking"
            if self.state == State.TRACK:
                errors = tuple(wrap(a - b) for a, b in zip(self._bearing.at(now), self._reference))
        try:
            command = self._control.step(dt, altitude, vz, attitude.orientation_xyzw, self._heading,
                                         errors=errors, rates=self._bearing.rates,
                                         hold_altitude=self._hold_altitude, approach_scale=approach_scale,
                                         freeze_integral=over_tilt)
        except ArithmeticError:
            self._fault("control_numeric_failure")
            return self._result(now)
        return self._result(now, command)

    def _result(self, now, command=None):
        t = self.config.tuning
        attitude, barometer = self._last_samples.get("attitude"), self._last_samples.get("barometer")
        tau, ttc_reason = self._ttc.at(now)
        angles = self._bearing.at(now) if self._bearing.time is not None else (None, None)
        vertical = self._vertical.at(now) if self._vertical.time is not None else (None, None)
        bearing_valid = (self._visual_usable and self._last_visual_time is not None
                         and now - self._last_visual_time <= t.max_image_age_s + TIME_EPSILON)
        from .geometry import box_edges
        image = self._processed_target
        margins, edges = box_edges(image.bbox, self.config.camera) if image is not None and image.bbox is not None else (None, ())
        diagnostics = Diagnostics(
            processed_image_stream_id=None if image is None else image.stream_id,
            processed_image_sequence=None if image is None else image.sequence,
            processed_image_timestamp_s=None if image is None else image.timestamp_s,
            processed_bbox=None if image is None else image.bbox,
            edge_margins_px=margins, touched_edges=edges,
            elevation_rad=angles[0], azimuth_rad=angles[1],
            elevation_rate_rad_s=self._bearing.rates[0] if self._bearing.count >= 2 else None,
            azimuth_rate_rad_s=self._bearing.rates[1] if self._bearing.count >= 2 else None,
            bearing_valid=bearing_valid,
            rate_observable=self._bearing.count >= 2 and bearing_valid,
            reference_elevation_rad=self._reference[0] if self._reference else None,
            reference_azimuth_rad=self._reference[1] if self._reference else None,
            altitude_m=vertical[0] if vertical[0] is not None and isfinite(vertical[0]) else None,
            vertical_speed_m_s=vertical[1] if vertical[1] is not None and isfinite(vertical[1]) else None, vertical_ready=self._vertical.ready,
            optical_ttc_s=tau, ttc_reason=ttc_reason, ttc_relative_residual=self._ttc.relative_residual,
            descent_feasible=self._descent_feasible,
            image_age_s=now - self._last_visual_time if self._last_visual_time is not None else None,
            attitude_age_s=now - attitude.timestamp_s if attitude else None,
            barometer_age_s=now - barometer.timestamp_s if barometer else None,
            vision_reason=self._vision_reason, input_rejection=self._input_rejection,
            rejected_elevation_rad=self._rejected_bearing[0] if self._rejected_bearing else None,
            rejected_azimuth_rad=self._rejected_bearing[1] if self._rejected_bearing else None,
            max_elevation_rad=t.max_elevation_rad, max_azimuth_rad=t.max_azimuth_rad,
            **self._control.last_diagnostics)
        expiry = now
        if command is not None:
            expiry = min(now + t.command_validity_s, attitude.timestamp_s + t.max_attitude_age_s,
                         barometer.timestamp_s + t.max_barometer_age_s)
            if self.state == State.ACQUIRE:
                expiry = min(expiry, self._started_at + t.acquisition_timeout_s)
            elif self.state == State.TRACK:
                expiry = min(expiry, self._last_visual_time + t.max_image_age_s)
            elif self.state == State.RECOVER:
                expiry = min(expiry, self._recovery_at + t.recovery_timeout_s)
            if expiry <= now:
                command = None
        self._sequence += 1
        return NavigationResult(self._sequence, now, expiry, self.state, self.stage, self._reason, command, diagnostics)
