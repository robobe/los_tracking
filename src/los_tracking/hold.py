"""Public sensor-only altitude/heading hold and continuous LOS handoff."""
from dataclasses import dataclass

from .config import NavigatorConfig
from .control import LosControl
from .estimation import VerticalEstimator
from .geometry import AttitudeHistory, combined_tilt, euler, wrap
from .types import AttitudeSample, BarometerSample, NavigationInput, finite


@dataclass(frozen=True)
class ControlHandoff:
    config: NavigatorConfig
    timestamp_s: float
    heading_rad: float
    hold_altitude_m: float
    vertical_state: tuple
    attitudes: tuple[AttitudeSample, ...]
    barometer: BarometerSample
    angles_rad: tuple[float, float]
    vertical_reference_m_s: float
    thrust_integral_n: float

    def __post_init__(self):
        if not isinstance(self.config, NavigatorConfig) or not isinstance(self.barometer, BarometerSample):
            raise ValueError('invalid handoff configuration/sample')
        if any(type(v) is not tuple for v in (self.vertical_state, self.attitudes, self.angles_rad)):
            raise ValueError('handoff containers must be immutable tuples')
        values = (self.timestamp_s, self.heading_rad, self.hold_altitude_m,
                  *self.vertical_state, *self.angles_rad, self.vertical_reference_m_s, self.thrust_integral_n)
        for value in values:
            finite(value, 'handoff state')
        if len(self.vertical_state) != 6 or len(self.angles_rad) != 2 or not 1 <= len(self.attitudes) <= 512:
            raise ValueError('invalid handoff dimensions')
        previous = None
        for sample in self.attitudes:
            if not isinstance(sample, AttitudeSample) or sample.timestamp_s > self.timestamp_s:
                raise ValueError('invalid handoff attitude')
            if previous and (sample.stream_id != previous.stream_id or sample.timestamp_s <= previous.timestamp_s or sample.sequence <= previous.sequence):
                raise ValueError('unordered handoff history')
            previous = sample
        time, first, altitude, velocity, count, ready = self.vertical_state
        t = self.config.tuning
        if (abs(self.angles_rad[0]) > t.roll_limit_rad or abs(self.angles_rad[1]) > t.pitch_limit_rad
                or combined_tilt(*self.angles_rad) > t.combined_tilt_limit_rad
                or type(count) is not int or type(ready) is not int
                or time != self.barometer.timestamp_s or first > time or time > self.timestamp_s
                or self.timestamp_s - time > t.max_barometer_age_s
                or self.timestamp_s - self.attitudes[-1].timestamp_s > t.max_attitude_age_s
                or time - first < t.vertical_warmup_s - 1e-9
                or count < 3 or ready != 1 or abs(self.thrust_integral_n) > self.config.tuning.vertical_integral_limit_n
                or not -t.max_descent_m_s <= self.vertical_reference_m_s <= t.max_climb_m_s):
            raise ValueError('invalid handoff estimator/controller state')


def restore_handoff(handoff, config):
    if not isinstance(handoff, ControlHandoff) or handoff.config != config:
        raise ValueError('handoff profile must match navigator')
    vertical = VerticalEstimator(config.tuning)
    vertical.time, vertical.first_time, vertical.altitude, vertical.velocity, vertical.count, _ = handoff.vertical_state
    history = AttitudeHistory(config.tuning.attitude_history_s, config.tuning.max_alignment_gap_s)
    for sample in handoff.attitudes:
        history.push(sample)
    control = LosControl(config)
    control.angles = handoff.angles_rad
    control.vz_reference = handoff.vertical_reference_m_s
    control.thrust_integral = handoff.thrust_integral_n
    return vertical, history, control


class AltitudeHeadingHold:
    """Caller-owned filter and hold; never consumes position or horizontal velocity."""
    def __init__(self, config, *, max_climb_m_s=1.0):
        if not 0 < max_climb_m_s <= config.tuning.max_climb_m_s:
            raise ValueError('invalid hold climb limit')
        self.config = config
        self.max_climb_m_s = max_climb_m_s
        self.vertical = VerticalEstimator(config.tuning)
        self.history = AttitudeHistory(config.tuning.attitude_history_s, config.tuning.max_alignment_gap_s)
        self.control = LosControl(config)
        self.barometer = self.attitude = None
        self.heading_rad = None
        self.last_now = None

    def observe(self, inputs: NavigationInput):
        for name in ('attitude', 'barometer'):
            sample = getattr(inputs, name)
            old = getattr(self, name)
            if sample is None or old == sample:
                continue
            if old and (sample.stream_id != old.stream_id or sample.sequence <= old.sequence or sample.timestamp_s <= old.timestamp_s):
                raise ValueError('hold sample stream/order changed')
            if name == 'attitude':
                self.history.push(sample)
                if self.heading_rad is None:
                    self.heading_rad = euler(sample.orientation_xyzw)[2]
            else:
                self.vertical.push(sample)
            setattr(self, name, sample)

    def ready(self, now_s):
        t = self.config.tuning
        return (self.vertical.ready and self.attitude is not None
                and 0 <= now_s - self.attitude.timestamp_s <= t.max_attitude_age_s
                and 0 <= now_s - self.barometer.timestamp_s <= t.max_barometer_age_s)

    def estimate(self, now_s):
        return self.vertical.at(now_s)

    def set_heading(self, heading_rad):
        """Select a hold/handoff heading without resetting filters or integrals."""
        self.heading_rad = wrap(finite(heading_rad, 'heading_rad'))

    def attitude_at(self, timestamp_s):
        """Aligned attitude at exposure time; never extrapolate."""
        return self.history.at(finite(timestamp_s, 'timestamp_s'))

    def command(self, *, now_s, target_altitude_m, yaw_rate_rad_s=None):
        if yaw_rate_rad_s is not None:
            finite(yaw_rate_rad_s, 'yaw_rate_rad_s')
            if abs(yaw_rate_rad_s) > self.config.tuning.yaw_rate_limit_rad_s:
                raise ValueError('yaw rate exceeds configured limit')
        if not self.ready(now_s):
            raise ValueError('hold sensors not ready')
        dt = 0.0 if self.last_now is None else now_s - self.last_now
        if not 0 <= dt <= self.config.tuning.max_control_gap_s:
            raise ValueError('hold control gap')
        self.last_now = now_s
        altitude, vz = self.estimate(now_s)
        return self.control.step(dt, altitude, vz, self.attitude.orientation_xyzw, self.heading_rad,
                                 hold_altitude=target_altitude_m, max_climb_m_s=self.max_climb_m_s, yaw_rate_rad_s=yaw_rate_rad_s)

    def handoff(self, *, now_s, target_altitude_m):
        if not self.ready(now_s):
            raise ValueError('handoff requires fresh initialized sensors')
        v = self.vertical
        return ControlHandoff(self.config, now_s, self.heading_rad, target_altitude_m,
                              (v.time, v.first_time, v.altitude, v.velocity, v.count, 1),
                              tuple(self.history.samples), self.barometer, self.control.angles,
                              self.control.vz_reference, self.control.thrust_integral)
