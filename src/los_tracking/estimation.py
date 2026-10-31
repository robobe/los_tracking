"""Measurement-driven filters; repeated control ticks never insert measurements."""
from collections import deque
from math import ceil, exp, isfinite, sqrt

from .geometry import clamp, wrap


TTC_HISTORY_CAPACITY = 256
TTC_MIN_SAMPLES = 8
TTC_MIN_POSITIVE_GROWTH_FRACTION = 0.6
VERTICAL_MIN_SAMPLES = 3
EPSILON = 1e-12


class VerticalEstimator:
    def __init__(self, tuning):
        self.tuning = tuning
        self.time = self.first_time = None
        self.altitude = self.velocity = 0.0
        self.count = 0

    def push(self, sample):
        if self.time is None:
            self.altitude = sample.altitude_m
            self.first_time = sample.timestamp_s
        else:
            dt = sample.timestamp_s - self.time
            predicted = self.altitude + self.velocity * dt
            residual = sample.altitude_m - predicted
            self.altitude = predicted + self.tuning.barometer_alpha * residual
            self.velocity += self.tuning.barometer_beta * residual / dt
        if not isfinite(self.altitude) or not isfinite(self.velocity):
            raise ArithmeticError("vertical estimator overflow")
        self.time = sample.timestamp_s
        self.count += 1

    @property
    def ready(self):
        return (self.count >= VERTICAL_MIN_SAMPLES
                and self.time - self.first_time >= self.tuning.vertical_warmup_s - EPSILON)

    def at(self, now):
        return self.altitude + self.velocity * (now - self.time), self.velocity


class BearingEstimator:
    def __init__(self, time_constant_s):
        self.time_constant_s = time_constant_s
        self.time = None
        self.angles = (0.0, 0.0)
        self.rates = (0.0, 0.0)
        self.count = 0
        self.last_raw = None

    def push(self, timestamp, angles):
        if self.time is None:
            self.angles = angles
        else:
            dt = timestamp - self.time
            alpha = 1 - exp(-dt / self.time_constant_s)
            beta = alpha * alpha / (2 - alpha)
            predicted = self.at(timestamp)
            residual = tuple(wrap(a - p) for a, p in zip(angles, predicted))
            self.angles = tuple(wrap(p + alpha * r) for p, r in zip(predicted, residual))
            self.rates = tuple(v + beta * r / dt for v, r in zip(self.rates, residual))
        self.time = timestamp
        self.last_raw = angles
        self.count += 1

    def at(self, now):
        return tuple(wrap(a + v * (now - self.time)) for a, v in zip(self.angles, self.rates))


class TtcEstimator:
    """Fit reciprocal apparent size to a local constant optical-depth closing model.

    For a rigid pinhole target, size s ~ 1/depth. Fitting 1/s makes TTC
    prediction evolve with the model instead of freezing an old TTC observation.
    Residuals qualify that model; they do not establish target identity/contact.
    """
    def __init__(self, tuning):
        self.tuning = tuning
        self.samples = deque(maxlen=TTC_HISTORY_CAPACITY)
        self.reason = "insufficient_history"
        self.relative_residual = None
        self.fit = None
        self.qualified_since = None

    def invalidate(self, reason):
        self.reason = reason
        self.samples.clear()
        self.fit = None
        self.relative_residual = None
        self.qualified_since = None

    def push(self, timestamp, bbox):
        size = sqrt(bbox[2]) * sqrt(bbox[3])
        self.samples.append((timestamp, size))
        while self.samples and timestamp - self.samples[0][0] > self.tuning.ttc_window_s + EPSILON:
            self.samples.popleft()
        self.fit = None
        self.relative_residual = None
        self.reason = "insufficient_history"
        if len(self.samples) < TTC_MIN_SAMPLES:
            self.qualified_since = None
            return
        span = timestamp - self.samples[0][0]
        if span < self.tuning.ttc_min_span_s:
            self.qualified_since = None
            return
        times = [t - timestamp for t, _ in self.samples]
        sizes = [s for _, s in self.samples]
        if sizes[-1] * sizes[-1] < self.tuning.ttc_min_area_px2:
            self.reason = "target_too_small"
            self.qualified_since = None
            return
        positive_steps = sum(new > old + EPSILON for old, new in zip(sizes, sizes[1:]))
        if positive_steps < ceil(TTC_MIN_POSITIVE_GROWTH_FRACTION * (len(sizes) - 1)):
            self.reason = "non_monotonic_growth"
            self.qualified_since = None
            return
        inverse_sizes = [1 / s for s in sizes]
        if not all(isfinite(v) for v in inverse_sizes):
            self.reason = "unresolved_growth"
            self.qualified_since = None
            return
        mean_t = sum(times) / len(times)
        mean_y = sum(inverse_sizes) / len(inverse_sizes)
        variance = sum((t - mean_t) ** 2 for t in times)
        if variance <= EPSILON or mean_y <= 0:
            self.reason = "unresolved_growth"
            self.qualified_since = None
            return
        slope = sum((t - mean_t) * (y - mean_y) for t, y in zip(times, inverse_sizes)) / variance
        intercept = mean_y - slope * mean_t
        if slope >= -EPSILON or intercept <= 0:
            self.reason = "not_closing"
            self.qualified_since = None
            return
        growth = self.samples[-1][1] - self.samples[0][1]
        if growth < self.tuning.ttc_min_growth_px:
            self.reason = "unresolved_growth"
            self.qualified_since = None
            return
        residual = sqrt(sum((y - intercept - slope * t) ** 2 for t, y in zip(times, inverse_sizes)) / len(times))
        self.relative_residual = residual / mean_y
        if self.relative_residual > self.tuning.ttc_max_relative_residual:
            self.reason = "noisy_growth"
            self.qualified_since = None
            return
        self.fit = (timestamp, intercept, slope)
        if self.qualified_since is None:
            self.qualified_since = timestamp
        self.reason = "qualified" if timestamp - self.qualified_since >= self.tuning.ttc_dwell_s - EPSILON else "qualification_dwell"

    def at(self, now):
        if self.fit is None:
            return None, self.reason
        timestamp, intercept, slope = self.fit
        age = now - timestamp
        if age > self.tuning.max_image_age_s + EPSILON:
            return None, "stale"
        remaining_depth_proxy = intercept + slope * age
        tau = -remaining_depth_proxy / slope
        if not isfinite(tau) or tau <= 0:
            return None, "model_elapsed"
        if self.qualified_since is None or timestamp - self.qualified_since < self.tuning.ttc_dwell_s - EPSILON:
            return None, "qualification_dwell"
        return tau, "qualified"


def descent_feasible(height_m, velocity_up_m_s, tau_s, tuning):
    """Kinematic supervision, not a guarantee of actual thrust authority/contact."""
    available = max(0.0, tau_s - tuning.ttc_time_margin_s)
    down_speed = clamp(-velocity_up_m_s, -tuning.max_climb_m_s, tuning.max_descent_m_s)
    ramp_time = min(available, (tuning.max_descent_m_s - down_speed) / tuning.vertical_slew_m_s2)
    max_drop = (down_speed * ramp_time + 0.5 * tuning.vertical_slew_m_s2 * ramp_time ** 2
                + tuning.max_descent_m_s * (available - ramp_time))
    return max(0.0, height_m) <= max_drop + tuning.feasibility_height_margin_m
