"""Original-LOS outer control and bounded Angle-mode RC conversion, without IO."""
from math import acos, cos, isfinite

from .geometry import clamp, combined_tilt, euler, wrap
from .types import RcChannels, finite


RC_CENTRE = 1500
RC_HALF_RANGE = 500
RC_MIN = 1000
RC_SPAN = 1000
NUMERICAL_TOLERANCE = 1e-12
MIN_VERTICAL_PROJECTION = 0.5


def limit_angles(roll, pitch, tuning):
    roll = clamp(roll, -min(tuning.roll_limit_rad, tuning.combined_tilt_limit_rad),
                 min(tuning.roll_limit_rad, tuning.combined_tilt_limit_rad))
    pitch_allowance = acos(clamp(cos(tuning.combined_tilt_limit_rad) / cos(roll), -1.0, 1.0))
    pitch_bound = min(tuning.pitch_limit_rad, pitch_allowance)
    return roll, clamp(pitch, -pitch_bound, pitch_bound)


def slew_pair(previous, desired, dt, tuning):
    fraction = 1.0
    for old, new, rate in zip(previous, desired, (tuning.roll_slew_rad_s, tuning.pitch_slew_rad_s)):
        if abs(new - old) > NUMERICAL_TOLERANCE:
            fraction = min(fraction, rate * dt / abs(new - old))
    return tuple(old + fraction * (new - old) for old, new in zip(previous, desired))


def encode_channels(roll, pitch, thrust, yaw_rate, config):
    """Round like Bullet's RcCodec, then retract quantization across angle limits.

    Continuous references obey slew exactly. Integer PWM has a one-count
    resolution error; it cannot express arbitrary sub-count motion per tick.
    """
    vehicle, tuning = config.vehicle, config.tuning
    scale = vehicle.rc_angle_scale_rad
    roll_channel = round(RC_CENTRE + RC_HALF_RANGE * roll / scale)
    pitch_channel = round(RC_CENTRE + RC_HALF_RANGE * pitch / scale)
    decode = lambda channel: (channel - RC_CENTRE) / RC_HALF_RANGE * scale
    while abs(decode(roll_channel)) > min(tuning.roll_limit_rad, tuning.combined_tilt_limit_rad) + NUMERICAL_TOLERANCE:
        roll_channel -= 1 if roll_channel > RC_CENTRE else -1
    while (abs(decode(pitch_channel)) > tuning.pitch_limit_rad + NUMERICAL_TOLERANCE
           or combined_tilt(decode(roll_channel), decode(pitch_channel)) > tuning.combined_tilt_limit_rad + NUMERICAL_TOLERANCE):
        pitch_channel -= 1 if pitch_channel > RC_CENTRE else -1
    yaw_channel = round(RC_CENTRE + RC_HALF_RANGE * yaw_rate / vehicle.rc_yaw_rate_scale_rad_s)
    while abs((yaw_channel - RC_CENTRE) / RC_HALF_RANGE * vehicle.rc_yaw_rate_scale_rad_s) > tuning.yaw_rate_limit_rad_s + NUMERICAL_TOLERANCE:
        yaw_channel -= 1 if yaw_channel > RC_CENTRE else -1
    throttle = round(RC_MIN + RC_SPAN * clamp(thrust / vehicle.max_thrust_n, 0.0, 1.0))
    channels = RcChannels(roll_channel, pitch_channel, throttle, yaw_channel)
    return channels, (decode(roll_channel), decode(pitch_channel))


class LosControl:
    def __init__(self, config):
        self.config = config
        self.tuning = config.tuning
        self.angles = (0.0, 0.0)
        self.vz_reference = 0.0
        self.vertical_memory = self.lateral_memory = 0.0
        self.vertical_bias = self.roll_bias = 0.0
        self.thrust_integral = 0.0
        self.last_diagnostics = {}

    def begin_track(self, errors, rates):
        """Reseed biases for continuity without changing the saved LOS."""
        t = self.tuning
        self.vertical_memory = self.lateral_memory = 0.0
        self.vertical_bias = self.vz_reference + t.elevation_kp_m_s_rad * errors[0] + t.elevation_kd_m_rad * rates[0]
        self.roll_bias = self.angles[0] + t.azimuth_kp * errors[1] + t.azimuth_kd_s * rates[1]

    def recover(self):
        self.vertical_memory = self.lateral_memory = 0.0

    def step(self, dt, altitude, vz, attitude, heading, *, errors=None, rates=(0.0, 0.0),
             hold_altitude=None, approach_scale=1.0, freeze_integral=False, max_climb_m_s=None, yaw_rate_rad_s=None):
        t, vehicle = self.tuning, self.config.vehicle
        if yaw_rate_rad_s is not None:
            finite(yaw_rate_rad_s, 'yaw_rate_rad_s')
            if abs(yaw_rate_rad_s) > t.yaw_rate_limit_rad_s:
                raise ValueError('yaw rate exceeds configured limit')
        tracking = errors is not None
        old_z, old_r = self.vertical_memory, self.lateral_memory
        if tracking:
            candidate_z = clamp(old_z - t.elevation_ki_m_s2_rad * errors[0] * dt,
                                -t.vertical_memory_limit_m_s, t.vertical_memory_limit_m_s)
            candidate_r = clamp(old_r - t.azimuth_ki_s_inv * errors[1] * dt,
                                -t.lateral_memory_limit_rad, t.lateral_memory_limit_rad)
            base_z = self.vertical_bias - t.elevation_kp_m_s_rad * errors[0] - t.elevation_kd_m_rad * rates[0]
            base_r = self.roll_bias - t.azimuth_kp * errors[1] - t.azimuth_kd_s * rates[1]
        else:
            candidate_z, candidate_r = old_z, old_r
            base_z = t.hold_altitude_gain_s_inv * (hold_altitude - altitude)
            base_r = 0.0

        requested_z = base_z + (candidate_z if tracking else 0.0)
        limited_z = clamp(requested_z, -t.max_descent_m_s, t.max_climb_m_s if max_climb_m_s is None else max_climb_m_s)
        new_z = self.vz_reference + clamp(limited_z - self.vz_reference,
                                        -t.vertical_slew_m_s2 * dt, t.vertical_slew_m_s2 * dt)
        if tracking and (candidate_z - old_z) * (requested_z - new_z) > 0:
            candidate_z = old_z
            requested_z = base_z + candidate_z
            limited_z = clamp(requested_z, -t.max_descent_m_s, t.max_climb_m_s if max_climb_m_s is None else max_climb_m_s)
            new_z = self.vz_reference + clamp(limited_z - self.vz_reference,
                                            -t.vertical_slew_m_s2 * dt, t.vertical_slew_m_s2 * dt)

        requested_roll = base_r + (candidate_r if tracking else 0.0)
        if not isfinite(requested_roll) or not isfinite(requested_z):
            raise ArithmeticError("LOS controller overflow")
        requested_pitch = t.approach_pitch_rad * approach_scale if tracking else 0.0
        limited_angles = limit_angles(requested_roll, requested_pitch, t)
        new_angles = slew_pair(self.angles, limited_angles, dt, t)
        if tracking and (candidate_r - old_r) * (requested_roll - new_angles[0]) > 0:
            candidate_r = old_r
            requested_roll = base_r + candidate_r
            limited_angles = limit_angles(requested_roll, requested_pitch, t)
            new_angles = slew_pair(self.angles, limited_angles, dt, t)

        angle_limited = any(abs(a - b) > NUMERICAL_TOLERANCE for a, b in
                            zip((requested_roll, requested_pitch), limited_angles))
        slew_limited = (abs(new_z - limited_z) > NUMERICAL_TOLERANCE
                        or any(abs(a - b) > NUMERICAL_TOLERANCE for a, b in zip(new_angles, limited_angles)))
        measured_roll, measured_pitch, measured_yaw = euler(attitude)
        measured_tilt = combined_tilt(measured_roll, measured_pitch)
        error_vz = new_z - vz
        candidate_thrust_i = clamp(self.thrust_integral + t.vertical_ki_n_m * error_vz * dt,
                                  -t.vertical_integral_limit_n, t.vertical_integral_limit_n)
        if freeze_integral:
            candidate_thrust_i = self.thrust_integral
        projection = max(MIN_VERTICAL_PROJECTION, cos(measured_roll) * cos(measured_pitch))
        hover = vehicle.mass_kg * vehicle.gravity_m_s2
        raw_thrust = (hover + t.vertical_kp_n_s_m * error_vz + candidate_thrust_i) / projection
        if not isfinite(raw_thrust):
            raise ArithmeticError("vertical controller overflow")
        thrust = clamp(raw_thrust, 0.0, vehicle.max_thrust_n)
        if (candidate_thrust_i - self.thrust_integral) * (raw_thrust - thrust) > 0:
            candidate_thrust_i = self.thrust_integral
            raw_thrust = (hover + t.vertical_kp_n_s_m * error_vz + candidate_thrust_i) / projection
            thrust = clamp(raw_thrust, 0.0, vehicle.max_thrust_n)
        yaw_rate = clamp(t.yaw_gain_s_inv * wrap(heading - measured_yaw),
                         -t.yaw_rate_limit_rad_s, t.yaw_rate_limit_rad_s)
        if yaw_rate_rad_s is not None:
            yaw_rate = yaw_rate_rad_s
        command, output_angles = encode_channels(*new_angles, thrust, yaw_rate, self.config)
        self.vertical_memory, self.lateral_memory = candidate_z, candidate_r
        self.thrust_integral = candidate_thrust_i
        self.angles, self.vz_reference = new_angles, new_z
        self.last_diagnostics = dict(
            requested_vertical_speed_m_s=requested_z, vertical_reference_m_s=new_z,
            requested_roll_rad=requested_roll, reference_roll_rad=new_angles[0], reference_pitch_rad=new_angles[1],
            output_roll_rad=output_angles[0], output_pitch_rad=output_angles[1],
            requested_tilt_rad=combined_tilt(requested_roll, requested_pitch),
            output_tilt_rad=combined_tilt(*output_angles), measured_tilt_rad=measured_tilt,
            measured_over_tilt=measured_tilt > t.combined_tilt_limit_rad,
            vertical_limited=abs(requested_z - limited_z) > NUMERICAL_TOLERANCE,
            angle_limited=angle_limited, slew_limited=slew_limited,
            thrust_n=thrust, thrust_saturated=abs(thrust - raw_thrust) > NUMERICAL_TOLERANCE,
            vertical_memory_m_s=candidate_z, lateral_memory_rad=candidate_r)
        return command
