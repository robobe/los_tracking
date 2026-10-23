"""Pinhole geometry, FLU body vectors, and body-to-world XYZW rotations."""
from bisect import bisect_left
from collections import deque
from math import acos, atan2, cos, hypot, pi, sin, sqrt

from .config import CameraCalibration
from .types import AttitudeSample


HISTORY_CAPACITY = 512
SLERP_LINEAR_THRESHOLD = 0.9995


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


def rotate(q, vector):
    x, y, z, w = q
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def euler(q):
    x, y, z, w = q
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = atan2(2 * (w * y - z * x), sqrt(max(0.0, 1 - (2 * (w * y - z * x)) ** 2)))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def slerp(a, b, fraction):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-v for v in b), -dot
    dot = clamp(dot, -1.0, 1.0)
    if dot > SLERP_LINEAR_THRESHOLD:
        q = tuple(x + fraction * (y - x) for x, y in zip(a, b))
        norm = sqrt(sum(v * v for v in q))
        return tuple(v / norm for v in q)
    angle = acos(dot)
    return tuple((sin((1 - fraction) * angle) * x + sin(fraction * angle) * y) / sin(angle)
                 for x, y in zip(a, b))


def world_bearing(bbox, camera: CameraCalibration, attitude, heading_rad):
    x, y, width, height = bbox
    xn = (x + width / 2 - camera.cx_px) / camera.fx_px
    yn = (y + height / 2 - camera.cy_px) / camera.fy_px
    body_ray = rotate(camera.camera_to_body_xyzw, (1.0, -xn, -yn))
    wx, wy, wz = rotate(attitude, body_ray)
    return atan2(-wz, hypot(wx, wy)), wrap(atan2(wy, wx) - heading_rad)


def combined_tilt(roll, pitch):
    return acos(clamp(cos(roll) * cos(pitch), -1.0, 1.0))


class AttitudeHistory:
    def __init__(self, horizon_s, alignment_gap_s):
        self.samples = deque(maxlen=HISTORY_CAPACITY)
        self.horizon_s = horizon_s
        self.alignment_gap_s = alignment_gap_s

    def push(self, sample: AttitudeSample):
        self.samples.append(sample)
        cutoff = sample.timestamp_s - self.horizon_s
        # Retain one point before the horizon so a boundary query can interpolate.
        while len(self.samples) > 2 and self.samples[1].timestamp_s < cutoff:
            self.samples.popleft()

    def at(self, timestamp_s):
        samples = list(self.samples)
        index = bisect_left([s.timestamp_s for s in samples], timestamp_s)
        if index < len(samples) and abs(samples[index].timestamp_s - timestamp_s) < 1e-9:
            return samples[index].orientation_xyzw
        if index == 0 or index == len(samples):
            return None
        left, right = samples[index - 1], samples[index]
        gap = right.timestamp_s - left.timestamp_s
        if gap > self.alignment_gap_s:
            return None
        return slerp(left.orientation_xyzw, right.orientation_xyzw,
                     (timestamp_s - left.timestamp_s) / gap)
