from math import cos, sin

import pytest
from los_tracking import (AttitudeSample, BarometerSample, CameraCalibration, NavigationInput,
                          NavigatorConfig, TargetObservation, VehicleConfig)


IDENTITY = (0.0, 0.0, 0.0, 1.0)
DEFAULT_BOX = (310.0, 280.0, 20.0, 20.0)


def from_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    cr, cp, cy = cos(roll / 2), cos(pitch / 2), cos(yaw / 2)
    sr, sp, sy = sin(roll / 2), sin(pitch / 2), sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


@pytest.fixture
def config():
    return NavigatorConfig(CameraCalibration.from_horizontal_fov(640, 480, 1.5707963267948966),
                           VehicleConfig(1.0, 33.953391), 0.0)


def samples(index, *, dt=0.01, bbox=DEFAULT_BOX, altitude=10.0, attitude=IDENTITY, camera=True):
    now = index * dt
    return NavigationInput(TargetObservation("camera", index, now, bbox) if camera else None,
                           BarometerSample("baro", index, now, altitude),
                           AttitudeSample("attitude", index, now, attitude))


def acquire(nav, *, start=0, count=35, **kwargs):
    for index in range(start, start + count):
        result = nav.update(samples(index, **kwargs), now_s=index * 0.01, enabled=True)
    return result
