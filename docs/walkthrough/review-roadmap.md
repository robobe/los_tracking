# Next chapters: review roadmap

Each row is **pending detailed review and documentation**. Source links identify
the next methods to inspect; these rows do not claim their math has already been
documented here.

| Chapter | Methods / source | Question to answer |
| --- | --- | --- |
| 2. Validation and sample acceptance | `_validate_tick`, `_accept` in [navigator.py](../../src/los_tracking/navigator.py), contracts in [types.py](../../src/los_tracking/types.py) | Which inputs are errors, ignored samples or latched faults, and why? |
| 3. Attitude alignment | `AttitudeHistory.push/at`, `slerp` in [geometry.py](../../src/los_tracking/geometry.py) | How is orientation reconstructed at image time without extrapolation? |
| 4. Vertical estimation | `VerticalEstimator.push/ready/at` in [estimation.py](../../src/los_tracking/estimation.py) | How do altitude measurements produce filtered altitude and vertical speed? |
| 5. Bbox to world LOS | `_process_target` and `world_bearing` in [navigator.py](../../src/los_tracking/navigator.py) and [geometry.py](../../src/los_tracking/geometry.py) | How do calibration and rotations convert pixels into accepted world bearings? |
| 6. Bearing and angular rate | `BearingEstimator.push/at` in [estimation.py](../../src/los_tracking/estimation.py) | How are bearing noise, angular rates and control-time prediction handled? |
| 7. Acquire and recover | `_candidate_ready`, `_track`, `_recover` in [navigator.py](../../src/los_tracking/navigator.py) | How are reference LOS, dwell, continuity and reacquisition preserved? |
| 8. Optical TTC | `TtcEstimator.push/at/invalidate` in [estimation.py](../../src/los_tracking/estimation.py) | Why fit reciprocal size, and when is the result qualified? |
| 9. Approach feasibility | `descent_feasible`, `_stage_and_feasibility` in [estimation.py](../../src/los_tracking/estimation.py) and [navigator.py](../../src/los_tracking/navigator.py) | How much descent is predicted before contact, and how do margins/stages affect supervision? |
| 10. LOS and hold control | `LosControl.begin_track/step/recover` in [control.py](../../src/los_tracking/control.py) | How do angular error/rate, memory and vertical PI generate references and thrust? |
| 11. Limits and RC encoding | `limit_angles`, `slew_pair`, `encode_channels` in [control.py](../../src/los_tracking/control.py) | How do total tilt, coordinated slew, saturation and quantization constrain commands? |
| 12. Result and expiry | `_result` in [navigator.py](../../src/los_tracking/navigator.py) | How are validity flags, diagnostics and caller-use deadlines constructed? |

[Back to walkthrough](../approach-walkthrough.md)
