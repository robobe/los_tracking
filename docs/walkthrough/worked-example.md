# Worked example: cold start to a tracking tick

Use the Bullet profile, level attitude, constant barometer altitude 10 m and a
fixed bbox `(310, 280, 20, 20)` at 100 Hz. This is an illustrative measurement
sequence, not a simulated plant: measured attitude stays level even when control
requests forward pitch.

```python
from pathlib import Path

from los_tracking import (
    AttitudeSample, BarometerSample, LosNavigator, NavigationInput,
    NavigatorConfig, State, TargetObservation,
)

# Run from the los_tracking package directory.
config = NavigatorConfig.from_json(Path("examples/bullet_profile.json"))
navigator = LosNavigator(config)
results = []
for i in range(36):
    now = i / 100
    inputs = NavigationInput(
        target=TargetObservation("camera", i, now, (310, 280, 20, 20)),
        barometer=BarometerSample("barometer", i, now, 10.0),
        attitude=AttitudeSample("attitude", i, now, (0, 0, 0, 1)),
    )
    results.append(navigator.update(
        inputs, now_s=now, enabled=True, command_horizon_s=0.02,
    ))

assert not results[0].usable
assert results[0].reason == "vertical_warmup"
result = results[-1]
assert result.state == State.TRACK and result.usable
if result.usable and now < result.valid_until_s:
    roll, pitch, throttle, yaw = result.command.as_tuple()
```

Trace through the entry method:

| Time | What happens |
| --- | --- |
| 0.00 s | Enter `ACQUIRE`; first sample initializes the vertical estimator. Return `vertical_warmup`, no command. |
| 0.01–0.09 s | New samples update history, but warmup span is insufficient. No command. |
| 0.10 s | Vertical state becomes ready. Capture hold altitude 10 m and heading 0; process the latest target and begin visual qualification. Hold control is available. |
| 0.20 s | Candidate observations span 0.1 s and exceed three frames. Enter `TRACK`, save LOS reference and seed continuity biases. |
| 0.35 s | Continue `TRACK`; no LOS drift or growing bbox, so no qualified TTC. Normal requested approach scale remains one. |

For the final tick:

```text
dt = 0.35 - 0.34 = 0.01 s
image_age = 0, remaining_image_budget = 0.1 s > 0.02 s horizon
bbox centre = (320, 290)
azimuth = 0
elevation = atan2(50 / 320, 1) ≈ 0.155 rad ≈ 8.88° down
LOS error and rate = 0 for this constant, level sequence
requested pitch = 7° × 1 = 7°
expiry = min(0.35 + 0.05, 0.35 + 0.05, 0.35 + 0.2, 0.35 + 0.1)
       = 0.40 s
```

The output pitch reference has only reached about 0.8° by 0.35 s because the
5°/s pitch slew starts at tracking entry. A 7° request therefore does not imply
an immediate 7° reference or measured attitude. Constant bbox size leaves TTC
unqualified; independent authority supervision remains active.

[Back to walkthrough](../approach-walkthrough.md)
