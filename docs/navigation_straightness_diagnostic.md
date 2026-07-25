# Navigation Straightness Diagnostic

Use `record_navigation_odom_goal_closure.sh` with `--assert-straightness` to
separate a curved global path, oscillating MPPI commands, and chassis response
from one navigation run. The recorder uses one ROS participant and captures
only lightweight path, Twist, odometry, motion-state, IMU, and status messages.
It does not publish velocity, change parameters, or trigger localization.

Example:

```bash
bash scripts/jetson/runtime_overlay/scripts/record_navigation_odom_goal_closure.sh \
  --pose-id delivery_point_20260713T165519Z_07403ec3 \
  --building-id B10 \
  --floor-id F1 \
  --send-goal \
  --duration-sec 90 \
  --sample-period-sec 0.25 \
  --assert-straightness \
  --label calibration3_to_calibration2
```

The fixed acceptance limits apply to the central cruise segment:

- cross-track peak-to-peak: at most `0.12m`
- cross-track RMS: at most `0.04m`
- traveled-length/chord ratio: at most `1.015`
- persistent MPPI yaw-command reversals: at most `3`
- persistent measured yaw-rate reversals: at most `3`
- smoothed-plan cross-track: at most `0.08m`
- smoothed-plan length/chord ratio: at most `1.02`

The report directory contains `straightness.md`, `straightness.json`,
`path_metrics.json`, `samples.csv`, and the existing odom-closure report. A
`RED` result is expected during diagnosis and makes the command return nonzero;
it is not a navigation task failure by itself.
