# robot_local_state

Local odometry wrapper and the only canonical owner of `odom -> base_link`.

Production runtime currently defaults to
`LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_spin_imu`. In that mode the
odom preprocessor keeps Ranger `/wheel/odom` unchanged and publishes a
mode-aware `/wheel/odom_ekf`: wheel pose remains authoritative outside actual
Ranger `SPINNING` mode, while corrected JT128 IMU yaw-rate integrates only the
spin interval. The spin interval begins on `/motion_state.motion_mode=2`, keeps
spin x/y fixed, includes the physical tail after the final zero command, and
ends after IMU yaw-rate is below `0.02rad/s` for `0.30s`. The resulting yaw and
position correction is preserved as one rigid SE(2) transform when normal
wheel odom resumes. A yaw correction is therefore applied to subsequent wheel
x/y increments around the settled spin anchor; x/y are not maintained as
independent additive offsets.
Stale or missing IMU data causes a bounded fallback to raw wheel odom rather
than an unbounded IMU heading.

`robot_localization` fuses corrected wheel `x/y/yaw/vx` and corrected IMU
yaw-rate, but does not fuse wheel yaw-rate a second time. It publishes
`/local_state/odometry` and remains the only owner of `odom -> base_link`.
FAST-LIO2 and raw `/lidar_imu` are unchanged. `wheel_imu` remains the immediate
rollback profile.

The earlier `+180/-180deg` acceptance is invalid. A 2026-07-14 incident showed
that the motion-test scripts could accumulate depth-50 subscription queues
while processing only one callback per control iteration. During the failed
180-degree test the script saw `177.765deg` while an independent recorder was
already at `216.897deg`; settled IMU integration reached `224.90deg`. The same
incident exposed an invalid yaw-only pose correction: historical replay at
13.656 seconds changed from a false `2.091115m` corrected lateral displacement
to `0.0000002m` after applying the SE(2) fix. Hardware acceptance must be rerun
with the depth-1, background-executor, stale-feedback-fail-closed test scripts.

Experimental drift-reduction profiles remain available for controlled A/B
tests. `LOCAL_STATE_EKF_PROFILE=wheel_pose_imu_vyaw` remains available for
controlled fusion diagnostics and keeps wheel x/y/yaw pose
and wheel forward speed, but removes wheel yaw-rate so corrected JT128 IMU gyro
is the only EKF yaw-rate input.

`LOCAL_STATE_EKF_PROFILE=wheel_imu_primary_vyaw` is the guarded replacement
candidate for the rejected IMU-only-yaw-rate profile. Corrected IMU yaw-rate is
the dominant dynamic input. Wheel x/y pose and forward speed remain fused, and
high-covariance wheel yaw-rate remains only as a bounded outage fallback;
chassis-integrated absolute wheel yaw is deliberately excluded because replay
showed it erases the IMU correction after a spin. The corrected IMU relay publishes
only new samples, validates the original source-stamp ordering, and restamps
the EKF-only derivative at local receipt time because canonical JT128 stamps
currently trail system time by a variable 0.68-2.86 seconds. It does not use
the old `0.8` Mahalanobis rejection gate. This profile is diagnostic-only after
the failed repeatability gate recorded in
`docs/phase_ranger_imu_primary_local_ekf.md`.

The IMU-primary profile passed live positive/negative spin shadow tests and a production
two-point field run on 2026-07-10. Spin estimates matched corrected IMU within
`0.05deg`; the two navigation legs finished within `5.60cm/0.61deg` and
`5.49cm/0.30deg`, but a repeated return leg later diverged by about `44.2deg`
between local and wheel yaw and degraded to `35.6cm/13.2deg`. The runtime was
therefore rolled back and remains diagnostic-only.

`LOCAL_STATE_EKF_PROFILE=wheel_imu_pose_soft_yaw_015` keeps the default EKF
fusion fields, but starts the wheel odom preprocessor with a softer wheel
yaw-pose covariance floor while leaving wheel yaw-rate at the stable default.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_012` keeps the same default
fusion fields and wheel yaw-pose covariance, but softens only wheel yaw-rate to
0.12 after the 0.15 candidate reduced yaw error but still exceeded the
two-point truth gate. `LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_010`
keeps wheel yaw-pose covariance at the stable default and uses the smallest
yaw-rate-only step above the stable `0.08` default.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_015`
uses the same shape with a 0.15 yaw-rate floor.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_yaw_offset_m061` keeps the default EKF fusion
fields but starts the wheel odom preprocessor with
`local_state_wheel_odom_ekf_yaw_offset_m061.yaml`, applying a diagnostic
`-0.061 rad` planar yaw offset and rotating the anchored wheel odom position
with the same offset.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062` keeps the default EKF fusion
fields but starts the wheel odom preprocessor with
`local_state_wheel_odom_ekf_xy_shear_p062.yaml`, applying a diagnostic `+0.062`
`y -> x` shear to anchored wheel odom position.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m061` keeps the default EKF fusion
fields but starts the wheel odom preprocessor with
`local_state_wheel_odom_ekf_xy_lateral_m061.yaml`, applying a diagnostic
`-0.061` `x -> y` body-lateral shear to anchored wheel odom position.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m040` keeps the same stable yaw
handling and applies a smaller diagnostic `-0.040` `x -> y` body-lateral shear
to test whether partial lateral correction can preserve yaw.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m050` keeps the same stable yaw
handling and applies an interpolated diagnostic `-0.050` `x -> y`
body-lateral shear between the `-0.040` and `-0.061` candidates.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m085` keeps the same stable yaw
handling and applies a midpoint diagnostic `-0.085` `x -> y` body-lateral shear
between the useful `-0.061` candidate and the over-correcting `-0.120`
candidate.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m120` keeps the same stable yaw
handling but applies a stronger diagnostic `-0.120` `x -> y` body-lateral shear
to test whether the two-point lateral trend continues without changing yaw
covariance.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_soft_yaw_016` keeps that
body-lateral correction and also raises wheel yaw/yaw-rate covariance floors to
`0.16`, combining the best single-direction lateral and soft-yaw evidence for a
controlled A/B test. `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_yaw_p979_n1011`
keeps that body-lateral correction and also scales anchored wheel yaw by sign:
positive yaw by `0.979` and negative yaw by `1.011`, with wheel yaw-rate scaled
the same way.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_018` keeps the default EKF fusion
fields, but starts the wheel odom preprocessor with `0.18` wheel yaw and
yaw-rate covariance floors. `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_016`
uses the same fusion fields with `0.16` yaw/yaw-rate floors after the `0.15`
candidate nearly passed the two-point gate but still exceeded the true
relocalization threshold. `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_014`
uses the same fusion fields with slightly firmer `0.14` yaw/yaw-rate floors.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_010` uses the same fusion fields
with the smallest soft-yaw step above the stable `0.08` yaw/yaw-rate floors.
`LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_015` keeps the default EKF fusion
fields, but starts the wheel odom preprocessor with moderately softer wheel yaw
and yaw-rate covariance floors. `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw` is
the more aggressive 0.25 diagnostic variant. These profiles let corrected JT128
IMU yaw-rate have more influence without removing wheel yaw.
`LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_vyaw` keeps chassis-integrated wheel x/y
pose as the local position anchor and wheel forward speed as the speed input,
but lets corrected JT128 IMU gyro be the only yaw-rate input. It deliberately
drops wheel yaw pose anchoring and is diagnostic-only.
`LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_yaw` keeps wheel x/y pose and wheel+IMU
yaw-rate, but does not fuse chassis-integrated yaw pose.
`LOCAL_STATE_EKF_PROFILE=twist_imu` integrates local x/y/yaw from wheel feedback
twist plus corrected JT128 IMU yaw-rate and intentionally does not fuse
chassis-integrated wheel pose. Use `twist_imu` only for controlled diagnostics,
because it changes the local odom anchoring behavior seen by Nav2 and
`robot_localization_bridge`.
`LOCAL_STATE_EKF_PROFILE=twist_imu_vyaw_only` is stricter than the other IMU
yaw-rate profiles: it uses wheel forward speed, but does not fuse wheel pose
`x/y/yaw` or wheel yaw-rate; corrected JT128 IMU gyro is the only yaw-rate
measurement. It is diagnostic-only after a 2026-07-08 spin test showed it can
leave `/local_state/odometry` with stale EKF yaw-rate after spin stop.
`LOCAL_STATE_EKF_PROFILE=twist_wheel_yaw_imu` also drops chassis-integrated
wheel x/y pose, but keeps wheel yaw pose as the heading anchor while propagating
x/y from wheel forward speed. It is a narrower diagnostic for cases where yaw
is acceptable but post-relocalization truth shows lateral XY drift.

`LOCAL_STATE_EKF_PROFILE=wheel_only` uses `local_state_ekf_wheel_only.yaml`,
starts the wheel odom preprocessor, skips EKF IMU fusion, and runs
`robot_localization` from `/wheel/odom_ekf` only. It preserves
`/local_state/odometry` and the single canonical `odom -> base_link` TF owner
for rollback and temporary chassis-odom isolation. The Jetson runtime default lives in
`scripts/jetson/runtime_overlay/config/local_state_ekf_profile.env`.

The raw `/lidar_imu` stream remains high-rate for JT128 and FAST-LIO2 mapping.
For IMU-fusing diagnostic EKF profiles, the EKF input branch is bounded:
`imu_gyro_bias_filter` still reads every raw IMU sample for bias estimation, but
publishes `/lidar_imu_bias_corrected` at 100 Hz by default and
`/local_state/imu_bias` at 10 Hz. The wheel odom preprocessor publishes
`/wheel/odom_ekf` from its timer at 50 Hz with
`publish_on_callback=false`, so it does not double-publish from both callback and
timer. The EKF output remains 50 Hz and remains the only owner of
`odom -> base_link`.

For the Jetson navigation runtime, `/lidar_imu_bias_corrected` is a nav-only
IMU stream. It preserves `/lidar_imu` for FAST-LIO2, but rotates the corrected
angular velocity and acceleration into `base_link` before publication. This
keeps the EKF yaw-rate input in chassis-frame semantics even when the JT128 IMU
is mounted with lidar pitch/yaw offsets. If `imu_link -> base_link` is missing,
the filter drops corrected samples rather than publishing sensor-frame yaw-rate
as chassis yaw-rate. `run_local_state.sh` treats the corrected IMU and bias
topics as startup readiness requirements for IMU-fusing EKF profiles.

FAST-LIO2 local-state remains available for diagnostics with
`LOCAL_STATE_MODE=fastlio`. In that mode FAST-LIO2 consumes canonical
`/lidar_points` and `/lidar_imu`, publishes raw `/Odometry` with its public TF
remapped away, then the C++ `robot_fastlio_mapping/fastlio_odom_bridge_node`
converts that stream to planar `/fastlio/base_odometry`. This package republishes
that odom as `/local_state/odometry` and remains the only canonical owner of
`odom -> base_link`.

## Parameters

- FAST-LIO diagnostic input: `/fastlio/base_odometry`
- FAST-LIO local-state config: `local_state_fastlio.yaml`
- EKF output topic: `/local_state/odometry` via `/odometry/filtered` remap
- Production spin-aware EKF config: `local_state_ekf_wheel_spin_imu.yaml`,
  selected by `LOCAL_STATE_EKF_PROFILE=wheel_spin_imu`, uses corrected wheel
  `x`, `y`, `yaw`, and `vx` plus corrected IMU `vyaw`; wheel `vyaw` is excluded
  to avoid applying the same spin feedback twice. The paired preprocessor
  config is `local_state_wheel_odom_ekf_spin_imu.yaml`.
- Previous bounded wheel+IMU EKF config: `local_state_ekf.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=wheel_imu`, remains the immediate rollback profile
  and uses planar wheel `x`, `y`, `yaw`, `vx`, and `vyaw` plus corrected IMU
  `vyaw`.
- Wheel-pose+IMU-yaw-rate experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_pose_imu_vyaw`, uses
  `local_state_ekf_wheel_pose_imu_vyaw.yaml` to fuse wheel `x`, `y`, `yaw`, and
  `vx` plus corrected IMU `vyaw`; it does not fuse wheel `vyaw`
- Pose-soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_pose_soft_yaw_015`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_pose_soft_yaw_015.yaml`, raising wheel yaw
  pose covariance from `0.08` to `0.15` while keeping wheel `vyaw` covariance at
  `0.08`
- Conservative twist-soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_012`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_twist_soft_yaw_012.yaml`, keeping wheel yaw
  pose covariance at `0.08` while raising wheel `vyaw` covariance to `0.12`
- Smallest twist-soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_010`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_twist_soft_yaw_010.yaml`, keeping wheel yaw
  pose covariance at `0.08` while raising wheel `vyaw` covariance to `0.10`
- Twist-soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_015`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_twist_soft_yaw_015.yaml`, keeping wheel yaw
  pose covariance at `0.08` while raising wheel `vyaw` covariance to `0.15`
- Planar yaw-offset calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_yaw_offset_m061`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_yaw_offset_m061.yaml`, applying
  `odom_yaw_offset_rad=-0.061` and rotating anchored wheel `x/y` with that
  offset
- Planar shear calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_shear_p062`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_xy_shear_p062.yaml`, applying
  `odom_position_y_to_x_shear=0.062`
- Body-lateral shear calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m061`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_xy_lateral_m061.yaml`, applying
  `odom_position_x_to_y_shear=-0.061`
- Small body-lateral shear calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m040`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_xy_lateral_m040.yaml`, applying
  `odom_position_x_to_y_shear=-0.040` while keeping wheel yaw and wheel `vyaw`
  covariance floors at the stable `0.08`
- Interpolated body-lateral shear calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m050`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_xy_lateral_m050.yaml`, applying
  `odom_position_x_to_y_shear=-0.050` while keeping wheel yaw and wheel `vyaw`
  covariance floors at the stable `0.08`
- Midpoint body-lateral shear calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m085`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_xy_lateral_m085.yaml`, applying
  `odom_position_x_to_y_shear=-0.085` while keeping wheel yaw and wheel `vyaw`
  covariance floors at the stable `0.08`
- Stronger body-lateral shear calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_m120`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_xy_lateral_m120.yaml`, applying
  `odom_position_x_to_y_shear=-0.120` while keeping wheel yaw and wheel `vyaw`
  covariance floors at the stable `0.08`
- Body-lateral plus soft-yaw calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_soft_yaw_016`, uses the
  production `local_state_ekf.yaml` fusion fields but starts the wheel odom
  preprocessor with `local_state_wheel_odom_ekf_xy_lateral_soft_yaw_016.yaml`,
  applying `odom_position_x_to_y_shear=-0.061` and raising wheel yaw and wheel
  `vyaw` covariance floors to `0.16`
- Body-lateral plus yaw sign-scale calibration profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_xy_lateral_yaw_p979_n1011`, uses the
  production `local_state_ekf.yaml` fusion fields but starts the wheel odom
  preprocessor with `local_state_wheel_odom_ekf_xy_lateral_yaw_p979_n1011.yaml`,
  applying `odom_position_x_to_y_shear=-0.061`,
  `odom_yaw_scale_positive=0.979`, and `odom_yaw_scale_negative=1.011`
- `0.18` soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_018`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_soft_yaw_018.yaml`, raising wheel yaw and
  wheel `vyaw` covariance floors from `0.08` to `0.18`
- `0.16` soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_016`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_soft_yaw_016.yaml`, raising wheel yaw and
  wheel `vyaw` covariance floors from `0.08` to `0.16`
- `0.14` soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_014`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_soft_yaw_014.yaml`, raising wheel yaw and
  wheel `vyaw` covariance floors from `0.08` to `0.14`
- `0.10` soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_010`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_soft_yaw_010.yaml`, raising wheel yaw and
  wheel `vyaw` covariance floors from `0.08` to `0.10`
- Soft-yaw experimental EKF profile: `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw`,
  uses the production `local_state_ekf.yaml` fusion fields but starts the wheel
  odom preprocessor with `local_state_wheel_odom_ekf_soft_yaw.yaml`, raising
  wheel yaw and wheel `vyaw` covariance floors from `0.08` to `0.25`
- Moderate soft-yaw experimental EKF profile:
  `LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_015`, uses the production
  `local_state_ekf.yaml` fusion fields but starts the wheel odom preprocessor
  with `local_state_wheel_odom_ekf_soft_yaw_015.yaml`, raising wheel yaw and
  wheel `vyaw` covariance floors from `0.08` to `0.15`
- IMU yaw-rate experimental EKF config:
  `local_state_ekf_wheel_xy_imu_vyaw.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_vyaw`, uses wheel `x`, `y`, and `vx`
  plus corrected IMU `vyaw`; it does not fuse wheel yaw pose or wheel `vyaw`
- Conservative experimental EKF config:
  `local_state_ekf_wheel_xy_imu_yaw.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_yaw`, uses wheel `x`, `y`, `vx`, and
  `vyaw` plus corrected IMU `vyaw`; it does not fuse wheel yaw pose
- Differential wheel-XY experimental EKF config:
  `local_state_ekf_wheel_xy_diff_yaw_imu.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=wheel_xy_diff_yaw_imu`, keeps wheel yaw pose, wheel
  `vx`, wheel `vyaw`, and corrected IMU `vyaw`, but fuses wheel `x`/`y` only
  as differential motion so wheel pose cannot act as an absolute x/y anchor
- Experimental twist-only EKF config: `local_state_ekf_twist_imu.yaml`,
  selected by `LOCAL_STATE_EKF_PROFILE=twist_imu`, uses wheel `vx` and `vyaw`
  plus corrected IMU `vyaw`; it does not fuse wheel pose `x`, `y`, or `yaw`
- Diagnostic twist+IMU-only-yaw-rate EKF config:
  `local_state_ekf_twist_imu_vyaw_only.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=twist_imu_vyaw_only`, uses wheel `vx` plus corrected
  IMU `vyaw`; it does not fuse wheel pose `x`, `y`, `yaw`, or wheel `vyaw`.
  Do not use it as a field default until EKF yaw-rate reset behavior is fixed.
- Experimental twist+wheel-yaw EKF config:
  `local_state_ekf_twist_wheel_yaw_imu.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=twist_wheel_yaw_imu`, uses wheel yaw pose, wheel
  `vx`, wheel `vyaw`, and corrected IMU `vyaw`; it does not fuse wheel pose
  `x` or `y`
- Wheel-only EKF config: `local_state_ekf_wheel_only.yaml`, selected by
  `LOCAL_STATE_EKF_PROFILE=wheel_only`, removes `imu0` entirely for temporary
  chassis odom isolation
- `local_state_wheel_odom_ekf.yaml`: applies nonzero pose covariance floors,
  including a bounded yaw covariance floor, so zero-covariance Ranger odometry
  anchors heading without completely dominating IMU yaw-rate smoothing
- `anchor_pose_to_first_sample`: enabled for the `/wheel/odom_ekf`
  preprocessor, subtracting the first wheel odom x/y/yaw and rotating later
  positions into that local frame before EKF fusion
- `imu0`: `/lidar_imu_bias_corrected`, using gyro `vyaw`
- `local_state_wheel_odom_ekf.yaml`: publishes `/wheel/odom_ekf` from
  `/wheel/odom` with covariance floors so zero-covariance Ranger messages do
  not dominate IMU fusion
- `local_state_imu_bias_filter.yaml`: estimates gyro bias while
  `/wheel/odom_ekf` and `/cmd_vel_safe` indicate the robot is stationary,
  publishes corrected IMU on `/lidar_imu_bias_corrected`, rotates the nav
  output to `base_link`, and publishes the current bias on `/local_state/imu_bias`
- `two_d_mode`: `true`
- `world_frame`: `odom`
- `publish_tf`: `true`, so this package remains the only `odom -> base_link` owner
- IMU linear acceleration is intentionally not fused in the first production profile
- `odom_yaw_offset_rad`: optional diagnostic planar correction applied by the
  wheel-only C++ passthrough node before publishing canonical
  `/local_state/odometry`. Field runtime keeps it `0.0` because Ranger
  `/wheel/odom` is treated as the chassis odometry truth.
- `rotate_odom_position_with_yaw_offset`: when `true`, the passthrough node
  rotates both odom-plane position and base yaw. Keep it `false` when correcting
  only the native chassis child-frame convention while preserving the same odom
  origin.
- `input_base_frame`: expected chassis frame in `/wheel/odom`, defaults to
  canonical `base_link`. The node always republishes canonical `base_link`.

## TF Contract

- This package is the sole publisher of `odom -> base_link`
- No map frame publication is allowed here
- Jetson default navigation runtime starts the wheel odom preprocessor, IMU
  bias filter, and `robot_localization` EKF through
  `scripts/jetson/runtime_overlay/scripts/run_local_state.sh`
- In FAST-LIO mode, `run_local_state.sh` checks endpoint registration once at
  startup. It does not run a periodic ROS graph endpoint self-monitor, so a
  transient graph probe miss during Nav2 startup cannot stop the canonical
  odom owner.
- `run_local_state.sh` stops the FAST-LIO odom bridge, `local_state_node`,
  and EKF fallback children with bounded INT/TERM/KILL waits so a ROS shutdown
  hang cannot leave a process alive after its ROS endpoints have disappeared.
- To run the wheel-pose+IMU EKF default explicitly, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu`
- To run the guarded IMU-primary yaw-rate candidate, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_primary_vyaw`
- To run wheel pose with IMU-only yaw-rate, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_pose_imu_vyaw`
- To run the default EKF with softer wheel yaw-pose covariance only, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_pose_soft_yaw_015`
- To run the default EKF with conservative softer wheel yaw-rate covariance only, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_012`
- To run the default EKF with the smallest softer wheel yaw-rate covariance only, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_010`
- To run the default EKF with softer wheel yaw-rate covariance only, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_twist_soft_yaw_015`
- To run the default EKF with `0.18` soft-yaw covariance, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_018`
- To run the default EKF with `0.14` soft-yaw covariance, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_014`
- To run the default EKF with `0.10` soft-yaw covariance, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_010`
- To run the default EKF with softer wheel yaw covariance, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw`
- To run the default EKF with moderate soft-yaw covariance, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu_soft_yaw_015`
- To run the field default explicitly, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_imu`
- To run the conservative wheel-x/y+IMU-yaw EKF experiment, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_xy_imu_yaw`
- To run the experimental wheel-twist+IMU EKF, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=twist_imu`
- To run the experimental wheel-twist+IMU-only-yaw-rate EKF, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=twist_imu_vyaw_only`
- To run the experimental wheel-twist+wheel-yaw+IMU EKF, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=twist_wheel_yaw_imu`
- To run EKF from chassis odometry only, set
  `LOCAL_STATE_MODE=ekf LOCAL_STATE_EKF_PROFILE=wheel_only`
- To run FAST-LIO local-state for diagnostics, set `LOCAL_STATE_MODE=fastlio`
- To run the wheel-only passthrough wrapper for diagnostics, set `LOCAL_STATE_MODE=passthrough`
- The upstream Ranger driver is started with `base_frame=base_link` and does
  not publish odom TF; this package remains the only owner of `odom -> base_link`.
- The Jetson passthrough profile uses `odom_yaw_offset_rad=0.0` and
  `rotate_odom_position_with_yaw_offset=false`: it republishes Ranger
  `/wheel/odom` as canonical `/local_state/odometry` without changing the SDK
  heading.
