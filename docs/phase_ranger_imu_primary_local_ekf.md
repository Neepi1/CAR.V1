# Ranger IMU-Primary Local EKF

## Scope

This phase changes only the Ranger Mini 3 local odometry input branch. It does
not change FAST-LIO2, JT128 pointcloud data, Nav2 plugins, AMCL, or the
`map -> odom` owner.

## Previous Failure

The rejected `twist_imu_vyaw_only` profile had no wheel yaw-rate fallback. A
2026-07-08 spin test showed `/local_state/odometry.angular.z`
remaining near `0.517rad/s` after corrected IMU yaw-rate had returned to zero.
At that rate, prediction alone adds about 29.6 degrees of false yaw per second.

Two input-contract defects amplified that failure:

- `imu0_twist_rejection_threshold=0.8` was a sub-1-sigma Mahalanobis gate and
  could reject a real high-rate-to-zero stop measurement.
- The corrected IMU timer could republish an old sample with its original
  timestamp. If the upstream IMU endpoint disappeared, robot_localization
  continued predicting from its last accepted yaw-rate.

## Candidate Design

`LOCAL_STATE_EKF_PROFILE=wheel_imu_primary_vyaw` uses:

- wheel pose `x/y`, twist `vx`, and low-weight wheel `vyaw` fallback;
- `/lidar_imu_bias_corrected.angular_velocity.z` as the dominant dynamic yaw
  measurement;
- no chassis-integrated absolute wheel yaw measurement;
- wheel yaw-rate covariance floor of `0.25`;
- corrected IMU yaw-rate covariance `0.0025`;
- no pose or twist Mahalanobis rejection gate.

The wheel fallback prevents an IMU delivery loss from leaving EKF yaw-rate
unconstrained. The 100:1 per-sample yaw-rate covariance ratio keeps corrected
IMU dynamics dominant while it is healthy.

A deterministic replay of the 2026-07-10 positive and negative 30-degree spin
captures is the dynamic configuration gate. The first candidate still fused
absolute wheel yaw and ended at `36.195deg` for the positive capture, exactly
matching wheel instead of the IMU reference `35.150deg`. Absolute wheel yaw is
therefore intentionally excluded; retaining it would erase the correction this
profile is meant to provide.

After excluding absolute wheel yaw, time-accurate replay produced:

| capture | wheel | IMU | candidate | candidate - IMU | final candidate vyaw |
|---|---:|---:|---:|---:|---:|
| positive 30 degree | 36.195 deg | 35.150 deg | 34.763 deg | -0.387 deg | 0 rad/s |
| negative 30 degree | -35.677 deg | -34.584 deg | -34.494 deg | +0.090 deg | 0 rad/s |

The replay waits for both DDS subscriptions before publishing and schedules
each row from the recorded `elapsed_sec`; fixed sleeps are not valid because
probe overhead stretches the angular-rate waveform.

The live static shadow gate also passed: over the final 8-second gate,
candidate position and yaw drift were zero, candidate angular velocity stayed
zero, and output was `49.92Hz` while the production EKF and shadow EKF ran
concurrently. The shadow did not publish TF. Corrected IMU stamps were unique
and monotonic, with covariance `0.0025`; the corrected publisher's stale
counter remained at one startup-only event and stopped increasing.

The corrected IMU helper validates that source stamps are monotonic, publishes
each accepted source sample at most once, and stamps the EKF-only derivative at
local receipt time. Field inspection found canonical JT128 stamps trailing
system time by a variable 0.68-2.86 seconds; preserving those stamps caused the
old 0.20-second freshness check to suppress an otherwise live 200Hz+ input.
The helper uses best-effort depth 1 and never republishes an old sample just to
maintain a nominal timer rate. Restamping and covariance override apply only to
`/lidar_imu_bias_corrected`; canonical `/lidar_imu` remains unchanged for
FAST-LIO2.

## Deployment Gate

Static, replay, and live spin gates passed on 2026-07-10. The live shadow
results were:

| direction | wheel | IMU | candidate | candidate - IMU | candidate spin XY |
|---|---:|---:|---:|---:|---:|
| positive | 34.050 deg | 32.627 deg | 32.636 deg | +0.009 deg | 0.7 mm |
| negative | -34.222 deg | -31.789 deg | -31.831 deg | -0.042 deg | 0.6 mm |

Both directions returned candidate yaw-rate to zero, so the profile advanced
to a bounded field A/B test. It was not yet a production acceptance.

The production two-point field gate also passed on 2026-07-10:

| leg | local distance | online XY | online yaw | localization evidence |
|---|---:|---:|---:|---|
| current pose -> `delivery_675235` | 5.99 m | 5.60 cm | 0.61 deg | delayed fresh trigger changed map->odom 4.54 cm / 0.37 deg |
| `delivery_675235` -> `delivery_512355` | 10.88 m | 5.49 cm | 0.30 deg | AMCL last correction 2.4 cm / 0.23 deg; scan-map best offset 0 cm / 0 deg |

Both API goals reached `final_pose_verified`; the second Nav2 action returned
native success. The first Nav2 action returned code 6, but the existing API
terminal correction reached the same 6cm commercial gate. Corrected IMU output
remained 78-91Hz and `stale_skips` stayed at the single startup event.

The automatic post-goal Isaac capture exposed a separate trigger freshness
race: it sometimes returned a localization result stamped just before the
force-accept arm time, so the bridge correctly rejected it as stale. This is
not treated as zero odometry error. The first leg obtained a later fresh result;
the second leg is supported by the small AMCL correction and independent
scan-map score, while the Isaac trigger race remains a separate localization
work item.

## Repeatability Failure And Rollback

A subsequent two-leg run without explicit relocalization invalidated the
promotion:

- leg 1 reached `delivery_675235` at `4.74cm / 1.02deg`, while AMCL rejected
  all observed candidates; the largest candidate was `1.874m`;
- leg 2 degraded at `delivery_512355` with `35.6cm / 13.2deg` residual;
- final normalized local-vs-wheel yaw separation was about `44.2deg`;
- AMCL accepted zero corrections and rejected about 134 candidates, with the
  largest candidate reaching `3.633m`.

Short spin agreement therefore did not generalize to a large, multi-turn
route. The field default was restored to `wheel_only`. The IMU-primary profile
remains available only for bounded diagnostics using the captured failed-leg
data.

A same-route `wheel_only` control run then separated the AMCL issue from the
fusion regression:

| profile / leg | AMCL candidates | accepted | rejected | max candidate | final result |
|---|---:|---:|---:|---:|---|
| IMU-primary -> `675235` | 128 | 0 | 131 | 1.874 m | 4.74 cm / 1.02 deg |
| IMU-primary -> `512355` | 133 | 0 | 134 | 3.633 m | degraded 35.6 cm / 13.2 deg |
| wheel-only -> `675235` | 109 | 54 | 62 | 0.245 m | 5.07 cm / 0.30 deg |
| wheel-only -> `512355` | 117 | 0 | 121 | 1.103 m | 5.78 cm / 0.12 deg |

The count deltas can differ slightly because several bridge events may occur
between observer samples. The directional conclusion is still unambiguous:

- IMU-primary introduced a separate local-yaw divergence and made the worst
  candidate substantially larger;
- AMCL also has an independent false-match problem on the return-to-`512355`
  leg, because wheel-only reached the goal without accepted corrections while
  AMCL continued proposing roughly 0.77-1.10m jumps;
- the current hard gate must not be widened to accept those candidates.

Reports:

- `/tmp/njrh_imu_primary_live/amcl_two_point_20260710T135739Z`
- `/tmp/njrh_wheel_only_live/amcl_two_point_20260710T140803Z`

Remaining gates:

1. Extended spin: positive and negative 90/180 degree tests at 0.2/0.4/0.6
   rad/s show no stale yaw-rate and final candidate-vs-IMU yaw under 0.5 degree.
2. Arc: left/right 1.5m circles do not regress radius or heading.
3. Repeatability: additional two-point cycles preserve the passed endpoint and
   correction bounds without a second `odom -> base_link` transform.

Dynamic tests require explicit authorization to move the robot. The failed
field A/B gate restored `wheel_only` with a complete runtime restart.

## Final Bounded Resolution

The later correction-frozen A/B separated the fusion topology from the Ranger
negative-spin source scale:

| candidate | final truth correction | decision |
|---|---:|---|
| IMU-primary yaw-rate | 23.97 cm / -13.34 deg | rejected |
| unanchored twist-only IMU yaw-rate | 1.023 m / -13.38 deg | rejected |
| wheel anchor + continuous IMU, negative scale 0.977672 | 40.0 cm / -5.74 deg | rejected |
| wheel anchor + continuous IMU, negative scale 1.0 | 71.19 cm / +4.105 deg | rejected; bracketed the sign crossing |
| wheel anchor + continuous IMU, fitted negative scale 0.986000 | 2.84 cm / 1.74 deg | passed repeat 1 |
| same fitted configuration, repeat 2 | 5.70 cm / 1.35 deg | passed with no accepted no-motion correction |

The field default is therefore `wheel_imu`, not IMU-primary: wheel `x/y/yaw`
provides the bounded absolute local pose and corrected IMU yaw-rate shapes the
dynamic response. `wheel_only` remains the rollback/isolation profile. The
positive Ranger spin scale remains `0.976386`; the two-point routes above
exercised the negative sign and do not justify changing the positive value.

The explicit Isaac trigger can still reject a result stamped about 1.08 seconds
before the force-accept arm time while its freshness slack is 1.0 second. That
is a separate localization freshness issue and is not masked by this odometry
configuration.
