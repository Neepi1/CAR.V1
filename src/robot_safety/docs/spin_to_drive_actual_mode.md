# Actual-mode spin-to-drive handoff

## Scope and field evidence (2026-09-08)

The sidepass capture
`/tmp/njrh_reports/navigation_obstacle_20260908T101746Z_sidepass_v3_LuFp9z`
contains a slow Ackermann command around 10:18:14 UTC with
`vx=0.024011962 m/s, wz=-0.021120096 rad/s`, actual mode `0`.
The former command classifier treated it as pure spin because `|vx|<0.03`
and `|wz|>=0.02`. The following `vx=0.04235` request could therefore be held
at zero despite never entering actual SPINNING. An isolated replay against the
old executable reproduced six zero outputs out of six examined drive outputs.

This fix removes that false hold; it does not claim to resolve the separate
low raw-MPPI speed or collision-monitor slowdown seen in the same trace.

## State contract

- Only `actual_motion_mode` with `available=true`, `fresh=true`, and a known
  chassis code can update the existing mode cache. Desired mode is not evidence.
- An observed entry into actual SPINNING (`2`), including the first valid
  observation, arms one pending spin-tail episode and resets stability counters.
- Repeated `2` heartbeats do not reset counters or the translation hold timeout.
  A consumed episode cannot rearm until another actual non-spin-to-spin entry.
- Rotation and zero requests pass through this particular check. They neither
  arm it from their velocities nor consume it while spin startup is still idle.
- The next translation request uses the unchanged wheel/optional local-odom/IMU
  settling checks. The default timeout remains two seconds from the first held
  translation request; feedback heartbeats cannot prolong it.
- Actual mode exit does not prove physical yaw has stopped. An observed episode
  remains pending across mode exit or feedback loss until settling/timeout.
- Release does not require actual Ackermann confirmation: the downstream driver
  needs the translation command to request that mode in the first place.
- Missing feedback with no previously observed spin does not arm a new hold.
  A complete spin cycle absent from feedback cannot be reconstructed by this
  change; no velocity-based guess or new feedback-availability stop is added.
- `spin_to_drive_linear_epsilon_mps=0.03` retains its existing use as a
  translation-request threshold. Its value and other handoff parameters are
  unchanged. The existing status subscription remains active if either spin
  settling or the separate mode-exit guard needs it.

No MPPI, path repair, footprint, StopZone, localization, TF, arm, elevator or
docking algorithm is modified. This shared final-output correction applies to
normal, API and docking command sources without changing their priority rules.

## Isolated regression

The test drives the real executable with synthetic input, mode, wheel and IMU
messages. The affected settle feature and its production defaults stay enabled.
Checks include slow arcs across 0.03, stale/unavailable/unknown feedback,
lateral modes, real spin preservation, stable-tail release, repeated heartbeat,
mode exit, feedback loss, timeout, subsequent spin, command sources, estop and
watchdog. Run with the mode-exit guard both enabled and disabled.

Run on the Jetson host, **not as a live-runtime probe**. The Docker network
namespace has only loopback, private IPC, no device mappings and a read-only
workspace mount. The fixture refuses to run outside that isolation/domain.
Use an already built binary; never start a chassis driver in this container.

```bash
workspace=/home/nvidia/workspaces/njrh-v3/workspace1
container_workspace=/workspaces/njrh-v3/workspace1
image_id=$(docker inspect NJRH-car --format '{{.Image}}')
docker run --rm --network none --runtime runc --entrypoint bash \
  -e ROS_DOMAIN_ID=183 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --mount "type=bind,source=$workspace,target=$container_workspace,readonly" \
  "$image_id" -c '
    source /opt/ros/humble/setup.bash
    source /workspaces/njrh-v3/workspace1/install/local_setup.bash
    timeout 50s python3 /workspaces/njrh-v3/workspace1/src/robot_safety/test/isolated_spin_to_drive_smoke.py \
      --binary /workspaces/njrh-v3/workspace1/install/robot_safety/lib/robot_safety/robot_safety_node \
      --mode-exit-guard true
  '
```

Repeat with `--mode-exit-guard false`. Existing package GTests and the isolated
normal-lateral watchdog smoke should also pass; the latter must run in the same
network-none isolation, not in the production container.

## Deployment and hardware acceptance

Build only `robot_safety`, compare local/Jetson source and deployed binary
SHA256, and retain the previous executable in `/tmp/njrh_reports`. Replacing
the executable does not replace the image already mapped by a running process.
The fix takes effect only after the user authorizes a whole-runtime restart.
Never individually restart safety/Nav2 or issue navigation commands as part of
this test.

Still requires separately authorized supervised field acceptance: low-speed
Ackermann sidepass, real spin followed by translation, and predock spin handoff.
Use the existing recorder and verify the actual mode plus the two command
boundaries; distinguish this fix from independent MPPI/SlowZone behavior.
