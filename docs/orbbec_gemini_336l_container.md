# Orbbec Gemini 336L in `NJRH-car`

## Scope

The Gemini 336L ROS 2 driver is owned by the reproducible `njrh-car:latest`
image and uses the exact packages already validated on the Jetson host. No
package is downloaded while building the Orbbec-only image layer. The camera
is the resident product docking backend; GS2 is retained only as an explicit
rollback profile.

The installation survives container recreation. The container receives the
host USB bus as a directory bind, so SDK reset/re-enumeration does not require
another container rebuild or restart.

## Validated hardware and software

| Item | Value |
|---|---|
| USB product | `2bc5:0807 Orbbec Gemini 336L` |
| Validated serial | `CPC8563000LM` |
| USB link | `USB3.2` |
| Camera firmware | `1.4.60` |
| ROS wrapper / Orbbec SDK | `2.8.6` |
| ROS distribution | `Humble` |
| Container architecture | `arm64` |
| Container | `NJRH-car` |

Runtime ownership is matched by the dedicated `camera336l` process identity.
The generic `orbbec_camera` executable name is intentionally insufficient:
another Orbbec used by arm/vision may run at the same time and must not make
the docking 336L appear ready when its commissioned process is absent.

## Active close-range preset

The active runtime selects Orbbec's `G336X AMR Default v0.0.5` preset. The
first A/B capture kept the validated `848x480@30 Y16` stream and showed no
near-range improvement. Because Orbbec recommends `1280x800`, `640x400`, or
`424x266` for this AMR preset, the second A/B profile uses `424x266@30`; camera
extrinsic, docking ROI, and detector parameters remain unchanged. The bundled
preset is pinned to SHA-256
`db8f907e14380207b20b9f08018ac9a1aa2597894e234154141e2186fa355e3e`.

Selection is owned by
`scripts/jetson/runtime_overlay/config/orbbec_depth_preset.env`. To roll back,
change its only active value to:

```bash
NJRH_ORBBEC_DEVICE_PRESET="Default"
NJRH_ORBBEC_DEPTH_WIDTH=848
NJRH_ORBBEC_DEPTH_HEIGHT=480
NJRH_ORBBEC_DEPTH_FPS=30
```

Then restart the complete product chain with
`sudo systemctl restart njrh-runtime.service`; do not restart only the camera
process. In `Default`, the launcher does not upload the AMR preset file unless
an explicit `NJRH_ORBBEC_PRESET_FIRMWARE_PATH` override is supplied.

AMR extends the nominal near-depth range but does not guarantee valid depth on
the dock's black, reflective, recessed, or occluded surfaces. Activation is
accepted only when an A/B capture at the same static pose demonstrates useful
near pixels and stable geometry; depth scale must then be rechecked at the
calibration distances.

Vendored host-cache packages:

- `ros-humble-orbbec-camera-msgs_2.8.6-1jammy.20260610.135119_arm64.deb`
- `ros-humble-orbbec-description_2.8.6-1jammy.20260610.135452_arm64.deb`
- `ros-humble-orbbec-camera_2.8.6-1jammy.20260610.141607_arm64.deb`
- `ros-humble-diagnostic-updater_4.0.7-1jammy.20260605.154445_arm64.deb`

The last package repairs an incomplete older container installation that had
the package recorded as installed but did not contain
`/opt/ros/humble/lib/libdiagnostic_updater.so`. That missing library prevented
`liborbbec_camera.so` from loading.

## Emergency offline repair

Run these commands on the Jetson host. They use only the host APT cache.

```bash
docker cp /var/cache/apt/archives/ros-humble-orbbec-camera-msgs_2.8.6-1jammy.20260610.135119_arm64.deb NJRH-car:/tmp/
docker cp /var/cache/apt/archives/ros-humble-orbbec-description_2.8.6-1jammy.20260610.135452_arm64.deb NJRH-car:/tmp/
docker cp /var/cache/apt/archives/ros-humble-orbbec-camera_2.8.6-1jammy.20260610.141607_arm64.deb NJRH-car:/tmp/
docker cp /var/cache/apt/archives/ros-humble-diagnostic-updater_4.0.7-1jammy.20260605.154445_arm64.deb NJRH-car:/tmp/

docker exec -u root NJRH-car bash -lc '
  dpkg -i \
    /tmp/ros-humble-diagnostic-updater_4.0.7-1jammy.20260605.154445_arm64.deb \
    /tmp/ros-humble-orbbec-camera-msgs_2.8.6-1jammy.20260610.135119_arm64.deb \
    /tmp/ros-humble-orbbec-description_2.8.6-1jammy.20260610.135452_arm64.deb \
    /tmp/ros-humble-orbbec-camera_2.8.6-1jammy.20260610.141607_arm64.deb
  ldconfig
'
```

Check the component before starting it:

```bash
docker exec NJRH-car bash -lc '
  set +u
  source /opt/ros/humble/setup.bash
  ldd /opt/ros/humble/lib/liborbbec_camera.so | grep "not found" && exit 1 || true
  ros2 pkg prefix orbbec_camera
'
```

## Manual validation launch

The validation launch intentionally disables point clouds and TF. Camera
extrinsics must remain repository-owned and single-sourced before this device
is added to the production runtime.

```bash
docker exec -it NJRH-car bash
source /opt/ros/humble/setup.bash

ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=camera336l \
  serial_number:=CPC8563000LM \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_d2c_viewer:=false \
  publish_tf:=false
```

Expected primary topics:

- `/camera336l/color/image_raw`
- `/camera336l/color/camera_info`
- `/camera336l/depth/image_raw`
- `/camera336l/depth/camera_info`
- `/camera336l/device_status`

The field validation observed one publisher for each image stream, about
`17.2 Hz` color and `14.8 Hz` depth with a temporary Python subscriber. The
configured sensor profiles reported by the driver were
`1280x720@30 MJPG` color and `848x480@30 Y16` depth. Subscriber-side rates are
lower than configured sensor rates and are not yet a production performance
acceptance result.

## Runtime status

- Driver packages: Orbbec 2.8.6 is part of the reproducible `njrh-car:latest` image build.
- USB device access: verified.
- Color/depth publication: verified.
- Depth-only runtime wrapper: enabled by the resident systemd runtime.
- Dock target adapter: enabled in `robot_docking_perception`.
- Camera TF/extrinsics: final-mount values are configured and static/range/lateral acceptance is complete.
- Product docking backend: `orbbec_336l`; `NJRH_GS2_AUTOSTART=false`.

## USB re-enumeration contract

The Gemini 336L may receive a new USB device number when the SDK initializes or
resets it, for example `/dev/bus/usb/002/033` becoming `002/034`. A privileged
container alone does not keep its original `/dev` snapshot synchronized with
new host device nodes. `njrh_container.sh` therefore bind-mounts the complete
host `/dev/bus/usb` directory into the same container path. Binding a single
device node is prohibited because it becomes stale after the next reset.

The four validated ARM64 Debian packages are stored under
`scripts/jetson/vendor/orbbec/` and installed by `Dockerfile.car`; installing
them manually into a running container is not a production installation.
After changing either the package set or USB mount contract, rebuild the image,
recreate `NJRH-car` once, and restart the complete `njrh-runtime.service` chain.
Use `bash scripts/jetson/njrh_container.sh rebuild-image`; it validates the
vendored checksums and creates a minimal temporary Docker context instead of
traversing reports and root-owned test artifacts in the working tree.
When adding the camera to an already validated field image, use
`bash scripts/jetson/njrh_container.sh build-orbbec-layer` instead. Its
`Dockerfile.orbbec-runtime` derives from the current `njrh-car:latest`, installs
only the four local packages with `dpkg`, verifies the linked driver has no
missing shared libraries, and performs no APT operation; this
is the required path when Nav2 and other ROS package versions must remain
bit-for-bit unchanged.

Acceptance requires three consecutive driver start/stop cycles. Every cycle
must report `device_online=true`, `connection_type=USB3.2`, publish fresh depth
and `CameraInfo`, and leave no camera process after shutdown. If the host device
number changes during a cycle, the running container must expose the new node
without being restarted.

Field acceptance on 2026-07-15 used image
`sha256:2345545cf8b909d7b8fd0858af183fa0e801e59633b55f11c228b69b14253a3a`.
The complete runtime reached `nav2_layer_ready` in 31 seconds before testing.

| Cycle | USB evidence | Driver/status evidence | Stream evidence |
|---:|---|---|---|
| 1 | start `002/036`, connected as `002/037`; new node visible inside the unchanged container | online, `USB3.2` | fresh depth image and `CameraInfo` |
| 2 | previous shutdown re-enumerated `002/039`; restart connected without container recreation | online, `USB3.2` | fresh depth image and `CameraInfo` |
| 3 | start `002/040`, connected as `002/041`; new node visible inside the unchanged container | online, `USB3.2` | fresh depth image and `CameraInfo` |

All camera launch and component processes were stopped after the third cycle.

## Docking backend architecture

The camera adapter owns perception only. It does not include `Twist`, does not
publish a velocity topic, and cannot bypass the docking/safety state machines.

```text
/camera336l/depth/image_raw + /camera336l/depth/camera_info
  -> robot_docking_perception/orbbec_depth_dock_node
  -> /dock/target_observation (DockTargetObservation, base_link convention)
  -> robot_docking_manager
  -> /cmd_vel_docking
  -> robot_safety
  -> ranger_base
```

`robot_api_server` also consumes `/dock/target_observation` for the fine-docking
entry gate. Camera mode disables blind forward approach: stale, unhealthy, or
invalid target geometry holds zero speed and cannot start the fine approach.
The default `NJRH_DOCKING_SENSOR_BACKEND=orbbec_336l` is rejected at startup
unless all six camera extrinsic values are present and a fresh
`/dock/target_observation` arrives. Set the backend explicitly to `gs2` only
for controlled rollback; normal runtime also sets `NJRH_GS2_AUTOSTART=false`.

## Intrinsics policy

Manual intrinsic calibration is not an activation prerequisite for the current
depth-only, center-ROI docking use case. The adapter consumes the factory
`sensor_msgs/CameraInfo` delivered with every stream instead of hard-coding a
matrix. The validated `848x480` profile reported approximately
`fx=416.82`, `fy=416.82`, `cx=422.68`, and `cy=238.34`.

The node rejects invalid calibration, image/CameraInfo resolution mismatch, and
frame mismatch. Recalibrate the depth intrinsics only if the lens/cover is
changed, the factory CameraInfo no longer matches the selected profile, or a
flat-board test shows repeatable image-position-dependent error. RGB intrinsics
and RGB-to-depth extrinsics are irrelevant while RGB and depth registration are
disabled.

Before activation, verify depth scale against a flat target at about 0.30 m,
0.50 m, and 0.80 m. The initial acceptance bound is the larger of 10 mm or
1.5 percent of reference distance. A constant scale error belongs in the depth
driver/adapter calibration; it must not be hidden in the docking controller.

## Required camera extrinsic

`robot_description` remains the only static-TF owner. The initial final-mount
measurement is configured in the active runtime sensor configuration as:

```yaml
docking_camera_frame: camera336l_link
docking_camera_depth_optical_frame: camera336l_depth_optical_frame
docking_camera_x: 0.394
docking_camera_y: 0.000
docking_camera_z: 0.350
docking_camera_roll: 0.000
docking_camera_pitch: 0.000
docking_camera_yaw: 0.000
```

The X value uses the measured 0.738 m chassis length, a 0.030 m camera depth,
and the observed approximately 0.005 m protective-cover-to-optical-origin
offset: `0.738 / 2 + 0.030 - 0.005 = 0.394 m`. Y is on the vehicle centerline.
Z is the current 0.350 m optical-center height above the project
ground-coincident `base_link`/`base_footprint`; only this height changed from
the initial 0.583 m mount. The zero RPY values describe the current level,
forward-facing mount. These values are a physical calibration candidate and
must pass the static target checks below before the backend is changed from GS2.

The driver stays at `publish_tf=false`. `robot_description` publishes
`base_link -> camera336l_link` from the measured mount and the fixed REP-103
`camera336l_link -> camera336l_depth_optical_frame` rotation. Do not enter rough
or estimated values merely to pass startup; camera mode intentionally fails
closed when these values are absent.

## Activation acceptance

Complete these checks with the final mount before changing the backend:

1. Confirm one and only one static camera TF owner and a fresh matching
   `848x480` depth image/CameraInfo pair.
2. Verify the three-distance depth scale check above.
3. Place a flat dock-face surrogate centered, then left and right of the vehicle
   centerline. Positive reported lateral error must require positive
   `base_link` Y motion; camera mode explicitly uses
   `controller.lateral_command_sign=+1`.
4. Repeat at positive and negative yaw. Positive `yaw_error_rad` must require a
   positive (counter-clockwise) vehicle correction.
5. Require median forward/lateral error within 15 mm and median yaw error within
   2 degrees over repeated static samples before allowing motion.
6. Show a plain wall, a narrow object, and no target. Each must remain invalid;
   `/cmd_vel_docking` must stay zero.
7. Run supervised low-speed docking trials, verify BMS contact hard-stop, and
   confirm the command path remains through `robot_safety`.
8. Record at least 20 consecutive final-mount docking attempts before removing
   GS2 as the rollback backend.

Camera and docking perception are assigned away from lidar point-cloud cores:
the depth driver defaults to CPU0 and the 10 Hz geometry adapter to CPU3. The
adapter does not subscribe to JT128, FAST-LIO2, AMCL, or Nav2 topics.

Both runtime wrappers hold a non-blocking owner lock in `/tmp`. A second camera
or perception launch exits with status 73 before loading another SDK pipeline.
This lock is separate from the `/dev/bus/usb` bind-mount contract: the bind
mount preserves device access across USB re-enumeration, while the owner lock
prevents two processes from opening the same depth stream.

The depth launcher enables the Orbbec device heartbeat by default and injects
an SDK configuration overlay with `StreamFailedRetry=1`,
`MaxStartStreamDelayMs=3000`, and `MaxFrameIntervalMs=3000`. The 2026-07-16
field fit showed that this retry path can recover the depth stream after USB
disconnect/re-enumeration without recreating the container. It does not fix
the underlying USB fault: the host still reported device-number changes and
`usb_submit_urb returned -19`. Production acceptance therefore still requires
camera firmware, cable, connector, and power validation. A stale or invalid
observation remains fail-closed and must publish zero docking velocity. Set
`NJRH_ORBBEC_ENABLE_HEARTBEAT=false` only for a controlled A/B diagnostic.

## Two-surface range campaign

The 2026-07-15 supervised campaign used the final `z=0.350 m` mount and moved
only through `/cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base_node`.
Every range step stopped on fresh fixed-face observations and confirmed zero
wheel velocity before the next static capture.

| Requested head gap | Head median | Fixed-face median | Protrusion median | Result |
|---:|---:|---:|---:|---|
| 0.80 m | 0.8209 m | 0.9335 m | 0.1128 m | accepted, 96.3% valid |
| 0.60 m | 0.5839 m | 0.7144 m | 0.1305 m | accepted, 100% valid |
| 0.50 m | 0.5004 m | 0.6297 m | 0.1292 m | accepted, 96.4% valid |
| 0.40 m | 0.3979 m | 0.5299 m | 0.1319 m | accepted, 100% valid |
| 0.30 m | 0.3189 m | 0.4301 m | 0.1112 m | accepted, 100% valid |
| 0.25 m | not observable | 0.3337 m | not observable | fixed face valid; head in depth blind zone |
| 0.20 m | not attempted | predicted below depth floor | not observable | fail-closed |

Near 0.29 m head gap, the apparent head/face separation fell to about 68 mm.
The former `geometry.front_cluster_window_m=0.08` merged both surfaces and
invalidated the fixed-face fit. The production default and both owned configs
now use `0.04 m`; the fixed face then measured `0.3939 m`, `0.12 deg`, and
confidence `0.91`. The calibration-only head visibility gate is 15 mm and its
head-to-face yaw allowance is 0.25 rad because the narrow telescoping head can
present an angled edge. These relaxations do not apply to vehicle yaw control:
the production observer still requires a 120 mm-wide fixed face and the motion
gate still uses fixed-face yaw.

At the 0.25 m station, expanding the accepted base-height band from
`0.16..0.36 m` to `0.05..0.60 m` did not reveal closer points. Across 28,669
planar samples the minimum observed forward gap was `0.313 m` and the 0.1%
quantile was `0.322 m`. This is the effective near-depth floor of the current
camera/preset/mount, not a software Z crop. Production control therefore uses
the fixed face only through the settled `0.34 m` handoff. During that visible
pre-contact approach, yaw must first settle inside `0.5 deg` for three frames.
The controller then combines forward and lateral translation in Ranger
PARALLEL mode; camera yaw noise below `3.0 deg` does not re-enter SPINNING, and
only one larger yaw recapture is allowed. At the first-contact handoff the
controller locks lateral/yaw commands and performs a `0.05 m/s` straight
contact crawl, slowing to `0.02 m/s` only in the final `0.06 m`. BMS contact
stops it, while odometry travel and a distance-derived time budget bound it.
Stale observation/odometry or loss of hard alignment fails closed; the crawl
uses straight retreat and reacquisition rather than any correction while the
contacts may be touching.

## Bidirectional motion fit

Use the supervised fit only while the camera backend is being calibrated and
the production docking job is inactive. Reverse motion is disabled unless the
operator passes `--enable-reverse` explicitly. The default sweep approaches
the fixed face from 0.80 m to 0.45 m and then returns to 0.80 m at no more than
0.04 m/s forward and 0.03 m/s reverse:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_orbbec_dock_bidirectional_fit.sh \
  --near-fixed-face-gap-m 0.45 \
  --far-fixed-face-gap-m 0.80 \
  --cycles 1 \
  --enable-reverse \
  --label fixed_face_forward_reverse_01
```

The tool requires fresh `/dock/target_observation` and `/wheel/odom`, checks
the live API docking/navigation ownership state, stops on invalid geometry,
e-stop, charging contact, lateral/yaw gate violations, or missing progress,
and confirms wheel zero speed between legs. Reports contain `samples.csv`,
`metrics.json`, and `summary.md`; the per-leg scale is the observed fixed-face
range change divided by wheel-forward displacement and should be close to 1.0.

### 2026-07-16 fit result

Two complete forward/reverse rounds passed with `fit_accepted=true`:

- `20260716T114105Z_fixed_face_forward_reverse_verified_01`
- `20260716T114815Z_fixed_face_forward_reverse_verified_03`

| Direction | Successful legs | Mean range/wheel scale | Population stddev | Mean intercept |
|---|---:|---:|---:|---:|
| Forward | 2 | 1.077868 | 0.002492 | -0.011880 m |
| Reverse | 2 | 1.085047 | 0.000580 | +0.009580 m |
| Combined | 4 | 1.081457 | 0.004020 | - |

All four fits had `R^2` between 0.99766 and 0.99860. The forward/reverse mean
scale difference was 0.664%, and the maximum endpoint absolute residual was
5.7 mm. This establishes repeatable, directionally consistent fixed-face
tracking over the calibrated sweep.

The combined scale is diagnostic evidence, not yet a camera extrinsic or range
correction. It compares camera range change with low-speed wheel displacement;
an independent physical distance datum is required to distinguish depth scale
error from Ranger low-speed odometry scale/quantization before applying any
correction. The approximately -0.052 m lateral result is also not applied until
the vehicle centerline is physically referenced to the dock centerline.

One intermediate run exposed an invalid observation represented by zero-valued
geometry. The fit guard now separates sensor health from geometry validity,
publishes zero velocity for invalid frames, and requires three distinct valid
below-limit observations before a reverse-leg minimum-gap abort. Reusing one
camera frame at the faster control-loop rate cannot advance that counter.

## Lateral motion fit

`run_orbbec_dock_lateral_fit.sh` performs a supervised left/right sweep through
the same safety chain. Motion requires `--enable-motion`; the default 2 cm
positive-body-Y probe must confirm that the camera lateral response has the
expected negative sign before a full sweep is accepted:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_orbbec_dock_lateral_fit.sh \
  --enable-motion \
  --sweep-amplitude-m 0.08 \
  --cycles 1 \
  --response-sign -1 \
  --label lateral_left_right_01
```

For Ranger Mini3, a pure `Twist.linear.y` request is reported by the chassis as
`MOTION_MODE_PARALLEL` (code 1), with the wheel group turned for lateral
translation. `MOTION_MODE_SIDE_SLIP` (code 3) is selected directly only for the
Mini V1 path in the current SDK. Both codes are treated as lateral modes by the
calibration report, but the 2026-07-16 Mini3 samples all used code 1.

The primary motion fit excludes steering-mode transition and stationary
samples. A sample is admitted only when the Ranger-reported lateral mode is
matched and `abs(/wheel/odom.twist.linear.y) >= 0.005 m/s`. The report also
retains an all-sample fit so wheel steering latency and stop tail remain
visible rather than being mistaken for motion scale.

### 2026-07-16 lateral result

The direction probe measured `+0.0273 m` wheel lateral displacement and
`-0.0238 m` camera lateral change, confirming `response_sign=-1`. Two complete
left/right/return sweeps, an equal-distance right/left pair, and one final
motion-filtered sweep supplied 11 admitted lateral legs:

| Direction | Admitted legs | Mean camera-derived/wheel scale | Population stddev |
|---|---:|---:|---:|
| Left (`+Y`) | 7 | 0.993203 | 0.037623 |
| Right (`-Y`) | 4 | 0.988106 | 0.018331 |
| Combined | 11 | 0.991349 | - |

The left/right mean difference was `0.005097`, or about 0.51%. The lowest
motion-only `R^2` was 0.9041 and most legs were between 0.97 and 0.99. Maximum
measured forward cross-coupling was below 1 mm. This does not support a
direction-specific Ranger lateral odometry correction, so no lateral scale or
left/right gain is applied.

The remaining error is concentrated outside steady lateral motion. Wheel
steering/mode transitions and stop tail produced approximately 11--19 mm
endpoint overshoot. A 10 second stationary observation captured 67 valid
frames with lateral standard deviation 3.4 mm and 11.5 mm peak-to-peak range;
the fitted visible face span varied by 41.7 mm. Lateral variation correlated
strongly with visible span (`r=0.925`), fitted yaw (`r=-0.923`), and range
(`r=0.933`). Production docking should therefore use temporal filtering and a
settled multi-frame terminal check; it should not compensate this observation
variation by changing the chassis odometry scale.

Accepted report directories include:

- `20260716T121554Z_lateral_left_right_verified_02`
- `20260716T121644Z_lateral_left_right_verified_03`
- `20260716T122046Z_lateral_equal_10cm_right_01`
- `20260716T122118Z_lateral_equal_10cm_left_01`
- `20260716T122443Z_lateral_left_right_motionfit_04`

## Expanded acquisition envelope

The 2026-07-16 coverage run extended the lateral sweep at an approximately
0.81 m fixed-face gap from the calibrated 8 cm amplitude to 15 cm. The complete
left/right/return sweep remained valid from -0.120 m through +0.198 m observed
lateral error. Per-leg median confidence was 0.875--0.882, maximum wheel-forward
cross-coupling was 0.7 mm, and the return observation was within 5.1 mm of its
starting lateral value. This validates camera acquisition for roughly 20 cm of
predock centerline error; it does not justify a Ranger lateral scale change.

Centerline reverse steps then checked the longer acquisition range:

| Final fixed-face gap | Valid-only range/wheel scale | Valid-only R2 | Use |
|---:|---:|---:|---|
| 1.00 m | 1.1142 | 0.9960 | camera takeover candidate |
| 1.20 m | 1.1007 | 0.9941 | preferred maximum takeover range |
| 1.40 m | 1.1109 | 0.9940 | early detection/lock only |

At 1.40 m, three stationary checks remained valid with confidence 0.827--0.834;
forward gap varied by about 1.3 mm, lateral by about 0.3 mm, and yaw by about
0.47 degrees across the sampled endpoints. The first generated report showed
an incorrect `R2=0.0687` because two explicitly invalid zero-geometry frames
were included in the regression. The report path now admits only healthy,
valid observations from `orbbec_336l_depth` with finite positive geometry and
records used/discarded sample counts. Replaying the unchanged 61-frame capture
kept 59 frames and produced the 1.40 m result above.

`max_depth_m=2.0` is an input filter limit, not a precision guarantee. The
current geometry contract stops at `geometry.max_forward_gap_m=1.50`, so 2.0 m
is neither a fine-docking fit range nor an accepted takeover range. Use 1.2 m
as the initial camera-control ceiling until repeated full docking attempts show
a commercial reason to expand it; 1.4 m may be used to identify and hold the
target before control takeover.

Coverage report directories include:

- `20260716T134504Z_lateral_coverage_15cm_02`
- `20260716T134713Z_coverage_range_1p0`
- `20260716T134751Z_coverage_range_1p2`
- `20260716T134828Z_coverage_range_1p4`

## AMR preset close-range result

The official `G336X AMR Default` preset version `0.0.5` is bundled under
`scripts/jetson/runtime_overlay/config/orbbec_presets/` and selected through
`config/orbbec_depth_preset.env`. The active profile is the preset's
recommended low-resolution `424x266 @ 30 Hz` mode. Setting the preset name to
`Default` provides an immediate rollback; a complete runtime restart is
required after changing it.

At the stationary first-contact pose, Default at 848x480 reported no depth
closer than 0.314 m. AMR at 424x266 reduced the minimum nonzero depth to
0.236 m and produced 335,576 pixels in the 0.15--0.25 m band across 523 frames.
It still produced no samples below 0.15 m, so neither the approximately 0.09 m
fixed panel nor the telescoping contacts are observable in the final contact
pose.

AMR consistently exposes the dock's upper protrusion. Across 110 stationary
samples, the fitted span was 0.1132 m, gap MAD was 0.12 mm, lateral MAD was
0.22 mm, and yaw MAD was 0.106 degrees. The runtime minimum span gate is 0.10 m
so this feature is accepted; the old 0.12 m gate rejected it. A live runtime
sample reported `valid=true`, confidence 0.765, 161 inliers, and 4.75 mm RMS.

This feature is suitable for pre-contact lateral/yaw alignment only. The
reported 0.242 m forward gap is relative to `charge_contact_link` and the
visible upper protrusion; it is not the physical insertion stroke. Final
insertion remains bounded straight motion over the measured 0.035 m stroke,
with BMS charging feedback as the stop authority. The detailed report is
`reports/orbbec_docking_calibration/20260716T091016Z_amr424_first_contact_zero_compression/summary.md`.
