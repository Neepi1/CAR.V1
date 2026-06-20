# Phase V1 Navigation Docking Validation

Phase V1 is a validation-only layer for the current N2/D3/R0-R2 runtime
contract. It does not change Nav2 plugins, controller parameters, TF
tolerances, pointcloud QoS/DDS, FAST-LIO2, Ranger odom, EKF fusion, or the
`robot_safety` speed chain.

## Scope

The validation target is:

- normal delivery goals default to `goal_completion_policy=pose_required`;
- `position_only` remains an explicit engineering opt-out;
- `dock_staging` is reserved for `/api/v1/docking/start`;
- manual `/api/v1/localization/trigger` succeeds on bridge-accepted correction
  by default, not on the post-relocalization settle barrier;
- predock yaw alignment is docking-owned and publishes only to
  `/cmd_vel_docking`;
- fine docking entry requires predock yaw alignment, post-predock settle, GS2
  freshness, bridge `map->odom` smoothing completion, and applied
  global-correction pause.
- ordinary navigation final verification may perform at most one same-goal Nav2
  retry after bridge smoothing; API-owned velocity correction remains disabled.

The scripts are intentionally observation-biased. The default runner does not
send navigation goals, docking requests, relocalization triggers, or velocity
commands.

## Scripts

Static/runtime config audit:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_fine_docking_entry_gate.sh
```

Observe a normal `pose_required` navigation that is already being driven by the
operator or App:

```bash
bash scripts/jetson/runtime_overlay/scripts/observe_pose_required_navigation.sh \
  --duration-sec 180
```

Verify the manual relocalization API contract. This is the only V1 script that
intentionally calls `/api/v1/localization/trigger`, and it does not send motion:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_manual_relocalization_api.sh
```

Observe a predock yaw alignment or docking attempt that is already running:

```bash
bash scripts/jetson/runtime_overlay/scripts/observe_predock_yaw_alignment_trace.sh \
  --duration-sec 180
```

Run a predock yaw probe. Its default mode delegates to the read-only observer.
The bounded yaw command is available only with the explicit
`--apply-small-yaw-test` flag and publishes only to `/cmd_vel_docking`:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_predock_yaw_alignment_probe.sh \
  --dry-run
```

Run the combined V1 collection in observe-only mode:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_v1_navigation_docking_validation.sh \
  --observe-only \
  --duration-sec 120
```

For the Phase N4 post-Nav2 final verification retry contract:

```bash
bash scripts/jetson/runtime_overlay/scripts/verify_post_nav2_final_verify_recovery.sh \
  --mock-nav2-succeeded \
  --mock-final-distance 0.269 \
  --mock-yaw-error 0.018 \
  --mock-tolerance 0.2 \
  --expect-retry-xy

bash scripts/jetson/runtime_overlay/scripts/observe_post_nav2_final_verify_recovery.sh \
  --duration-sec 180
```

## Pass Criteria

The combined report writes `allowed_to_run_full_docking_test=true` only when:

- normal `pose_required` observation is ready;
- manual relocalization verification was explicitly included and passed;
- predock yaw probe was explicitly included and passed;
- fine docking entry gate verification passed;
- no command-owner conflict was observed in runtime reports;
- the speed chain remains intact through `robot_safety`.

If the command is run with only `--observe-only`, manual relocalization and the
predock yaw probe remain `UNKNOWN`, so full docking is not cleared by that run
alone.

## Hardware Validation

Required field checks before a full docking test:

- run the observe-only V1 runner for at least 120 seconds while the runtime is
  active;
- run manual relocalization verification only while the robot is stationary and
  localization recovery is expected;
- run the predock yaw probe with motion only in an open area with human stop
  supervision;
- confirm `/cmd_vel_docking -> robot_safety -> /cmd_vel_safe -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base_node`;
- confirm no hidden `/global_localization/trigger` is mixed into normal
  navigation or predock staging.

Rollback is removing these validation scripts and documentation. No production
runtime behavior is changed by Phase V1.
