# robot_safety

Final command arbitration point before the chassis bridge.

## Canonical Contract

- normal navigation/App command input: `/cmd_vel_collision_checked`
- docking command input: `/cmd_vel_docking`
- final chassis command output: `/cmd_vel`
- safe command mirror: `/cmd_vel_safe`
- estop input: `/safety/estop`
- optional localization gate: `/localization/health`
- state output: `/safety/status`
- motion-allowed output: `/safety/motion_allowed`

## Arbitration Policy

`robot_safety` is the only package allowed to publish the post-arbitration chassis command on `/cmd_vel`. `/cmd_vel_safe` is a mirror for diagnostics and the Ranger Mini 3 mode-controller shadow observer. Docking commands use a separate `/cmd_vel_docking` input so Nav2 `collision_monitor` zero commands on `/cmd_vel_collision_checked` cannot overwrite near-field docking or undocking commands.

Command topics are treated as latest-only control streams, not durable command queues. The node creates the normal, API, docking, final, and mirror Twist endpoints with `KEEP_LAST(1)` QoS by default so old nonzero velocity samples cannot be drained after a newer stop command. A near-zero command from the active command owner is also stop-dominant: `robot_safety` immediately publishes zero, keeps a short zero burst window, and rejects late nonzero samples during that window. This prevents spin/drive/arc commands from continuing only because an upstream queue or source-priority cache still contains older velocity samples.

The current arbitration order is:

1. `ESTOP_ACTIVE`
2. `LOCALIZATION_INVALID` when `require_localization_health=true`
3. `COMMAND_STALE` when upstream control stops refreshing commands inside the watchdog window
4. `DOCKED_CONTACT_BLOCK` when normal motion is requested while docked or charging
5. `OK`

When a fresh `/cmd_vel_docking` message exists, normal `/cmd_vel_collision_checked` messages are ignored for `docking_cmd_priority_timeout_sec`. This keeps App zero bursts and Nav2/collision-monitor zero output from interleaving with controlled docking motion.

After a pure yaw command, `robot_safety` can hold the first following linear command at zero until the spin tail has settled. The gate checks `/wheel/odom.twist.twist.angular.z` and, by default, `/lidar_imu_bias_corrected.angular_velocity.z` because the Ranger wheel twist can report zero before the physical body yaw-rate has actually stopped. `/local_state/odometry` remains an optional diagnostic gate only; production release is based on raw wheel odom plus the corrected 100 Hz IMU tail detector. The local-state runtime must keep `imu_gyro_bias_filter_node` resident even when the EKF profile is wheel-only; otherwise this gate can only fail open after its timeout. This handles Ranger Mini 3 spin stop tail: upstream Nav2/API may have already published zero yaw, while the chassis is still rotating for a short interval. The gate is intentionally placed in `robot_safety` because this package is the final command arbitration point before `/cmd_vel`; it does not change Nav2, AMCL, or the chassis SDK motion model.

After docking or lateral capture, the Ranger SDK can keep reporting `MOTION_MODE_PARALLEL` while all command topics are already zero. The official SDK only switches back to `MOTION_MODE_DUAL_ACKERMAN` when it receives a nonzero dual-Ackermann-style Twist. `robot_safety` therefore has a mode-exit guard for normal/API drive commands: if the mode-controller status reports an actual lateral mode and the next command intends dual-Ackermann drive, it first emits a bounded low-speed `linear.x` probe with `linear.y=0` and `angular.z=0`, then releases the original command after the actual mode returns to dual Ackermann. Docking commands are exempt so fine docking can still intentionally use side-slip.

API terminal pose recovery may also need a small side-slip command after Nav2 reaches a normal goal but final pose verification still sees a lateral residual. When `allow_api_lateral_cmd=true`, `robot_safety` passes bounded `/cmd_vel_api.linear.y` through the same final arbitration path, clamps it with `api_lateral_max_mps`, and still relies on the Ranger mode controller forced-mode contract before the chassis accepts lateral motion. Normal Nav2 commands remain lateral-zeroed.

For push-in spring charging docks, controlled undocking must be a continuous low-speed motion through the charger switch travel. `robot_safety` stores the last fresh `/cmd_vel_docking` command and republishes it from the safety timer while the docking-priority window is active, so watchdog/status refreshes do not insert zero commands between valid undock updates.

When `block_normal_motion_when_docked=true`, BMS contact, `/docking/status` docked/charging, or the persistent dock-contact latch blocks normal `/cmd_vel_collision_checked` output and publishes zero with `/safety/status=DOCKED_CONTACT_BLOCK`. A latch is treated as stale safety memory when fresh BMS says no contact and there is no current docked/charging status, so an old latch cannot permanently block navigation after a clean no-contact state is visible. `allow_docking_cmd_when_docked=true` preserves `/cmd_vel_docking` so the controlled docking/undocking owner can still move. The watchdog timer evaluates a fresh docking command in docking context, so the dock/contact interlock does not insert zero commands between valid `/cmd_vel_docking` updates.

When any blocking state is active, the node publishes a zero twist and latches the current safety state on `/safety/status`.

## Parameters

- `watchdog_timeout_sec`: stop the robot when upstream control stalls
- `cmd_vel_qos_depth`: Twist command stream QoS depth, default `1` for latest-only velocity control
- `zero_cmd_priority_enabled`: make near-zero Twist commands immediately stop-dominant for the active command owner
- `zero_cmd_priority_epsilon`: absolute per-axis Twist threshold treated as a zero command
- `zero_cmd_priority_burst_sec`: short window where final zero is repeated and late nonzero samples are rejected
- `docking_cmd_vel_in_topic`: docking/undocking command input
- `docking_cmd_priority_timeout_sec`: freshness window where docking input overrides normal input
- `publish_rate_hz`: safety refresh rate for watchdog zeroing and state publication
- `require_localization_health`: block motion until localization is explicitly healthy
- `publish_zero_on_startup`: force an initial zero command before any navigation source is active
- `block_normal_motion_when_docked`: zero normal commands while docked or charging
- `enable_bms_contact_guard`: use BMS charging-contact evidence for normal-motion blocking
- `enable_docking_status_guard`: use `/docking/status` prefixes for normal-motion blocking
- `enable_docked_latch_file_guard`: use the persistent docked latch file for normal-motion blocking
- `docked_status_prefixes`: lower-case status prefixes that mean docked/contact, default `docked,charging`
- `battery_state_topic`: BMS input for charging-contact evidence
- `docking_status_topic`: docking status input for docked/charging evidence
- `docking_contact_latch_file`: persistent explicit dock-contact state shared with API/docking manager
- `allow_docking_cmd_when_docked`: keep controlled `/cmd_vel_docking` motion available while normal motion is blocked
- `spin_to_drive_settle_enabled`: hold linear drive briefly after pure spin until actual wheel odom yaw rate is settled
- `spin_to_drive_odom_topic`: odom topic used for the actual yaw-rate settle check, default `/wheel/odom`
- `spin_to_drive_wz_threshold_radps`: actual yaw-rate threshold treated as stopped, default `0.02`
- `spin_to_drive_stable_samples`: consecutive settled odom samples required before releasing linear drive
- `spin_to_drive_require_local_odom_stable`: optional diagnostic gate for local EKF odom stability, default `false`; production release is based on raw wheel yaw-rate stability
- `spin_to_drive_local_odom_topic`: local odom topic sampled for diagnostics or for the optional local-stability gate, default `/local_state/odometry`
- `spin_to_drive_local_wz_threshold_radps`: local odom yaw-rate threshold treated as settled, default `0.03`
- `spin_to_drive_local_stable_samples`: consecutive local odom samples required only when `spin_to_drive_require_local_odom_stable=true`
- `spin_to_drive_local_stable_duration_sec`: minimum local odom stable duration only when the optional local gate is enabled, default `0.30`
- `spin_to_drive_local_yaw_delta_threshold_rad`: max local yaw drift across the stable window
- `spin_to_drive_local_odom_max_age_sec`: maximum accepted age of the local odom settle sample
- `spin_to_drive_require_imu_stable`: require the high-rate IMU yaw-rate tail detector before releasing linear drive after a spin, default `true`
- `spin_to_drive_imu_topic`: IMU topic used for the spin-tail detector, default `/lidar_imu_bias_corrected`
- `spin_to_drive_imu_wz_threshold_radps`: IMU yaw-rate threshold treated as physically stopped, default `0.035`
- `spin_to_drive_imu_stable_duration_sec`: minimum continuous IMU-stable duration before releasing linear drive, default `0.30`
- `spin_to_drive_imu_max_age_sec`: maximum accepted age of the IMU settle sample, default `0.10`
- `spin_to_drive_timeout_sec`: fail-open timeout after the first held linear-drive request, default `2.0`
- `spin_to_drive_linear_epsilon_mps`: linear command threshold used to distinguish pure spin from drive
- `spin_to_drive_odom_max_age_sec`: maximum accepted age of the odom yaw-rate sample
- `mode_exit_guard_enabled`: guard normal/API drive commands from starting at full speed while the chassis still reports a lateral motion mode
- `mode_controller_status_topic`: status source containing actual Ranger motion mode, default `/ranger_mini3_mode_controller/status`
- `mode_exit_guard_probe_speed_mps`: bounded dual-Ackermann probe speed used to switch out of lateral mode, default `0.06`
- `mode_exit_guard_timeout_sec`: maximum probe duration before holding zero instead of passing the original command, default `1.0`
- `mode_exit_guard_status_max_age_sec`: maximum accepted age of the mode-controller status sample, default `0.5`
- `allow_api_lateral_cmd`: allow bounded `/cmd_vel_api.linear.y` for API-owned terminal pose recovery, default `false`
- `api_lateral_max_mps`: absolute clamp for API lateral speed before final publication, default `0.10`

## Notes

- This package does not own planners, controllers, or collision monitoring. It only arbitrates the final command.
- The docking command hold is not a bypass: the held command is still published only by `robot_safety`, only while the docking command is fresh. Ordinary Nav2 reverse is limited to low-speed MPPI terminal correction and does not own docking/undocking motion.
- Jetson runtime executes the compiled C++ node directly and fails fast if the binary is missing; the Python fallback path has been removed.
- `/cmd_vel_safe` is a diagnostic mirror when the runtime publishes the final command on `/cmd_vel`; the effective chassis command remains owned by `robot_safety`.
