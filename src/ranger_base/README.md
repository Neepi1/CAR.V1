# ranger_base

Project-maintained Ranger Mini 3 chassis core derived from the Weston Robot /
AgileX ROS 2 driver.

## Ownership

- Subscribes only to the final post-safety `/cmd_vel` command.
- Is the only process allowed to call the Ranger UGV SDK and own `can0`.
- Selects DUAL_ACKERMAN, PARALLEL, SPINNING, or SIDE_SLIP from the requested
  Twist and confirms every mode transition from chassis feedback.
- Publishes `/wheel/odom`, `/motion_state`, `/system_state`, `/battery_state`,
  and `/ranger_base/status`.
- Does not publish the canonical `odom -> base_link` transform in production.

`robot_safety` remains the only final velocity arbiter and `/cmd_vel`
publisher. Nav2, docking, teleop, and the App do not call this driver directly.

## Mode-Switch Contract

The Ranger firmware ignores velocity commands while `mode_changing=1`.
`ranger_base` therefore runs this state machine:

```text
STABLE -> STOPPING -> WAITING_ACK -> STABLE
                           `-> MODE_SWITCH_TIMEOUT (zero held, retry continues)
```

During a transition, the driver:

1. sends zero motion;
2. waits for feedback linear and wheel-odom angular velocity to settle;
3. sends/retries the motion-mode CAN frame;
4. waits for `actual_mode == desired_mode && mode_changing == 0`;
5. releases only a fresh subsequent Twist command.

Relevant parameters:

- `mode_switch_handshake_enabled` (default `true`)
- `mode_switch_retry_period_sec` (default `0.10`)
- `mode_switch_timeout_sec` (default `2.0`)
- `mode_switch_stable_duration_sec` (default `0.15`)
- `mode_switch_stop_linear_threshold_mps` (default `0.02`)
- `mode_switch_stop_angular_threshold_radps` (default `0.03`)

`/ranger_base/status` reports desired/actual mode, `mode_changing`, alignment,
transition state, and elapsed transition time. The historical
`/ranger_mini3_mode_controller/status` name is a temporary compatibility alias
published by this node; no separate mode-controller process exists.

## UGV SDK

The build uses the pinned source in
`external_sources/jetson_ugv_sdk_20260713/ugv_sdk`. Its CAN transmitter owns
frame memory, serializes writes, and coalesces pending `0x111` motion and
`0x141` mode commands. This prevents a newer velocity frame from overwriting a
pending mode frame and prevents stale velocity backlog.

## Runtime

Start and stop only through the full resident owner:

```bash
sudo systemctl restart njrh-runtime.service
```

Do not start a second `ranger_base_node` or a standalone mode controller.

## Hardware Validation

- 20 consecutive DUAL_ACKERMAN -> SPINNING -> DUAL_ACKERMAN transitions.
- Verify nonzero `0x111` is absent while mode feedback reports changing.
- Verify each transition confirms in less than 2 seconds.
- Repeat normal navigation, predock yaw alignment, lateral docking, and undock.

## Rollback

Set `RANGER_MODE_SWITCH_HANDSHAKE_ENABLED=false` to restore the old immediate
mode/velocity behavior without changing code. A full source rollback can point
`ranger_base` back to the upstream installed `ugv_sdk`, but that also restores
the unsafe unowned asynchronous CAN buffer and is diagnostic-only.
