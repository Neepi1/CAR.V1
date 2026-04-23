# robot_chassis_bridge

Wrapper for Ranger Mini 3 chassis topics and diagnostics.

## Parameters

- `mock_mode`: publish zeroed wheel odom for integration
- `wheel_odom_topic`: default `/wheel/odom`
- `cmd_vel_in_topic`: final command ingress, default `/cmd_vel`
- `cmd_vel_out_topic`: platform-specific egress, default `/platform/cmd_vel`
- `odom_frame` / `base_frame`: canonical local frames

## TF Contract

- Does not publish `odom -> base_link`
- Only publishes wheel odometry messages for `robot_local_state` consumption
