# robot_safety

Final command arbitration point before the chassis bridge.

## Canonical Contract

- input command: `/cmd_vel_collision_checked`
- final output: `/cmd_vel`
- estop input: `/safety/estop`
- optional localization gate: `/localization/health`
- state output: `/safety/status`
- motion-allowed output: `/safety/motion_allowed`

## Arbitration Policy

`robot_safety` is the only package allowed to publish the final `/cmd_vel` in the canonical stack.

The current arbitration order is:

1. `ESTOP_ACTIVE`
2. `LOCALIZATION_INVALID` when `require_localization_health=true`
3. `COMMAND_STALE` when upstream control stops refreshing commands inside the watchdog window
4. `OK`

When any blocking state is active, the node publishes a zero twist and latches the current safety state on `/safety/status`.

## Parameters

- `watchdog_timeout_sec`: stop the robot when upstream control stalls
- `publish_rate_hz`: safety refresh rate for watchdog zeroing and state publication
- `require_localization_health`: block motion until localization is explicitly healthy
- `publish_zero_on_startup`: force an initial zero command before any navigation source is active

## Notes

- This package does not own planners, controllers, or collision monitoring. It only arbitrates the final command.
- Jetson runtime should execute this repository node directly so field behavior and source behavior stay aligned.
