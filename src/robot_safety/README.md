# robot_safety

Final command arbitration point before the chassis bridge.

## Canonical Contract

- normal navigation/App command input: `/cmd_vel_collision_checked`
- docking command input: `/cmd_vel_docking`
- safe output: `/cmd_vel_safe`
- estop input: `/safety/estop`
- optional localization gate: `/localization/health`
- state output: `/safety/status`
- motion-allowed output: `/safety/motion_allowed`

## Arbitration Policy

`robot_safety` is the only package allowed to publish the post-arbitration safe command. The Ranger Mini 3 mode controller consumes `/cmd_vel_safe` and publishes `/cmd_vel` for `ranger_base_node`. Docking commands use a separate `/cmd_vel_docking` input so Nav2 `collision_monitor` zero commands on `/cmd_vel_collision_checked` cannot overwrite near-field docking or undocking commands.

The current arbitration order is:

1. `ESTOP_ACTIVE`
2. `LOCALIZATION_INVALID` when `require_localization_health=true`
3. `COMMAND_STALE` when upstream control stops refreshing commands inside the watchdog window
4. `OK`

When a fresh `/cmd_vel_docking` message exists, normal `/cmd_vel_collision_checked` messages are ignored for `docking_cmd_priority_timeout_sec`. This keeps App zero bursts and Nav2/collision-monitor zero output from interleaving with controlled docking motion.

When any blocking state is active, the node publishes a zero twist and latches the current safety state on `/safety/status`.

## Parameters

- `watchdog_timeout_sec`: stop the robot when upstream control stalls
- `docking_cmd_vel_in_topic`: docking/undocking command input
- `docking_cmd_priority_timeout_sec`: freshness window where docking input overrides normal input
- `publish_rate_hz`: safety refresh rate for watchdog zeroing and state publication
- `require_localization_health`: block motion until localization is explicitly healthy
- `publish_zero_on_startup`: force an initial zero command before any navigation source is active

## Notes

- This package does not own planners, controllers, or collision monitoring. It only arbitrates the final command.
- Jetson runtime executes the compiled C++ node directly and fails fast if the binary is missing; the Python fallback path has been removed.
