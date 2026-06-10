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
4. `DOCKED_CONTACT_BLOCK` when normal motion is requested while docked or charging
5. `OK`

When a fresh `/cmd_vel_docking` message exists, normal `/cmd_vel_collision_checked` messages are ignored for `docking_cmd_priority_timeout_sec`. This keeps App zero bursts and Nav2/collision-monitor zero output from interleaving with controlled docking motion.

For push-in spring charging docks, controlled undocking must be a continuous low-speed motion through the charger switch travel. `robot_safety` stores the last fresh `/cmd_vel_docking` command and republishes it from the safety timer while the docking-priority window is active, so watchdog/status refreshes do not insert zero commands between valid undock updates.

When `block_normal_motion_when_docked=true`, BMS contact, `/docking/status` docked/charging, or the persistent dock-contact latch blocks normal `/cmd_vel_collision_checked` output and publishes zero with `/safety/status=DOCKED_CONTACT_BLOCK`. `allow_docking_cmd_when_docked=true` preserves `/cmd_vel_docking` so the controlled docking/undocking owner can still move. The watchdog timer evaluates a fresh docking command in docking context, so the dock/contact interlock does not insert zero commands between valid `/cmd_vel_docking` updates.

When any blocking state is active, the node publishes a zero twist and latches the current safety state on `/safety/status`.

## Parameters

- `watchdog_timeout_sec`: stop the robot when upstream control stalls
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

## Notes

- This package does not own planners, controllers, or collision monitoring. It only arbitrates the final command.
- The docking command hold is not a bypass: the held command is still published only by `robot_safety`, only while the docking command is fresh, and ordinary Nav2 reverse remains disabled.
- Jetson runtime executes the compiled C++ node directly and fails fast if the binary is missing; the Python fallback path has been removed.
