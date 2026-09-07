# Navigation terminal control

This directory owns the ordinary-navigation terminal-control policy and the
post-Nav2 lateral/yaw correction execution loops used by `robot_api_server`.

## Owned behavior

- distance-based Nav2 terminal speed limits;
- the commercial final-pose gate (`0.06 m` XY by current configuration);
- lateral entry/completion hysteresis (`> 0.04 m` to enter, `<= 0.03 m` to
  finish by current configuration);
- deterministic correction order: yaw, lateral side-slip, then
  forward/reverse;
- lateral divergence detection and one-time direction reversal;
- local-costmap path sampling with unavailable, stale, malformed, unknown,
  occupied, and out-of-bounds data treated as blocked;
- reverse-permit acquisition/refresh release for the correction window;
- zero-command bursts, drive-mode release, wheel-stop confirmation, and final
  settled-pose recheck before success.
- bounded final-yaw motion, stop-lead handling, wheel angular-stop proof, XY
  drift rejection, and settled-yaw revalidation.

`NavigationTerminalRuntimePort` separates the deterministic controller from
`NavigationTerminalRuntimeModule`. The runtime module now owns the complete
ROS edge used by terminal control: `/cmd_vel_api`, terminal `/speed_limit` and
reverse-permit publishers; Ranger mode-status, `/wheel/odom`, local-costmap,
and `/rosout` subscriptions; their synchronized caches; physical-stop waits;
and the narrow callbacks into mission, localization, safety, docking-contact,
and Ranger-mode domains. The composition root only builds its configuration
and connects those callbacks.

The runtime extraction preserves the existing topic names, QoS profiles,
reverse-permit hysteresis and refresh period, zero-command cadence, stop
thresholds/timeouts, mode-ack requirement, costmap evidence, and MessageFilter
drop accounting. It adds no admission gate and does not change any terminal
success criterion.

## Explicitly not owned

- Nav2 goal submission, cancellation, lifecycle, planner, or controller
  configuration;
- map/floor switching or localization triggering;
- docking and predock alignment;
- elevator or arm workflows;
- action clients or HTTP routes;
- robot-safety arbitration and the final velocity chain.

The production command remains `/cmd_vel_api` through the existing adapter;
the module does not bypass `robot_safety` and does not publish directly.

## Regression test

`test/features/navigation/terminal_control/test_navigation_terminal_control.cpp`
freezes speed bands, lateral hysteresis, axis ordering, costmap fail-closed
behavior, reverse-permit cleanup, zero-command cleanup, final-yaw motion, and
physical-settle revalidation.

`test/features/navigation/terminal_control/test_navigation_terminal_runtime_module.cpp`
freezes the ROS-facing ownership and QoS-compatible behavior: command, speed
limit and permit publication; mode/odometry stop proof; local-costmap caching;
and `/rosout` MessageFilter evidence. The adjacent Python contract prevents
these publishers, subscriptions, caches, or `NavigationTerminalRuntimePort`
implementation from returning to the root node.
