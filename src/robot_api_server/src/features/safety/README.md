# Safety API

This directory owns the App gateway's narrow adapter to `robot_safety`:

- `safety_state` owns the immutable status snapshot, legacy JSON projection,
  and the existing motion-admission interpretations;
- `safety_ros_adapter` owns the process-resident `/safety/status` and
  `/safety/motion_allowed` subscriptions plus the `/safety/estop` publisher;
- `safety_configuration_module` uniquely declares the three safety topic
  parameters;
- `safety_module` owns both App HTTP routes, the floor/elevator-atomic resume
  admission transaction, and the ordered emergency stop fan-out used when an
  accepted Nav2 goal has no proven terminal result.

The state subscriptions use reliable transient-local QoS and are independent
of App page leases. Unknown `motion_allowed` evidence continues to defer to the
final `robot_safety` arbiter. `COMMAND_STALE` remains a first-command warmup
case, while an explicit non-stale denial remains a hard block.

The composition root injects floor/elevator admission and the existing teleop,
terminal-control, and predock stop effects as ports; it does not implement a
safety route, retain a safety topic scalar, or duplicate the stop sequence.
Final velocity arbitration, estop enforcement, watchdogs, speed limits,
reverse permissions, and motion interlocks remain exclusively in
`robot_safety`; this directory never publishes chassis velocity.
