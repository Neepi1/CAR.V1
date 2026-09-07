# Elevator configuration

`ElevatorConfigurationModule` owns commissioning drafts, schema validation,
immutable release publication, optimistic revision checks, rollback, and
private waypoint assets. It has no ROS, Nav2, FloorSwitch, TF, safety,
arm-motion, or Twist port.

`ElevatorRuntimeConfigurationModule` is a separate construction-time boundary.
It declares the 15 existing elevator adapter, external-arm-client, scoped-BT,
collision-bypass permit, and recovery-service ROS parameters, applies the
established clamps, and combines them with explicit map, navigation,
FloorSwitch, and safety inputs into `ElevatorModuleConfig`. It never calls or
implements the mechanical-arm black box, starts no transaction, publishes no
motion, and adds no gate.
