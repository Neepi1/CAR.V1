# Docking configuration

Owns both runtime parameter composition and commissioning-time docking
configuration policy.

`DockingConfigurationModule` is the single owner of all 100 docking, predock,
fine-entry, dock-contact, and undock ROS parameter declarations. It preserves
the established defaults, clamps, and dependent bounds, then returns the
existing runtime, interlock, correction-pause, pose-resolver, alignment,
predock-control, undock, executor, HTTP, and status configuration objects
ready for construction. Neighboring service-timeout, navigation-action,
map-frame, pose-freshness, and BMS thresholds are explicit inputs; the module
does not redeclare them or retain runtime state.

`DockingPredockPoseResolver` applies the complete explicit/automatic manual
predock-point selection order and its distance/yaw sanity validation. Map asset
access is injected read-only. Neither configuration unit writes a pose, starts
docking, publishes motion, changes TF, or adds a gate.
