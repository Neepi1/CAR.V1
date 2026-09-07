# Predock alignment

This directory owns the complete pre-dock staging-capture boundary.

`PredockAlignmentPolicy` owns the pure, parameterized business rules:

- map-pose error projection into approach-frame forward/lateral coordinates;
- signed yaw normalization across `-pi/pi`;
- inclusive handoff, recovery, forward-capture, lateral-capture, staging, and
  yaw-hysteresis predicates;
- exact diagnostic detail construction;
- atomic field projection from pose/yaw/lateral/fine-entry results into the
  already locked `DockingJob` model.

`PredockControlModule` owns the stateful transaction around those rules:

- early-handoff probing and canonical pose-evidence projection;
- bridge-smoothing wait followed by correction pause/freeze;
- admission of recoverable staging residuals to the near-field docking manager;
- fresh dock-observation and fine-entry checks;
- the final floor-interlock and `/docking/start` handoff.

With the production default
`docking_delegate_staging_motion_to_manager=true`, this module never acquires
the docking velocity owner and never publishes physical yaw or side-slip
commands. It records the measured staging residual, verifies that it lies in
the manager's capture envelope, and delegates it with the same
`/docking/start` handoff. The old API staging servo remains behind an explicit
rollback-only false setting; it is not part of the active command path.

The lifecycle executor sees only the semantic operations
`probe_handoff`, `verify_staging_pose`,
`validate_current_pose_near_approach`, and
`start_fine_docking_handoff`. Emergency cleanup uses
`stop_and_release_motion`. Low-level control stages cannot leak back into the
executor without failing the workspace boundary contract.

The executor also owns the physical handoff barrier. The predock behavior tree
must first return a proven terminal Nav2 result through its dedicated staging
capture checker. The executor then enters `PREDOCK_NAV2_STOP_VERIFY`, publishes
only zero, and proves zero motion from the existing wheel/local-odometry stop
observer. If actual stop cannot be proven it terminates with
`DOCK_FAILED_PREDOCK_NAV_STOP_UNPROVEN`; `/docking/start` is never called. This
makes ownership one-way: Nav2 coarse approach -> proven stop -> docking manager.

The composition root constructs both objects from the same clamped ROS
parameters and injects ports for fresh pose, safety admission, shared command
ownership, bridge state/pause, Ranger drive mode, `/cmd_vel_docking`, dock
observation, floor interlock, and the existing docking-manager service. The
module creates no ROS publisher, subscription, client, or TF owner. Commands
still follow `/cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base`.

This keeps the API as orchestration and evidence ownership while
`robot_docking_manager` is the single owner of near-field physical alignment.
TF ownership and the downstream `robot_safety` boundary are unchanged.
