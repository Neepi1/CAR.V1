# Ranger Mini3 SmacPlannerLattice Profile

## Status

This is an opt-in engineering profile. The repository default remains
`nav2_smac_planner/SmacPlanner2D`; the Jetson may explicitly select
`ranger_lattice` while the moving gates in this document are being completed on
the physical Ranger Mini3.

The project does not fork or patch `nav2_smac_planner`. It owns a pinned motion
primitive artifact and a planner parameter profile consumed by
`RangerMini3LatticePlanner`, a thin subclass of the stock ROS 2 Humble
`SmacPlannerLattice` plugin. General planning remains the upstream implementation;
the subclass owns only the bounded clear-corridor policy described below.

## V1 motion contract

| Property | V1 value | Reason |
|---|---:|---|
| Global costmap resolution | 0.05 m | Must exactly match the lattice grid |
| Minimum turning radius | 0.81 m | Conservative MPPI/controller contract already used by this product |
| Heading bins | 16 | Nav2 generator recommendation and adequate 5 cm endpoint connectivity |
| Primitive set | 104 | 72 generated translations plus 32 project-owned one-bin spins |
| Primitive motion | Forward dual-Ackermann plus in-place spin | Matches the two normal-navigation modes the chassis can execute deterministically |
| Reverse expansion | Search enabled, runtime bounded | Product safety only leases reverse near a goal; long-route regression forbids reverse segments |
| Pure spin primitives | Two per heading | Required for path-entry heading changes; arbitrary mid-path execution is not yet admitted |
| Lateral primitives | None | Docking/API owns side-slip |
| Internal path smoothing | Enabled with pinned Humble defaults | Removes lattice heading-bin discontinuities while preserving the 0.81 m curvature contract |

The profile's expected planner frequency is 1 Hz, matching the Lattice behavior
tree's path-validity check cadence. It is not an unconditional replanning loop:
the current path is retained while valid, and a new Lattice plan is requested
only when the goal changes or the global costmap invalidates the path.

Grid endpoint discretization makes the sharpest translating primitive radius
0.84721 m, slightly more conservative than the 0.81 m input contract. The set
contains 16 straight, 28 left-turn, 28 right-turn, and 32 in-place spin
primitives. Translating primitive lengths range from 0.25 m to 0.57557 m.

The Ranger Mini3 SDK source contains a tighter nominal radius for one hardware
revision, but that value is not yet the verified controller envelope for this
robot. V1 therefore uses the existing 0.81 m MPPI contract. A smaller lattice
radius is not permitted until repeatable minimum-radius field tests pass at the
same speed and payload used in navigation.

The planner profile explicitly pins the `SmacPlannerLattice` internal smoother
parameters from Navigation2 1.1.20: `max_iterations=1000`, `w_data=0.2`,
`w_smooth=0.3`, `tolerance=1e-10`, and `do_refinement=true`. This is the
curvature-aware smoother inside the lattice planner. It is not the Behavior
Tree `SmoothPath` action or the previously tested Savitzky-Golay path smoother.
In the Jetson plan-only probe, enabling this internal smoother removed all four
heading-metadata spikes while keeping maximum geometric curvature at
approximately 1.235 1/m and planning time below 20 ms.

The zero-displacement spin transitions are a native Humble State Lattice
feature: `NodeLattice` detects `trajectory_length < 1e-4` and applies
`rotation_penalty`. They are limited to one adjacent heading bin in each
direction. At path entry, the existing RotationShim samples the first
translating path point and emits `linear.x=0`, `linear.y=0`, `angular.z!=0`;
the normal safety and Ranger mode chain then selects `SPINNING`. Humble's shim
does not consume an arbitrary zero-displacement spin block later in a path, and
Ackermann MPPI cannot execute that block itself. V1 therefore admits only
plans whose spin segments are at cumulative distance 0 m (plus the separately
handled final goal heading); a mode-aware mid-path executor is a later phase.
Lateral motion is deliberately excluded because a `nav_msgs/Path` has no
side-slip mode tag and normal MPPI navigation does not own docking translation.

## Clear opposite-heading corridors

Stock Humble State Lattice charges every one-bin zero-displacement rotation the
same `rotation_penalty`. For two collinear poses facing in opposite directions,
the stock search can therefore choose a partial path-entry spin and finish the
heading change with a minimum-radius Ackermann arc. On the calibration route,
that produced about 107.7 degrees of startup spin and up to 1.286 m of geometric
cross-track deviation even though the direct corridor was fully free.

Reducing `rotation_penalty` globally is not admissible: `rotation_penalty=1.0`
produced the desired 180-degree startup spin, but the blocked-route regression
also found three zero-displacement spin transitions after translation had begun.
MPPI cannot execute those transitions under the V1 motion contract.

`RangerMini3LatticePlanner` therefore calls the stock planner first and changes
its result only when all of these checks pass:

- route length is at least `1.0 m`;
- goal yaw differs from the direct route bearing by no more than `0.20 rad`;
- samples every `0.025 m` stay inside the global costmap;
- every sampled center cell has cost `0`;
- the full Ranger footprint is collision-free at every sample.

The returned path is then a straight geometric line. The existing
`GoalScopedRotationShimController` rotates the chassis to that line before MPPI
starts translation. If any check fails, the original stock Lattice result is
returned unchanged. This preserves ordinary obstacle detours and keeps
`rotation_penalty=3.0`, so the fix cannot make arbitrary mid-path spin cheaper.
The policy is independently rollbackable with `GridBased.direct_corridor_enabled=false`.

For every stock-Lattice fallback, the wrapper also restores the exact requested
goal pose after the search. Humble State Lattice returns a final pose quantized
to the 5 cm grid and one of 16 heading bins; using that quantized pose as the
`FollowPath` goal can otherwise discard a commissioned predock yaw before the
ordinary goal checker sees it. The searched path is not rewritten. The wrapper
only appends the original pose when the residue is at most `0.08 m` and
`0.21 rad`, and every 2.5 cm / 0.05 rad interpolation sample is clear for the
full footprint. A larger or colliding residue fails planning instead of
claiming arrival at a different pose.

A nested project planner that directly embedded both Smac2D and
SmacPlannerLattice was evaluated and rejected: it met path-quality targets but
reproducibly double-freed memory during plugin process teardown. No part of
that prototype remains in the runtime design.

The final 2026-07-21 Jetson plan-only A/B used the same two fixed endpoints in
both directions. Smac2D produced 11.312 m paths. The Ranger lattice produced
11.600 m and 11.617 m paths, a 2.55 percent and 2.69 percent increase. Lattice
planning took 7.85-14.30 ms, translating curvature stayed at or below
1.235 1/m, no reverse segments were present, all spin segments occurred at
cumulative distance 0 m, and the isolated planner exited cleanly after
lifecycle cleanup. A clean `robot_nav_config` build also passed all eight C++
tests, including parsing the checked-in artifact through the installed Humble
`nav2_smac_planner` types.

The first moving field gate on 2026-07-21 did not pass. From the measured start
pose to `delivery_675235`, the initial 6.964 m plan was valid, but the ordinary
1 Hz behavior-tree replanning changed the terminal branch as the robot
approached the goal. Reconstructing the 14 s and 16 s poses produced 4.491 m
and 5.495 m replacement paths even though the robot was roughly 1 m and 0.66 m
from the target. The controller consequently commanded left curvature, then
right curvature, then left curvature. The Ranger feedback followed those
commands; this was not chassis tracking error. Native Nav2 stopped at 0.156 m
with about 1.017 rad of yaw error, and the existing API terminal correction
later completed the task at 0.054 m and 0.023 rad. API recovery success does
 not satisfy the Lattice moving gate. V1 therefore remained rejected at that
 point. The follow-up implementation uses Nav2 Humble's stock
 `GlobalUpdatedGoal` and `IsPathValid` nodes in a Lattice-only behavior tree.
 It addresses the observed branch replacement without disabling replanning for
 dynamic obstacles.

The next attempt found an independent startup failure in Humble's stock
RotationShim. Its first current-time TF lookup requested `map -> base_link`
about 2 ms newer than the latest odometry transform. The caught exception set
`path_updated_ = false`, so the shim emitted one spin command and then handed
the still-unaligned path to MPPI. MPPI requested a low-speed reverse that the
normal safety policy correctly rejected. The project controller now measures
the path-entry heading directly when path and robot pose already share the map
frame. If a transform is genuinely required it requests the latest complete TF;
an unavailable measurement holds zero and remains armed for the next cycle.
Unit tests cover unavailable measurement, engage/disengage hysteresis, short
paths, and non-finite input.

After those two forward fixes, the first bidirectional hardware pair passed:

| Leg | Native Nav2 | Final XY | Final yaw | Command/mode evidence |
|---|---|---:|---:|---|
| `delivery_675235 -> delivery_512355` | succeeded | 0.0594 m | 0.00765 rad (0.44 deg) | one continuous startup spin, then forward-only Ackermann |
| `delivery_512355 -> delivery_675235` | succeeded | 0.0527 m | 0.03033 rad (1.74 deg) | one continuous startup spin, then forward-only Ackermann |

Both legs stayed on the ordinary Nav2, collision-monitor, `robot_safety`, and
`ranger_base` command chain. `/motion_state` changed from `SPINNING` to
`DUAL_ACKERMAN`; no reverse or lateral Nav2 samples and no mid-route pure-spin
re-entry were observed. This closes the original branch-loop and startup-TF
reproductions, but it is only two of the required ten alternating legs and does
not yet promote the profile to the repository default.

A later 2026-07-22 `predock -> delivery_512355` repeated run exposed the
terminal ownership boundary rather than a chassis tracking error. The terminal
Lattice replan had a 0.403 m chord but a 1.370 m Ackermann hairpin with 0.272 m
cross-track excursion. MPPI twice reported no progress at a 0.366 m mostly
lateral residual.

The runtime now resolves that boundary inside the same `FollowPath` action.
`GoalScopedRotationShimController` keeps MPPI Ackermann-only, but starts its
terminal controller when the current pose is within 0.40 m, body-frame forward
error is at most 0.15 m, and either the residual is lateral-dominant or the
remaining path is an Ackermann hairpin. The stages are mutually exclusive:
yaw, side-slip, forward/reverse, then physical-stop settle. Lateral and reverse
commands require fresh controller-owned permits, remain costmap checked, and
still pass through `velocity_smoother`, `collision_monitor`, `robot_safety`, and
`ranger_base`. The API proactive early-cancel handoff is disabled; it remains a
fallback only after a true Nav2 abort.

The final opposite-lateral field case triggered at 0.151 m with 0.128 m lateral
error, moved about 0.103 m laterally through the normal Nav2 command chain, and
returned native Nav2 success at 0.020 m / 2.26 degrees. This does not add
lateral primitives to the Lattice or lateral sampling to MPPI; it gives a
kinematically explicit terminal owner to a residual Ackermann cannot close.

## Owned artifacts

- `lattice/ranger_mini3_ackermann_0p05m_0p81m_16.config.json`: generator input.
- `lattice/ranger_mini3_ackermann_0p05m_0p81m_16.json`: runtime primitive file.
- `lattice/ranger_mini3_ackermann_0p05m_0p81m_16.manifest.json`: source pin,
  hashes, and motion-mode ownership contract.
- `tools/generate_ranger_mini3_lattice.py`: reproducible development generator.
- `tools/validate_ranger_mini3_lattice.py`: runtime-independent contract check.
- `config/planner_profiles/ranger_mini3_lattice.yaml`: planner-only override.
- `src/ranger_mini3_lattice_planner.cpp`: stock-Lattice subclass and footprint-checked corridor admission.
- `include/robot_nav_config/ranger_direct_corridor.hpp`: deterministic straight-path policy used by runtime and tests.
- `config/planner_profiles/preserve_base.yaml`: empty default override.

The generator is pinned to Navigation2 tag `1.1.20`, commit
`a097086719c88f781aa59788eca29ac6ca5e56db`, matching the installed Humble
package on the Jetson. It validates normalized hashes for every upstream
generator module before producing output.

## Generate and validate

Generation requires a local checkout of the pinned Navigation2 source and
NumPy. It does not add dependencies to the navigation runtime.

```bash
python3 src/robot_nav_config/tools/generate_ranger_mini3_lattice.py \
  --nav2-source /path/to/navigation2-1.1.20

python3 src/robot_nav_config/tools/validate_ranger_mini3_lattice.py
```

The validator rejects an artifact that changes the 5 cm grid, 0.81 m radius,
16 headings, forward-only translation, endpoint grid alignment, heading
connectivity, left/right coverage, exactly two adjacent-bin spins per heading,
zero-translation spin geometry, or manifest SHA256.

## Runtime selection

`standard_navigation.launch.py` gives only `planner_server` a second parameter
file and gives `bt_navigator` an explicit behavior-tree path. The default
second planner file is empty and the default tree remains
`navigate_to_pose.xml`, so the production Smac2D behavior is unchanged.
`run_nav2_navigation.sh` recognizes two explicit profiles:

- `NJRH_NAV2_PLANNER_PROFILE=smac2d`:
  preserve the base `SmacPlanner2D` configuration and ordinary periodic tree.
- `NJRH_NAV2_PLANNER_PROFILE=ranger_lattice`: validate and load the Ranger
  lattice profile plus `navigate_to_pose_ranger_lattice.xml`, which keeps a
  valid path and replans on goal change or path invalidation.

The actual environment variable is `NJRH_NAV2_PLANNER_PROFILE`. Store it in
`/etc/njrh/runtime.env`; do not launch a second Nav2 owner manually. Any profile
change requires one complete restart:

```bash
sudo systemctl restart njrh-runtime.service
```

After startup, verify the selected plugin and artifact before moving:

```bash
timeout -k 1s 5s ros2 param get /planner_server GridBased.plugin
timeout -k 1s 5s ros2 param get /planner_server GridBased.lattice_filepath
timeout -k 1s 5s ros2 lifecycle get /planner_server
```

The hard-kill grace is required on this Fast DDS runtime. Never leave a
short-lived ROS CLI graph probe blocked after inspection; use the resident
runtime health snapshot or `/api/v1/robot/pose` for repeated monitoring.

Before each V1 hardware route, run the live path contract probe. It calls the
active planner action but never publishes a velocity command. Exit code 0
admits the route; exit code 3 rejects a route containing reverse motion, a
mid-path spin, or moving curvature above the 1.25 1/m field gate:

```bash
python3 scripts/jetson/runtime_overlay/scripts/probe_ranger_lattice_live_path.py \
  --goal-x -6.364297 \
  --goal-y -3.032889 \
  --goal-yaw -1.603842 \
  --output-json /tmp/ranger_lattice_live_path.json
```

The probe also accepts `--start-x`, `--start-y`, and `--start-yaw` together to
reconstruct a historical replan. Its terminal profile is used to compare the
initial path with replacement paths after the robot has moved. Initial-plan
admission remains necessary, while the Lattice behavior tree prevents an
otherwise-valid route from being replaced once per second.

Rollback is only the environment value `NJRH_NAV2_PLANNER_PROFILE=smac2d`
followed by the same full service restart.

## Hardware A/B gates

Do not begin with autonomous movement. Pass each gate in order:

1. Build `robot_nav_config` and `robot_bringup`, start the full runtime with the
   Lattice profile, and confirm all navigation lifecycle nodes are active.
2. Request plans without executing them. Confirm planner success, no reverse
   segments, collision-free footprint checks, planning time below the 3.0 s
   hard limit, no translating-path curvature above 1.25 1/m, and no spin
   segment away from cumulative distance 0 m. Reject a V1 route with a
   mid-path spin rather than handing it to Ackermann MPPI.
3. Compare Smac2D and Ranger Lattice plans for the same clear straight route.
   For opposite-heading collinear poses, require maximum cross-track deviation
   below 0.05 m and path length within 0.10 m of the direct Euclidean distance.
   A blocked centerline must retain a collision-free Lattice detour.
4. Run low-speed one-way travel with an operator ready to stop. Confirm the
   planned path never demands a curvature tighter than 0.81 m, spin transitions
   produce `SPINNING`, and mode feedback remains `DUAL_ACKERMAN` on every
   translating segment.
5. Run at least ten alternating legs between the two measured floor marks.
   Require 100 percent Nav2 completion, no spin/drive oscillation, no planner
   timeout, no safety bypass, and no new local-costmap collision failures.
6. Compare endpoint error, travel time, path length, MPPI command variation,
   stop count, RotationShim entries, and AMCL corrections against Smac2D. Do not
   promote Lattice unless it improves path shape without degrading the existing
   6 cm ordinary goal contract.

Required physical verification remains: loaded minimum-radius circles in both
directions, narrow-passage footprint clearance, dynamic-person interruption,
pre-dock approach, and full-speed braking before this becomes the default.
