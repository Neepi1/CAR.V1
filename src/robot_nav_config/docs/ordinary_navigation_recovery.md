# Ordinary navigation recovery

## Phase 2 candidate: retained task and bounded recovery actions

Status: **offline candidate only**, pending physical acceptance of phase 1 and
separate authorization to deploy/restart. Do not mix the new private service
schema, BT library, controller library or XML with their older versions.

The phase-1 tree below deterministically failed after the second FollowPath
abort. The red test `SecondFailureKeepsOriginalTaskWaiting` reproduced that
failure against the unchanged tree before implementation.

The candidate replaces only the ordinary Ranger tree's one-shot Fallback with
`OrdinaryRecoveryLoop`. The original NavigateToPose goal remains active. Actual
progress-checker failure from that FollowPath attempt is required before
retrying; Humble's empty action result cannot establish an obstacle cause by
itself. Unknown controller faults, action rejection/cancellation and planner
failure still terminate, rather than being disguised as waiting.

The first verified retry is immediate. Consecutive failures wait 5 seconds,
then 10 seconds between attempts. Waiting ticks cannot send extra goals. Every
retry computes a fresh plan, prepares it, then follows that exact path; a static
global replan is **not** taken as proof that a dynamic obstacle has cleared.
Existing local repair, MPPI collision checking and the final velocity chain
remain responsible for actual commands. No costmap clearing, forced reverse,
manual relocalization, or direct velocity publication is added.

Startup alignment may be rearmed once per blockage episode. Later exact-path
recoveries suppress the wrapper's idle-triggered startup rearm as well. A new
episode requires >=0.20 m net translation in the progress checker's frame over >=2 seconds,
with multiple moving pose samples and no long sample gap or frame change. Map
goal coordinates are never subtracted from odom-frame robot coordinates. A true progress
checker result alone, waiting, yaw-only motion and centimetre jitter do not
reset it. This is recovery bookkeeping, not a new arrival tolerance or motion
permission. A new outer goal (including a new timestamp at the same endpoint)
has its own recovery opportunity. This policy still requires field evaluation
when a newly available detour needs a different startup heading without prior
translation; software tests cannot prove every such scenario will recover.

The BT inspects the existing controller evidence service at <=2 Hz, transmitting
only the endpoint. `inspect_only` cannot prepare motion. It reports version-1
JSON on `/navigation/ordinary_recovery_status` at 2 Hz or on transitions:
`goal_stamp_ns`, `stamp_ns`, `phase`, `retries`. Phases include `tracking`,
`checking_failure`, `waiting`, `recovering`, `succeeded`, `failed`, `idle`.
The goal stamp is the original NavigateToPose input header stamp, unchanged by
Humble's navigator; the API also binds it to its exact accepted action handle.
There is no new lease, sensor subscription or public API endpoint.

For the initial ordinary goal and its standard same-goal final verification
retry, the API's configured result timeout now counts execution time excluding
continuously observed matching `waiting`/`recovering` intervals. Both ROS stamp
and local receipt expire after 1.5 seconds. Old goals, replayed/out-of-order
status, unknown phases and missing status cannot pause this budget. Long API
scheduling gaps are not retroactively credited. Near-goal handoff watches reset
during proven waiting; user cancellation is still checked first. The separate
short final-yaw-drift reposition deadline is intentionally unchanged. This does
not add a task-deadline API or promise success through permanent obstruction.

Acceptance: repeat crossings, head-on blockage, changing sides, blockage during
a detour, complete closure then removal, two separated blockages in one task,
cancel during wait, and an ordinary unobstructed goal. Record original action
identity, internal attempt count, wait/recovery phase, progress epoch, command
continuity, minimum clearance and final pose. Also regress predock and elevator
flows. Passing isolated tests is not physical acceptance.

## Phase 1 reference (previous one-retry implementation)

The sections below describe the prior implementation for comparison; the
candidate loop and API timeout semantics above supersede its one-retry limit.

## Evidence and scope

The recorded 2026-09-07 19:24:18.156Z `Failed to make progress` ended
FollowPath and then NavigateToPose. At final verification the robot was still
0.955336 m from the original goal. The running Ranger tree had no controller
failure branch. The operator subsequently reached the same target by manually
submitting it again and observing a new startup turn. This supports restoring
the startup/replan sequence, but does not isolate yaw as the sole cause of the
earlier stall or prove the complete MPPI field failure has been replayed.

Only the ordinary **ranger_lattice** runtime default changes. The old Ranger
tree and the Smac2D baseline remain available. `navigate_to_predock.xml`, all
elevator trees, goal precision, controller speeds, obstacle maps and sensors
are unchanged. This does not expand ordinary terminal control or reuse an
elevator controller/session. The progress checker has an evidence-only hook;
its progress decisions and thresholds are unchanged.

## Control flow

1. Clear the previous task's BT abort flag. Run the existing valid-path
   pipeline with an ordinary FollowPath adapter.
2. Only an actual FollowPath **ABORTED** result can enter the recovery branch.
   Rejection, cancellation and initial planner failure cannot enter it.
3. Compute a new path from the current pose to the original `{goal}`.
4. Call the Nav2-private `PrepareOrdinaryNavigationRecovery` service. Humble's
   installed FollowPath result is `std_msgs/Empty`, not a typed error code.
   The actual progress checker must have failed after that FollowPath started;
   the ordinary controller must have run in that attempt. A reset, resumed
   progress, another goal, inactive lifecycle or old plan rejects preparation.
5. Preparation is inert: it publishes no velocity, grants no permission and
   does not reconfigure or restart a node. Only the **identical new Path** in
   the next `setPlan()` consumes it. Any different path discards it, including
   another plan for the same endpoint. Its timestamp must be later than the
   recorded failure; timestamps are never rewritten.
6. Consumption resets only startup/terminal alignment state and the existing
   local-repair generation through `set_plan()`. MPPI receives the fresh path;
   the controller server resets progress at its normal new-action boundary.
   The normal first-turn logic decides whether yaw correction is necessary.
   There is no artificial two-second idle wait, lifecycle cycle, costmap clear,
   manual relocalization, or direct API velocity command.
7. The retry follows exactly that prepared path. It does not first pass through
   another global-plan selector that could silently replace the prepared path.
   The existing controller-local dynamic repair remains active. The outer BT
   stays RUNNING until this second FollowPath succeeds or fails. A second
   failure exits; there is no third FollowPath in this tree.

The retry retains the prepared path while its exact Ranger endpoint still
matches the outer goal. An explicit outer-goal replacement instead replans and
uses Humble's normal FollowPath goal-update handling; it cannot silently keep
driving toward the previous endpoint. This is not an extra recovery attempt.

The ordinary BT uses a standard, memoryful Fallback with two children: the
normal pipeline, then a single recovery Sequence. This provides exactly one
retry without a loop back through the normal global path selector. Cancellation
halts the active child through Humble's existing action adapter. A deferred
preparation alone cannot move the robot after cancellation. A subsequent,
different path retires it; lifecycle shutdown also clears it.

The existing startup collision check, controller failure tolerance, progress
limits, local MPPI no-control waiting and outer API task timeout remain in
force. A blocked startup turn still produces no valid rotation command and is
handled by the existing bounded controller error wait; recovery does not assert
that free space exists or clear obstacles to create it. This patch does not add
a new obstacle-classification algorithm or unlimited retry/wait budget.

## Ownership and observability

All new implementation lives in `navigation_recovery/` under this package.
The progress checker and wrapper share a small in-process evidence object keyed
by their controller-server node; no failure is inferred from `/rosout`, stale
API text or an uncorrelated topic. The single private ROS service lives in this
package, not in the shared robot interface package.

`[ordinary-recovery]` warnings record preparation outcome, exact path consumption,
initial/final startup yaw error, and whether startup rotation was required.
The BT increments Nav2's existing `number_recoveries` feedback after preparation.
The App's original job remains running; this patch adds no App UI phase or new
public HTTP endpoint. Commands retain
Nav2 -> velocity_smoother -> collision_monitor -> robot_safety -> /cmd_vel -> Ranger.

## Verification and deployment

- `test_ordinary_recovery_state`: real failure evidence, exact goal/path identity,
  one-shot consumption, stale failure, cancellation/new-path replacement,
  other controller selection and lifecycle reset.
- `test_ordinary_recovery_bt`: actual Humble BT control nodes and checked-in XML,
  with non-moving action leaves connected to the production evidence core.
  Covers outer RUNNING then success, one retry maximum, unknown failures,
  cancellation/halt, and stale blackboard failure on a new task.
- `test_ordinary_controller_recovery`: actual progress checker, wrapper, private
  ROS service and RotationShim command generation on an isolated synthetic
  costmap. Idle rearm is set to 100 s, proving immediate recovery is explicit.
  RPP is used as the lightweight primary controller fixture; this is not a full
  MPPI stochastic or physical chassis reproduction.

ROS tests use a separate domain and localhost only. No production goal,
velocity, TF, scan or pointcloud probe is required. Deployment stages a separate
build, verifies source differences and hashes, and atomically replaces binaries
without overwriting an in-use shared library inode. The new BT has a new file
name so the old running BT process cannot accidentally load an unregistered
node before the next **explicitly authorized full runtime restart**.

Hardware acceptance remains outstanding until that restart and a supervised
run: reproduce the stalled ordinary goal, confirm one outer NavigateToPose,
at most two internal FollowPaths, one necessary startup realignment, unchanged
final precision, immediate cancellation, and zero rotation against an obstacle.
Also repeat an ordinary successful route, predock and elevator flows for
non-regression. Do not claim a navigation success rate from software tests.
