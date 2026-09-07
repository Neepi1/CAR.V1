# Elevator-scoped motion

## Scope

`robot_nav_config::ElevatorScopedPlanner` and
`robot_nav_config::ElevatorScopedController` are not general navigation
profiles. They can be selected only by elevator-runtime behavior trees. Door
and cabin transitions use `navigate_elevator_scoped_motion.xml`; nearby
hall-call motion uses `navigate_elevator_hall_call_scoped_motion.xml` so its
startup-heading rule stays independent from cabin-ingress heading ownership.
Schema-v3 four-point reverse-entry effects use the separate
`navigate_elevator_reverse_docking.xml` tree, except source-floor
`reverse_entry_staging`, which uses
`navigate_elevator_reverse_entry_staging.xml` so its lateral-first staging
order cannot change the later reverse-entry/cabin actions.

The runtime mapping is fixed and pose-gated:

- `hall_call`: ordinary Nav2 when the fresh map pose is unavailable or the
  target is farther than 2.5 m; otherwise the hall-call scoped behavior tree;
- `source_landing`: elevator-scoped behavior tree;
- `enter_cabin`: elevator-scoped behavior tree;
- `target_landing`: elevator-scoped behavior tree;
- schema-v3 `reverse_entry_staging`: the reverse-entry-staging behavior tree;
- schema-v3 `reverse_enter_cabin`, `cabin_panel_approach`,
  `return_cabin_center`, and target landing: the reverse-docking behavior tree.

Obstacle policy is intentionally split at hall-call completion. The approach
to `hall_call` remains costmap- and collision-monitor-checked. Once that goal
has succeeded and the exact elevator execution transaction owns `DOORWAY`,
both schema-v2 `source_landing` and schema-v3 `reverse_entry_staging` join the
same obstacle-unchecked policy as later cabin transit: the bounded planner
returns the direct start/goal pair without reading costmap cells, controller
clearance and obstacle-driven route revision are disabled, and the existing
transaction permit routes `/cmd_vel_nav` around collision_monitor. This does
not change either staging motion order; schema-v3 remains yaw -> lateral ->
longitudinal. Permit expiry or every action exit restores the checked route.

The App does not choose planner/controller IDs and never publishes velocity.
An ordinary saved pose, direct pose, docking goal, or delivery mission cannot
select this profile.

The nearby `hall_call` selection closes the startup-rotation failure seen in
`elevator-test-1785871336509-1293896807666816-2`. The robot was about 1.045 m
from the hall-call pose; the goal was mostly behind it while the commissioned
final yaw differed by only about 0.018 rad. Ordinary RotationShim attempted a
roughly 2.62 rad startup spin and aborted when its swept footprint touched a
lethal local-costmap cell. The hall-call policy first evaluates the complete
padded-footprint sweep for the normal turn toward the travel chord. When that
normal turn and direct route are clear, the first commanded motion remains the
in-place turn. Only when that checked route is blocked does the bounded search
translate forward, reverse, or laterally to a pose from which the turn and
onward route are both clear; it then turns and proceeds. It does not erase the
blocking cell or retry an unsafe spin.

## Motion and safety contract

The scoped planner accepts only a finite start/goal pair within 2.5 m. Door and
cabin entry starts in the live start yaw proven by the completed landing step,
while the goal retains its independently commissioned yaw. Its bounded
16-heading, grid-aligned omnidirectional search can compose forward, reverse,
left/right lateral, and checked in-place rotation primitives at every state.
It can therefore make several intermediate turns and translations instead of
rotating to one goal/chord heading and holding it for the complete route.

Nearby hall-call motion has a separate rule: its normal candidate is
`turn to travel chord -> drive forward -> turn to commissioned final yaw`. The
candidate is accepted immediately when every sampled filled footprint is
clear. If that complete candidate is blocked, the fallback uses the same full
16-heading motion set as cabin motion. It may translate, turn at a checked
staging pose, translate again in any allowed axis, and turn again before the
final connection. Therefore a free hall-call start never takes an unnecessary
lateral/reverse escape, while a blocked startup turn is not retried in place or
reduced to one translation-only escape.

For profiles that still use checked planning, an entirely free direct route is
accepted immediately. If it intersects an
exact padded-footprint cell with cost `254` or `255`, the planner does not
erase the cell and does not widen a tolerance: a bounded 5 cm search is
mandatory. A direct route that is hard-clear but crosses soft inflation is no
longer accepted without comparison. The same search evaluates
forward/reverse/lateral alternatives, their mode switches, and integrated
soft-cost exposure. A searched route is selected when its total objective is
lower, or when it reduces soft-cost exposure while remaining within 10 percent
of the direct objective. If no better searched route exists, the hard-clear
direct route remains a valid fallback; inflation therefore remains a cost and
does not become a wall. Global search is bounded by both 60,000 expansions and
a 1.0 s steady-clock deadline. Before expanding 16-heading states, an
8-connected obstacle wavefront expands hard cells by the footprint's inscribed
radius and computes distance-to-goal. A start cell outside that necessary
connected component is reported as `no_path` immediately; it is never
converted into permission to cross a `254/255` cell. A weighted wavefront
heuristic then guides the detailed search, whose filled-footprint edge checks
remain authoritative.

The 16 headings use exact integer grid directions, and each pose yaw is derived
from the same direction used by its forward primitive. The controller therefore
never receives a path whose geometry disagrees with its advertised body axis.
Every translation edge is sampled at 2.5 cm and every rotation edge at 0.05 rad.
Forward is preferred, lateral is slightly penalized, reverse is more strongly
penalized, and every real motion-mode transition, including rotation, carries a
fixed switch cost. The resulting path can clear a door jamb, turn after the
footprint is inside the cabin, and converge to the commissioned cabin pose
instead of retrying one fixed-axis route.

The two schema-v2/general elevator behavior trees bound transient planning
failure before motion to three 0.5 s-separated retries. Both schema-v3 trees
wait and recompute their own commissioned maneuver for up to 30 seconds (60
retries at 0.5 s), so a moving person or door leaf can clear without replacing
the maneuver with an uncommissioned detour. In all four trees the retry wrapper
ends before the single `FollowPath` node. No tree clears a costmap, resends an
old path, or resets an active controller action.

Every scoped `NavigateToPose` attempt now has an explicit controller session.
Before submitting the Nav2 goal, `robot_api_server` sends `BEGIN` to the exact
FollowPath plugin selected by the behavior tree, using
`transaction_id:effect_sequence` as the identity. A new identity clears the
terminal timer, route index, failure state, cached path, blockage history and
pending replan generation. Periodic paths inside that same action retain the
identity and may still hot-swap the remaining route without resetting motion.
All API return paths send matching `END`; if that response is lost, the next
new `BEGIN` is itself an authoritative reset. A stale `END` cannot clear a
newer attempt. Nav2 goal coordinates and path stamps are not used as the retry
identity.

## Schema-v3 reverse-entry policy

The four stored poses are motion contracts, not loose goal hints:

1. `hall_call -> landing`: adopt `landing.yaw` first (commissioning requires
   approximately a 180-degree change), move laterally onto the landing's
   longitudinal axis, then close the remaining forward/reverse residual;
2. `landing -> cabin`: keep the same yaw and reverse along the commissioned
   door axis into the car;
3. `cabin -> cabin_panel`: keep yaw and move laterally toward the configured
   cabin panel side;
4. after floor switch, prove target-map localization and a fresh stable live
   robot pose, then plan from that live pose to target `cabin` and continue to
   `landing`. Source/target `cabin_panel` coordinates are not cross-map pose
   anchors and are never required to match.

The commissioned sequence remains authoritative, but every post-call stage is
obstacle-unchecked: the planner does not read costmap cells, the controller does
not perform command-clearance checks or obstacle-driven replans, and the exact
transaction permit bypasses `collision_monitor`. Weighted A* is not allowed to
replace the prescribed sequence with a weaving route. The pre-call hall
approach and ordinary navigation remain obstacle checked.

The order is covered by the captured B11/F2 start
`(-2.560861, 0.322567, -2.602372)` and landing goal
`(-2.235149, 0.020067, -2.574592)`. In the commissioned goal frame, the move is
43.01 cm lateral followed by an 11.23 cm reverse closure. The former shared
reverse-docking policy started with that short reverse and repeatedly met the
door-frame hard cell. The intent-specific profile preserves the same target,
clearance rule, speed limits, and safety chain while changing only this source
staging order.

Heading ownership is intent-specific. `source_landing` uses the yaw saved with
the frozen landing pose. Completion of that step proves the live start yaw for
`enter_cabin`, while the cabin target keeps its own saved final yaw. The
landing-to-cabin chord is therefore neither the entry axis nor the final cabin
orientation. `target_landing` continues to use the directed cabin-to-landing
exit heading.

Every 2.5 cm translation sample and every 0.05 rad rotation sample must have a
clear filled, padded footprint in the global costmap. The union of the chosen
direct or searched samples is the automatically generated elevator corridor;
it is derived from the current landing/cabin start and goal poses and adds no
App field or map annotation.

This scoped clearance policy does not mutate or replace either Nav2 inflation
layer. The shared global/local `inflation_radius` values are `0.60/0.60 m`,
and the orange/pink costmap visualization therefore remains unchanged. Inside
the elevator-only route, derived non-lethal costs `1..253` are not an
independent rejection reason. Instead, every costmap cell intersecting the
filled footprint is inspected: lethal obstacle or keepout cost `254` and
unknown cost `255` fail closed. A blocked, out-of-map, unknown, non-finite, or
over-distance request fails planning and produces no command.

The controller compresses the dense path into contiguous yaw, lateral,
forward, and reverse segments. It completes and settles one segment before
advancing to the next and never commands two motion axes at once. This is the
important execution contract: a searched detour is followed segment by
segment instead of being overwritten by a direct command to the final pose.
The active segment now constrains the controller error: a planned forward or
reverse edge cannot be replaced by lateral endpoint error, and a planned
lateral edge cannot be replaced by forward endpoint error. Direction is also
one-way for the duration of that edge, so overshooting a segment never makes
the controller reverse back across an already checked corridor. After all
physical segments finish, one explicit `final_pose` stage retains the full
pose residual for bounded final convergence. It preserves the ordinary
`0.06 m` XY and `0.05 rad` yaw goal gate. The elevator-only upper limits are:

- lateral: at most 0.40 m/s;
- forward: at most 0.40 m/s;
- reverse: at most 0.40 m/s;
- yaw: at most 0.50 rad/s.

The velocity smoother admits that envelope (`vx >= -0.40 m/s`,
`|vy| <= 0.40 m/s`) without changing its acceleration/deceleration limits.
While the typed elevator execution session is engaged, `robot_safety` selects
the matching 0.40 m/s reverse/lateral envelope; after that session ends it
immediately returns to the ordinary 0.08 m/s reverse and 0.05 m/s lateral
limits.
Ordinary `FollowPath`, its terminal handoff, goal tolerances, collision policy,
and safety arbitration keep their existing limits.

Before each non-zero command, the local costmap checks the same filled, padded
footprint at the current pose and three projected samples. Translation projects
over the smaller of 0.15 m and the active axis segment's remaining distance;
rotation projects over the smaller of 0.10 rad and the remaining yaw. A short
segment therefore cannot be rejected by a cell beyond its already checked
endpoint. The result identifies the first blocking sample, status, cell, and
cost. The command-producing callback takes the local costmap's standard mutex
and waits for an in-progress map update to finish before evaluating the three
projected samples. Ordinary mutex contention is therefore never reported as
`costmap_busy`, never converted into a false obstacle and never injects a
zero-command pulse. Waits above 20 ms are throttled diagnostics. Transform
failure, out-of-map, lethal and unknown cells remain fail-closed. This catches
a live scan obstacle anywhere inside the robot body, not only on its
perimeter. A genuinely blocked command becomes zero immediately.

At most once per 0.5 seconds, a hard blockage non-blockingly copies an immutable
costmap
snapshot and submits it to a dedicated worker. The bounded search therefore
does not run while `computeVelocityCommands()` owns the controller thread or
the live costmap mutex. The worker has a 60,000-expansion/1-second bound and
searches a complete serial forward/reverse/lateral/spin route from the current
pose to the one canonical final goal. It receives no reference-path suffix,
rejoin candidate, or pending-repair state. The worker publishes no command and
touches no live costmap.

The controller hashes the immutable costmap, quantized start pose, plan
generation, active segment, replan reason, and exact blocking evidence. It does
not repeat a signature already attempted in the action, including A/B/A
costmap oscillation. Changed pose, segment, blockage, or costmap evidence is a
new event and may supersede an unfinished route revision. A successful result
is generation-checked, swept-footprint-checked again against the current live
costmap, transformed into the canonical map frame, and atomically replaces the
whole unfinished execution route. It never splices into a historical route.
The `blocked_timeout_sec=3.0` parameter is a diagnostic `WAIT_CLEAR` notice
threshold, not an action-abort deadline. While waiting, the command remains
exact zero. Lateral and reverse commands also require fresh lifecycle-scoped
permits on:

- `/ranger_mini3/nav_terminal_lateral_enable`
- `/ranger_mini3/nav_terminal_reverse_enable`

Ordinary elevator legs retain the standard output chain:

```text
Nav2 controller
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> collision_monitor
  -> robot_safety
  -> /cmd_vel
  -> ranger_base
```

Every post-call elevator motion (`SOURCE_LANDING_FACE_CABIN`,
`REVERSE_ENTRY_STAGING`, `ENTER_CABIN`, `REVERSE_ENTER_CABIN`,
`CABIN_PANEL_APPROACH`, `RETURN_CABIN_CENTER`, and `TARGET_LANDING`) has the
same transaction-scoped exception:

```text
Nav2 controller
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> /cmd_vel_nav
  -> robot_safety
  -> /cmd_vel
  -> ranger_base
```

The elevator runtime refreshes the exact transaction ID on
`/ranger_mini3/elevator_entry_collision_bypass`; `robot_safety` independently
requires that transaction to own a live execution lease in `DOORWAY` mode.
The permit fails closed on expiry or any lifecycle transition. Source landing
and reverse-entry staging retain their dedicated planners/controllers and
commissioned motion order, while later transit selects
`ElevatorCabinEntryDirect`; every selected post-call profile emits an unchecked
bounded path and disables command-clearance checks and obstacle-driven route
revisions. The permit also bypasses `collision_monitor`. All final safety
arbitration, command freshness, speed limits, and reverse/lateral permits
remain active. Only the pre-call hall approach is excluded.

The profile does not change MPPI, odometry, EKF, AMCL, goal tolerances, TF
ownership, or ordinary navigation behavior.

## Localization correction and route anchoring

The global planner owns map-frame geometry. An obstacle route revision is
searched on an immutable local-costmap snapshot, but the accepted complete
route is transformed back to the canonical map frame before it replaces the
current execution route. The explicit final-pose segment always reads
`final_goal_`, which is the original map-frame goal supplied by Nav2. A snapshot
endpoint is never allowed to become a second completion authority.

This distinction is required by the 2026-08-05 cabin-entry trace. A local
route revision was accepted at `1785944426.527`. Near the end of the same action, AMCL
produced a correction burst whose `map->odom` parameter translation reached
`0.426 m`, followed by `0.205 m` and `0.136 m`. The old execution route had been
serialized in `odom`, so its endpoint did not move with the canonical map goal.
The controller consequently held zero at the stale endpoint while Nav2 still
measured about `0.125 m` of map-frame error; the 20-second progress checker then
aborted. This was not an A* timeout and not evidence that the chassis rejected a
known non-zero command.

The controller observes the canonical goal transformed into the local costmap
frame, but localization no longer owns a motion gate. The `0.08 m` / `0.08 rad`
thresholds remain diagnostic compatibility parameters: crossing either records
one coalesced correction event while the active map-frame target continues to
be transformed through the latest TF every control cycle. It does not reset the
segment/controller, revoke lateral or reverse permits, publish
`LOCALIZATION_SETTLING`, or launch a localization-only route search.

`setPlan()` explicitly distinguishes goal scope. A different canonical goal
performs the full action reset. A compatible same-goal refresh starts at the
current robot pose, replaces the remaining route suffix, and preserves the
active semantic phase and direction. An incompatible same-goal refresh is
ignored instead of interrupting the route already being executed. A real hard
footprint blockage still commands exact zero and may launch one bounded
background four-wheel-steer route revision; exact live evidence is deduplicated
and at most one worker request is in flight. The elevator behavior tree keeps a
single plan followed by a single `FollowPath`; it does not use a 1 Hz periodic
planner/controller pipeline.

Nav2's `SpeedLimit.speed_limit == 0.0` value is also handled as its documented
`NO_SPEED_LIMIT` sentinel. It restores controller scale instead of silently
turning every elevator command into zero; collision and safety stops remain
owned by the fixed downstream chain.

## Progress checking

The controller server loads one `ElevatorAwareProgressChecker`. Its normal
profile is the frozen ordinary-navigation contract: `0.03 m` translation,
`0.05 rad` yaw, and `12 s`. An elevator-scoped controller publishes a typed
state on `/ranger_mini3/nav_elevator_scoped_progress_state`. Fresh `TRACKING`
uses the elevator profile of `0.015 m`, `0.015 rad`, and `20 s`.
`REPLANNING` and `WAIT_CLEAR` explicitly mean that the controller is commanding
zero because a real route blockage is being handled; they pause and reset the
progress baseline instead of turning deliberate waiting into a false motion
failure. Localization corrections remain in `TRACKING`.

This closes the observed source-landing failure where XY error was already
`0.0467 m`, yaw error remained `0.060855 rad`, and the last measured fine-yaw
correction was `0.0407 rad`. That correction was real motion but was smaller
than the ordinary `0.05 rad` progress quantum, so the outer checker reported
`Failed to make progress` before the controller could cross the unchanged
`0.05 rad` goal gate. The elevator profile counts that measured correction and
gives the bounded final stage time to finish; it does not manufacture progress
or relax completion accuracy.

Returning from a paused state to `TRACKING` establishes a fresh progress
baseline. An action reset discards the previous state immediately, so it cannot
lend elevator limits to a following ordinary goal. If the controller stops
publishing for `0.50 s`, the next cycle returns to the normal profile. The state
message carries no command, does not authorize motion, and does not bypass the
costmap, collision monitor, or `robot_safety`.

The clearance Module exposes pose and swept-path evaluation, and the search
Module exposes `search_elevator_scoped_path()`. Planning, route-revision
acceptance, and command execution call the same filled-footprint seam; polygon/cell
intersection and the bounded queue are hidden in their implementations. A
small pure route Module converts dense samples into executable axis segments.
No Adapter layer is introduced because all callers already share `Costmap2D`.
This keeps the special cost interpretation local to the elevator chain instead
of spreading parameter switching across Nav2.

## Failure behavior

Planner rejection includes search status, expansion count, direct-route
clearance status, blocking cell, and cost. Controller total timeout, invalid
or stale state terminates the same elevator-owned `NavigateToPose` action. A
hard blocker with no accepted route holds exact zero in `WAIT_CLEAR`; TF or
costmap contention also waits without launching a search. Changed live evidence
may revise the route in the same action, while identical evidence is not
searched again. The elevator runtime then reacquires its owner hold, waits for
wheel and local odometry to prove a settled stop, and follows the existing
phase-aware cleanup policy.

The captured B11/F2 hall-call snapshot from 2026-08-05 is now a regression
fixture. For start `(-4.054318, -0.965755, 2.246880)` and goal
`(-2.654357, 0.350301, 0.581367)`, the old implementation spent its full second
and returned `time_limit`; the direct candidate first met hard cell `(39,67)`
with cost `254`. The snapshot has no inscribed-footprint connected component
between those poses, so the new search returns `no_path` before expanding any
16-heading state. This is an asset/pose corridor problem, not permission for
the planner to cross the marked hard obstacle. That exact field case needs its
map/pose association corrected before a real goal can succeed.

If Nav2 reaches a terminal result before a new `motion_allowed` sample arrives,
the runtime reports that Nav2 result directly instead of masking it as
`ELEVATOR_MOTION_NOT_ALLOWED`.

The latest 2026-08-05 cabin-entry failure exposed the opposite defect in the
old rejoin design. The initial full route was accepted, a lateral segment was
blocked, and a 0.341 m local rejoin was accepted. Its following forward segment
was then blocked, but `pending_repair_` prohibited a second search until the
unreachable historical rejoin boundary had been completed. The controller held
zero until Nav2 reported failed progress. This was not a missing global path,
not a `robot_safety` rejection, and not a chassis command failure.

Regression coverage now proves both required properties: identical evidence,
including A/B/A costmap oscillation, is searched once; changed blockage or pose
evidence can atomically replace an unfinished route revision in the same
`FollowPath`. Neither production request nor result contains a remaining route,
rejoin index, or pending-repair gate.

## Hardware validation still required

No source-level or non-moving validation authorizes an unattended elevator
run. With an empty stationary elevator and an operator at the emergency stop:

1. verify the schema-v3 release, both panel sides, and all four poses on both
   floors;
2. verify the door is fully open and both global/local costmaps show a clear
   footprint corridor; the visible inflation colors are expected to remain
   unchanged;
3. run one `hall_call -> landing` test and verify the observed order is
   `yaw -> lateral -> longitudinal`; stop on any oscillation;
4. run one `landing -> cabin` reverse test and confirm every logged segment is
   serial, settles before the next segment, and never mixes motion axes;
5. run `cabin -> cabin_panel`; verify the lateral direction matches the
   configured side. After the manually confirmed floor switch, verify the new
   target-map pose barrier and run `live target-map pose -> cabin ->
   target_landing`;
6. confirm no lateral/reverse permit remains true after action termination;
7. confirm `/cmd_vel_api.linear.y` remains zero and every command traverses the
   fixed safety chain;
8. place a rigid obstacle anywhere inside one projected padded footprint and
   confirm the planner/controller rejects it with no non-zero final command;
9. confirm a nearby obstacle cell outside the padded footprint does not cause
   a false rejection solely because its soft inflation overlaps the route.
10. reproduce the F2 door-jamb case and confirm the planner reports
    `detour=true`, every controller transition logs the matching `planned`
    phase/segment, the controller advances through the searched segments, and
    the original lethal cell remains unchanged;
11. introduce a removable front-left local obstacle, confirm final velocity is
    zero while blocked, then remove it or leave a safe right-side corridor and
    confirm the same action accepts a complete right-side-slip route revision
    to the original final goal and leaves no permit latched; repeat as a mirrored
    front-right/left-side-slip case.
12. while forcing a route revision, confirm the 15 Hz controller no longer emits
    repeated `Control loop missed its desired rate` warnings. Leave the
    costmap/start/segment/block evidence unchanged for more than 3 seconds and
    confirm one deterministic search is not repeated, `WAIT_CLEAR` retains
    exact zero, and no controller exception is raised solely at the 3 second
    notice threshold. After one revision is accepted, block its next segment
    with changed evidence before it completes. Confirm the same `FollowPath`
    launches a new bounded search from the then-current pose, replaces the
    unfinished revision, and still terminates at the original map-frame goal.
13. From a stopped pose within 2.5 m of `hall_call`, place a static obstacle in
    the in-place rotation sweep while leaving a checked translation corridor.
    Confirm the runtime logs `profile=elevator_scoped`, the same Nav2 goal stays
    active, translation occurs before any turn, and no
    `RotationShimController detected collision ahead` abort is emitted.
14. Start more than 2.5 m from the same `hall_call` and confirm the runtime logs
    `profile=ordinary_nav2`; ordinary delivery/navigation goals must never
    acquire elevator lateral or reverse permits.
15. From a stopped pose within 2.5 m of `hall_call` with a clear footprint
    sweep, confirm the first non-zero command is pure yaw toward the travel
    chord and that no startup-escape translation is logged. This is the normal
    path; the translation-first behavior in step 13 is fallback-only.
16. Repeat the source-landing case that stopped at about `4.67 cm / 0.0609 rad`.
    Confirm the state changes to `TRACKING`, a measured yaw change
    above `0.015 rad` refreshes progress, the same FollowPath remains active,
    and completion still satisfies `XY <= 0.06 m`, `yaw <= 0.05 rad`.
17. After the elevator action terminates, wait more than `0.50 s` and verify
    the checker returns to its ordinary profile; an ordinary navigation run must
    still use `0.03 m / 0.05 rad / 12 s`.
18. Temporarily block schema-v3 planning before motion, verify no `FollowPath`
    starts, remove the transient blockage within the 30-second bounded retry
    window, and confirm only the newly computed path is followed once.
21. Until the rear lidar is installed and integrated into
    `/perception/obstacle_points`, perform reverse-entry tests only with an
    empty stationary elevator and an operator at the emergency stop. The
    existing front/side sensing is not accepted as commercial rear blind-zone
    coverage.
19. Revalidate the corrected B11/F2 map and commissioned hall pose offline;
    require a hard-cell-connected footprint corridor before sending the real
    goal. Do not treat a repeated `no_path` as a reason to weaken cost `254/255`.
20. Reproduce a supervised 180-degree elevator spin in which the bridge accepts
    a cumulative localization correction of at least `0.08 m` or `0.08 rad`.
    Confirm the same `FollowPath` remains in `TRACKING`, logs the correction as
    diagnostic evidence, and continues calculating yaw error from the latest TF
    without a localization-owned zero interval or localization route search.
    A compatible same-goal refresh must preserve the active spin; a different
    goal must still reset it. Completion must use the original map-frame goal and satisfy
    `XY <= 0.06 m`, `yaw <= 0.05 rad`, with no API terminal fallback and no
    residual lateral/reverse permit.
