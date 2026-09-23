# Confirmed body geometry alignment — 2026-09-19

Local candidate only. No deployment, node/service restart, vehicle movement,
production ROS probe or production binary build is part of this change.

## Business intent and scope

The user confirmed that the existing StopZone is the real physical body.
Planning with a smaller rectangle can accept a trajectory that clears that
smaller body yet brings observed points into StopZone. Correct the physical
model; do not bypass collision monitoring or retain an old motion command.
This addresses a geometric mismatch, not every possible stop after passing.

Both `src/robot_nav_config/config/nav2.yaml` and the Jetson runtime overlay
change only the two footprints and four inflation radii, plus comments.
No new geometry runtime synchronizer, subscription, gate, lock or state machine.

| Consumer | Half X / Y (m) | Interpretation |
| --- | --- | --- |
| Scan self mask | 0.39 / 0.36 | Updated 2026-09-21 sensing exclusion; activation requires restart |
| Raw global/local footprint | 0.47 / 0.36 | Confirmed body, equals unchanged StopZone |
| Padded footprint | 0.50 / 0.39 | Retained 0.03 padding; native collision/clearing |
| RangerClearance preference | 0.52 / 0.41 | Unchanged finite soft cost, not a hard stop |
| Repair hard envelope | 0.58 / 0.47 | Padded footprint + retained 0.08 margin |
| Repair preferred envelope | 0.63 / 0.52 | Hard envelope + retained 0.05 preference |

Padded circumradius is approximately 0.634 m. Adding the retained 0.08 m
ObstaclesCritic margin and rounding outward on the 0.05 m grid gives 0.75 m.
Apply this radius to global inflation, local inflation, MPPI's matching inflation
parameter and local post-keepout inflation. Keep scaling factors unchanged.
Inflation radius is not an additional rectangular hard-stop distance.

StopZone remains unchanged, including its two-return threshold. The clearance
critic still reads its own startup parameters, not live StopZone messages.
Static regression checks bind those values to the confirmed physical geometry.
Speeds, MPPI weights/samples, repair search budgets/retries, goal tolerances,
elevator test policy, docking contact logic and BMS protections are unchanged.

## Effects requiring attention

- Native obstacle-layer clearing now clears the larger **current** padded
  footprint. This includes the retained 3 cm padding beyond the physical body.
  It does not clear all future candidate poses. Collision monitor still uses
  its scan input, not the obstacle layer's cleared cells.
- FootprintApproach consumes the enlarged published polygon. Its 2 s prediction
  and 0.1 s simulation step remain, but a predicted contact can occur earlier.
- Repair is more conservative: its hard rectangle is 1.16 x 0.94 m and preferred
  rectangle 1.26 x 1.04 m. These widths alone are not a corridor guarantee:
  rotation, cell rasterization, uncertainty and Ackermann curvature also matter.
- Global planning, shared-costmap terminal control, pre-dock transit and elevator
  consumers see the corrected geometry. No special controller or safety bypass
  is added, removed or reconfigured.

## Regression method

Tests first reproduce the undersized-body mismatch against the saved runtime
configuration, then run against this candidate. Python contracts bind both
YAMLs, all four radii, longitudinal mask/StopZone separation and lateral coincidence, unchanged policy and derived
repair envelopes. Native tests load the actual YAML into installed Humble
Costmap2DROS and use InflationLayer plus CriticManager/ObstaclesCritic; they are
not merely idealized polygon tests or complete stochastic MPPI runs.

- `test_body_geometry_contract`: both maps, front/rear/sides/corners at four
  orientations, nonlethal center with colliding perimeter, clear/removed input,
  existing unknown and out-of-bounds behavior. Global fixture is a geometry
  check, not a claim that production MPPI consumes the global costmap.
- `test_body_geometry_clearing`: native ObstacleLayer marking/clearing through
  synthetic static observations, inside/outside controls, rotated current body,
  retained future-pose edge obstacle. No real scan raytrace is replayed.
- `test_keepout_and_constraints`: real filter mask add/move/delete/transform,
  post-filter inflation, neutral-mask equivalence and native ConstraintCritic.
  The body-edge case derives its edge from the loaded candidate footprint.

Run native tests only in isolated network and IPC namespaces with a private
ROS domain; never against the production graph. No actual costmap activation,
vehicle goals, commands or devices are needed. Compile only test executables,
not the API or controller production library. Evidence, old/new YAML and
per-test results: `/tmp/njrh_reports/body_geometry_20260919/`.

## Not yet hardware-validated

Historical recorded-grid replay holds old costmap/path fixed; it cannot model
changed clearing or trajectory choices, identify the exact collision-monitor
zone without scan/action evidence, or establish the cause of every old mark.
Native FootprintApproach timing, live scan/TF alignment, closed-loop MPPI choice,
repair detour success/time in new tight corridors, compute deadlines and actual
braking distances need supervised hardware acceptance after separate deployment
authorization. Include obstacle removal, turns, normal endpoints, pre-dock and
elevator non-regression. No claim of safe stopping at 1.2 m/s or elimination of
all post-obstacle stops follows from these tests.

## Scan mask width update (2026-09-21)

At the user's request, both Scan-worker startup YAMLs change only
`scan_worker_self_mask_min_y=-0.36` and `scan_worker_self_mask_max_y=0.36`.
X remains +/-0.39 m. StopZone, SlowZone, MPPI, height/range slicing, frequency,
QoS, DDS, TF and mapping parameters are unchanged. No production build is needed.
The helper's no-parameter defaults are not changed.

The extra excluded strips are `|x|<=0.39` and `0.28<|y|<=0.36` in base_link.
Both self returns and real obstacle returns in these strips disappear from
`/scan` and its downstream `/flatscan`. Within the central 0.78 m length, the
StopZone's lateral visible band is now zero; front/rear bands remain 8 cm.
Ultrasound is a future user plan, not current replacement coverage. This does
not prove the optical cause of returns or certify stopping protection.

Configuration tests check both startup profiles, unchanged X bounds, exact
StopZone geometry and thresholds, MPPI/repair geometry and remaining front/rear
bands. Runtime caches the mask at construction: synchronize files, then wait for
a separately authorized full-runtime restart, and verify active bounds via the
existing status output. Do not use live parameter writes as activation proof.
Supervised hardware validation of side obstacles, false-stop reduction and
localization remains pending; no movement or restart is part of this edit.
