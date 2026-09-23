# Separate sensing, stopping and planning geometry

## Scan-mask update (2026-09-21; restart required for activation)

Only the sensing exclusion changes to X +/-0.39 m, Y +/-0.36 m; no MPPI or
collision-monitor parameter changes. StopZone's central lateral visible band
is now zero, while front/rear retain 8 cm. Future ultrasound is not integrated.
See [scope and verification limits](body_geometry_alignment.md#scan-mask-width-update-2026-09-21).

## Current local geometry, 2026-09-19 (not deployed)

The confirmed body is now StopZone-sized, not the smaller old configured body.
Raw footprint is `0.47/0.36`, padded is `0.50/0.39`, all four inflation radii
are `0.75 m`. The unchanged preference critic remains `0.52/0.41`; repair's
retained extra margins make its hard envelope `0.58/0.47` and preferred envelope
`0.63/0.52`. These are different consumers, not interchangeable dimensions.
Native footprint clearing expands and FootprintApproach sees the larger polygon.
See [current verification and hardware boundaries](body_geometry_alignment.md).

## Historical configuration, 2026-09-08 (dimensions below superseded)

All dimensions are half extents in `base_link`:

| Consumer | X | Y | Meaning |
| --- | ---: | ---: | --- |
| Configured unpadded body | 0.36 m | 0.25 m | Unchanged |
| Padded footprint and scan self mask | 0.39 m | 0.28 m | Unchanged |
| Collision monitor StopZone | 0.47 m | 0.36 m | Two visible scan returns stop |
| MPPI preferred envelope | 0.52 m | 0.41 m | Finite trajectory cost, not collision/failure |

Previously the StopZone Y boundary equalled the inclusive scan self mask;
lateral points inside StopZone could already have been removed from `/scan`.
The new polygon provides an 8 cm visible band on all four sides, assuming an
observable obstacle at scan height. This geometric evidence does not prove
the cause of an unrecorded contact.

Do not implement the preferred envelope by enlarging shared `footprint_padding`:
that also expands obstacle-layer clearing and other footprint consumers.
Native `collision_margin_distance=0.08` is an inflation-derived scoring
parameter, not an exact expansion of a rectangle. Native inflation 0.60 m,
collision margin 0.08 m, physical footprint and footprint clearing are unchanged.

## MPPI scoring

`navigation_clearance/RangerClearanceCritic` is loaded by Humble's existing
critic manager after `ObstaclesCritic` in the shared FollowPath MPPI profile.
It adds a horizon-normalized squared intrusion cost, weight 30.0, beginning at
the preferred rectangle and increasing towards StopZone. It never sets
`fail_flag`, publishes a command or suppresses the native collision critic.
Leaving an occupied preference band sooner scores below staying or moving
inward. This is a preference, not a guarantee that selected trajectories remain
outside the band or that StopZone permits escape.

Only lethal cells (254) enter the index. Inflation 1..253 is not counted again
as physical occupancy. Native unknown/out-of-map handling remains authoritative.
Occupied cells are treated conservatively as squares, projecting their half-cell
extent into the robot frame. At 5 cm resolution this adds up to approximately
3.54 cm of grid uncertainty at 45 degrees, not another configured body padding.

A reusable one-metre spatial index is rebuilt from the current costmap each
scoring pass under the existing controller costmap lock. There is no additional
subscription, TF lookup, worker, stale cache or costmap write. Scoring is skipped
inside the existing 0.75 m terminal-scope distance to preserve near-goal behavior.
RPP and elevator-specific controllers do not load this critic. Shared FollowPath
pre-dock transit does receive it; docking contact control and stopped ownership
handoff do not change. Goal tolerances remain 0.06 m / 0.05 rad.

## Local reference repair

The runtime reads the same preference width. It inspects and first searches
with the existing hard repair rectangle (0.47/0.36) plus 0.05 m. If that cannot
connect the start/rejoin, it retries with the unchanged hard repair rectangle.
The searches split the existing 0.40 s expansion allowance and account for at
most 60,000 iterations. Existing one-time cold Dubins lookup setup remains
outside this allowance; this is not a hard wall-clock real-time guarantee.

Attachment uses the original hard repair envelope, not a second preference
gate, so a robot that has moved inside the preference band remains eligible.
Original goal identity, hot `setPlan`, terminal scope, no-control retry and
background-search non-stopping behavior remain. Accepted paths log
`preferred_clearance=true/false`.

## Verification and activation

- Actual Humble critic-manager/pluginlib tests compare clear, intruding,
  escaping and rotated trajectories; preserve native collision failure and
  near-goal behavior; verify footprint/costmap immutability and obstacle clearing.
- The complete checked-in MPPI profile is initialized and called on a synthetic
  map. Batch/timing results are recorded, not claimed as chassis reaction times.
- Repair tests cover preferred detours, a start inside the preference band,
  original-clearance fallback, attachment and the unchanged exact endpoint.
- Static contracts bind both configs, the mask's visible band, StopZone,
  preferred envelope, repair margin and unchanged physical footprint.

Build/tests use a device-free network-isolated container, never the production
ROS graph. Stage verified artifacts atomically without overwriting loaded
library inodes. Activation requires an explicitly authorized **whole runtime**
restart; no individual node restart or live parameter update.

Hardware acceptance remains outstanding: start with supervised low-speed
front/rear/lateral and rotating approaches, then dynamic crossing, obstacle
removal, constrained corridors, ordinary endpoints, pre-dock and elevator
non-regression. Record stopping distance, false stops and control deadlines.
This fixed StopZone is **not certified for 1.2 m/s**. It cannot prove detection
of obstacles removed by the unchanged mask or outside sensor height/FOV.
