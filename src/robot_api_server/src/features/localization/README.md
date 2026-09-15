# Localization

`localization_feature_module` is the aggregate boundary for the complete
API-side localization family. It owns both `localization_module` and
`post_relocalization_settle_module`, including their cross-domain port wiring.
Construction is deliberately completed in two phases because navigation needs
the localization facade while the settle barrier needs navigation's local
costmap evidence. Both phases finish before the HTTP gateway starts, so this
is dependency resolution only and is not a runtime fallback. The API root owns
one localization aggregate and no individual localization submodule.

`localization_module` is the API gateway's top-level localization module. It
owns the resident `/tf` and `/tf_static` observations, composed
`map -> base_link` pose cache, `/localization_result` cache,
`/localization/bridge_status` cache, Isaac trigger client, bridge-correction
pause client, AMCL no-motion client, AMCL post-Isaac refinement wait, and the
`POST /api/v1/localization/trigger` transaction.

Ordinary localization does not request navigation admission. AMCL refinement
completion requires the matching explicit sequence and applied TF target, not
`safe_for_goal_start`. HTTP responses expose `localization_complete` and
`localization_detail` independently of the optional legacy `wait_for_settle:true`
navigation postcheck. The legacy aggregate HTTP status and `ok` remain compatible;
navigation/docking consumers retain their own existing settle checks.

The module consumes the canonical TF tree but never publishes TF. In
particular, it cannot become a second `map -> odom` or `odom -> base_link`
owner. Navigation, docking, teleop, status, and floor-switch code consume
explicit snapshots or commands through its public facade. The composition
aggregate supplies only cross-domain ports for floor/elevator admission and
delayed side-effect accounting. Its settle callbacks delegate to the sibling
module described below.

`localization_configuration_module` is the construction-time owner of all 48
API-side localization, TF-observation, AMCL-refinement, bridge-acceptance, and
post-relocalization-settle ROS parameter declarations. It preserves the
existing frame normalization, defaults, clamps, and dependent bounds, then
returns the existing localization and settle configs plus the floor-health
topic projection. Shared service timeout and the navigation-owned AMCL
no-motion endpoint remain explicit cross-domain inputs/projections. It starts
no process, triggers no localization, publishes no TF, and adds no gate.

`post_relocalization_settle_module` owns the complete stability transaction
between an accepted explicit localization sequence and the next motion stage.
It owns the settle state and mutex, sequence/TF/bridge/local-costmap checks,
large-correction minimum wait, zero-command cadence, cancellation, timeout,
and the intentionally warning-only post-undock exceptions. It receives narrow
read-only/effect ports from localization, navigation terminal runtime, and
teleop. It does not publish TF, alter the accepted correction, relax ordinary
settle failures, or bypass `robot_safety`.

`amcl_runtime_status` owns the read-only interpretation of the TTL-bound
environment file written by `run_amcl_shadow_localization.sh`. Its single
interface accepts the file path, current wall-clock time, and TTL, then returns
the parsed snapshot used by status reporting and navigation readiness
decisions.

The module preserves the existing file contract:

- a missing or unreadable file is unavailable and stale;
- a readable file is available even when fields are incomplete;
- the last duplicate key wins;
- writer escaping, boolean spellings, and numeric fallbacks are unchanged;
- `seeded` includes explicit seeded, seed-success, or seed-response evidence;
- negative clock skew is clamped to zero age;
- a snapshot becomes stale only when age is strictly greater than its TTL.

The parser does not start or stop AMCL, inspect the ROS graph, publish TF,
select a floor, or decide whether stale data is authoritative. Those
responsibilities remain with the top-level localization module, runtime
scripts, localization bridge, and consuming domain policies.

`localization_result_model` owns immutable localization-result snapshots and
diagnostic text. `tf_pose_utils` owns frame normalization, angle wrapping,
timestamp helpers, and quaternion-to-yaw math. Neither module publishes TF or
changes localization readiness.
