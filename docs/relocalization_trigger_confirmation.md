# Relocalization Outcome Confirmation

## Incident and Root Cause

On 2026-09-10 (Asia/Shanghai), the B10/F1 floor transaction completed at
18:52:47. At 18:59:40 the bridge accepted an explicit Isaac result and a
post-Isaac AMCL refinement, but the manual localization HTTP request returned
503. Later localization and floor-switch requests returned 409.

The wrapper used the bridge's **all-source rejection counter** as the terminal
outcome of its current Isaac request. An unrelated AMCL candidate could increment
that counter while the accepted transform was still settling. A later acceptance
could clear the rejection reason, producing `BRIDGE_REJECTED_RESULT` with an empty
reason. The API then retained unresolved side-effect evidence without a late
completion path. The App classified any HTTP 409 as an active floor switch.

This behavior was reproduced with the production wrapper executable and fake
Isaac/bridge peers in ROS domain 217, not by changing live localization state.

## Ownership and Completion

- Existing `TriggerLocalization` stays available for existing ROS clients.
- API uses `TriggerLocalizationTracked` on `/global_localization/trigger/tracked`.
  Each call carries a process-unique request ID. Retrying the same ID reads its
  retained outcome and does not dispatch Isaac again.
- The wrapper serializes trigger and asset operations through completion-status
  publication. An unresolved dispatched request retains ownership. No retry loop
  rearms force-accept or replays Isaac.
- `LocalizationTriggerStatus` carries request ID, map identity, asset epoch/digest,
  localizer generation and baseline/accepted explicit sequence. A late success
  requires the same identity/generation, exactly the next Isaac explicit sequence,
  the direct service response and fresh, settled canonical `map->odom` evidence.
- Rejected AMCL candidates remain diagnostics. They do not determine the Isaac
  request's outcome. Missing results, stale TF or unsettled transforms still fail
  confirmation; thresholds are not widened.
- A 1 Hz node-owned timer only reconciles already received evidence for UNKNOWN
  requests. This is part of the completion protocol, not a diagnostic probe or a
  high-rate ROS polling subprocess.
- API service callbacks and status messages release **only their matching request's**
  admission evidence, exactly once. HTTP timeout/destruction never releases it.
  Late pause/no-motion service replies likewise settle their own evidence.
- App reads structured `code`/`reason_code`; only an actual active transition is
  shown as "floor switch in progress". `DELAYED_SIDE_EFFECT_UNKNOWN` remains an
  explicit unresolved-operation warning.
- App verification also reads `delayed_side_effect_recovery_blocked`; a newer
  pose alone cannot clear the local warning while backend evidence is unresolved.

## Scope and Limits

No changes to TF ownership, timestamps, AMCL matching/gates, MPPI, odometry,
motion commands or startup ordering. No switch to AMCL-only localization.

Terminal history is bounded to 128 retained requests; unresolved requests are
never evicted. History is process-local. A process restart is not presented as
proof that a previous request succeeded. A timed-out arm whose effect remains
unprovable stays UNKNOWN; this patch does not invent a cancellation result or
blindly clear the shared admission count.

This is a coordinated deployment of `robot_interfaces`,
`robot_global_localization` and `robot_api_server`. The old service definition is
unchanged. Updating only the API without the tracked wrapper service is not a
valid deployment. Activate using the complete `njrh-runtime.service` lifecycle,
never by restarting individual ROS nodes.

## Validation

- `isolated_trigger_confirmation`: real wrapper, fake Isaac/bridge, unrelated
  rejection, empty rejection reason, AMCL refinement, no result and no settle.
- `isolated_tracked_trigger_confirmation`: request idempotency, late completion,
  unknown retention and foreign explicit-sequence rejection.
- `test_trigger_evidence`: matching-only, exactly-once release under duplicate
  concurrent callbacks; does not release a different pending request.
- `test_localization_trigger_transport`: real API module with fake ROS service;
  late reply after HTTP wait and UNKNOWN followed by matching status.
- App tests cover actual localization flow, HTTP parsing and error classification.

On 2026-09-10, the final candidate passed all six global-localization CTests,
three selected API CTests and 41 App tests. App static analysis was clean.
After coordinated deployment and a whole-chain restart, stationary manual
localization returned HTTP 200 / LOCALIZATION_COMPLETE in 7.66 seconds, with
explicit sequence 2, required AMCL refinement and post-settle checks successful.
The delayed-operation count remained zero and both admission interlocks were
clear. Running executable hashes matched the tested deployment candidates.

The first startup attempt failed the existing odom->base_link freshness check
(2.32 seconds old). A subsequent two-second stationary observation was fresh
(maximum age about 54 ms). Startup also selected a stale persisted B15/F1 record;
that record was restored to the verified pre-restart B10/F1 map without changing
map assets. The second complete restart became ready; the acceptance checker
waited about 121 seconds. Neither the transient startup TF timeout nor the missing
floor-switch-to-autostart persistence update is claimed to be permanently fixed
by this localization-result change.

Actual map-switch and robot-motion acceptance remain separate. No movement goals
were sent. The installed App has not been updated; only its source and tests were
updated. All test and diagnostic probe processes exited. Full evidence is in
reports/relocalization_fix_20260910/summary.md.
