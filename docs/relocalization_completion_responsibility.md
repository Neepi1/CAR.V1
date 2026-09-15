# Relocalization completion and navigation admission

## Evidence

The stationary floor-switch trace `manual-floor-switch-20260914T172123Z-1`
accepted its explicit localization at 17:21:31.764604 UTC, but committed the
target context only at 17:21:44.275379 UTC. During the transition the bridge's
`safe_for_goal_start` remained false because the floor context was not yet
committed. The trigger waited for that flag; the floor transaction waited for
the trigger. The existing 12-second confirmation timeout/reconciliation path
eventually allowed the transaction to finish.

Evidence is retained under `/tmp/njrh_reports/floor_switch_timing_20260914T172123Z`.
This is a completion-responsibility dependency, not evidence that Isaac spent
the entire interval computing a pose.

## Change

- Global trigger success and tracked late reconciliation keep current-request,
  map identity, explicit-sequence, canonical TF ownership/freshness and applied
  correction checks. They no longer require `safe_for_goal_start`.
- The startup helper's late-result observation uses the same responsibility
  boundary; it does not re-trigger Isaac.
- API AMCL-refine completion checks its matching sequence and applied target,
  independently of navigation permission.
- Ordinary App localization requests `wait_for_settle:false`; App result proof
  retains new-sequence, map-pose and completed-correction checks but does not
  require `safe_for_goal_start`.
- Legacy clients explicitly requesting `wait_for_settle:true` retain aggregate
  HTTP status/`ok` semantics. Additive `localization_complete` and
  `localization_detail` preserve the localization outcome separately from the
  existing `post_relocalization_settle_*` navigation outcome.

No floor-specific bypass, new gate, lock, timeout extension, automatic retry,
motion goal or localization algorithm change is introduced. The bridge still
computes navigation admission normally. Navigation/docking settle checks,
atomic map commit, TF smoothing, sensors and the velocity/safety chain are
unchanged.

## Validation and activation

Regression tests exercise the real C++ wrapper and API module with fake peers
in localhost-only ROS domain 217, and the startup callback without ROS entities.
They cover false navigation permission with applied localization, late results,
wrong ownership, stale TF, old/foreign sequences and unapplied corrections.
App tests exercise the HTTP request and completion display.

Candidate build logs, red/green evidence and source/binary hashes are recorded
under `/tmp/njrh_reports/relocalization_completion_fix_20260914`.
The API candidate must reproduce the currently deployed binary baseline before
replacing only this localization translation unit, so unrelated pending source
changes are not silently deployed.

Hardware acceptance remains separate: after an authorized complete service
restart and App update, perform a supervised stationary floor switch and manual
localization. Record trigger acceptance, correction completion and context
commit timestamps. Do not claim a measured time saving before this test.
