# App navigation resume after mapping

## Evidence and scope (2026-09-15)

The `api_resume` start at 14:17:09 UTC reached confirmed context readiness at
14:19:41.697 UTC (152.7 s). Existing logs showed a retryable localization-input
check publishing `failed` before the caller returned to `starting`; App treats
the failed context as terminal. It also inherited an already-closed common-start
CPU session, keeping new navigation wrappers on CPU0,1,4 during initialization.
Nav2 nodes existed early, but lifecycle work waited for localization-stack
readiness. The optional pre-trigger bridge baseline query spent 10 s and
returned no sequence; normal wrapper acceptance then succeeded.

Evidence: `/tmp/njrh_reports/navigation_resume_20260915/`. These are observations
of the recorded run, not measurements of the repaired version.

## Implementation

- Initialization checks still have the existing bounded observation budget and
  retry healthy owners, but publish only `starting`, never transient `failed`.
  Terminal local-state failures, child exits and cancellation remain unchanged.
- After existing ready-runtime reuse and after cleanup traps are installed,
  API resume drops its inherited CPU session/old common receipt and opens its
  own optional eight-core startup session for normal EKF navigation. It owns
  only newly launched navigation descendants, not the API, common sensors,
  mapping or arm. Existing readiness/exit events restore original steady masks.
  `NJRH_STARTUP_CPU_BOOST_ENABLED=false` remains supported.
- With existing background lifecycle/preload enabled, accelerated EKF API resume
  stages Nav2 creation after the existing map/Isaac initialization check, still
  before the initial localization result. The staging flag defaults to true in
  this branch and respects explicit false. One owned worker configures all nodes,
  then activates them in order, concurrently with the localization request.
  The launch receipt remains a background wait; it cannot block dispatch.
  Legacy/direct paths and full common cold-start ordering are unchanged.
- A terminal initial-localization failure on API resume no longer enters the
  unbounded later-result observer. The existing wrapper first consumes its
  bounded confirmation/reconciliation and pre-dispatch-only retry policy. Then
  the App start exits through existing cleanup, retaining the original failure
  detail/code instead of overwriting it with generic failure or `starting`.
  Input initialization in progress, cancellation and floor handoff keep their
  separate existing behavior; common sensors/CPU policy are unchanged.
- `NJRH_INITIAL_LOCALIZATION_SEQUENCE_BASELINE_WAIT_SEC` defaults to `0`, which
  performs no ROS query. Positive explicit values retain the former diagnostic
  observation. An absent baseline stays absent: no invented zero/old sequence,
  and no sequence-based timeout reconciliation. Normal wrapper acceptance still
  requires its transaction's positive explicit sequence and actual map->odom TF.

No new navigation permission, lock, lifecycle client, localization retry, or
motion command is added. There is no API ABI/protocol change or APK change.
No lidar, IMU, FAST-LIO, DDS, map asset, TF ownership, safety, docking, or elevator
parameter is changed. Runtime production changes are limited to the startup
shell script; no API binary rebuild is required.

## Validation and limits

Targeted pytest uses actual shell orchestration with fixture-owned processes
and fake ROS/OS boundaries, including a deterministic dependency barrier proving
configuration progresses while localization is still pending. CPU CLI tests
prove API/arm/common siblings are untouched. Existing lifecycle tests exercise
all-configure/all-activate ordering, retained late replies, handoff and cleanup.
The old waiting-client assertion requiring `safe_for_goal_start=true` is updated
to the already-deployed localization-only contract; the trigger client itself
is unchanged. Do not restore that removed navigation-admission dependency.

Test/deploy records go in `/tmp/njrh_reports/navigation_resume_fix_20260915_01`.
Run Jetson tests in a separate network/device-isolated container, never against
the production ROS graph. Deployment backs up and checks exact preimages, then
replaces scoped files atomically; README edits preserve unrelated local/remote
differences.

Separately authorized acceptance: stop navigation, map/save/stop mapping, then
start the physically correct selected map through the App. Record start time,
Nav2 worker/configuration, Isaac initialized/trigger/accept, Nav2 active, context
ready and CPU-session restoration. A ready reused runtime must not open a new
CPU session. No motion is required for startup timing.

Existing outer lifecycle timings (planner 35.053 s, controller 16.842 s) include
discovery and response waiting, not proven native configure execution time.
Use existing `NJRH_NAV2_LOG_LEVEL=info` and native node timestamps during an
authorized timing run if that split is needed. The changes remove known serial
waits; they do not guarantee a 10 s or 40 s resume without hardware measurement.
