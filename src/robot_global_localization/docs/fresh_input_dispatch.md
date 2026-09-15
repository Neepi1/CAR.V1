# Fresh input before the single Isaac request

## Scope and evidence

The 2026-09-13 cold-start capture found raw pointcloud header age near 4.73 s,
scan publication header age near 4.69 s, and FlatScan reception header age near
4.69 s. These are same-period observations, not a per-frame transport trace.
Startup input later caught up. Frequency fluctuation during startup is not
itself a new admission failure or a reason to change the driver.

The wrapper's pre-arm test checked receipt age only. Its post-arm test logged
zero good samples but continued to Isaac. Isaac consumed its one-shot request
on old input; the resulting old-stamped pose was rejected by the bridge.
The source defect was reproduced with the actual pre-fix executable and fake
FlatScan/Isaac/bridge peers in a network-disabled container: failed post-arm
validation still made exactly one Isaac call.

## Corrected behavior

1. Retain the existing reload/service preparation.
2. Before bridge arm, require actual header age and receipt age both within
   `localizer_input_max_age_sec` (unchanged default 0.5 s), a nonzero/nonfuture
   header, and the existing FOV coverage.
3. Arm the bridge once. Keep the post-arm check because an arm response can
   be delayed long enough to invalidate a previously fresh sample.
4. Require the existing two distinct received samples with advancing original
   header stamps, current header/receipt freshness, coverage, and the original
   arm-window constraint. Repeated delivery of one header is not two new scans.
5. Only after that check passes, dispatch one Isaac request. Do not continue
   after a failed check, restamp data, widen result acceptance, or dispatch again
   to repair a stale result.

No scan frequency threshold, fixed warmup delay, new retry loop, DDS/QoS change,
JT128 change, AMCL/Nav2 change, bridge algorithm change or motion is included.
The old input wait parameter remains 1 s **per existing wait**. This patch does
not change its value or repurpose the result-processing budget. The resident
startup coordinator already retries proven `not_dispatched` wrapper preflight
failures within its existing outer budget; these are not Isaac computation
requests. Sustained input failure still exhausts that budget normally.

## Tracked outcomes

`arm_attempted`, `arm_confirmed`, and `dispatch_attempted` distinguish three
facts. `dispatch_attempted` is set before `async_send_request`, so a send
exception cannot falsely prove no request escaped.

- No arm attempted: known `LOCALIZATION_NOT_DISPATCHED`.
- Arm explicitly acknowledged, but no Isaac call attempted: also known
  `LOCALIZATION_NOT_DISPATCHED`. Detail retains `bridge_force_accept_armed=true`
  for post-arm input failure. This does **not** claim cancellation or disarm;
  no computation was requested. Repeating the same tracked ID returns its old
  outcome without rearming. A new explicit request may prepare normally.
- Arm response unknown, or Isaac dispatch attempted: retain UNKNOWN. A later
  definite arm acknowledgement with no attempted Isaac call resolves to a known
  not-dispatched/rejected outcome by reading that same future, without issuing
  anything else. Dispatched requests retain their original result reconciliation.
  Do not fabricate cleanup or automatically repeat Isaac.

## Verification and activation

`isolated_fresh_input_dispatch.py` runs the real wrapper with fake peers under
`ROS_DOMAIN_ID=218`, `ROS_LOCALHOST_ONLY=1`, inside a device-free container with
network disabled. It deliberately separates header age from receipt age.
Both legacy and tracked interfaces are covered. Existing request-idempotency,
late-result, AMCL-rejection, and asset-reload regressions remain required.

The baseline RED case is `--case post_arm_stale --tracked`: expected zero Isaac
calls, actual one. Test windows use an explicit 3 s override to exercise old
input followed by new input within one call; production defaults are unchanged.

Reports, preimage copies, candidate build and deployment hashes are stored in
`/tmp/njrh_reports/isaac_fresh_dispatch_20260913_4vJecI`.
Source/binary deployment is not activation. A separately authorized full
`njrh-runtime.service` restart is required; do not restart one localization node.
Cold-start hardware acceptance still needs original-stamped input, dispatch,
result and bridge acceptance evidence. The separate planner lifecycle timeout
is outside this fix, so no whole-chain ready-time or success claim is made.
