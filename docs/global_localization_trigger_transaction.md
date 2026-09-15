# Explicit global-localization transaction

`/global_localization/trigger` owns one complete business relocalization:

1. prove post-reload readiness, a fresh FlatScan, the Isaac trigger service,
   and bridge-arm service before dispatch;
2. arm `robot_localization_bridge` once and retain that immutable arm time;
3. dispatch one Isaac grid-search request;
4. drain any `/localization_result` stamped before the arm window without
   completing or restarting the transaction;
5. admit the current-arm result at its original stamp only when historical
   `odom -> base_link` exists and the latest odom TF is fresh; the field bridge
   retains 30 seconds of history for the wrapper's 20-second result window and
   never substitutes latest TF;
6. require explicit bridge sequence advance, settled bridge-owned
   `map -> odom` before returning success. Navigation admission is not part of
   localization completion; see [responsibility boundary](relocalization_completion_responsibility.md).

The result contains `dispatch_state=not_dispatched|dispatched` on failures.
Startup and FloorSwitch may retry only the former. A post-dispatch timeout may
still be reconciled from an exact newer explicit sequence, but must never issue
another automatic Isaac request. This prevents the observed one-result-behind
loop in which each re-arm made the previous result stale for the new request.

The 2026-09-04 startup failure was a separate timing boundary: Isaac returned a
valid original-stamp result near the end of its request window, but concurrent
held-Nav2 startup delayed bridge handling until that stamp was older than tf2's
default history. Production startup now completes and accepts the initial
transaction before held Nav2 preload, while the explicit 30-second bridge cache
covers the unchanged 20-second result window. Neither measure changes result
timestamps or TF ownership.

Isaac is not a continuous correction source. An unarmed late/background Isaac
result is recorded in bridge diagnostics and cannot change canonical
`map -> odom`. AMCL gated corrections remain the continuous localization path.

This change does not alter sensor QoS/timestamps, TF ownership, Nav2 plugins,
the velocity chain, or any mechanical-arm service.
