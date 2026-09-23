# Elevator arm automatic retry candidate — 2026-09-21

## Authorized activation — 2026-09-22

This section supersedes the pre-deployment status recorded below. The tested
candidate was activated with one authorized `njrh-runtime.service` restart,
requested at 10:01:51.937 UTC. No full build was performed: only the two changed
translation units had been compiled, followed by the verified incremental link.

The old API exited before installation. At 10:01:57.587 UTC, while systemd was
`deactivating` and no API process existed, the prepared executable and four scoped
source/header files were installed; the old files remain backed up. No systemd
unit or production startup script was modified. This avoids replacing an open
executable and provoking the process-ownership guard's deleted-executable case.

New API PID `2435585` loads SHA-256
`1efe0b0f53143cb733641204fc01ec900342af0ddfb4131f356f7c93b5b4ffcb`.
All four deployed source/header hashes match the tested candidate. API listening
was logged at 10:02:13.460 UTC; Nav2 lifecycle ready at 10:02:56 UTC and confirmed
navigation context at 10:03:01.006 UTC, approximately 64 and 69 seconds from the
restart request. Existing AMCL status reports ready/static standby. Service
MainPID `2433724` remained active with `NRestarts=0`; API and control-chain
processes were unique. Deferred docking startup logged its node at 10:03:51 UTC.

Deployment evidence and recoverable backups:
`/tmp/njrh_reports/elevator_arm_fault_20260921_T9YgNz/activation_8ygpm5_4`.
Only existing files/processes/logs and read-only API status were checked. No
navigation goal, velocity, arm call, real-robot test or latch-clearing operation
was issued. The 48 isolated tests are not physical elevator acceptance. Retry
log emission under a real failed arm operation remains untested.

## Authority and scope

User choice: continuously retry the current failed arm step until it succeeds or
the elevator test is cancelled. This is not a new elevator transaction, navigation
retry, persistent lock, health gate or mechanical-arm implementation change.

The actual running API was checked at 2026-09-21 16:31:41 UTC: PID 2069542,
SHA-256 `9d828fb181b7c1f790769a05e6ac20f3b255df1988c58518afac03aa3da470ed`.
The deployed arm-client source matched the local starting source:
`934f07bb296cb49be221f888d43dba1ec183e067db01347e6d76bc9a753f1f83`.
Branch: `codex/elevator-config-management`; HEAD:
`8d01ee83994b7e83ab34cddf892b4541504a2959` with unrelated existing worktree changes.

The reviewed fault-propagation follow-up also changes the ROS adapter's internal
call to the arm client and shares its existing cached health predicates. Public
option/outcome/class data layouts and HTTP routes/payload fields are unchanged.
The two button-call C++ signatures add an optional internal runtime-failure probe;
both production callers and the client must be rebuilt together. Elevator FSM,
navigation, safety policies and the arm black box are unchanged.

## Retry rules

| Observation | Client behavior |
| --- | --- |
| Ready task reports `state=failed` | Retry ready, with no button yet. |
| Button task reports `state=failed` | Complete release, then ready and the same button again. |
| Button succeeded, release reports `state=failed` | Retain button success; retry release only. |
| GET transport failure / HTTP 408, 429, 500, 502, 503, 504 | Poll the same task ID within its original task deadline. |
| Unknown POST acceptance, missing task ID, protocol/permanent query error, task deadline exhausted | Preserve an explicit error and existing release cleanup; never blindly resubmit the button. |
| Old hall backend returns `501 capability_unavailable` | Preserve existing manual-confirmation fallback after release. |
| Cancel | No further ready/button attempts; retain one bounded release cleanup. |
| Existing local adapter fault | Return its original code/detail after bounded release cleanup; do not retry the failed runtime or invent a remote terminal result. |

There is no action-attempt count limit. Confirmed failures use a one-second retry
backoff, checked for cancellation at most every 20 ms; successful actions gain no
fixed delay. Each distinct physical attempt uses the original mission ID for its
first submission and `:retry:N` for subsequent submissions. Polls keep their task
ID and never POST again. This avoids intentionally reusing a failed mission ID;
it does **not** prove the external black box's idempotency implementation.

Only the exact task state `failed` permits a new attempt. A transport error,
`terminal=true` with an unknown/cancelled state, or the local wait timing out is
not proof of a remote physical failure. A genuine backend rejection without an
accepted task is not silently retried under an assumed idempotency contract.
Continuous retry cannot guarantee success when the hardware is unavailable.

The existing single-request and accepted-task budgets remain configured by
`elevator_arm_request_timeout_sec` / `elevator_arm_task_timeout_sec` (defaults
12 / 180 seconds); the new loop has no aggregate attempt cap. Connect, partial
send and partial recv now share one steady-clock request deadline; each GET is
capped by the remaining task budget. Dripping responses cannot renew a request
forever. A reply observed after the task deadline is not accepted as timely
success. Protocol/oversized responses are not treated as transient network loss.

Cancellation uses the existing probe; no new mutex or worker is introduced.
It does not cancel the black-box remote task (no such API is introduced).
A current request may finish up to its request deadline. Release preserves the
old bounded, non-cancellable one-task cleanup semantics, including cancellation
arriving just before its POST. No new release is submitted while one is pending;
no further release retry occurs once cancellation is observed. Consequently
cancel/shutdown is **not instantaneous**: cleanup can consume its existing
request + task budgets, plus the current request and OS scheduling delay.

Retry logs go to the existing process stderr stream as `[elevator-arm]` entries,
including task ID, attempt, original error code/message. Repeated GET failures
are logged at most once per second. No ROS communication is added. Actual
production log routing/rotation of these new messages awaits authorized activation.

## Verification and activation boundary

### Reviewed runtime-fault propagation

The synchronous arm effect occupies the business worker, so its outer
`heartbeat/poll_health` cannot run until it returns. The optional internal
failure probe now observes the same cached executor/hold/correction evidence
before ready/button submission, after queries and during retry backoff. No
extra ROS request, arm status/health request, worker or lock is created.

The ordinary monitor still performs its existing mode-renewal recovery first.
The arm probe does not promote cached mode-renewal failure/unknown ownership
into a new rejection; the independent keepalive and existing subsequent business
boundary retain that responsibility. The cached monitor predicates and error
priority have an exhaustive 1024-combination equivalence test.

An observed local fault remains distinct from user cancellation and the black
box's task result. When both are observed together the local fault is retained,
with its original code/detail. A fault-observer exception becomes the existing
runtime-health exception category, not success. Cleanup diagnostics are appended
without replacing that fault. A request already in progress remains subject to
its existing timeout; the code does not claim to interrupt that socket call or
cancel the remote arm task. Before any submission, an already present fault
returns without issuing an arm command. After work has started, one bounded
release cleanup is retained. If a release is already pending it is not repeated;
that task keeps its original deadline and failure stops further release retries.

Follow-up isolation contains 29 arm cases, 4 transport cases, 2 real
SingleThreadedExecutor/arm-client combination cases, 8 executor lifecycle cases
and 5 cached-evidence cases: 48 total, all passed. The 2 combination cases also
pass 5 repeats under private network/IPC/PID/mount namespaces and a separate DDS
domain. Their timer-boundary fault injection verifies worker propagation, not
the original action queue defect or a full RuntimePort/physical elevator run.
The original executor lifetime source contract also passes.

Follow-up source, red/green records and candidate artifacts are staged under
`/tmp/njrh_reports/elevator_arm_fault_20260921_T9YgNz`. Only the client and its
adapter caller require production translation-unit compilation; unchanged
objects must come from the hash-verified current deployed candidate, not a stale
workspace build. Installation, restart and hardware acceptance remain separate.

The black box currently exposes no confirmed field distinguishing physical press
completion from failure while returning to ready. Thus a generic `state=failed`
still retries as requested; no unverified error-code allowlist is introduced.
Without the black-box task contract, unlimited retry cannot also promise that a
physically completed press is never repeated. This external gap remains open.

Tests use an injected external HTTP transport or a kernel-assigned loopback mock
port, never the real 8083 service or the robot ROS domain. Red/green runs cover
query recovery, more than three ready failures, failed-button release/ready/retry,
release-only retry, cancellation and deadline behavior. Real socket tests cover
dripping responses, normal GET/POST payloads, size limits and loopback-only access.

Evidence and staged sources: `/tmp/njrh_reports/elevator_arm_retry_20260921_Jcgzzx`.
Only the affected client and isolated tests are compiled; no complete API build,
installed file replacement, restart, arm command or vehicle motion is performed.
No commit/push. The production executable remains unchanged.

Outstanding: external mission-ID/new-attempt behavior, physical press/release
recovery and App cancellation on hardware; socket connect saturation / large
send backpressure; production log routing. Full API acceptance has not been
performed. These tests are not physical elevator acceptance. A later deployment
must audit affected header dependencies and the actual running candidate, and
must account for the known process-ownership guard reaction to replacing a live
executable; do not replace it under a no-restart authorization.
