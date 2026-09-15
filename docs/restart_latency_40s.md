# Full-runtime restart latency work

## Acceptance target (2026-09-12)

The operator authorized autonomous complete `njrh-runtime.service` restarts to
reach less than 40 seconds, including old-chain shutdown, and at least five
tests. Completion requires the same final version to meet that target in five
recorded full restarts; a systemd active state alone is not navigation readiness.
Nav2 activation, exact-map context ready, AMCL ready/seeded and API availability
are measured separately. No robot motion, independent node restart, sensor or
localization tuning is authorized by this latency work.

The starting baseline `/tmp/njrh_reports/amcl_inputs_restart_afZNKTfQ` measured
11.77 seconds for the restart command, 110.3 seconds for Nav2 activation,
128.58 seconds for the navigation context, and 250.3 seconds for AMCL readiness.
Reports and candidate backups live under `/tmp/njrh_reports/restart_40s_QLBMw4GG`.
The 40-second target is **not yet accepted**.

First trial `phase1_trial01_i9likdfm` (restart at
`2026-09-12T01:23:00.345Z`, Jetson clock) measured API 75.80s, Nav2 146.65s,
context 177.63s and AMCL 197.65s. It failed the target; AMCL was earlier than
the baseline, but Nav2 and context were slower. These mixed results are not
five-run acceptance evidence.

## Cold status observer correction

The first trial's early AMCL runner exited before creating its node because
the resident status observer was still unavailable. The `starting` progress
submission had incorrectly become a prerequisite for initialization. Only
`starting` and `waiting_seed` progress (`ready=false`, `degraded=false`) now
permit a failed submission with an explicit warning. The independent startup
owner checks remain, and final `ready`/`disabled` evidence must still be
committed successfully before reporting READY. Failure/degraded semantics are
unchanged. Tests exercise the actual shell functions with a unavailable IPC
boundary, not a replacement startup implementation.

Targeted regression: 35 passed (early startup, startup sequence and composite
inputs). Deployed correction trial `amcl_progress_trial02_mugu9xzc` measured
AMCL 136.73s, Nav2 153.73s and context 174.91s. Early AMCL now really launches,
but this run also needed one automatic systemd restart after cleanup residuals;
it fails both latency and single-start stability acceptance. See its
`analysis.md` and `journal.log`. No navigation motion was used for validation.

## One-shot client auxiliary endpoints

Readiness and lifecycle helpers do not serve their own parameters or require
ROS log publication. Their client-only nodes now disable parameter services
and rosout at construction. The C++ readiness helper also disables its unused
parameter-event publisher. Humble rclpy retains its parameter-event publisher:
no private-field destruction or unsupported constructor option is used.

AMCL seed/scan-status/pose waiters, the no-motion client and the startup global
localization trigger client use the same two Python options. This does not
disable target-node parameter services, remove any TF/subscription checks,
change initial parameter overrides/ROS clocks, or add/remove localization
triggers. Console logging, client caching and lifecycle ordering stay intact.

Offline startup regressions: 139 passed, 4 environment-related skips. The
private-domain smoke test compares actual endpoint graphs to the prior binary,
exercises real lifecycle service responses, fresh/stale synthetic odometry,
and the Python initial simulated-clock override. Existing composite-input
scenarios separately cover TF/Scan/map behavior. Isolated timing is not a
production restart benchmark; a full-restart measurement is required after
deployment. See `isolated_startup_clients_smoke.py` in `src/robot_bringup/test`.

## Shutdown snapshot correction

Trial 02 recorded lifecycle CLI children remaining after runtime shutdown, then
an automatic service restart. The cleanup sweep now rechecks its process class
after waiting for an old PID snapshot and performs a final CLI sweep after
stopping runtime owners. Process-query executables (`pgrep`, `pkill`, and awk
variants) are not runtime nodes merely because their search arguments match a
node name. Real node arguments containing those words remain in scope. Signal
escalation budgets, process ownership patterns and failure on live residuals
are unchanged. Seven isolated shell regressions cover these races, read-only
`--check`, and an unkillable synthetic process. Full-restart validation remains
required; the tests send no signals to live robot processes.

Trial 03 (`lean_cleanup_trial03_3xtbj6jq`, Jetson clock
`2026-09-12T01:59:26.519Z`) measured API 84.83s, AMCL 133.48s, Nav2 146.48s,
and context 168.50s. The new service did not automatically restart, but old-chain
shutdown still reported late PID 1578681; startup then spent another five
seconds cleaning it. Its command/state were not captured, so no identity is
inferred. Residual errors now include exact PID/PPID/STAT/argv for diagnosis.
This run fails acceptance despite the final service being active.

## Avoid redundant CLI/environment startup

The API launcher reuses the ROS and project environment prepared by common_env,
restoring missing setup layers and reloading local setup after a build. It
directly execs the same installed API binary through the existing affinity
helper; arguments, build fallback, supervisor and failure propagation remain.
This removes redundant full setup and ros2 run overhead, not readiness checks.
Synthetic installation tests exercise actual launcher code before deployment.

The local-state launcher resolves robot_localization once with ament's package
index API instead of two ros2 CLI invocations. Overlay precedence, the missing
package error and the executable check remain. A stationary measurement at
`2026-09-12T02:04:34Z` on CPU 0,1,4 returned `/opt/ros/humble` in all cases:
CLI calls 1.877s + 2.193s, direct index 0.452s. This isolates lookup overhead;
it does not explain all observed local-state startup delay.

Trial 04 (`cli_overhead_trial04_f4c_ueqs`, Jetson clock
`2026-09-12T02:08:36.534Z`) measured API 61.59s, Nav2 189.47s, context 222.29s,
and AMCL 232.47s. Old-chain stop succeeded; the new service had zero automatic
restarts. API was earlier, but whole-runtime readiness regressed. It is not an
accepted run. At 02:11:03--13, CPUs 0/1/2/4 were 98--100% busy; planner main
thread spent 8.78s runnable but waiting for CPU and ran for only 0.739s. A later
02:12:52--57 sample was steady-state, not evidence of startup process ranking.
These observations cannot prove a DDS deadlock or isolate all latency causes.

The same run's accepted localization response exposed explicit sequence 1,
correct bridge ownership, TF publication gap 20ms and result age 233ms. The
subsequent separate TF observer timed out with `map` absent from its new buffer,
then startup waited for the already-accepted correction again. This is evidence
for examining observer continuity, not permission to remove actual TF proof.

An isolated no-network dummy-launch A/B (`launch-ab/summary.md`) passed 14
checks. Median first-child latency was 0.681s through ros2 CLI versus 0.509s
through the public launch API; total latency was 0.931s versus 0.731s. That is
only about 0.2s, not the production 22s gap. Production launch entrypoints were
therefore left unchanged.

## Keep the trigger client through actual TF confirmation

The startup trigger helper now optionally checks actual `map->odom` TF using
the same ROS node after an accepted service response. Its volatile TF reader
is created only after acceptance, preserving the old independent observer's
queue boundary. It does not trust wrapper text as actual TF reception, reuse
pre-acceptance TF messages, restamp transforms, use private rclpy take APIs, or
change the TF owners/QoS. Short executor spins keep the node responsive during
the original TF wait. The outer timeout includes that formerly separate TF
budget. RPC acceptance, explicit sequence and floor handoff remain distinct
from TF observation. Legacy helper output still uses the old observer.

An initial prewarmed-reader candidate was rejected: a real isolated ROS test
queued 20 old TF messages before acceptance and none afterward, yet clearing
the tf2 buffer still let it report success. The retained-node/new-reader design
addresses that queue, not just the tf2 cache. Failure evidence is preserved in
`trigger-tf/red-results.json`. The corrected candidate passed all five actual
isolated ROS cases, including that unchanged queued-message stimulus; it issued
exactly one service call per case. Offline regressions passed 23 focused checks
plus 68 related startup checks. No production TF is published by these isolated
tests. Whole-restart timing remains necessary afterward.

## Trials 05 and 06

Trial 05 (`trigger_node_trial05_o1ozrm07`, Jetson clock
`2026-09-12T02:29:07.286Z`) measured API 67.66s, AMCL 113.71s, Nav2 127.71s
and exact context 154.45s. The retained trigger client reported actual TF
reception, without MAP_TO_ODOM_TIMEOUT or a later-localization fallback.
AMCL's parent joined later, but that is not its readiness timestamp. The target
is still unmet. CPU 0/1/4 remained approximately 98% busy during T+15--70s;
the 5-second /proc recorder covers only T+0--115s, not final context readiness.
It consumed about 2.09 CPU seconds across that interval, so it is not free.

The trial also exposed a shutdown ownership bug: a bare `orbbec_camera`
process pattern selected an independently managed camera (`camera`, serial
CV2T6610007C). Its replacement became a cleanup residual and added about five
seconds before the new common chain. Navigation cleanup now matches only its
canonical camera336l launch argument/container namespace, retaining its wrapper
and perception owners. Optical-frame strings and similarly prefixed namespaces
do not prove ownership. The external camera/black-box service is not modified.
Nine synthetic-process shell tests cover selection, shutdown races and read-only
absence checks. A full restart must additionally confirm that the other camera's
PID/start identity survives while the old navigation camera is replaced.

Trial 06 (`camera_scope_trial06_i0zmxg2l`) took API 79.59s, AMCL 125.56s,
Nav2 138.56s and context 159.95s. External camera PIDs 1611069/1611539 and their
start ticks survived unchanged. This proves ownership isolation, not faster
navigation startup. Stop still failed because the inventory omitted the actual
`/occupancy_localization.launch.py` parent (it only listed the `_stack` variant).
The surviving PID 1612720 respawned component PID 1623809 during cleanup, as
shown by the recorded PPID/argv. The exact launcher path is now included;
ten synthetic cleanup tests retain both parent and child absence checks.

## Final-context observer continuity

Normal final startup now uses `observe_startup_context.py`: one short-lived
node discovers the wrapper service, then creates a new volatile bridge reader.
It makes no service call and retains the original reliable/volatile sensor-data
depth-five QoS, exact accepted-sequence check, handoff and durability checks.
An adopted floor handoff keeps its existing service check. Missing service does
not use the bridge fallback; missing bridge status retains the old accepted
transaction plus fresh TF/owner fallback. Legacy callers retain their old path.
Timing logs separate observer creation, service, bridge, owner, AMCL status and
durable context write. The new helper is in the cleanup inventory. Focused
helper/shell regressions passed 26 cases; combined with cleanup, 36 passed.
Actual isolated DDS checks and the next full restart remain deployment criteria.

The API asset-verification hypothesis was measured separately, not assumed:
13 canonical map manifests verified in 0.920s wall / 0.845s CPU against installed
libraries; the earlier count of 39 also included backups. This does not explain
the long pre-listen interval, so no map verification or digest rules were changed.

## Background lifecycle scheduling (trial 08 candidate)

Trial 07 (`context_commit_trial07_6a2_ncyh`) had a clean old-chain stop,
zero automatic restarts and unchanged external-camera process identities.
API/AMCL/Nav2/context took 69.29/121.21/133.21/153.21s; it still failed 40s.
The final observer measured creation 3.428s, service discovery 1.785s and bridge
reception 0.496s; the complete final commit took approximately nine seconds.

The systemd runner now forwards `NJRH_NAV2_LIFECYCLE_BACKGROUND_START`, with
its disabled default retained. With both this flag and
`NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK` enabled, lifecycle
activation starts after localization-stack initialization and before the trigger.
Previously the early branch ignored AFTER and the runner omitted START.
The experiment enables those two flags in the deployed runtime.env only;
parallel-core lifecycle configuration remains disabled.

Joining the bounded lifecycle worker now reuses its actual confirmed result,
without repeatedly spawning fresh lifecycle observers while it runs. A failed
worker permits one active-state fallback pass only outside a floor handoff.
Its PID remains owned until join completes, so existing signal cleanup and
handoff quiescence remain valid. The parent, not the helper, writes READY.
Sixteen new behavioral tests cover scheduling, forwarding, success/failure,
TERM cleanup and handoff; 78 related tests passed with two environment skips.
Full-restart timing and navigation readiness remain separate acceptance work.

Trial 08 (`background_lifecycle_trial08_y0by_8av`) had a clean stop and no
automatic restart. API/Nav2/AMCL/context measured 69.91/130.29/143.29/151.15s.
Lifecycle activation genuinely started before the trigger completed, but the
whole-runtime improvement was not material and AMCL was later. This is not a
qualifying run. The external-camera identities again survived unchanged.

## Retain pending lifecycle responses

The lifecycle helper formerly removed an outstanding ChangeState request after
its short response timeout. Humble rclpy then discarded any later response to
that request. The same real operation could therefore succeed while startup
waited on a slower GetState reply. Trial 07 recorded configure timeout 5.138s
followed by a 17.256s GetState wait; it did not record the original late response,
so the entire 17 seconds cannot be claimed as avoidable.

After the unchanged short timeout, confirmation retains the original future
and reads actual lifecycle state in the same node. It spins during GetState
discovery as well. A delayed successful transition is accepted only in the
existing trust-response mode; non-trusting AMCL still requires state evidence.
No timed-out or response-exception transition is reissued. The confirmation
budget, ordered transitions and next-transition handoff permission remain.
Outstanding requests are cleaned on every return/exception path. Behavioral
tests include delayed, missing, rejected, exception and undiscovered-service
cases, non-trusting callers, old-state redispatch and handoff. The lifecycle
suite contains 26 cases; lifecycle/AMCL/occupancy regressions passed 46 cases.

The trigger helper separately adds monotonic stderr timings around its existing
initialization, discovery, RPC, accepted-TF and cleanup calls. Main-to-return
total excludes module imports; shell process timing includes them and cleanup.
Baseline observation gets its own shell timing. No input/RPC/TF behavior or
timeout is changed by instrumentation. Timing/trigger/startup regressions
passed 108 cases with one environment skip. Isolated DDS and full-restart
results must still be evaluated separately from these deterministic fixtures.

## Trial 09 and next bounded initialization experiment

`retained_response_trial09_0bbsdtz_` restarted at Jetson UTC
2026-09-12T03:34:06.651142Z. Complete shutdown/start measurements were API
71.526s, Nav2 138.349s, confirmed map context 162.803s and AMCL ready/seeded
181.349s. Service stayed active with NRestarts=0. Retained lifecycle responses
were observed, but this did not improve overall startup. The first seed client
timed out; a later attempt succeeded. Client timeout alone is not proof that
the first request was not executed by the bridge. No run qualifies for 40s.

Fast DDS build provenance rules out an accidental Debug/O0 build: installed
2.6.12 uses Release/-O3 and the expected system library hash. This does not
establish or rule out a version-level performance regression.

The next experiment sets `NJRH_NAV2_PRESTART_AFTER_LOCALIZATION_STACK=true`
only for systemd cold startup. Existing map/Isaac/input initialization completes
before the held Nav2 process group is launched; that launch still precedes the
initial localization request and does not wait for a localization result.
Its purpose is to test whether avoiding the initial simultaneous Nav2 process
construction reduces contention. No extra readiness criterion, fixed delay,
CPU allocation, sensor parameter or localization timeout is introduced.
Opt-out and non-systemd starts keep their existing ordering. Floor handoff
must retain ownership and may not cause this branch to launch an old map.
Default remains false until an explicit deployment choice enables the trial.

Validation must cover actual shell order, launch ownership/failure, floor
handoff, opt-out and non-systemd behavior before deployment. Full restart
measurement, not isolated function timing, decides whether this trial helps.
Deployment and all measurements stay under the report root above; physical
navigation, elevator travel and docking remain outside stationary acceptance.

Trial 10 (`staged_nav2_trial10_b4fk6x_p`, Jetson UTC
2026-09-12T03:53:49.061733Z) completed with a clean stop and NRestarts=0:
API HTTP confirmation 39.711s, AMCL 95.938s, Nav2 100.938s and exact context
118.078s. API's own listening log was 34.070s, versus 64.494s in trial 09,
so the early improvement is real rather than a faster observer. The maximum
readiness time fell from 181.349s to 118.078s in this one comparison; further
runs are needed to establish repeatability. There are still zero qualifying
40-second runs. Jetson targeted startup regressions passed 96 cases.

Remaining visible serialization: held Nav2 process launch was recorded at
resident elapsed 27s, but background lifecycle dispatch at 42s. The function
currently waits for the launch receipt before forking its background worker.
That is a measured scheduling boundary, not proof that its entire 15s can be
removed from end-to-end time. Planner configure then took 23.255s and controller
configure 6.568s. Do not change readiness evidence or declare success from a
new background task merely being launched.

## Held-launch wait overlap

The latest measured full restart (`user_restart_20260912_25u2bwmd`,
2026-09-12T13:08:54.808Z) reached API/AMCL/Nav2/exact context at approximately
37.36/103.19/112.19/130.55 seconds. Resident startup logged localization-stack
readiness at 31 seconds but lifecycle dispatch only at 46 seconds. The launch
receipt wait was still in the parent, delaying the initial localization flow.

The existing lifecycle worker now owns both the held-launch receipt wait and
the subsequent unchanged lifecycle sequence. The parent records and retains
that same PID, then proceeds with the existing pre-trigger baseline and Isaac
request. It joins the worker before final Nav2/context readiness. Foreground
startup and disabled-background behavior retain their original ordering.

The worker's receipt check observes the sibling launch PID without reaping it
or writing runtime context. Exit code 70 is reserved for missing/invalid held
launch proof and cannot use the existing active-state fallback; actual lifecycle
failures retain that fallback. Pending floor handoff still joins the worker and
prevents old-map lifecycle dispatch. This change does not alter handoff outcomes,
timeouts, trigger counts, CPU placement, AMCL preparation, TF/map evidence,
navigation parameters, collision policy or the separate composition candidate.

`nav2_lifecycle_worker_started` is a parent scheduling stage, not activation
success. The child logs `held Nav2 receipt confirmed` before starting lifecycle
requests, without writing shared startup status. Tests exercise the actual main,
worker and receipt functions with isolated fake process/ROS boundaries.
Deployment is file-only; an authorized full-service restart and its measured
readiness remain necessary to validate latency. The observed 15-second wait is
not a promise of 15 seconds saved end-to-end.

## Final context observer overlap

The authorized full restart `authorized_restart_iapj058g` (UTC
2026-09-12T21:31:06.134292Z, including old-chain stop) reached core Nav2 active
at 105.866s and exact context ready at 127.965s. Final context commit alone took
11s: observer creation 1.932s, service discovery 1.930s and bridge observation
5.082s. The earlier held-wait change worked, but whole-startup improvement was
only 2.587s in this single comparison; these are not additive guaranteed savings.

Normal systemd startup with an accepted explicit localization sequence and a
pending background Nav2 lifecycle worker now starts one bounded native context
observer before joining Nav2. It opens one low-frequency bridge reader early,
without publishing data, making RPCs, changing parameters or claiming READY.
Only the final parent commit writes its private atomic marker. The same node
then rechecks service graph presence and accepts a bridge status only if its
actual DDS source timestamp is at/after both the marker and service check.
Prewarm/queued/zero/future timestamps cannot supply proof. Final exact sequence,
map identity, persistence/readback, wrapper ownership and existing fallbacks
remain where they were. A service graph entry is not proof of RPC responsiveness.

The observer uses Humble's public C++ MessageInfo interface: Humble rclpy's
ordinary callback does not expose that metadata, so neither executor internals
nor sensor/TF timestamps were modified. The unchanged Python observer remains
the compatibility path if native preparation is unavailable or expires. There
is at most one owned observer; handoff does not retry the old map, and TERM,
failure, adopted handoff and successful join clean its private IPC. Warmup is
bounded by the existing lifecycle-plus-costmap budgets; final service/bridge
budgets are unchanged. No new navigation admission or motion policy is added.

This batch targets the final observation client's cold setup and reader wait.
It does not change the separate global-costmap check, planner configuration,
AMCL, Nav2 node composition, CPU placement or launch order. Source/binary
deployment and stationary tests do not establish end-to-end speedup; an
explicitly authorized full-service restart remains the timing acceptance step.

## First batch: overlap initialization, retain evidence

- Normal common-managed EKF startup starts the map lifecycle job without
  synchronously joining it before bridge/wrapper construction. The occupancy
  owner retains its child PID, joins that same result after launching its
  helpers, and cleans it on failure/cancel. Legacy/FAST-LIO ordering is unchanged.
- The existing early AMCL resident option defaults to true. It activates and
  prepares the resident without seeding and without claiming tracking ready.
  Completion begins only after accepted initial localization, overlapping Nav2
  activation through the existing readiness-before-lifecycle option.
- In normal background mode an unfinished resident child defers completion
  without blocking Nav2; the parent retains/reaps it before starting completion.
  Explicit strict foreground AMCL mode keeps its original blocking join.
- The existing Python lifecycle client pre-creates and caches GetState and
  ChangeState endpoints for its requested nodes in one ROS context. State is
  never cached; lifecycle transitions remain ordered. Discovery, creation and
  response timings are logged separately. Pre-creation moving outside a
  configure timer must not be misreported as total startup savings.

Existing TF ownership, map identity, seed semantics, readiness rules, five-core
allocation and failure/cancellation behavior remain. No new motion gate,
lease, persistent ROS observer or per-node recovery policy is introduced.

## Verification

### C++ GetState reply retention

The native `lifecycle-active` check previously issued a new GetState after
every 800 ms response timeout and no longer inspected the earlier future.
The private real-service reproduction in
`/tmp/njrh_reports/restart_40s_QLBMw4GG/lifecycle_pending/baseline_delayed_active_4yadu22g`
returned ACTIVE after 1.2 seconds for every request, yet the installed baseline
issued five requests and failed after 5.770 seconds with state unavailable.
No robot node, sensor, TF or state-change service was used by that fixture.

The check now keeps a single pending GetState across interruptible spin slices
until the original total deadline, then removes any unresolved request. A
completed non-active response still requires another actual query. Existing
active-state criteria and global-costmap publisher checks are unchanged. This
fix is independent of the Python ChangeState retention work above. Its isolated
regression is `src/robot_bringup/test/isolated_lifecycle_reply_smoke.py`.
Deployment, candidate results and subsequent full-restart timing are tracked
under the same `lifecycle_pending` report directory. No 40-second improvement
or five-run success is inferred from this targeted reproduction.

Behavior regressions exercise the real startup shell paths with isolated
process/ROS adapters: slow map, map failure, cancellation, early AMCL overlap,
false opt-out, strict/non-strict join and single child ownership. Lifecycle
tests cover delayed responses, missing responses, cancellation, endpoint reuse
and unchanged transition order. See `test_amcl_early_startup.py`,
`test_occupancy_lifecycle_overlap.py` and
`test_nav2_lifecycle_sequence_behavior.py` in `src/robot_system_tests/test`.

Deployment uses exact preimage/candidate hashes and preserves unrelated dirty
files. The read-only timing recorder reads existing files/journal and creates
no ROS participants. Each candidate is measured without interpreting missing
observation as authority for another restart. Physical navigation and docking
remain untested by stationary restart measurements.
