# Five-core navigation placement experiment

## Scope and status

`NJRH_NAVIGATION_CPU_PROFILE=navigation_5cpu` selects grouped placement within CPU0-4.
`site_default` (the default) preserves the resolved field allocation. This is
an opt-in capacity experiment, not a claim that five cores are sufficient for
startup, localization and moving MPPI peaks. No algorithms, rates, DDS, TF
ownership, motion/safety policies, startup ordering, GPU settings or CPU quota
are changed. No ROS/C++ binary rebuild is required.

The arm owner must separately place its processes/threads on CPU5-7. This code
does not operate the arm or edit its black box. Navigation-only affinity is not
exclusive CPU isolation; kernel/IRQ work and shared memory/GPU remain outside
this placement policy. The container's global CPU set remains unchanged.

The implementation and tests may be synchronized while `site_default` remains
selected. Synchronization alone does not activate or validate five-core motion.

## Resolution and inheritance

The single entry point remains `scripts/jetson/runtime_overlay/scripts/cpu_affinity.sh`:

1. Restore saved pre-profile values inherited from a prior five-core resolver.
2. Load the existing default configuration and field runtime override.
3. Resolve the existing Nav2 controller profile (`current` / `control_wide`).
4. Apply the selected navigation profile to an explicit navigation-key list.

The five-core profile also sets both controller-profile CPU sets, so subsequent
controller resolution cannot put it back on CPU5. Pre-profile values are exported
to child shells; switching back to `site_default` restores original values before
reloading the latest field override. No wildcard rewrite of `NJRH_CPUSET_*` is used.
Since 2026-09-14, an explicit separate mapping-key list also follows the five-core
limit; see [mapping allocation](mapping_five_cpu_profile.md). Arm-specific and
unknown site keys are not changed. An unknown profile
is a configuration error, not a robot admission rule.

On a five-core common-runtime startup only, the existing affinity helper places
the startup shell on CPU0,1,4. Unprefixed helpers inherit this initial mask. Named
navigation launches get their explicit group values below. The default startup does
not add a new taskset call. CPU affinity is not a hard containment boundary against
a future child explicitly setting a different mask; verify actual threads.

### Controller-only placement (2026-09-20)

At the user's request, `navigation_5cpu` moves only `controller_server` and its
in-process local costmap from CPU0,1,4 to CPU1-3. Both controller profile aliases
resolve to the same `1-3` mask, including re-sourcing from an old API environment.
The generic NAV_CONTROL group, planner, BT, API, sensors and downstream command
chain retain their preceding masks. `site_default` is unchanged.

This removes controller eligibility on CPU0/4, but allows competition with the
existing IMU/EKF group on CPU2 and JT128 driver on CPU3. It is not exclusive core
allocation or a proven solution to control deadline misses. No motion, MPPI,
sensor, safety, priority, startup-boost or IRQ/RPS parameter changes are included.

Activation for this specific single-process change uses the existing per-thread
affinity mechanism after verifying that cold-start restoration has completed;
it does not restart a node, create a startup session or alter its old records.
Verify all controller threads and unchanged peer masks. Future launches resolve
the new steady mask from the same profile. Whole-profile selection changes still
follow the complete-runtime restart procedure below.

Resolver/guard regressions and thread verification do not validate moving MPPI,
IMU/EKF continuity or lidar headroom under load. Those remain hardware checks;
no navigation goals are sent by this configuration change.

### Historical grouped candidate T (2026-09-11; not accepted)

T's 125-second capture measured scan 15.006 Hz, source cloud median 19.999 Hz,
corrected IMU 98.384 Hz and odometry 49.936 Hz. Two real FlatScan CLI processes
were verified on CPU0,1,4. However, map-to-odom and bridge status were absent;
planner lifecycle activation timed out and the existing owner cleaned up Nav2
and localization. AMCL was not running. This is not a matched full-load comparison
with S and does not prove that the CLI placement caused the rate improvement.
At 11:50:48 UTC the runtime was unhealthy with navigation unavailable. Keep the
five-core selection, but do not call the whole-chain goal complete or change DDS,
timeouts or processing to conceal this failure. The bridge's five-second sample
showed almost no CPU runqueue wait, so further blind core permutations are not
justified by the current bridge evidence.

| Work | Eligible CPUs |
| --- | --- |
| Official JT128 driver | 3 |
| Legacy and non-legacy sensor startup wrappers | 3 |
| Pointcloud/scan/FlatScan processing workers | 1,4 |
| IMU remap/filter, wheel preprocessing, EKF, localization bridge | 2 |
| Chassis, safety, velocity smoother, collision monitor, docking manager | 1 |
| Nav2 controller/local costmap (updated 2026-09-20) | 1-3 |
| Nav2 planner/BT, AMCL, Isaac, docking vision | 0,1,4 |
| API, health guard, camera key, map servers, lifecycle supervision | 0,1,4 |
| Common/resident runtime owner and unprefixed children | 0,1,4 |
| Existing FlatScan graph/rate CLI checks | 0,1,4 |

These are eligibility masks, not primary/overflow priorities or exclusive CPU
isolation. Process-level placement does not independently pin in-process workers.
Concrete launches and every actual thread must be verified: a declared camera or
worker key alone is not evidence that a launcher consumes it.

L introduced the user's distinction: the official driver can consume roughly
one core by itself; its downstream pointcloud processing is a separate workload.
Unlike F, L removes Nav2/localization/helpers from CPU3 as well as removing the
driver's access to CPU3. The pointcloud startup wrapper also uses CPU3, not the
driver's CPU4. Remaining compute may share CPU2 with state estimation and CPU1
with control, so those rates and gaps must be validated together, not inferred
from improved lidar rates. This is application-process separation only: no IRQ,
RPS, scheduling priority, frequency, timeout, ordering or algorithm changes.
Tests resolve the actual launch masks, reject cross-group overlap and exercise
the distinct startup prefixes. Hardware capacity remains unaccepted.

L's stationary 65-second capture measured scan 15.003 Hz, corrected IMU 99.705 Hz,
local odometry 49.984 Hz and odom-to-base TF 49.984 Hz. Source cloud status reported
a 20 Hz median and zero greater-than-100-ms publication gaps. However, bridge
force-accept failed before Isaac dispatch, map-to-odom was absent, and AMCL did
not start; these figures are not full-load or whole-chain acceptance.

M changes only the seven system-group eligibility values from CPU0 to CPU0-2.
Repeated CPU0 saturation and API-thread scheduling wait motivate removing that
single-core constraint, without moving IRQ/RPS or allowing downstream/driver
overlap. CPU1/2 control and state-estimation timing must be checked for regressions.
This is a capacity hypothesis, not a proven fix for the bridge's disappearance:
the stalled bridge itself showed little runqueue wait in the sampled window.
No service deadlines, discovery settings, priorities or retry rules are changed.

M failed its first startup with odometry loss and an existing whole-chain
automatic restart. The second attempt reached ready, but the following capture
measured IMU 46.49 Hz, odometry 16.73 Hz and map-to-odom loss in the final windows,
while scan stayed near 15 Hz. After the observer exited, a five-second CPU sample
still showed CPU1/2 only 5.8/5.5 percent idle versus CPU3 63.4 percent idle.

N moves the compute/system/owner pool from 0-2 to canonical mask 0-1,3, leaving
the state group on CPU2 and the official driver on CPU4. Pointcloud remains on
CPU3, but that core is no longer reserved exclusively for downstream processing.
Its measured spare capacity can be used by compute; cloud/scan rates must be
rechecked to reject contention regressions. Tests now enforce driver and state
separation from this pool, not the earlier unjustified exclusive downstream core.
This remains five-core process allocation only, not a DDS or startup-policy fix.

N reached ready on its first start and restored map TF to 49.64 Hz, but scan
averaged only 12.14 Hz. Source cloud status had a 16.86 Hz median and 200
greater-than-100-ms publication gaps across the warmed status windows. After
the observer exited, a pointcloud thread used 0.075 seconds of CPU and waited
0.802 seconds runnable in a five-second window. These are not full acceptance.

O changes only the downstream group from CPU3 to CPU1,3, including its existing
startup wrapper. Driver CPU4, state CPU2 and compute/system CPU0,1,3 remain
unchanged. This can increase competition with the CPU1 chassis/safety chain;
cloud/scan gains must not come at the expense of odometry, TF or final-command
cadence. No thread priority, rate, transport or processing changes are involved.

O kept odometry/TF near 50 Hz and completed AMCL seeding, but scan remained
12.04 Hz and source cloud status had a median 18.08 Hz. A five-second driver
capture measured a normal thread running 0.665 seconds and queued 3.201 seconds;
six existing FIFO/70 driver threads shared CPU4. This does not establish an
intrinsic two-core requirement for the official driver in every workload.

P changes only LIDAR_DRIVER/HESAI_ROS_DRIVER from CPU4 to CPU3,4. No other
navigation group is allowed on CPU4; the driver may also compete on shared CPU3.
The startup wrapper remains CPU4, downstream stays CPU1,3 and state stays CPU2.
Eligibility is not a primary/overflow priority. The shared-core cost must be
checked alongside cloud/scan gains and driver runqueue wait, without changing
the driver's worker count, scheduling policy, QoS or timestamps.

P was rejected after repeated startup failures on 2026-09-11. In the 10:58 UTC
attempt, the IMU filter exited at 10:58:54 with zero input, while driver parsing
and canonical IMU remap became ready around 10:58:58. This is an observed upstream
startup/readiness race, not proof that an already-running IMU filter rejected
valid data. It does not establish that the mask change alone caused the delay.
Only P's extra driver CPU3 eligibility is removed; the five-core O allocation
is restored. O previously started and retained odometry/TF but scan was only
12.04 Hz, so restoration is not completion of the frequency-stability objective.
No startup waits/order, RPS/IRQ, DDS or worker scheduling policy are changed.

Restoring O also reproduced IMU readiness failure. At 11:03:11 UTC a five-second
startup sample had CPU0/1/3 idle 3.7/6.5/7.3 percent, while CPU4 was 91.7 percent
idle. Source inspection showed that non-legacy pointcloud bootstrap calls
run_driver.sh while still inheriting CPU1,3; the LIDAR_STARTUP=4 setting only
covered the legacy common entry. Thus the table's driver-wrapper declaration
did not describe the active upstream bootstrap path.

Q changes only that existing startup prefix to LIDAR_STARTUP for both sensor
entries. run_driver.sh still explicitly launches the official driver on CPU4,
pointcloud workers on CPU1,3 and IMU remap on CPU2. The whole lidar chain is NOT
placed on CPU4. CPU4 bootstrap contention after driver start is a remaining risk
to verify. No extra sequencing gate, wait, process, thread or priority is added.

Q reached Nav2 ready on its first start (stage elapsed 101 seconds) and remained
ready with AMCL seeded. However, its 127.894-second capture measured scan 12.606 Hz;
source cloud median was 18.517 Hz with 164 greater-than-100-ms publication gaps.
A later five-second driver sample had a normal thread running 0.671 seconds and
waiting runnable 3.274 seconds. Thus initialization improved but sustained rates
were not accepted.

R keeps Q's CPU4 bootstrap and changes only the official driver eligibility from
CPU4 to CPU3,4. Downstream remains CPU1,3, state CPU2, and compute/system CPU0,1,3.
This separates the runtime-headroom experiment from P's congested bootstrap path;
it is not evidence that the official driver inherently requires two cores.
CPU3 competition must be checked alongside source cloud, scan, odometry and TF.
No processing, priorities, rates, DDS, RPS/IRQ or startup waits are modified.

R failed: scan averaged 11.390 Hz and map-to-odom was absent throughout the
125-second capture, although odometry and odom-to-base continued at 49.8 Hz.
The bridge logged odometry timeout and showed minimal runqueue wait; therefore
not every failure is established as CPU scheduling starvation.

At 11:27:14 UTC, CPU0-3 frequency was 1.984 GHz and CPU4-7 was 1.574 GHz; both
policies allowed 1.984 GHz. This was a read-only instantaneous snapshot, not a
fixed hardware-speed claim. CPU1/3 were about 6 percent idle, CPU4 24 percent idle.
S returns to the disjoint Q grouping but swaps its physical CPU3 and CPU4 roles:
driver/bootstrap CPU3, pointcloud CPU1,4, compute/system CPU0,1,4. State CPU2 and
control CPU1 stay unchanged. This tests driver placement on the currently faster
cluster without changing frequency, governor, power, IRQ/RPS or processing.
All major rates and bridge continuity still require verification; S is not an
assumed fix for the bridge transport/endpoint symptom.

S's capture still measured scan 11.263 Hz and no map-to-odom, while odometry and
odom-to-base continued near 49.63 Hz. It is not accepted. A live process snapshot
at 11:37:59 UTC identified the existing `ros2 topic info -v /flatscan` child
PID1212469 on CPU3, inherited from the sensor supervisor. The earlier short
children-file probe was inconclusive because that kernel interface is absent;
an empty result is not evidence of no checks.

T keeps S's worker placement and explicitly runs only existing FlatScan graph
and rate-check commands in the NAV_SUPERVISION pool. Their deadlines, periods,
ROS arguments, results and failure policy are unchanged. Site-default operation
and disabled affinity retain the original unbound invocation. This reduces a
verified source of driver-core competition; it is not a claimed cure for bridge
endpoint loss. Tests execute both real check functions with mocked OS/ROS calls,
including the output parser and unchanged non-five-core/disabled paths.

Candidate F tested only JT128 eligibility from 3,4 to 4. It was rejected: the
125-second capture measured 94.73 Hz corrected IMU and 14.36 Hz scan, received
no local odometry, and the existing health guard confirmed the missing endpoint
before automatically restarting the chain at 01:02 UTC. A five-second driver
thread capture found a normal thread waiting 2.82 seconds for scheduling while
six FIFO/70 driver threads shared CPU4. This does not alone prove the internal
cause of endpoint loss, but the candidate failed hardware regression. G restores
the driver's 3,4 eligibility only; all work remains within CPU0-4.

That F failure is not proof that the official driver intrinsically requires two
cores. Historical July source used CPU4; the latest healthy September 10 field
snapshot already allowed CPU4,6, with pointcloud on 5, AMCL on 6 and bridge on 7.
Compressing these other workloads changes the comparison conditions.

The preceding D baseline measured approximately 50 Hz local odometry/canonical
TF, 14.93 Hz scan and 97.88 Hz corrected IMU. Stationary observations do not
validate moving MPPI or peak-load capacity. Compare source status windows and
gaps as well as reception; pointcloud gap counts reset each status window.

The G repeat activated Nav2 but failed its final fresh-map-TF check, then
terminated its incomplete runtime. AMCL exit 137 was downstream of cleanup,
not evidence of OOM. H changes only the runtime owner/unprefixed-helper mask
from 0,1,3 to 0-4 to test receiver/startup scheduling headroom. Explicit sensor,
state, control and Nav2 process groups are unchanged; readiness rules and
timeouts are unchanged. Helpers may briefly compete on CPU2/4, so this is not
exclusive core isolation and requires another source/receiver timing capture.

H failed the existing IMU-output startup check before the real driver input
arrived, with complete-runtime automatic retries. Candidate I keeps the extra
CPU2 headroom for helpers but excludes CPU4 (owner mask 0-3), so initialization
on CPU4 no longer shares its eligibility with those helpers. No startup order,
deadline, driver thread count, scheduling priority or node algorithm is changed.

I also failed sustained readiness: the first startup lost odometry before the
bridge could accept the pose; a subsequent automatic attempt again failed its
final fresh-TF check. At 01:22:52 UTC an EKF thread spent 4.467 of a 5.009 second
window runnable but waiting for CPU2, with only 0.418 seconds executing. Its
policy was normal SCHED_OTHER/nice 0; no container CPU quota was configured.
J moves only IMU remap and wheel-odom preprocessing from CPU2 to CPU1, retaining
EKF/filter/bridge on CPU2. CPU1 control-chain timing must be checked alongside
the state chain; this does not alter preprocessing semantics or frame ownership.

J failed: a 125-second window measured 46.56 Hz corrected IMU, 12.78 Hz scan
and 47.38 Hz wheel preprocessing, without local odometry. After two automatic
retries, a further 65-second check still measured only 57.71 Hz IMU, 12.99 Hz scan
and 47.61 Hz odometry, without map-to-odom. Brief API readiness was not acceptance.
K removes these unsuccessful experiments: both preprocessors return to CPU2 and
the runtime owner returns to CPU0,1,3, exactly the preceding D CPU sets. This
remains five-core operation, not restoration of the eight-core site allocation.
Repeated startup and sustained frequencies are still unproven (G already showed
that this placement can also fail).

A read-only eth1 snapshot found all 44,718 IRQ257 events in five seconds on CPU0,
despite an allowed IRQ mask of 0-7; RPS/XPS masks were both 00. A separate load
window found CPU0 fully busy, with approximately 40% hard/soft IRQ time. This is
a scheduling lead, not proof of the sole cause. No IRQ/RPS/XPS changes have been
made: the existing mapping-only RPS convention requires explicit authorization
before applying software receive steering to navigation.

Configuration strings use kernel-canonical range notation (currently `0-1,4`).
The existing Nav2 startup guard compares strings with `/proc` output; writing the
equivalent `0,1,3` caused a false affinity failure in candidate D. The configuration
uses matching notation without changing the guard or its timeout. Tests
execute the real guard against a mocked kernel inventory for both controller profiles.

The sensor wrapper mask is applied by the existing common process launcher before
environment initialization. The child PID/ownership and all existing waits/order
remain unchanged; site-default and non-sensor launches retain their old behavior.

## Selection and recovery

Use the existing field `config/cpu_affinity_runtime_override.env`, preserving its
other entries. Select the candidate with:

```bash
export NJRH_NAVIGATION_CPU_PROFILE="navigation_5cpu"
```

Select the original resolved site allocation with:

```bash
export NJRH_NAVIGATION_CPU_PROFILE="site_default"
```

Change the field selector only as part of an authorized complete-runtime restart.
Do not live-rebind individual nodes or change the override while new navigation
children may be spawned: that creates mixed placement. The permitted restart is:

```bash
printf 'nvidia\n' | sudo -S systemctl restart njrh-runtime.service
```

Save the actual field override and hashes under `/tmp/njrh_reports` first. Recovery
changes only this experiment's selection/restores its saved files; do not reset
the repository or replace unrelated runtime edits. The local and Jetson default
affinity files have a pre-existing Isaac-default difference; field baseline means
the actual remote defaults plus its override, not a copy of local defaults.

Navigation and mapping share some sensor processes. The original experiment left
dedicated mapping keys unchanged; the user subsequently requested five-core mapping
as well. Use the mapping allocation document for current behavior; do not restore
the site allocation merely to run mapping. No automatic mode changes or
mapping/navigation gates are introduced.

## Verification

The resolver/startup-prefix tests use real Bash configuration and fake OS boundaries:

```bash
python3 -m pytest src/robot_system_tests/test/test_navigation_cpu_profile.py src/robot_system_tests/test/test_isaac_cpu_affinity_scope.py -q
```

They cover the navigation key list, old inherited values, both controller profiles,
re-sourcing, API-like child environments, current field override precedence,
restoration, non-navigation keys, configuration errors and opt-in parent placement.
They do not start a robot node, contact ROS, rebind a live PID, or move the robot.

Before deployment: compare source hashes, run Bash syntax checks and resolver tests.
After authorized activation: verify every navigation thread's `Cpus_allowed_list`
is within CPU0-4; verify unique critical processes and existing passive health data.
Check again after map switches, because they may create new processes. The old
`inspect_runtime_cpu_affinity.sh` contains legacy AMCL forbidden-core assumptions;
do not use its old PASS/FAIL as five-core acceptance.

Compare matched site-default and five-core runs with the arm owner providing similar
representative load. Record startup/Nav2 readiness, map switch/localization times,
control deadline misses, TF expiry, sensor backlog, and operator-run navigation,
dynamic avoidance, elevator and docking results. Use current system deadlines,
not relaxed timeouts or tolerances. Mark untested scenarios as unverified. A new,
reproducible regression requires investigation within the user-selected five cores.
Do not restore the site allocation without explicit user authorization, reduce
sensor fidelity or weaken safety checks. These are acceptance checks, not new
runtime gates. Reports belong under `/tmp/njrh_reports`.

## 2026-09-10 field trial: not accepted

An authorized five-core activation at 17:31:33 UTC did not reach Nav2 readiness.
The common startup failed its local-state readiness check; automatic retries also
logged IMU-bias publishers present but no messages received before the deadline.
The original field override was restored and the complete runtime restarted at
17:34:50 UTC. Nav2 reached ready at approximately 17:36:25 UTC (95 seconds from
restart request; the resident startup-stage log reports 65 seconds).

That historical snapshot was on `site_default`, with unique critical processes and a healthy
API. It is not the current selection: five-core operation was subsequently re-enabled
at the user's request and must not be automatically reverted. No arm, sensor-QoS,
timeout or motion-policy changes were made. The same
RELIABILITY_QOS_POLICY warning appears in a successful restored startup: the
readiness probe creates both best-effort and reliable subscriptions, so that
warning alone does not establish the failure cause. Five-core capacity/latency
is NOT accepted, and the causal scheduling/input-readiness issue remains open.
Evidence is under `/tmp/njrh_reports/navigation_five_cpu_activation_20260910T172936Z`.
