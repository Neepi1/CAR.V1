# Resumable AMCL startup preparation

## Composite observation and seed-budget correction (2026-09-12)

The full restart in `/tmp/njrh_reports/scan_cold_restart_PJMau06e` confirmed
that successful preparation checkpoints were reused, but separate MAP, SCAN,
scan-frame echo, and three TF clients still incurred independent discovery.
AMCL activated at 00:20:25 UTC and received its first initial pose at 00:22:45.
One scan attempt had only 3.090 seconds available and a seed client was launched
with 0.830 seconds. These client/attempt timings are not proof of CPU saturation
or a DDS deadlock.

`runtime_readiness_probe amcl-inputs` now uses one bounded ROS context and one
TF buffer to observe pending map/scan/TF conditions concurrently. It reports
each successful condition and the actual received scan frame independently;
the existing same-process/map/owner checkpoint retains partial results even
when another condition times out. The scan header replaces the separate
`ros2 topic echo`. Existing successful steps, lifecycle checks, map identity,
TF ownership, scan processing and static-standby semantics are unchanged.
An empty received scan frame uses the configured fallback; absence of a scan
does not invent a received frame. No persistent extra observer is introduced.

Before requesting an initial pose, the runner requires the full configured
bounded client time to fit inside the existing attempt deadline. Insufficient
remaining time returns pending without creating a client. Completion propagates
the existing pending code 26 back to the resident supervisor immediately instead
of repeatedly entering the same insufficient budget. The 45-second attempt
budget and seed service wait/call limits are not enlarged, and no resident node
is restarted. A fresh completion attempt still requests a seed; seed success is
not cached across requests. AMCL_READY now includes an absolute UTC timestamp.
The opt-in `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY=true` foreground mode keeps
its existing bounded in-attempt retries rather than requiring a background
supervisor to reschedule pending work.

This optimization does not change five-core affinity, Nav2 lifecycle ordering,
Isaac/AMCL algorithms, readiness acceptance, navigation or docking control.
Deployment and full-restart hardware acceptance are recorded separately in
`/tmp/njrh_reports/amcl_inputs_DvnV1HG9`; startup speed is not accepted until a
later explicitly authorized complete-runtime restart has been measured.

Targeted shell regression:

```bash
python3 -m pytest src/robot_system_tests/test/test_amcl_startup_sequence.py \
  src/robot_system_tests/test/test_amcl_composite_startup.py -q
```

The actual C++ command is covered by
`src/robot_bringup/test/isolated_amcl_inputs_smoke.py`, using synthetic map/scan/TF
in a separate network-none, private-IPC, no-device container and ROS domain 224.
Never run the fixture in the production domain. Cases include delayed input,
partial timeout and resume, missing/invalid input, actual scan-frame handling,
and bounded cleanup. Shell tests also preserve process/map/owner invalidation,
seed retry, resident survival and pending-code propagation.

## Scope and evidence (2026-09-11)

This repair changes startup orchestration only, not AMCL, Isaac, Fast DDS,
localization acceptance, TF ownership, scan processing, or navigation control.
The preserved restart log recorded AMCL active at 18:24:06.634 UTC and its first
initial pose at 18:27:37.618 UTC: a 210.984-second gap. The resident log also
showed repeated lifecycle queries timing out after activation. This is evidence
of redundant preparation and ineffective outer timeout handling, not proof of
a new DDS deadlock. Source/report baseline is retained under
`/tmp/njrh_reports/amcl_startup_fix_dLbKYObU` and the earlier
`dds2612_restart_capture_yuYq6ltU` report.

## Implementation

The normal readiness background worker now invokes one complete sequence:
start/reuse AMCL, confirm lifecycle and `tf_broadcast=false`, prepare map/scan/TF,
start/reuse scan admission, seed AMCL, then apply the existing readiness rules.
The optional early-resident A/B branch remains available; completed preparation
is shared with its later completion phase.

`amcl_startup_progress.sh` records only successful preparation steps. Its cache
identity includes the validated AMCL PID/start time, live startup-owner PID/start
time, boot identity, optional handoff nonce, selected map path/content/assets,
AMCL configuration, mode and building/floor/map identifiers. A changed or unknown
identity never reuses old preparation. Cache failure does not block startup.
The public status heartbeat is not used as proof of lifecycle activation.

Seed is deliberately not reused: each completion request still requests its
initial pose, including after a same-map relocalization. A failed seed retries
without repeating already successful map/scan/TF preparation. Existing readiness
and static-standby semantics are unchanged.

The existing `NJRH_AMCL_READINESS_COMPLETION_TIMEOUT_SEC` (default 45 seconds)
now supplies a monotonic deadline to individual clients and sleeps. Temporary
clients have a 0.2-second TERM-to-KILL grace; the AMCL/relay resident launch is
never enclosed in that timeout process group. Insufficient time for an entire
warmup sleep does not count as completed warmup. A pending attempt returns 26,
retaining resident processes and completed preparation for the existing retry
supervisor. This is an attempt budget, not a guarantee that initialization
finishes in 45 seconds. Missing binaries/configuration and invalid TF ownership
are not converted into successful readiness.

Optional graph-status queries also use the remaining attempt budget. A resident
heartbeat clears its inherited startup deadline and keeps its normal lifecycle.
No new navigation permission, motion, or map-readiness gate was introduced.

## Verification and hardware acceptance

Run the isolated, mocked behavior tests (no ROS graph or robot commands):

```bash
python3 -m pytest src/robot_system_tests/test/test_amcl_startup_sequence.py \
  src/robot_system_tests/test/test_nav2_lifecycle_sequence_behavior.py \
  src/robot_system_tests/test/test_navigation_localization_startup.py \
  src/robot_system_tests/test/test_floor_startup_handoff.py -q
```

The new sequence cases cover one activation/preparation, cross-shell checkpoint
reuse, process/map/owner invalidation, unknown identity, seed retry without
seed caching, short-client cleanup and resident survival at budget expiry.
On the Windows test host the combined suite passed 47 tests with one
environment-dependent skip. Three unrelated AMCL workspace-contract assertions
fail identically against both the preserved pre-fix scripts and current scripts;
they are not silently removed by this repair.

Deployment does not authorize a service restart or robot movement. After an
operator-authorized full `njrh-runtime.service` restart, record node creation,
lifecycle activation, each `AMCL_STEP_BEGIN/END`, seed response, and AMCL_READY
timestamps. Confirm same-process preparation executes once, all canonical TF
owners remain unchanged, and map/owner changes do not reuse old preparation.
Real startup time improvement remains unaccepted until that run is recorded.
