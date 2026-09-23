# Positive BMS current scope

## Behavior and scope

Unverified positive current alone is not physical charging-dock contact during
ordinary navigation, teleop or the API predock phase. It remains raw telemetry,
but cannot create a new safety stop/contact latch or API predock cancellation.
`runtime.docking_active`, a return-to-dock job and a fresh `/cmd_vel_docking`
command are not evidence that the fine-docking controller has taken over.

The existing docking manager handles a newly received current sample while its
fine-docking states are active. It calls the existing `begin_contact_stop` in
that callback and publishes zero through `/cmd_vel_docking -> robot_safety`.
The start service does not reinterpret a current-only cached sample from before
fine docking. No new phase, delay, lock, service or permit topic is introduced.

Confirmed occupancy is intentionally different: retained dock evidence and
existing confirmed dock/safety-memory states still allow positive current to
maintain contact. Thus positive current during a controlled undock cannot be
mistaken for no contact merely because fine docking has ended. Existing release
requirements, failure/cancel behavior and freshness timeouts are unchanged.

`CHARGING`, `present + valid voltage`, existing FULL handling, thresholds, motion
parameters and docking-success criteria are not redesigned here. In particular,
the existing manager writes the strong latch on entry to `ContactStopping`;
wheel/mode stop confirmation precedes Park/success, not that first latch write.
This fix does not prove the physical meaning of current or independently
confirm electrical charging. It also does not clear historical latched state.

## Implementation boundary

- Safety: current alone requires already retained memory, docked status or
  strong persistent evidence; external predock commands do not qualify it.
- API Power: preserve raw current; classify it only with existing confirmed
  occupancy. The internal context query reads runtime, fresh safety memory and
  strong/manual latch evidence, without deriving context from BMS itself or
  invoking the occupancy evaluation/auto-clear path. It executes outside the
  power snapshot mutex. Predock job state alone is excluded.
- Docking manager: only newly delivered samples in existing fine-docking states
  can establish current-only contact. Existing Docked/latch/safety-memory paths
  remain valid for on-dock protection and explicit controlled undock.

No change to Nav2, MPPI, arm, localization, chassis, collision policies, API
protocols, contact thresholds or retry counts.

## Evidence and tests

The 2026-09-17 `nav_jerk_record_20260917T183757Z_0cwt5c8x` recording contains
safety events at epoch `1789670340.389924256` and `1789670396.129331008`:
current approximately `+0.7` and `+1.1`, status UNKNOWN, present=false,
external/cached docking context=false, docked/persistent evidence=false,
memory before/after=false; both are followed by `bms_contact_rising_edge`
zero output. These observations prove the classifier/stop path, not physical
charger contact or current polarity. Recording receive delays must not be used
as exact internal propagation latency.

Regression sources:

- `src/robot_safety/test/test_bms_contact_history_isolated.py`
- `src/robot_docking_manager/test/test_current_scope_isolated.py`
- `src/robot_api_server/test/features/power/test_power_module.cpp`
- `src/robot_api_server/test/features/docking/lifecycle/test_dock_contact_interlock_module.cpp`
- Existing `src/robot_system_tests/test/test_dock_memory_undock_isolated.py`

ROS tests require private network/IPC/PID/mount namespaces, loopback only,
private shared memory and domain 183, remapped outputs, executable copies and
no chassis. Do not run them in the production ROS domain. Candidate build,
red/green logs and manifests: `/tmp/njrh_reports/bms_current_scope_dm83qPLJ`.

## Hardware acceptance still required

Candidate results: Safety 25/25; current-scope manager/combined ROS tests 5/5;
Power 4/4; API occupancy module 19/19; retained safety policy 6/6; isolated API
100-round concurrent query regression passed. These are not hardware acceptance.

The wider existing undock suite was **17/18**, not a clean pass. The successful
manager+safety undock case reported success but retained safety memory. Repeated
paired tests produced candidate 3/6 and baseline 4/6 (the other cases test failed
undock retention). Thus the symptom also reproduces with the exact old binaries;
the root cause is not established. This change does not alter or silently relax
the release rules to pass that test. The user subsequently authorized deployment
with that outstanding result explicitly disclosed. Deployment is not a clean
undock regression pass. Full raw logs, including failed attempts, are retained
in the report directory.

After separately authorized deployment, verify normal navigation/predock with
the raw BMS current trace, real fine-contact stop distance and stop confirmation,
confirmed-dock retention, and a full controlled undock. No physical motion or
production activation is authorized by isolated test success.

## Authorized deployment: 2026-09-18

The already tested incremental binaries for API, Safety and Docking were
installed, followed by one explicitly authorized whole-runtime restart at
09:48:33 UTC. No compilation was repeated during deployment. The candidate had
rebuilt only six production translation units (API four, Safety one, Docking
one), reusing byte-verified baseline objects. No full API rebuild was performed.

Deployment evidence and old-file backups:
`/tmp/njrh_reports/bms_current_deploy_LW2EJiIa`.
Production sources/tests match the tested candidate. Existing unrelated remote
API/Safety README differences were preserved by applying only this update's
paragraphs. The persistent dock record was preserved; no motion request,
undock request or record clearing was performed. Physical navigation, contact
stop and controlled-undock acceptance remain unverified.

A robot manually placed on a charger without existing confirmed occupancy or
another accepted contact signal will not be blocked by positive current alone.
Queued samples delivered after fine-docking start retain existing ROS delivery
semantics; this patch rejects the cached pre-start object, not every possible
upstream stale message or unverified hardware timestamp.
