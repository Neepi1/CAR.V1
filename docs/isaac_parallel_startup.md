# Parallel initial-localization startup

## Scope and audit

2026-09-09 candidate, not yet cold-start accepted. This supersedes the normal
EKF/accelerated-profile scheduling described by the older S3/S4 documents.
The fixed Humble/Isaac version is reused; no algorithm, map assets, LiDAR,
QoS/DDS, TF owner, AMCL tuning, motion or arm-black-box changes are included.

The Isaac constructor creates its trigger service before asynchronous NITROS
initialization finishes. In the installed `nitros_node.cpp`, line 736 logs
`[NitrosNode] Node was started` AFTER successful `runGraphAsync()` and enabling
NitrosSubscriber input callbacks. `Initializing and running GXF graph` and
service discovery alone are not completion evidence. The earlier cold timeout
and one successful warm trigger support investigating this race, but do not
prove it was the sole cause of the observed missing result.

## Scheduling

Common resolves/cleans the selected-map startup before creating sensor owners,
then launches the resident navigation wrapper. Static TF, JT128 and Ranger/EKF
launch immediately afterwards without waiting for map or Isaac readiness.
The occupancy launch starts map_server and Isaac independently; map lifecycle
activation no longer waits for scan/odom. The normal common-managed path never
starts/restarts common-owned sensor/TF dependencies. Its cleanup targets exact
map_server/lifecycle node names, not Nav2's filter-map processes.

Navigation preloads as before. Common alone owns safety/floor/mode helpers,
including when navigation loads earlier than those helpers. API remains ahead
of docking. The old `NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE` flag
does not serialize the normal EKF/accelerated map/Isaac branch anymore;
diagnostic FAST-LIO/legacy common scheduling retains its original order.
No-map startup does not create a localization branch.

The normal launch registers ProcessStart/IO/Exit handlers, with only the Isaac
logger set to INFO. A private per-invocation JSON record includes target YAML,
PID and Linux process-start ticks. It becomes ready only from the exact Isaac
completion marker; process exit/respawn invalidates it. It is a startup receipt,
not a persistent lease, live floor-reload contract or proof of localization.
The marker dependency is version-specific and covered by tests. Missing marker
is explicit waiting, not fabricated readiness or a fixed startup sleep.

The existing compiled localization-stack observer and file-only Isaac check
share the configured 45-second observation budget (the existing process-exit
grace still applies). A bounded check that finds unfinished initialization
publishes a `starting` context with the missing dependency and retries observation,
without issuing an Isaac request or tearing down healthy owners. Cancellation
and dead-owner cleanup remain available. Docking remains deferred during this
initialization wait. Actual dispatched-result timeout still follows the existing
later-explicit-result policy, not automatic repeated Isaac dispatch.

After these three branches converge, existing local odom/TF and scan-owner
checks run before the one initial localization transaction. The existing C++
wrapper requires fresh valid FlatScan input and new post-arm samples before
dispatch, then verifies the original-stamp result and bridge acceptance.
Nav2 lifecycle activation still follows accepted localization. This change
does NOT claim fully parallel Nav2 lifecycle activation without map->odom.

## Validation and deployment

`test_isaac_parallel_startup.py` covers premature service-only readiness,
launch ordering, split output, wrong logger/PID/map, process exit/respawn,
PID recycling, delayed startup with one trigger, and temporary-file isolation.
`test_navigation_localization_startup.py` explicitly overrides every state path
used by its shell harness, including the Nav2 held-ready path and TMPDIR.
It must never delete a real `/tmp/njrh_nav2_launch_hold_ready.env` again.

Tests run locally and in a separate network-disabled, device-free container
with a private `/tmp` and read-only code, NOT inside the live `NJRH-car`.
The deployment backs up exact target files, verifies expected preimage hashes,
installs files atomically and verifies resulting hashes. No service restart
or localization/motion request is part of deployment.

Pending separately authorized hardware acceptance: complete-runtime restart;
record sensor availability, target map active, Isaac completion, trigger,
result, bridge acceptance, Nav2 active and docking-start timestamps. Confirm
single owners, original map/odom TF authority and zero automatic restarts.
Repeat three starts on the same physically correct target map before claiming
the cold-start failure is resolved or promising a startup-time improvement.
