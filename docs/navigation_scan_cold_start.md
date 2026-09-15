# Cold-start scan ownership observation

## Scope (2026-09-12)

The normal `systemd_autostart` path now proves `/scan` ownership with one
continuous exact-owner client, using the existing `SCAN_OWNERSHIP_TIMEOUT_SEC`
(12 seconds by default). It does not call the mapping-recovery sequence to
re-enable a publisher merely because a new DDS participant has not discovered
it yet. This changes orchestration only, not JT128, scan data/QoS, TF, CPU
allocation, AMCL, localization acceptance, navigation or safety parameters.

On the recorded restart, scan messages were observed before a later short
owner check returned zero; the fallback then created separate count, service
discovery, service-call and final-owner clients. That branch consumed 18
seconds. A fresh discovery cache's zero count cannot prove graph absence.
The normal common-owned scan worker is enabled and creates its publisher on
construction, so cold startup needs observation, not a restore command.

The exact single-publisher condition remains unchanged. Missing, duplicate or
foreign publishers cannot report startup ready. This is not a claim that DDS
discovery can never time out, or that all 18 seconds will be saved.

API navigation resume and projected-mapping teardown retain the existing
ownership handoff/restore behavior. Generic zero-count observation outside
this cold-start branch is not redesigned in this phase. No AMCL startup
ordering or readiness checks were changed.

## Verification

`src/robot_system_tests/test/test_navigation_scan_startup.py` executes the real
systemd branch and scan helpers with only the ROS probe boundary replaced.
It reproduces the old short-discovery/zero-count/control-service branch and
checks late discovery, true absence, duplicate publishers and a foreign owner.
The broader startup harness sources the real scan helper and mocks its ROS
boundary; no robot, ROS context or service is used by these tests.

After a separately authorized complete service restart, compare the existing
`common_local_state_ready`, `navigation_scan_owner_ready`, `nav2_layer_ready`
and AMCL-ready logs. The cold scan branch should contain one exact-owner proof
and no scan-output enable request. Use restart-request time as the full-chain
zero point; script-local elapsed values are a separate metric. Hardware timing
and mapping/resume transition acceptance remain pending until measured.
