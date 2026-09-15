# Mapping on CPU0–4

The user extended the active `navigation_5cpu` allocation to mapping on
2026-09-14. No new profile selector, readiness gate or estimator change is added.

| Work | Allowed CPUs |
|---|---|
| FAST-LIO and its launcher | 1-4 |
| Mapping owner, SLAM Toolbox, PGO | 0,1,4 |
| FAST-LIO odometry bridge | 2 |
| Existing cloud preprocessing, scan conversion, optional FAST-LIO remap | 1,4 |
| Existing JT128 driver | 3 |
| Mapping-session eth1 RPS/XPS | 4 |

The subsequent user-requested FAST-LIO placement excludes CPU0, without moving
the other mapping roles. CPU1-4 is scheduling eligibility, not exclusive use:
FAST-LIO may share CPU2 with IMU/EKF and CPU3 with JT128. Affinity alone does not
enable OpenMP. A subsequent separately authorized four-thread build is described
in [FAST-LIO OpenMP](fastlio_openmp_threads.md). For the affinity adjustment,
live rebinding targets only the identified FAST-LIO launcher/executable and all
their threads; the mapping owner and RPS/XPS are left unchanged.

The explicit mapping-key list is resolved after site/inherited values alongside
the existing navigation profile. It remains separate from the navigation-key
list so mapping does not acquire the navigation startup CPU0–7 boost. Selecting
`site_default` restores pre-profile mapping values too; arm/custom keys and
existing navigation steady masks are untouched. The projected-map entry no
longer overwrites the selected five-core masks with old CPU7/3,7 mode defaults.
Its owner is placed before launching helpers. Existing RPS backup/restoration
behavior is retained; IRQ routing and navigation-session RPS are not changed.

The affinity adjustment changes no FAST-LIO math, point density, map resolution, timestamp,
DDS/QoS, TF frame, lifecycle or safety setting changes. More eligible CPUs do not
automatically make its core matching loop parallel or reduce total CPU work.

Tests execute the real Bash resolver and mapping-entry prefix with OS/ROS
boundaries mocked: stale inherited masks, explicit old mode overrides,
re-sourcing and child shells, baseline restoration, unchanged arm/navigation,
disabled affinity and no mapping startup boost. Run:

```bash
python3 -m pytest src/robot_system_tests/test/test_navigation_cpu_profile.py src/robot_system_tests/test/test_startup_cpu_affinity_wiring.py -q
```

Deployment alone affects future launches. A currently running mapping session
is not restarted by these files. If explicitly authorized, identify its exact
owner/descendants and process start times, change every existing thread's mask,
retain the same PIDs, and verify all resulting masks; do not select arbitrary
ROS or mechanical-arm processes. Move the existing session's RPS/XPS mask to
CPU4 without changing its original restoration backup. This is not a change to
the navigation-profile selector during live navigation.

Hardware verification: bounded procfs CPU/runqueue sampling, unchanged process
identities, continuing existing map/scan/odometry status, and subsequent
user-operated mapping quality and next-session startup/cleanup. A scheduling
snapshot alone does not certify moving-map accuracy or total performance gains.
Reports and before-state backups belong under `/tmp/njrh_reports`.
