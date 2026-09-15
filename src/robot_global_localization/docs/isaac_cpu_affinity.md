# Isaac floor-switch CPU placement

The 2026-09-09 field diagnostic found delayed component-manager RPC dispatch
on the busy CPU6 pool. An isolated, non-moving reproduction using the deployed
C++ port and real Isaac component completed unload plus absence proof in
1.265 seconds on CPU3 while scan input remained active. This is evidence for a
scoped scheduling change, not a guarantee of every production floor transition.

Only `NJRH_CPUSET_OCCUPANCY_GRID_LOCALIZER` and
`NJRH_CPUSET_ROBOT_GLOBAL_LOCALIZATION` default to `3`. The shared localization
pool, AMCL, IMU, scan, lidar, DDS configuration and 15-second component timeout
are unchanged. CPU3 is shared with Nav2, not exclusively reserved for Isaac.
The field Nav2 `control_wide` profile remains `3,5`.

Existing API parents can retain concrete CPU6 environment values. A field
`cpu_affinity_runtime_override.env` must therefore explicitly export these two
keys as `3`, preserving its other entries. Changing only a shell `:-3` default
does not override inherited `6`. Do not deploy unrelated dirty runtime scripts
or rebuild binaries for this configuration-only change.

## Acceptance and rollback

1. Save and hash the field override before an atomic replacement; abort if it
   changed since inspection.
2. Verify the resolver with inherited CPU6 values and verify every thread of
   both running processes, not merely each process leader.
3. Use normal navigation stop/start to restore any already-failed empty Isaac
   container. Affinity changes alone do not reload a missing component.
4. Verify the same 15-second parameter, exact map identity, active component,
   fresh localization, and Nav2 readiness. Perform stationary floor switches;
   keep the robot stopped and never bypass safety or identity evidence.
5. Real-hardware acceptance still includes repeated floor changes and ordinary
   navigation under load, since CPU3 shares the Nav2 pool. Do not report motion
   regression coverage from a stationary test.

Rollback restores the saved field override and restarts only navigation through
the normal interface. In the repository, revert only these two default changes.
Do not reset unrelated workspace edits or clear retained transaction state.

The Bash regression is `robot_system_tests/test/test_isaac_cpu_affinity_scope.py`.
