# Source-independent manual map switching

## Business rule

Loading a target map must not require successful localization on the previous
map. Selecting the wrong initial map can itself explain failed localization.
The existing target asset load, explicit target relocalization and target
completion checks remain unchanged. This fix adds no navigation start, stop,
cancel, runtime restart, pose rewrite or sensor subscription.

## Removed coupling

- API source `ready`, `confirmed`, epoch/digest and explicit sequence no longer
  decide whether a target may be requested. They only prove the existing exact
  already-active no-op. An unlocalized same-map request reloads rather than
  falsely reporting success. A validated new asset version is not rejected
  merely because the old version has the same logical map ID.
- The optional negative interlock ignores only `FLOOR_RUNTIME_CONTEXT_INVALID`
  for manual map-switch admission. It does not mutate retained health, hide an
  active transaction behind invalid health, or change another operation's policy.
- Bridge BEGIN accepts an absent/unseeded source. The existing FloorManager
  already sends empty source metadata when it has no healthy source snapshot.
  This is now supported directly; no fabricated ready file or source seed is
  needed. Supplied metadata validation and concurrent-transaction checks remain.
- Target COMMIT still requires a newer explicit localization and a settled,
  published target transform. Optional source compensation does not fabricate
  a valid source when none existed.

## Verification

Unit regressions cover failed/missing/starting/unlocalized sources, same-map
reload versus proven no-op, invalid source health alongside an active switch,
source-free BEGIN and refusal to COMMIT an unlocalized target. Existing
transaction/cancel/identity/target-commit tests remain applicable.

`src/robot_api_server/test/features/floor_switch/floor_switch_unready_http_smoke.py`
runs a real API executable with temporary synthetic maps and an external fake
FloorSwitch action in ROS domain 213, localhost only. Run with `--node` pointing
to the candidate and `--output-dir` under `/tmp/njrh_reports`. Test with
`--negative-interlock true` and `false`. It checks HTTP 202 and exact terminal
identity for four source states, missing-target rejection, and no source-context
rewrite. Mock Action success is API-boundary evidence, not physical map loading.

The bridge's `test/run_isolated_floor_transition_smoke.sh` supports
`NJRH_TEST_NODE_EXECUTABLE`, `NJRH_TEST_SOURCE_ROOT`, `NJRH_TEST_LOG_FILE` and an
isolated `NJRH_TEST_ROS_DOMAIN_ID`. It disables TF publication and verifies the
actual ROS service accepts source-free BEGIN without granting target readiness.

## Deployment and remaining hardware check

Deploy API and bridge together, retaining the unchanged ROS interface. Do not
deploy unrelated navigation recovery candidates with this fix. A full runtime
restart requires separate operator authorization; no individual node restart.

At diagnosis on 2026-09-09, startup had already failed with
`LOCALIZATION_RESULT_TIMEOUT`, and the resident localization/Nav2 stack had
been torn down. Removing source readiness does not create missing services or
fix Isaac's missing result. A live switch still needs its actual map/localizer/
bridge services. This patch deliberately does not add automatic startup or
restart to conceal that separate failure.

Physical acceptance remains: with the runtime services present, request two
maps consecutively from an unlocalized source and verify actual target loading,
target localization and no false success. No vehicle movement is required or
authorized by these tests.
