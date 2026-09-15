# Coherent API candidates

## Confirmed defect, 2026-09-15

The installed API `361959633ed549be41bdd5f7598e77bfa9ea717a6f1aa81ce0abe56eed388ef3`
linked `MappingModule` compiled with a 232-byte `MappingProcessRuntime` declaration
to the implementation compiled with the new 288-byte declaration (AArch64).
The new state mutex overlapped the following start-job object. Live evidence
showed all 16 HTTP workers waiting on one futex, including its recorded owner.
This is incompatible object layout, not a need for longer HTTP timeouts.

The most recent restart changed the localization bridge, not this API binary.
The bridge's immediate explicit-relocalization behavior is not changed here.

## Repair and scope

Reconstruct the approved deployed API source revisions, including both the new
mapping-runtime header and implementation and the deployed localization change.
Build the entire executable and its API-owned libraries using one source tree.
Do not borrow old `.o` files, even when they previously passed tests. Header
changes can affect translation units outside the file being edited.

`scripts/jetson/build_api_coherent_candidate.py` requires a new output directory,
records all source hashes, uses CMake's complete dependency graph, audits the
compiled dependency files for foreign API headers, verifies source stability,
and installs only into its candidate staging directory. It rejects missing
shared libraries. `candidate.json` is build evidence, not a claim of passed tests.
An existing build output is neither reused nor deleted.

In the container, after sourcing ROS and the workspace:

```bash
nice -n 15 taskset -c 0,1,2,3,4 python3 scripts/jetson/build_api_coherent_candidate.py \
  --source /tmp/njrh_reports/APPROVED_SNAPSHOT/robot_api_server \
  --output /tmp/njrh_reports/NEW_UNIQUE_CANDIDATE --jobs 2
```

Use `--dependency-cache` only with the verified previous CMake cache to preserve
approved dependency package locations. It imports package locations, not old
object files or result/output directories. Selecting the source snapshot remains
a reviewed deployment decision: never silently build all unrelated workspace
changes. The builder does not install into the live workspace or restart it.

## Regression and acceptance

`test/infrastructure/http/api_pose_query_regression.py` executes the actual API
with low-rate synthetic map/odom/base TF. Its first robot-pose response must be
HTTP 200 with the known coordinates, so a fast `NO_FRESH_POSE` rejection cannot
hide the faulty map/mapping snapshot path. It then performs 100 rounds of 12
concurrent pose, navigation-state, maps and status GETs, below the 16-worker limit.
Each request must succeed within the client bound. Tests never send motion,
map switching, localization triggers, or arm requests.

The runner executes a byte-identical copy under a distinct fixture path, never
the production executable path. Ancestor PID namespaces can still see child
processes; PID isolation alone does not prevent the production exact-executable
watchdog from counting a second API. This distinction is mandatory.

The runner requires private PID, mount, network and IPC namespaces, private SHM,
loopback only, a distinct ROS domain and temporary state. Do not run the Python
fixture against the production ROS domain. In the container as root:

```bash
bash scripts/jetson/test_api_pose_queries_isolated.sh \
  /tmp/njrh_reports/NEW_UNIQUE_CANDIDATE/stage/lib/robot_api_server/robot_api_server_node \
  /tmp/njrh_reports/NEW_UNIQUE_QUERY_TEST
```

The old production binary reproduced a query timeout under this isolated
concurrent workload. A shorter low-concurrency test passed, demonstrating why
startup-only HTTP checks were insufficient. Preserve both results, not just the
failing one. Run the unchanged test against the clean candidate, plus targeted
mapping concurrency/save, HTTP/status, and localization regressions.

Reports and exact candidate hashes are under
`/tmp/njrh_reports/api_coherent_build_20260915`. Physical navigation, mapping,
elevator and docking acceptance remain user-operated. Before activation, verify
the original production PID/hash and no unintended restart. Do not replace the
running executable out of band: activation requires separate authorization and
the prescribed complete `njrh-runtime.service` restart. No single-node restart.
