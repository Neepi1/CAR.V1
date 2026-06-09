# Phase 1.12 CPU / IRQ Affinity Audit

This audit records the existing runtime placement framework and the Phase 1.12
diagnostic extension. It does not change the commercial navigation architecture:
FAST-LIO2 stays mapping-only by default, `/perception/obstacle_points` remains
the local costmap obstacle source, and final velocity arbitration still goes
through `robot_safety`.

## Existing Runtime CPU Policy

| Owner | File | Current role | Notes |
|---|---|---|---|
| CPU set defaults | `scripts/jetson/runtime_overlay/config/cpu_affinity.env` | Defines Jetson 8-core service groups and concrete `NJRH_CPUSET_*` variables. | CPU4 is the JT128 driver, CPU5 pointcloud ingress, CPU6 localization/local perception, CPU7 map-odom bridge/mapping backend. |
| Affinity helper | `scripts/jetson/runtime_overlay/scripts/cpu_affinity.sh` | Converts service names to `NJRH_CPUSET_*`, starts commands with `taskset`, and can retag every thread under live PIDs. | Phase 1.12 adds an optional `cpu_affinity_runtime_override.env` source after defaults for reversible A/B without editing the baseline file. |
| Runtime launchers | `run_driver.sh`, `run_local_state.sh`, `run_local_perception.sh`, `run_global_localization.sh`, `run_localization_bridge.sh`, `run_nav2_navigation.sh`, `run_robot_safety.sh`, and related helpers | Source `cpu_affinity.sh` and use the configured groups when starting services. | Existing broad cleanup in older launchers is not copied by Phase 1.12 scripts. |
| Phase 1.11 CPU A/B | `run_pointcloud_cpu_affinity_ab.sh` | Reversible pointcloud CPU plan around local perception and nav scan chain. | It edits the baseline CPU config block. Phase 1.12 keeps new experiments in a runtime override file instead. |
| CPU observation | `diagnose_pointcloud_cpu_pressure.sh`, `inspect_pointcloud_cpu_affinity.sh` | Read-only process/core/thermal placement diagnostics. | No full-density `/lidar_points` subscriber is added. |

## Current Bottleneck Hypothesis

Recent 20-second field sampling showed CPU4 dominated by `hesai_ros_driver`,
CPU6 shared by local perception, nav scan conversion, and occupancy localization,
and CPU7 mostly idle. The likely remaining issue is scheduling/placement
contention around full-density ingress, derived pointcloud branches, net IRQ, and
softirq handling. This is not enough evidence to change timestamps, reliable
QoS, DDS middleware, or Nav2 controller/planner parameters.

## Phase 1.12 Additions

| Tool | Default mode | Applies changes | Purpose |
|---|---|---:|---|
| `collect_cpu_irq_softirq_snapshot.sh` | Read-only 20 s snapshot | No | Captures tegrastats averages, project thread placement, ksoftirqd, `/proc/interrupts`, `/proc/softirqs`, NET_RX deltas, IRQ affinity, and RPS/XPS masks into `reports/`. |
| `identify_lidar_network_irq.sh` | Read-only | No | Infers the LiDAR NIC from the running Hesai config, reports candidate LiDAR IPs, IRQ lines, IRQ deltas, NET_RX softirq deltas, RPS/XPS masks, and whether the NIC is also the SSH/default route interface. |
| `run_cpu_core_allocation_ab.sh` | Dry-run plan | Only with `--apply`; live retag only with `--restart` | Writes a reversible `cpu_affinity_runtime_override.env` and applies live `taskset -pc` through existing helpers. It does not kill processes. |
| `run_lidar_irq_affinity_ab.sh` | Dry-run plan | Only with `--apply` | Backs up LiDAR IRQ `smp_affinity_list` and interface RPS/XPS masks, then applies explicit IRQ/RPS/XPS profiles. It refuses SSH/default-route interfaces unless `--allow-ssh-interface-risk` is provided. |
| `run_pointcloud_cpu_irq_experiment.sh` | Dry-run plan | Only with `--apply` | Runs baseline snapshot, CPU/IRQ profile apply, profile snapshot, local/nav pointcloud diagnostics, and restores automatically unless `--keep-applied` is set. |

## Recommended First Experiment

Start read-only:

```bash
bash scripts/jetson/runtime_overlay/scripts/collect_cpu_irq_softirq_snapshot.sh --duration-sec 20
bash scripts/jetson/runtime_overlay/scripts/identify_lidar_network_irq.sh --duration-sec 10
bash scripts/jetson/runtime_overlay/scripts/run_cpu_core_allocation_ab.sh --profile split_local_nav_v1 --print --no-diagnostics
bash scripts/jetson/runtime_overlay/scripts/run_lidar_irq_affinity_ab.sh --profile irq_keep_default --print --no-diagnostics
```

If CPU6 remains concentrated and CPU7 remains available, the first reversible
CPU-only applied run should be:

```bash
bash scripts/jetson/runtime_overlay/scripts/run_pointcloud_cpu_irq_experiment.sh \
  --cpu-profile split_local_nav_v1 \
  --irq-profile irq_keep_default \
  --duration-sec 120 \
  --apply
```

Only test IRQ profiles after identifying the LiDAR NIC and confirming it is not
the SSH/default-route interface, or after accepting the explicit SSH risk flag.

## Mapping-Mode A/B Result

Field A/B on the Jetson while live 2D mapping was active showed that the low
`/lidar_points` rate was not caused by `pointcloud_axis_remap` compute time:
`last_publish_duration_ms` stayed around 2 ms while raw input inter-arrival
jitter increased. Pinning mapping-owned FAST-LIO2 to CPU7 reduced CPU6
contention, and applying eth1 RPS/XPS to CPU5 recovered
`raw_input_hz` / `lidar_points_publish_hz` to approximately 19-20 Hz. The eth1
IRQ (`IRQ257`) still rejected affinity writes even with sudo, so production
runtime must not depend on IRQ migration.

The validated mapping-scoped default is now owned by
`scripts/jetson/runtime_overlay/scripts/run_projected_map.sh`:

- mapping-owned FAST-LIO2 frontend/deskew defaults to CPU7 through
  `NJRH_SLAM2D_FASTLIO_CPUSET`.
- live 2D mapping temporarily writes eth1 RPS/XPS masks for CPU5 through
  `NJRH_SLAM2D_LIDAR_RPS_XPS_*`.
- mapping cleanup restores the previous RPS/XPS masks from the script EXIT trap
  or the API server's `mapping_lidar_rps_xps_state_dir` restore path, and still
  cleans only mapping-private FAST-LIO2 processes.
- navigation runtime remains unchanged: FAST-LIO2 is not resident by default,
  and no IRQ/RPS/XPS writes are done by navigation startup.

## Restore

```bash
bash scripts/jetson/runtime_overlay/scripts/run_cpu_core_allocation_ab.sh --restore --restart --no-diagnostics
bash scripts/jetson/runtime_overlay/scripts/run_lidar_irq_affinity_ab.sh --restore --no-diagnostics
```

The combined experiment runs both restore steps automatically unless
`--keep-applied` is explicitly passed.

## Guardrails

- No QoS reliability change for `/lidar_points`, `/lidar_points_nav`, or local
  obstacle branches.
- No DDS middleware/default transport change.
- No timestamp restamping fallback for pointcloud or local perception latency.
- No Nav2 planner/controller/EKF/FAST-LIO2 logic changes.
- No new high-frequency Python PointCloud2 subscriber.
- No navigation-mode IRQ/RPS/XPS write; live 2D mapping has a scoped RPS/XPS
  profile and restores it on exit.
- No broad `killall` or `pkill -9` in the new Phase 1.12 scripts.
