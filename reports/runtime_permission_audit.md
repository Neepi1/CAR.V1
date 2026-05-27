# Runtime Permission Audit

## Scope

This audit covers the Jetson runtime container, common ROS services, App/API map writes, dashboard test runtime writes, and released floor asset ownership.

## Runtime User Policy

- Container name: `NJRH-car`
- Future container default user: `root`
- Production ROS process user: `root`
- Runtime file creation mask: `umask 0002`
- Backend map writes: `robot_api_server` runs as `root`; root execution is intentional and is the single ownership model for this container.

Root is also used for container/device preparation:

- creating or fixing `/tmp/isaac_ros_nitros` as `root:root` with `1777`
- creating `/dev/gs2` device aliases and permissions
- repairing bind-mounted asset permissions before starting runtime services
- stopping stale mixed-owner processes during recovery

Map, pose, keepout, runtime preview, and dashboard-generated assets must all stay `root:root`.

## Checked Runtime State

Observed on Jetson before repair:

- `docker inspect NJRH-car` had an empty `Config.User`, so plain `docker exec NJRH-car ...` entered as `root`.
- Long-lived repository services were previously running as `nvidia`, while some assets were `root` owned.
- `maps_release/B3/F4/current` was a non-empty real directory owned by `root`, which exposed the mixed-owner risk when services ran as non-root.
- `maps_release/B3/F3/current` also contained root-owned runtime mirror files.

## Fixes Applied

- `scripts/jetson/njrh_container.sh` now starts new containers with `--user root`, `USER=root`, and `HOME=/root` by default.
- `scripts/jetson/runtime_overlay/scripts/common_env.sh` sets `umask 0002`, covering mapping, navigation, dashboard, and helper scripts that source the common environment.
- `robot_api_server` sets `umask(0002)` and is allowed to run as root.
- `robot_api_server` map activation can quarantine a stale `current/` directory inside the same floor directory and recreate a fresh root-owned runtime mirror.
- `prepare_release_asset_permissions` recursively repairs `maps_release` to `root:root`, directory mode `2775`, file mode `664`.

## Expected Ownership Contract

Writable runtime assets must satisfy:

- owner/group: `root:root`
- directories: `2775`
- files: `664`

Covered paths:

- `/workspaces/njrh-v3/workspace1/maps_release`
- `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps`
- `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/maps3d`
- `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/waypoints`
- `/workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/web_dashboard/runtime_logs`

## Manual Operator Rule

Plain `docker exec NJRH-car ...` now enters as root, which matches the runtime model. Prefer setting the workspace explicitly for operational commands:

```bash
docker exec -u root -w /workspaces/njrh-v3/workspace1 NJRH-car bash
```

or:

```bash
bash scripts/jetson/njrh_container.sh shell
```
