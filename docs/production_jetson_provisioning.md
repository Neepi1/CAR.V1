# Jetson 一键量产部署

本流程面向 Ranger Mini 3 的 Jetson Orin NX 量产交付。它冻结并复用当前已验收的导航链，不修改 odom、EKF、AMCL、MPPI、目标容差、FAST-LIO2、JT128 QoS/DDS 或时间戳。

## 交付状态机

```text
GitHub clone
  -> enroll-trust（每个工厂镜像/信任域一次）
  -> deploy（下载、验签、预检、离线构建测试、原子切换）
  -> READY_LOCKED
  -> accept-hardware（工位静态验收）
  -> activate（显式允许运动并检查完整 ROS/Nav2/安全链）
  -> ACTIVE
```

`deploy` 成功后仍保持运动锁，systemd 服务处于 disabled/inactive。没有站点地图资产时状态为 `READY_NO_MAP_LOCKED`，该状态不能执行 `accept-hardware` 或 `activate`。

## 固定量产基线

- Jetson 型号：`NVIDIA Jetson Orin NX Engineering Reference Developer Kit Super`
- TNSPEC：`3767-303-0000-D.1-1-1-jetson-orin-nano-devkit-super-`
- JetPack：`6.2.1+b38`
- L4T：`36.4.3-20250107174145`
- Docker 必须注册并能实际运行 `nvidia` runtime
- 主机工作区：`/home/nvidia/workspaces/njrh-v3/workspace1`
- 容器工作区：`/workspaces/njrh-v3/workspace1`
- 上游运行资产：`/home/nvidia/workspaces/isaac_ros-dev`
- CAN：`can0 / 500000`
- JT128：`eth1 / 192.168.1.100/24 -> 192.168.1.201`，UDP `2368/10110`
- Orbbec：唯一 `2bc5:0807` 设备且 USB 速率至少 `5000 Mbps`
- 目标文件系统部署前至少保留 `60 GB` 可用空间，Docker 数据目录至少
  `30 GB`；这是为大体积 upstream 分片重组、事务 staging 和回滚备份预留

示例 manifest 中的 Orbbec `serial: "AUTO_ENROLL"` 只表示一次性首装登记：仅当
`/etc/njrh/device-identity.json` 尚不存在、并且预检只发现一台合格的 Orbbec
时，`deploy` 才把实测 serial 作为同一原子事务的一部分写入该 root `0600`
身份文件。它不是永久通配符；身份文件存在后，即使后续 release 仍写
`AUTO_ENROLL`，部署、校验和激活也必须匹配已登记 serial，否则保持运动锁并
失败关闭。量产现场不得通过删除或改写身份文件来更换相机；换件必须走受控的
重新登记和硬件验收流程。若发布系统能生成逐机 manifest，也可直接把该值替换为
目标机的精确 serial。

目标机预装要求：Git、Python 3、curl、OpenSSL、Docker、NVIDIA Container Runtime、iproute2、can-utils、tcpdump。不要在部署脚本中临时下载“最新版”系统依赖；这些依赖属于受控 JetPack 工厂镜像。

## 信任根

发布清单使用 Ed25519 detached signature。私钥只能保存在离线发布机或受控签名服务，不能进入 Git、GitHub Release、Docker 镜像或 Jetson。

首次建立工厂信任根：

```bash
bash scripts/jetson/provision_njrh.sh enroll-trust \
  /secure/factory-release-public.pem \
  <从独立可信通道取得的64位小写SHA256>
```

已存在不同公钥时命令会拒绝隐式轮换。量产镜像可预置：

```text
/etc/njrh/trust/release-ed25519-public.pem
```

文件必须是 root 所有的普通文件，权限为 `0644` 或 `0600`。

## 在黄金 Jetson/CI 构建发布

发布前必须先形成干净 Git commit。构建器会拒绝 tracked、untracked 或 ignored 规则之外的工作树漂移。

仓库中的 `device-release.example.json` 和 `release-lock.json` 只是模板/黄金机
快照，包含 candidate、零摘要或占位 URL，不能直接用于目标机部署。只有下面
构建器输出目录中的 `published` manifest、lock 和签名才是可部署发布。

推荐先把运行镜像推到公开或已完成目标机凭据配置的 GHCR，并保留 release-specific tag：

```bash
export RELEASE_ID=car-v1-2026.07.25
docker tag njrh-car:latest ghcr.io/neepi1/car-v1/njrh-car:${RELEASE_ID}
docker push ghcr.io/neepi1/car-v1/njrh-car:${RELEASE_ID}
```

生成发布 bundle：

```bash
python3 scripts/jetson/provision/build_production_release.py \
  --template scripts/jetson/provision/device-release.example.json \
  --release-id "${RELEASE_ID}" \
  --output-dir "/tmp/${RELEASE_ID}" \
  --private-key /secure/factory-release-ed25519-private.pem \
  --oci-runtime-image "ghcr.io/neepi1/car-v1/njrh-car:${RELEASE_ID}" \
  --include-site-assets
```

`--include-site-assets` 会先完整验证 map asset registry、MapManifest v2、11 个角色文件 canonical digest、poses、active/current 投影，以及电梯 immutable release 与地图 epoch/digest 绑定。黄金机仍为 legacy 地图时构建会按设计失败，必须先由地图/电梯配置管理流程发布合格的 `maps_release`。

如目标环境不能访问 OCI registry，可改用：

```bash
--docker-save-runtime-image "njrh-car:${RELEASE_ID}"
```

发布目录包含：

- `device-release.json`
- `release-lock.json`
- `device-release.json.sig`
- `njrh-upstream-runtime.tar.gz` 或它的
  `njrh-upstream-runtime.tar.gz.part-NNNN-of-NNNN` 分片
- `njrh-runtime-overlays.tar.gz` 或对应分片
- `njrh-site-assets.tar.gz` 或对应分片（启用站点资产时）
- `njrh-runtime-image.tar` 或对应分片（仅离线镜像模式）

GitHub Release 要求每个上传文件小于 2 GiB。构建器会把超过 1900 MiB 的
payload 确定性切分，删除不可上传的原大文件，并把每个分片的 URL、SHA-256、
精确大小及整包 SHA-256 写入签名 manifest。目标机逐片限长下载、验哈希并
流式重组，整包摘要不一致则保持运动锁并拒绝解包。参见
[GitHub Releases 官方限制](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases#storage-and-bandwidth-quotas)。

将这些文件原名上传到同一个 GitHub Release。构建器只生成产物，不自行上传，也不会读取或保存 GitHub token。

## 新 Jetson：clone 后一条命令部署

建议 clone 与发布 tag/commit 一致：

```bash
git clone --branch <release-tag> --depth 1 \
  https://github.com/Neepi1/CAR.V1.git \
  /home/nvidia/workspaces/njrh-v3/workspace1
cd /home/nvidia/workspaces/njrh-v3/workspace1
```

信任根已预置后，执行：

```bash
bash scripts/jetson/provision_njrh.sh deploy \
  https://github.com/Neepi1/CAR.V1/releases/download/<release-tag>
```

该命令会：

1. 通过 HTTPS 下载三份发布元数据；
2. 在写入正式 release 目录或生成设备 API secret 前，使用工厂公钥验证
   manifest signature，并绑定当前 clone 的完整 Git commit；
3. 核对 Git commit、release-lock、平台、镜像和所有 artifact SHA-256；
4. 停止并复核旧 runtime/container，写入运动锁；
5. 被动检查 CAN `0x221`、JT128 UDP 2368、Orbbec USB3 和 NVIDIA runtime；
6. 当签名 manifest 显式使用 `AUTO_ENROLL` 且身份文件不存在时，把唯一合格 Orbbec 的实测 serial 作为同一原子事务的一部分登记到 `/etc/njrh/device-identity.json`；已有身份只严格校验，绝不自动换绑；
7. 在无网络、无 privileged 的临时容器中构建 28 个 ROS 包并运行测试；
8. 只允许精确 6 个已登记历史 Python 契约失败，出现任何新增失败即终止；
9. 对 upstream、`.runtime`、站点资产、install selector 和 runtime.env 做整版本原子切换；
10. 安装但不启用 systemd；包含并通过验证的站点资产时进入
    `READY_LOCKED`，未包含站点资产时进入 `READY_NO_MAP_LOCKED`。

全新设备若没有 API secret，`deploy` 会生成唯一的 64 位十六进制 token，只在工位终端显示一次，并以 root `0600` 写入 `/etc/njrh/secrets.env`。工位必须立即把它保存到凭据库/App 配对记录。旧设备若 token 仍在 `/etc/njrh/runtime.env`，部署器会迁移而不回显。
运行时仅通过进程环境把该 secret 交给 `robot_api_server`；启动脚本不得把 token
展开到 ROS 参数或 shell 命令字符串中，避免普通进程列表暴露凭据。

黄金系统镜像只能预置信任根和受控系统依赖，不能预置
`/etc/njrh/device-identity.json`、`/etc/njrh/hardware_acceptance.json` 或
`/var/lib/njrh/provision`；否则复制出的新板不再满足首次登记和逐机验收语义。

## 工位验收与激活

静态硬件、急停、机械安装、线束、站点地图和安全区域由工位人员确认后：

```bash
bash scripts/jetson/provision_njrh.sh accept-hardware \
  /opt/njrh/provisioning/releases/<release-id>/device-release.json \
  --inspection-id <工位追溯号> \
  --confirm-hardware-inspected
```

准备允许运动时：

```bash
bash scripts/jetson/provision_njrh.sh activate \
  /opt/njrh/provisioning/releases/<release-id>/device-release.json \
  --confirm-motion-enabled
```

激活前会再次执行完整验签、硬件预检、tree digest、site asset、secret、runtime.env、install selector、transaction journal 和 systemd 检查。服务启动后还必须通过 `check_commercial_runtime_ready.sh`，确认 JT128、local-state、唯一 TF owner、Nav2 lifecycle、velocity smoother、collision monitor、robot_safety、地图、costmap 和安全状态；失败会立即重新写入运动锁并停服务。

## 运行参数冻结

量产生成的 `/etc/njrh/runtime.env` 固定包含：

- `NJRH_NAV2_PLANNER_PROFILE=ranger_lattice`
- `NJRH_NAV_LOCAL_STATE_MODE=ekf`
- `NJRH_LOCAL_STATE_EKF_PROFILE=wheel_spin_imu`
- `LOCAL_STATE_EKF_PROFILE=wheel_spin_imu`
- `NJRH_AMCL_LOCALIZATION_MODE=gated`
- `NJRH_ALLOW_BASE_IMAGE_FALLBACK=false`

最终速度链保持：

```text
Nav2 -> velocity_smoother -> collision_monitor
     -> robot_safety -> /cmd_vel -> ranger_base
```

App 仍只能通过受控 API/`/cmd_vel_api` 进入 `robot_safety`，不能直接向底盘发速度。

## 诊断

```bash
bash scripts/jetson/provision_njrh.sh status
bash scripts/jetson/provision_njrh.sh verify \
  /opt/njrh/provisioning/releases/<release-id>/device-release.json
```

报告写入 `/tmp/njrh_reports`。部署失败会保持 `FAILED_LOCKED`，不会自动启动旧或新运行时。事务 journal 与旧版本备份保留在 `/var/lib/njrh/provision` 和目标目录旁，供审计与恢复；不要手工删除或改写。

## 当前仍需真实硬件完成

- 生成并离线保管正式 Ed25519 私钥，分发公钥及 fingerprint；
- 将 immutable runtime image 推送到 GHCR，并确认新 Jetson 可拉取；
- 把 legacy 站点地图升级为合格的 MapManifest v2 + registry + 电梯 release；
- 在一块空白同型号 Jetson 上完成一次 `deploy -> accept-hardware -> activate`；
- 验证断电重启、下载中断、构建失败、原子切换中断和 readiness 失败均保持运动锁；
- 完成 Ranger Mini 3 静态工位检查和受控区域实车验收。
