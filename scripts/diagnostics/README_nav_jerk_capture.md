# 导航顿挫记录器

目的：保留正常导航、避障、接近目标时的原始证据，找出**首先观测到**速度变化、
零速、断更或反向的公开链路边界。不预判 MPPI、BMS、安全节点或底盘的责任。
这是观察工具，不是导航功能，也不会恢复/取消任务。

## 2026-09-17 修订：六组证据与持续漏采

旧版在正式录制阶段改变 `spin_once` 的 timeout 参数，触发 Humble 执行器重复创建
ready iterator，繁忙的前排话题可能让后排 Scan/TF/odom 回调饥饿。现固定为 50ms
**最长空闲等待**，有事件即处理，不是 20Hz 采样，也没有新增 sleep。
HTTP 改用现有 API 的 `X-Robot-Token`，不是 Bearer。

每秒检查连续传感/odom/TF 最近接收时间；超过诊断阈值（默认 2 秒）打印
`INCOMPLETE continuous capture` 并保留记录。HTTP 错误打印 `HTTP_INCOMPLETE`。
二者只描述记录质量，不推断生产节点故障、不阻止运动。最终记录保留曾经发生过的缺口；
事件话题和空闲命令不会因不发布而被这项检查判为超时。
`report` 新增 `marker_coverage.csv`，逐个人工标记、逐话题列出前后各 5 秒的条数、
首末采集序号、内部最大间隔和窗口两端空白。`/tf_static` 可沿用此前收到的保留消息。

本次再次读取当前 ROS graph/参数，四级速度重映射未改变。六组数据对应如下：

|组|原始证据|边界|
|---|---|---|
|① 碰撞动作/原因|`/rosout` 完整日志级别、时间、节点、文本；现有运行日志文件及轮转内容|当前没有 CollisionMonitorState 发布者；日志没提供的原因/解除时刻不能编造|
|② 实际扫描|collision_monitor 当前实际订阅的 `/scan`，完整 CDR，包含 ranges/角度/stamp/frame|不以最小距离替代原扫描|
|③ TF/轮廓/区域|`/tf`、`/tf_static`、`/local_costmap/published_footprint`；开始/结束实际碰撞参数|不启用原本关闭的区域可视化，不改车体或区域|
|④ 四级速度/限速|`/cmd_vel_nav_raw` → `/cmd_vel_nav` → `/cmd_vel_collision_checked` → `/cmd_vel`，以及 `/speed_limit`|保留完整、有符号 vx/vy/wz；API/对桩/其他现有来源也保留|
|⑤ 实际运动/安全|`/local_state/odometry`、`/wheel/odom`、`/ranger_base/status`、motion/actuator、安全与互锁状态|现有字段原样保留；没有 CAN 原始样本时间就标 unknown|
|⑥ 路径/局部地图/控制器日志|`/received_global_plan`、`/transformed_global_plan`、`/ranger_mini3/ordinary_local_repair_path`，公开 plan 等；局部 costmap 全量+更新；`/rosout`/运行日志|公开路径/地图不是已证明的“控制器内部当周期快照”，不拿路径末端充当业务目标|

本次只读快照的区域（base_link，单位米）：

- StopZone：x=±0.47，y=±0.36，stop，max_points=1。
- SlowZone：x=[-0.55,0.85]，y=±0.45，slowdown_ratio=0.65，max_points=12。
- FootprintApproach：published_footprint，time_before_collision=2s，simulation_time_step=0.1s。
- 输入 `/scan`；source_timeout=1.5s，transform_tolerance=0.1s。以上只是快照，
  每次录制重新读实际参数，不用这些常量代替查询失败的参数。

新的临时候选位于 `/tmp/njrh_reports/nav_jerk_candidate_v2_20260917/`，不是部署到生产工作区。
旧候选和旧证据未覆盖。需要手动使用新候选时，在 Jetson SSH 中执行：

```bash
docker exec -it NJRH-car bash -lc '
  source /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/common_env.sh
  python3 /tmp/njrh_reports/nav_jerk_candidate_v2_20260917/nav_jerk_capture.py record \
    --workspace /workspaces/njrh-v3/workspace1 --duration 300
'
```

回车打标；另一终端的 `mark`、离线 `report` 也使用这个新候选路径。
**持续采集修正不等于低 CPU 验收**：隔离的多话题负载对照约 0.72 个 CPU 核，仍明显；
没有通过降低频率、丢弃 Scan/轮速或省掉 TF 来降低数字。正式运动时的 300 秒开销和
完整覆盖尚未验收。不要把先前短时 8 路测试的 0.36 核当成当前全量录制的开销。

## 范围与风险

- 一个临时 rclpy participant、每个 topic 一个 raw subscription、一条 rosbag2_py 写盘路径。
- 只创建 GET/LIST parameter 客户端；不创建 Action 客户端，不发布速度/TF/目标，
  不调用运动、恢复或定位服务，不改生产参数、QoS、日志级别、可视化或启动方式。
- HTTP 限于审计过的三个 GET：`navigation/state`、`status`、`docking/state`。
  **总计不超过 1 次/秒**，轮询三个接口，单次超时 0.7 秒；不用 POST 或订阅租约。
  优先读取 `ROBOT_API_TOKEN`，同机时可读取匹配 API 可执行进程的环境；不输出令牌。
- 默认不订阅 PointCloud2、图像、MPPI MarkerArray。`--deep` 仅允许已发现相关
  PointCloud2；不会启用原来关闭的发布者。这个选项开销明显更高。
- DDS/序列化/磁盘写入都有开销，不保证“零开销”或“源端无丢帧”。记录器 CPU/RSS、
  `/proc` I/O、队列峰值和丢弃、磁盘余量每秒记录一次。
  当前多话题模拟约占 0.72 个 CPU 核；正式全话题负载尚未验收，详见上节及验证记录。
  `/proc/io` 在本机内核不可用时明确留缺口，不能填作 0。
- 不启动任何生产程序，不部署，不重启，不安装依赖，不切分支，不回放 bag。

## 使用

文件在本地项目中。**正式 Jetson 工作区没有自动同步这次新增脚本**。
需要在用户决定开始采集时，仅手工同步这个诊断目录（不需要构建/重启）：

```powershell
# 本地 PowerShell，workspace1 根目录；只同步新增诊断目录。
scp -r .\scripts\diagnostics nvidia@192.168.31.23:/home/nvidia/workspaces/njrh-v3/workspace1/scripts/
ssh -t nvidia@192.168.31.23
```

Jetson SSH 终端内：

```bash
docker exec -it NJRH-car bash
source /workspaces/njrh-v3/workspace1/scripts/jetson/runtime_overlay/scripts/common_env.sh
cd /workspaces/njrh-v3/workspace1

# 实际 graph、端点/QoS、运行参数、配置和可执行文件身份；有界消息检查。
python3 scripts/diagnostics/nav_jerk_capture.py inspect

# 默认 300 秒，预算 2048 MiB，另保留 256 MiB 空闲空间。
python3 scripts/diagnostics/nav_jerk_capture.py record --duration 300 --disk-budget-mb 2048
```

脚本打印唯一 `report_dir=/tmp/njrh_reports/nav_jerk_record_...`。
原始数据从订阅建立后开始保留，不等人工标记或自动事件才保存。
`READY` 表示本次关键消息已收到且连续话题近期有数据，**不是小车就绪判定**。
静止时控制器/平滑器不发消息可能显示 `INCOMPLETE`；继续录制、由操作者通过 App
正常发目标即可。最终 coverage 及逐标记窗口覆盖分别报告接收情况，记录器不会阻止导航。
发现流程在录制时长之前；默认 300 秒录制包含消息就绪检查，结束参数/配置快照另计。

感觉到顿挫时按回车，或从另一个 SSH 终端（无需 ROS 环境）运行：

```bash
python3 /workspaces/njrh-v3/workspace1/scripts/diagnostics/nav_jerk_capture.py mark \
  --directory /tmp/njrh_reports/nav_jerk_record_<实际目录> --label '绕障到侧面时顿挫'
```

`mark` 只原子写入本记录目录的 `marks/*.json`，绝不向 ROS 发消息。
同一 Jetson 的 monotonic 时间可对应；异机标记使用记录器收到标记的时间并保留原值，
不把不同主机 monotonic 时钟直接相减。标记可以有少量人工反应延迟。

Ctrl+C 只停止记录器；保留 bag，标记为 interrupted/incomplete，不停车、不取消导航。
超时自然结束；预算/空间不足则停止接收并尽量完成有限队列与 bag 收尾。
预算检查有每秒一次的采样间隔，并为已接收队列/SQLite 收尾预留空间，不是文件系统硬配额。
已有输出目录拒绝复用；不要把过去的证据目录作为新 record 的输出目录。

离线导出（不初始化 ROS、不播放 bag；默认 sqlite3 在 Windows 也能导出）：

```bash
python3 scripts/diagnostics/nav_jerk_capture.py report \
  --directory /tmp/njrh_reports/nav_jerk_record_<实际目录>
```

可先 `scp -r` 整份目录到本机再执行相同 report 命令。非 sqlite3 bag 的原始 CSV
导出需要已有 rosbag2_py 对应插件；不会自动安装。每次 report 创建新的 `report_<随机>`。
默认使用采集时冻结的配置；想改变离线候选阈值时显式传 `--config`。
配置是 JSON 兼容 YAML，可只用标准库读取；普通 YAML 需要已有 PyYAML。

## 当前实机映射（2026-09-17 15:38 UTC / 23:38 北京时间）

本地分支 `codex/elevator-config-management`，审计开始 SHA
`8d01ee83994b7e83ab34cddf892b4541504a2959`。没有切分支。
实际程序 argv、运行参数和 graph 共同确认以下连接；每次 inspect/record 都重新发现，
不把这张表用作名字猜测或永久配置。

|公开边界/来源|实际话题|类型|审计时发布端|
|---|---|---|---|
|控制器组合输出 / 平滑器输入|`/cmd_vel_nav_raw`|Twist|controller_server ×1|
|平滑器输出 / 碰撞输入|`/cmd_vel_nav`|Twist|velocity_smoother ×1，behavior_server ×3|
|碰撞输出 / 普通安全输入|`/cmd_vel_collision_checked`|Twist|collision_monitor ×1|
|安全最终输出 / 底盘入口|`/cmd_vel`|Twist|robot_safety ×1|
|安全诊断镜像|`/cmd_vel_safe`|Twist|robot_safety ×1|
|API 末端/遥控来源|`/cmd_vel_api`|Twist|robot_api_server ×2|
|对桩来源|`/cmd_vel_docking`|Twist|robot_api_server ×1，docking ×1|

同一话题承担的角色去重。`/cmd_vel_nav` 还有电梯专用安全输入角色，但是否使用
由现有许可/状态决定；普通导航顺序不因此改变。发布端数量不是进程数量。
最终 `/cmd_vel` 的底盘订阅节点实际叫 `/ranger_base_node`。

运行 `FollowPath.plugin=robot_nav_config::GoalScopedRotationShimController`，
`primary_controller=robot_nav_config::RangerMPPIController`。
内部纯 MPPI 输出没有独立端点，**不能把 `/cmd_vel_nav_raw` 称为纯 MPPI 输出**。

同时发现：控制器 odom `/local_state/odometry`，`/wheel/odom`、`/wheel/odom_ekf`，
原始/校正 IMU，`/motion_state`、`/actuator_state`、`/ranger_base/status`，
原始 `/battery_state`，安全/许可/定位桥/AMCL 状态，Action feedback/status，
`/speed_limit`、Path、`/tf`、`/tf_static`、`/parameter_events` 和 `/rosout`。
实际避障输入是 `/scan`，被 local_costmap 与 collision_monitor 订阅；没有恢复旧点云避障链。
局部 costmap 参数 `10 × 10 m`、`0.05 m` 分辨率，约 40000 cells，
录制全量 `/local_costmap/costmap`、增量 `costmap_updates` 和 published_footprint。
不订阅大体量全局地图；缺初始全量或增量无法证明连续时，不宣称可完整重建。

当前 Nav2 `1.1.19`、rclpy `3.3.17`、rosbag2 `0.15.15`、rmw_fastrtps_cpp `6.2.9`，
已加载 Fast DDS `2.6.12`；域 `0`，已查节点 `use_sim_time=false`。
sqlite3/mcap 已安装，默认仅使用 sqlite3 主录制路径。

实机 inspect 观察到关键传感/最终速度消息，但车静止，前三段未发速度，正确输出
`INCOMPLETE`。没有发目标来“补齐” READY。

### 版本证据边界

Jetson 容器源码目录无 `.git`，不能把本地分支/SHA称为实机部署版本。
审计的 launch、nav2.yaml、safety.yaml、safety CPP、Ranger CPP、导航与对桩 HTTP 模块
本地/实机 SHA256 相同。system_status CPP 字节哈希不同但忽略行尾后无内容差异。
RotationShim CPP **有真实内容差异**：本地含更新后的 ordinary recovery inspect/
preserve_startup 代码，而 Jetson 源码快照是此前形式。本轮不覆盖或修复这些差异。
已保存运行 controller、safety、Ranger、API 及已加载插件库的哈希；缺完整构建来源证明时，
`source_binary_consistency=unverified`。仅配置/源码同哈希不证明二进制由它构建。

审计原件：`/tmp/njrh_reports/nav_jerk_inspect_20260917T153852Z_ewfgggsr`。

## 数据、索引和结论

- `bag/`：完整原消息 CDR，不按自动事件裁剪、不压缩。
- `index.jsonl`：每条入盘消息的全局采集序号、topic ordinal、接收 monotonic/wall/ROS
  时间、原始 header/frame、类型、原始字节数，以及小消息完整解码数据。
  `topic + bag_timestamp_ns` 是明确关联键；bag 时间使用唯一有序存储键，
  **不是原始源时间**，wall 跳变时仍保留真实 wall 值。
- 当前 Humble rclpy 的回调只传消息，不暴露 MessageInfo；逐消息 GID/RMW 源时间为 null。
  graph 中 GID、发布/订阅节点、QoS 全部保留。多发布者消息不能强行归到某个节点。
- reliability 取兼容发布端的策略；只有所有发布端都是 transient_local 才请求保留数据。
  混合 durability 的历史保留缺口会标出。只改记录器 QoS，不改生产端。
  RMW graph 返回 depth=0 时只是返回值，不推测生产队列真是 0。
- 当前 rclpy 不暴露订阅 matched count：填 null，另列 graph 数、收到消息的事实、
  incompatible_qos 和可用的 message_lost 事件。没有 DDS/source 丢失事件不意味着丢失为 0。
- `parameters_start/end.json`、`provenance_start/end.json`、文件快照保留运行参数、
  启动文件/BT、分支/差异、程序路径及 SHA。超时字段明确列出，不以源码默认值补齐。
- `auxiliary.jsonl`：人工标记、最多 1Hz 的 HTTP/性能、每 10 秒 graph、日志分块、轮转事件。
  日志保留原文和 inode/offset；初始只保留每个文件末尾 1MiB，旧内容截断量有记录。
  rename 轮转先读旧 inode；copytruncate 无法找回的瞬间内容明确 unknown。
- `capture_result.json`：终止原因、覆盖率、入盘数、有界队列丢弃范围与计数、写盘错误。
- 离线 `timeline.csv` / `state_events.csv`、`raw_bag.csv`、`events.csv` / `events.jsonl`、
  `quality.json`、`derivative_estimates.csv`、`velocity.svg`、`summary.md`。
  raw_bag.csv 是完整原始 CDR 的 base64 CSV，不是伪造的字段解码。
  小消息解码在 timeline.csv；原始 Scan/Grid/Path 完整内容仍在 bag 和 raw_bag.csv 中。
- 每个事件窗默认前后 5 秒，列出最先观察到变化的公开边界、上游最近样本及索引、
  实际 odom、相邻状态和 HTTP 阶段。分为直接观察、相关线索、待验证，不自动判故障。
- 只在最近观测 Action 活跃区间分析正常控制链的长间隔；不把 API/对桩偶发指令的
  空闲停发当作断流。正常安全停车/正常到点停车也可能成为检索候选，不能直接判异常。
- 单次命令阶跃/零速/反向阈值可调，角速度零附近有死区。加速度/jerk 是接收时间估计，
  不跨长缺口、不用重复/回退源时间计算。图中缺口断线，不插值，没有离线平滑篡改原值。
- 默认不计算源年龄：同步未证明。只有操作者独立验证相同 ROS 时钟域后，才可在
  **诊断配置**中设 `source_clock_verified=true`。这不会修改车辆参数。
- 原始 BMS current/voltage/status/present/percentage 不转换、不代替为“接触”布尔值。
  Ranger 公共 stamp 是发布时刻，不能当 CAN 帧接收时刻；CAN 原始样本序号/年龄缺失。
  当前 actuator 数组的部分 SDK 字段重复，不能冒充 8 路独立实测轮速/转角。
- 现有 collision-monitor state 在这台 1.1.19 上未观察到发布者，日志与速度边界只能
  提供相关线索。内部有效 MPPI 限制、拒绝原因、每周期耗时均属于 observability_gaps。

## 隔离测试

```bash
# 普通离线测试；无 ROS、无网络
python3 -m unittest discover -s scripts/diagnostics -p test_nav_jerk_capture.py -v
python3 scripts/diagnostics/test_nav_jerk_required_evidence.py -v

# 已 source 现有 ROS 环境且具备 unshare/mount 权限时执行；不安装包。
bash scripts/diagnostics/run_nav_jerk_isolated_tests.sh
```

ROS 集成测试强制新 net+IPC namespace、私有 /dev/shm、domain 187、loopback-only，
启动测试自有发布者，不连接 Ranger CAN，不运行真实导航/安全程序。离线测试及模拟
录制通过不代表导航顿挫已解决，也不等于 300 秒实车负载验收。
测试记录见 [validation_nav_jerk_capture.md](validation_nav_jerk_capture.md)。

尚待操作者采集：一次流畅导航和一次顿挫导航；核对每次实际 topic_map、coverage、
队列/写盘错误和记录器负载，再判断现场第一处可见变化。不能从源码或隔离测试替代这一步。
