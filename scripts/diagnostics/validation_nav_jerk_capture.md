# 导航顿挫记录器验证记录

日期：2026-09-17（UTC）。对象仅为新增诊断工具，不是导航修复或实车验收。

## V2 修订与旧版实录缺口（优先于下方初版结论）

用户实录 `nav_jerk_record_20260917T163419Z_ydczuw7x`：19 个标记的前后 ±5 秒
均没有 `/wheel/odom`、`/scan`；轮速仅录制开头约 5 秒，HTTP 106/106 次为 401。
实录记录器平均约 83.45% 单核 CPU。初版短时集成测试未覆盖“readiness 之后、
繁忙前排话题竞争”的阶段，HTTP 在模拟测试中关闭；初版验收不足。

确认原因与本次修正：

1. Humble `wait_for_ready_callbacks` 在传入 timeout 改变时重建 ready iterator。
   旧主循环反复传剩余时间，导致排后的订阅饥饿。记录器现在保持固定 50ms 最大空闲等待，
   不固定采样周期、不加 sleep、不修改 ROS 库。实际安装库的独立 ready-event 回归
   旧版 wheel/Scan/TF 均 0 条，修正后都能继续。
2. HTTP 从 Bearer 改为项目现有 `X-Robot-Token`。同一实机只读 `/api/v1/status`
   认证对照分别得到 401/200；本次新增本地假 HTTP 服务测试，旧实现失败、新实现通过。
3. 增加每秒连续接收新鲜度、HTTP 成功/失败计数及每标记窗口覆盖；发生过持续缺口时，
   即使结束前恢复也不宣称完整。只报告采集质量，不推断车辆故障或添加业务条件。
4. 消息索引改用缓存的公开字段模式，避免逐字段反复解析类型和重复 JSON 清洗；原 CDR
   全部保留。等价测试曾发现 byte 字段转换差异，已修正并保留用例，未放宽断言。

新版主体 SHA256：`653e9c301f160833bf364c51a879df0f1e883dc0fac23dc369a2d011c5f89198`。
新版配置 SHA256：`9e419fa77c5b6541a3ec4eaf0f2c68343a635e45ce9c348d51442e28ef72a0dc`。
本地与 Jetson **临时候选** `/tmp/njrh_reports/nav_jerk_candidate_v2_20260917/` 哈希一致。
没有覆盖旧候选、旧 bag、生产工作区文件，没有重启或运行真实运动任务。

六组当前运行 graph/参数审计：
`/tmp/njrh_reports/nav_jerk_six_group_audit_1789668538891271904.json`。
四级速度端口与 Scan 未变化，区域参数见 README。没有碰撞状态话题；日志没输出的
原因不能从速度零值反推成 StopZone。公开 Path/costmap 保留内部当周期不可见的限制。

旧报告新窗口审计（离线、不连 ROS）：
`C:/tmp/njrh_reports/nav_jerk_v2_old_capture_window_audit/marker_coverage.csv`。
19/19 个轮速窗口、19/19 个 Scan 窗口准确标为 `MISSING_IN_WINDOW`；不能补回原来未收的数据。

多话题隔离对照（非性能验收）：

- 第一次仅修调度：旧版标记窗只有 raw/smooth，新版 wheel=200、Scan=199、TF=200，
  costmap/update/footprint/path/rosout 各 50；CPU 均值旧版 73.55%、新版 72.13%。
  证据 `/tmp/njrh_reports/nav_jerk_ros_test_8_94oo58/six_group_load_comparison.json`。
- 加入索引转换优化后一次对照：旧版 73.65%、新版 71.20%；新版 wheel=201、Scan=200、
  TF=200，地图/路径/日志各 50。证据 `nav_jerk_ros_test_v2b_vmf2`；该轮字段等价测试
  发现 byte 差异失败，故不作为最终全绿验收。
- 两轮旧/新录到的数据量不同，不能拿 CPU 差直接声称相同比特吞吐提升百分比；
  不能把 71% 单核称为“很轻量”，也不与用户实录 83% 作同条件性能比较。
- cProfile 仅在隔离测试记录器上启用，结果位于 `nav_jerk_ros_test_pv3_phyl/load_candidate/`。
  接收端主要开销仍在 rclpy executor；本轮没有因此换系统库或省掉必需证据。

最终版本完整复验：`/tmp/njrh_reports/nav_jerk_ros_test_u4p2vfef`。

|V2 检查|结果|
|---|---|
|Linux 原离线回归|17/17，0.274s|
|新增 HTTP/持续缺口/标记窗口回归|3/3，0.574s|
|真实 ROS 隔离集成（含字段等价、持续竞争）|10/10，112.812s|
|Windows 回归|原回归 15 通过、2 POSIX 跳过；新增 3/3|
|py_compile / diff --check|通过|
|主体与配置临时候选 SHA256|与本地相同|

最终隔离标记窗：wheel、Scan、TF 各 200 条，四段速度各 533 条，
地图/增量/footprint/参考路径/rosout 各 50 条；TF static 保留消息也已检查。
该轮 CPU 对照旧版均值 72.68%、新版 71.74%（单核口径）。依旧不构成低开销验收。

正式 300 秒实车全量采集、HTTP 三个接口全程健康及低 CPU 目标仍未验收。
本次使用测试先行方式补上之前遗漏的持续竞争、认证、窗口完整性和字段等价条件；
隔离测试通过只证明对应采集行为，不证明导航顿挫已解决。

## 初版范围与版本（保留历史，不作为 V2 当前结果）

- 本地分支 `codex/elevator-config-management`，HEAD
  `8d01ee83994b7e83ab34cddf892b4541504a2959`；未切换分支、commit 或 push。
- 本轮只新增 `scripts/diagnostics/` 内工具、配置、测试和说明，根 README 新增入口。
  未修改生产导航、底盘、安全、BMS、定位、对桩的源码或参数。
- Jetson 正式工作区未部署这些新增文件。用于验证的副本仅放在
  `/tmp/njrh_reports/nav_jerk_test_candidate_20260917/`。
- 已核对本地和测试副本 SHA256：
  - nav_jerk_capture.py：`e0fc9d928820464c75a641d56f92d347257b7f06b646a80627a45bb22174fae2`
  - nav_jerk_capture.yaml：`00e6ff9cc603ed00a356513558a97e029e1c59089e6d5f5debf96265169c52d2`
- 未重启服务、发目标/速度/TF、调用运动或恢复服务、回放 bag、安装依赖。

## 最终测试结果

|检查|结果|范围|
|---|---|---|
|Linux 离线单元回归|17/17 通过，0.239 秒|无 ROS 图、无运动程序|
|真实 ROS 模拟集成|7/7 通过，54.968 秒|独立 net/IPC namespace、私有 /dev/shm、domain 187、loopback-only|
|已有 navigation_observer_evidence 回归|11/11 通过，0.08 秒|复用的现有诊断工具回归，未改该工具|
|Windows 离线单元回归|15 通过、2 跳过|两个 POSIX inode 轮转用例已在 Linux 通过|
|py_compile / git diff --check|通过|诊断 Python 文件；根 README 无空白错误|

最终隔离产物：`/tmp/njrh_reports/nav_jerk_ros_test_twvma5jp`。
包括 `normal/`、`interrupt/`、`disk/`、`budget_during_capture/`、
`offline_report/` 及各子测试日志。故障注入只在测试进程内替换存储或预算函数，
生产记录器没有故障注入入口。

离线回归覆盖：重映射/角色去重、多发布者、缺话题、未连接端口不得标为已核实，
Twist/TwistStamped 有符号值，平滑速度与零附近噪声，主动零速/短停再起步/断流区别，
空闲停发，重复源时间不求导，wall/ROS 时钟跳变，数据缺口断线，
中间层首次可见清零、实际 odom 变化证据（不称为因果），缺初始 costmap，
文件打标及退出瞬间未入队的标记，有界队列/丢弃，磁盘预留，日志 rename 轮转及新文件首部，
原始 QoS UNKNOWN/depth/时长不被猜测值覆盖。

ROS 集成覆盖：

1. 模拟公开链路，真实 raw subscription + rosbag2 sqlite3 写入；中间层周期清零。
   对应 bag/index 数量一致，并实际反序列化 TwistStamped 校验原始 frame/vx 和关联键。
2. CLI 文件标记后 SIGINT，只结束记录器；bag metadata、标记和 incomplete 结果保留。
3. 开始前预算不足，拒绝创建 bag，保留部分元数据。
4. 录制已有原始消息后注入预算耗尽，正常收尾已有 bag，reason=disk_budget。
5. 真实 reliable 订阅者与 best_effort 发布者不兼容，观察到 QoS 事件，收不到消息，
   明确 INCOMPLETE，不把 graph 端点数量称为已匹配数量。
6. 发布者先发一次 transient_local 消息，晚加入订阅者仍收到保留消息。
7. bag 写入后、索引写入处注入 ENOSPC，报告 writer_error，保留已写 bag；不伪造完整记录。

normal 测试刻意没有 safety mirror 发布者，因此完整性为 INCOMPLETE；这不等于存储测试失败。
最终 normal 录制约 5.019 秒，共 797 条消息，已知记录器丢弃为空、写盘错误为空；
DDS/源端丢失仍为 unknown。没有把接近 100 次的观察数误作“源端恰好发布了 100 次”。

## 测试中暴露并处理的问题

- 当前 Humble 的 rclpy Subscription 没有 `get_publisher_count()`；改为显式 unknown，
  单独保存 graph 发布者数量和实际收到消息的证据。
- 当前回调分派只向 callback 传消息，不传 MessageInfo。逐消息 GID/RMW 源时间为 null；
  graph 的 GID 保留。多发布者命令不能被分配给任意选定的一个来源。
- 额外回归修正了轮转新文件再次 tail 截断、退出瞬间标记遗漏、端口断接误报 verified。
- 集成夹具曾把发现窗口缩短为 1.5 秒，冷发现偶尔完全缺端点，磁盘中途耗尽测试无法进入
  有数据的录制阶段。保留失败证据于 `nav_jerk_ros_test_3tl31t1m`，将**测试配置**对齐工具
  默认 6 秒，并保持“必须已有真实 bag/index 数据”的断言；没有放宽生产配置或把空 bag 算通过。

## 实际负载边界

最终模拟 8 路约 20 Hz 数据的 5 秒片段：记录器约 **36.0–38.9% 单核 CPU**、
RSS **71084 KiB**，队列峰值 8 条/4672 bytes，采样时积压 0，已知丢弃为空。
这些是短时模拟负载，不是正式 80 个话题、全尺寸 Scan/costmap/TF 的资源上限。
不能称为“零开销”，也不能据此声称完整实车采集足够轻量。

当前环境 `/proc/self/io` 不存在，因此 I/O 计数保存为不可用，不填 0；RSS/CPU 仍可记录。
在正式工作区运行时优先复用现有 process counter helper，临时测试目录缺 helper 时使用
本工具的等价 `/proc/stat` 读取；这一区别在 performance 记录中明确标出。

## 实机只读 inspect

证据目录：`/tmp/njrh_reports/nav_jerk_inspect_20260917T153852Z_ewfgggsr`。
本地只读副本：`C:/tmp/njrh_reports/nav_jerk_inspect_20260917T153852Z_ewfgggsr`。
这里只做参数/graph/配置/进程身份查询和有界关键消息检查，未录制运动、未发目标。

- 发现 80 个待记录话题，速度话题去重后 7 个。完整连接、类型、发布/订阅节点、GID、
  QoS 见 `topic_map.json`；摘要见使用说明的实机映射表。
- 静止时 `/cmd_vel_nav_raw`、`/cmd_vel_nav`、`/cmd_vel_collision_checked` 无消息，输出
  INCOMPLETE；最终速度、odom、Scan 有消息。没有用生产命令去凑 READY。
- `/cmd_vel_nav` 是 velocity_smoother 与 behavior_server 多发布者；API 和对桩话题也有
  多端点。`/cmd_vel_nav_raw` 是外层 RotationShim/RangerMPPI 组合输出，不是纯 MPPI 输出。
- installed rclpy 3.3.17、Nav2 1.1.19、rosbag2 0.15.15、rmw_fastrtps_cpp 6.2.9，
  运行进程加载 Fast DDS 2.6.12；生产域 0，已查节点 use_sim_time=false。
- Jetson 源码目录没有 `.git`。部分关键源码/配置与本地同哈希，但 RotationShim CPP
  存在真实内容差异，system_status CPP 仅忽略行尾后内容相同。
  运行源码/二进制一致性仍标 unverified，没有拿本地分支替代运行版本。
- 已保留运行程序和加载库哈希；例如 controller plugin SHA256：
  `93d517862fa066c73faf0cd422ebe57935ffc7cd7fc9e453ea5933f1113b7286`。
- 收尾时 Ranger 83814、safety 84017、API 84490、controller 85441、smoother 85451、
  collision 85456、docking 86475 的 PID/启动时间未改变；未发现遗留记录器进程。

## 尚未验证/不可证明

- 没有做 300 秒正常导航、动态避障、末端运动的实车采集与负载验收。
  需用户决定采集时机，核对完整 topic_map、丢弃/积压、CPU/RSS 后评估扰动。
- 没有实测 mcap 导出、deep 大点云模式、机械臂同时重载高峰、真实磁盘设备卡死或断电。
  SIGINT 和可控 ENOSPC 不等于能在内核 I/O 永久阻塞时强制完整收尾。
- 没有在真实机器人上制造时钟跳变或 QoS 冲突。相关异常测试全部离线/隔离。
- 不测量内部 MPPI 计算耗时/有效约束，不证明控制器当周期使用的地图，
  不证明 CAN 源样本时刻，不根据接收先后推断毫秒级内部延迟或因果。
- 未验证时钟同步时源年龄为 unknown；原始 IMU 不直接当作纵向冲击。
- 当前无 CollisionMonitorState 发布者，状态/日志缺项保留；不为记录开启发布者或可视化。
- 录制开始后新建话题只记入定期 graph，不临时增加订阅；缺项列为观测缺口。

本轮采用先补回归再改实现的方式；只验证记录器行为，不宣称解决了机器人顿挫。
