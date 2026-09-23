# 轻量导航事件日志记录器

## 1.4 部署入口：原消息与内部快照

新功能需要载入带内部快照的 MPPI 库；仅部署文件、尚未完成授权的整链重启时，
运行中的旧插件不会生成内部帧。脚本会报缺项，不会自动重启或启动导航。
默认不加 `--motion` 仍是原来的日志模式；下面的环境变量仅作用于记录器进程，
匹配本次核对的车端 Humble / Fast DDS / ROS 域 0。

重启生效后，SSH 中使用已部署的默认 helper 路径：

```bash
docker exec -it NJRH-car bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/njrh-v3/workspace1/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
  cd /workspaces/njrh-v3/workspace1
  python3 -B scripts/diagnostics/nav_event_lite.py record --motion \
    --log-dir scripts/jetson/runtime_overlay/web_dashboard/runtime_logs \
    --live-params --duration 300
'
```

原消息 helper：`scripts/diagnostics/native/build/navlite_raw_capture`。
离线同模型重演 helper：`scripts/diagnostics/native/build/navlite_mppi_replay`。
按回车打标，最后一次标记后至少再留 5 秒。缺项见 `evidence_quality.json`；
日志完整不代表运动证据完整。CPU实测、边界和候选验证见
[验证记录](validation_nav_event_evidence.md)。此次部署不更改运动、安全或地图参数。

## 原日志模式与历史说明

从用户提供的 `nav_event_lite.py` 补充而来，保留原下载文件不改。
默认日志模式只读取现有本地日志；无 ROS/DDS、HTTP、子进程、运动/定位/恢复请求。
不替代 `nav_jerk_capture.py` 的原始速度、轮速、Scan、TF、路径和地图记录。

## 本次补充

- 1.3（2026-09-20已部署启用）：接通生产模块的离散事件；`report` 补充底盘收到的
  输入、SDK提交缓存、模式和轮式里程计缓存。一次性 `diagnostics_ready` 仅标识
  模块带有诊断代码，不证明导航就绪或运动成功。录制热路径不变。
- 1.2：补采旧版 `Ordinary MPPI recovered a valid control`，仅证明原文出现，
  没有返回速度时不推断非零恢复。
- 1.2：`report` 流式导出 `navlite.csv`，解码已存在的 MPPI/外层控制器、
  碰撞监控、安全输出/仲裁/状态日志，逐标记列出证据覆盖与缺项。
  解析只在离线报告运行，录制不逐帧解析，不增加 ROS/HTTP/进程轮询。
- 区分碰撞节点普通日志与明确的 `Robot to stop/slowdown/approach/continue` 动作文案；
  只有明确文案计入动作数，不把生命周期配置日志当停车。
- 增加普通恢复、局部修路、下划线形式模式切换关键词。
- 缺文件、轮转缺口、截断/超长行、退出时积压、磁盘不足等输出 `INCOMPLETE` 和原因。
- 独立离线 `report` 导出全局日志 CSV、人工标记前后默认各 5 秒 CSV 和摘要。
- 默认记录 300 秒，自动创建唯一 `/tmp/njrh_reports/nav_event_lite_*` 目录；旧证据不覆盖。
- 检测读取时钟跳变；人工标记要求同机、同次系统启动的时钟，避免跨主机混用 monotonic。

保持默认每 0.2 秒轮询、每文件每轮最多 16 KiB、最多 8 文件、单行上限 8 KiB。
事件文件默认上限 16 MiB，标记文件约 1 MiB；磁盘检查两秒一次，预留 128 MiB。
CPU/RSS/积压每两秒统计一次。分析只在退出采集后运行，不在 record 时绘图、排序大数据或压缩。
没有零开销保证；观察每份 summary.json 中单核口径 CPU 和输入积压。

## SSH 使用（Linux / Python >=3.8）

先进入记录，再通过 App 正常执行测试；默认跳过脚本启动前已有的日志历史。

```bash
docker exec -it NJRH-car bash -lc '
  cd /workspaces/njrh-v3/workspace1
  python3 -B scripts/diagnostics/nav_event_lite.py record \
    --log-dir scripts/jetson/runtime_overlay/web_dashboard/runtime_logs
'
```

无需 `source` ROS 环境，不创建 ROS participant。按回车打标；也可输入短标签后回车。
Ctrl+C 只结束本记录器；如需完整 +5 秒窗口，在最后一次打标至少 5 秒后结束。
输出目录自动唯一；可用 `--duration 600`、`--output /tmp/njrh_reports/<新目录>` 自选。
非交互调用加 `--no-stdin`；第二个 SSH 终端打标：

```bash
docker exec NJRH-car python3 -B \
  /workspaces/njrh-v3/workspace1/scripts/diagnostics/nav_event_lite.py mark \
  --output /tmp/njrh_reports/<实际记录目录> --label '绕到障碍侧面顿挫'
```

记录结束后生成报告（不会发 ROS 消息，不回放任何内容）：

```bash
docker exec NJRH-car python3 -B \
  /workspaces/njrh-v3/workspace1/scripts/diagnostics/nav_event_lite.py report \
  --output /tmp/njrh_reports/<实际记录目录> --window 5
```

`events.jsonl` 和 `marks.jsonl` 保持原样；每次 report 新建 `report_*`，不覆盖上次报告。
`timeline.csv` 是所保留事件的读取时间线，`marker_windows.csv` 按标记给出前后事件。
`event_line` 指向原始 `events.jsonl` 行号，source/inode/generation/offset 指回源日志。
source_stamp_ns 单独保留，未证明源时钟同步，不用于毫秒级因果推断。

### NAVLITE 离线证据（1.3）

`navlite.csv` 保留错误原文、异常类型/阶段、失败连续次数/持续时间、动作/区域、
选中来源、原因、有符号输入/输出以及完整 `fields_json`；原始日志仍在 `events.jsonl`。
`output_status` 区分：

- `returned_zero/nonzero`：计算返回；外层 `controller` 不冒充纯 MPPI。
- `published_zero/nonzero`：生产日志声称发布的样本，不证明底盘实际运动。
- `no_command_returned`：本次 MPPI 异常无返回，不等于发送零命令。
- `no_message`：碰撞监控未发布；若来自 `safety_arbitration`，仅代表该输入
  未被选中，不是整个安全节点停发。
- `state_only`：状态变更，不是发布了速度。
- `driver_observation`：底盘现有循环的离散观察。`sdk_*` 是最近一次SDK调用参数，
  **不是CAN发送确认**；`actual_*` 是wheel odom缓存，不是独立且已证明新鲜的测量。
- `diagnostics_ready_not_motion`：模块诊断版本标识，不算输出或行驶证据。
- `unknown`：缺字段、未知格式、坏格式或非有限输出，保留原文，不猜测。

`requested_out` 与实际日志输出分列。碰撞监控 `published=0` 时实际输出列为空，
不能把期望零速算作发布零速；`DO_NOTHING` 放行零速也不算恢复非零。
`input_known=0` 或 NaN/Inf 不转换成零。MPPI `elapsed_sec` 是失败段持续时间，
不是内部计算耗时。`source=0/1/2` 在已核对 Safety 源码中是普通/API/对桩，
CSV 保留原值（-1 未知），不猜未来版本映射。

报告 `evidence_coverage` 给出各类事件数和首末证据行号，每个人工标记另有窗口覆盖。
**只表示看到了某类离散证据，不保证完整覆盖一次停车；不自动推断故障或因果。**
`unobserved_evidence` 与日志采集 `incomplete` 分开：即使日志采集完整，生产没写
关键事件也会缺证据。旧 `collision_action_counts` 只计旧动作文案，新格式查
`navlite.csv`/`evidence_coverage`，不能拿旧计数 0 证明没有停车。

2026-09-20 核验的生产 controller/collision 仍为 WARN，经授权部署并整链重启后，
MPPI/外层控制器/碰撞/安全/底盘五个模块的诊断启动标记均已落到默认日志文件。
使用 WARN 写变化事件、同类持续异常最多1Hz；不能恢复历史未写出的数据。
没有连续轮速、Scan/TF/地图及分段计算耗时，
相关原因仍需额外证据。记录器自带 SHA256 留在 summary，不能证明源码和运行二进制一致。

退出 0 表示正常结束，不代表证据完整；还要看 `incomplete`、`incomplete_reasons` 和
逐标记的 `window_clipped`。人为提前结束时，标记窗口可不足，脚本不会因此阻止导航。
如果系统强杀/掉电导致 summary 停留 recording，报告拒绝把它当完整采集；保留原始文件。

## 当前日志映射与盲区

2026-09-18 09:59:58 UTC 实机 fd/argv 核对：

| 默认文件 | 实际内容 |
|---|---|
| resident_navigation_runtime.log | 导航启动脚本和 ros2 launch 汇集的 controller/collision/Nav2 日志 |
| robot_safety_common.log | Safety stdout/stderr |
| ranger_chassis_common.log | Ranger stdout/stderr |
| robot_api_server.log | API stdout/stderr |

实机 controller_server/collision_monitor 当时均为 `--log-level warn`。
INFO 被抑制时无法读到停车/恢复文案；脚本不改变生产日志级别，也不声称能证明没有停车。
路径映射是核对快照，不保证未来其他启动分支相同；可用重复 `--log <文件>` 明确选择文件。
模式/动作是日志文本线索，不是底盘或碰撞状态的独立测量。

只保留关键词命中内容，未命中 INFO 不保存。控制超周期日志每秒聚合，仅保存首/末样本和计数。
有界读取可能有积压；轮转后的旧 inode 晚写、快速 copytruncate 无法保证完整获取。
旧生产没有速度/轮速事件，不能据此确定谁先清零；诊断候选启用后可读取离散输入输出及
反馈缓存，仍不是连续原始消息。没有 Scan/TF/轮廓，不能画障碍或推断 StopZone 命中
几何。HTTP 没有失败是因为没有执行 HTTP 查询。
离线报告按**日志读取时间**划窗；写日志缓冲和人工反应延迟可能让真实事件落在窗口外。
旧版标记无 boot_id 时，不冒充已验证同域时间；报告将其列为缺口并排除自动关联。

## 假日志回归

```bash
python3 -B scripts/diagnostics/test_nav_event_lite.py
```

测试只启动记录器子进程，输入为临时假日志，无真实底盘/ROS 接口；不会读取正式日志。
覆盖分类、标记/离线报告、缺文件、Ctrl+C、轮转、截断长行、预算/余盘、重复输出、
重复/回退源时间、标记时钟不匹配。测试脚本的 subprocess 仅用于隔离测试，不在记录器内。
真实长时导航下的负载和日志覆盖仍需用户采集验证，假日志通过不代表实车验收。

2026-09-18：在 Jetson 已有 Python 3.10.12 上，临时假日志目录内 **14/14** 通过。
四文件、约 200 行/秒的约 6.4 秒短测：记录器平均单核 CPU 1.883%，两秒采样峰值
2.374%，max RSS 14020 KiB，事件文件 359490 bytes，观察积压 0。
该数字不包含测试日志生产进程；不代表所有实车日志负载下都保持相同占用。
时钟跳变测试只替换测试进程中的取时函数，不调整 Jetson 系统时间。
完整测试输出：`/tmp/njrh_reports/nav_event_lite_validation_fmhlQCaR/regression_final.log`。

2026-09-20 1.2：Jetson 假日志回归 **20/20** 通过（27.770 秒）。约 200 行/秒、
四文件的 6.413 秒短测：平均单核 CPU 1.518%，两秒采样峰值 1.976%，
max RSS 15844 KiB，读取积压 0。未运行真实导航采集，不代表实车验收或零开销。
新覆盖包括旧恢复漏采、原异常转义、计算零/非零恢复、停发与零发布、状态解除、
缺事件、坏格式、NaN/Inf；录制间隔/读取预算保持不变。
测试证据目录：`/tmp/njrh_reports/nav_lite_script_20260920_pMVlQj`。

2026-09-20 1.3：假日志 **21/21** 通过；四文件短测平均单核CPU 1.299%、
两秒峰值1.582%、RSS 16616 KiB、积压0。该数据不含生产模块日志开销。
真实MPPI + 合成地图 **10/10**、路径修补运行时 **8/8** 通过，真实失败/恢复日志
已经过文件采集和CSV解析验证；不是只构造同名异常或手写日志。
测试程序 `test_navlite_mppi_evidence.py` 仅允许私有网络/IPC/挂载命名空间、
独立SHM和ROS域181；不要在实车ROS域启动隔离测试程序。
跨模块集成及仍未验证的边界见 [证据闭环](../../docs/navlite_chain_evidence.md)。
# 1.4: raw evidence + exact MPPI inputs

The previous file-only profile cannot explain why a trajectory approached an obstacle.
`record --motion` now keeps the existing lightweight log tail and adds **one native C++
serialized recorder**, plus an explicitly requested bounded MPPI file snapshot session.
There is no Python ROS executor, HTTP polling, online plotting, point cloud, video,
parameter change, goal/cancel request or robot command. Default `record` stays log-only.

This is diagnosis, not a navigation behavior change. The MPPI plugin candidate must
be loaded by an authorized whole-chain restart before internal frames can appear. Do not mistake a
missing plugin for a successful full recording; `EVIDENCE_INCOMPLETE` names the gap.
No automatic deployment/restart is performed. Source/binary equivalence is marked
unverified unless separately proven by the deployment record.

## Build only the diagnostic helper

Use the existing Humble environment inside NJRH-car (no dependency installation):

```bash
source /opt/ros/humble/setup.bash
source /workspaces/njrh-v3/workspace1/install/setup.bash
cd /workspaces/njrh-v3/workspace1
cmake -S scripts/diagnostics/native -B /tmp/njrh_reports/navlite_native_build
cmake --build /tmp/njrh_reports/navlite_native_build --target navlite_raw_capture -j1
# Optional offline same-model replay (uses only the installed dependencies):
cmake -S scripts/diagnostics/native -B /tmp/njrh_reports/navlite_native_build -DNAVLITE_BUILD_MPPI_REPLAY=ON
cmake --build /tmp/njrh_reports/navlite_native_build --target navlite_mppi_replay -j1
```

Build/deploy of the MPPI library is separate and incremental; do not build the entire API.
The native helper build does not change installed production libraries. Its isolated
test wrapper creates private network, IPC and SHM, and does not use the robot's ROS domain.

## One recording entry point (after explicitly authorized activation)

```bash
python3 -B scripts/diagnostics/nav_event_lite.py record --motion \
  --native-helper /tmp/njrh_reports/navlite_native_build/navlite_raw_capture \
  --log-dir scripts/jetson/runtime_overlay/web_dashboard/runtime_logs \
  --live-params --duration 300 --motion-max-mb 512 --mppi-max-mb 256
```

`--live-params` issues a bounded read-only dump (at most 4 seconds per selected process)
before capture; failures are explicit. Without it, parameter provenance is only the
running launch files/command overrides, not proof of current dynamic parameters.
Bindings are resolved from running process remaps, then native graph/QoS/type checks;
same-topic roles are subscribed once. Unknown/missing publishers are not invented.
Initial discovery and a motionless controller may show incomplete: MPPI frames do
not exist until the user normally runs a navigation task. The recorder never starts one.

During recording Enter adds a mark; from another terminal:

```bash
python3 -B scripts/diagnostics/nav_event_lite.py mark --output /tmp/njrh_reports/nav_event_lite_... --label sidepass_stop
```

Mark uses only a local file. Keep recording for at least 5 seconds after the last mark.
Ctrl+C stops only the recorder and removes its own diagnostic request, not navigation.
Output directories must be new; old evidence is never overwritten or deleted.

If the native helper exits early, the parent closes this recording and preserves
partial evidence; it never stops the robot. Use the same sourced Humble environment
for recording and optional replay so the installed ROS shared libraries are visible.

`LOG_CAPTURE_COMPLETE` describes only text. `EVIDENCE_READY` means required streams
and internal frames have appeared, **not** complete ±5-second coverage or proven causality.
Final offline report checks each marker. Exit 3 denotes incomplete evidence; 2 denotes
an initialization/I/O failure. Partial bags and original indices are retained.

## Offline export and same-model replay

```bash
python3 -B scripts/diagnostics/nav_event_lite.py report \
  --output /tmp/njrh_reports/nav_event_lite_... --window 5 \
  --replay-helper /tmp/njrh_reports/navlite_native_build/navlite_mppi_replay
```

The replay helper reads final pre-shift sequences offline and uses the commissioned
Ranger response model, not a newly invented kinematic model. It creates prediction
CSV; this is not real future motion, a bag replay, or a ROS publication.

Outputs include:

- `runtime_start/`: running paths/hashes, launch parameters, bounded live reads,
  versions, branch/SHA/workspace status when available. No API token is collected.
- `topic_manifest.json`: resolved role bindings; `motion/topic_map.json`: actual
  graph endpoints/GIDs/QoS/type and missing items.
- `motion/bag/`: original serialized messages; `receive_index.csv`: unique storage
  timestamp + topic association, receive monotonic/wall/ROS time, GID and RMW timestamps.
  The unique storage timestamp is an **index**, not the sensor's source time.
- `motion/quality.json`, `health.jsonl`: counts, known recorder drops, intervals,
  CPU/RSS/queue/clock changes. DDS/source loss is unknown, never asserted zero.
- `mppi_*/`: true locked master costmap, actual transformed reference, pose/Twist,
  prediction history, constraints, final sequence/offset, command/error and compute ID.
  See [binary schema](native_mppi_snapshot_schema.md).
- `report_*/`: original log CSVs plus offline raw decoded CSV and marker coverage;
  no interpolation across missing intervals, no inference of millisecond causality
  from arrival order.

Public costmap publication is not the controller's snapshot. Collision-monitor
logs, raw scan and TF allow geometric comparison, but do not by themselves prove
which cached scan/transform was consumed internally in a particular callback.
There is no guarantee every possible root cause can be proven from one recording;
missing evidence is exposed rather than replaced with a guess.

Diagnostic buffers and budgets do not block control: full queues lose **diagnostic
frames**, record the loss and invalidate complete reconstruction. They never hold
the robot, reuse an old command or change safety. Defaults are bounded; costs must
be measured, not called zero. Validation results are in
[candidate validation](validation_nav_event_evidence.md).

---
