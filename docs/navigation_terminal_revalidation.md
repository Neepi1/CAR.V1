# 普通导航末端验收：历史失败与当前验收分离

## 证据与范围

故障报告：`/tmp/njrh_reports/navigation_latest_20260916T1742_ktvLiH/incident_report.md`。
2026-09-16 18:07 UTC 只读核对：运行 API PID 3906310，二进制 SHA256
`1e003a085204f55d90c4261cc8c66ddb169e17304dcd1d2ab4a4187036300c78`。
对应候选源为 `undock_optional_id_20260916TdqmyUD/approved_source/robot_api_server`，
电梯执行器补丁记录在 `elevator_executor_20260916T1630_IOUxjE`。
本地还有未部署的 recovery-wait 预算改动，不能把本地树当作运行候选。

缺陷：API 横移修正失败后，同目标 Nav2 重试成功、当前位姿刷新，最终判断仍
无条件要求旧 `terminal_lateral_result.succeeded`。普通 XY/yaw 达标时重试策略
又不允许继续重试，最终产生 `degraded_final_pose_verify`。

## 修复约束

- `terminal_lateral_result` 和 `attempted` 保留原值，用作历史诊断和一次修正约束。
- 只有后续同目标重试成功，才可以用当前完整验收替代旧修正失败的否决。
- 仍检查 fresh map pose、bridge、普通 XY、严格横向目标和严格 yaw 目标。
- 停稳使用 terminal runtime 现有 odom/模式回执/稳定时间快照，不新增等待预算、
  业务阶段、订阅或运动。重试发送前重置已有稳定计时，避免复用重试前停稳证据。
- 重试失败、取消或超时不能提供成功恢复证据；重试次数/阈值/策略不放宽。
- `navigation_completion_policy.cpp::final_verify_retry` 不负责宣告任务成功，
  普通阈值已满足时不增加重试；严格条件不满足要报告当前原因。
- 现有直接成功、position-only、API 修正成功路径不增加停稳检查或等待。

## 验证边界

新增回归运行真实 executor、mission、pose/retry policy、terminal geometry，
使用隔离的真实 ROS Action 客户端/服务端提供初始 Nav2 结果，机器人 I/O
（修正、重试结果、位姿和停稳）通过既有 port 模拟。
这不是实车导航、MPPI 或真实横移测试，不能证明实车误差和运动方向。

所有 ROS 测试必须隔离 network/PID/IPC/mount、私有 `/dev/shm`、独立 DDS domain；
禁止接入实车 topic/action。测试源和构建产物仅放 `/tmp/njrh_reports`，不安装。
测试日志和具体 diff 归档：
`/tmp/njrh_reports/navigation_terminal_revalidation_20260916`。

尚需用户授权部署后，再进行有监护的真实同目标恢复验证；2026-09-16 源码修复不部署、不重启、
不移动机器人，不调整任何运动参数。

## 2026-09-16 隔离测试结果

- 先在未修复本地源码和已部署候选源码分别执行 17 项回归：均 9 通过、8 失败。
  失败包括恢复后仍被旧结果否决，以及缺失当前失败原因；正常路径通过。
- 修复后，本地 9 个测试程序共 61 项通过；部署候选加相同生产补丁，8 个
  测试程序共 55 项通过。差额是本地已有未部署的 recovery-wait 测试，不作为部署内容。
- 新增 executor 回归最终 18 项，覆盖恢复成功、XY 超限、严格横向/yaw 超限、
  未停稳、过期位姿、取消、重试 abort/timeout/canceled、次数耗尽、最后一次
  合法重试成功、bridge 超时/取消和三条原正常路径。
- 现有停稳证据通道另测无数据、旧样本、运动样本、过期样本，以及配置关闭停稳
  检查的原行为。测试显式使用非零稳定时长；生产停稳参数不变。
- 两版执行模块/terminal runtime 源码契约检查通过；`git diff --check` 通过。
- 两版 executor 的 18 项回归各重复 5 轮全部通过；本地生产修改与 Jetson 隔离
  测试源逐文件哈希一致。
- 18:48 UTC 实车 PID 3906310、原候选二进制 SHA256、service NRestarts=0 均未变。

证据以报告目录的 `final_local_tests.log`、`deployed_green_tests.log` 和两版
`green_build/test_results` 的 gtest XML 为准。测试使用全新隔离构建目录编译相关
导航源文件，不借用旧 API `.o`。没有构建或安装新的完整生产 API 候选；内部
虚接口改变后，未来授权部署仍须遵守 coherent-build 规则。

## 2026-09-17 生产候选与部署准备

本次用户授权部署，随后明确授权重启。已同步修复源码、安装通过测试的完整候选并
完成一次规定的整链重启。新版身份和静态就绪检查通过，没有实车运动验收。

- 重新核对实际运行二进制 `1e003a085204f55d90c4261cc8c66ddb169e17304dcd1d2ab4a4187036300c78`，
  基线取 `elevator_executor_20260916T1630_IOUxjE/final_source`，而非本地/main。
- 基线只叠加 6 个生产文件、4 个测试文件和测试目标声明；未发布本地 recovery-wait 改动。
- 全新目录完整构建 API 及其自有库，117 个生产对象依赖审计通过，无外部 API
  头文件混用、无缺失动态库，未借用旧对象。构建约 1804 秒。
- 候选 SHA256：`1e02bead73ab728577d26ee2d2f45f71c66cb3588d2e98b948e43b1198ddcbaa`。
- 12 个 gtest 程序共 90 项、2 个源码契约测试通过；执行器 18 项另重复 5 轮通过。
  完整 API 隔离只读并发查询 1202 次通过，最慢 83.111 ms，运动请求 0。
- Jetson 工作区 6 个生产文件和 4 个测试文件与候选哈希一致；CMake 仅补测试目标，
  保留其原有其他改动。同步前文件备份保存在报告目录 `workspace_before`。
- 本地两个 mission cpp 及相关测试仍保留原未部署 recovery-wait 差异，未被候选覆盖。
- 10:46 UTC 复查：API PID 22205 唯一、仍为原二进制；service active/running，
  NRestarts=0，API healthy=true，navigation_active=true，导航任务 idle。

报告与精确 diff：`/tmp/njrh_reports/navigation_terminal_deploy_20260917THkL9Bn`。
`candidate_verified/candidate.json` 是仅构建阶段记录；后续通过结果见 `test_summary.json`，
不能把其中初始 `tests_passed=false` 误读为回归失败。
首次配置发现新测试误链接 `robot_safety` 的可执行导出目标，仅改测试 include/依赖声明后
在新目录重建通过；结果汇总脚本的 XML 目录也已纠正，均未改生产行为。

10:53:50 UTC 执行规定的整链重启；旧链停止后，通过临时 ExecStartPre 安装候选，
10:54:07 UTC 安装完成且新服务启动。临时配置已核对后移除并 daemon-reload，未改变
既有重启策略、未单独重启节点。本轮未重新编译。
10:55:21 UTC 只读检查：唯一 API PID=57284，运行与安装哈希均为上述新候选；
navigation_active=true、healthy=true、AMCL_READY、safe_for_goal_start=true，导航任务 idle。
service MainPID=55480、active/running、NRestarts=0。旧二进制保存在报告目录
`api_before_activation`，当前状态记录为 `deployment_state.json`。
真实同目标恢复、严格横向精度、传感器时序与首次修正发散原因仍需实车验证。
