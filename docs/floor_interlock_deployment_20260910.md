# API 楼层当前状态互锁部署

## 范围

本次只部署本地已有的 `FloorRuntimeInterlock` 新版，未进一步修改业务算法。

- 历史 `FAILED_LOCKED` 不再永久覆盖后续当前状态。
- 当前活动切图仍互斥；其他请求的拒绝不能清除活动事务。
- 当前地图上下文无效仍拒绝导航运动；已列明的源图无关恢复操作可进入自身校验。
- **离桩的 invalid-map 准入未在这次放宽。** BMS 记忆锁来源、接触停车故障恢复、
  离桩方向验收，以及其他 floor-manager/bridge/elevator 候选均未包含。
- 未更改机械臂黑盒、导航参数、碰撞策略、建图、TF、DDS 或传感器配置。

## 构建及部署证据

以当前已安装 API 为基线，原始 link 输入重新链接并按原安装过程去除 build RPATH，
与生产产物逐字节一致。只替换 `floor_runtime_interlock.cpp.o`，以及为新头文件重新编译
的原基线 `floor_switch_module.cpp.o`；后者没有带入本地其他未部署源码改动。
其余对象和依赖保持原基线。不使用较早的完整 API 候选覆盖当前已修复的重定位等功能。

| 对象 | SHA256 |
|---|---|
| 原运行 API | b7f9e5c51332bd56d761338ee86d941aa5e34286698da444934e728637774b06 |
| 新 API | 69028718894a941e651e25ca0c903770c79ad5b6cc91b6938433e2cecf730312 |
| interlock.cpp，本地/Jetson 一致 | 71a6c0682b9d2621bcd33ec4a83cfc4dffda7a838a305003b57b87854155c8ea |
| interlock.hpp，本地/Jetson 一致 | 1fd8902f3b7a74e1a3967506067a77b6152524e9483c2115c46fc2e101ce0803 |

旧二进制、两个源文件、测试及安装头文件备份：
`/tmp/njrh_reports/floor_interlock_deploy_20260910/pre_deploy/`。

## 验证

- 原逻辑的“事务失败后当前健康”和“健康由失败恢复”两项用例均复现失败；新版 25/25 单测通过。
- 实际候选 API 在独立 network + IPC namespace、ROS domain 229、私有临时地图及端口下，
  6 项 HTTP 断言通过：活动互斥、其他事务不清锁、失败后当前健康不锁定、当前 invalid
  拒绝导航、invalid 源图允许切图参数校验、恢复健康后准入恢复。没有发送有效运动目标。
- 依赖检查通过，候选运行时没有 `/tmp` RPATH。
- 16:33:15 UTC 使用唯一允许的完整 `njrh-runtime.service` restart；16:33:18 service active。
- 16:34:17 UTC：新 API PID 3974650，运行镜像哈希等于候选，HTTP 正常；
  `/api/v1/status.floor_runtime_interlock.detail` 为新版当前状态文案，blocked=false。
- 地图保留 B10/F1 `map_20260602T205005Z_12995ed9ed`；未手动切图或重定位。
  完整启动脚本按已有流程执行初始定位，不是另发定位探针。

- 16:37:01 UTC 最终核验：runtime ready，B10/F1 身份一致；日志 `nav2_layer_ready`
  为启动阶段 63s。API、planner/controller/BT、速度平滑/碰撞监控、安全、对桩管理器及
  深度观察节点各唯一；service active，NRestarts=0。常驻健康观察器报告局部 odom 与
  对桩观察各一个发布者，年龄约 0.067s/0.074s；336L sensor_healthy=true，但当前
  目标 valid=false / insufficient_points，不宣称已识别充电桩或完成实车对桩。

完整运行验收见 `/tmp/njrh_reports/floor_interlock_deploy_20260910/summary.md`。
真实切图失败后恢复、实车离桩仍需用户监督验证；本次没有移动小车。
