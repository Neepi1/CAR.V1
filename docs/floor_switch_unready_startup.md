# 定位未就绪时的切图修复（2026-09-09）

## 问题和范围

现场失败码为 `NAV_IDLE_UNPROVEN`（31）：初始定位没有成功，Nav2 尚未
激活，`bt_navigator=unconfigured` 且 action status 发布者为 0。旧准入
逻辑必须看到 action 状态，因此在加载新地图之前就拒绝。

只删掉该检查不够：旧图健康检查会继续阻挡 BEGIN；启动流程又等待普通
导航许可，而切图中的目标必须到最后 COMMIT 才有普通导航许可，形成相互等待。

本轮修改针对这条未定位启动路径；不修改雷达、scan、TF 架构、定位算法、
Nav2 控制参数或运动速度。正常已激活导航的切图仍走原路径。

## 修改

1. 同时查询 bt_navigator/controller_server 的真实 GetState。只有服务 owner
   唯一、两者明确 unconfigured/inactive、响应新鲜且没有活动目标矛盾，才可替代
   action idle 证据。状态未知、缺失、超时不放行。
2. 保留安全停车与双里程计静止证明。已有活动导航先取消，取消响应本身不等于
   导航已结束，仍需 action 的终态/idle 证据。
3. 健康旧图只作为失败回退证据，不再作为初始化精确新图的必要条件。
4. 冷启动通过一对事务 request/ack 文件交接当前目标；旧启动副作用必须正常
   结束或已有确定结果，不能杀客户端后假设请求已撤销。目标身份包括 building、
   floor、map、epoch、`sha256:` digest 及本次 nonce。
5. 新目标顺序为 BEGIN → 地图/掩码 → localizer 重载 → 显式定位 → 目标 pending
   TF → Nav2/AMCL → 双代价地图刷新 → COMMIT → 复核静止/idle → 解除本事务停车。
   pending TF 只用于启动初始化，不能作为普通导航许可。
6. 取消立即将交接标为 failed；已发请求继续按既有结果/未决规则处理，后续
   lifecycle、AMCL seed/no-motion 等步骤不得继续发送。异常路径也根据真实清理
   结果报告是否需要恢复，不再仅看 BEGIN 是否收到响应。
7. 旧启动进程不再回写目标 context；后续再次热切图后也不得使用旧图参数重新
   初始化 AMCL。完整重启的新启动实例忽略历史终态文件，不把历史记录变成新锁。

## 参数与截止

- `startup_handoff_timeout_sec=90`：等待启动 owner 接管的阶段上限。
- `startup_ready_timeout_sec=120`：等待目标 Nav2/AMCL 初始化的阶段上限。
- 两者不是保证等待时长，更不会延长调用方的总截止。现有 API 120 秒总截止
  保持不变；到期取消后不能继续启动后续步骤。单阶段和总截止同时存在，先到者生效。
- 非默认自定义 lifecycle/AMCL helper 不在本轮验证范围。

## 验证与部署状态

本轮先修改工作区，并在 Jetson `/tmp/njrh-floor-cleanup-test.nslY7C/candidate`
编译、测试；ROS 集成测试强制 domain 213、本机通信，不接入生产图，不发真实运动。

- evidence tracker：双生命周期、过期/未知状态、活动目标冲突。
- writer：原子写、越界路径、错误 schema/version、旧 nonce、旧序列与错误目标。
- Python/脚本：真实 C++ 序列化互通、typed pending TF、AMCL/COMMIT 交接、取消和
  后续热切图后旧 owner 的权限失效。
- 真实 FloorSwitch action 集成：冷启动无健康旧图、正常热路径、unknown 拒绝、
  错误目标 ACK 拒绝、完整目标初始化与 COMMIT 后解除 hold。

本轮已运行结果：

- 真实 FloorSwitch action 集成 **5/5 通过**，包括完整成功路径；外围 ROS 服务
  与启动回执为隔离模拟，不能替代真实 Isaac/AMCL/车体硬件验收。
- 启动交接与 lifecycle Python/脚本测试 **37/37 通过，无跳过**；包括在 Jetson
  编译真实 C++ writer 后由 Python 读取请求的跨语言测试。
- 事务 core/executor/evidence/reconciliation/cleanup、writer、graph 及清理接线
  共 **8 组 CTest 全通过**。
- 修改的 shell 语法检查及 `git diff --check` 通过。
- 最终候选 `floor_manager_node` 编译通过，SHA-256：
  `d21af096e65c94c1056d9035762d6be30b9f3ea50a2990824105a4286c3ca4fe`。

**已定向部署；现场重启未通过完整就绪验收。**

2026-09-09 17:12:13 UTC（北京时间 9 月 10 日 01:12:13）补齐启动依赖后
重新启动 `njrh-runtime.service`。部署清单共 35 个文件，SHA-256 全部核对通过；
8 个运行脚本权限为 0755，动态库依赖检查通过。先前漏传的
`navigation_startup_receipt.sh` 已补齐，递归检查另 32 个静态脚本依赖均存在。

现场随后连续在 `/dock/target_observation` 新鲜消息检查失败；现有 common
启动器报告 `common startup failed: docking_sensor` 后关闭整条运行链。
17:14:25 UTC 已停止自动重启，`MainPID=0`、服务状态为 failed，不可表述为
导航恢复正常。没有发送运动、切图或手动重定位命令，也没有解除停车保护。
实际切图/定位验收仍未完成。

后续更新：17:21:17 UTC 已部署对桩观测失败隔离的最小修复并再次启动。
17:21:38 收到新鲜对桩观测，整链不再因该检查退出；运行中 floor-manager
指纹也已与候选核对一致。详见 `docking_startup_failure_isolation.md`。
这不替代真实切图及定位验收。

车端部署备份：
`/home/nvidia/workspaces/njrh-v3/workspace1/reports/floor_startup_deploy_20260909_8mdLNN`。
本地执行记录：`reports/floor_startup_deploy_20260909/result.md`。

## 仍需硬件验收及已知限制

对桩传感器启动阻断已修复；仍需由现场验证初始定位失败后切到正确地图、
定位/AMCL/代价地图一致、无旧图回写，以及取消时不发生运动。

本轮没有声称解决所有切图失败后的自动恢复：BEGIN 后若没有有效旧图可回退，
或目标请求结果未知/目标定位失败，既有恢复保护仍可能保留。不能把这种情况当作
普通可重试成功，更不能直接清掉停车状态。受控恢复需另行验证旧请求已终结、
精确目标定位与代价地图，并由同一事务所有者完成；本轮不实现无条件清锁。
