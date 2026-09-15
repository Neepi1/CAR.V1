# 门控审计：逐项修复记录与第一项方案

记录日期：2026-09-09。

依据：[项目门控台账](project_gate_inventory.md)及当前工作区源码审查。结论不是现场故障复现，也不是 Jetson 已部署版本的证明。工作区存在其他未提交修改，实施时必须保留，不能夹带部署。

## 用户确定的工作方式

- 依次处理八项审计发现，先处理第一项。
- 后续用户授权“就这个切图，怎么简单怎么来”：第一项已进入离线候选实现；不扩大到其他七项，不部署或重启现场。
- 每项实施先说明范围，再建立能捕获真实冲突的回归测试，然后修改、验证、记录结果。
- 不把所有门控删除，不用更长超时或重启代替故障恢复契约。

## 审核问题队列

| 顺序 | 问题 | 当前状态 |
| --- | --- | --- |
| 1 | 切图失败时，floor-manager 期待 ABORT 恢复源图，但 bridge 普通 ABORT 执行失败锁定 | 已完成离线验证、floor-manager 定向部署和完整服务重启；实际切图故障/重试验收待进行 |
| 2 | 精对桩残留暂停清理使用的业务原因，与 bridge 实际输出的状态变化原因不一致 | 待处理 |
| 3 | 部分超时请求留下全局未决副作用计数，没有对应迟到结果结案路径 | 待处理 |
| 4 | 预对桩充电快捷成功路径与普通交接，对取消完成及实际停稳的成功要求不同 | 待处理 |
| 5 | 最终 bridge 校验的约 2 秒外层等待，包含允许约 8 秒的同步服务调用 | 待处理 |
| 6 | ContactStopping 可持续等待停车证明，但停止服务已返回“已停止”，离桩仍拒绝 | 待处理 |
| 7 | 预对桩正常、Nav2 失败、总超时三条路径的控制权交接规则不一致 | 待处理 |
| 8 | 电梯测试碰撞旁路与专用控制器净空检查关闭叠加，存在商用实时障碍保护缺口 | 待处理，现场安全措施不应因排队而延后 |

第 7 项须保留用户此前要求：正常预对桩 XY、yaw 都应达到约定条件，不能未经确认直接放宽角度。第 8 项源自此前测试放宽要求；任务权限、急停和看门狗不能替代实时障碍保护。

## 第一项：修改前的问题

1. BEGIN 只使原运行上下文失效、进入切图事务，不代表 Nav2 地图或定位资产已经被修改。
2. BEGIN 成功后，即使仅 pending-context 写盘失败、尚未加载目标地图，失败清理仍按 BEGIN 已成功选择普通 OP_ABORT。
3. floor-manager 的清理函数随后要求 bridge 返回 runtime_context_valid=true，才承认源图恢复。
4. bridge 普通 OP_ABORT 的明确契约却是 runtime_context_valid=false、failed_locked=true。
5. 两端后置条件相反，使一部分本可验证恢复源图的早期失败升级成失败锁定。

证据入口：

- [BEGIN 后写盘失败](../src/robot_floor_manager/src/floor_manager_node.cpp:2533)
- [按 BEGIN 选择 ABORT 类型](../src/robot_floor_manager/src/floor_manager_node.cpp:3003)
- [上游要求源上下文有效](../src/robot_floor_manager/src/floor_manager_node.cpp:1690)
- [普通 ABORT 保留失败锁](../src/robot_localization_bridge/src/floor_transition_context.cpp:244)
- [现有未变更恢复操作](../src/robot_localization_bridge/src/floor_transition_context.cpp:281)

不是所有切图失败都能自动恢复：源证据不充分、目标资产可能已改变、旧请求可能迟到执行时，保持停车是合理的。也不能由这项静态审查断言过去所有失败均由此造成。

## 第一项：采用的最小修改方案

### 1. 区分“事务已开始”与“目标副作用可能已发出”

在 floor-manager 现有事务 adapter 内维护事务级、单调的目标副作用提交记录。它只记录事实，不增加一个新的独立锁。

- 首个目标地图变更请求是 [load_map_with_client 的 async_send_request](../src/robot_floor_manager/src/floor_manager_node.cpp:1984)。必须在调用发送函数之前记录“已发出或可能发出”，不能等成功响应再记。
- 覆盖 NavMap、filters、localizer apply、显式定位等目标副作用入口，避免后续调整执行顺序时遗漏。
- 超时、取消、异常或失败响应，都不能自动把记录重置为“从未发出”。新事务才初始化；重启后的未知状态不能推定为从未发出。
- 不直接复用 core 的 mutation_started_ 判断资产修改：它现阶段也代表源 runtime-context 已失效，语义不同。

### 2. 分开两个操作的结果约定

| 已知事实 | 失败处理 | 允许的最终结果 |
| --- | --- | --- |
| BEGIN 未生效且无未决 BEGIN，没有目标副作用 | 现有精确租约清理 | 清理证明成功后，以失败结束 |
| BEGIN 已生效或结果未知，但能证明没有发出目标副作用，且源状态可证明 | 现有 OP_ABORT_PREMUTATION，使用更高命令序号隔离迟到 BEGIN | 源上下文恢复并完成清理后，以可恢复失败结束 |
| 已发出目标副作用，或是否发出无法确认 | 普通 OP_ABORT 与现有安全资源保留路径 | 失败锁定，不宣称源图已恢复 |

普通 OP_ABORT 的“成功”只表示已接受终止并保持失败保护，不能再交给要求 runtime_context_valid=true 的源恢复判断。保留 bridge 普通 ABORT 的安全语义，不把它改成无条件解锁。

### 3. 源恢复要完整证明，再释放本事务资源

按顺序确认：精确事务及更高命令序号的 OP_ABORT_PREMUTATION 成功；收到响应之后的新鲜源身份/定位/TF 健康证据；源运行上下文成功持久化；本事务 correction pause 释放被证明；最后释放本事务 motion hold 并验证结果。

- 源身份包含楼栋、楼层、地图、资产版本和摘要，不能只看 floor_id。
- 开始前的源快照不能单独当作恢复后的健康证明。显式序列与 bridge 恢复响应比较，不与可能滞后的任务入口缓存强行相等；源资产和 BEGIN 序列仍受 bridge 现有恢复证明约束。获证后刷新源快照，持久化同一份新健康消息中的代际与序列。
- 不释放电梯、对桩或其他任务持有的资源；恢复后仍有其他 owner 暂停不等于本事务清理失败，也不能擅自清除。
- 写盘、证据或资源释放任一环节不确定，保留或重新取得更高序号的安全 hold，不能报告已经恢复。
- 切图任务仍然报告失败，说明“源图已恢复，可重试”；不能伪装成目标切图成功，也不自动恢复旧目标、继续乘梯或让小车运动。

### 4. 修改范围与明确不做的事

生产修改集中在 robot_floor_manager 的目标副作用提交记录、失败清理操作选择和结果解释。优先复用已有 OP_ABORT_PREMUTATION，不新增 ROS 接口，不改变正常目标图加载、Isaac/AMCL 定位及 COMMIT 成功流程。

配套调整真实接口回归测试、包 README 与相关文档。检查 API 对“可恢复失败”的消费，验证不留下该事务失败锁；这不是允许任意 healthy 心跳清除已有 FAILED_LOCKED。

本项不实现已修改目标资产后的自动回滚，不强行清除现场已有失败锁，不解决源图本就不可证明的其他问题，不修改其他七项门控、导航参数、雷达/DDS/TF 架构。保留工作区现有 source-independent map switching 等其他修改。

## 验收标准

实施时先让跨模块回归测试在当前实现失败，再确认修改后通过。必须覆盖真实操作选择与真实 FloorTransitionContext，不能让 mock 自行假设“普通 ABORT 恢复源图”。

1. BEGIN 成功后、首次地图请求发出前：注入单次写盘失败、取消或资产复验失败；有效源证据齐全时恢复源上下文，失败任务可重试。
2. BEGIN 响应超时：高序号 PREMUTATION ABORT 后的迟到 BEGIN 不得再次生效。
3. LoadMap 已发出但响应超时、异常或取消，服务稍后执行：始终不得按未变更路径恢复或释放安全 hold。
4. NavMap 已成功，filters/localizer/显式定位随后失败：不得误走 PREMUTATION。
5. 源身份或定位代际不匹配、源 health 陈旧、源恢复证明缺失：拒绝假恢复。
6. 恢复写盘失败、释放响应丢失、迟到 RELEASE：保持或重新证明安全 hold；保留其他 owner 的资源。
7. API、floor-manager、bridge 对该事务结局一致：失败但源已恢复，与失败锁定明确区分；新导航仍走原有正常准入检查。
8. 正常切图成功链与目标图身份、定位、TF、代价地图验收保持通过。

## 离线验证记录

- 先将旧操作选择等价提取为生产 selector，真实 bridge 状态机测试实际跑红：BEGIN 成功、目标请求未发出时，得到 runtime_context_valid=false、failed_locked=true。修正后同一测试通过。
- 补充结果返回契约回归，先复现“已恢复源图但失败返回不带源身份、无恢复说明”，修正后通过。早期失败不再被描述成目标切图成功。
- 真实状态机与生产 selector 联测：11 项通过；包括迟到 BEGIN、目标请求派发后的保守处理、证据不足、错事务/资产摘要和正常 COMMIT。
- 事务 executor 回归：11 项通过；含正常切图、恢复失败、不可恢复失败、取消、原有 handoff。
- 节点调用位置/释放顺序静态契约：5 项 pytest 通过。它们不能替代实际 ROS RPC 的故障注入验收。
- Windows 本机无 C++ 编译器，因此复用 Jetson 的现有工具链，在 `/tmp/njrh-floor-cleanup-test.nslY7C` 独立目录编译；不修改运行工作区、不替换二进制、不调用机器人 ROS 服务。
- `robot_localization_bridge`、`robot_floor_manager` 两个完整候选包编译通过；收尾改动后再次构建最终 floor-manager 可执行文件和 executor 测试，三个生产源文件 SHA-256 与本地一致。
- 最终 CTest：floor-manager 13/13 测试目标通过，bridge 5/5 通过。ROS 节点相关单测使用 ROS_DOMAIN_ID=213、ROS_LOCALHOST_ONLY=1，不接入现场 ROS 域。
- API 的既有 FloorRuntimeInterlock 10 项纯 C++ 回归通过；未修改 API 实现。独立复核未发现新增误放行或循环等待。
- `git diff --check` 通过。未修改 ROS 接口、配置参数或现场运行二进制，未部署/重启；硬件验证未执行。

复核命令（在已准备的隔离候选目录和依赖环境中运行）：

```sh
ctest --test-dir /tmp/njrh-floor-cleanup-test.nslY7C/candidate/build/robot_floor_manager --output-on-failure -j1
ctest --test-dir /tmp/njrh-floor-cleanup-test.nslY7C/candidate/build/robot_localization_bridge --output-on-failure -j1
```

后续在获准部署、车辆静止且现场有人监护时验证无目标变更失败的恢复及再次切图；会产生迟到请求或部分切图的故障注入优先在隔离 ROS 环境执行，不在运行车辆上试错。已有的 FAILED_LOCKED 不会被本修复强行清除。

## 2026-09-09 定向部署与重启

- 用户明确授权部署、重启。部署前源文件 Git blob 与本地 HEAD 一致，没有覆盖现场其他版本的 floor-manager 修改。
- 仅同步本次 `robot_floor_manager` 源文件、测试，以及已通过候选验证的可执行文件、两个静态库、对应头文件和包清单。没有部署候选 bridge，也没有替换 Nav2、安全、地图资产或运行配置。
- 旧包源码和安装文件备份在 Jetson 主机：`/home/nvidia/workspaces/njrh-v3/workspace1/reports/floor_cleanup_deploy_20260909.nKxEOr/`，分别为 `source_before` 和 `install_before`。备份未删除。
- 部署脚本：`reports/deploy_floor_cleanup_20260909.sh`。可执行文件使用同目录原子替换，不覆盖运行中进程的映像。
- 用户提供认证后，执行完整 `sudo systemctl restart njrh-runtime.service`。服务于 **2026-09-09 15:50:27 UTC（北京时间 23:50:27）**重新进入 active/running。
- 新 floor-manager PID `1317064`；直接读取 `/proc/1317064/exe` 的 SHA-256 与候选一致：`c8a159dae0003f9bb1e36bf138572f61d91e78f3ec071ef223cd069206c277da`。
- 首轮 common startup 因 Orbbec `/dock/target_observation` 新鲜消息检查超时而退出；systemd 自动重试一次。第二轮于 **15:51:20 UTC**进入 active/running，相机观测检查通过，API 8080 已恢复。当前 floor-manager PID `1319791`，再次读取进程映像 SHA-256 与上方候选一致，`NRestarts=1`。
- 第二轮初次 Isaac 定位返回 `LOCALIZATION_RESULT_TIMEOUT`：wrapper 在 20 秒内未收到本次 trigger 的结果或 bridge 显式接受；日志同时证明 `/flatscan` 新鲜且为 1440 点。驻留脚本保留 Nav2/Isaac 进程等待定位，未自动重触发。最终只读检查为导航 stopped、`safe_for_goal_start=false`、`map->odom` 未就绪、pose API 503；因此**导航未恢复就绪**，不能仅凭 systemd active/API 在线验收导航成功。本次未扩大修改定位/启动逻辑。
- 切图 interlock `blocked=false`，未知副作用计数 0，导航目标 idle；安全状态为 `DOCKED_CONTACT_BLOCK`、`motion_allowed_valid=true`，不是 ESTOP_ACTIVE。不执行解锁或自动离桩。
- 部署前车辆有新鲜 BMS 充电接触、对桩状态为 docked、导航目标 idle；本次未发送导航目标、离桩、切图或解锁命令。
- 以上证明部署/新进程加载成功，不代替真实切图失败恢复与重复切图的硬件验收。
