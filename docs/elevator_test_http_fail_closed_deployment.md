# 电梯手工测试 HTTP 接口：生产执行部署

日期：2026-07-29；普通故障重试契约更新：2026-08-04；机械臂按键接入：2026-08-25；呼梯方向规则更新：2026-08-27；乘梯执行租约移除：2026-08-27

> 2026-08-04 起，普通乘梯阶段故障不再进入需要现场解锁的长期
> `LOCKED`。下文的严格 `recover` 流程仅保留给损坏、旧版或确实无法核实
> 运行资源的维护状态，不再是导航、开门确认、乘梯、切层或取消失败的常规路径。

## 本轮交付

车端 `robot_api_server` 正式公开并部署：

```text
POST /api/v1/elevator-test/start
GET  /api/v1/elevator-test/state
POST /api/v1/elevator-test/confirm
POST /api/v1/elevator-test/cancel
POST /api/v1/elevator-test/recover
```

App 通过 `/api/v1/openapi` 发现五个接口。所有执行状态以车端持久事务为准，
App 不模拟阶段、不直接调用 Nav2、不切地图，也不发布底盘速度。

## 2026-08-25 机械臂按键接入

生产 overlay 已启用 `127.0.0.1:8083` 黑盒客户端。App 仍通过原 `start`
请求选择 `building_id`、`elevator_id`、源层/目标层和各自精确 map/release；
App 不直接调用机械臂接口。`B1`、`B2` 等是楼栋 ID，不是楼层 ID；一次事务的
源层和目标层必须来自同一个 `building_id` 的同一电梯发布版，电梯不能跨楼栋。

- 到 `hall_call` 后：执行 `ready`，按本次路线方向键，再执行 `release`；只等待
  三个功能任务各自返回结果，不查询 health/status，也不以机械臂姿态、busy、
  ready 或 `safe_to_drive` 字段阻断流程。方向以已核验的当前源层和任务目标层
  数值比较：`目标层 > 当前层` 发送 `up`，`目标层 < 当前层` 发送 `down`。
  若某栋楼确有地下层，楼层 ID 必须明确写作 `-2`、`-1`，例如 `-2 -> -1`
  为 `up`、`F1 -> -1` 为 `down`；`B1/B2` 会被楼层解析器拒绝。同层不构成
  有效乘梯路线。
- 当前 8083 `/api/v1/elevator/call` 已执行物理呼梯，任务成功后自动推进。若兼容
  的旧后端明确返回 `501 capability_unavailable`，车端仍在机械臂 `release` 任务
  完成后恢复 `CALL_BUTTON_PRESSED` 人工确认，不伪造按键成功。
- 人工确认源层门开后，小车按既有四点链进入 `cabin`，再到 `cabin_panel`；
  此处自动 `ready -> press-floor -> release`。目标楼层仅允许明确的 `-2`、`-1`
  或 F1～F20；B1/B2 只允许出现在 `building_id`。
- 源层门开、目标层到达、目标层门开仍为人工观察。机械臂任务的 HTTP 拒绝、
  明确失败或超时作为功能失败返回；不再增加安全状态证明。
- `start` 以及常规/恢复 hold 释放均不检查机械臂状态。机械臂即使报告为
  `ready`、busy 或 `safe_to_drive=false`，也不会因此拒绝乘梯测试。

## 执行边界

`start` 会：

1. 固定不可变的电梯发布版本以及源层/目标层 map ID、asset epoch、digest；
2. 核验源层实时 localizer/bridge 身份、Nav2/建图/回充空闲；
3. 建立 owner-scoped safety hold 和 transaction-owned operating-mode lease；
4. 在自动机械臂按键与保留的人工物理观察之间串行执行 Nav2 与严格
   FloorSwitch action；
5. 仅在目标地图、Isaac localizer、bridge、costmap 与 Nav2 readiness 全部
   提供同一目标资产证据后继续出梯。

确认事件词表固定为（自动成功的按键事件不会要求 App 确认）：

```text
CALL_BUTTON_PRESSED
SOURCE_DOOR_OPEN
TARGET_BUTTON_PRESSED
TARGET_FLOOR_ARRIVED
TARGET_DOOR_OPEN
```

`CALL_BUTTON_PRESSED` 仅在当前物理呼梯能力不可用时出现；
`TARGET_BUTTON_PRESSED` 仅在未启用自动按键的兼容模式出现。源层开门、目标层
到达和目标层开门始终需要人工确认。地图切换发生在目标层开门确认之后，由车端
原子事务执行，不由 App 直接切换。

## 乘梯运行身份与模式续期

乘梯测试不再创建、续期或释放 `/safety/set_execution_lease`。FSM 中对应枚举只为
读取历史日志保留，正常事务不会发出这两个 effect。运行身份由精确 transaction、
owner hold、`robot_elevator_manager/elevator_<transaction>` operating-mode contract
及目标动作身份共同确定。

operating mode 使用一条独立 keepalive worker。模式续期失败后可重新提交同一精确
owner/mission；若服务明确证明旧 ID 已过期并退役，则换用本事务的新 mode lease ID。
真正释放 operating mode 前先停止并 join worker，避免迟到续期重新建立模式。

停车保持期间，模式续期失败不会取消已经提交的 `FloorSwitch`：`begin_floor_switch`
和 `await_floor_switch` 不读取 keepalive failure，也不以它触发 action cancel。切图仍由
floor manager 在 owner hold 和 correction-pause handoff 下完成。下一段真实运动前，
API 必须重新取得精确 operating mode；不能证明时 `robot_safety` 保持零速。这样既不
让与切图无关的续期错误制造 `FAILED_LOCKED`，也不允许模式缺失时继续运动。

回归测试直接锁定三条源码契约：FSM 不生成 execution-lease effect；API 不创建
`SetExecutionLease` 客户端；`begin_floor_switch`/`await_floor_switch` 不含模式续期失败
取消分支。底层 keepalive 与旧 execution arbiter 的隔离单元测试仍保留，以保证历史
恢复和其他调用方没有被本次乘梯流程修改破坏。

## 安全与恢复

- `prepare` 失败默认仍进入 `FAILURE_CLEANUP`，执行 hold/cancel 并按失败关闭
  处理。只有 ROS runtime adapter 能够显式证明该拒绝发生在任何运行时效果
  之前，执行层才允许直接落为 `FAILED / RUNTIME_PREFLIGHT`。执行层不得根据
  错误字符串自行猜测“零副作用”；未知、缺字段或未证明的失败一律按需要清理
  处理。
- 所有 hold 命令携带 `(owner, transaction_id)` 内严格单调的
  `command_sequence`。`robot_safety` 永久保留当前进程生命周期内的最高
  已接受序号，因此迟到的 `RELEASE(N)` 不能覆盖清理阶段的
  `ACQUIRE(N+1)`。
- 取消意图与 Nav2/FloorSwitch goal 提交在线性化临界区内排序；取消之后
  不会再启动新的自动效果。
- 普通阶段失败按当前已提交的运行身份清理：`SwitchFloor` 成功前绑定精确
  source floor/map，成功后绑定精确 target floor/map。资源取消、回收和本事务
  hold 释放成功后终态为 `FAILED/CANCELLED`，可直接重新开始测试。
- 非终态或保留 hold 的 journal 在进程重启后绝不续跑。车端进入
  `LOCKED`，重新获取序列化 owner hold、取消未知 action、等待 action
  平面空闲并证明 wheel/local 双里程计稳定停车。
- 重启恢复先等待安全端点并重新获取该事务的精确 owner hold，再等待启动较晚的
  Nav2/FloorSwitch action server。仅“启动端点暂未就绪”使用独立且有限的 40 次
  尝试窗口（每轮仍受端点/服务超时约束）；真实清理证明失败仍最多三次。首次窗口
  耗尽会持久化 `ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN`，不会续跑任务。
- cancel-all 已提交但响应尚未到达时，车端保留同一个 future 并进入有限的启动
  等待窗口；后续轮次只观察这一个请求，不重复提交。只有收到明确应答后才推进
  action 空闲证明；pending 不等于成功，也不允许释放 hold。仅 future 异常或
  证据真正丢失时才记录 `*_CANCEL_RESPONSE_UNKNOWN` 并要求整套运行时重启。
  若随后由生产 supervisor 做完整运行链冷启动，旧进程持有的 action、lease 和
  owner hold 已随整链退出；新执行层只对 schema v3、普通阶段、current floor/map
  与 `SOURCE_OUTSIDE` 或 `TARGET_OUTSIDE` 精确一致的这个耗尽形状收口为可重试
  `FAILED`，并记录 `NORMAL_STAGE_LOCK_CLEARED_ON_COLD_START`。该收口不宣称小车
  移动或双里程计停车证明。`RETAIN_LOCK`、prepare、身份错配、日志损坏和存储/
  审计故障仍保持 `LOCKED`。
- `LOCKED` 或保留 hold 时，除只读、stop/cancel 外的所有车端写接口均被
  409 阻断。新的电梯事务不能覆盖旧 journal。
- 进舱、舱内、乘梯、切层前以及取消失败均走上述普通清理，不再依赖
  `motion_not_authorized_proven` 才能从长期锁中恢复。该字段仍作为诊断证据保留，
  但不决定普通失败能否重试。

`/navigate_to_pose/_action/status` 是事件状态，不是 0.75 秒心跳。车端会锁存
最后一次 active/terminal 状态，直到收到新的 action 状态；0.75 秒新鲜度只用于
motion hold 与 wheel/local odometry 等实时证据。因此人工等待开门或确认楼层时，
已经结束的 Nav2 goal 不会仅因时间经过而被误判为不空闲。

### 受限人工恢复

`POST /api/v1/elevator-test/recover` 不是普通故障的取消/重试按钮。它只处理
历史版本或运行证据损坏后遗留的维护事务。当前 schema v3 中由旧版普通阶段
策略产生的 `ELEVATOR_EXECUTION_ON_SITE_SERVICE_REQUIRED`，若日志中的 current
floor/map 与 source 或 target 精确相同，会在服务重启时自动转换并清理，无需 App
人工恢复。仍需维护恢复时，请求必须精确携带并匹配：

```json
{
  "transaction_id": "elevator-test-...",
  "expected_state": "LOCKED",
  "effect_sequence": 2,
  "operator_id": "commissioning_app",
  "reason": "operator_confirmed_source_outside_stationary_door_zone_safe",
  "source_outside_confirmed": true,
  "confirmed_floor_id": "F2"
}
```

`source_outside_confirmed` 和 `confirmed_floor_id` 只用于上述进舱导航失败。
车端同时要求 `confirmed_floor_id == current_floor_id == source_floor_id`、事务
owner hold 已知有效、操作员与启动事务的操作员一致。请求先追加
`ON_SITE_SOURCE_OUTSIDE_CONFIRMED` 审计，再把本次清理分类为
`SOURCE_OUTSIDE`；它本身不会释放 hold。轿厢运行、楼层切换、目标层出梯或
未知来源的锁仍返回冲突并要求现场服务。

示例中的 `effect_sequence` 仅为格式示意；调用方必须使用刚从 state 响应读取的
当前正整数，不能写死或重放旧序号。

现场确认是一次性持久证据。若确认后的车端空闲、资源、双里程计或 hold 证明
失败，事务会以新 `effect_sequence` 保持 `LOCKED`，但已变为
`cleanup_disposition=SOURCE_OUTSIDE`，并保留
`ON_SITE_SOURCE_OUTSIDE_CONFIRMED`。此时重试必须使用新序号，发送
`source_outside_confirmed=false` 和空 `confirmed_floor_id`；车端重新执行全部安全
证明，不要求操作员虚构第二次现场观察。生产 ROS 适配器只允许同一事务及完全
相同楼层/地图/epoch/digest 的 `RETAIN_LOCK -> SOURCE_OUTSIDE` 单向提升，任何
反向转换、目标层转换或身份漂移继续失败关闭。

接口单次提交，服务端异步完成日志先行的双阶段流程：

1. `RECOVERY_VERIFYING`：先持久化恢复意图和原锁快照，再获取本事务的
   recovery safety hold；对 Nav2/FloorSwitch 分别执行 cancel-all，并按
   cancel 应答中的精确 goal ID 等待应答后的终态证据；空应答本身只证明该
   应答时点没有目标，之后出现任何 active goal 都使本轮证明失败；同时证明 Nav2、
   WebSocket teleop、mapping、docking 均空闲，且没有 execution session、
   operating-mode lease 或 localization pause；使用 fresh wheel/local
   odometry 共同证明小车已经稳定停车；
2. `RECOVERY_RELEASE_PENDING`：先持久化 active journal 和不可覆盖的
   `history/<transaction>.<sequence>.recovery_release_pending.yaml`，再做一次
   紧邻释放的 action cancel/终态屏障，随后调用 robot_safety 专用的
   `/safety/release_motion_hold_if_execution_idle`。该服务在 arbiter 同一
   临界区内精确比较最终资源为空样本的 generation、再次确认 execution
   session/lease 均为空、核对 owner/transaction hold 与新鲜命令序号，然后
   原子释放；API 还必须从响应再次证明序号准确、generation 仅前进一次、
   session/lease 为空且 owner hold 已不存在；最后才把 active journal 原子写成
   `FAILED / RECOVERY_COMPLETE`，并写入对应的不可变 complete 审计。

恢复期间 `state` 始终为 `LOCKED`。任何身份、序号、状态、日志写入、空闲、
资源、双里程计停车或 hold absence 证明失败，事务都保持 `LOCKED`，全局写
互锁不解除。`runtime_applied=true` 表示事务可能已取得执行 session、模式或
定位暂停等资源；除上面严格限定且经过现场确认的源楼层进舱失败外，车端不会
自动解锁，必须由现场服务按资源所有权逐项处理。

HTTP 写接口和 WebSocket teleop 使用同一个电梯事务互锁判定；
`LOCKED` 及恢复过程都不能通过 WebSocket 绕过。恢复不能通过删除、改名或
手工编辑 `maps_release/.elevator_test/active.yaml` 实现；该文件是恢复证据，
不是操作员复位开关。

当 `api_token` 未配置时，`operator_id` 只是写入审计日志的声明身份，不能
作为鉴权证据。因此无 token 配置下，`/elevator-test/recover` 只接受 Jetson
本机 loopback 维护连接，局域网 App 不能调用；配置 token 后才允许通过已认证
的远端连接恢复。商用部署必须启用认证，并从已认证会话注入真实操作员 ID。

active journal 采用兼容的 v1 外层和 v2 snapshot 语义。hold 状态为 UNKNOWN
时，持久化给旧版读取器的兼容投影强制写成 active，避免软件回退时把 UNKNOWN
误判为已释放。旧 journal 缺少 `runtime_applied` 时同样按 UNKNOWN/可能已执行
处理，只允许现场服务，不进入自动 preflight-orphan 恢复。journal 损坏、符号
链接、未知 schema 或持久化不确定时，即使无法渲染 transaction snapshot，
HTTP 与 WebSocket 写入互锁仍保持 recovery-required。

状态明确投影：

```json
{
  "safety_hold_state_known": true,
  "safety_hold_active": true,
  "dual_odom_stop_proven": true,
  "recovery_required": true
}
```

`safety_hold_active` 只有在 `safety_hold_state_known=true` 时才可解释；
`false/false` 表示 UNKNOWN，不表示 hold 已释放。UNKNOWN 的终态继续保持
`recovery_required=true`。这些字段来自实际运行证据，不能根据“请求成功”
推断。

## 不变项

- App 不直接发底盘速度；
- 最终速度链仍为
  `Nav2 -> velocity_smoother -> collision_monitor -> robot_safety -> /cmd_vel -> ranger_base`；
- `map->odom` 仍仅由 `robot_localization_bridge` 发布；
- `odom->base_link` 仍仅由 `robot_local_state` 发布；
- 不修改 FAST-LIO2、JT128 点云/QoS/DDS/时间戳以及现有导航调参。

## 部署验证

部署前在隔离 ROS domain 构建并运行 C++/Python 测试；App 运行全量测试、
静态分析与 release APK 构建。生产部署只允许整套重启
`njrh-runtime.service`。重启后只读核验接口、参数、进程唯一性、资产身份、
TF 所有权和速度链；在操作员明确发起测试前不调用 `start`，不移动小车。

恢复路径仍需在真机静止状态验证：精确 preflight orphan 能完成双里程计停车
和 hold absence 证明；恢复期间 HTTP 写请求与 WebSocket teleop 均被阻断；
错 transaction/序号、`runtime_applied=true` 以及任一证据缺失都保持
`LOCKED`；全过程保留 `active.yaml` 和审计记录，且不产生 Nav2 goal、
FloorSwitch commit 或非零速度。

## 2026-07-29 Jetson 运行态核验

生产整栈重启后，B11/F2 的 resident navigation 启动记录证明：

- 重定位前 bridge 显式序列基线为 `0`；
- Isaac 触发结果被 `robot_localization_bridge` 接受为显式序列 `1`；
- 同一证据样本包含 `has_map_to_odom=true`，且发布者所有者为
  `robot_localization_bridge`；
- Nav2 核心生命周期节点全部 active，随后 AMCL 进入 `AMCL_READY`；
- 电梯执行 journal 不存在，没有未完成或恢复锁定的测试事务。

`GET /api/v1/elevator-config` 中的 `runtime_applied=false` 是不可变发布清单的
设计字段：发布配置本身不会移动小车、切图或释放安全保持。它不表示生产
adapter 未部署。操作员调用 `start` 后，车端会先固定发布版本并执行实时
prepare；只有 prepare 的完整安全证据通过，具体事务状态中的
`runtime_applied` 才会变为 `true`。因此 App 判断执行能力必须以
`/elevator-test/*` 的事务状态为准，不能把配置查询里的清单字段当成运行态。
