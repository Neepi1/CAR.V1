# P6 电梯链路：机械臂与视觉后接阶段

日期：2026-07-23
状态：接口、事务围栏与负向互锁收口阶段；**未授权真实电梯运动或真实切层**

> 2026-08-25 更新：本文标题保留历史阶段名。生产
> `robot_api_server` 已接入 `127.0.0.1:8083` 机械臂黑盒，自动执行厅外呼梯
> 按键效果和轿厢选层按键效果；门开、到层、目标层门开仍为人工观察。当前黑盒
> 的物理呼梯端点返回 `501 capability_unavailable`，因此厅外呼梯会在
> `release` 任务完成后降级到人工 `CALL_BUTTON_PRESSED`，不会伪造成功。当前为
> 功能验证模式，不使用 health/status/避让状态作为流程门控。下文关于“后接”
> 的描述是 7 月设计基线，最新运行契约以本更新和
> `elevator_test_http_fail_closed_deployment.md` 为准。

## 2026-08-05 当前方案：四点倒车入梯

当前新增 schema v3，作为后续现场标定和乘梯联调的目标契约。每层需要四个
内部点：`hall_call`（呼梯作业位）、`landing`（倒车入梯接驳位）、
`cabin`（轿厢中心位）、`cabin_panel`（轿厢面板作业位），并显式选择
`hall_call_panel_side` 与 `cabin_panel_side` 的 `LEFT/RIGHT`。左右均以机器人
在相应标定点的车头方向为基准，系统不从地图坐标猜测。

流程固定为：呼梯完成后转到与 `landing` 相同的朝向（约 180°），先纵向
对线再横移到接驳位；确认开门后保持该朝向直线倒车到 `cabin`；随后按轿厢
面板侧横移到 `cabin_panel`。切层以目标层 `cabin_panel` 为定位锚点，之后
先横移回 `cabin`，再前进到目标层 `landing`。schema v2 三点发布版继续兼容，
但不会被静默补造第四点或左右属性。

后尾激光雷达尚未安装，因此当前实现只建立软件契约和受控动作链；在后向
盲区完成覆盖和实车验收前，不得据此宣称可无人值守倒车入梯。

## 0. 本轮新增：不可变 release 运行预检

`robot_elevator_manager::load_elevator_release()` 现在是电梯执行链的配置
入口。它只做无副作用预检：

1. 从 `.elevator_config/current` 只固定一次 release，或使用调用方给出的
   精确 `expected_release_id`；
2. 后续只读取该 release 目录中的
   `configuration.yaml`、`elevators.yaml`、
   `elevator_internal_poses.yaml`、`validation.json`、`manifest.json` 和
   `current.json`；
3. 联合校验 release/generation/lineage、发布标志、legacy configuration
   digest、canonical `sha256:<64 lowercase hex>` 地图绑定以及对应 schema
   的拓扑和内部点；schema v1 五点/门槛只做历史完整性校验，不能执行；
4. 首选电梯为空时，按 `elevator_id` 稳定选择能精确服务 source/target
   floor+map 的电梯；
5. 对 schema v2/v3 返回按值冻结的 release、选中电梯、source/target 地图
   摘要和相应角色/面板侧。任务期间 `current`
   再切换也不会改变已返回对象。

Jetson/Linux 实现固定 release 目录 FD，并使用
`openat(O_NOFOLLOW)`、`fstat`、2 MiB 单文件上限和单硬链接约束读取文件。
任何软链接、硬链接、路径逃逸、重复 JSON key、摘要不一致、额外/缺失内部点
或非有限坐标都会 fail closed。

地图 bundle 摘要已经与 `FloorSwitch` preflight 统一为 canonical SHA-256。
当前 release 一旦绑定某张地图，HTTP 禁行区写入口会拒绝原地修改该地图，
返回 `ELEVATOR_CONFIG_MAP_IN_USE`，避免已发布 release 静默过期。

这仍不是 live 执行许可。现有地图资产没有权威、持久、单调的
`asset_epoch`，FloorSwitch 仍为 preflight-only，localizer 也尚不能提供带
目标 asset identity 的真实 reload/result evidence。因此 loader 成功后仍然：

- `runtime_applied=false`；
- 不构造或发送 `FloorSwitchGoal`；
- 不调用 Nav2、map server、localizer、costmap、safety/mode/pause 服务；
- 不发布 Twist 或 TF；
- 不移动小车。

## 1. 本阶段范围

本阶段先完成与机械臂、视觉算法无关的部分：

- 电梯和跨层任务的确定性状态机；
- 楼层、电梯和三个内部 waypoint 的数据契约；
- owner-scoped 模式、运动 hold、执行 TTL 和定位修正暂停；
- `landing -> cabin` 和 `cabin -> landing` 的串行导航与停车确认边界；
- 串行 Nav2 effect 和失败锁定；
- floor switch 的目标成功屏障；
- 可重复的纯单元测试和隔离 mock 测试边界。

机械臂点按和视觉识别后续通过 `PressButton.action`、
`ElevatorObservation.msg`、`ArmState.msg` 接入，不进入状态机核心。

本阶段不包括：

- 真实呼梯、选层或按钮接触确认；
- 真实门状态、楼层显示和置信度输入；
- 真实电梯进舱、乘梯或出舱；
- 用 mock 结果代替真实门、楼层、按钮或机械臂许可；
- 调整 odom、EKF、AMCL、MPPI、导航容差或 JT128/FAST-LIO2 链路。

## 2. 当前实现快照

| 组件 | 当前已有 | 当前没有 |
|---|---|---|
| `robot_interfaces` | Elevator/Mission/FloorSwitch/PressButton actions；门、机械臂、floor、safety、mode、correction-pause 消息与服务 | 接口文件本身不提供运行节点或安全保证 |
| `robot_elevator_manager` | 纯 C++ schema-v1/v2 topology 与 YAML catalog loader、schema-v2 三点事件驱动 FSM 和 GTest；核心已编排 `BEGIN_FLOOR_TRANSITION -> resume -> switch`，安全清理 effect 可确认和重试；旧门槛分类只保留给 v1 历史资产校验 | ROS 节点、action server、Nav2/floor/safety/mode adapter、真实 mock node |
| `robot_mission_manager` | 同层/跨层纯 C++ 串行 effect FSM 和 GTest；跨层 completion 校验 floor/map、asset epoch/digest、显式重定位、`TARGET_HALL`、上下文及残留 goal/hold/lease/pause；清理 ID 跨重试稳定 | ROS action server、Nav2/elevator adapter、任务恢复节点 |
| `robot_mode_manager` | owner/mission/lease 仲裁核心、服务节点、状态 heartbeat 和单元测试 | 生产 bringup 与所有 profile 消费者的实车验证 |
| `robot_safety` | owner-scoped motion hold、历史/通用 execution TTL arbiter、乘梯 transaction-owned mode owner/mission/lease 校验、电梯期仅允许 Nav2 正常源、状态主题、最终 Twist gate、单元测试和手动隔离脚本 | 精确 Nav2 goal 身份绑定、进程丢失和真实停车距离验收 |
| `robot_localization_bridge` | owner-scoped correction pause 与 legacy Bool 组合；`BeginFloorTransition` BEGIN/COMMIT/ABORT 围栏；typed `/localization/floor_health`；单元测试和隔离 ROS smoke | localizer generation、TF 唯一性和目标 asset epoch 的外部权威证明；实车切层验收 |
| `robot_floor_manager` | 旧服务仅允许 `resume_navigation=false` 选择；`resume_navigation=true` 零副作用拒绝；新增 preflight-only `FloorSwitch.action`、typed status、纯 C++ pause 交接/source invalidation/epoch/digest/readiness/失败锁定核心 | Action 到 bridge、真实 map/filter/localizer/costmap adapter 的 live 串接和目标 readiness 观测 |
| `robot_api_server` | 永久订阅 typed floor/health；事务活动、runtime invalid 或 `FAILED_LOCKED` 时阻止新导航、建图、地图/点位/禁行线/电梯配置写入、重定位、停靠、脱桩和 safety resume；stop/cancel 仍允许；乘梯流程使用精确 mode contract 且不创建 execution lease；已实现电梯配置草稿、校验、不可变发布、回滚和内部点隔离 | 完成切层后对请求解析代际的强一致 token；配置发布后的 live 应用 adapter |
| 机械臂/视觉 | 预留接口 | action server、observation publisher、标定和现场证据 |

特别注意：

- `robot_elevator_manager` 和 `robot_mission_manager` 当前没有 `main()` 或
  ROS 节点，不能接收 action，也不会移动机器人。
- 纯 FSM 的 `ElevatorFsmOptions.mock_ports_enabled` 默认为 `false`，会在任何
  运动 effect 产生前拒绝启动，这是预期的 fail-closed 行为。
- topology loader 现在要求、读取并保存 YAML 顶层 `mock_ports_enabled`；
  `elevators.example.yaml` 的受控默认值为 `false`。
- 单元测试和非运动 mock 场景必须显式使用 `ElevatorFsmOptions{true}` 才会发出
  `MOCK_*` effect。当前仍没有 ROS mock adapter 或定时门状态模拟器；未来
  runtime adapter 必须显式把 catalog 中的值传给 FSM，不能隐式启用 mock。
- 新 `FloorSwitch.action`、`FloorSwitchStatus.msg` 和
  `BeginFloorTransition.srv` 已定义。floor manager 已提供 Action 名称，但当前
  只执行不改变资产的严格 preflight；bridge 已实现独立 BEGIN/COMMIT/ABORT
  围栏。两者尚未串成 live transaction，不能据此宣称原子跨层已完成。
- `FloorSwitchStatus` 的初始 `IDLE` 和 preflight `BLOCKED` 当前不会被 API
  当作失败锁；只有明确 mutation state、bridge runtime invalid/active 或
  `FAILED_LOCKED` 才阻断。这个兼容围栏不等于完整正向 readiness 证明。

## 3. 包边界

```text
App / business request
        |
        v
robot_mission_manager
  same floor + same map -> one Nav2 target
  different floor       -> one ElevatorTask -> one final Nav2 target
  same floor + new map  -> reject; explicit FloorSwitch workflow required
        |
        v
robot_elevator_manager
  schema-v2 three-point topology + elevator FSM
        |
        +--> Nav2 adapter ------------------------------+
        +--> robot_mode_manager                         |
        +--> robot_floor_manager                        |
        +--> robot_localization_bridge pause ownership  |
        +--> future arm / vision adapters               |
        +--> robot_safety hold / exact mode contract ---+
                                                        |
Nav2 -> velocity_smoother -> collision_monitor -> robot_safety
     -> /cmd_vel -> ranger_base
```

职责约束：

- mission manager 不识别门、不按按钮、不切图；
- elevator manager 不发布 Twist、不发布 TF、不修改导航参数；
- floor manager 只拥有楼层资产事务，不决定何时进出电梯；
- mode manager 只拥有运行模式控制面，模式不是运动许可；
- `robot_safety` 是最终运动许可和 `/cmd_vel` 唯一发布者；
- `robot_localization_bridge` 是 `map -> odom` 唯一发布者；
- `robot_local_state` 是 `odom -> base_link` 唯一发布者。

## 4. 任务编排契约

### 4.1 Mission FSM

同层任务：

```text
IDLE -> NAV_TARGET -> SUCCEEDED
```

这里的“同层”还要求 source/target map 相同。同层但 map 不同的请求会在产生
effect 前拒绝，必须由独立 FloorSwitch 工作流处理，不能借电梯任务绕过。

跨层任务：

```text
IDLE -> ELEVATOR_TASK -> NAV_TARGET -> SUCCEEDED
```

每个 effect 带独立 `transaction_id`。只有当前精确 transaction 的成功
回执才能产生下一个 effect。旧回执、重复回执和其他任务回执均不得推进状态。
任意时刻最多存在一个 active effect。

取消、abort 或 active effect 失败时：

```text
HOLD_AND_CANCEL -> FAILURE_LOCKED
```

锁定不会被迟到的成功消息解除。`HOLD_AND_CANCEL` 失败会保留稳定
`cleanup_id`、产生新的 effect transaction 并继续重试；安全清理成功也只结束
active cleanup effect，不解除 `FAILURE_LOCKED`。恢复必须由独立、显式的
recovery 流程完成。

当前 `MissionFsm::EffectCompletion` 已携带并验证：

- 精确目标 floor/map、非零 asset epoch、asset digest 和显式重定位序列；
- `runtime_context_valid=true`、`final_zone=TARGET_HALL`；
- 无活动 Nav2 goal、无 safety hold、无残留 mode/execution/pause lease。

任何一项不满足都会进入安全清理和失败锁定，不能继续发送最终配送点。每次
进程启动还必须注入新的 `instance_id`，防止旧进程回执与新 transaction 撞号。

### 4.2 Elevator FSM

当前纯核心的有序 effect 为：

```text
导航到 hall_call
-> 获取厅外 motion hold
-> MOCK 呼梯
-> 设置 ELEVATOR_WAIT
-> 释放厅外 hold
-> 导航到源层 landing
-> MOCK 等待源层开门
-> 设置 DOORWAY
-> 导航到源层 cabin
-> 确认 cabin 目标成功且车体停稳，获取舱内 hold
-> MOCK 按目标楼层
-> 获取 owner-scoped correction pause
-> 设置 ELEVATOR_RIDE
-> MOCK 乘梯
-> MOCK 确认目标层开门
-> 请求 FloorManager 获取自己的 correction pause 并使 source runtime context 失效
-> 在 hold 下精确释放电梯事务的 correction pause
-> 调用目标 FloorSwitch
-> 验证目标楼层 readiness
-> 设置 DOORWAY
-> 释放舱内 hold
-> 从 cabin 导航到目标层 landing
-> 确认 landing 目标成功且车体停稳，获取厅外 hold
-> 精确释放 mode lease
-> 释放出口 hold
-> COMPLETE
```

任何 active effect 失败、取消、乱序或序号不匹配都会进入
`FAILURE_CLEANUP` 并产生 `HOLD_AND_CANCEL`。所有非 `NONE` effect 都带
`accepted=true`；非法启动请求在任何运动前直接拒绝且不产生 effect。普通阶段
故障不再按“车可能位于轿厢任意位置”永久锁定：`SWITCH_FLOOR` 成功前使用日志中
精确的源层运行身份清理，成功后使用精确的目标层运行身份清理。资源回收和本事务
hold 释放完成后终态为 `FAILED/CANCELLED`，允许操作员重新开始测试。只有日志
损坏、schema 不支持或运行资源确实无法核实时才保留维护锁；底层碰撞、急停和
`robot_safety` 速度仲裁不因此绕过。

若完整运行链冷启动时 Nav2/定位端点尚未就绪，旧版共享 40 次窗口可能留下
`ELEVATOR_EXECUTION_RESTART_RECOVERY_UNPROVEN`。下一次完整运行链冷启动只会将
schema v3、普通阶段、精确匹配 source/target floor-map 与 cleanup disposition 的
该终态收口为 `FAILED`；不会继续乘梯任务，也不会虚构运动或停车证明。prepare、
`RETAIN_LOCK`、身份错配、日志损坏及存储/审计故障不走此路径。

当前核心已经编排
`BEGIN_FLOOR_TRANSITION -> RESUME_LOCALIZATION_CORRECTIONS -> SWITCH_FLOOR`。
该顺序表达的是 correction-pause 所有权交接，不是已经可部署的跨层协议。生产
adapter 只有在 floor transaction 真正提供以下 fencing 时，才能确认每个 effect：

```text
hold 已确认
-> BeginFloorTransition：
     FloorManager 先获取自己的 owner-scoped pause
     普通 source runtime context 立即 invalid
-> 确认 begin 后，elevator owner 精确释放自己的 pause
   （FloorManager pause 仍使 effective pause=true）
-> FloorSwitch 加载并标记 target asset epoch
-> 进入目标显式定位阶段时，FloorManager 精确释放自己的 pause
-> 只接受 target epoch 的显式定位
-> readiness barrier
-> commit target runtime context
```

纯 `floor_transition_core` 已表达 begin transaction、独立 pause record、
source invalidation、target epoch/digest 和 failure lock。bridge 的 typed
transaction adapter 已存在，floor manager Action 仍是 preflight-only；真实
资产 reload、localizer generation、bridge target epoch 与 typed costmap
证据尚未接成一个执行器。在这些 seam 实现前，必须保持真实电梯 motion 和
真实 floor switch disabled。

## 5. Nav2 串行规则

1. 电梯事务任意时刻最多一个活动 Nav2 goal handle。
2. 不使用一次灌入多个点的 `NavigateThroughPoses` 代替阶段许可。
3. 前一个 action 必须明确终态，且阶段后置条件成立，才能发送下一目标。
4. goal 接受前保持 hold。首段 `hall_call` 在 action 接受后，只能在 fresh
   interlock 证明 hold 已释放、无 motion/effective blocker、且没有遗留 execution
   session/lease 时进入普通 Nav2 首指令暖机；此时 `COMMAND_STALE` 是预期状态。
   其余电梯段必须在门、机械臂及 transaction-owned 精确 operating mode
   均有效后才可释放 hold；乘梯流程不创建 execution lease。
5. goal 结束后先重新获取 hold，再检查实际停稳和楼层状态。
6. cancel 未确认或停稳未确认时不得发送新 goal。
7. mission 活动时，普通 App 导航入口必须拒绝外部目标，避免 Nav2 preempt。
8. 电梯内部 waypoint 不经过 API 的 post-Nav2 速度修正或 API fallback。
9. manager 不发布 `/cmd_vel_api`、`/cmd_vel_collision_checked` 或 `/cmd_vel`。

## 6. 电梯拓扑与四点倒车模型

每栋楼建议提供：

```text
maps_release/<building_id>/elevators.yaml
```

schema v3 要求每个服务楼层恰好一个以下角色：

- `hall_call`
- `landing`
- `cabin`
- `cabin_panel`

并要求：

- `hall_call_panel_side: LEFT|RIGHT`
- `cabin_panel_side: LEFT|RIGHT`

`landing` 是厅外倒车接驳点，车头朝厅外、车尾朝轿厢；`cabin` 必须位于
该朝向的正后方。`cabin_panel_side` 由操作员按标定车头方向明确选择，是面板
左右语义的权威来源；发布校验不再根据 `cabin -> cabin_panel` 的地图 XY
反推左右，也不要求两个实测点形成纯横移向量。`hall_call -> landing` 固定执行
转向、纵向对线、横移。此后的所有轿厢内部及进出段——`landing -> cabin`、
`cabin -> cabin_panel`、目标层 `cabin_panel -> cabin`、`cabin -> landing`——
统一使用无代价地图碰撞检查的直达 Nav2 链，并按实时目标残差闭合 yaw、横向和
前后误差。厅外呼梯和接驳段仍保留正常碰撞检查。schema v3 不包含门槛线、门框点、
`clearance_m` 或 `jamb_clearance_m`。schema v2 的三点模型仅保留给已有发布版。

配置管理模块自动为角色生成全楼唯一的内部 pose ID。坐标不写入本层普通
`poses.yaml`，而是写入发布版本专用的
`elevator_internal_poses.yaml`；`elevators.yaml` 只保存 topology 对这些 ID 的
引用。普通点列表、普通点写入/删除和普通导航入口都不得访问这些点。

配置资产布局为：

```text
maps_release/<building_id>/.elevator_config/
  draft.json
  drafts/<draft_revision>/
    configuration.yaml
    validation.json
  current -> releases/<release_id>
  current.json -> current/current.json
  releases/<release_id>/
    configuration.yaml
    elevators.yaml
    elevator_internal_poses.yaml
    validation.json
    manifest.json
    current.json

maps_release/<building_id>/elevators.yaml
  -> .elevator_config/current/elevators.yaml
maps_release/<building_id>/elevator_internal_poses.yaml
  -> .elevator_config/current/elevator_internal_poses.yaml
```

发布按整栋楼生成不可变 release，并用 `expected_draft_revision` /
`expected_release_id` 防止多个标定端互相覆盖。回滚会从历史内容生成新
release，不覆盖旧版本。发布前重新验证精确 `building/floor/map_id`、地图
资产摘要、地图边界、四角色、两项面板侧、倒车几何、面板点 yaw 以及 topology YAML 回读。有效草稿会
固化每个绑定地图的 `map_asset_digest`；复核后若地图发生变化，发布必须以
`MAP_ASSET_DIGEST_CHANGED` 拒绝，不能静默绑定新资产。重新绑定必须是显式
操作：App 删除旧的服务端管理 digest 后重新保存，由服务端写入当前 digest
并返回新的草稿 revision，再次进入人工复核。

Linux 发布先完整写入并 `fsync` 不可变 release，最后只原子切换一次
`.elevator_config/current` 目录选择器。需要同时读取 topology 与内部点位的
消费者必须先固定同一个 `release_id`，再从该 release 目录读取两份文件；
不得在 selector 切换两侧分别读取兼容投影。

地图删除与配置发布/回滚按同一锁序串行。当前 release 仍引用的地图必须以
HTTP `409 ELEVATOR_CONFIG_MAP_IN_USE` 拒绝删除，先发布解除引用的新版本。

配置发布仅表示资产落盘，响应固定包含
`asset_published=true, runtime_applied=false`。它不会切图、重定位、启停
Nav2、发送目标、解除 hold 或发布速度；因此不能把配置发布成功当作真实
电梯链已启用。

旧 schema v1 的五角色和门槛几何仍可被 loader 校验，以证明历史 release
没有被篡改，但 loader 返回 `LEGACY_READ_ONLY`，FSM 也拒绝 schema v1 route。
旧 schema v2 可继续读取、回滚和执行；新的 App 编辑流程写 schema v3。

四点模型不把“到点”单独当成门已打开或通道安全的证明。首段 `hall_call`
按普通厅外接近契约放行；它仍要求无急停、
定位有效、非在桩、无 hold、无既有 execution session，并持续经过实时激光、
Nav2 costmap、collision monitor 和 `robot_safety`。该目标成功后，从呼梯点到
源层 landing/进梯接驳位的 schema-v2 `SOURCE_LANDING_FACE_CABIN` 与 schema-v3
`REVERSE_ENTRY_STAGING` 改为复用后续轿厢段的无障碍检测契约：规划器不读取
costmap 障碍，控制器不做 clearance/障碍重规划，精确 transaction permit 让
`/cmd_vel_nav` 绕过 collision monitor；schema-v3 原有 yaw -> 横移 -> 纵移顺序
不变。其余 Nav2 goal 仍必须确认门/楼层观测、机械臂收回与 transaction-owned
精确 operating mode。
Nav2 goal 成功后还要确认实际轮速为零，才能推进状态。

## 7. 安全不变量

### 7.1 Motion hold

- hold 由 `(owner, transaction_id)` 唯一标识，多个 hold 组合生效；
- 只有精确 owner/transaction 能释放；
- hold 无 TTL，失败返回或 FSM 析构不能释放；
- 任一错误先 hold，再 cancel，再等待实际停稳；
- 启用障碍检查的阶段若门状态、障碍感知、Nav2 或实际停车证据不满足，不自动
  推进下一阶段；明确采用无障碍检测契约的接驳/轿厢段仍要求 Nav2 终态与实际停车。

### 7.2 Execution lease 兼容边界

- 新乘梯事务不建立、续期或释放 execution session；FSM 中对应 effect 仅为历史
  日志分类保留；
- `/safety/set_execution_lease` 和 failure-locked arbiter 仍为其他调用方及历史
  残留资源恢复保留，本次修改不删除其实现；
- 新乘梯开始前若发现遗留/外部 execution session，仍拒绝进入，避免旧状态与新
  transaction 并存；
- 精确 post-call transaction permit 可选择 `/cmd_vel_nav` 绕过 collision monitor，
  API 与 docking 速度仍被拒绝。

### 7.3 Operating mode

- `ELEVATOR_WAIT`、`DOORWAY`、`ELEVATOR_RIDE` 使用精确 owner/mission/lease；
- mode lease 需要 heartbeat，但 lease 过期回到 `NORMAL` 不等于允许运动；
- `robot_safety` 将本次乘梯 mode contract 锁存到显式 release；专用模式失效时保持
  零速，停车切图期间的续期失败不能取消已提交的 `FloorSwitch`；
- 不全局开放横移或倒车，不修改普通导航参数。

### 7.4 Localization correction pause

- 乘梯期间使用 owner-scoped correction pause 冻结新全局修正；
- pause 只拒绝新候选，bridge 继续发布最后接受的 `map -> odom`；
- legacy Bool 只能释放自己的 synthetic record，不能清除电梯 record；
- 到达目标层后先在 motion hold 下执行 `BeginFloorTransition`：FloorManager
  获取自己的 pause，并使普通 source runtime context 失效；
- begin 被权威确认后，电梯 owner 只能精确释放自己的 pause；FloorManager pause
  必须持续覆盖 target assets 加载；
- 只有 floor transaction 进入目标定位阶段后，FloorManager 才释放自己的 pause，
  且 bridge 只接受 target epoch 的显式定位；
- 如果其他 owner 仍持有 pause，FloorSwitch 必须失败锁定，不能全局清除。

### 7.5 Floor switch failure

恢复依赖按持久化阶段收口：事务日志必须先写入
`RUNTIME_EFFECT_INTENT=BEGIN_FLOOR_TRANSITION`，车端才可能提交
`FloorSwitch.action`。因此该意图之前的失败（例如前往呼梯位）只清理 Nav2、
事务资源和停车证据，不依赖尚未参与事务的 floor Action；出现该意图之后，
FloorSwitch endpoint、cancel response 与终态证明仍全部必需。阶段未知或旧日志
无法证明处于切层前时继续按已提交处理，不放宽安全边界。

完整运行链重启可能把一个已证明处于切层前的失败恢复到同一冻结电梯配置的另一
端点。此时清理允许使用当前已确认的源端或目标端，但必须同时精确匹配
building/floor/map/asset epoch/digest，并证明 localizer、bridge、唯一 TF、运行资源
空闲和双里程计停车。无关楼层仍拒绝；一旦日志表明可能提交过 FloorSwitch，仍只
接受原记录的外部楼层及 FloorSwitch 终态证明。

机器人物理上已乘梯到目标楼层后，不得在切层失败时自动恢复源楼层地图并宣称
运行上下文有效。

进入 map mutation 后任何失败都应表现为：

```text
FloorSwitchStatus.state = FAILED_LOCKED
active_context_valid = false
recovery_required = true
motion hold remains active
```

允许的后续动作只有：在 hold 下重试同一目标楼层事务，或等待人工恢复。只有
外部能够明确证明电梯从未移动且机器人仍在源层时，才可考虑源层补偿事务。

## 8. FloorSwitch 成功屏障

新 `FloorSwitch.action` 的成功不能只表示 service 返回或文件路径已保存。至少
同时要求：

- `building_id/floor_id/map_id` 与目标一致；
- asset digest 与目标版本一致；
- nav map、localizer map/params 和 filters 属于同一 asset epoch；
- map server 和 filter server 已加载目标资产；
- localizer 实际重载目标资产，而不是只缓存路径字符串；
- 显式定位结果带目标 epoch，并被 bridge 接受；
- `map -> odom` owner 唯一、发布新鲜；
- `safe_for_goal_start=true`；
- `correction_active=false`；
- current/target sequence 已收敛；
- AMCL/定位健康满足本项目 gate；
- costmap clear 后收到目标 epoch 下的新消息；
- Nav2 lifecycle 和 action server ready；
- motion hold 全程由调用方保持，floor manager 不自行释放。

当前 legacy floor-manager service 明确拒绝 `resume_navigation=true`，新的
Action 也只做 preflight，均不满足该成功契约，因此不能用于真实跨层出梯放行。

## 9. Mock 的允许范围

当前 mock 允许验证：

- FSM effect 的严格顺序；
- transaction 和 effect sequence 关联；
- 同时最多一个 active effect；
- 当前 transaction 的错误、取消和 effect-sequence 乱序进入 failure cleanup；
  外部 transaction 或迟到的旧回执只被忽略，不能推进新状态；
- schema-v1/v2 topology 标识符和角色完整性校验；
- v1 历史门槛完整性校验以及 v1 runtime fail-closed；
- v2 `hall_call -> landing -> cabin -> target landing` 串行 effect；
- mode、motion interlock 和 correction-pause 的纯仲裁逻辑；
- legacy pause 与 owner-scoped pause 的组合规则；
- floor transaction 的 pause 交接、source invalidation、target readiness 和
  retryable failure cleanup；
- mission/elevator/floor/safety/mode/pause 纯核心组合后的非运动跨层成功路径，
  包括最终无残留 goal/hold/lease/pause。

当前 mock 不提供：

- 真实 ROS elevator/mission action server；
- 真实或仿真的 Nav2 goal；
- 门打开稳定时长、消息 freshness 或视觉 confidence gate；
- 机械臂 stowed/fault/contact gate；
- 真实 floor switch；
- 电梯动力学、门关闭时序、轿厢振动或定位退化；
- 任何真实机器人运动许可。

隔离 ROS smoke 必须使用独立 `ROS_DOMAIN_ID`，并与 live runtime、底盘 bridge
和 `/cmd_vel` 隔离。它只证明接口和合成 topic 的 fail-closed 行为，不证明
真实停车距离或电梯安全。

## 10. 尚缺真实资产

### 10.1 每部电梯、每个楼层

- `hall_call`、`landing`、`cabin` 三个内部 pose 的实测坐标和 yaw；
- source/target `map_id`、asset digest 和版本关系；
- 电梯门净宽、轿厢净尺寸、门槛高度和 Ranger 完整 footprint 余量；
- 静态地图中可规划的门口/舱内区域；
- keepout、speed 和 binary filter 资产；
- 目标层厅外可可靠触发全局定位的可见区域；
- 门口、舱内和厅外的最大允许速度/加速度，经现场安全评估确认。

### 10.2 后续机械臂与视觉

- hall-call 和 cabin-panel 的机械臂作业位姿；
- 相机、机械臂、`base_link` 外参与版本；
- 按钮类别、楼层、方向和接触确认的映射；
- 机械臂 `STOWED/MOVING/PRESSING/FAULTED` freshness；
- 门状态、楼层显示、confidence 和 observation timestamp；
- 反光、暗光、遮挡、多人、电梯门部分开启等失败样本；
- 机械臂伸出时底盘强制 hold、底盘未停稳时禁止伸臂的双向互锁。

## 11. 后续实现顺序

1. 将 preflight-only `FloorSwitch.action` 接到现有
   `BeginFloorTransition` 围栏和纯 transaction core，并实现精确 bundle
   identity、真实 localizer reload、bridge/costmap evidence。
2. 为 elevator manager 增加 ROS node/action server 和异步 port adapter。
3. 为 mission manager 增加 ROS node/action server 和全局 mission/Nav2 所有权。
4. 将纯核心已有的 safety effect ack、稳定 cleanup ID 与失败重试语义映射到
   ROS adapter，并用隔离 launch test 验证。
5. 实现 fake Nav2、fake FloorSwitch、fake safety/mode/pause 的隔离 launch
   test，并覆盖精确 transaction-owned mode contract 与切图期间不取消契约。
6. 将 manager 加入 bringup，但保持真实电梯 motion disabled。
7. 接入 `PressButton.action`、`ElevatorObservation`、`ArmState`。
8. 增加 observation freshness、confidence、稳定时长、arm-stowed 和连续门状态
   gate。
9. 完成封闭测试区硬件验收后，才允许真实跨层任务。

## 12. 后续硬件验收门槛

### 12.1 上车前

- 全部单元测试和隔离 ROS 测试通过；
- 新 floor transaction 可在 mock 资产上重复成功并正确 failure lock；
- active 乱序/失败事件产生 `accepted=true` 的安全 effect，并可确认、可靠重试；
- hold/cancel 失败回执不会清除锁存的安全 effect；
- 图中每个服务/action server 唯一；
- `/cmd_vel`、`map -> odom`、`odom -> base_link` 发布者唯一；
- mission 活跃时外部导航目标被拒绝；
- 所有内部 elevator pose 不可作为普通配送点调用；
- mock adapter 在生产 launch 中默认关闭且不可自动回退。

### 12.2 空载、静止电梯

- 先只验证厅外停止、hold、按钮作业和门状态，不跨门槛；
- 再以最低批准速度单独验证一次进舱和一次出舱；
- 人为制造门关闭、视觉过期、低 confidence、arm fault、Nav2 abort、
  manager crash、lease expiry 和 floor-switch failure；
- 每种故障都必须在最终 `/cmd_vel` 观察到零，并保持 failure lock；
- Nav2 success 但实际轮速未停稳、门状态失效或安全层阻塞时不得推进；
- 目标层切图必须满足完整 readiness 后才能释放舱内 hold；
- Mission 成功回执同时证明目标 floor/map、`TARGET_HALL`、有效 runtime context、
  已提交 target asset epoch，以及无残留 hold/lease/pause；
- 记录 raw/nav/collision/safe/final command、wheel/local odom、TF owner、mode、
  hold、lease、pause、floor epoch 和 action transaction。

### 12.3 跨层重复验收

只有单次故障注入全部通过后，才能进行多轮跨层往返。每段必须确认：

- 一个且仅一个 Nav2 goal；
- 前一阶段明确终态后才开始下一阶段；
- 无 API terminal fallback 或内部 waypoint 二次速度修正；
- 无门口自旋/前进反复切换；
- 无横移模式残留；
- 无失效 observation 被接受；
- 无 floor/map/asset epoch 串层；
- 成功后无残留 hold、lease、pause、action 或探针；
- 失败后 hold 保留，且不会自动继续堆样本。

在完成风险评估、现场管理方批准和空载封闭测试前，不进入载物或公众环境验收。

## 13. 当前结论

本轮已完成“可测试的纯安全编排核心”和非运动跨包场景，但不是“可用电梯
功能”。机械臂与视觉可以后接而无需改写核心状态机；在 ROS action/port
adapters、live 原子 FloorSwitch、真实语义资产和硬件故障注入全部完成前，
任何 mock 或纯核心成功都不得触发真实电梯进出。
