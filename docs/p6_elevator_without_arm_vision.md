# P6 电梯链路：机械臂与视觉后接阶段

日期：2026-07-23
状态：接口、事务围栏与负向互锁收口阶段；**未授权真实电梯运动或真实切层**

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
   digest、canonical `sha256:<64 lowercase hex>` 地图绑定、拓扑、门槛和
   五类内部点；
4. 首选电梯为空时，按 `elevator_id` 稳定选择能精确服务 source/target
   floor+map 的电梯；
5. 返回按值冻结的 release、选中电梯、source/target 地图摘要、门槛和五类
   pose。任务期间 `current` 再切换也不会改变已返回对象。

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
- 楼层、电梯、门槛和内部 waypoint 的数据契约；
- owner-scoped 模式、运动 hold、执行 TTL 和定位修正暂停；
- 完整 footprint 进舱/出舱判定；
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
| `robot_elevator_manager` | 纯 C++ topology 与 YAML catalog loader、完整 footprint 门槛分类、事件驱动 FSM 和 GTest；核心已编排 `BEGIN_FLOOR_TRANSITION -> resume -> switch`，安全清理 effect 可确认和重试 | ROS 节点、action server、Nav2/floor/safety/mode adapter、真实 mock node |
| `robot_mission_manager` | 同层/跨层纯 C++ 串行 effect FSM 和 GTest；跨层 completion 校验 floor/map、asset epoch/digest、显式重定位、`TARGET_HALL`、上下文及残留 goal/hold/lease/pause；清理 ID 跨重试稳定 | ROS action server、Nav2/elevator adapter、任务恢复节点 |
| `robot_mode_manager` | owner/mission/lease 仲裁核心、服务节点、状态 heartbeat 和单元测试 | 生产 bringup 与所有 profile 消费者的实车验证 |
| `robot_safety` | owner-scoped motion hold、执行 TTL/failure lock、mode owner/mission/lease 交叉校验、电梯期仅允许 Nav2 正常源、状态主题、最终 Twist gate、单元测试和手动隔离脚本 | 精确 Nav2 goal 身份绑定、进程丢失和真实停车距离验收 |
| `robot_localization_bridge` | owner-scoped correction pause 与 legacy Bool 组合；`BeginFloorTransition` BEGIN/COMMIT/ABORT 围栏；typed `/localization/floor_health`；单元测试和隔离 ROS smoke | localizer generation、TF 唯一性和目标 asset epoch 的外部权威证明；实车切层验收 |
| `robot_floor_manager` | 旧服务仅允许 `resume_navigation=false` 选择；`resume_navigation=true` 零副作用拒绝；新增 preflight-only `FloorSwitch.action`、typed status、纯 C++ pause 交接/source invalidation/epoch/digest/readiness/失败锁定核心 | Action 到 bridge、真实 map/filter/localizer/costmap adapter 的 live 串接和目标 readiness 观测 |
| `robot_api_server` | 永久订阅 typed floor/health；事务活动、runtime invalid 或 `FAILED_LOCKED` 时阻止新导航、建图、地图/点位/禁行线/电梯配置写入、重定位、停靠、脱桩和 safety resume；stop/cancel 仍允许；旧 HTTP live-switch 旁路已关闭；已实现电梯配置草稿、校验、不可变发布、回滚和内部点隔离 | 跨节点 execution lease；完成切层后对请求解析代际的强一致 token；配置发布后的 live 应用 adapter |
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
  topology + threshold + elevator FSM
        |
        +--> Nav2 adapter ------------------------------+
        +--> robot_mode_manager                         |
        +--> robot_floor_manager                        |
        +--> robot_localization_bridge pause ownership  |
        +--> future arm / vision adapters               |
        +--> robot_safety hold / execution lease -------+
                                                        |
Nav2 -> velocity_smoother -> collision_monitor -> robot_safety
     -> /cmd_vel -> ranger_base
```

职责约束：

- mission manager 不识别门、不按按钮、不检查门槛、不切图；
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
-> 获取 execution lease/session
-> MOCK 呼梯
-> 设置 ELEVATOR_WAIT
-> 释放厅外 hold
-> 导航到 hall_wait
-> MOCK 等待源层开门
-> 设置 DOORWAY
-> 导航到源层 doorway
-> 导航到源层 cabin
-> 验证完整 footprint 已在舱内
-> 获取舱内 hold
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
-> 导航到目标层 doorway
-> 导航到目标层 exit
-> 验证完整 footprint 已在厅外
-> 获取出口 hold
-> 精确释放 mode lease
-> 精确释放 execution session
-> 释放出口 hold
-> COMPLETE
```

任何 active effect 失败、取消、乱序、序号不匹配或门槛判定失败都会进入
`FAILURE_CLEANUP` 并产生 `HOLD_AND_CANCEL`。所有非 `NONE` 安全 effect 都带
`accepted=true`；非法启动请求在任何运动前直接拒绝且不产生 effect。清理失败
会以新序号重发，只有确认 hold、唯一 Nav2 goal 已取消且停稳后才进入永久
`LOCKED`。这仍只是未来 ROS adapter 的指令，不代表 live 适配已经完成。

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
4. goal 接受前保持 hold；action 已接受且门、机械臂、模式、执行租约均有效后
   才可释放 hold。
5. goal 结束后先重新获取 hold，再检查停稳、footprint 和楼层状态。
6. cancel 未确认或停稳未确认时不得发送新 goal。
7. mission 活动时，普通 App 导航入口必须拒绝外部目标，避免 Nav2 preempt。
8. 电梯内部 waypoint 不经过 API 的 post-Nav2 速度修正或 API fallback。
9. manager 不发布 `/cmd_vel_api`、`/cmd_vel_collision_checked` 或 `/cmd_vel`。

## 6. 电梯拓扑与完整 footprint

每栋楼建议提供：

```text
maps_release/<building_id>/elevators.yaml
```

当前纯 topology 核心要求每个服务楼层恰好一个以下角色：

- `hall_call`
- `hall_wait`
- `doorway`
- `cabin`
- `exit`

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
资产摘要、地图边界、五角色、门槛几何以及 topology YAML 回读。有效草稿会
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

每层门槛需要：

- `left`、`right`：本层 map frame 中的门槛端点；
- `cabin_reference`：确定哪一侧是舱内；
- `clearance_m`：车体完全越过门槛的法向安全距离；
- `jamb_clearance_m`：车体相对左右门框的安全距离。

门槛判定使用已变换到当前 map frame 的完整机器人 footprint。设指向舱内的
单位法向量为 `n`、门槛上一点为 `p`，每个顶点为 `v_i`：

```text
d_i = n · (v_i - p)
```

- 所有顶点超过舱内 clearance，且全部位于门框净宽内，才是 `INSIDE`；
- 所有顶点超过厅外 clearance，且全部位于门框净宽内，才是 `OUTSIDE`；
- 跨线、接触 clearance band、门框越界、几何无效或 footprint 无效均为
  `STRADDLING`/invalid，并 fail closed。

不得只用 `base_link`、目标点到达、Nav2 result code 或车体中心越线代替完整
footprint 判定。有效 footprint 必须与 Nav2 共源并包含 padding，不能在电梯
包中维护另一套车体尺寸。

## 7. 安全不变量

### 7.1 Motion hold

- hold 由 `(owner, transaction_id)` 唯一标识，多个 hold 组合生效；
- 只有精确 owner/transaction 能释放；
- hold 无 TTL，失败返回或 FSM 析构不能释放；
- 任一错误先 hold，再 cancel，再等待实际停稳；
- footprint 为 `STRADDLING` 时不自动前进、后退或重试。

### 7.2 Execution lease

- 进入电梯专用运动前建立 execution session；
- adapter 以 steady-clock TTL 持续续租；
- manager 崩溃或 heartbeat 过期后，session 保持 engaged、lease 失效，
  `robot_safety` 进入 `EXECUTION_LEASE_MISSING` 并持续输出零；
- 普通 owner 不能接管 failure-locked session；
- 只有配置的 recovery owner 可建立恢复租约；
- 恢复后仍需显式关闭 session，旧 lease ID 不得复用。
- session 期间只允许正常 Nav2/collision-monitor 源，API 与 docking 速度被拒绝；
- mode heartbeat 必须与 execution owner/mission 一致；safety 同时约束 heartbeat
  age 与消息携带的 lease remaining，任一先到期即零速。

### 7.3 Operating mode

- `ELEVATOR_WAIT`、`DOORWAY`、`ELEVATOR_RIDE` 使用精确 owner/mission/lease；
- mode lease 需要 heartbeat，但 lease 过期回到 `NORMAL` 不等于允许运动；
- 专用模式失效时仍由 safety hold/execution failure lock 保持零速；
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
- topology 标识符、角色完整性和门槛几何校验；
- 完整 footprint 的 inside/outside/straddling 判定；
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

- 五个内部 pose 的实测坐标和 yaw；
- source/target `map_id`、asset digest 和版本关系；
- 左右门槛端点、舱内参考点、法向 clearance 和门框 clearance；
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
   test，并把跨节点 execution lease 覆盖到 API 最终提交窗口。
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
- Nav2 success 但 footprint 仍 `STRADDLING` 时不得推进；
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
