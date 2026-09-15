# 项目门控台账：作用、判定与恢复逻辑

审计日期：2026-09-09。审计对象：当前本地工作区源码、Jetson runtime overlay 配置及启动脚本；基准提交 `d9da8d1`，包含工作区未提交修改。

这是一份代码与配置说明，不是小车此刻的状态快照。未连接 Jetson 查询现场参数，未修改控制代码、部署或重启，也未触发任何运动。源码、安装产物和现场进程可能不是同一版本。

## 1. 统计口径与更正

本台账的“门控”指：对任务准入、控制权交接、速度输出、状态推进或定位数据接纳作出放行／拒绝／等待判断的独立机制。它不等于一把持久锁。

- 同一检查在导航、回桩、切图多个调用点复用，只列一个主条目；条目内部的时间、距离、身份等子条件不各算一个。
- 不同控制层虽可能都看“是否停车”，但决定的是不同交接，例如底盘能否换模与对桩任务能否宣布成功，分别列出并说明关联。
- 普通参数合法性检查、线程 `mutex`、测试断言不计数。已退休或只做诊断的机制放附录。
- 包含存在于代码但关闭、仅特定流程接入或尚待部署的机制；不能把条目数当成“现场启用数”或“正在拦车数”。

上一次答复的“机器人链路 81 类、加 HTTP 后 83 类”是初步分类。本次逐项复核后，将 `max_beam_error` 移出正式门控计数：本地证据不能证明它是一道独立的整个位姿硬拒绝门。正文收录 **80 项机器人链路门控／交接机制，加 2 项 HTTP 准入，共 82 项**。这是有明确范围的审计台账，不宣称穷尽第三方算法、机械臂、设备固件及全仓库每个分支。

| 分组 | 编号 | 数量 | 负责什么 |
|---|---|---:|---|
| 最终安全仲裁与底盘 | [S01–S19](#group-s) | 19 | 最终命令能否执行、控制源、方向许可、模式交接 |
| API 导航／回桩／地图准入 | [A01–A18](#group-a) | 18 | 能否提交任务、离桩前置、验收、事务证据 |
| Nav2 控制与检测 | [N01–N08](#group-n) | 8 | 起步、到点、进展、碰撞、恢复 |
| 近场对桩 | [D01–D05](#group-d) | 5 | 特征、接近、接触、刹停和退桩 |
| 楼层切换 | [F01–F03](#group-f) | 3 | 切图许可、原子提交与失败恢复 |
| 电梯和高层任务 | [E01–E04](#group-e) | 4 | 人工确认、取消屏障与恢复状态 |
| 定位、局部状态和启动 | [L01–L23](#group-l) | 23 | 数据接纳、TF、纠偏、启动就绪 |
| HTTP 服务准入 | [H01–H02](#group-h) | 2 | 鉴权和连接容量 |
| **合计** | | **82** | **不是 82 把持久锁** |

## 2. 如何阅读

每条均说明：作用、触发条件、拦截结果、放行／恢复方式、配置状态以及源码入口。

状态文字的含义：

| 标记 | 含义 |
|---|---|
| 默认启用 | 仓库启动默认会接入；尚未证明现场进程使用此配置 |
| 条件生效 | 仅在指定任务、控制源或状态下参与 |
| 默认关闭／被替代 | 实现还在，但默认路径不使用它或已换成另一实现 |
| 接入待确认 | 有代码或插件，但未确认当前默认启动路径、部署或调用 |
| 历史／非门控 | 不进入正式计数，不能据此断言它会拦车 |

配置取值应沿“源码默认 → 参数文件 → profile／环境变量 → 启动命令覆盖 → 现场进程参数”确认。不能只看 YAML 里的一个 `false`。例如 AMCL profile 和真实切图许可存在启动脚本覆盖。

恢复方式也不能混淆：

- **自动恢复**：新数据或新许可满足条件后，下一周期重新判断；不等于已经失败的任务会自动重新执行。
- **流程恢复**：必须完成标准离桩、取消确认、切图提交或重定位等流程，才能解除对应阻断。
- **失败关闭**：超时后失败或继续保持零速，不自动放行。
- **超时放行**：达到预算后允许继续；仅解除本条等待，其他安全条件仍须通过。

普通导航通常依次经过 API 准入、Nav2 控制、速度平滑、碰撞监测、`robot_safety` 和底盘模式握手。门控之间不是简单的“全部 AND 后输出一个总错误”：不同阶段分别判定，不同控制通道也不同。

尤其注意：`/safety/status=OK` 只代表该节点的主状态检查通过。零速优先窗口、方向裁剪、spin settle、上游控制器或底盘握手仍可能令车辆不动。必须同时看当前控制源、命令话题、许可和底盘实际模式。

<a id="group-s"></a>

## 3. 最终安全仲裁与底盘（S01–S19）

本组主要实现位于 `robot_safety`。S18、S19 位于接收最终 `/cmd_vel` 的 `ranger_base`。通用配置见 [robot_safety.yaml](../scripts/jetson/runtime_overlay/config/robot_safety.yaml)。

<a id="s01"></a>

### S01 急停

- **作用**：急停生效时禁止软件运动命令到达底盘。
- **触发条件**：`/safety/estop` 将 `estop_active_` 置为真；它在主仲裁顺序中先于其他条件。
- **拦截结果**：发布零速度，状态为 `ESTOP_ACTIVE`；其他许可不能越过此检查。
- **放行／恢复**：收到解除急停状态后重新评估其余门控；解除急停本身不自动重启失败任务。
- **配置状态**：默认接入，无按时间自动消除的逻辑；话题可配置。
- **源码**：[current_snapshot](../src/robot_safety/src/robot_safety_node.cpp#L954)、[on_estop](../src/robot_safety/src/robot_safety_node.cpp#L1305)。

<a id="s02"></a>

### S02 按拥有者和事务管理的暂停运动（motion hold）

- **作用**：让任务在切换／清理阶段暂停运动，避免一个调用者误解除另一个调用者的暂停。
- **触发条件**：`MotionInterlockArbiter` 中有任意 `(owner, transaction_id)` hold；申请和释放带递增序号，迟到或重用序号的命令会被拒绝。
- **拦截结果**：`MISSION_MOTION_HOLD`，最终输出零。可以同时存在多个 hold，释放一个不代表全部解除。
- **放行／恢复**：拥有者释放其精确 hold，所有 hold 清除后才通过本条。恢复接口还要求观察到的 `generation` 未变化、执行会话已空闲及更新的命令序号。hold 本身没有自动到期解除。
- **配置状态**：服务默认提供；没有任务申请时不拦截。不能把线程互斥锁当成此业务 hold。
- **源码**：[apply_hold_locked](../src/robot_safety/src/motion_interlock_arbiter.cpp#L38)、[release_hold_if_execution_idle_locked](../src/robot_safety/src/motion_interlock_arbiter.cpp#L208)、[主仲裁](../src/robot_safety/src/robot_safety_node.cpp#L960)。

<a id="s03"></a>

### S03 执行会话租约

- **作用**：执行会话开始后必须由合法拥有者持续持有租约，避免失联任务继续运动。
- **触发条件**：会话已经 `engaged`，但租约缺失或到期；拥有者、任务、事务和 lease ID 还用于拒绝串任务操作及迟到续约。
- **拦截结果**：`EXECUTION_LEASE_MISSING`，零速；到期使租约失效但不会把执行会话自动变成未开始。
- **放行／恢复**：有效租约在到期前可续期；已过期的旧 lease ID 不能直接复活，需要合法恢复／会话释放。精确会话释放后，本条不再要求旧租约。
- **配置状态**：条件生效。当前电梯测试工作流不申请这个旧通用执行租约，不能说每次乘梯都必须有它；S04 的电梯模式契约仍可能生效。
- **源码**：[apply_execution_locked](../src/robot_safety/src/motion_interlock_arbiter.cpp#L316)、[expire_locked](../src/robot_safety/src/motion_interlock_arbiter.cpp#L444)、[当前接口说明](../src/robot_safety/README.md#L20)。

<a id="s04"></a>

### S04 特殊运行模式契约

- **作用**：确认当前电梯／恢复特殊模式属于正确任务，而且模式租约仍有效。
- **触发条件**：执行会话或已记录的电梯契约要求检查，但模式状态未到、新鲜度不合格、租约过期，或拥有者／任务不匹配。无旧执行会话时，电梯分支要求 `robot_elevator_manager`、合法 `elevator_…` mission 及相应事务匹配。
- **拦截结果**：`EXECUTION_MODE_INVALID`，零速。模式状态存在不等于授权有效。
- **放行／恢复**：匹配任务刷新有效模式状态／租约，或通过正常清理退出特殊契约；旧任务的许可不能放行新任务。
- **配置状态**：特殊模式条件生效；普通导航未进入特殊契约时不会凭空要求电梯租约。
- **源码**：[execution_contract_required / execution_mode_contract_valid](../src/robot_safety/src/robot_safety_node.cpp#L834)、[主仲裁](../src/robot_safety/src/robot_safety_node.cpp#L972)。

<a id="s05"></a>

### S05 执行任务的命令来源保护

- **作用**：合法电梯执行期间，防止 API 遥控／对桩通道抢走该执行任务的控制。
- **触发条件**：需要且已满足特殊执行契约，但输入源不是该链路认可的 `NORMAL` 导航源。
- **拦截结果**：`EXECUTION_SOURCE_BLOCKED`，该来源命令被置零。
- **放行／恢复**：使用当前任务的合法导航来源，或正常结束该执行契约；增加速度或刷新错误来源不会解除。
- **配置状态**：条件生效；不是“所有时候 API 都不能控制”。
- **源码**：[current_snapshot](../src/robot_safety/src/robot_safety_node.cpp#L978)。

<a id="s06"></a>

### S06 最终仲裁中的定位健康检查

- **作用**：可选择在最终速度出口直接根据定位健康停机。
- **触发条件**：`require_localization_health=true` 且收到的定位健康为失败。
- **拦截结果**：`LOCALIZATION_INVALID`，零速。
- **放行／恢复**：健康变为正常后重新评估；本条关闭不等于上游定位／TF／API 检查也关闭。
- **配置状态**：仓库 overlay 为 **false**，因此当前默认不由本条直接拦车。
- **源码／配置**：[主仲裁](../src/robot_safety/src/robot_safety_node.cpp#L985)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L64)。

<a id="s07"></a>

### S07 在桩时禁止普通运动

- **作用**：避免已接触充电桩或仍被认定在桩时，普通导航自旋／移动碰到桩体。
- **触发条件**：`block_normal_motion_when_docked=true`，并有新鲜 BMS 接触、`docked/charging` 状态或有效持久在桩证据之一。强持久证据不会仅凭一次“无接触”自动失效；弱证据可以被新鲜无接触且非在桩状态反证。
- **拦截结果**：普通运动为 `DOCKED_CONTACT_BLOCK` 并置零。配置允许新鲜 docking 通道例外，但该通道仍经过 S16。
- **放行／恢复**：通过标准离桩和状态收敛，或在明确离开桩区时走 A09、A10、S17 的受限恢复。仅等待 BMS 数据过期不等于清除强持久证据。
- **配置状态**：默认开启；BMS、docking status、持久文件三个证据开关均 true。
- **源码／配置**：[dock_contact_active](../src/robot_safety/src/robot_safety_node.cpp#L2082)、[强弱持久证据](../src/robot_safety/src/dock_contact_policy.cpp#L91)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L65)。

<a id="s08"></a>

### S08 最终速度命令看门狗

- **作用**：上游不再刷新命令时停止执行旧运动。
- **触发条件**：主命令年龄超过 `watchdog_timeout_sec`；定时器再次检查状态。
- **拦截结果**：`COMMAND_STALE`、零速。API、docking、电梯旁路还各有更短的新鲜度窗口，属于该命令链的新鲜度子条件。
- **放行／恢复**：收到新的合法命令后重新判定；不会自动恢复已取消或失败的上游任务。
- **配置状态**：默认 1.0 秒；API／docking 优先窗口和电梯旁路命令窗口均为 0.25 秒。
- **源码／配置**：[current_snapshot](../src/robot_safety/src/robot_safety_node.cpp#L995)、[命令新鲜度](../src/robot_safety/src/robot_safety_node.cpp#L1113)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L4)。

<a id="s09"></a>

### S09 多通道命令优先级

- **作用**：阻止 Nav2 零速、API 速度与精对桩速度互相覆盖。
- **触发条件**：有新鲜 docking 命令时，API／普通命令不能抢占；有新鲜 API 命令时，普通命令让位。电梯合法旁路激活时另按 S11 选择输入流。
- **拦截结果**：低优先级回调忽略该输入，不代表整车必须停；高优先级命令可能仍在执行。
- **放行／恢复**：高优先级来源结束或超过其优先窗口后，后续低优先级新命令可重新成为来源；不排队重放历史命令。
- **配置状态**：API 和 docking 优先级均默认 true，窗口均 0.25 秒。
- **源码／配置**：[on_normal_cmd / on_api_cmd / on_docking_cmd](../src/robot_safety/src/robot_safety_node.cpp#L1264)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L9)。

<a id="s10"></a>

### S10 零速度优先窗口

- **作用**：收到合法停止命令后短时保持停止，抵御队列中迟到的非零命令。
- **触发条件**：有资格停止当前控制源的近零命令开启零速窗口；较低优先级来源的零速不能无条件覆盖 docking。
- **拦截结果**：窗口内的非零命令也输出零；主 `/safety/status` 仍可能是 `OK`。
- **放行／恢复**：窗口自然结束，且没有新的合法停止命令延长窗口，再处理新命令。持续收到有效零命令会继续保持。
- **配置状态**：默认 true，窗口 0.25 秒、近零 epsilon 为 0.0001。
- **源码／配置**：[handle_zero_priority_command](../src/robot_safety/src/robot_safety_node.cpp#L1204)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L21)。

<a id="s11"></a>

### S11 电梯 collision_monitor 旁路许可

- **作用**：只让精确电梯事务在指定阶段使用 `/cmd_vel_nav`，而不是普通 `/cmd_vel_collision_checked`。
- **触发条件**：许可无事务 ID、超过 0.75 秒、hold 未解除、特殊模式契约无效、owner／mission 不匹配或不在 `DOORWAY` 时，旁路不获授权。
- **拦截结果**：无授权的旁路输入被忽略；已有旁路失效时清缓存并零速，模式契约可能同时触发 S04。不能理解成失效后自动重放普通导航运动。
- **放行／恢复**：当前事务刷新匹配许可并满足模式／hold 条件，切换阶段先输出零，再接收新鲜命令。急停、方向许可和 BMS 等并未被该旁路取消。
- **配置状态**：电梯专用、按事务生效；许可 0.75 秒，输入命令 0.25 秒。与 N06 的普通碰撞检测关联但不重复计数。
- **源码／配置**：[elevator_entry_collision_bypass_authorized](../src/robot_safety/src/elevator_entry_collision_bypass_policy.cpp#L8)、[定时失效处理](../src/robot_safety/src/robot_safety_node.cpp#L1339)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L14)。

<a id="s12"></a>

### S12 倒车许可

- **作用**：把倒车授权绑定到命令来源，普通导航不能擅自继承遥控或离桩的倒车权。
- **触发条件**：命令 `linear.x < 0`，但相应来源无有效倒车许可。电梯特殊契约只允许 NORMAL 加终端 reverse permit；docking 必须用 docking reverse permit；API 可用 legacy／teleop；普通导航可用 legacy／terminal。
- **拦截结果**：整条 Twist 置零，不只是把负 X 改成零；获准后还受该来源速度上限约束。
- **放行／恢复**：任务持续刷新自己的许可；许可过期即失效。`allow_reverse=false` 不表示所有授权离桩均被禁止。
- **配置状态**：默认全局 `allow_reverse=false`；动态许可有效期 0.75 秒。普通导航倒车上限 0.08 m/s，电梯专用 0.40 m/s。
- **源码／配置**：[reverse_allowed](../src/robot_safety/src/robot_safety_node.cpp#L1611)、[命令裁剪](../src/robot_safety/src/robot_safety_node.cpp#L1790)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L48)。

<a id="s13"></a>

### S13 横移许可

- **作用**：限制哪些来源可以发横移命令，避免普通 Ackermann 导航误切平移模式。
- **触发条件**：docking 以外的来源，API 横移开关未允许，或普通导航没有新鲜 terminal lateral permit。
- **拦截结果**：`linear.y` 被清零，其他合法轴不一定一起清零；不同于 S12 的整条置零。
- **放行／恢复**：API 按配置允许，或普通导航刷新终端横移许可；docking 的通道许可仍不能越过 S16 接触保护。
- **配置状态**：API 横移默认允许，上限 0.10 m/s；普通导航 0.05 m/s、电梯专用 0.40 m/s，终端许可同样有时效。
- **源码／配置**：[sanitize_command_for_mode_contract](../src/robot_safety/src/robot_safety_node.cpp#L1817)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L53)。

<a id="s14"></a>

### S14 自旋转直行的尾旋稳定等待

- **作用**：底盘自旋尚有惯性尾转时，暂缓接续平移。
- **触发条件**：从新鲜底盘反馈确认实际进入 SPINNING 后标记 pending；后续要执行平移而轮速／IMU 稳定证据不足时等待。单凭低速 Twist 不会触发一次新的自旋事件。
- **拦截结果**：该平移命令输出零；纯旋转／停止不消耗 pending。主安全状态可能仍为 `OK`。
- **放行／恢复**：轮速与 IMU 达到稳定要求后自动放行；**等待达 2.0 秒时会告警并放行本条命令**，属于有界软等待，不是永久失败锁；其他门控仍会检查。
- **配置状态**：默认 true；轮速角速度阈值约 1.15°/s、连续 5 样本；IMU 约 2.01°/s、稳定 0.30 秒。附加 local odom 稳定要求为 false，IMU 要求为 true。
- **源码／配置**：[apply_spin_to_drive_settle_gate](../src/robot_safety/src/robot_safety_node.cpp#L1980)、[实际自旋事件](../src/robot_safety/src/robot_safety_node.cpp#L1531)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L24)。

<a id="s15"></a>

### S15 横移模式退出保护

- **作用**：实际仍在 PARALLEL 时，不直接把直行／转弯命令按正常已换模状态执行。
- **触发条件**：非 docking 输入请求双 Ackermann 驱动，而新鲜底盘反馈仍是横移模式。
- **拦截结果**：等待期构造受限直行探测请求，由 S18 在底盘确认前保持物理零速；**超过 1.0 秒则本层保持零速**。与 S14 的“超时放行”不同。
- **放行／恢复**：实际模式离开横移后恢复正常命令；docking 源不走这道外层保护，但仍走底盘握手。定时器仅在普通流不新鲜时做横移退出处理，不能对新鲜横移流持续插零。
- **配置状态**：默认 true；状态新鲜度 0.5 秒，探测速度配置 0.06 m/s。该探测值不是绕过底盘握手的运动许可。
- **源码／配置**：[apply_mode_exit_guard](../src/robot_safety/src/robot_safety_node.cpp#L1867)、[定时器](../src/robot_safety/src/robot_safety_node.cpp#L1399)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L43)。

<a id="s16"></a>

### S16 BMS 接触后的精对桩内存互锁

- **作用**：充电接触出现后立即停止继续顶桩，并防止后续旧命令继续前推、横移或自旋。
- **触发条件**：新鲜 BMS 接触上升沿立即停止；在 docking 命令／docked 状态／强持久在桩证据上下文中接触会设置内存 latch。有效互锁由内存 latch、新鲜接触或强持久证据共同决定。
- **拦截结果**：非零 docking 命令被禁止，只有精确停止，或持有效离桩许可的纯负 X 倒车可以通过。BMS 变旧不自动清除内存 latch。
- **放行／恢复**：标准离桩期间观察到 reverse session，随后明确关闭 reverse permit，且有新鲜无接触反馈，才自动清内存 latch；单纯遥控移走没有这段会话证据。另一条受限清理路径为 S17。持久文件仍需上层正确收敛。
- **配置状态**：`bms_docking_interlock_enabled=true`，允许标准倒车离桩 true；与 S07 保护的通道不同。
- **源码／配置**：[BMS 更新与命令判定](../src/robot_safety/src/robot_safety_node.cpp#L1455)、[try_release_bms_docking_interlock](../src/robot_safety/src/robot_safety_node.cpp#L1563)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L75)。

<a id="s17"></a>

### S17 不运动清除残留 BMS 内存锁的证据门控

- **作用**：机器人已明确离开桩区时，允许受限清除残留内存锁，而非无条件清锁。
- **触发条件**：调用 reconcile 时，上层没有提供离开桩区的证据，BMS 不新鲜／仍接触／无接触稳定不足，docking 仍报在桩，离桩许可仍有效，或仍有新鲜 docking 命令，任一项都会拒绝。
- **拦截结果**：服务返回对应失败码，内存锁保持；不产生倒车、不绕过在桩保护。
- **放行／恢复**：上层提供位置证明，safety 独立通过上述电气／运行证据检查后才清私有内存 latch；无接触稳定要求为 3.0 秒。API 负责几何判断和持久文件，safety 不自行计算离桩距离。
- **配置状态**：服务默认接入。上层还要等安全状态实际发布为清除；此门控不是“所有导航必须再等 3 秒”。
- **源码／配置**：[on_reconcile_dock_interlock](../src/robot_safety/src/robot_safety_node.cpp#L680)、[evaluate_dock_interlock_reconcile](../src/robot_safety/src/dock_contact_policy.cpp#L123)、[配置](../scripts/jetson/runtime_overlay/config/robot_safety.yaml#L77)。

<a id="s18"></a>

### S18 底盘实际模式切换握手

- **作用**：CAN 速度发送前，确认硬件实际模式已匹配所需模式，而非只凭软件发过切换命令。
- **触发条件**：实际模式不匹配或 `mode_changing` 尚未结束；先要求新反馈证明线速度和角速度稳定，再申请并等待模式确认。
- **拦截结果**：底盘直接发送零运动，非零 Twist 不进入 SDK 运动发送分支。状态经历 STOPPING、WAITING_ACK，超时标记 MODE_SWITCH_TIMEOUT。
- **放行／恢复**：只有反馈确认 `actual_mode == desired_mode` 且非切换中，后续新 Twist 才释放。2.0 秒超时后**仍保持零并重试模式请求**，不是超时即强行运行。
- **配置状态**：`run_ranger_chassis.sh` 默认握手 true；停稳 0.15 秒，线速度阈值 0.02 m/s，角速度约 1.72°/s，模式请求重试周期 0.10 秒。
- **源码／配置**：[EnsureMotionModeReady](../src/ranger_base/src/ranger_messenger.cpp#L796)、[TwistCmdCallback](../src/ranger_base/src/ranger_messenger.cpp#L549)、[启动脚本](../scripts/jetson/runtime_overlay/scripts/run_ranger_chassis.sh#L35)。

<a id="s19"></a>

### S19 自旋停止命令的模式保持

- **作用**：自旋尚未停稳时收到零命令，保持 SPINNING 接收零速，避免马上退回 Ackermann 导致额外换模。
- **触发条件**：输入已经近零，当前／最近请求为 SPINNING，且反馈缺失、模式仍在切换或角速度超过约 1.72°/s。
- **拦截结果**：保持停止命令的自旋模式，不是把一个非零旋转命令截成数段；它控制的是停止后的模式交接。
- **放行／恢复**：有稳定停止反馈后，后续零命令可以请求回默认模式，并继续通过 S18 的硬件确认。本条没有单独的“等几秒直接认为停稳”计时器。
- **配置状态**：启动脚本默认 true；属于交接机制，与 S14 平移等待不是同一条。
- **源码／配置**：[ShouldHoldZeroCommandInSpinningMode](../src/ranger_base/src/ranger_messenger.cpp#L989)、[零命令模式选择](../src/ranger_base/src/ranger_messenger.cpp#L568)、[启动脚本](../scripts/jetson/runtime_overlay/scripts/run_ranger_chassis.sh#L30)。

<a id="group-a"></a>

## 4. API 导航、对桩与换层编排门控（18 项）

本组按独立阻断机制计数，不把同一机制在 HTTP 入口、后台工作线程、动作发送前的重复检查分别计数。`robot_safety` 的急停、许可、BMS 锁本体不在本组重复计数；A09～A11 描述 API 消费这些证据后的独立准入与编排。配置状态均指仓库文件，不代表现场进程：启动脚本允许通过 [`ROBOT_API_SERVER_CONFIG`](../scripts/jetson/runtime_overlay/scripts/run_robot_api_server.sh#L15) 覆盖整个配置。

<a id="a01"></a>

### A01 运行任务及模式互斥准入

- 作用：避免建图、普通导航、对桩和地图选择等流程同时争用机器人运行环境；也避免一个导航目标尚未结束就叠加第二个目标。
- 触发/拒绝条件：普通导航目标遇到建图正在运行/启动，或已有导航目标运行；导航栈启动遇到建图、对桩、模式转换或目标任务未空闲；离线选图遇到运行栈或任务仍活跃。不同接口有各自的忙碌集合，并非所有任务两两无条件互斥。
- 阻断结果：拒绝新的目标、启动或离线地图选择请求，通常返回 HTTP 409；不因这次拒绝自动终止原任务，也不等同于持续发布零速。
- 放行/恢复逻辑：原任务正常结束，或经所属取消/停止流程真正完成后，重新请求并重新检查；某次 HTTP 超时不证明原任务已经空闲。
- 当前仓库配置状态：这些业务检查未见独立关闭开关；运行状态来源包含建图/导航启动命令与运行上下文，见[配置](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L61)。这是源码固定准入，不是普通线程 mutex 计数。
- 源码函数：[NavigationModule::Impl::handle_goal](../src/robot_api_server/src/features/navigation/navigation_module.cpp#L489)、[handle_start](../src/robot_api_server/src/features/navigation/navigation_module.cpp#L1211)、[FloorSwitchModule::Impl::offline_runtime_busy_detail](../src/robot_api_server/src/features/floor_switch/floor_switch_module.cpp#L354)。

<a id="a02"></a>

### A02 地图与 keepout 完整性降级阻断

- 作用：地图/禁行区资产更新失败或证明不完整时，防止机器人继续把不可信地图当作可安全使用的地图。
- 触发/拒绝条件：`maps_module_.integrity_degraded()` 为真；其来源包含内存降级标志和持久化资产完整性标记。keepout 修改会在首次写入前先落盘降级锁，成功证明后再清除。
- 阻断结果：导航目标、导航启动、对桩准入和换层入口拒绝，相关入口返回 HTTP 503；本门主要阻止新操作，不单独承担底盘持续零速职责。
- 放行/恢复逻辑：执行资产修复/更新事务并重新证明正确地图身份、非 keepout 内容未漂移及相关投影后，成功路径才清标志。持久化标记存在时，单纯等待或重启 API 不是可靠解锁办法。
- 当前仓库配置状态：未见绕过完整性检查的开关；持久标记位于配置的地图根目录，见 [`maps_root`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L8)。是否已触发降级需读取现场状态，本次未读取。
- 源码函数：[NavigationModule::Impl::handle_goal](../src/robot_api_server/src/features/navigation/navigation_module.cpp#L482)、[MapsModule::Impl::map_asset_integrity_degraded](../src/robot_api_server/src/features/maps/maps_module.cpp#L935)、[keepout 提交证明与清锁](../src/robot_api_server/src/features/maps/maps_module.cpp#L3224)。

<a id="a03"></a>

### A03 运行地图上下文及目标楼层一致性

- 作用：避免使用“用户已选中、但运行栈尚未真正切换完成”的地图，或把其他楼层的保存点送给当前地图上的导航。
- 触发/拒绝条件：保存点导航读取已确认运行地图时仍存在 pending 上下文，或已确认楼宇/楼层与请求不一致；对桩要求运行上下文 `confirmed=true` 且 `state=ready`，请求地图必须为该楼层 active manifest，并与运行地图匹配。普通直接坐标目标不经过完全相同的保存点楼层比对。
- 阻断结果：拒绝本次导航/对桩准入，返回 409 或 503，并要求完成地图/楼层切换；不会靠改请求中的楼层字符串替换运行地图。
- 放行/恢复逻辑：通过正式地图选择、运行栈启动或换层流程建立并确认正确上下文，再重试；pending 超时不自动产生 ready 证明。
- 当前仓库配置状态：未见总禁用开关；[`runtime_map_context_file`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L71) 指定上下文文件，启动中上下文 TTL 为 300 秒，但 TTL 不是就绪许可。
- 源码函数：[NavigationModule::Impl::handle_goal](../src/robot_api_server/src/features/navigation/navigation_module.cpp#L568)、[DockingHttpModule::Impl::handle_start](../src/robot_api_server/src/features/docking/lifecycle/docking_http_module.cpp#L195)。

<a id="a04"></a>

### A04 API 消费端 bridge 定位就绪与纠偏稳定门

- 作用：发目标、做最终位姿验收或进入精对桩前，确认使用的是 canonical `map->odom`，且没有尚未完成的定位切换/纠偏。
- 触发/拒绝条件：bridge 状态缺失、`map->odom` 不存在或发布 owner 不是 `robot_localization_bridge`、定位降级、纠偏暂停/冻结、AMCL correction pending/not ready，或 `safe_for_goal_start=false`；最终验收还要求 `correction_active=false`。代码允许满足明确证据的 AMCL 静止待命例外；fine 对桩另有“canonical bridge 已稳定且残差足够小”的 `AMCL_NOT_TRACKING` 例外，不能简化成“AMCL 非 tracking 一律拒绝”。
- 阻断结果：目标发送前等待，期限内未就绪则任务 `failed_goal_start_readiness`，不向 Nav2 发目标；最终验收/精对桩等待期间可发布零速，失败则不通过对应交接或验收。
- 放行/恢复逻辑：新的 bridge 状态满足该上下文的就绪条件后，正在等待的流程自动继续；超过等待期限则该次流程失败，需要解决定位问题并按流程重试。普通导航不会因为这个失败自行强制重定位或 force-accept。
- 当前仓库配置状态：目标前就绪检查是固定路径；[最终验收等待](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L285)、[yaw 等待](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L366)、[fine 等待](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L183) 均配置为 true。普通目标自动重定位配置为 [false](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L259)。
- 源码函数：[evaluate_bridge_readiness](../src/robot_api_server/src/features/navigation/runtime/navigation_bridge_wait.cpp#L38)、[NavigationGoalExecutor::wait_for_goal_start_readiness](../src/robot_api_server/src/features/navigation/mission/navigation_goal_executor.cpp#L86)、[PredockControlModule::bridge_safe_for_fine_entry](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L216)。

<a id="a05"></a>

### A05 Nav2 结果后的最终位姿验收

- 作用：把“Nav2 返回结果”与“配送点确实到达”分开，避免仅凭 action success 就宣告业务完成。
- 触发/拒绝条件：最终地图位姿不可用、bridge 未稳定、位置或要求的朝向未达标，或严格末端纠偏仍待完成；`position_only` 与 `pose_required` 的朝向要求不同。接受 slack、有限重试和末端纠偏都受策略约束，不是任意放宽误差。
- 阻断结果：不把任务标为完成；位姿缺失可进入 `failed_final_pose_verify`，有限纠偏/重试后仍不合格进入 `degraded_final_pose_verify`。也可在 Nav2 abort 后通过经过验证的有限补偿取得真正成功。
- 放行/恢复逻辑：当前任务在允许次数内自动重试/纠偏，重新验收通过才设置 `task_complete`；预算耗尽或证据不足不能靠时间流逝变成成功，需要处理原因后重新执行任务。
- 当前仓库配置状态：[`post_nav2_final_verify_enabled=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L285)，最多 3 次 final verify 重试；位置成功阈值 0.06 m、slack 0.02 m，默认完成策略为 [pose_required](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L276)。这描述当前配置，不宣称改一个开关即可绕过全部最终审计分支。
- 源码函数：[NavigationGoalExecutor::run 中商业完成重算](../src/robot_api_server/src/features/navigation/mission/navigation_goal_executor.cpp#L653)、[不可用/不合格结果处理](../src/robot_api_server/src/features/navigation/mission/navigation_goal_executor.cpp#L839)。

<a id="a06"></a>

### A06 API 末端纠偏 costmap 路径检查

- 作用：API 自行输出小范围末端纠偏速度时，先检查拟移动方向的局部栅格证据；它是额外保护，不替代 collision monitor 或完整车体碰撞检查。
- 触发/拒绝条件：local costmap 或同坐标系机器人位姿不可用/过期、栅格结构无效，或沿前向/横向探测线采样遇到越界、未知栅格、达到占用阈值的栅格。
- 阻断结果：返回 `clear=false`，阻止这次末端移动/使纠偏分支失败；不将未知区当自由区，不自行删除障碍数据。
- 放行/恢复逻辑：只有后续取得新鲜有效的 costmap/位姿且探测路径无阻挡，新的检查才通过；已经失败的任务需按其重试流程处理，等待超时不是越障许可。
- 当前仓库配置状态：[`navigation_terminal_recovery_costmap_guard_enabled=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L313)，证据最大年龄 0.50 秒，占用阈值 50，lookahead 0.15 m；源码确有关闭后直接放行的分支。
- 源码函数：[NavigationTerminalControl::costmap_path_clear](../src/robot_api_server/src/features/navigation/terminal_control/navigation_terminal_control.cpp#L209)。

<a id="a07"></a>

### A07 物理停车与末端稳定交接门

- 作用：区分“发过零速”和“底盘实际停稳”，防止 Nav2 尚有机械运动时交给对桩，或末端微调后立即以滑移中的姿态宣布到位。
- 触发/拒绝条件：轮速 odom 缺失/过期、线速度或角速度仍超阈值、稳定持续时间不足；要求模式证据的末端分支还检查新鲜实际模式为代码 0 且 mode_aligned。Nav2→对桩交接显式传 `require_dual_ackermann_mode=false`，所以该交接只强制实际停稳，不强制这项模式证明。
- 阻断结果：输出零速/释放特殊模式后等待；硬物理稳定门失败则不启动对桩管理器，或将末端纠偏标为 blocked。注意：单纯 final-yaw 和 API predock-yaw 的角速度等待是较弱分支，超时后仍可能因新鲜位姿复验通过而成功，不能把它们写成“未等到角速度稳定永不放行”。
- 放行/恢复逻辑：期限内取得连续停稳证据即自动继续，之后还复验位姿；硬门期限结束仍不满足则本次交接/纠偏失败，需要处理底盘或反馈后重试。弱 yaw 分支的超时复验例外不等于解除所有安全锁。
- 当前仓库配置状态：[terminal settle](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L318) 为 true：线速≤0.01 m/s、角速≤约1.15°/s（0.02 rad/s）、稳定≥0.30 秒、odom≤0.20 秒、超时2.50秒；[yaw actual-stop](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L356) 也为 true，但逻辑强度如上。
- 源码函数：[NavigationTerminalRuntimeModule::Impl::actual_stop_stable_snapshot](../src/robot_api_server/src/features/navigation/terminal_control/navigation_terminal_runtime_module.cpp#L504)、[run_lateral_correction 硬失败](../src/robot_api_server/src/features/navigation/terminal_control/navigation_terminal_control.cpp#L437)、[DockingJobExecutor 交接停车门](../src/robot_api_server/src/features/docking/lifecycle/docking_job_executor.cpp#L579)、[run_final_yaw_motion 超时复验例外](../src/robot_api_server/src/features/navigation/terminal_control/navigation_terminal_control.cpp#L762)。

<a id="a08"></a>

### A08 API 末端动作位姿新鲜度与位移包络

- 作用：末端旋转/小范围补偿只能使用可信的实时位姿，并限制原本为修正朝向而产生的额外平移。
- 触发/拒绝条件：final-yaw 当前位姿不可用或不在 map 帧、开启新鲜度要求时位姿过旧、相对动作起点的 XY 漂移超过配置上限；停车后的位姿复验再次检查同样条件。侧向纠偏也会检查其有限恢复范围。
- 阻断结果：停止该末端动作并输出零速收尾，标记 `failed_final_yaw_align` 或 `failed_final_pose_verify`；不会用旧 TF 外推成“已到位”。
- 放行/恢复逻辑：动作过程中每轮都重新获取证据；一旦已经以 blocked 退出，需要任务级有限重试或新的恢复/导航流程，不能仅清显示状态继续原动作。
- 当前仓库配置状态：[`navigation_final_yaw_align_require_fresh_pose=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L348)，允许 XY 漂移 0.08 m；[机器人位姿新鲜度](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L202) 为0.5秒；通用末端恢复距离为[0.40 m](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L296)。
- 源码函数：[NavigationTerminalControl::run_final_yaw_motion](../src/robot_api_server/src/features/navigation/terminal_control/navigation_terminal_control.cpp#L726)、[停车后漂移复验](../src/robot_api_server/src/features/navigation/terminal_control/navigation_terminal_control.cpp#L776)。

<a id="a09"></a>

### A09 导航前 dock 安全状态新鲜度门

- 作用：API 不能在没有收到最新在桩安全状态时猜测“应该没接触充电桩”。这与 robot_safety 自身的接触锁是两层不同检查。
- 触发/拒绝条件：配置要求 dock safety interlock 状态，且状态从未收到或已过期；此时 `pre_navigation_recovery_action=BLOCK`，并关闭自动离桩可用标志。
- 阻断结果：导航前置序列失败，不发送待发 Nav2 目标；此项本身不自动发送离桩或清锁命令。
- 放行/恢复逻辑：恢复安全状态发布后，新的请求/前置检查使用新鲜状态再决策；不是等待满1秒就自动放行，也不把丢消息当“锁已清”。
- 当前仓库配置状态：[`navigation_require_dock_safety_interlock_state=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L89)，状态最大年龄1.0秒，来源 `/safety/dock_interlock_state`。
- 源码函数：[DockContactInterlockModule::Impl::snapshot](../src/robot_api_server/src/features/docking/lifecycle/dock_contact_interlock_module.cpp#L596)、[PreNavigationUndockModule::run_if_needed](../src/robot_api_server/src/features/docking/lifecycle/pre_navigation_undock_module.cpp#L34)。

<a id="a10"></a>

### A10 导航前受控离桩/残留锁协调门

- 作用：普通导航不能直接拖着充电接触离开；API 先根据运行对桩状态、接触证据、接触锁与桩区位置选择受控离桩或有证据的残留锁协调。
- 触发/拒绝条件：确认在桩、充电或位置不明但仍有在桩记忆锁时，要求 `CONTROLLED_UNDOCK`；若证明已在桩区外且无实时接触，要求 `CLEAR_STALE_INTERLOCK`；对桩仍活跃且非已在桩状态时也拒绝普通导航。解析不出清锁所需 dock_id、服务失败或离桩未完成均不通过。
- 阻断结果：暂不发送导航目标，先调用所属受控流程或等待已在运行的离桩流程；失败则目标前置失败。真正的离桩运动仍由对桩管理器和 robot_safety 仲裁，不是本门直接放开普通速度。
- 放行/恢复逻辑：明确离桩成功后继续；配置需要离桩后重定位时还须通过 A11。残留锁协调必须成功返回；服务/动作超时仍拒绝目标，不把计时到期当成离桩成功。
- 当前仓库配置状态：[自动离桩期限28秒](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L127)，离桩后重定位 true；[旧BMS锁自动离桩许可=false](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L100)，但对 safety memory latch 的桩区分析另有分支，不能只凭这个开关推断总体放行。
- 源码函数：[DockContactInterlockModule::Impl::snapshot 的桩区/恢复动作归约](../src/robot_api_server/src/features/docking/lifecycle/dock_contact_interlock_module.cpp#L608)、[PreNavigationUndockModule::run_if_needed](../src/robot_api_server/src/features/docking/lifecycle/pre_navigation_undock_module.cpp#L34)、[wait_for_completion](../src/robot_api_server/src/features/docking/lifecycle/pre_navigation_undock_module.cpp#L199)。

<a id="a11"></a>

### A11 离桩后定位与 TF/costmap 稳定屏障

- 作用：把“机械上离桩成功”和“已经可以安全恢复导航”分开，等待恢复全局纠偏、显式定位和控制器所需 TF/costmap 证据就绪。
- 触发/拒绝条件：离桩后旧 `docking_fine` 暂停未能释放，重定位未成功，或 post-undock settle 屏障失败；所需稳定 TF 样本、costmap 更新与新消息过滤丢弃检查由定位屏障实现。
- 阻断结果：保持 pending Nav2 目标不发送；可同时报告机械状态 `undocked` 和 `post_undock_navigation_readiness_failed=true`，这不是状态矛盾。等待期间按配置发布零速。
- 放行/恢复逻辑：重定位及后续 settle 完整成功才释放待发目标；其中任一步失败，当前目标不会因机器人已经离桩而越过屏障，需要修复定位/暂停状态后重新走恢复和任务流程。
- 当前仓库配置状态：[`undock_relocalize_after_success=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L124)、[post-undock settle=true](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L217)：最少800ms、最多5000ms、2个稳定TF样本、2次local costmap更新，并拒绝新增message-filter丢弃。并非现场屏障成功的证明。
- 源码函数：[DockingStatusModule::Impl::complete_post_undock_relocalization](../src/robot_api_server/src/features/docking/lifecycle/docking_status_module.cpp#L473)、[PreNavigationUndockModule::wait_for_completion 的目标释放条件](../src/robot_api_server/src/features/docking/lifecycle/pre_navigation_undock_module.cpp#L211)。

<a id="a12"></a>

### A12 精对桩全局纠偏暂停握手门

- 作用：精对桩/近场对齐时先取得全局纠偏暂停，使 map 位姿不会在近距离控制中被另一个全局修正突然改变；它冻结的是纠偏，不是允许绕过安全仲裁。
- 触发/拒绝条件：进入 staging/fine 前请求暂停返回失败，或启用暂停要求但任务没有记录 `correction_pause_applied`；恢复流程若仍有活跃 fine owner，也不能擅自释放其暂停。
- 阻断结果：不进入精对桩；暂停请求失败会结束对桩任务并报告 `DOCK_FAILED_PREDOCK_SETTLE`，入口复验失败则报告 fine entry condition failed。
- 放行/恢复逻辑：暂停服务成功并通过入口复验才交接；完成/失败清理路径负责恢复纠偏，旧暂停清理还检查 owner。暂停/恢复服务超时不代表状态已经按请求改变，可能同时触发 A18 的副作用不明保护。
- 当前仓库配置状态：[`docking_pause_global_correction_during_fine=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L188)，服务为 `/robot_localization_bridge/set_correction_paused`；delegate=true 不取消这项握手。
- 源码函数：[PredockControlModule::start_fine_docking_handoff 的暂停请求](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L1158)、[evaluate_fine_entry](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L1061)、[DockingCorrectionPauseModule::set_paused / release_stale_if_needed](../src/robot_api_server/src/features/docking/lifecycle/docking_correction_pause_module.cpp#L74)。

<a id="a13"></a>

### A13 精对桩目标观测新鲜度/可用性门

- 作用：精对桩交接必须看到当前可用的桩目标观测，不能只依赖地图中记住的桩位置或旧传感器样本。
- 触发/拒绝条件：开启观测要求时，观测年龄小于0、超过0.5秒或 `usable=false`；使用 target_observation 后端和 GS2 后端时返回不同失败码。
- 阻断结果：fine entry 检查失败，报 `DOCK_TARGET_OBSERVATION_TIMEOUT` 或 `GS2_DOCK_DETECT_TIMEOUT`，不启动本次精对桩；不会“没有看见也盲靠”。
- 放行/恢复逻辑：需要传感器/目标检测重新提供新鲜可用观测，然后按对桩任务流程重新检查；本分支不是无限等待目标出现的自动重启器。
- 当前仓库配置状态：[`fine_docking_entry_require_gs2_fresh=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L177)，但实际配置的[观测后端为 `target_observation`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L84)，来源为 `orbbec_336l_depth`。参数名含 GS2 不等于当前使用 GS2。
- 源码函数：[PredockControlModule::evaluate_fine_entry](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L1069)。

<a id="a14"></a>

### A14 预对桩几何捕获范围与 Nav2→fine 交接门

- 作用：只在桩附近可处理的几何范围内，把预对桩导航交给近场对桩；Nav2结果和精对桩入口复用这类 capture 判断，作为同一门计数。
- 触发/拒绝条件：位姿不可用，前向误差不在 capture window、横向误差超可纠正上限，或基础/接触朝向超硬失败范围；非 delegate 路径还要求已完成 predock yaw 对齐、横向误差和朝向达到更严的 fine-entry 阈值。
- 阻断结果：不交接/不启动 fine，返回相应 pose-drift/entry-condition/yaw/lateral 失败；Nav2 abort 后也必须验证在 capture 内，不能仅因“离桩很近”继续。早期交接还必须先证明原 Nav2 目标终止。
- 放行/恢复逻辑：允许的任务内补偿或重新导航使位姿进入捕获范围，再重新验证；范围外不能靠等待降级放行。delegate 路径由对桩管理器承担后续物理 yaw/lateral/forward，API不再先把所有残差修到严格阈值。
- 当前仓库配置状态：[`docking_delegate_staging_motion_to_manager=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L137)；capture 前向[-0.40,0.55]m、横向最大0.25m，见[范围配置](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L157)。[Nav2早期handoff=false](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L109)。虽然[fine严格阈值和yaw要求](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L178)存在，delegate 分支会提前按manager capture结果返回，不能宣称这些API严格子门当前全部运行。
- 源码函数：[PredockAlignmentPolicy::pose_allows_staging_recovery / forward_capture_window_met](../src/robot_api_server/src/features/docking/predock_alignment/predock_alignment_policy.cpp#L115)、[DockingJobExecutor 的 Nav2 接管判断](../src/robot_api_server/src/features/docking/lifecycle/docking_job_executor.cpp#L385)、[PredockControlModule::evaluate_fine_entry 的 delegate/严格分支](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L1076)、[bridge settle 后再验capture](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L1174)。

<a id="a15"></a>

### A15 API 预对桩实际自旋反馈证明

- 作用：API亲自做predock yaw对齐时，不能把地图朝向变化误当成底盘真的完成自旋；要看到实际底盘模式反馈。
- 触发/拒绝条件：已经发送非零旋转命令且要求 actual spin，但尚未确认新鲜反馈 `actual_motion_mode_code==2`（SPINNING）；若地图角度先进入容差却仍未确认物理自旋，也拒绝宣布成功；等模式超过期限同样失败。
- 阻断结果：停止这次API yaw对齐，报 `PREDOCK_YAW_ALIGN_NO_CONFIRMED_PHYSICAL_SPIN` 或 `PREDOCK_YAW_ALIGN_MODE_SWITCH_TIMEOUT`，不凭地图角度完成来交接。
- 放行/恢复逻辑：期限内收到新鲜、实际可用的SPINNING=2反馈后才确认；初始就已对齐而没有发非零旋转命令的情况不凭空要求转一下。失败后需检查底盘模式/反馈并按任务流程重试，超时不是自动确认。
- 当前仓库配置状态：[`predock_yaw_align_require_actual_spin=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L150)，模式切换等待2秒；但当前 [delegate=true](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L137) 的主staging路径明确不由API输出物理yaw/lateral命令，因此这是“代码存在、该API分支通常不走”，不是现场开启断言，也不推断管理器没有自己的反馈门。
- 源码函数：[PredockControlModule::run_yaw_align 的物理证据分支](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L494)、[实际模式确认与超时](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L552)、[delegate 路径说明](../src/robot_api_server/src/features/docking/predock_alignment/predock_control_module.cpp#L1174)。

<a id="a16"></a>

### A16 楼层运行负向互锁

- 作用：楼层事务正在改变地图/定位，或已失败锁定时，跨入口阻止导航、对桩和不合适的地图操作，避免只在最初HTTP入口检查一次。
- 触发/拒绝条件：typed floor status 的状态/阶段处于切图流程活动集合；状态、阶段或 detail 含 `FAILED_LOCKED`；定位健康报告 `transition_active=true` 或 `runtime_context_valid=false`。当前工作区对手动切图入口 `live_floor_switch_start` 有限定例外：调用 `decision_for_map_switch()`，只在决策副本中忽略单独的 `FLOOR_RUNTIME_CONTEXT_INVALID`，因此当前源图上下文无效本身可以不阻止手动切图；活动换层和 `FAILED_LOCKED` 仍阻止。普通导航等仍用原始决策，继续被无效上下文挡住。它是负向证据归约：没有负面消息不等于已经取得所有正向就绪证明。
- 阻断结果：阻断时，HTTP响应外层 `code=FLOOR_TRANSITION_BLOCKED`，`reason_code` 为 `FLOOR_TRANSITION_ACTIVE`、`FLOOR_RUNTIME_CONTEXT_INVALID` 或 `FLOOR_TRANSITION_FAILED_LOCKED`，拒绝新请求/后台阶段继续；手动切图的上述例外仅允许继续后续预检，不代表目标图已就绪。该API归约器不直接负责全局持续发零速。
- 放行/恢复逻辑：普通活动/无效上下文分支可随后续明确无负面证据的状态自动清除；手动切图例外不清除原始健康状态，也不解锁普通导航，目标地图仍须走完自己的资产、定位、TF等证明流程。`FAILED_LOCKED` 是粘滞分支，后续普通健康/完成消息不会清除。源码注明要有显式恢复协议或完整runtime重启才能重建状态；本处未实现一个可直接调用的清锁API，不能把重启API写成已证明安全的恢复操作。
- 当前仓库配置状态：[`floor_runtime_negative_interlock_enabled=true`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L31)。上面的源图无效上下文例外是当前工作区按 operation 分流的源码逻辑，不是这个开关被关闭；禁用开关本身可跳过此消费端互锁，但不等于满足底层换层、定位或安全证明。
- 源码函数：[FloorRuntimeInterlock::observe_floor_switch_status](../src/robot_api_server/src/features/floor_switch/floor_runtime_interlock.cpp#L60)、[observe_localization_health](../src/robot_api_server/src/features/floor_switch/floor_runtime_interlock.cpp#L105)、[decision_for_map_switch](../src/robot_api_server/src/features/floor_switch/floor_runtime_interlock.cpp#L171)、[FloorSwitchModule::Impl::interlock_decision](../src/robot_api_server/src/features/floor_switch/floor_switch_module.cpp#L220)、[interlock_response 按 live_floor_switch_start 分流](../src/robot_api_server/src/features/floor_switch/floor_switch_module.cpp#L259)。

<a id="a17"></a>

### A17 地图选择提交前源身份防漂移

- 作用：防止“预检时是一份地图，提交时已被替换成另一份地图”的检查—使用竞态；它不同于A02的全局持久完整性降级标志。
- 触发/拒绝条件：离线楼层选择经floor-manager preflight后，重新查找的manifest与原请求不是同一精确资产源，或提交前资产身份快照验证失败/变化；激活步骤再次检测到 `ExactMapAssetSourceDrift` 也拒绝。
- 阻断结果：返回 HTTP409、`FLOOR_SELECTION_SOURCE_DRIFT`，不把漂移后的源当作已验证目标提交。激活前会清运行上下文，因此激活阶段失败后的上下文恢复仍须正式流程，不能假设旧上下文必然继续可用。
- 放行/恢复逻辑：从稳定的正确地图资产重新开始选择/preflight并重新证明；不能复用原预检响应绕过重验，也不通过延迟一段时间自动消除漂移。
- 当前仓库配置状态：未见本门总禁用开关；使用配置的 [`maps_root`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L8) 和资产事务。这里证据直接对应 `handle_offline_switch` 的提交路径，不扩张成“所有live换层证明都由这一条实现”。
- 源码函数：[FloorSwitchModule::Impl::handle_offline_switch 提交前重验](../src/robot_api_server/src/features/floor_switch/floor_switch_module.cpp#L1089)、[激活阶段漂移处理](../src/robot_api_server/src/features/floor_switch/floor_switch_module.cpp#L1117)。

<a id="a18"></a>

### A18 超时副作用不明：DELAYED_SIDE_EFFECT_UNKNOWN

- 作用：发送ROS服务/action等有副作用请求后，如果超时，防止迟到的旧请求与新任务重叠；“客户端没等到结果”不等于“远端没有执行”。
- 触发/拒绝条件：共享 `delayed_side_effect_unknown_count` 非零。相关操作开始时先登记 pending evidence，只有明确收到可接受结果/终态证据的路径才调用 `resolve()`；超时、异常或缺少新鲜终态证明时不自动减计数。因而计数也会短暂覆盖仍在进行中的已登记操作，并非仅在超时那一刻才出现。
- 阻断结果：经 motion-admission fence 的导航、对桩、地图等新副作用请求被拒绝，返回 `DELAYED_SIDE_EFFECT_UNKNOWN`；某些制造不明结果的路径同时锁安全停车，例如 cancel-all 超时。不能将此门仅理解为普通mutex，关键是跨请求保留的未决证据。
- 放行/恢复逻辑：被跟踪操作在仍有明确处理路径时得到完成/拒绝/空闲证明并显式 `resolve()`，计数归零后再检查其他门；若已经超时返回而未保留迟到结果解析路径，本次核查未发现通用“等一等自动清零”或用户清零接口。必须经有证据的恢复处理；单纯超时、后续HTTP重试或改显示状态不能解锁。进程重建会重建内存计数，但不证明外部副作用消失，不能作为安全恢复建议。
- 当前仓库配置状态：未见禁用此门的开关；共享计数在组合根初始化并注入各域。服务/action期限取自[服务超时配置](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L242)及各操作配置，但期限不是解锁TTL。
- 源码函数：[ElevatorModule::Impl::acquire_motion_admission](../src/robot_api_server/src/features/elevator/elevator_module.cpp#L216)、[ElevatorMotionAdmissionFence::acquire_for_submission](../src/robot_api_server/include/robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp#L230)、[PendingSideEffectEvidence::resolve](../src/robot_api_server/src/features/navigation/runtime/navigation_action_runtime.cpp#L35)、[NavigationActionRuntime::Impl::cancel_all_goals_with_evidence](../src/robot_api_server/src/features/navigation/runtime/navigation_action_runtime.cpp#L309)、[共享计数初始化](../src/robot_api_server/src/application/composition/application_composition_module.cpp#L604)。


<a id="group-n"></a>

## 5. Nav2 与控制器门控（N01–N08）

本组按“拒绝一次动作、阻止非零指令或拒绝状态交接”的独立机制计数，不按 `if`、错误码或阈值数量计数。配置状态仅描述仓库源码、基础 YAML 和 Jetson runtime overlay；没有读取现场 ROS 参数、节点生命周期或部署二进制。N06、N07、N08 是项目明确接入的 Nav2 外部组件；本轮本地未找到对应第三方实现源码，因此将项目接入证据与组件行为语义分开，不伪造外部函数行号。

<a id="n01"></a>

### N01 起步航向观测与对齐门控

- **作用**：新导航目标开始时，先由 RotationShim 完成路径起步航向对齐，再把控制交给主路径控制器；同一目标的普通重规划不反复抢占起步对齐。
- **触发/拒绝条件**：`rotate_to_heading_once` 开启且当前目标尚未消费起步对齐。航向误差缺失或非有限值时返回 `kHold`；误差大于进入阈值时返回 `kRotate`，开始旋转后使用更小的退出阈值。路径太短是明确的放行分支，不是无期限等待。
- **阻断结果**：`kHold` 发布零速度并撤销终点侧移/倒车许可；`kRotate` 只输出受控旋转，暂不调用主路径控制器。它不是最终 `robot_safety` 急停门控。
- **放行/恢复与超时**：观测恢复且误差进入退出范围，或路径被判定过短，标记本目标起步对齐已完成。该 guard 自身无独立超时；持续不能对齐的结果由上层 FollowPath/进展监控处理，不能把日志节流时间当恢复期限。
- **仓库接入状态**：overlay 开启；进入阈值 约25.8°（0.45 rad）、退出阈值约4.30°（0.075 rad）。源码：[StartupAlignmentGuard::evaluate](../src/robot_nav_config/include/robot_nav_config/startup_alignment_guard.hpp#L25)、[computeVelocityCommands 的起步分支](../src/robot_nav_config/src/goal_scoped_rotation_shim_controller.cpp#L554)。配置：[起步对齐开关与阈值](../scripts/jetson/runtime_overlay/config/nav2.yaml#L379)。

<a id="n02"></a>

### N02 终点姿态接管与阶段停车交接门控

- **作用**：在普通 Ackermann 跟踪无法合适收敛的近终点残差范围内，交给有限范围的终点控制器；各航向、侧移、纵向阶段之间必须先稳定停下，不能同时执行互相冲突的校正。
- **触发/拒绝条件**：启用接管、目标处于距离/纵向残差边界内，并满足目标在后方或侧向残差/路径几何不适合普通跟踪等条件才接管。接管后输入非有限、时钟回退或总时限超出均进入失败；`Settling` 阶段实际线/角速度未低于阈值，或稳定时间不足时不进入下一运动阶段。
- **阻断结果**：停车阶段输出零指令；失败时撤销临时许可并抛出控制器失败，不把“已进入终点区域”当成功。完整 XY/yaw 仍须被接受；`Complete` 后残差再次出界会重新进入稳定停车。
- **放行/恢复与超时**：实际速度持续满足停车阈值后选择下一阶段；航向、侧向、纵向误差分别进入退出范围后再停车交接。overlay 稳定阈值为 0.01 m/s、约1.15°/s（0.02 rad/s），持续 `0.30 s`，总时限 `20 s`；重置/新一次接管建立新状态。
- **仓库接入状态**：overlay `terminal_handoff_enabled: true`，最大距离 `0.40 m`、纵向边界 `0.15 m`。源码：[should_start_terminal_handoff](../src/robot_nav_config/include/robot_nav_config/terminal_pose_handoff.hpp#L71)、[TerminalPoseHandoffController::update](../src/robot_nav_config/include/robot_nav_config/terminal_pose_handoff.hpp#L211)、[控制器失败出口](../src/robot_nav_config/src/goal_scoped_rotation_shim_controller.cpp#L867)。配置：[终点接管参数](../scripts/jetson/runtime_overlay/config/nav2.yaml#L401)。终点预测碰撞是 N03，不在此重复计数；最终侧移/倒车许可仲裁复用 safety。

<a id="n03"></a>

### N03 预测足迹碰撞门控

- **作用**：实际发送终点校正或电梯小范围运动命令前，检查命令方向上的预测车体足迹是否安全，防止只因目标误差可收敛就执行近障运动。
- **触发/拒绝条件**：普通终点分支中，非零命令遇到 costmap 不存在、frame/TF 不可用、姿态非法，或任一预测足迹代价非法/负数/达到致命障碍，拒绝该命令。电梯分支在 `command_clearance_check_enabled` 开启时，对非零命令执行其范围路径净空检查。
- **阻断结果**：替换为零速度，并撤销本轮侧移/倒车等临时许可。电梯控制器可转为 `WAIT_CLEAR`/`REPLANNING` 并尝试有边界的路线修订；并非直接放宽足迹或授权倒车。
- **放行/恢复与超时**：后续周期重新检查，净空恢复才允许非零命令。普通终点采样当前足迹及前视路径，配置前视 `0.15 m`；本检查没有独立“等够时间就通行”。电梯 `blocked_timeout_sec` 用于持续阻塞提示，不是绕过碰撞的通行计时器；其他阶段时限仍由调用控制器承担。
- **仓库接入状态**：普通终点接管启用时此检查在调用链内。overlay 中三处电梯净空开关明确为 `false`，所以不能声称这些电梯分支当前按该参数执行此预测检查；其他 collision monitor / safety 检查不因此消失。源码：[terminal_command_is_clear](../src/robot_nav_config/src/goal_scoped_rotation_shim_controller.cpp#L900)、[拒绝后零速出口](../src/robot_nav_config/src/goal_scoped_rotation_shim_controller.cpp#L879)、[电梯检查与阻塞处理](../src/robot_nav_config/src/elevator_scoped_controller.cpp#L711)。配置：[普通终点前视](../scripts/jetson/runtime_overlay/config/nav2.yaml#L427)、[电梯开关 1](../scripts/jetson/runtime_overlay/config/nav2.yaml#L204)、[开关 2](../scripts/jetson/runtime_overlay/config/nav2.yaml#L290)、[开关 3](../scripts/jetson/runtime_overlay/config/nav2.yaml#L352)。按共同机制合计一项。

<a id="n04"></a>

### N04 普通导航失败恢复资格门控

- **作用**：只为已确认的、当前目标/本次尝试的进展失败开放重新规划恢复，拒绝拿旧失败、其他目标或失败前路径重试。
- **触发/拒绝条件**：FollowPath 必须明确 aborted；监控必须确认对应目标和本次尝试后的进展失败。`RecoveryState::prepare` 还要求新路径仍为相同目标、至少有两个 pose、路径时间戳晚于失败时间；不匹配则拒绝准备恢复。
- **阻断结果**：无法证明可恢复时 BT 返回失败，不启动恢复子树；等待失败证据及退避期间保持恢复控制流程等待，而不是凭时间判断“障碍已清除”。
- **放行/恢复与超时**：失败后先失效旧监控样本，最多等待 `2 s` 取得检查结果；第一轮立即重试，重复失败等待 `5 s`，之后上限 `10 s`。新的有效进展 epoch 重置重试计数；只有失败后的匹配路径获得恢复准备。该机制不是有限次数后强行通过。
- **仓库接入状态**：代码及专用 BT 已存在；[项目恢复说明](../src/robot_nav_config/docs/ordinary_navigation_recovery.md#L3) 明确标为 **Phase 2 offline candidate**，仍待硬件验收和单独部署/重启授权。本段描述当前源码的保留任务＋连续失败 5/10 秒等待策略，不沿用旧 Phase 1 固定一次重试概述。脚本默认 `smac2d` 选择普通 `navigate_to_pose.xml`，`ranger_lattice` profile 才默认选择带本恢复节点的 BT，也可被环境指定 BT 覆盖。源码：[OrdinaryRecoveryLoop::tick](../src/robot_nav_config/include/robot_nav_config/navigation_recovery/recovery_loop.hpp#L34)、[RecoveryState::inspect](../src/robot_nav_config/src/navigation_recovery/recovery_state.cpp#L134)、[RecoveryState::prepare](../src/robot_nav_config/src/navigation_recovery/recovery_state.cpp#L167)。接入：[恢复 BT](../src/robot_nav_config/behavior_trees/navigate_to_pose_ranger_lattice_recovery.xml#L6)、[Jetson profile/BT 选择](../scripts/jetson/runtime_overlay/scripts/run_nav2_navigation.sh#L17)。

<a id="n05"></a>

### N05 Nav2 进展检查门控

- **作用**：机器人长期没有足够位置或航向进展时，拒绝继续把 FollowPath 判为正常推进；同时区分电梯慢速运动和明确的有意等待。
- **触发/拒绝条件**：有效基线建立后，在时间窗口内平移和航向变化均未达到各自最小值，返回 `false`。无效 pose/时间也拒绝。只有新鲜电梯状态才能选择电梯 profile；状态超过 `0.50 s` 后退回普通 profile，不保留旧电梯豁免。明确的暂停状态重置基线并暂时通过进展检查，不能将此通过误读为已完成目标。
- **阻断结果**：ProgressChecker 返回失败，交由 Nav2 controller server/BT 处理；项目恢复状态也记录该次普通进展失败，供 N04 精确匹配。它本身不是直接发布最终零速度的节点。
- **放行/恢复与超时**：平移或航向任一达到阈值即刷新基线。普通为 3 cm／约2.86°／12秒；电梯为1.5 cm／约0.86°／20秒。重置、profile 变化或策略中的时间回退分支建立新基线；暂停结束后的首个跟踪样本重新计时。
- **仓库接入状态**：overlay controller server 的 progress checker 明确选用自研 `ElevatorAwareProgressChecker`。源码：[ElevatorAwareProgressChecker::check](../src/robot_nav_config/src/elevator_aware_progress_checker.cpp#L99)、[ElevatorAwareProgressPolicy::check](../src/robot_nav_config/include/robot_nav_config/elevator_aware_progress_policy.hpp#L29)。配置：[进展阈值/状态新鲜度](../scripts/jetson/runtime_overlay/config/nav2.yaml#L138)。

<a id="n06"></a>

### N06 Collision monitor 障碍与输入失效门控

- **作用**：在 `/cmd_vel_nav` 到 `/cmd_vel_collision_checked` 之间执行独立的近障停止、减速、碰撞时间约束及观测失效处理。三个 polygon 与输入 stale 属同一组件的同一命令安全边界，合计一项。
- **触发/拒绝条件**：配置的 StopZone 障碍计数超过 `max_points: 1`（计数门槛为超过一个返回；配置本身没有单列“必须相邻”的条件）触发停止；SlowZone 超过配置计数触发比例减速；FootprintApproach 根据已发布足迹和碰撞预测时间约束速度。配置启用 `/scan`，`source_timeout: 1.5 s` 用于观测新鲜度；输入过期不是“没有障碍”。外部 Nav2 实现未在本轮本地读取，具体版本的内部计数/TF失败边界仍须以部署版本验证。
- **阻断结果**：按最严格适用动作抑制或缩减输出；StopZone 输出停止，SlowZone 配置比例 `0.65`，Approach 配置碰撞前时间 `2.0 s`。后面仍经过 `robot_safety` 最终仲裁，本组件不是旁路终端 `/cmd_vel`。
- **放行/恢复与超时**：随新观测和新指令重新评估，障碍条件解除且输入恢复有效后才能恢复相应命令。`stop_pub_timeout: 0.3 s` 是停止命令发布相关参数，不是超时后允许穿过障碍；`simulation_time_step: 0.1 s` 是预测步长，不是健康超时。
- **仓库接入状态**：三个区域和 scan source 均配置 `enabled: true`；StopZone 半长/半宽 `0.47/0.36 m`。项目 launch 明确创建外部 `nav2_collision_monitor`，但配置存在不证明现场已激活，也不证明当前巡航速度下的制动距离通过硬件验收。源码接入：[standard_navigation.launch.py 节点](../src/robot_bringup/launch/standard_navigation.launch.py#L357)。配置：[完整 collision monitor 边界](../scripts/jetson/runtime_overlay/config/nav2.yaml#L764)。

<a id="n07"></a>

### N07 Velocity smoother 指令超时门控

- **作用**：平滑 Nav2 速度并在上游停止更新时使目标速度回到零，避免旧命令无限延续；它与最终 safety 的独立命令 watchdog 是不同处理边界。
- **触发/拒绝条件**：上游命令超过配置的 `velocity_timeout: 1.0 s` 没有更新。加减速度上限和死区属于同一 smoother 的正常整形，不拆成额外门控。
- **阻断结果**：外部 Nav2 velocity smoother 的超时目标为停车，按配置减速度平滑收敛；不应把它表述为与硬件急停等价的瞬时断电制动。
- **放行/恢复与超时**：新有效速度命令到来后重新平滑输出；当前为 `30 Hz`、`OPEN_LOOP`，线/角最大减速度配置为 `[-0.95, -0.30, -1.10]`。虽然配置了 odom topic，`OPEN_LOOP` 不能被写成“持续依赖 odom 闭环反馈”。
- **仓库接入状态**：launch 创建外部 `nav2_velocity_smoother`，overlay 配置明确有 1 秒超时。未发现本地外部组件实现，不能据此断言部署二进制的精确发布节奏。源码接入：[velocity_smoother 节点与 remap](../src/robot_bringup/launch/standard_navigation.launch.py#L350)。配置：[smoother 参数](../scripts/jetson/runtime_overlay/config/nav2.yaml#L651)。

<a id="n08"></a>

### N08 普通导航 GoalChecker 成功判定门控

- **作用**：只在机器人同时满足普通目标的位置和航向误差时承认 FollowPath 到达，而非进入近目标区域就提前成功。
- **触发/拒绝条件**：XY 或 yaw 误差不满足配置容差时不判成功；`stateful: false` 要求位置与航向持续重新检查，避免曾短暂满足 XY 后被推离仍保留旧成功条件。
- **阻断结果**：拒绝“目标完成”的状态交接，FollowPath 仍需继续控制或由其他故障/取消路径结束。GoalChecker 本身不负责选择恢复动作，也不是实际停车证据检查。
- **放行/恢复与超时**：配置 XY 6 cm、yaw 约2.86°（0.05 rad） 同时满足时可返回到达；本检查无独立总计时器，持续失败由 N05 或上层动作超时处理。该 `SimpleGoalChecker` 不应被描述成同时要求实际速度为零。
- **仓库接入状态**：overlay 选择外部 `nav2_controller::SimpleGoalChecker`，普通 `navigate_to_pose.xml` 的 FollowPath 显式指定 `goal_checker`。外部类实现未在本地找到，因此这里是配置/调用链证据。源码接入：[普通 FollowPath 的 goal_checker_id](../src/robot_nav_config/behavior_trees/navigate_to_pose.xml#L7)。配置：[容差与无状态检查](../scripts/jetson/runtime_overlay/config/nav2.yaml#L155)。Predock capture envelope 不在本表另计：与 A14 对接交接条目共用；额外注意当前 [navigate_to_predock.xml](../src/robot_nav_config/behavior_trees/navigate_to_predock.xml#L8) 写的是 `goal_checker`，不能仅因 YAML 声明 `DockStagingGoalChecker` 就声称该 BT 正在使用它。

<a id="group-d"></a>

## 6. 对接控制门控（D01–D05）

<a id="d01"></a>

### D01 对接观测新鲜度与目标捕获门控

- **作用**：细对接捕获/对齐阶段必须依赖近期目标观测；没有有效目标时不允许默认盲进。
- **触发/拒绝条件**：`control_step` 的捕获/对齐路径中，观测缺失或接收年龄大于 `command_timeout_ms` 则零速等待；`handle_acquire` 需要 detection 有效且有效检测计数达到阈值才进入 Align。检测有效性包含来源/标志及点数、跨度、置信度等适用条件。
- **阻断结果**：观测不新鲜时发布零速度与等待状态；未捕获时留在 Acquire，超过有限重试后 `dock_feature_not_found` 失败。当前 `allow_blind_approach: false`，不能借重试进入 BlindApproach。
- **放行/恢复与超时**：新鲜且有效目标累计达到 `stable_frames_required: 3` 后进入 Align；观测新鲜度阈值 `300 ms`。Acquire 每超过 `2 s` 检查重试预算（配置 `max_retries: 3`）；观测 stale 的提前返回本身没有强制退出总期限。此实现的 Acquire 计数按控制周期累加，不应宣称严格证明三个互不相同的 sensor sequence。进入 ContactVerify 后视觉在近盲区不再具有运动许可权，改由 D04 的 BMS/odom/距离时间边界接管。
- **仓库接入状态**：overlay 使用 `target_observation`，source 为 `orbbec_336l_depth`，关闭盲进。源码：[control_step](../src/robot_docking_manager/src/docking_manager_node.cpp#L601)、[observation_fresh](../src/robot_docking_manager/src/docking_manager_node.cpp#L682)、[handle_acquire](../src/robot_docking_manager/src/docking_manager_node.cpp#L868)。配置：[观测后端](../scripts/jetson/runtime_overlay/config/docking.yaml#L4)、[禁止盲进](../scripts/jetson/runtime_overlay/config/docking.yaml#L63)、[超时/检测计数](../scripts/jetson/runtime_overlay/config/docking.yaml#L101)。

<a id="d02"></a>

### D02 近场对接对齐可执行性与重对齐预算门控

- **作用**：避免近场控制器持续反复切换航向调整，或在无法生成有效动作时仍声称对齐正在进行。
- **触发/拒绝条件**：需要再次航向捕获但 `yaw_realignments` 已达到预算，或者平移阶段扣除死区/限制后纵向与横向指令均为零且仍未完成，会进入 `AlignmentBlocked`。节点检查该输出并作为失败处理。
- **阻断结果**：不发新的非零对齐指令；报告 `yaw_realign_budget_exhausted` 或 `no_action_available` 等原因，节点进入 Failed、零速、撤销倒车许可并释放对接强制模式。
- **放行/恢复与超时**：正常航向捕获以新的 observation sequence 累计稳定样本，进入退出容差后回到原平移阶段；预算用完不是等待若干秒自动增加预算，重新开始任务会 reset 控制器。此机制本身没有独立总时限。
- **仓库接入状态**：overlay 重对齐进入阈值 `3°`、稳定帧数 `3`、最多额外重对齐 `1` 次。源码：[NearFieldDockingController::update 的阻断路径](../src/robot_docking_manager/src/near_field_docking_controller.cpp#L109)、[无可用平移动作](../src/robot_docking_manager/src/near_field_docking_controller.cpp#L216)、[DockingManagerNode::handle_align](../src/robot_docking_manager/src/docking_manager_node.cpp#L904)。配置：[对齐预算](../scripts/jetson/runtime_overlay/config/docking.yaml#L132)。

<a id="d03"></a>

### D03 接触后真实停止与 Docked 交接门控

- **作用**：BMS 发现充电接触后，先持续制动并证明车体已稳定停下，之后才完成 Docked 状态交接，避免把电接触瞬间直接当机械停车完成。
- **触发/拒绝条件**：进入 `ContactStopping` 后，要求 `/motion_state` 与 `/wheel/odom` 都有接触后的新 sequence、两路接收年龄有效，且轮速线/角速度有限并低于阈值。任何条件不足都重置稳定样本/时长积累。
- **阻断结果**：持续发布接触制动零速度，不进入最终 Docked。此阶段收到 stop/fail 请求也只记录延后处理并继续零速，不用请求取消来绕过真实停车证明；此时退桩启动也会被拒绝。
- **放行/恢复与超时**：以不同 wheel sequence 累计至少 `5` 个稳定样本，并连续稳定 `0.50 s` 才调用 `finalize_docked_stop`。两路反馈最大年龄 `0.50 s`，速度阈值 0.01 m/s、约1.15°/s（0.02 rad/s）。`feedback_timeout_s: 3.0` 只产生超时告警并继续零速，不会超时放行。
- **仓库接入状态**：节点内固定的接触后状态屏障，overlay 提供阈值，没有独立总开关。源码：[handle_contact_stopping](../src/robot_docking_manager/src/docking_manager_node.cpp#L1126)、[stop_docking 的延后分支](../src/robot_docking_manager/src/docking_manager_node.cpp#L517)、[start_undocking 拒绝未确认制动](../src/robot_docking_manager/src/docking_manager_node.cpp#L555)。配置：[contact_stop](../scripts/jetson/runtime_overlay/config/docking.yaml#L84)。最终 safety 的 dock latch 是被复用/下游机制，不在 D03 再拆项。

<a id="d04"></a>

### D04 接触确认爬行距离、时间与有限重试门控

- **作用**：进入相机近盲区后，只允许有界直行接触尝试，不能一直靠视觉缺失继续推进。
- **触发/拒绝条件**：缺少接触起点 odom 或 odom stale 立即失败；未检测到充电且位移达到 `contact_verify_max_distance_m`，或超过当前动态接触时限，结束本次接触尝试。
- **阻断结果**：失败时零速并撤销许可；若允许重试且预算未用尽，先零速，再进入受 D05 保护的退靠，重新捕获目标。不是将原有接触距离计数清零后原地无限继续。
- **放行/恢复与超时**：充电接触转入 D03；未接触而额度尚有余量才继续有限爬行。overlay 最大接触位移 `0.31 m`、配置接触确认上限 `16 s`；实际时限取该上限与控制器按行程估算的时限的较小值。重试启用且最多 `2` 次；最后 `0.06 m` 区间减为 `0.02 m/s`，此减速不另计门控。
- **仓库接入状态**：overlay `contact_verify_retry_enabled: true`。源码：[handle_contact_verify](../src/robot_docking_manager/src/docking_manager_node.cpp#L1193)、[begin_contact_retry](../src/robot_docking_manager/src/docking_manager_node.cpp#L1245)、[动态时限设置](../src/robot_docking_manager/src/docking_manager_node.cpp#L945)、[contact_timeout_sec](../src/robot_docking_manager/src/near_field_docking_controller.cpp#L242)。配置：[接触时限](../scripts/jetson/runtime_overlay/config/docking.yaml#L99)、[距离/速度/重试参数](../scripts/jetson/runtime_overlay/config/docking.yaml#L140)。

<a id="d05"></a>

### D05 退靠/退桩运行证据与进展边界门控

- **作用**：退靠及退桩都必须由有效 odom 证明实际起动、持续进展和完成距离，不以“发过速度”或“等过时间”代替退出接触区域的证据。
- **触发/拒绝条件**：起点 odom 在期限内不可得、运行中 odom 过期、没有真实起动、持续无进展、总时限超出均失败；退靠另外检查横向漂移上限。相关底盘反向/模式许可复用 safety，不在此重复计数。
- **阻断结果**：起点证据未就绪先零速；违规后停止并发布细分失败状态，不宣称退桩成功。退靠失败由 `fail` 清理，退桩失败由 `fail_undock` 清理并零速。
- **放行/恢复与超时**：退桩位移达到配置目标 `0.60 m` 才成功；退靠达到动态目标后回 Acquire。overlay odom 最大年龄 `0.50 s`、等待起点 `2 s`、起动 `6 s`、无进展 `2 s`、总时限 `20 s`，进展增量 `0.005 m`；退靠侧漂上限 `0.05 m`。命令准备阶段 `0.5 s` 零速等待只完成交接准备，不能代替后续运动证明。
- **仓库接入状态**：节点动作内固定机制；overlay 配置两条动作参数。源码：[handle_contact_backoff](../src/robot_docking_manager/src/docking_manager_node.cpp#L1275)、[handle_undocking](../src/robot_docking_manager/src/docking_manager_node.cpp#L1414)、[fail_undock](../src/robot_docking_manager/src/docking_manager_node.cpp#L1678)。配置：[退桩](../scripts/jetson/runtime_overlay/config/docking.yaml#L70)、[接触重试退靠](../scripts/jetson/runtime_overlay/config/docking.yaml#L149)。

<a id="group-f"></a>

## 7. 楼层切换门控（F01–F03）

<a id="f01"></a>

### F01 真实 FloorSwitch 总许可

- **作用**：区分仅预检能力与会执行真实地图/定位资产切换的 Action 路径，不以一个合法请求自动授权在线变更。
- **触发/拒绝条件**：`live_floor_switch_enabled` 为 false 时，Action 不进入真实事务执行；预检结果明确返回 `LIVE_FLOOR_SWITCH_DISABLED`。普通请求字段是否合法不是本项另一个门控。
- **阻断结果**：拒绝真实切换，不启动受 F02 管理的地图/localizer 变更事务。true 只开放执行入口，不跳过资产、停车、定位、costmap 等后续屏障。
- **放行/恢复与超时**：该参数须由实际启动配置设定；没有“等待一段时间自动置 true”的逻辑。重试相同请求不能绕过关闭值；也不能把参数设 true 当所有硬件验收已完成。
- **仓库接入状态**：基础 YAML 与节点声明默认 **false**，但 **Jetson overlay 的 `run_floor_manager.sh` 默认通过命令行覆盖为 true**，环境变量 `NJRH_LIVE_FLOOR_SWITCH_ENABLED` 可再覆盖。必须同时记录这两层，不能仅凭基础 YAML 写“现场真实切换禁用”。源码：[Action 路径选择](../src/robot_floor_manager/src/floor_manager_node.cpp#L627)、[evaluate_floor_switch_preflight](../src/robot_floor_manager/src/floor_switch_preflight.cpp#L83)。配置：[基础默认](../src/robot_floor_manager/config/floor_manager.yaml#L8)、[overlay 启动覆盖](../scripts/jetson/runtime_overlay/scripts/run_floor_manager.sh#L17)。

<a id="f02"></a>

### F02 精确目标资产与原子提交证据屏障

- **作用**：确保一次 live FloorSwitch 的目标始终是同一 building/floor/map/epoch/digest，并且仅在新目标地图、定位、桥接及 costmap 都被证明可用后提交可导航上下文。
- **触发/拒绝条件**：执行前加载精确不可变快照失败则在变更前退出。事务中要求：先持有运动 hold、Nav2 idle 与新鲜停车证据；修正暂停所有权交接和上下文失效成立；各次 map/filter/localizer 操作仍匹配目标资产；新显式重定位 sequence 非零；bridge/定位准备就绪且对应目标上下文；目标 global/local costmap 新鲜；最后提交返回 `runtime_context_valid` 与 `safe_for_goal_start`。缺任何所需证据或发生身份漂移都不能提交。
- **阻断结果**：保持目标运行上下文未提交，失败进入 F03 清理而不是恢复 mission。底层 hold、暂停租约、TF/定位健康规则为各自模块的既有门控，本项只计跨模块有序原子提交这一屏障，不把每个布尔量另计。
- **放行/恢复与超时**：所有同一事务、同一目标证据齐全后才提交；传感器/状态消息旧值不能冒充新世代证明。基础配置普通服务 `10 s`、localizer apply `30 s`、显式定位 `75 s`、证据等待 `10 s`、证据最大年龄 `0.75 s`。apply/显式定位存在专门的类型化状态协调/重试流程，不能把任一 RPC 成功响应独立当整个事务成功。
- **仓库接入状态**：live Action 已接 `FloorAssetSnapshotLoader` 和 `FloorTransitionExecutor`；是否进入由 F01 决定。源码：[execute_live_floor_switch 的快照预检](../src/robot_floor_manager/src/floor_manager_node.cpp#L775)、[FloorTransitionCore::advance](../src/robot_floor_manager/src/floor_transition_core.cpp#L168)、[最终准备/提交条件](../src/robot_floor_manager/src/floor_transition_core.cpp#L273)、[运行期 perform 适配](../src/robot_floor_manager/src/floor_manager_node.cpp#L2398)。配置：[证据与超时](../src/robot_floor_manager/config/floor_manager.yaml#L35)、[Jetson apply/trigger 超时覆盖](../scripts/jetson/runtime_overlay/scripts/run_floor_manager.sh#L19)。A17 是 API 离线切图源资产防漂移边界；它与本项共享精确资产身份思想，但不能声称 live 证明由 `handle_offline_switch` 实现。此处按“一次 live 精确身份＋原子提交屏障”计一项。

<a id="f03"></a>

### F03 FloorSwitch 失败恢复与不可证明状态锁定

- **作用**：切换失败或取消后，只有证明旧源运行上下文真正恢复，才允许把系统留在可导航的失败终态；否则保持恢复锁，避免半切图继续运行。
- **触发/拒绝条件**：事务 effect 失败、取消、sequence 不匹配等进入清理。清理须证明精确 bridge ABORT、源身份新鲜健康、源上下文持久化恢复及相关所有权释放；证据不全仍保留无效运行上下文/恢复需求。
- **阻断结果**：`FloorTransitionCore` 根据清理证据进入 `Failed` 或 `FailedLocked`；后者保持 `runtime_context_valid=false`、`recovery_required=true`，停车/暂停继续保留。清理响应“成功”但仍带 `runtime_context_invalid=true` 也不会解锁。
- **放行/恢复与超时**：只有精确源状态恢复证明齐全，才可结束为未锁的失败；清理失败可继续走失败清理，不因超时自动放行。运行适配复用 F02 的服务与证据时限。此 core 保留事务身份，终态下同一事务回放是幂等观察，不是自动重跑/解锁按钮。
- **仓库接入状态**：live 事务内固定恢复路径，F01 开启后才有相应运行机会；无单独 YAML 解锁开关。源码：[FloorTransitionCore::dispatch 的失败清理](../src/robot_floor_manager/src/floor_transition_core.cpp#L121)、[perform 的 bridge ABORT/源恢复路径](../src/robot_floor_manager/src/floor_manager_node.cpp#L3003)、[恢复不完整时保留锁](../src/robot_floor_manager/src/floor_manager_node.cpp#L3045)。配置复用：[floor manager 证据/超时](../src/robot_floor_manager/config/floor_manager.yaml#L35)。

<a id="group-e"></a>

## 8. 电梯执行与任务状态门控（E01–E04）

<a id="e01"></a>

### E01 电梯阶段人工确认门控

- **作用**：在没有相应自动物理证据的关键电梯阶段，必须收到当前执行上下文的现场确认才推进，不能把上一阶段、另一楼层或另一操作者的确认复用。
- **触发/拒绝条件**：仅 `awaiting_confirmation=true` 时接收；transaction、state、effect_sequence、event、observed_floor、operator 必须匹配当前期待值。确认事件包括来源门开、目标楼层到达、目标门开；未启用自动按钮时还包括呼梯和目标层按钮已按。
- **阻断结果**：字段不匹配返回冲突，不推进 effect；当前根本没有人工确认门时也拒绝提交。缺少确认会停留在等待阶段，而不是因 HTTP 重试推进两次。
- **放行/恢复与超时**：完全匹配后写入接受事件，清除等待标记并继续本次 FSM。该确认机制未发现独立“人工等待到点就自动确认”的计时器；运行效果的 RPC/导航时限是另一层。取消/失败进入相应清理，不等价于确认成功。
- **仓库接入状态**：overlay `elevator_runtime_adapter_enabled: true` 且自动按钮控制 true，因此两个按钮确认由自动控制路径替代，门开/楼层到达确认仍由 `confirmation_for` 定义。源码：[confirmation_for](../src/robot_elevator_manager/src/elevator_execution_module.cpp#L217)、[Implementation::confirm](../src/robot_elevator_manager/src/elevator_execution_module.cpp#L2957)、[设置等待确认](../src/robot_elevator_manager/src/elevator_execution_module.cpp#L3403)。配置：[电梯适配/按钮开关](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L10)。

<a id="e02"></a>

### E02 电梯恢复 cancel-all 完成屏障

- **作用**：恢复时不能把“取消请求已发送”或“cancel-all 已返回”直接当所有动作已结束；必须证明取消后的目标终态和稳定 idle。
- **触发/拒绝条件**：每轮先清空旧证明。cancel-all 响应列出的每个 goal 都要有响应后的终态；响应后出现未知 goal 状态、未在取消名单中的 active goal，或已终态目标重新 active，该轮标记 unsafe。需要核对哪些 action server，由本次事务是否记录 FloorSwitch 意图决定，不盲目要求不存在的 FloorSwitch 动作。
- **阻断结果**：不能进入恢复释放阶段；unsafe 直接返回 `ELEVATOR_RESTART_ACTION_STATUS_UNSAFE`，证据不足到期返回 `ELEVATOR_RESTART_ACTION_TERMINAL_UNPROVEN`。safety hold 的实际保留/释放属于复用动作，本项计“取消终态证明”这一独立交接。
- **放行/恢复与超时**：cancel-all 明确无匹配目标，或所有列出目标均获得新的终态，才有 `IdleProven`；需要的各 action server 都如此且连续满足 `navigation_idle_stable_sec` 后通过。等待期限取 `max(service_timeout_sec, endpoint_timeout_sec)`；本屏障对象自身没有定时器，期限由 `wait_for_restart_action_idle` 承担。
- **仓库接入状态**：ROS runtime port 订阅 action status 并使用本屏障，overlay 电梯适配开启；不能因此证明现场 server 已响应。源码：[ElevatorRecoveryActionBarrier::observe_post_response_status](../src/robot_api_server/src/features/elevator/execution/elevator_recovery_action_barrier.cpp#L47)、[state](../src/robot_api_server/src/features/elevator/execution/elevator_recovery_action_barrier.cpp#L102)、[wait_for_restart_action_idle](../src/robot_api_server/src/features/elevator/execution/elevator_ros_runtime_port.cpp#L2098)。配置接入：[电梯适配开启](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L10)。

<a id="e03"></a>

### E03 电梯持久恢复锁与日志恢复边界

- **作用**：对中断事务、残留资源或不可信日志保留恢复状态，避免进程重启后隐式恢复旧电梯动作，或在资源归属未知时接受新执行。
- **触发/拒绝条件**：日志记录非终态、LOCKED、hold 状态未知/仍有效，或历史 runtime 已产生效果但资源未协调，要求恢复。存储/审计失败及身份不明确也走失败关闭路径。启用持久锁策略时恢复状态为 LOCKED；不能把“非持久策略”解读为不校验日志或立即允许运动。
- **阻断结果**：撤销 `motion_authorized`，建立恢复/清理流程；持久策略阻止隐式续跑。未知副作用继续按保守路径处理，底层动作取消、租约和停车证明复用既有机制。
- **放行/恢复与超时**：持久策略需显式恢复流程重新绑定冻结配置、协调动作/资源、证明停车并按序写入恢复意图后才可能释放。非持久策略改为自动失败清理/重启协调；代码对严格限定的“普通阶段清理预算耗尽＋精确身份＋完整运行链冷启动”有终结为 FAILED 的分支，并明确不声称获得 odom 停车证明。不能把该窄分支推广为所有 LOCKED 均可重启清除，也不存在一般性到期自动通行。
- **仓库接入状态**：类型默认 `persistent_recovery_lock_enabled=true`，但生产 API 组装 **显式传 false** 给 runtime port 和 execution module；故本清单计的是仓库中存在的可选持久机制，不是当前 overlay 已开启的长期锁。非持久路径及存储失败关闭仍存在。源码：[策略默认](../src/robot_elevator_manager/include/robot_elevator_manager/elevator_execution_module.hpp#L471)、[生产显式关闭](../src/robot_api_server/src/features/elevator/elevator_module.cpp#L349)、[重启恢复分支](../src/robot_elevator_manager/src/elevator_execution_module.cpp#L4897)、[限定冷启动终结分支](../src/robot_elevator_manager/src/elevator_execution_module.cpp#L4847)、[lock_for_storage_failure_locked](../src/robot_elevator_manager/src/elevator_execution_module.cpp#L4683)。配置接入：[电梯 production adapter](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L10)。

<a id="e04"></a>

### E04 Mission FSM 失败锁与 HoldAndCancel 交接

- **作用**：高层任务 effect 失败或取消后，先进入失败锁并发出 hold-and-cancel，禁止把失败的电梯/导航效果当普通任务完成后继续发下一目标。
- **触发/拒绝条件**：任意 active effect 失败、abort/cancel，或电梯返回成功但无法证明冻结目标资产/目标厅位置/新定位及没有残留 goal、hold、lease、pause，进入 `kFailureLocked`。已经失败锁定时 `start` 拒绝新任务；已有任务/effect 活跃时也不接并发任务。
- **阻断结果**：状态锁定、取消原 effect 身份，发出带独立 transaction/cleanup_id 的 `kHoldAndCancel`。本包是纯 FSM，发出 effect 不代表它自身已经向底盘发布零速；实际执行端必须完成效果。
- **放行/恢复与超时**：hold-and-cancel 失败会以新 transaction_id 重试；成功只清除 active effect，**仍保留 failure lock**。此类未提供通用 reset/unlock 方法，不能声称一次停车确认自动恢复 Idle；其时间控制由外部执行器承担。
- **仓库接入状态**：`robot_mission_manager` 当前 CMake 构建/导出 FSM 库，并无该包独立运行节点的证明；本项属于已实现库级门控，不列为现场必然生效。源码：[MissionFsm::start](../src/robot_mission_manager/src/mission_fsm.cpp#L95)、[complete_effect](../src/robot_mission_manager/src/mission_fsm.cpp#L148)、[lock_failure](../src/robot_mission_manager/src/mission_fsm.cpp#L298)。构建接入：[add_library(mission_fsm)](../src/robot_mission_manager/CMakeLists.txt#L6)。目标资产/残余所有权条件属于依赖结果复核，不在这里逐条件新增数量。


<a id="group-l"></a>

## 9. 定位、局部状态与启动链路（L01—L23）

统计口径：同一控制目的下的时间、TF、身份等子条件合为一项，不按 `if` 数量计数。这里的“启用”指仓库默认启动脚本与配置组合，不是 Jetson 现场参数快照；环境变量、安装目录旧配置和实际启动入口都可能改变结果。L01—L23 有本地实现证据。附录 Q01 只有第三方质量配置契约证据，不确认独立硬门控、不计数；退休实现 R01 也不计入正式条目。

定位门控通常阻止“接纳某个位姿、更新 map→odom 或声明 ready”，不直接发布停车命令；车辆最终能否移动还要由 API、任务和 `robot_safety` 的相应门控决定。

<a id="l01"></a>

### L01 显式定位触发前准入

- 作用：在派发 Isaac grid-search 请求前，确认触发服务、可用输入和 bridge 的显式接纳许可已经具备。
- 触发拒绝条件：Isaac 服务在等待期内不可用；`/flatscan` 未收到、接收年龄不在 0—0.5 秒内或有效视场小于 115°；bridge force-accept 服务不可用、超时或返回失败。输入的这项年龄按接收时刻计算，不等同于原始传感器 stamp 年龄。
- 阻断结果：返回 `accepted=false` 和 `dispatch_state=not_dispatched`，不会发送本次 grid-search。注意 **arm 后新输入等待是软等待**：未等到连续合格新输入时，只记录警告并继续使用最近的新鲜输入，不会单独否决触发。
- 放行/恢复：等待期内条件满足即自动继续；超过时限则本次请求失败，恢复数据/服务后需上层重新触发。arm 后软等待超时自动继续，不应误写成“超时必停”。
- 仓库状态：默认 `require_grid_search_trigger=true`、`localizer_input_freshness_enabled=true`，服务调用等待 10 秒、输入等待 1 秒、arm 后期望连续 2 个合格样本。`run_global_localization.sh` 优先读取安装目录配置，现场需确认安装版本。
- 代码：`GlobalLocalizationNode::on_trigger()`、`wait_for_fresh_localizer_input()`、`wait_for_localizer_input_after_arm()`：[派发前分支](../src/robot_global_localization/src/global_localization_node.cpp#L293)、[freshness 与软等待](../src/robot_global_localization/src/global_localization_node.cpp#L1044)、[默认参数](../src/robot_global_localization/config/global_localization.yaml#L9)。

<a id="l02"></a>

### L02 localizer 重载后稳定与新输入门控

- 作用：组件重载成功不立即等同于可接受下一次定位，先证明新 localizer generation 已进入可用状态。
- 触发拒绝条件：重载后尚未达到最短 settle 时间；触发服务未连续达到所需 ready 次数；输入序号没有超过重载时基线；输入年龄无效或过期。时钟无效会清空连续 ready 计数并继续等待。
- 阻断结果：在向 Isaac 派发下一次触发前等待；到期返回 `LOCALIZER_POST_RELOAD_NOT_READY`，本次不派发。这个检查不重复执行组件重载。
- 放行/恢复：settle、服务连续 ready 和新鲜新输入同时成立时自动清除 pending。超时本身不清除 pending，也不自动放行；后续调用若条件已满足仍可通过，必要时通过新的成功重载重新建立基线。
- 仓库状态：实际非幂等成功 reload 后 arm；默认最短 settle 1 秒、readiness 界限 8 秒、服务连续 ready 3 次、输入年龄上限 0.5 秒。
- 代码：`PostReloadReadinessGate::arm()/observe()`、`GlobalLocalizationNode::wait_for_post_reload_readiness()`：[状态策略](../src/robot_global_localization/src/post_reload_readiness_gate.cpp#L21)、[接入触发流程](../src/robot_global_localization/src/global_localization_node.cpp#L955)、[reload 后 arm](../src/robot_global_localization/src/global_localization_node.cpp#L522)。

<a id="l03"></a>

### L03 显式定位结果完成证据门控

- 作用：防止把“触发服务返回”误报为“定位完成”。必须观察本次结果/bridge 处理、bridge 接纳及可用的 map→odom。
- 触发拒绝条件：服务调用异常，或到期既无响应也无结果；等待期内没有本次 arm 对应结果/明确 bridge 接纳；bridge 给出实质拒绝；map→odom owner 错误、缺失或发布健康证据不满足。map→odom 还需能实际查询到 TF。
- 阻断结果：请求返回 `accepted=false`，区分 `ISAAC_DISPATCH_TIMEOUT`、`LOCALIZATION_RESULT_TIMEOUT`、`BRIDGE_REJECTED_RESULT`、`BRIDGE_ACCEPT_TIMEOUT`、`MAP_TO_ODOM_*`。不把旧 map→odom 的存在当作本次定位成功。
- 放行/恢复：当前期限内观察到完整证据则自动成功。直接服务响应仍 pending、但结果已经出现时可以继续；预期的 AMCL observe-only 拒绝、某些瞬态 triggered stale 拒绝不会立即结束等待。最终超时或实质拒绝后需上层重新触发，旧 TF 不会因等待结束而自动变成成功证据。
- 仓库状态：默认结果等待 20 秒、bridge 接纳等待 12 秒、map→odom 等待 8 秒，`require_bridge_acceptance=true`；发布健康年龄/间隔界限为 1000 毫秒。发布健康判断允许 TF 年龄或 bridge 发布间隔任一符合界限，但 owner、已有 map→odom 和实际 TF 查询仍须成立。
- 代码：`on_trigger()`、`wait_for_bridge_acceptance()`、`map_to_odom_ready()/wait_for_map_to_odom()`：[结果链](../src/robot_global_localization/src/global_localization_node.cpp#L374)、[bridge 接纳](../src/robot_global_localization/src/global_localization_node.cpp#L1277)、[TF 完成证据](../src/robot_global_localization/src/global_localization_node.cpp#L1478)。

<a id="l04"></a>

### L04 localizer 资产身份与安全重载门控

- 作用：只把符合请求身份、路径与地图参数契约的资产加载为当前 localizer，并防止从未经证实的持久化记录恢复“已定位”状态。
- 触发拒绝条件：楼栋/楼层/地图/epoch/digest 或受限资产路径不符合契约；同一 active identity 却指向不同 canonical 路径；组件管理器不可用、目标组件快照不符合预期；bootstrap 的 ready/confirmed 上下文、current manifest 身份或 live localizer 参数不匹配。不能仅凭一个 digest 字符串宣称已在此层重新计算全部资产内容摘要。
- 阻断结果：拒绝资产应用或拒绝建立 active identity。检查在卸载前失败时不进入卸载；目标加载失败不会宣称切换成功。
- 放行/恢复：完全相同 identity 和路径可幂等成功；新资产在卸载/加载确认成功后更新 identity 并增加 generation。加载失败时仅在目标组件明确 absent 的安全条件下尝试恢复旧组件；presence 不明时不盲目再加载。回滚失败保留 not-ready，需修复组件/资产后重新走流程。bootstrap 由 wrapper 定时重试，默认重试间隔 5 秒。
- 仓库状态：真实组件 reload 与 runtime-context bootstrap 路径存在；允许资产根默认 `/workspaces/njrh-v3/workspace1/maps_release`。这项启用不等于整条跨楼层移动任务已获授权。
- 代码：`IsaacAssetReloader::apply()`、`bootstrap_from_runtime_context()`：[应用与幂等检查](../src/robot_global_localization/src/isaac_asset_reloader.cpp#L810)、[失败回滚](../src/robot_global_localization/src/isaac_asset_reloader.cpp#L890)、[bootstrap 证据](../src/robot_global_localization/src/isaac_asset_reloader.cpp#L975)。

<a id="l05"></a>

### L05 Isaac 结果必须属于当前显式 arm

- 作用：屏蔽未请求的连续 Isaac 结果和上一次请求排队残留的旧结果。
- 触发拒绝条件：没有显式 force-accept arm；结果 stamp 早于 arm 时刻减允许 slack。floor transition 已 failure-locked 时，连新的 force-accept arm 请求也被拒绝。
- 阻断结果：结果被忽略，canonical map→odom 不更新；不会因为某个旧结果抵达就消耗为一次有效定位。
- 放行/恢复：显式 arm 后，属于当前允许时间范围的结果进入后续候选检查。pre-arm 结果被忽略后仍可等待后续合格结果。成功应用或明确的大跳变拒绝会消费 arm；floor failure lock 必须先走恢复流程。这里没有“等几秒就把未 arm 结果放行”的逻辑，wrapper 的请求超时也不能等同于此处自动撤销 arm。
- 仓库状态：启用，`force_accept_min_pose_stamp_slack_sec=1.0`。显式结果不因总传输耗时超过 `triggered_max_result_age_ms` 就一律拒绝，仍必须满足 L06 的原 stamp TF 历史等条件。
- 代码：`explicit_localization::classify_isaac_result()`、`LocalizationBridgeNode::on_pose()`、`on_force_accept_request()`：[分类策略](../src/robot_localization_bridge/include/robot_localization_bridge/explicit_localization_result_policy.hpp#L18)、[结果接入](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L902)、[arm 请求](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1557)。

<a id="l06"></a>

### L06 定位候选时间、frame、协方差和 TF 有效性门控

- 作用：在计算 map→odom 前，证明候选与连续 odom 处于可解释的同一时间/坐标链中，并筛去不可靠的 AMCL 候选。
- 触发拒绝条件：pose 超前当前时间超过 50 毫秒；非显式候选过期；pose frame 不是 map；启用协方差限制时超过 XY/yaw 限值；原 pose stamp 处找不到 odom→base_link；最新 odom TF 又已超过允许年龄。多个 TF 子条件合计一类，不逐项凑数。
- 阻断结果：候选无效，不更新 map→odom，并记录具体拒绝原因。bridge 局部 odom 超时也会报 localization health 不健康，不能据此假定旧 TF 被直接删除。
- 放行/恢复：后续有效候选自动再评估；仅 `tf_history_missing` 类型可保留同一 Isaac pose 供后续重试，其它失败通常标记该 stamp 已使用。缺失历史 TF 不会靠改 pose stamp 解决；显式结果虽豁免 wall-age 限制，但不豁免原 stamp TF 查找和最新 TF 年龄。
- 仓库状态：AMCL 年龄上限 1000 毫秒，最新 odom TF 上限 100 毫秒，历史缓存 30 秒；AMCL XY covariance ≤0.15、yaw covariance ≤0.10（协方差原始单位，不能当作角度阈值）。
- 代码：`build_candidate()`、`lookup_odom_base()`、`candidate_should_retry_later()`、`refresh_state()`：[候选构建](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1861)、[历史及最新 TF](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1751)、[retry 分支](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2223)、[odom health](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2964)。

<a id="l07"></a>

### L07 AMCL 来源、seed 与观测状态准入

- 作用：AMCL 只在初始全局参考建立且自身输入链可用时成为 map→odom 修正来源。
- 触发拒绝条件：AMCL 输入关闭；尚无 odom 或初始 map→odom；seed 未成功；pose 超龄；scan-admission 状态未收到、超过 2.5 秒、输出频率不大于零或存在错误。`shadow` 模式只观察不应用；只有把 `amcl_accept_corrections_while_moving` 设为 false 后，运动中候选才因这一子条件被拒绝。
- 阻断结果：当前 AMCL 候选不更新 map→odom；不影响已经存在的初始全局参考自动消失。Isaac 显式目标尚在平滑中的互斥时序归 L11。
- 放行/恢复：新鲜状态、seed 和 pose 到齐后，后续回调自动重新判定；shadow 需切为 gated 才允许应用。运动限制若启用，停止后可恢复；不存在超时自动将 shadow 改为 gated 的逻辑。
- 仓库状态：`amcl_localization_profile.env` 默认 `gated`，`run_localization_bridge.sh` 覆盖 YAML 中的 `amcl_input_enabled=false` 和 `amcl_gate_mode=shadow`。默认允许运动中修正，scan admission 开启；只看 YAML 会得出错误启停结论。
- 代码：`on_amcl_pose()`、`amcl_scan_admission_ready()`、`accept_amcl_candidate()`：[来源前置条件](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1008)、[scan 状态](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1803)、[shadow/运动分支](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2445)、[脚本覆盖](../scripts/jetson/runtime_overlay/scripts/run_localization_bridge.sh#L35)、[profile](../scripts/jetson/runtime_overlay/config/amcl_localization_profile.env#L13)。

<a id="l08"></a>

### L08 全局修正暂停与 owner/transaction 租约门控

- 作用：楼层切换等流程可暂时冻结全局修正，且一个调用方不能误释放另一调用方持有的暂停。
- 触发拒绝条件：存在任一 pause 记录时，Isaac、AMCL 和 refine 候选都不能应用；租约命令需要精确 owner/transaction，旧的或重复 command sequence 被拒绝。这里是业务暂停记录，不是线程 mutex。
- 阻断结果：返回/记录 `GLOBAL_CORRECTION_PAUSED`，全局修正保持冻结；已有 map→odom 可以继续由 bridge 发布，不等于失去整棵 TF。
- 放行/恢复：各 owner 用匹配的 transaction 和更高 command sequence 释放自己的记录，全部记录清空后自动允许后续候选。释放不存在的精确记录会建立 sequence fence，但不会清掉其它 owner。**没有 TTL 到期自动释放机制**；持有方异常退出后不能假设等一会儿就解除。
- 仓库状态：暂停服务和租约服务启用；legacy SetBool 只管理自己的兼容记录。是否当前 paused 必须读运行状态，不能由参数静态判断。
- 代码：`CorrectionPauseArbiter::apply()/snapshot()`、`accept_candidate()/accept_amcl_candidate()`：[租约及顺序](../src/robot_localization_bridge/src/correction_pause_arbiter.cpp#L8)、[Isaac 阻断](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2254)、[AMCL 阻断](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2439)。

<a id="l09"></a>

### L09 floor-transition 上下文准入与失败锁

- 作用：把跨楼层的“旧参考失效→目标定位→提交新参考”变成显式事务，避免普通 AMCL 候选穿过切图阶段。
- 触发拒绝条件：BEGIN/COMMIT 接口未启用；目标身份/事务或命令顺序不符；已有冲突事务；BEGIN 没有证明 floor-manager 的精确 pause 租约；COMMIT 时仍有 pause、没有 BEGIN 后的新显式定位序号，或 map→odom target 尚未完成平滑/发布。failed-locked 时所有候选被拒绝；transition active 时只允许未暂停的显式候选。
- 阻断结果：拒绝阶段命令，或拒绝全局候选；BEGIN 成功会把旧 runtime context 置为 invalid/recovery-required。普通 ABORT 留下 failure lock，不恢复“旧图仍健康”的假象。
- 放行/恢复：持有正确事务，完成目标定位和 TF 发布证据，释放所有 pause 后 COMMIT。仅能证明还没改资产且精确源上下文未变时，`abort_pre_mutation()` 可恢复源上下文；已有 failure lock 不能由该捷径清除。没有超时自动解锁分支，须显式恢复协议。
- 仓库状态：包/YAML 默认 false，但 overlay `run_localization_bridge.sh` 默认传入 `live_floor_transition_service_enabled=true`。这只说明 bridge 接口默认开，不等于 floor-manager 整条跨楼层动作默认获准。
- 代码：`on_begin_floor_transition_request()`、`FloorTransitionContext::begin()/commit()/abort()/abort_pre_mutation()/candidate_allowed()`：[接口开关](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1382)、[事务状态机](../src/robot_localization_bridge/src/floor_transition_context.cpp#L115)、[候选准入](../src/robot_localization_bridge/src/floor_transition_context.cpp#L348)、[overlay 覆盖](../scripts/jetson/runtime_overlay/scripts/run_localization_bridge.sh#L59)。

<a id="l10"></a>

### L10 map→odom 修正幅度与连续一致性门控

- 作用：普通在线定位不能把机器人突然校正到远处；AMCL 中等修正需要重复一致，不因单次候选直接跳变。
- 触发拒绝条件：显式同图修正超过强制平移上限；不允许大修正时越过普通上限；非显式路径超过在线平移/转角限制；AMCL 候选超过允许区间，或中等候选还未累计足够一致次数。门控衡量机器人参考位姿处的实际 SE(2) 修正，不只比较 map→odom 参数的平移分量。
- 阻断结果：保留当前全局参考，不应用候选；过大修正要求重新走显式定位/恢复。首次尚无 map→odom 的初始锁定不走已有参考的跳变比较，仍受其它候选有效性条件约束。
- 放行/恢复：小 AMCL 修正直接接受；中等修正连续一致后接受，不一致重置基线和计数。显式同图定位可越过普通在线限制，但仍受 20 米限制；只有已证明精确 pending target localizer 的跨地图事务，才能豁免“不同地图原点之间的平移差”限制，不能把任意 force-accept 当无限制放行。
- 仓库状态：AMCL 小区间 0.12 米/约11.5°（0.20 rad），中等区间 0.28 米/约20.1°（0.35 rad），一致次数 3；硬阈值配置 0.60 米/约45.8°（0.8 rad），但中等区间外也会拒绝，不能理解为“0.60 米以内全部允许”。非显式通用在线限值 0.80 米/约45.8°；显式平移上限 20 米。
- 代码：`accept_candidate()`、`accept_amcl_candidate()`、`amcl_candidate_agrees_with_previous_medium()`、`forced_translation_within_limit()`：[通用/显式修正](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2223)、[AMCL 分级](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2457)、[连续一致性](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1973)、[跨图限值例外](../src/robot_localization_bridge/include/robot_localization_bridge/explicit_localization_result_policy.hpp#L89)、[阈值配置](../scripts/jetson/runtime_overlay/config/localization_bridge.yaml#L45)。

<a id="l11"></a>

### L11 Isaac 后 AMCL 精修时序门控

- 作用：Isaac 粗定位完成后，允许一次受控 AMCL 精修，同时避免新旧 seed、正在平滑的 Isaac target 和运动中的数据互相竞争。
- 触发拒绝条件：Isaac 显式 target 尚未 settled；距 seed/refine 参考时刻不足最小延迟；pose 的接收时刻或 stamp 仍属于旧 seed；精修修正过大或一致次数不足。要求静止时一旦开始运动，会放弃本轮精修，不是一直扣住该候选等待停车。
- 阻断结果：延迟/丢弃本轮精修候选；成功、放弃或超过精修时间窗后转回普通 AMCL 门控。普通 AMCL 还可能在 Isaac 接受后的短暂抑制窗口内被忽略。
- 放行/恢复：目标 settled、最小延迟结束且收到 seed 后的新 pose，连续一致后应用；每个显式定位序号最多完成一次精修。运动导致放弃后，即使再停车也不会复活同一序号的精修窗口；新一轮显式定位可产生新机会。可限次调用 no-motion-update 促使 AMCL 产出新 pose，但请求成功不等同于精修已通过。
- 仓库状态：默认启用，窗口 10 秒、最小延迟 0.25 秒、静止要求 true；精修最大 10 米/50°（0.872664626 rad），一致 2 次，相邻候选约定差 ≤0.08 米/约4.6°（0.08 rad）；普通 AMCL 在 Isaac 后默认抑制 2 秒。
- 代码：`evaluate_post_isaac_refine_gate()`、`on_amcl_pose()`、`accept_post_isaac_refine_candidate()`：[时间/运动策略](../src/robot_localization_bridge/include/robot_localization_bridge/post_isaac_refine_gate.hpp#L59)、[状态分流](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L1042)、[幅度与一致性](../src/robot_localization_bridge/src/localization_bridge_node.cpp#L2340)、[配置](../scripts/jetson/runtime_overlay/config/localization_bridge.yaml#L70)。

<a id="l12"></a>

### L12 AMCL scan 数据准入

- 作用：只向 `/scan_amcl` 转发时间、frame 与 TF 合格的原始扫描，并限制输入频率；不通过重写 stamp 掩盖 TF 对不上。
- 触发拒绝条件：配置了节点 warmup 且尚未完成；frame 不符；stamp 超前超过 50 毫秒或超过 1000 毫秒年龄；到达过频；在原 scan stamp 处不能查询到目标 odom 变换。
- 阻断结果：丢弃当前 scan，不发布到 `/scan_amcl`；更新丢弃计数/错误状态，继而可能使 L07 不接受 AMCL 修正。该节点不排队无限等待，也不修改 scan stamp/frame/ranges。
- 放行/恢复：每个新 scan 自动再评估；短暂 TF 查找最多等默认 20 毫秒，失败丢掉该帧，后续帧可自行恢复。节点 `require_seeded` 没有接入 seed 状态，不能把它当已实现的硬 seed 门；当前启动流程也是先启动 relay 再发 seed。
- 仓库状态：profile 默认启用 C++ relay、10 Hz、frame=`lidar_level_link`、target=`odom`。脚本传给节点的 `require_seeded=false`、`require_tf_warmup=false`、`startup_warmup_sec=0.0`；外层另有 AMCL TF warmup 流程，不能把 profile 中名称相似的 `ONLY_AFTER_*` 直接视为节点硬开关。
- 代码：`AmclScanAdmissionNode::onScan()/tfReady()/startupWarmupReady()`：[逐帧准入](../src/robot_localization_bridge/src/amcl_scan_admission_node.cpp#L147)、[seed 未接线说明](../src/robot_localization_bridge/src/amcl_scan_admission_node.cpp#L110)、[节点实参](../scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh#L1017)、[实际启动/seed 顺序](../scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh#L1228)。

<a id="l13"></a>

### L13 IMU bias 静止确认与安全样本更新门控

- 作用：只在确认车辆静止时学习陀螺仪零偏，防止把真实转动误学成传感器偏差。
- 触发拒绝条件：odom 不新鲜或不静止；新鲜 cmd_vel 表示运动；最近一次运动命令仍处于 holdoff；静止持续时间不足；角速度分量非有限数或超过安全样本上限。默认不强制 cmd_vel 必须持续新鲜，但旧运动命令后的 holdoff 仍有效。
- 阻断结果：不更新 bias、不按“静止已确认”分支把角速度归零；如果已有 bias，继续使用已有 bias 校正。此项不是整条 IMU 输出禁令，输出还由 L14 检查。
- 放行/恢复：odom/cmd 条件连续稳定满规定时间后自动恢复 bias 更新；任何不合格状态会重置静止起算时刻。没有人工 unlock 服务要求。
- 仓库状态：odom/cmd 静止判断均开启，odom/cmd timeout 各 0.5 秒，静止要求 2 秒，运动命令 holdoff 1 秒；bias 安全样本角速度上限约1.72°/秒（0.03 rad/s）。
- 代码：`StationaryGate::confirmed()/candidate()`、`sample_is_safe_for_bias_update()`、`on_imu()`：[静止策略](../src/robot_local_state/include/robot_local_state/stationary_gate.hpp#L45)、[样本安全](../src/robot_local_state/src/imu_gyro_bias_filter_node.cpp#L223)、[应用分支](../src/robot_local_state/src/imu_gyro_bias_filter_node.cpp#L428)、[配置](../scripts/jetson/runtime_overlay/config/local_state_imu_bias_filter.yaml#L12)。

<a id="l14"></a>

### L14 corrected IMU 输出质量门控

- 作用：防止重复、过期或无法变换到 canonical frame 的 IMU 衍生数据持续进入 EKF/安全消费者。
- 触发拒绝条件：source stamp 在 1 秒以内倒退或重复；没有新样本 generation；最近样本超过输出年龄上限；启用目标 frame 变换且变换失败。超过 1 秒的 source 时间回退被识别为时间重置并重新建立基线，不等同于永久拒绝该流。
- 阻断结果：跳过当前 corrected IMU 输出并累加计数。不会修改原始 `/lidar_imu`；FAST-LIO 使用的原始流不在此门内。
- 放行/恢复：新 generation、可用 TF 和符合年龄的后续样本到来后自动恢复。timer 不会靠重复发送上一帧让停流看起来健康。采用接收时刻衍生 stamp 时，年龄按接收时间计算；不是宣称原始传感器延迟已消失。
- 仓库状态：`corrected_output_publish_only_new_sample=true`、`corrected_output_require_monotonic_stamp=true`、`drop_output_on_transform_failure=true`、目标 base_link；输出年龄上限 0.20 秒，timer 100 Hz，`corrected_output_preserve_source_stamp=false`。
- 代码：`on_imu()`、`on_corrected_output_timer()`、`transform_corrected_output()`：[单调 stamp](../src/robot_local_state/src/imu_gyro_bias_filter_node.cpp#L414)、[timer 新鲜/去重](../src/robot_local_state/src/imu_gyro_bias_filter_node.cpp#L354)、[变换失败](../src/robot_local_state/src/imu_gyro_bias_filter_node.cpp#L475)、[配置](../scripts/jetson/runtime_overlay/config/local_state_imu_bias_filter.yaml#L26)。

<a id="l15"></a>

### L15 local odom 重发年龄门控

- 作用：防止输入底盘 odom 停流后，wrapper 无限期把旧位姿换新 stamp 重发为“新 odom”。
- 触发拒绝条件：尚无缓存 odom，或距离最近一次输入接收超过 `republish_latest_max_age_sec`。
- 阻断结果：本次重发 timer 不发布；不把 frame 不同的警告误算为拒绝——`on_wheel_odom()` 对 child frame 不同仅警告后规范化 frame。
- 放行/恢复：新的输入 odom 到来后自动恢复重发；超时不会清除缓存但也不会自动放行旧缓存。正常收到消息时是否立即发布另由 `publish_on_callback` 决定。
- 仓库状态：当前默认 `wheel_spin_imu` profile 的 wheel-odom EKF 输入预处理器 `republish_latest=true`、年龄上限 0.5 秒。最终 EKF 自身行为不是本项实现。
- 代码：`LocalStateNode::on_wheel_odom()/on_republish_timer()`：[重发门](../src/robot_local_state/src/local_state_node.cpp#L490)、[frame 仅告警](../src/robot_local_state/src/local_state_node.cpp#L455)、[profile 配置](../scripts/jetson/runtime_overlay/config/local_state_wheel_odom_ekf_spin_imu.yaml#L24)、[默认 profile](../scripts/jetson/runtime_overlay/config/local_state_ekf_profile.env#L83)。

<a id="l16"></a>

### L16 canonical `/scan` 唯一发布者与切换所有权门控

- 作用：导航时由 resident pointcloud 节点持有 `/scan`，建图时交给 corrected-cloud 切片节点；不能让两者同时占有同一 canonical topic。
- 触发拒绝条件：预期 owner/count 不满足；关闭 resident scan 服务不可用或拒绝；关闭后 publisher 未退场；恢复时图上仍有其它 publisher；新建图 publisher 的唯一 owner 无法证明。
- 阻断结果：拒绝启动/交接/恢复对应扫描链，不盲目再开一个 publisher。此流程不停止 resident 点云主干，因此 `/lidar_points` 可继续供 FAST-LIO 使用。
- 放行/恢复：先做幂等 owner 检查；topic 已空时允许重新开启 resident 并验证唯一 owner。服务响应不确定时以实际 ROS graph 证明请求效果；图上身份短暂 UNKNOWN 时继续有界观察。仍失败则入口退出；停止建图/修复冲突后再走交接。不是靠固定 sleep 认定已唯一。
- 仓库状态：导航和 `run_projected_map.sh` 建图入口均调用；通用所有权等待默认 12 秒，建图新 owner 等待默认 30 秒。
- 代码：`set_resident_scan_output()`、`restore_navigation_scan_owner()`、`wait_for_scan_owner()`：[所有权辅助函数](../scripts/jetson/runtime_overlay/scripts/scan_ownership_helpers.sh#L78)、[恢复逻辑](../scripts/jetson/runtime_overlay/scripts/scan_ownership_helpers.sh#L125)、[导航入口](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1737)、[建图交接](../scripts/jetson/runtime_overlay/scripts/run_projected_map.sh#L708)。

<a id="l17"></a>

### L17 导航启动前 local-state 新鲜/稳定门控

- 作用：先证明连续局部运动估计可用，再让全局定位和导航使用它；API resume 比一般启动多要求连续稳定观测。
- 触发拒绝条件：local-state endpoint 不正确/不可见；odom 缺失、过旧或超前；odom→base_link 缺失或过旧；stable 模式下连续样本数不足或时间戳未持续前进。
- 阻断结果：返回 `LOCAL_STATE_ENDPOINT_NOT_READY`、`LOCAL_STATE_ODOM_NOT_FRESH`、`ODOM_BASE_TF_NOT_FRESH` 或 `LOCAL_STATE_ODOM_TF_NOT_STABLE`，启动流程失败而不是直接进入初始定位。
- 放行/恢复：期限内条件满足自动继续。fresh 模式可以使用已通过的 runtime-health snapshot 快路；stable 模式仍做连续样本观察。超时后需恢复 odom/TF 并重新启动/恢复流程，不会以“等够时间”替代证据。
- 仓库状态：默认等待 12 秒；odom 年龄上限 0.75 秒、允许超前 0.25 秒、TF 年龄上限 0.25 秒；API resume 要求 3 个稳定样本。普通入口 fresh，API resume 入口 stable。
- 代码：`ensure_common_local_state_ready_for_navigation_start()`、`runtime_readiness_probe` 的 stable-local-state 实现：[准入函数](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1242)、[API resume 调用](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1758)、[普通启动调用](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1798)。

<a id="l18"></a>

### L18 localizer stack 启动就绪门控

- 作用：证明全局定位 wrapper、Isaac 服务、当前选定地图和 FlatScan 组成的栈已经可用；与 L01 的单次请求即时输入检查不同。
- 触发拒绝条件：composite probe 失败且详细回退仍无法确认两项触发服务、所选地图的 map-server/OccupancyGrid 或 FlatScan publisher；修复 FlatScan 后仍不可用。可选的 localization-result publisher 预检查只有显式开启才阻断。
- 阻断结果：不能继续初始全局定位/声明栈 ready，写入对应启动失败原因。
- 放行/恢复：composite 成功即可继续；失败时进入分项诊断、map-server 确认及受控 FlatScan helper 修复，再验证。修复期内恢复可自动继续，最终失败退出启动流程；不是 composite 第一次失败就无条件拒绝整个启动。
- 仓库状态：composite/service/map 等默认等待 45 秒；FlatScan 首次等待 5 秒、修复等待 20 秒；`NJRH_INITIAL_LOCALIZATION_REQUIRE_RESULT_PUBLISHER=false`，结果是否真正产生由 L03 负责。
- 代码：`ensure_localization_stack_ready_for_navigation()`、`recover_flatscan_helper_for_navigation()`：[就绪与回退](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1314)、[入口阻断](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1809)、[composite 实现](../src/robot_bringup/src/runtime_readiness_probe.cpp#L1583)。

<a id="l19"></a>

### L19 强 runtime-ready 上下文提交门控

- 作用：把“本次启动确实完成了一次指定楼层的显式定位”持久化为可审计事实，拒绝借用旧 TF 或另一事务的定位结果。
- 触发拒绝条件：没有正的、本事务 explicit sequence；live bridge 序号不等于预期；bridge 状态观察失败且无本事务成功 trigger 证据；替代证明中定位 owner 已退出或 map→odom 不新鲜；上下文写盘失败或写后不匹配当前楼层。
- 阻断结果：不设置 `runtime_ready=1`，不能写成可信 `ready/confirmed`；启动入口因提交失败退出。L03 是单次请求结果门，这里额外保护持久化身份与事务绑定，二者不是同一个 if 的重复计数。
- 放行/恢复：正常用 live bridge 精确序号证明；若 CLI 暂时漏观测，可在已有本事务成功 trigger、owner 存活且 TF 新鲜的前提下使用受限回退。写盘并回读确认全部成功才放行；不同事务的新序号不会被当作更好证据自动接受。
- 仓库状态：默认启用；bridge 状态等待 15 秒，替代 TF 等待 5 秒、年龄上限 0.5 秒。回退不是关门开关，仍要求完整替代证据。
- 代码：`commit_runtime_ready_context()`：[序号、替代证据、持久化](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L97)、[调用失败退出](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1940)。

<a id="l20"></a>

### L20 Nav2 lifecycle 与 global costmap 启动门控

- 作用：launch 进程活着不足以说明 Nav2 可接单，须等待标准节点 active 和 global costmap 就绪。
- 触发拒绝条件：导航层进程退出；标准 Nav2 lifecycle 节点没有全部 active；global costmap 未满足就绪探针。
- 阻断结果：不声明 runtime ready；`run_navigation_runtime_services.sh` 在失败时记录 failed 并拆除未完成的 navigation layer。
- 放行/恢复：期限内 lifecycle 激活和 costmap 满足后自动继续；超时不能只凭进程在运行而放行，需要修复激活/地图链后再启动。
- 仓库状态：runtime-services 入口开启，外部 lifecycle bringup 默认启用，其等待上限默认 210 秒，global costmap 默认 90 秒。`run_nav2_navigation.sh` 自身已移除阻塞 map/topic/TF readiness，**不代表外层 runtime-services 的这一门已删除**。
- 代码：`wait_for_nav2_layer_ready()`、`wait_for_global_costmap()`：[外层准入](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1374)、[失败处理](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1895)、[global costmap probe](../src/robot_bringup/src/runtime_readiness_probe.cpp#L1612)、[内层脚本去阻塞声明](../scripts/jetson/runtime_overlay/scripts/run_nav2_navigation.sh#L262)。

<a id="l21"></a>

### L21 AMCL tracking 对整体 runtime-ready 的可选阻断

- 作用：可选择要求 AMCL 的启动/seed/状态流程完成后，整体导航 runtime 才标 ready；它不同于每帧 scan 与 correction 的 L07/L12。
- 触发拒绝条件：打开 `NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY` 后，后台 readiness 失败且前台有界恢复仍失败，或最终非 disabled 模式的 `AMCL_READY` 仍为 false。
- 阻断结果：不提交整体 ready，上下文记 failed，脚本退出；保留定位/Nav2 供诊断的策略由入口负责。默认关闭时 AMCL 尚未 ready 本身不阻断整体 ready，但后台维护/heartbeat 仍存在，不能写成“AMCL 不再检查”。
- 放行/恢复：开启时后台完成或前台重试成功才继续；关闭时后台持续尝试 readiness。另需注意 AMCL runner 默认允许 seed 后 static-standby：可跳过 fresh scan/pose 的同步等待，`AMCL_READY` 不必然等价于刚有一次新修正；L07 仍会在真正应用候选时检查 pose 和 scan 状态。
- 仓库状态：整体阻断默认 **关闭**（`${NJRH_REQUIRE_AMCL_TRACKING_FOR_NAV_READY:-false}`）；AMCL 模式默认 gated。runner 的 `NJRH_AMCL_STATIC_STANDBY_WITHOUT_POSE_OK`、`...SKIP_POSE_WAIT` 等快路默认 true。
- 代码：runtime-services ready 分支、`complete_amcl_readiness_sequence()`：[总 ready 开关](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L1902)、[后台恢复](../scripts/jetson/runtime_overlay/scripts/run_navigation_runtime_services.sh#L974)、[seed/static-standby 实际逻辑](../scripts/jetson/runtime_overlay/scripts/run_amcl_shadow_localization.sh#L1222)。

<a id="l22"></a>

### L22 IMU bias filter 输出就绪后才启动 EKF

- 作用：确保 corrected IMU 和 bias 两路的预期生产者出现且确实产出过消息，避免只启动了进程就立刻依赖其输出。
- 触发拒绝条件：bias filter 子进程没保持存活；等待期内没有观察到 `imu_gyro_bias_filter` 对两个 topic 的 publisher，或任一路未收到消息。
- 阻断结果：`run_local_state.sh` 退出，EKF 启动步骤不再继续。该探针检查“来源＋收到消息”，并不证明 bias 已收敛，也不逐条检验消息 header 年龄；不要把它写成完整 IMU 质量认证。
- 放行/恢复：两个 publisher 与两路消息在期限内被观察到后自动继续；超时需修复输入/TF/filter 后重跑 local-state 启动。允许显式关闭 READY_CHECK，但不是默认行为。
- 仓库状态：`LOCAL_STATE_IMU_BIAS_FILTER_ENABLED=true`、`LOCAL_STATE_IMU_BIAS_FILTER_READY_CHECK=true`，等待 8 秒；使用 IMU 的 EKF profile 另要求 bias filter 必须开启。
- 代码：`run_local_state.sh` filter/EKF 启动段、`wait_for_imu_bias_filter()`：[启动阻断](../scripts/jetson/runtime_overlay/scripts/run_local_state.sh#L371)、[实际探针](../src/robot_bringup/src/runtime_readiness_probe.cpp#L628)。

<a id="l23"></a>

### L23 建图前置数据与 cloud/odom/原 stamp TF 配对门控

- 作用：在当前 corrected-cloud→slam_toolbox 入口确认 resident 基础链健康，且 FAST-LIO 的点云、odom 和 TF 来自可配对的同一建图实例。
- 触发拒绝条件：composite preflight 中静态 TF、scan/odom freshness、local-state endpoint、参考 odom 或相对位置 sanity 不成立；FAST-LIO corrected cloud/odom 没准备好或 pair probe 不通过；发布后的 mapping scan 在原始 stamp 处不能转换到指定 odom frame。`/scan` 唯一 owner 的同一检查归 L16，不在这里再单列一次。
- 阻断结果：拒绝继续启动或判定建图 runtime 成功，入口非零退出；不会通过添加 relay/重写 scan stamp 掩盖错误。与已退休 R01 不同，本项是启动阶段的链路证据检查，不是生产逐帧 TF relay。
- 放行/恢复：各有界等待内恢复则继续；超时退出，cleanup 负责收束本次建图及扫描所有权，再次启动重新验证。关闭 composite 只改走 legacy 分项 preflight，不是取消所有门控。
- 仓库状态：`SLAM2D_COMPOSITE_PREFLIGHT_ENABLED=true`；默认 odom source=fastlio。scan 年龄上限 0.5 秒，local odom 上限 1 秒；FAST-LIO cloud/odom 最长各等 60 秒，pair 等 8 秒，原 stamp TF 最长 30 秒且要求 3 个合格样本。相对 odom sanity 的默认距离界限是 25 米，属于粗异常检查，不能当定位精度保证。
- 代码：`require_resident_common_mapping_prereqs()` 及建图启动尾段：[composite/legacy 路由](../scripts/jetson/runtime_overlay/scripts/run_projected_map.sh#L482)、[FAST-LIO 配对](../scripts/jetson/runtime_overlay/scripts/run_projected_map.sh#L655)、[原 stamp TF 证据](../scripts/jetson/runtime_overlay/scripts/run_projected_map.sh#L747)、[默认值](../scripts/jetson/runtime_overlay/scripts/run_projected_map.sh#L16)。



<a id="group-h"></a>

## 10. HTTP/API 基础设施准入（2 项）

<a id="h01"></a>

### H01 HTTP API token 鉴权

- 作用：配置密钥时，只有携带匹配 `X-Robot-Token` 的请求才能进入认证后的API业务路由。
- 触发/拒绝条件：有效API token非空，但请求头缺失或不匹配。配置值为空时会尝试读取 `ROBOT_API_TOKEN` 环境变量；最终仍为空则该token检查直接放行。`OPTIONS` 在此认证入口提前返回，不能写成所有HTTP请求都验token。
- 阻断结果：返回HTTP401，不执行该业务路由；不改变机器人运动状态，也不自动锁底盘。
- 放行/恢复逻辑：客户端携带正确token重新请求即可；没有基于失败次数的永久账户锁。服务端token来源改变是否需重启按其启动配置流程处理，不凭空声称支持热更新。
- 当前仓库配置状态：overlay [`api_token: ""`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L5)；现场环境变量未读取，因此不能断言“鉴权关闭”或“鉴权开启”。
- 源码函数：[ApiGatewayModule 构造时环境变量回退](../src/robot_api_server/src/infrastructure/http/api_gateway_module.cpp#L27)、[token_allowed](../src/robot_api_server/src/infrastructure/http/api_gateway_module.cpp#L120)、[dispatch_authenticated](../src/robot_api_server/src/infrastructure/http/api_gateway_module.cpp#L202)。

<a id="h02"></a>

### H02 HTTP 活跃连接容量准入

- 作用：限制同时占用服务器处理资源的连接，避免无限接收客户端连接把API拖垮；它不是按请求频率的业务限流。
- 触发/拒绝条件：新连接被accept后，`active_connections >= max_connections`。
- 阻断结果：向新连接返回HTTP503 `server busy`并关闭该连接，不把它加入处理队列；原有连接继续处理，不直接停止机器人任务。
- 放行/恢复逻辑：原连接关闭/处理结束并归还计数后，新客户端重连可自动通过；被拒连接不会在服务端排队等候，也不需要人工清锁。
- 当前仓库配置状态：[`max_http_connections=16`](../scripts/jetson/runtime_overlay/config/robot_api_server.yaml#L6)，网关构造时将范围限制在4～64，未见“0表示无限”模式；现场可有启动配置覆盖。
- 源码函数：[HttpServer::Impl::serve 的容量判断](../src/robot_api_server/src/infrastructure/http/http_server.cpp#L346)、[worker_loop 的连接计数释放](../src/robot_api_server/src/infrastructure/http/http_server.cpp#L232)、[ApiGatewayModule 容量配置归一化](../src/robot_api_server/src/infrastructure/http/api_gateway_module.cpp#L24)。

## 11. 附录：质量参数、退休实现与只诊断对象


<a id="q01"></a>

### Q01 Isaac 匹配质量参数（附录，不确认独立硬门控、不计数）

- 作用：控制第三方 Isaac scan-to-map 匹配误差的计算/输出质量配置，与 Nav2 障碍距离、inflation 和碰撞净空无关。
- 触发拒绝条件：本地 overlay 明确配置 `max_beam_error=0.50`，同时有 `max_output_error=0.30`、`min_output_error=0.05`、`min_scan_fov_degrees=115.0`。但在本次检查的本地实现范围没有找到 Isaac 核心误差计算/拒绝代码，**不能据此断言任意 beam 误差超过 0.50 就拒绝整个位姿，也不能猜测边界比较符号、聚合公式或重试策略**。本地说明称 `max_beam_error` 限制单 beam 对总误差的贡献，更接近误差截断配置。
- 阻断结果：配置会影响第三方匹配质量/输出选择；是否、何时硬拒绝整次结果只能以匹配版本的第三方实现或隔离验证确认。没有结果时，上层 L03 会等到超时并返回失败，这不等于证明这些质量参数正是拒绝原因。
- 放行/恢复：输入/匹配结果满足第三方实现规则时才会有可供上层评估的输出；此处无法从本地 YAML 推断自动重试或清锁行为。改小/改大质量参数也不保证解除场景几何混淆，应通过静止条件下重复试验验证。
- 仓库状态：overlay YAML 为 0.50；`occupancy_localization.launch.py` 把该默认配置加入组件参数列表，再叠加调用者的 `localizer_params`，因此入口实参可能覆盖。这里只确认仓库契约，未读取现场组件参数。
- 代码/契约：无本地核心算法函数可列；`generate_launch_description()` 负责参数注入：[质量参数](../scripts/jetson/runtime_overlay/config/jt128_occupancy_grid_localizer.yaml#L10)、[参数合并](../scripts/jetson/runtime_overlay/launch/occupancy_localization.launch.py#L80)、[本地调参说明](../src/robot_global_localization/docs/isaac_relocalization_tuning.md#L3)。

<a id="r01"></a>

### R01 已退休：mapping_scan_tf_gate（附录，不计正式项）

- 作用：旧建图路径在原 scan stamp 的 TF 未到齐时缓存 scan，TF 可用并经过可选 post-settle 后再放行。
- 触发拒绝条件：无效 scan、队列溢出，或等待原 stamp TF 超过有界时间；TF 不可用时暂留队列而不重写 stamp。
- 阻断结果：当前旧 relay 的 scan 暂不输出；超时/溢出丢帧。
- 放行/恢复：TF 后到且等待未超时时自动释放；超过上限直接丢弃，后续帧另行判断。不需要人工“开门”，也不无限缓存。
- 仓库状态：**生产已退休**；README 明确只保留为可编译的历史诊断工具，当前 mapping launch 直接把 corrected slice 发布到 `/scan`，不运行此 relay。不能把“源码还在/可编译/进程清理 pattern 仍提到”误算为当前启用。
- 代码：`MappingScanTfGatePolicy::evaluate()`、`MappingScanTfGateNode::on_scan()/process_queue()`：[旧策略](../src/robot_fastlio_mapping/include/robot_fastlio_mapping/mapping_scan_tf_gate_policy.hpp#L44)、[旧实现](../src/robot_fastlio_mapping/src/mapping_scan_tf_gate_node.cpp#L164)、[退休声明](../src/robot_fastlio_mapping/README.md#L23)、[现行直连 launch](../scripts/jetson/runtime_overlay/launch/jt128_slam_toolbox_mapping.launch.py#L115)。


<a id="r02"></a>

### R02 ElevatorLocalizationReplanGate 当前只记录修正诊断

`ElevatorLocalizationReplanGate` 名称仍带 Gate，但当前定位修正幅度达到阈值时只标记一次诊断事件；代码明确允许 FollowPath 活跃期间 map→odom 改变，控制器每周期变换 map-frame 目标。稳定持续时间只用于合并一段修正事件，不暂停运动，也不因此请求替换路径。返回枚举中保留 `kHold`/`kRequestReplan` 不能当调用链实际存在对应运动阻断的证据。

源码：[observe 的修正事件处理](../src/robot_nav_config/include/robot_nav_config/elevator_localization_replan_gate.hpp#L99)、[调用方消费诊断事件](../src/robot_nav_config/src/elevator_scoped_controller.cpp#L565)。非有限输入的防御性返回不在本目录按普通输入检查新增运动门控；本项不计入 N/D/F/E 的 20 项。

## 12. 常见现象对应哪些门控

这张表是查证入口，不是凭提示文字直接认定根因。一次失败可能同时留下多个后果，优先对齐同一任务 ID、事务 ID 和时间。

| 现象／提示 | 优先核对 | 必须区分的事情 |
|---|---|---|
| `DELAYED_SIDE_EFFECT_UNKNOWN` | A18 | 请求超时与远端未执行是两回事；计数不以等待时长自动归零 |
| 普通导航被在桩状态阻止 | S07、A09、A10 | API 是否收到新鲜安全状态；实际接触、持久在桩证据与内存锁不是同一字段 |
| 已有充电接触，但精对桩还报失败 | S16、D03、D04 | 接触立即停与任务取得实际停车／接触确认并宣告成功是不同交接 |
| 已遥控离开桩，普通导航仍被挡 | S16、S17、A10 | 遥控移动本身未必产生标准离桩会话及释放证据；不可据此直接清全部锁 |
| `FLOOR_TRANSITION_BLOCKED`／`FAILED_LOCKED` | A16、F03、L09 | API 负向互锁、floor-manager 事务锁和 bridge 上下文锁不是同一份状态 |
| 电梯恢复提示与旧日志不一致 | E02、E03 | 持久恢复锁默认关闭不代表活动任务互斥、取消终态证明也被关闭 |
| 位姿过期／`map->odom` 未就绪 | A04、A08、L03、L06、L17 | 消息存在、原始 stamp 的 TF 可用、发布源正确和本次定位成功是不同证据 |
| 无进展失败 | N05，再看 N01–N03、S10、S14、S18 | progress checker 是失败判定点；应继续找为什么没有位姿进展，不能直接认定它就是起因 |
| `safety=OK`，车却不动 | N01–N07、S09–S15、S18 | 上游可能发零、选中另一个命令源，或底盘仍在换模；主状态 OK 不覆盖全部子门控 |
| 自旋一下、停一下 | N01、N02、A07、S10、S14、S18、S19 | 路径／控制阶段重入、合法停稳交接、命令优先窗口和底盘握手需要分开取证 |
| 建图 scan 低频／没数据 | L16、L23、R01 | 先确认实际生产者与交接状态；不能仅凭历史 TF gate 文件还在就断言它在丢帧 |

## 13. 默认关闭、替代及证据边界速查

| 条目 | 本地审计结论 |
|---|---|
| S06 | safety 最终定位健康门默认 false，但上游定位就绪门仍存在 |
| S14 的 local-odom 子条件 | 默认 false；轮速／IMU 稳定条件仍在，不是整条 spin settle 关闭 |
| A15 | 参数要求实际自旋，但 delegate=true 时通常不走 API 自做 staging 分支 |
| N03 的电梯子路径 | 三组 `command_clearance_check_enabled=false`；不等于普通导航全部碰撞检测关闭 |
| N04 | 存在离线候选恢复实现；是否被当前实际 BT 使用、是否部署要另外确认 |
| E03 | 电梯生产适配显式关闭持久恢复锁；活动执行互斥与终态证明不随之消失 |
| E04 | 高层 Mission FSM 库有失败锁；本次没有证明现场导航入口在调用该库 |
| L21 | AMCL tracking 对整体 runtime-ready 的同步阻断默认 false；后台定位准入仍存在 |
| F01 | 基础配置 false，但 Jetson `run_floor_manager.sh` 默认覆盖为 true；不能只看基础 YAML |
| Q01 | 质量参数契约，缺第三方核心实现证据，不计独立门控 |
| R01、R02 | 分别为退休 relay 与只诊断的 gate 命名对象，均不计正式条目 |
| H01 | 最终 token 可能来自环境变量；YAML 空字符串不足以判断鉴权开启或关闭 |

## 14. 这些机制如何形成可追溯证据

排查时先确定“哪个层级没有放行”，再分析该条为何不通过：

1. **请求层**：保留接口、返回码、目标／事务 ID，区分“请求被拒”和“请求接受后任务失败”。对应 A、H 组。
2. **任务层**：对齐 Nav2 action 的接受、取消、终态，以及 docking／floor／elevator 状态。API 超时不能替代终态证据。对应 N、D、F、E 组。
3. **速度层**：对齐当时的 `/cmd_vel_nav_raw`、`/cmd_vel_nav`、`/cmd_vel_collision_checked`、`/cmd_vel_api`、`/cmd_vel_docking`、最终 `/cmd_vel`。不是每个任务都经过相同输入支路。
4. **仲裁层**：查看 `/safety/status`、`/safety/motion_interlock_state`、`/safety/dock_interlock_state`、有效许可及被选中的命令来源。不能拿一个主错误码代替全部子状态。对应 S 组。
5. **硬件与定位层**：比较 `/ranger_base/status` 的实际模式／切换状态与轮速、IMU；定位则比较原始消息 stamp、TF 历史、bridge 当前／目标序号及精确地图身份。对应 S18、S19、L 组。

本次没有在机器人上新增探针、订阅、录包或高频轮询。上述是取证所需信息范围，不是要求全部同时新增采集负载。

## 15. 文档验证与尚未完成的现场验证

本次交付是文档，不涉及运动逻辑修改。已检查 82 个正式编号唯一且分组数量一致；326 个文档链接的目标文件、行号范围或内部锚点检查通过。另检查了 Markdown 基本结构和修改的空白格式。不将这些文档检查称为 ROS 或硬件验收，也不把“行号存在”夸大为第三方实现已逐行验证。

以下事项仍须在后续获准的现场核验中完成：

- 对齐本地源码、Jetson 安装产物和运行进程版本，读取实际启动入口、环境覆盖及相关参数，才能给出“现场启用数”。
- 用同一任务／事务的记录确认各层的阻断顺序，特别是 S07／S16／A10、A16／F03／L09、S14／S18 的关系。
- 对新鲜度恢复、暂停释放、已超时服务的迟到结果、真实刹停、标准离桩等分别验证。涉及运动的测试需场地和人员确认；本次未执行。
- N06–N08 和 Q01 涉及外部实现，配置契约不能替代与实际安装版本一致的源码／隔离测试证据。
- 工作区有未提交及候选实现；该文档不授予自动部署、重启、清锁或绕过安全链的权限。

后续新增／删除／合并门控时，应同步更新其编号、作用、放行条件、部署状态和来源；只改变阈值不另增一个门控编号。以上分组计数以本文件的编号条目为准，而不是搜索 `gate`、`lock` 或 `require_` 的命中数。
