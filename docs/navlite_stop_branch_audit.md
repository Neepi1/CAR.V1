# NAVLITE 停车分支审计

2026-09-18。仅诊断候选，未部署、未重启、未移动车辆。不改变速度、轮廓、
footprint、修路、重试、任务、BMS 或安全判据。

## 以运行候选为准

控制器 PID 517369 加载的 libgoal_scoped_rotation_shim_controller.so：
SHA256 93d517862fa066c73faf0cd422ebe57935ffc7cd7fc9e453ea5933f1113b7286。
与 Jetson 报告 mppi_dynamics_20260908/package_stage/lib 中的候选一致。
对应源码保存在该报告 source/src/robot_nav_config。
本地 shim 另有未部署的 recovery inspect/preserve-startup 逻辑。
本轮隔离编译只把诊断 diff 应用到运行候选源码，patch fuzz=0，不混入该差异。

碰撞监控为未修改的系统包 ros-humble-nav2-collision-monitor
1.1.19-1jammy.20251018.015923，dpkg -V 无输出。
核心库 SHA256：
1fb3f7999205589bb21350aa4737709b477de212dce047649dbb89d9b64c97b5。
只提供未应用的版本限定补丁：
scripts/diagnostics/patches/nav2_collision_monitor-1.1.19-navlite.patch。
源码取自官方同版本 tag，头文件取自实机。此为版本/接口核验，不是完整可复现
构建证明。因类布局增加诊断字段，未来需一并构建库和头文件消费者（包括入口
可执行文件），不能只替换库。本轮没有替换任何系统文件。

## hold 与修路的真实关系

- hold_position 默认 false，运行源码唯一显式赋值也为 false。
  no_path、无回接位、worker 忙、Mode::kWaitClear 均不直接要求停车。
- compute_command 每周期调用原控制器；只有精确 MPPI 无解异常转换成零速。
  control_waiting_ 只是本次结果，不会跳过下一周期计算。成功返回后清除此状态，
  但成功返回的命令自身仍可能为零，不能当作已恢复运动。
- 路径检查从最近点向前，而非检查全部历史前缀。最近点搜索遍历整个路径，
  自交/平行折返处没有严格单调进度保证；本次记录不足以证明发生了这种歧义。
- 异步结果 generation 不匹配时丢弃；超龄成功结果拒绝；应用前用最新位姿和
  costmap 重验原路径及前向 footprint 接驳。原路径已清则丢弃绕行修补，
  接驳不安全则回到后台修路。这些均不建立 hold。

## 停车、维持、解除与输出

| 首次分支 | 维持条件 | 解除条件 | 随后输出 |
|---|---|---|---|
| MPPI 精确无解异常 | 新一周期仍无解 | 后续 compute 正常返回 | 返回该命令，仍可能为零 |
| 启动朝向不可测 | 原测量/guard 未满足 | 原条件满足 | 既有启动旋转或控制器命令 |
| 末端 footprint 拒绝 | 投影检查仍不清晰 | 后续原检查通过 | 既有末端命令，可能仍处于停稳阶段 |
| Collision STOP | 新一轮区域点数仍命中 | 输入回调重新计算不再 STOP | 当前 slowdown/approach/pass 结果 |
| Collision 停发零速 | 最终请求持续为零并超过停发时间 | 后续最终请求非零 | 恢复发布；不是永久锁存 |
| Safety 零速优先窗口 | 合法零指令刷新原截止时间 | 时间到且新指令获准 | 原 prepare 后指令 |
| Safety watchdog | 输入未及时刷新 | 新获准指令且其他条件通过 | 原 prepare 后指令 |
| Safety 模式/停稳/BMS 等 | 原对应拦截条件仍成立 | 原解除条件，未修改 | 最终发布；状态解除不代表车已动 |

碰撞实现依据：
[官方 Humble 1.1.19](https://github.com/ros-navigation/navigation2/blob/1.1.19/nav2_collision_monitor/src/collision_monitor_node.cpp)。
其动作/放行日志为 INFO，而当前进程以 WARN 启动；原日志不能观察这些动作。
进程加载参数文件中的 stop_pub_timeout=0.3，持续零速后可能不再发布消息。
这是加载文件快照，未额外查询动态参数，不声称不存在运行中参数更新。

## 新增事件（均为 WARN，兼容当前阈值）

- NAVLITE controller：实际返回分支、精确零/非零、有符号命令、位姿/frame/
  source stamp。rotation_shim_return 是外层输出，不冒充纯 MPPI。
- NAVLITE repair：结果/当前 generation、快照年龄/索引、status、
  hold_position、command_decision=false。快照索引不是实时全局进度；
  invalid/clear 结果中的零索引是默认值，不是命中障碍证据。
- NAVLITE collision：最终 action/region、输入/请求输出、published、停发原因。
  published=0 时 requested_out 不是实际发送的消息。
- NAVLITE safety：最终发布命令、实际拦截/prepare 分支、原状态、可用输入、
  零速截止时间、原模式/停稳观测。source 0=普通、1=API、2=对桩、-1=未知。
  input_known=0 与 NaN 表示诊断输入不可得，不是发布了 NaN 速度。
- NAVLITE safety_state：原状态变化/解除，publication=state_only，不冒充恢复运动。

每个输出流在语义变化时记录；持续相同异常最多每秒一次；正常连续输出不逐帧
写日志。同一 Safety 原因的 timer/callback 重复合并为同一日志限频键。
限频使用 steady_clock，只控制日志。没有新增 ROS 通信、线程、锁、业务
定时器或状态机；限频小类按包保留，避免 Safety 依赖 Nav2。

## 验证与未验证边界

完整记录：Jetson /tmp/njrh_reports/navlite_stop_audit_20260918。

- 控制器、运行时、Safety、Collision 修改单元基于安装接口编译/语法检查，
  没有完整构建 API 或生产候选。
- Runtime 隔离回归 8/8，2.538 秒：远障无 hold、无解后重算恢复、已通过直线
  路段障碍无 hold、正常返回零与无解区分、生命周期清理、未知异常等。
- 两个包内日志限频类通过变化、解除、持续异常 1Hz 测试。
- 轻量脚本 15/15：新增覆盖停车、解除和停发事件原文保存。
- 真实 C++ Runtime 测试日志进入 fixture 的 resident_navigation_runtime.log，
  再被轻量脚本读到：3 条 NAVLITE，保留源时间戳。
- ROS 测试仅在私有 network/IPC/mount、私有 SHM、domain 181 进行。
  首次测试配置误用超范围 domain 242，在测试开始前失败，已保留记录；
  这不是车辆运行失败。
- 实机各进程 stdout 已核对为脚本默认读取的四个文件；新生产日志是否真正
  落盘仍需授权部署后验证，不能拿 fixture 替代生产验证。
- Collision 补丁尚未做完整区域动作集成/生产候选构建；Safety 本轮为编译、
  限频检查，尚未重跑完整输出集成。超龄异步结果强制注入、自交路径等仍属
  静态审计边界，不冒充动态回归通过。
- 没有实车复现，没有里程计/扫描/代价图回放，不能断言早期人工标记的首停
  来源，也不能按日志接收顺序推断毫秒级因果。
- 脚本把 NAVLITE 存为 diagnostic 原文；旧 collision_action_counts 未必统计
  新格式，判断时查 NAVLITE collision 原文，不以旧计数器为准。

后续需单独确认部署；先验证三层真实事件均落到脚本读取文件，再进行用户
控制的下一轮采集。本轮不要求用户现在复测，不触发任何车辆动作。
