# 乘梯呼梯后移动段的碰撞旁路

## 范围与修复

前往呼梯位仍经过正常避障。呼梯后的接驳、进轿厢、中心与面板之间移动、
切图后返回中心以及出梯到目标层接驳位，使用已有的乘梯专用免碰撞 Nav2 路径。
任务完成后，普通导航继续经过 collision_monitor。

本次根因是模式不一致：FSM 在呼梯后到进梯接驳位的阶段使用 `ELEVATOR_WAIT`，
API 已刷新该段的旁路许可，但安全层只接受 `DOORWAY`。此次仅将安全层允许的
模式补为 `ELEVATOR_WAIT` 或 `DOORWAY`，没有增加许可、锁或状态机阶段。

现有三个移动配置 `ElevatorScoped`、`ElevatorReverseEntryStagingScoped`、
`ElevatorCabinEntryDirect` 均已使用 `unchecked_direct_path: true`，对应控制器
已关闭 command clearance、route revision 和 localization replan。
因此保留 schema-v2 的 `ElevatorScoped/ElevatorFollowPath` 及全部已有调参，
不为此换成另一套控制器，也不修改 schema-v3 的 yaw→横移→前后移动顺序。

## 命令选择与保留的保护

旁路有效时，最终安全仲裁器从 `/cmd_vel_nav` 接收 **velocity_smoother 后**的
速度，忽略 `/cmd_vel_collision_checked`，包括碰撞监控持续输出的零速度。
collision_monitor 本身、scan、代价地图发布和普通导航配置均不关闭。

必须同时满足：许可未过期、transaction id 精确匹配、模式所有者是
`robot_elevator_manager`、模式契约有效、没有 motion hold、模式属于上述两种。
`NORMAL`、`ELEVATOR_RIDE`、无许可、错误事务或错误所有者均不能开启旁路。

既有的急停、取消/hold 停车、指令过期停车、定位保护、在桩保护、速度上限、
倒车及横移许可继续有效。导航作用域退出会清空旁路许可；许可丢失时先清除
缓存命令并停车，后续普通导航重新使用经过 collision_monitor 的速度源。

这意味着这些专用段不会因障碍物而自动停车。仅可在清空人员和障碍、可随时
急停的受控乘梯测试中验证；软件回归通过不等同于公共环境的碰撞安全验收。

## 回归与交付边界

- `test/test_elevator_entry_collision_bypass_policy.cpp`：先复现同一有效许可
  在 `ELEVATOR_WAIT` 被拒绝，再验证模式补齐；覆盖空许可、过期、错事务、
  错所有者、hold、无效契约及普通/乘梯静止模式的拒绝。
- API runtime policy 回归：呼梯前不旁路，所有呼梯后 intent 均旁路；
  schema-v2 保留已标定的坐标、yaw 和原有 scoped profile。
- `test/isolated_elevator_collision_bypass_smoke.py`：真节点路由测试在无网络的
  隔离容器内执行，不连接生产 `/cmd_vel` 或 CAN。旧二进制在 `ELEVATOR_WAIT`
  下输出全零，候选修复版在该模式及 `DOORWAY` 下均选择平滑后的导航速度。
  18 个场景通过，涵盖持续碰撞监控零速度、许可清除/过期/错误事务、普通模式、
  hold、急停、定位失效、导航指令过期及各自恢复；最后验证普通模式确实重新
  选择碰撞监控的速度。测试不测真实碰撞和停车距离，也不覆盖 BMS 接触。
- 本次纯策略测试 5/5、API runtime policy 测试 53/53、乘梯配置契约 11/11
  通过。配置契约测试仅修正了模块搬迁后的源码路径，没有修改生产配置。

真节点脚本拒绝在非 Docker、非 network-none、非 domain 184 或非 localhost-only
环境运行。使用现有 ROS 2 Humble 镜像、新建 `--network none --runtime runc`
容器、私有 IPC、无设备挂载，将工作区与候选测试目录只读挂载。加载现有 ROS
及工作区 install 环境后，设置 `ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1`，执行：

```bash
timeout 50s python3 /test/isolated_elevator_collision_bypass_smoke.py \
  --binary /test/build/robot_safety_node
```

不要在生产容器里直接运行此脚本；它只适用于上述独立容器，且候选二进制必须
来自独立构建目录，不覆盖生产 install。

用户后续授权部署与重启后，已按最小二进制补丁发布，见仓库
`reports/elevator_post_call_collision_bypass/README.md`。硬件仍需验证呼梯位→接驳位的连续自旋、
轿厢内各段及出梯方向、取消/急停停车，以及退出任务后普通导航恢复碰撞检查。
