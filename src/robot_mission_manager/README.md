# robot_mission_manager

## Frozen target asset identity

Every `MissionRequest` must carry the target map's nonzero
`expected_asset_epoch` and canonical lowercase
`sha256:<64 lowercase hex characters>` digest. `MissionFsm::start()` copies
that identity into its mission state and every emitted elevator/navigation
effect. A successful elevator completion is accepted only when its final
`building/floor/map/asset_epoch/asset_digest` exactly matches the frozen target
tuple; format-only validity is not sufficient.

This is an intentionally fail-closed source contract. Existing C++ callers
that omit the new request fields still compile because the fields have safe
defaults, but `start()` rejects their `0`/empty identity. The generated
`MissionTask` and `ElevatorTask` ROS action types are wire-incompatible with
their previous schemas, so future producers and consumers must be rebuilt and
deployed together. No live mission/elevator action adapter exists yet.

`robot_mission_manager` 当前只提供纯 C++、事件驱动的任务编排核心
`MissionFsm`。本阶段没有 ROS 节点，不发布 Twist 或 TF，也不会直接调用
底盘、Nav2、楼层切换或电梯设备。

## 当前契约

- 请求字段与 `robot_interfaces/action/MissionTask.action` 对齐：任务、楼宇、
  源楼层/地图、目标楼层/地图、目标 `pose_id` 和可选首选电梯。
- 所有会进入资产或路由查找的标识符都必须是有界、路径安全的字符串。
- 构造 FSM 时必须注入每次进程启动唯一的 `instance_id`；effect transaction
  同时包含该 fencing ID 与 mission ID，进程重启后不得复用旧回执。
  `instance_id` 最长 32 字符、mission ID 最长 64 字符，使生成的 transaction
  仍满足电梯核心的 128 字符上限。
- 同层且同地图任务只产生一个 `NAV_TARGET` effect；同层但不同 map 的请求会在
  产生 effect 前拒绝，必须走独立的 floor-switch 工作流，不能误路由到电梯。
- 只有目标楼层不同的任务才先产生 `ELEVATOR_TASK`；该 effect 使用精确
  `transaction_id` 报告成功后，才产生最终 `NAV_TARGET`。
- 任意时刻最多存在一个 active effect。旧 transaction 的延迟事件只会被忽略。
- 外部 abort/cancel 或 active effect 失败时，先产生 `HOLD_AND_CANCEL`，
  同时进入永久 `FAILURE_LOCKED`；迟到的成功事件不能解除锁定。
- `HOLD_AND_CANCEL.cancel_transaction_id` 指明适配层应取消的旧 effect。
  清理失败会保留同一稳定 `cleanup_id`、产生新的重试 transaction 并保持
  active；成功回执只结束当前清理 effect，不解除失败锁定。适配层必须用
  `cleanup_id` 作为幂等 hold 身份，不能因重试 transaction 改变而累积新 hold。
- 电梯成功回执必须同时证明精确目标 floor/map、非零 asset epoch、合法 digest、
  非零显式重定位序列、`TARGET_HALL`、有效 runtime context，以及没有活动 Nav2
  goal、safety hold、mode/execution lease 或 correction pause。

## 后续 ROS 适配层边界

适配层负责把 effect 串行映射到 Nav2、`robot_elevator_manager` 和
`robot_safety` 的 owner-scoped 接口，并把精确 transaction 回执送回核心。
适配层不得同时派发多个 effect，也不得在上一 action 未明确结束时发送下一目标。

机械臂与视觉后续通过 `robot_elevator_manager` 的端口接入，任务核心不直接感知
按钮、门状态或相机。失败恢复必须由独立、显式的 recovery 流程完成，不能通过
重建 FSM 自动解锁。

## 测试

```bash
colcon build --packages-select robot_mission_manager --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select robot_mission_manager --event-handlers console_direct+
```

纯核心测试覆盖同层/跨层编排、同层跨 map 拒绝、终态证明、旧回执 fencing、
标识符长度与路径安全，以及稳定 cleanup ID 重试。跨包非运动场景还组合了
elevator/floor/safety/mode/pause 核心。

这些测试不授权移动机器人。当前仍没有 Mission ROS action server、Nav2/elevator
adapter 或任务恢复节点。真实硬件还需验证 action transaction 绑定、
hold/cancel 落地顺序、跨层到达后的定位与楼层就绪屏障，以及全程无并发速度源。
