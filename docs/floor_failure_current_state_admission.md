# 切图失败后的当前状态准入

本轮修正此前“只放开离线编辑”的不完整处理：失败应保留为事务结果，不能在 API
进程内永久挡住后续恢复操作。本页仅记录 API 门控部分；不代表已部署或整车验收完成。

## 约定

- 每次楼层状态或定位健康消息替换对应来源的旧证据，不再永久缓存第一次
  `FAILED_LOCKED`。失败详情仍由事务历史保存；不从报错文本推导运行所有权。
- 任一来源报告当前切图活动，冲突操作仍返回 `FLOOR_TRANSITION_ACTIVE`。
  `FAILURE_CLEANUP` 本身是活动；终态 `FAILED` 携带上一个阶段名称时不是活动。
  兼容 floor-manager 实际的 `RUNNING/LOAD_NAV_MAP` 格式；另一个请求的拒绝或
  预检查记录不能清除当前活动事务，只有它自己的终态才结束对应活动证据。
- 当前 `runtime_context_valid=false` 仍作为真实诊断，导航目标、实时采点、对桩等
  仍被拒绝。健康字段不因一次恢复操作被修改，也不会把尚未定位说成 ready。
- 建图、重新定位、启动导航服务和明确重新选图不需要旧图已定位。这些入口只越过
  “源图无效”条件，继续接受各自的资产、运行冲突、进程、未决副作用和目标验收。
  放行启动服务不等于允许机器人运动。
- 停止和取消原本不通过这个门控，本轮不增加约束。未完成 RPC、活动切图和具体
  资源冲突不能因旧失败记录变为终态就被清除。

## 精确操作范围

源图无关的 17 个内部操作名（含异步工作和提交阶段）为：

| 流程 | 内部 operation |
|---|---|
| 建图 | `mapping_start`, `mapping_start_worker`, `mapping_start_context_clear`, `mapping_process_launch`, `mapping_save`, `mapping_save_commit` |
| 导航服务启动 | `navigation_start`, `navigation_runtime_resume`, `navigation_runtime_resume_commit`, `navigation_runtime_launch` |
| 重新定位 | `manual_localization`, `localization_trigger` |
| 楼层选择/切图 | `live_floor_switch_start`, `floor_switch`, `floor_switch_noop_commit`, `floor_switch_submit`, `floor_selection_commit` |

原来审核过的 12 个离线配置/显式坐标操作继续允许在无活动切图时编辑：电梯配置
save_draft/publish/rollback 和 pose_save/pose_delete/pose_batch_replace，各含 `_commit`。
不使用前缀匹配或通配；`navigation_goal_worker`、`navigation_goal_submit`、
`navigation_pre_send`、`current_pose_save`、实时禁行区编辑和地图删除不在豁免组内。

## 测试与限制

真实 C++ 门控单测先复现 9 个新失败用例，修正后 25/25 通过，使用独立 `/tmp`
目录和 `g++ -std=c++17 -Wall -Wextra -Werror`，没有启动生产 ROS。
覆盖终态历史、健康恢复、再次定位失效、所有源图无关阶段、两个观测来源的活动优先级。

`reports/floor_failure_asset_admission/isolated_api_smoke.py` 在 localhost、
独立 ROS domain 229、端口 18519、临时 maps_root 下运行真实 API 二进制。生产已安装
旧二进制已复现：失败终态后即使当前 health valid，空导航请求仍被旧失败返回503，
没有到达应有的参数400。候选完整 API 隔离绿测试已完成，包含六个恢复入口、当前invalid
拒绝导航、活动状态拒绝冲突、草稿200和发布自身校验422。指纹和完整结果见
[真实 HTTP 回归](../reports/floor_failure_terminal/api_http_smoke.md)。未部署或重启生产服务。

这不是删除所有安全门控。API 单元测试不能证明物理停止、RPC 已结束或目标地图
已正确加载；这些仍需要 floor-manager/bridge/电梯资源清理和独立安全仲裁共同证明。

## 电梯清理的定位依赖

非持久锁模式下，安全清理不能要求失败的地图先恢复可导航，否则 `hold_and_cancel`
和 `finalize_recovery` 会反复等待 `safe_for_goal_start`，自身失败事务无法结束。
`check_elevator_cleanup_runtime_readiness` 现在只在该模式、已证明位于源层/目标层外部
区域时跳过地图 ready 探针；持久锁模式保留原严格身份和新鲜度证明。

这个成功结果不附带静止、hold 或资源已释放标志。调用者仍需证明任务终止、双路里程计
静止、自己的 mode/pause 已释放、没有未决副作用、无外部电梯 hold，再使用带代际的条件
释放服务。`kRetainLock` 不会被解释成外部区域，也没有改变导航开始的定位要求。
采用真实生产入口调用的 callback policy 做 C++ 红绿测试；Python 检查仅验证接线与
安全条件未被删除，不把字符串检查当作运行清理成功的证据。
