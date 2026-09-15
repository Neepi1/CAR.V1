# 切图失败不再封锁离线点位和电梯配置

## 问题与范围

现场切图失败后，`FloorRuntimeInterlock` 保留 `FAILED_LOCKED`。同一个
判断既用于运动，又用于资产编辑，导致 `PUT /api/v1/elevator-config/draft`
返回 HTTP 503，无法保存或发布 F12 的电梯点位。此前完整运行时重启仅清除
残留，并未修正这个耦合。

本次修改的是 **API 操作准入范围**，不是把未知运行时标记为健康，也不是
删除失败记录或解除安全停车。代码修改集中于 `features/floor_switch/`。

## 策略

| 操作 | 无活动切图、但保留失败状态时 |
|---|---|
| 电梯配置保存草稿、发布、回滚 | 允许进入原有资产处理和校验 |
| 已给定 XY/yaw 的普通点位保存、删除、批量替换 | 允许进入原有处理和校验 |
| 从当前车辆位置采点 | 仍阻止；需要可信的当前地图位姿 |
| 导航、对桩、离桩、恢复运动 | 仍阻止 |
| 实时禁行区修改、地图删除、建图保存 | 不纳入离线豁免；可能影响运行资产 |
| 切图、定位恢复 | 保持原有独立策略，不凭普通健康心跳清锁 |

放行名单使用 12 个精确内部操作名，包含入口和提交前检查，不使用 `save`
或 `commit` 通配。调用统一走 `decision_for_operation(operation)`；默认
`decision()` 仍报告实际运行失败，不会因一次资产操作被清空。

活动切图继续阻止这些修改。额外记录两条观测通道最新的活动标志，防止
历史失败的高优先级遮住后来出现的活动事务。状态更新仍在原互斥量保护下；
不新增线程、订阅、定时器或运行时锁。

所有原有资产身份、版本冲突、发布校验和活动电梯任务准入仍在原位置执行。
地图加载顺序、TF、DDS、定位、Nav2、机械臂和底盘保护均未修改。

## 验证

1. 复现现场响应：草稿请求返回 `FLOOR_TRANSITION_BLOCKED` /
   `FLOOR_TRANSITION_FAILED_LOCKED`。
2. 将同样的 failure_code=99 / filter-mask 错误送入真实 C++ 策略：
   新增两条资产测试先失败（原策略 13/15 通过），改后通过。
3. 复核新增“历史失败 → 新活动状态”四种同/跨通道序列：先复现漏挡，
   修正后全部 **16/16 C++ 测试通过**。活动结束只恢复资产编辑，仍不授权运动。
4. 补充既有模块接线检查，防止 HTTP 或运行调用丢掉 operation 参数。
   `floor_switch_module_contract.py` 及既有 workspace floor interlock contract
   均通过；修改的两个 C++ 模块使用车端原编译参数完成 `-fsyntax-only`
   检查。该检查不是链接或真实 HTTP 集成验收。

测试在 Jetson 独立临时目录 `/tmp/njrh-floor-lock-test.l3pNj6` 执行，
不启动 ROS、不发布速度、不改生产安装目录。

## 部署状态与剩余验收

**用户授权后，已部署并完整重启运行服务。** 默认 build 与 installed 不同，
因此没有使用默认 build 或全包重编。随后找到精确基线：
`/tmp/njrh_reports/map_switch_source_independent_20260909/api_build`。
原对象重新链接与 installed **逐字节一致**；仅重新编译 policy 和持有该对象的
floor-switch module，再替换这两个对象链接新程序，动态依赖检查全部通过。

- 旧 API SHA256：`98082b15919bf9ddbcb585de3aa20f037000b5756ea7a245ce36681e3f6ac88c`
- 新 API SHA256：`bbcb46b4d56383e43d0978df845d39e6e44c9b2824f3a0bf2b72707cb74201b2`
- 部署备份：`/workspaces/njrh-v3/workspace1/reports/floor_failure_asset_deploy.4L2Dju`
- 受控重启后 systemd MainPID=1416459，NRestarts=0；API PID=1419986，
  `/proc/1419986/exe` SHA256 与候选一致。
- 初次读回：API healthy、navigation_active=true、floor interlock blocked=false；
  AMCL 仍在初始化。没有发布运动目标，也没有故意触发生产切图失败。
- 最终读回：运行服务稳定、NRestarts=0；运行地图仍为 B15/F1/test-001，
  context ready/confirmed，Nav2 层就绪、导航目标 idle。AMCL 仍报告等待初始
  位姿种子，因此不能声称连续定位全部就绪；车辆保持停止。

真实 HTTP 隔离回归已完成：`reports/floor_failure_asset_admission/isolated_api_smoke.py`
使用 domain 229、localhost、独立端口 18519、临时地图目录。旧 binary 复现
草稿 503；新 binary 得到 DRAFT_SAVED，发布仍因草稿本身不完整返回正确的
422/DRAFT_REVIEW_REQUIRED；导航仍503，停止/取消仍202，历史失败后的新活动
事务仍阻止编辑。所有测试进程退出，不改生产图或点位。

此修复不代表此前 filter-mask 服务未激活的根因已修复，或未决地图操作可以
被直接清除。完整实车切图/重定位/乘梯验收仍未执行。
