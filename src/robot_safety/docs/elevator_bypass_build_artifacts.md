# 乘梯旁路的构建产物回归

## 2026-09-10 根因与最小修复

呼梯后接驳阶段使用 `ELEVATOR_WAIT`。当前策略源码已允许该模式和
`DOORWAY`，但离桩修改的增量构建仅重编译主节点，链接了旧的
`libelevator_entry_collision_bypass_policy.a`，使已修复行为退回只允许
`DOORWAY`。源码哈希一致不能证明最终程序具有同一行为。

本次不改业务规则、参数、FSM 或机械臂。先用原链接输入逐字节复现运行版，
再只重编译该策略库并重新链接；主节点对象和另外两个支持库保持原样，
保留近期离桩修改。候选程序先放在 `/tmp/njrh_reports`，未获整链重启授权前
不替换运行 executable。编译完成、同步文件与实机已生效必须分别说明。

## 防止再次混入旧库

常规构建应通过 CMake/colcon 的目标依赖重建，不再仅编译主节点后盲目复用
历史 `link.txt`。特殊最小补丁须记录所有链接输入哈希并验证最终 executable。
策略库 build/install 副本必须配套修复，不能只发布单个新 executable。

现有纯策略测试必须包含 `AuthorizesPostCallStagingInElevatorWait`；旧测试
只有 DOORWAY 用例时，即使全绿也无法发现本次回归。使用下面的检查直接链接
指定的实际产物，而不是重新编译源码来替它验证：

```bash
bash src/robot_safety/test/verify_elevator_bypass_archive.sh \
  /workspaces/njrh-v3/workspace1/install/robot_safety/lib/libelevator_entry_collision_bypass_policy.a
```

此检查只编译/运行无 ROS 的策略测试，结果写入独立 `/tmp/njrh_reports` 目录，
不会启动节点、发布速度或改写传入的库。旧产物必须失败，新产物必须通过。
还需运行 `isolated_elevator_collision_bypass_smoke.py` 验证最终候选节点的
速度源选择，并运行离桩实节点隔离回归。生产服务重启后另行核对
`/proc/<pid>/exe` 哈希和进程唯一性；硬件乘梯成功率仍需用户运动测试。

本次详细证据与部署状态：`/tmp/njrh_reports/elevator_bypass_relink_20260910/summary.md`。
