# MPPI 原始失败与三层停车事件

2026-09-20 增量：已完成MPPI/外层控制器候选链接、真实插件生命周期、真实无解/恢复
日志到轻量记录器CSV的闭环；Safety/Collision/Ranger独立隔离验证已补。
旧的“仅语法检查”是09-18历史状态，最新边界见 [闭环记录](navlite_chain_evidence.md)。
上述候选已于2026-09-20授权部署并整链重启，五个模块生产日志已核验；未进行实车验收。

2026-09-18：诊断候选，未部署。承接 navlite_stop_branch_audit.md；停止沿
修路失败导致 hold 残留的方向修改，不调整任何运动/安全/重试参数。

## 已确认的原始条件

运行 RangerMPPI 插件 SHA256
993b25c1b9275a8b312f7fa84e5bf7c3b0782353a2379f3911abb719014c2bf3，
对应 mppi_output_constraints_20260908 候选，不以本地 main 推定部署状态。

`ObstaclesCritic::score` 的 all_trajectories_collide 赋给 fail_flag；
`Optimizer::fallback` 失败计数超过原有上限后抛出完整原文
`Optimizer fail to compute path`。当前启用评分器中仅 ObstaclesCritic
置位此标志，CostCritic 未启用。判定还包括配置不接受的未知格/越界，
不能直接等价成碰到真实物体，也不能把所有异常归为碰撞。

外层仅在 enabled、MPPI primary、非空 reference path 且 runtime_error
全文匹配时返回新零命令；其他错误原样抛出；正常返回零命令不是失败。

同一 evalControl 内 reset 不清 fail_flag，后续评分看到 true 会跳过。
已安装库隔离验证为4次优化入口、后3次进入时 true；下一外部调用 prepare
会清标志并重算。本轮只记录此行为，不更改库或计数，也不认定它已解释历史
人工标记窗口。原始来源见官方 Navigation2 1.1.19 optimizer.cpp、
critic_manager.cpp、critics/obstacles_critic.cpp。

## 事件约定

- `NAVLITE mppi`：原生计算阶段、原始异常、fail_flag、计算序号、失败次数、
  steady 时长；计算恢复零/非零分开，零后首次非零再次记录。
- `NAVLITE controller`：精确异常转零与外层真实返回；不冒充纯MPPI。
- `NAVLITE collision`：输入、请求输出、published、最终动作/区域/原因、
  Scan 输入状态与上个事件。published=0 不代表发送零速。
- `NAVLITE safety` / `safety_arbitration`：输入、选中来源、最终输出和原因；
  被忽略的输入记 no_message；状态解除和非零发布分开。

变化立即写 WARN，持续相同异常每事件流最多1Hz；诊断字段不参与控制。
复用现有数据，无新增 ROS 节点/话题/订阅、线程或锁。
计数是计算调用而非目标UUID。未知输入/内部原因明确unknown，不造数据。

Collision 补丁仅存放在 scripts/diagnostics/patches，尚未应用；改变的是
CollisionMonitor/Scan 私有诊断字段，未来必须连同入口及头文件消费者一致
重建，禁止只换库。当前只有 Scan 源的 getData 分支被观测，其他源unknown。

## 证据与验证范围

完整报告：Jetson及本机 `/tmp/njrh_reports/navlite_failure_origin_20260918`
（Windows 本机为 C:/tmp/njrh_reports/...），incident_report.md。

- 隔离真实 MPPI/插件10/10；Runtime8/8；计数及限频测试通过。
- 轻量脚本15/15；真实 C++→日志文件→脚本捕获12条事件。
- Safety、Collision/Scan 安装接口语法检查通过；它们的完整输入输出集成
  和生产实际日志落盘仍未验证，不称实车验收。
- 10:37:36.659121152 UTC 旧事件只有“该周期进入零速分支”证据；连续次数、
  起止时间、恢复命令、下游放行未知。不能覆盖10:36:43–47及10:37:27–31标记。
- 当前进程 stdout 路由已核对：resident_navigation_runtime.log 和
  robot_safety_common.log，均为轻量脚本默认输入。新生产事件须授权部署后
  核验三层均实际落盘，然后才做下一轮用户控制的采集。
- 无当前速度话题/轮速采集，不能用非零日志证明车已动；没有回调不等于停发
  原因已知，不用接收顺序断言毫秒级内部因果。

失败计数小测试：`c++ -std=c++17 -Isrc/robot_nav_config/include
scripts/diagnostics/test_navlite_failure_trace.cpp -o /tmp/navlite_trace_test`，
然后执行 `/tmp/navlite_trace_test`；其余 ROS 测试必须私有网络/IPC/SHM隔离，
不要在实车 ROS 域运行。
