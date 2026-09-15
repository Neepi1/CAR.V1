# 待修复：本地里程计间歇停更

- 状态：OPEN / 用户要求记录，暂缓修复（2026-09-11）。
- 范围：robot_local_state EKF 的运行、输出投递、DDS/TF 链路；具体根因尚未确认。
- 现象：EKF 进程仍在，/local_state/odometry 计数与时间戳停滞，随后 DDS endpoint_missing，
  odom->base_link 过期，AMCL/Nav2 失败；健康守护之后才自动重启整链。
- 已有早于 GPU 切换的同类记录，不能认定是今天 GPU 引入；也未排除其对触发概率的影响。
- 原始 IMU、点云、/scan 在故障窗口仍更新，不能将整条雷达链误判为停止。
- 2026-09-11 两段 55 秒故障取栈观察均未复现，未附加 GDB。没有证据足以认定 Fast DDS 死锁。
- 第二次自动恢复后 API 曾恢复 safe_for_goal_start=true、amcl_ready=true；暂时就绪不等于修复。

## 下次修复所需证据与验收

在断流已发生、守护退出前采集唯一 EKF 的线程栈，结合输入新鲜度、发布计数、DDS endpoint、
CPU 调度和 UDP 接收状态，区分具体阻塞位置。不得靠延长超时、删除门控或重定位掩盖故障。
凭确认的根因提出最小修改，再执行针对性回归与整链启动/长时间静止及运动验证。
新的生产修改、人工重启或运动测试须取得相应授权；此文件不是执行授权。

证据目录：/tmp/njrh_reports/jt128_gpu_deploy_20260911_dnNe7o/

- odom_diagnosis.md：诊断结论与边界。
- restart_journal.txt：先断流、后守护重启的时间线。
- navigation_second_run.log、local_state_second_run.log、rates_final.json：源端与消费者证据。
- diagnosis_current.json：第二次自动恢复后的暂时健康快照。

Windows 证据副本：C:/tmp/njrh_reports/jt128_gpu_deploy_20260911_dnNe7o/。
