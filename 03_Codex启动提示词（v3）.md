# Codex 启动提示词（v3）

请先完整阅读仓库根目录下的以下文件，然后再开始编码：
1. `00_新增关键要求与TF修复优先级.md`
2. `AGENTS.md`
3. `01_商业化评估与可执行总规范（v3）.md`
4. `02_实现任务清单.yaml`

## 绝对要求
- 必须先输出一份分阶段实施计划，再开始改代码。
- 必须按 `02_实现任务清单.yaml` 的 phase 顺序推进。
- 第一优先级不是“继续堆功能”，而是：**复用本地 car 项目 + TF 审计 + canonical TF tree**。
- 允许使用 FAST-LIO2，并且要把它作为离线建图前端。
- 必须把 PGO / 回环作为离线建图正式后端纳入，不允许省略。
- 如果本地 car 项目中已经有可用参数、launch、依赖或镜像配置，必须先复用；缺失时才允许联网下载。
- 不允许把 MPPI / SmacPlanner2D / velocity_smoother / collision_monitor / local_perception 这些默认值省略。

## 先做的 4 件事
1. 扫描本地 car 项目资产并出复用报告
2. 做 TF 审计并出报告
3. 把 JT128 / FAST-LIO2 / PGO / Isaac localizer 包成 wrapper
4. 落地唯一 `map->odom` 与唯一 `odom->base_link`

## 默认实现选择（必须照做）
- DDS: Fast DDS (`rmw_fastrtps_cpp`)
- Mapping frontend: FAST-LIO2
- Mapping backend: PGO / loop closure
- Default planner: SmacPlanner2D
- Optional planner profile: SmacHybrid
- Default controller: MPPI
- Fallback controller: RPP
- Path smoother: Simple Smoother
- Progress checker: PoseProgressChecker
- Goal checker: SimpleGoalChecker
- Local costmap obstacle source: `/perception/obstacle_points`
- Velocity chain:
  `controller_server -> /cmd_vel_nav_raw -> velocity_smoother -> /cmd_vel_nav -> collision_monitor -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> robot_chassis_bridge`

## 强制的 TF 架构
```text
map
 └── odom                      (only robot_localization_bridge)
      └── base_link            (only robot_local_state)
           ├── lidar_link      (static)
           ├── imu_link        (static)
           ├── base_footprint  (optional static)
           └── other static frames
```

## 禁止事项
- 禁止多个节点发布 `map->odom`
- 禁止多个节点发布 `odom->base_link`
- 禁止重复静态外参发布
- 禁止把第三方内部算法 frame 直接变成导航主树的一部分
- 禁止直接把原始 JT128 点云喂给 local costmap

## 你必须实现的关键包
- `robot_chassis_bridge`
- `robot_hesai_jt128`
- `robot_fastlio_mapping`
- `robot_pgo_mapping`
- `robot_map_toolkit`
- `robot_local_state`
- `robot_local_perception`
- `robot_global_localization`
- `robot_localization_bridge`
- `robot_floor_manager`
- `robot_elevator_manager`
- `robot_mode_manager`
- `robot_mission_manager`
- `robot_safety`
- `robot_nav_config`
- `robot_bringup`
- `robot_system_tests`

## 对 wrapper 的特别要求
### robot_hesai_jt128
- 优先复用本地 car 项目配置
- 网络参数、IP、端口、frame_id、外参必须参数化
- 支持 bag/mock 模式

### robot_fastlio_mapping
- 优先复用本地 car 项目参数和 launch
- 必须统一导出 frontend_result
- 必须处理 TF 输出，不让非 canonical frame 污染导航主树

### robot_pgo_mapping
- 优先复用本地 car 项目版本
- 必须导出标准 `mapping_result`
- 必须输出 `loop_report.json`

### robot_global_localization
- 统一封装 pointcloud_to_flatscan + Isaac localizer
- 必须实现 floor asset apply / reload / restart / trigger / health
- 不得直接与 Nav2 map 资产混淆

## 首轮交付
先完成：
1. 工作区骨架
2. 本地 car 项目复用扫描器
3. TF 审计脚本
4. 第三方依赖解析脚本（本地优先，网络兜底）
5. `robot_interfaces`
6. `robot_description`
7. `robot_chassis_bridge`
8. `robot_hesai_jt128`
9. `robot_fastlio_mapping`
10. `robot_pgo_mapping`
11. `robot_map_toolkit` import/export 骨架
12. `robot_local_state`
13. `robot_global_localization`
14. `robot_localization_bridge`
15. `robot_nav_config`
16. `robot_bringup`
17. 基础测试与报告

## 重要
- 每个包必须带 README、参数文档、测试。
- 所有硬件相关参数必须放到 yaml。
- 所有状态机必须支持 mock。
- 所有 launch 必须支持 bag replay 或 mock。
- 每完成一个 phase，都要更新 docs。
- 必须生成：
  - `reports/car_project_reuse_report.md`
  - `reports/tf_audit_report.md`
  - `reports/third_party_resolution_report.md`
