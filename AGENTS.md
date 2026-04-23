# AGENTS.md

本仓库是一个 ROS 2 Humble 多楼层室内外配送机器人导航栈，目标平台为 Jetson Orin + JT128 + Ranger Mini 3。

## 阅读顺序
编码前必须按顺序阅读：
1. `00_新增关键要求与TF修复优先级.md`
2. `AGENTS.md`
3. `01_商业化评估与可执行总规范（v3）.md`
4. `02_实现任务清单.yaml`

## 第一原则
当前最大的工程问题不是“少一个算法”，而是 **TF 树治理与现有已跑通组件的收口**。

先做：
1. car 项目本地复用扫描
2. TF 审计
3. JT128 / FAST-LIO2 / PGO / Isaac localizer wrappers
4. canonical TF tree

在这之前，不要盲目联网下载最新版依赖。

## 绝对固定的架构选择
- ROS 2: Humble
- DDS: `rmw_fastrtps_cpp`
- 离线建图前端：FAST-LIO2
- 离线建图后端：PGO / loop closure
- 在线默认连续 odom：wheel odom + LiDAR IMU -> robot_localization EKF
- 在线全局定位：Isaac Occupancy Grid Localizer
- `map->odom`：robot_localization_bridge
- 默认 global planner：SmacPlanner2D
- 默认 local controller：MPPI
- fallback controller：RPP
- 必须启用 velocity smoother
- 必须启用 collision monitor
- local costmap obstacle source：`/perception/obstacle_points`
- floor switch 必须是原子操作
- 最终 cmd_vel 必须由 `robot_safety` 仲裁

## 本地 car 项目复用规则
优先复用本地 car 项目中的：
- JT128 驱动参数
- 网络配置
- 安装外参
- FAST-LIO2 参数与 launch
- PGO 参数与 launch
- Isaac localizer 参数与 launch
- docker/devcontainer/install scripts
- 已验证的 third-party 版本

只有在本地不存在或无法兼容时，才允许联网下载。
必须生成：`reports/car_project_reuse_report.md`。

## TF 规则
### 最终主树
```text
map
 └── odom                      (only robot_localization_bridge)
      └── base_link            (only robot_local_state)
           ├── lidar_link      (static)
           ├── imu_link        (static)
           ├── base_footprint  (optional static)
           └── other static frames
```

### 禁止项
- 不允许多个 `map->odom`
- 不允许多个 `odom->base_link`
- 不允许重复静态外参
- 不允许把 FAST-LIO2 / PGO / Isaac localizer 的内部算法 frame 直接并入导航主树

## 你必须实现的包
- `robot_interfaces`
- `robot_description`
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

## 包边界规则
- `robot_fastlio_mapping`: FAST-LIO2 wrapper only
- `robot_pgo_mapping`: loop closure / backend wrapper only
- `robot_map_toolkit`: offline map production only
- `robot_local_state`: only local EKF odom
- `robot_global_localization`: only pointcloud->flatscan + Isaac localizer + asset reload + trigger + health
- `robot_localization_bridge`: only map->odom generation
- `robot_local_perception`: only local obstacle filtering
- `robot_floor_manager`: only floor asset switch
- `robot_elevator_manager`: only elevator FSM
- `robot_mode_manager`: only mode switching
- `robot_mission_manager`: only high-level mission orchestration
- `robot_safety`: only estop/watchdog/cmd_vel arbitration

## Nav2 固定实现
- planner: SmacPlanner2D
- optional planner profile: SmacHybrid
- controller: MPPI
- fallback controller: RPP
- smoother: Simple Smoother
- velocity_smoother: required
- collision_monitor: required
- local costmap obstacle source: `/perception/obstacle_points`
- global costmap: Static + Inflation + Filters
- local costmap: Voxel + Inflation

## 地图资产规则
每层必须生成：
- `nav/nav_map.yaml`
- `nav/nav_map.pgm`
- `localizer/localizer_map.png`
- `localizer/localizer_params.yaml`
- `filters/*.yaml`
- `filters/*.pgm`
- `reports/asset_report.json`
- `poses.yaml`

Nav2 map 和 localizer png 必须来自同一 occupancy 中间结果。

## 测试规则
必须至少有以下测试：
- TF 唯一性测试
- mapping_result 生成测试
- floor asset 生成测试
- localizer asset reload 测试
- map->odom bridge 测试
- MPPI 默认配置测试
- floor switch 测试
- elevator mock 测试
- odom anomaly 测试
- localizer failure 测试

## 输出方式
每个 phase 必须先输出实施计划，再改代码。
每完成一个 phase：
- 更新 README
- 更新 docs
- 说明还需要哪些真实硬件验证
