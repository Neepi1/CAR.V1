# lidar_level_link 2D 切片链验证报告

生成时间：2026-04-22  
验证环境：Jetson `192.168.31.23` / 容器 `NJRH-car`

## 目标

确认当前 2D 建图切片链满足以下工程约束：

1. 原始输入点云保持为 `/lidar_points`，`header.frame_id = lidar_link`
2. `nav_cloud_preprocessor` 先把点云从 `lidar_link` 变换到 `lidar_level_link`
3. `nav_cloud_preprocessor` 输出 `/points_nav`，且 `header.frame_id = lidar_level_link`
4. `pointcloud_to_laserscan` 消费 `/points_nav`
5. 2D 切片在 `lidar_level_link` 坐标系下完成
6. `/scan_raw` 与 `/scan` 的 `header.frame_id` 均为 `lidar_level_link`
7. 不是“切完再改 frame_id”，而是切片前点云已经位于 `lidar_level_link`

## 配置结论

当前仓库配置已切换为：

- `scripts/jetson/runtime_overlay/config/jt128_nav_cloud_preprocessor.yaml`
  - `input_topic: /lidar_points`
  - `output_topic: /points_nav`
  - `output_frame_id: lidar_level_link`
- `scripts/jetson/runtime_overlay/launch/jt128_slam_toolbox_mapping.launch.py`
  - `nav_cloud_preprocessor.output_frame_id = lidar_level_link`
- `scripts/jetson/runtime_overlay/launch/jt128_localization_sensing.launch.py`
  - `nav_cloud_preprocessor.output_frame_id = lidar_level_link`
- `scripts/jetson/runtime_overlay/config/jt128_scan_slam2d.yaml`
  - `target_frame: lidar_level_link`

## 实机 live 验证

在 Jetson 容器内实时读取 ROS 话题与 `pointcloud_to_laserscan` 参数，得到：

| 项目 | 实际值 |
| --- | --- |
| `/lidar_points.header.frame_id` | `lidar_link` |
| `/points_nav.header.frame_id` | `lidar_level_link` |
| `/scan_raw.header.frame_id` | `lidar_level_link` |
| `/scan.header.frame_id` | `lidar_level_link` |
| `pointcloud_to_laserscan.target_frame` | `lidar_level_link` |

关键样本数据：

- `/lidar_points`
  - `frame_id = lidar_link`
  - `first_point_xyz = (4.0478, 0.0142, -0.3136)`
  - `width = 57600`
- `/points_nav`
  - `frame_id = lidar_level_link`
  - `first_point_xyz = (-2.6977, -6.6586, 1.5799)`
  - `width = 13726`
- `/scan_raw`
  - `frame_id = lidar_level_link`
  - `first_finite_range = 8.9332`
- `/scan`
  - `frame_id = lidar_level_link`
  - `first_finite_range = 8.9372`

## 为什么这说明“切片前已经变换到 lidar_level_link”

这次验证里，`/points_nav` 不只是把 `frame_id` 从 `lidar_link` 改名为 `lidar_level_link`，而是点云数值本身已经发生了实际变化：

- 原始点云首点：`(4.0478, 0.0142, -0.3136)` in `lidar_link`
- 预处理点云首点：`(-2.6977, -6.6586, 1.5799)` in `lidar_level_link`

如果只是“切完再改 header.frame_id”，点云坐标值不会出现这种变化。  
现在看到的是：

1. 原始云先进入 `nav_cloud_preprocessor`
2. 在预处理阶段完成 `lidar_link -> lidar_level_link` 变换
3. 输出 `/points_nav(lidar_level_link)`
4. `pointcloud_to_laserscan` 再基于该 leveled cloud 做高度切片

因此当前切片依据的实际 frame 是 `lidar_level_link`，不是 `lidar_link`，也不是“先在别的 frame 切完再改名”。

## 当前生效链路

```text
/lidar_points                frame_id = lidar_link
  -> nav_cloud_preprocessor  transform to lidar_level_link
  -> /points_nav             frame_id = lidar_level_link
  -> pointcloud_to_laserscan target_frame = lidar_level_link
  -> /scan_raw               frame_id = lidar_level_link
  -> /scan                   frame_id = lidar_level_link
```

## 测试结果

仓库约束测试：

- `src/robot_system_tests/test/test_workspace_contracts.py`
- 结果：`22 passed`

## 结论

本次调整后，2D 建图与重定位切片链已经满足“先把点云变换到 `lidar_level_link`，再在 `lidar_level_link` 中切片”的要求。  
当前 2D 建图扫描链吃的不是原始 `lidar_link` 点云坐标，而是 `nav_cloud_preprocessor` 变换后的 `/points_nav(lidar_level_link)`。
