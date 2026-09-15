# JT128 CPU 点云遍历融合

## 范围与语义

本次仅优化 `pointcloud_accel_core.cpp` 当前 canonical 点云入口：将轴转换和内部 XYZI 缓存填充的两次全量遍历合并为一次。实现位于 `include/robot_hesai_jt128/detail/fused_normalization.hpp`，生产 core 与测试使用同一实现。

适用条件是 little-endian、连续行、FLOAT32 XYZ 位于 0/4/8、数据完整及现有两种 swap/negate 旋转。所有检查和容量预留在修改前完成；不适用则完全不修改输入或缓存，交回原有处理路径。本次没有修订原回退路径对非标准数据的历史处理行为。

保留完整 `/lidar_points`，不降采样、不删除字段，不改点序、原始时间戳、frame 配置、QoS、DDS。内部缓存仍先准备、发布 trunk 后交换；保留原独占缓存复用判据。不修改 scan worker、角度格、高度切片、自遮罩、TF、Nav2、StopZone、GPU 驱动、IMU 或 CPU 绑核。生产旧 PointCloud2 障碍/清障分支仍关闭，避障继续使用现有 `/scan` 链路。

## 验证

- 单元测试调用生产 helper，覆盖完整输出字节、强度缺省、特殊浮点位模式、连续多行、第二种旋转、容量复用、非适用布局不修改和 intensity 别名。
- 真实 core 集成测试通过私有话题验证 trunk、原时间戳、scan 角度格、高度过滤、自遮罩和跨帧输出；另验证缺少自遮罩 TF 时的既有行为。
- 测试仅使用合成点云，独立 ROS 域 143、localhost-only；不订阅生产高频点云，不操作生产服务。
- 同为 `RelWithDebInfo/-O2` 的微基准比较 40000 点、32 字节步长的旧双遍参考与新生产 helper；交替 AB/BA，每次逐字节检查输出。输入复制及校验不计时；结果不能等同整进程 CPU 收益。

## 部署与硬件验收

2026-09-13 UTC 13:42:50 完成实机构建与隔离测试：6 个融合单元测试、4 个原自遮罩测试、2 个真实 core 集成测试全部通过。集成测试以 4 帧覆盖缓存再使用；没有生产点云订阅。旧静态契约两项仍在 `test_workspace_contracts.py:12274/12552` 要求已移除的 IMU 启动提示，本次修改前 core 同样失败，未扩改无关 IMU 逻辑。

400 组 AB/BA、20 组预热、`-O2 -g -DNDEBUG`、CPU4 单核低优先级下：旧双遍线程 CPU median/p95 = 416.624/524.480 us，新生产 helper = 247.216/402.304 us；wall median/p95 = 422.256/1937.472 → 245.520/917.440 us。算法 CPU median 减少 40.66%，全部输出逐字节相同。此结果不包含 DDS、scan worker 与进程其余成本，不能声称整个 axis 进程或导航链减少 40.66%。

本轮不重启服务、不移动小车。源文件、测试与候选程序的同步/哈希及测试日志写入 `/tmp/njrh_reports/pointcloud_fusion_20260913_7yOl9I`。部署采用已校验文件备份和可执行文件原子替换，磁盘候选更新不会改变仍运行的旧进程。

启用须另获整套 `njrh-runtime.service` 重启授权；启用后核对真实进程 exe 哈希、唯一性及现有低频 diagnostics 的 trunk/scan 发布频率和时延，再同条件比较 CPU。合成测试通过不代表实车避障验收完成。
