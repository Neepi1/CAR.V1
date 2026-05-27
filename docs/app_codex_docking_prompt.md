# App Codex Prompt: Docking API Integration

请在 Android App 端接入车端 `robot_api_server` 的对桩接口。不要让 App 直接发布 `/cmd_vel` 或绕过安全链路。

## 后端接口

车端地址：

```text
http://<robot-ip>:8080/api/v1
```

保存充电桩点位：

```http
POST /api/v1/maps/poses
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_xxx",
  "pose_id": "dock_main",
  "type": "dock",
  "name": "主充电桩",
  "x": 1.25,
  "y": -0.35,
  "yaw": 3.14159
}
```

点位语义：这是最终充电接触成功时机器人 `base_link` 在地图中的位姿，不是预对桩点。车端会按配置自动从这个位姿后退 `0.80m` 生成预对桩点。

启动对桩：

```http
POST /api/v1/docking/start
Content-Type: application/json
```

```json
{
  "building_id": "B1",
  "floor_id": "F1",
  "map_id": "map_xxx",
  "dock_id": "dock_main",
  "resume_navigation": true
}
```

取消对桩：

```http
POST /api/v1/docking/cancel
Content-Type: application/json
```

```json
{"reason":"app_cancel"}
```

查询对桩状态：

```http
GET /api/v1/docking/state
GET /api/v1/status
```

`/status` 中新增字段：

```json
{
  "mode": "IDLE|MAPPING_2D|NAVIGATION|DOCKING|ERROR",
  "docking_active": true,
  "docking": {
    "active": true,
    "state": "accepted|nav_to_predock|fine_docking|docked|failed|canceled",
    "dock_id": "dock_main",
    "last_status": "..."
  }
}
```

## App 行为

- 地图编辑页允许用户添加 `type=dock` 的充电桩点位。
- 用户点“回充/对桩”时调用 `POST /api/v1/docking/start`，不要直接发速度。
- 对桩过程中轮询 `/status` 或 `/docking/state`。
- `docking.state == "docked"` 显示已对桩/充电。
- `docking.state == "failed"` 显示失败原因 `docking.last_status` 或 `message`。
- 用户点取消时调用 `/api/v1/docking/cancel`。

## 工程边界

后端执行链路是：

```text
App
 -> robot_api_server
 -> Nav2 导航到预对桩点
 -> robot_docking_manager GS2 精对桩
 -> robot_safety
 -> ranger_mini3_mode_controller
 -> ranger_base_node
```

App 不启动 Nav2 lifecycle，不订阅 ROS DDS，不直接控制底盘。
