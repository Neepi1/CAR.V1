# App Codex Prompt: Robot Runtime State Contract

请在 Android App 端接入车端 `robot_api_server` 的轻量业务状态，不要通过 ROS topic/node 名称推断建图或导航状态。

## 后端接口

轮询：

```text
GET http://<robot-ip>:8080/api/v1/status
```

响应里新增并优先使用这些字段：

```json
{
  "mode": "IDLE|MAPPING_2D|NAVIGATION|ERROR",
  "state": "idle|starting|running|saving|stopping|stopped|ready|navigating|canceling|error",
  "mapping_active": false,
  "navigation_active": false,
  "healthy": true,
  "message": "",
  "mapping": {
    "active": false,
    "state": "stopped",
    "map_topic": "/map",
    "map_endpoint": "/api/v1/mapping/2d/map"
  },
  "navigation": {
    "active": false,
    "state": "stopped",
    "action": "/navigate_to_pose"
  }
}
```

## App 判断规则

- 建图中：`mode == "MAPPING_2D" && mapping_active == true`
- 导航中：`mode == "NAVIGATION" && navigation_active == true`
- 空闲：`mode == "IDLE"`
- 异常：`mode == "ERROR" || healthy == false`

不要用 `/map` 是否存在判断建图；导航时也会有 `/map`。
不要用 `/cmd_vel` 是否非零判断导航；避障、等待、到点时都可能为 0。
不要用 `slam_toolbox` 节点是否存在判断建图；以后可能会保留进程但没有 active session。

## 页面建议

首页/机器详情：

```text
GET /api/v1/status
```

建图页：

1. `POST /api/v1/mapping/2d/start`
2. `POST /api/v1/subscriptions/acquire`，申请 `live_map, tf, teleop`
3. 轮询 `GET /api/v1/status`
4. `mode == MAPPING_2D && mapping_active == true` 后显示建图中
5. 地图图像仍从 `GET /api/v1/mapping/2d/map` 获取
6. 停止或保存成功后，如果 `mapping_active == false`，退出建图态

导航页：

1. `POST /api/v1/floors/switch`，带 `resume_navigation: true`
2. 轮询 `GET /api/v1/status`
3. `mode == NAVIGATION && navigation_active == true` 后显示导航链已启动
4. 下发目标：`POST /api/v1/navigation/goal`
5. 目标接受后继续轮询 `/status`
6. 取消：`POST /api/v1/navigation/cancel`
7. 直到 `navigation_active == false` 或 `mode == IDLE`，显示已停止

## UI 文案映射

```text
IDLE -> 空闲
MAPPING_2D / starting -> 建图启动中
MAPPING_2D / running -> 建图中
MAPPING_2D / saving -> 保存地图中
MAPPING_2D / stopping -> 停止建图中
NAVIGATION / starting -> 导航链启动中
NAVIGATION / ready -> 导航就绪
NAVIGATION / navigating -> 导航中
NAVIGATION / canceling -> 取消导航中
ERROR -> 异常，显示 message
```

## 兼容要求

如果旧后端暂时没有 `mode` 字段，App 可以降级为：

```text
mapping_active == true -> MAPPING_2D
navigation_active == true -> NAVIGATION
否则 IDLE
```

但新版本应优先使用 `mode/state/healthy/message`。
