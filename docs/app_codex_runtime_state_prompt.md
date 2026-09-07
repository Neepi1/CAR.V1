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

1. 轮询 `GET /api/v1/status`
2. `mode == NAVIGATION && navigation_active == true` 只表示驻留导航运行栈
   已存在；在 status 报告导航 ready/running 且下一步位姿身份确认前，显示
   “导航链启动中”，不要开放下发目标
3. 调用 `GET /api/v1/robot/pose`，确认返回的
   `building_id/floor_id/map_id` 就是目标点所属运行地图
4. status 已就绪且位姿三元组确认后，才显示“导航就绪”并下发目标：
   `POST /api/v1/navigation/goal`
5. 目标接受后轮询 `GET /api/v1/navigation/state`，以
   `task_complete`、Nav2 result 和 job state 判断本次任务
6. 取消：`POST /api/v1/navigation/cancel`
7. 驻留导航链在单次目标结束后仍可保持 active；不要用
   `navigation_active == false` 判断单次目标是否结束

不要在导航页调用 `POST /api/v1/floors/switch` 并携带
`resume_navigation:true`；该调用固定返回
`LIVE_FLOOR_SWITCH_DISABLED`。运行中跨层切图使用独立事务：调用
`POST /api/v1/floor-switch/start`，保存车端返回的 `transaction_id`，轮询
`GET /api/v1/floor-switch/state` 到明确终态；取消仅调用
`POST /api/v1/floor-switch/cancel` 后继续轮询。不得回退到离线选图或重新
启动 Nav2。导航/定位运行栈可以保持常驻，但必须没有活动目标且车辆已停稳，
由车端严格事务负责验证。

地图编辑页：

1. 编辑器选图只保存为 App 本地 `building_id/floor_id/map_id` 状态，按
   `map_id` 读取地图和语义层，不调用 `/api/v1/floors/switch`
2. 普通静态编辑不需要机器人当前位姿
3. “现场标点”调用 `GET /api/v1/robot/pose`
4. 只有返回的三元组与编辑器地图完全一致，并且本次目标已经结束、小车
   已停稳，才允许保存当前位置
5. 普通点调用 `/api/v1/maps/poses/save_current`
6. 电梯 schema-v3 四类内部点（`hall_call`、`landing`、`cabin`、
   `cabin_panel`）先 `GET /api/v1/elevator-config`，保留完整
   configuration、两种面板左右侧和最新 draft revision；再把
   `/api/v1/robot/pose` 的 `x/y/yaw` 写入所选角色，携带最新
   `expected_draft_revision`，将完整 configuration 提交到
   `PUT /api/v1/elevator-config/draft`

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
