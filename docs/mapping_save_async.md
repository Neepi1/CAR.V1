# 建图保存与电梯测试锁去耦

本次只移除电梯测试对其他 API 的准入依赖，不删除地图文件写入互斥、建图
会话的 start/save/stop 串行，也不修改机器人底层安全链或机械臂黑盒。
旧公开接口中的 epoch 和部分端口槽位保留二进制兼容，但非电梯业务不再调用。
电梯模块内部的任务状态、取消/恢复流程不在本次改动范围；不能据此声称已经
删除电梯内部所有锁或重写了恢复策略。

## 新客户端协议

- `POST /api/v1/mapping/2d/save`：携带 `async:true`、`request_id`、楼栋、楼层和名称。
- 返回 202 仅表示受理，GET `/api/v1/mapping/2d/save/status?request_id=...` 查询结果。
- 状态包含 `state`、`phase`、`map_saved`、`mapping_stopped`、`result_status` 和 `result`。
- `running/saving` → `running/stopping`（资产已落盘）→ `finished`。
- 仅 `finished && map_saved && mapping_stopped` 表示保存和退出均完成。
- 保存后退出异常，必须保留 map_id 和已落盘事实，不显示“地图没保存”。
- 同 ID 同目标返回旧任务；同 ID 不同目标返回冲突。查询/重试不会重新写图。
- 接收前冻结 OccupancyGrid，页面释放 live_map 不影响正在执行的保存。
- 结果记录在 `runtime_maps_dir/save_jobs`；API 重启后的未结束记录为
  `recovery_required`，不重放，不停止新会话。查询中会复核已提交资产身份。
- 日志分别记录 `phase=assets_committed` 和 `phase=shutdown_finished` 耗时。
  尚未增加每个进程的退出明细，不擅自缩短已有退出预算。

老客户端省略 async 时仍得到原同步结果；新版 App 用 openapi 中的状态接口
进行能力协商，不能单独更新后端就声称老 App 的 12 秒超时已解决。
App 将未完成请求 ID 持久化，断线后继续查同一请求。三分钟页面等待结束只显示
结果待确认，不把它当作未保存，也不自动发送新请求。已由后端停止的建图不再
重复调用 stop。

## 验证与待验收

- 路由测试覆盖状态查询不等电梯测试锁，正常路由次序和仅电梯端点准入。
- 保存任务测试覆盖慢退出、重复 ID、已保存后退出失败、重启后的未完成记录。
- `test_mapping_save_routing` 使用真实路由、建图模块、地图写入和进程管理类，
  替换 OS 进程边界；模拟退出等待 31 秒，检查状态/心跳、资产锁和仅一份地图。
- 实机验收必须在新版 API 和 App 均启用后，由用户执行一次保存：对照新任务
  状态、不可变地图身份和退出耗时。隔离测试成功不等同于已完成实机保存验收。
- 当前轮只暂存候选二进制；不替换运行文件，不重启、不移动小车。
