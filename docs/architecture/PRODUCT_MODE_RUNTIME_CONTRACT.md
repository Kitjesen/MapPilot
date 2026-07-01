# LingTu 产品模式运行合同

本文说明六个一层产品模式需要串联的功能。代码合同在
`src/runtime/profiles/product_mode_contracts.py`，静态图测试在
`src/runtime/tests/test_profile_graph_snapshots.py`。

## 总原则

前端、CLI、MCP 只发请求和显示状态，不直接决定导航路径，也不直接绕过
`CmdVelMux` 控制机器狗。

目标点只进入任务或目标服务。只有 `tracking`、`nav`、`inspection` 会启动
`nav.mission -> nav.local_planner -> nav.path_follower -> nav.velocity_mux`。

速度命令必须统一经过：

```text
Teleop / VisualServo / PathFollower / Recovery
  -> nav.velocity_mux
  -> nav.out.cmd_vel
  -> endpoint / driver
```

## 六个模式

| Profile | 产品模式 | 必须串联 | 禁止串联 | 切换策略 |
| --- | --- | --- | --- | --- |
| `teleop` | 遥控 | Gateway/Teleop/MCP -> CmdVelMux -> Safety -> NavOut | SLAM、全局规划、局部规划、路径跟踪 | 冷重启 |
| `teleop_avoid` | 遥控避障 | SLAM/地图/可通行代价 + Teleop -> CmdVelMux -> Safety -> NavOut | mission、local planner、path follower、语义规划 | 冷重启 |
| `map` | 建图 | SLAM -> Occupancy/Voxel/Elevation/ESDF/Traversability -> Gateway/MapManager；Teleop -> CmdVelMux -> NavOut | mission、local planner、path follower、语义规划 | 冷重启 |
| `tracking` | 跟踪 | GoalService/NavIn -> Mission -> LocalPlanner -> PathFollower -> CmdVelMux -> NavOut | 语义规划 | 冷重启，未来可做同图热切候选 |
| `nav` | 导航 | Web/CLI/MCP/语义目标 -> Mission -> OctoPlanner3D -> LocalPlanner -> PathFollower -> CmdVelMux -> NavOut | 目标直接变电机命令 | 冷重启，未来可做同图热切候选 |
| `inspection` | 巡检 | Scheduler/Patrol/Semantic -> GoalService -> Mission -> OctoPlanner3D -> LocalPlanner -> PathFollower -> CmdVelMux -> NavOut | 巡检任务直接控制底盘 | 冷重启，未来可做同图热切候选 |

## 功能链路

### 遥控链

```text
GatewayModule.cmd_vel
MCPServerModule.cmd_vel
TeleopModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel
  -> nav.velocity_mux.driver_cmd_vel
  -> nav.out.cmd_vel
  -> nav.safety.cmd_vel
```

### 地图链

```text
SlamBridgeModule.map_cloud
  -> OccupancyGridModule.map_cloud
  -> VoxelGridModule.map_cloud
  -> ElevationMapModule.map_cloud
  -> nav.maps.map_cloud

OccupancyGridModule.costmap
ESDFModule.esdf
ElevationMapModule.elevation_map
  -> TraversabilityCostModule
  -> GatewayModule.costmap
```

### 导航执行链

```text
GatewayModule / MCPServerModule / SemanticPlannerModule
  -> nav.goals / nav.mission.goal_pose
  -> nav.mission.global_path
  -> nav.local_planner.global_path
  -> nav.local_planner.local_path
  -> nav.path_follower.local_path
  -> nav.path_follower.cmd_vel
  -> nav.velocity_mux.path_follower_cmd_vel
  -> nav.out.cmd_vel
```

## 当前切换结论

现在只提供切换预检，不做在线热切。`tracking`、`nav`、`inspection` 被标为
同图热切候选，是为了后续实现时有明确边界；当前 `switch-plan` 仍返回
`required_lifecycle=cold_restart`。

验收命令：

```bash
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_product_modes_required_wires_are_contract_locked -q
python lingtu.py switch-plan teleop nav --json
python lingtu.py switch-plan tracking inspection --json
```
