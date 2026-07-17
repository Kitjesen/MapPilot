# MuJoCo 导航场景设计准则

这份文档约束 LingTu 的 MuJoCo 导航验收场景。目标不是做一个“看起来像房间”的演示，而是让场景能验证真实产品链路：

```text
static scene -> map.pcd -> octomap.ot -> OctoPlanner3D
live lidar/person/terrain -> LocalPlanner / Safety
global path -> local path -> PathFollower -> RL locomotion
```

## 1. 设计结论

轨迹规划和腿式导航论文里的场景设计有几个共同点：

- 多楼层场景必须显式建模楼层和连接器。楼梯、坡道、电梯不是普通障碍物，而是楼层之间的拓扑连接。
- 四足机器人不能只按 2D collision 验证。必须检查身体包络、台阶高度、踏面深度、支撑区域、坡度和粗糙度。
- 可穿越性必须进入局部规划代价，而不是只做颜色显示。
- benchmark 必须包含可通过、可绕行、不可通过、动态障碍、定位压力和地形风险，不应该只放一个能成功的场景。
- OctoPlanner3D 的证据必须是三维 OctoMap occupied voxels 和三维 global path，不能用 2D 投影冒充。

参考：

- MuNES: Multifloor Navigation Including Elevators and Stairs
- Rough Terrain Navigation for Legged Robots using Reachability
- Traversability-Aware Legged Navigation by Learning from Real-World Experience
- Safe and Robust Mobile Robot Navigation in Uneven Indoor Environments
- OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees
- BARN / Benchmarking Metric Ground Navigation
- TartanGround

## 2. 场景族

当前代码里的标准场景 preset：

| Preset | 目的 | 期望 |
| --- | --- | --- |
| `stair_easy` | 普通上楼 | 10-15cm 台阶应规划并跟踪通过 |
| `stair_limit_23cm` | 能力边界 | 最高 23cm 台阶，可规划但应带高风险/降速 |
| `stair_blocked_30cm` | 负例 | 30cm 台阶必须被拒绝，不能硬冲 |
| `multifloor_two_connectors` | 路径选择 | 至少两个楼层连接器，验证全局规划选路 |
| `multifloor_stack_3` | 多楼层体素楼 | 三层楼板、侧边楼梯连接器、顶层目标 |
| `rough_terrain_traversability` | 地形风险 | 粗糙地面和低矮障碍进入 traversability cost |
| `stairs3d` | 兼容别名 | 旧场景入口，仅保留兼容 |

## 3. 房间设计

房间不能是空盒子，至少要有：

- 主通道：正常最短路径。
- 备用通道：主路径受阻时可绕行。
- 决策点：门洞、T 字路口、楼梯口、坡道口。
- 局部障碍：箱子、柱子、半高障碍。
- 视野遮挡：墙角、门框、临时遮挡。
- 安全边界：墙、栏杆、楼梯边缘、禁行区。

推荐尺寸：

- 门宽：`0.8m` 紧张，`1.0m` 常规，`1.2m` 宽松。
- 走廊宽：`0.8m` 紧张，`1.2m` 常规，`1.6m` 宽松。
- 房间：小房间 `4m x 4m`，中等房间 `8m x 6m`。
- 障碍与路径最小安全距离应覆盖 `0.2m`、`0.35m`、`0.5m` 三档。

## 4. 楼梯设计

楼梯必须按机器狗能力边界设计。

当前 LingTu 验收参数：

- 最大台阶高度：`0.23m`
- 普通台阶高度：`0.10m - 0.18m`
- 极限台阶高度：`0.20m - 0.23m`
- 踏面深度：不小于 `0.32m`，推荐 `0.38m - 0.45m`
- 楼梯宽度：不小于 `0.7m`，推荐 `0.9m - 1.2m`
- 平台长度：至少 `1.6m`

验收分三类：

| 类型 | 参数 | 期望 |
| --- | --- | --- |
| Easy stair | 10-15cm 高，40cm+ 踏面 | 应可规划并跟踪 |
| Limit stair | 20-23cm 高，42cm+ 踏面 | 应谨慎规划，局部规划应降速或提高风险 |
| Blocked stair | >23cm 或踏面不足 | 全局/局部应拒绝，不应硬冲 |

## 5. 地形设计

地形要验证 traversability，不只是几何碰撞。

需要覆盖：

- 平地：低风险。
- 楼梯：中高风险，需要支撑判断。
- 坡道：按坡度给风险。
- 粗糙地面：按 roughness 给风险。
- 低矮障碍：区分可跨越和不可跨越。
- 禁行区：硬拒绝。
- 未知区：默认高风险，不能当自由空间。

LocalPlanner 应消费：

```text
registered_cloud       # 实时障碍
traversability_grid    # 地形风险
esdf / clearance       # 安全距离
global_path corridor   # 全局路径约束
odometry               # 当前位姿
```

## 6. 动态障碍

动态障碍不能写进静态 OctoMap。

正确链路：

```text
static scene -> map.pcd -> octomap.ot -> OctoPlanner3D
dynamic person -> live lidar points -> LocalPlanner / Safety
```

动态人测试至少要覆盖：

- 横穿路径。
- 同向慢走。
- 逆向靠近。
- 在楼梯口停留。
- 从遮挡处出现。

验收指标：

- 最小人机距离。
- 是否减速。
- 是否停车。
- 是否绕行。
- cancel / emergency stop 是否及时。

## 7. 每个场景必须产出

```text
scene.xml
map.pcd
octomap.ot
metadata.json
octomap_occupied.xyz
global_path.json
local_path_timeseries.jsonl
trajectory.csv
octoplanner3d_map_viewer.html
report.json
```

报告必须包含：

- `planner = octoplanner3d`
- `search_algorithm = octomap_3d_astar`
- `max_step_height`
- `robot_radius`
- `path_z_range_m`
- `min_clearance_m`
- `traversability_grid`
- `dynamic_obstacle_source`
- `product_ready`
- `blockers`

## 8. 当前可运行命令

只验证地图和 OctoPlanner3D：

```bash
python sim/scripts/mujoco/saved_map_plan_gate.py \
  --scene-preset stair_limit_23cm \
  --out-dir artifacts/mujoco_nav_suite/stair_limit_23cm \
  --map-source synthetic_hits \
  --spacing 0.16 \
  --resolution 0.16 \
  --planner-clearance-cells 2 \
  --planner-robot-radius 0.25 \
  --planner-goal-tolerance-m 0.45 \
  --planner-goal-xy-tolerance-m 0.45 \
  --planner-goal-z-tolerance-m 0.7 \
  --strict
```

多楼层三维体素楼：

```bash
python sim/scripts/mujoco/saved_map_plan_gate.py \
  --scene-preset multifloor_stack_3 \
  --out-dir artifacts/mujoco_nav_suite/multifloor_stack_3_plan \
  --map-source synthetic_hits \
  --spacing 0.18 \
  --resolution 0.18 \
  --planner-clearance-cells 2 \
  --planner-robot-radius 0.25 \
  --planner-goal-tolerance-m 0.55 \
  --planner-goal-xy-tolerance-m 0.55 \
  --planner-goal-z-tolerance-m 0.8 \
  --strict
```

验证跟踪、局部规划和可穿越性：

```bash
python sim/scripts/mujoco/saved_map_tracking_gate.py \
  --scene-preset stair_limit_23cm \
  --out-dir artifacts/mujoco_nav_suite/stair_limit_23cm_tracking \
  --map-source synthetic_hits \
  --use-traversability-grid \
  --check-obstacle \
  --local-obstacle-source live_lidar \
  --drive-mode policy \
  --product-acceptance
```

负例：

```bash
python sim/scripts/mujoco/saved_map_plan_gate.py \
  --scene-preset stair_blocked_30cm \
  --out-dir artifacts/mujoco_nav_suite/stair_blocked_30cm \
  --map-source synthetic_hits \
  --strict
```

`stair_blocked_30cm` 的正确结果不是成功到达，而是明确拒绝或产生 blocker。

## 9. 对当前状态的评价

已经从单个 `stairs3d` 临时场景推进到命名场景族，但还不是完整产品验收。

已经有：

- 多个 scene preset。
- 23cm 台阶边界。
- 30cm blocked 负例。
- two-connectors 场景入口。
- 三层 stacked multi-floor 场景入口。
- rough-terrain 场景入口。
- 场景内置 traversability grid。
- plan/tracking report viewer 已支持 plan gate 和 tracking gate 两种报告；plan-only viewer 不再空白。

仍需补齐：

- 当前 `stair_blocked_30cm` 已能暴露 blocker：`blocked_scene_unexpectedly_reached_goal`。这说明 OctoPlanner3D 的体素级 step/clearance gate 仍需加强，不能把这个场景当作通过。
- 当前 `multifloor_stack_3` 严格产品配置会暴露 blocker：`traversable_scene_failed_to_plan`。关闭 ground support 的诊断版能规划 39 个三维路径点，说明地图和楼层连接可视化有效，但 OctoPlanner3D 的 ground-support 连通模型还不能正确处理多层重叠楼板和侧向楼梯连接器。
- 每个场景的正式视频录制标准。
- viewer 自动展示每帧 local path 和实际轨迹。
- 动态人多行为脚本。
- 在线 terrain module 替代内置网格。
- 负例必须在 CI 里断言“失败是正确结果”。
