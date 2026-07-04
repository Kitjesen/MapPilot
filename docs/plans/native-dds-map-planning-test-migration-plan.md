# Native DDS / Map / Planning Test And Migration Plan

## 结论

现在测试到的是「Map Workbench 后端合同 + Gateway 入口 + Web 构建」阶段，还没有完成真机上的 `pcd_to_octomap -> active map -> OctoPlanner3D preview -> LocalPlanner -> CmdVelMux dry-run` 全链路。

DDS 也还不能说全部完成：

| 链路 | 当前状态 | 缺口 |
| --- | --- | --- |
| Livox -> DDS | C++ `sdk2_stream` 已发布 `LivoxFrame / raw_packet / Imu` | 需要板子上长期跑包率、点数、时间戳验收 |
| DDS -> SLAM | C++ `cyclone_runtime.cpp` 已订阅 Livox/IMU，发布 odom/cloud/status | 需要真实 Fast-LIO2 live mapping 稳定性验收 |
| SLAM -> Gateway/Map | Gateway 能读状态和地图点云；MapService 能保存/导入/构建 artifact | 大点云/地图快照仍要做长期压力测试 |
| Nav DDS endpoint | C++ `lingtu_nav_native_endpoint` is now the field endpoint for `goal/odometry/registered_cloud/traversability -> OctoPlanner3D -> global_path -> local_path -> cmd_vel`; old Gateway-polling `lingtu_nav_cyclone_endpoint` was removed | Needs target-board build and live DDS route validation |
| Local planner | C++ core + Python Module 包装已存在 | 还不是独立 C++ DDS runtime |
| Global planner | OctoPlanner3D 是默认主全局规划器 | 还缺真 `octomap.ot` 上的 no-motion preview 验收 |

## 已经测过

| 测试 | 覆盖 | 结果 |
| --- | --- | --- |
| `src/nav/tests/test_maps_service.py` | MapService 导入、裁剪、构建 OctoMap、激活、artifact gate | 通过 |
| `src/gateway/tests/test_gateway_session_map_contract.py` | Gateway map route、Map Workbench route、真实 PCD 导入落盘 | 通过 |
| `src/gateway/tests/test_gateway_route_split.py` | 路由注册和 OpenAPI 合同 | 通过 |
| `src/gateway/tests/test_gateway_app_bootstrap.py` | 前端 client links 暴露 | 通过 |
| `npm --prefix web run build` | Web 编译，Map Workbench UI/API 类型 | 通过 |

当前最新相关测试：

```bash
python -m pytest src/nav/tests/test_maps_service.py src/gateway/tests/test_gateway_route_split.py src/gateway/tests/test_gateway_session_map_contract.py src/gateway/tests/test_gateway_app_bootstrap.py -q
```

验收结果：`149 passed`。

## 还没有测完

| 项 | 为什么重要 | 当前缺口 |
| --- | --- | --- |
| `octoplanner3d_pcd_to_octomap` 真实 C++ 二进制 | 证明 PCD 能变成 OctoPlanner3D 可读 3D map | 本地测试用 fake converter；真 converter 还没跑实测 |
| `octomap.ot` 元数据同源校验 | 防止 PCD 和 OctoMap 不匹配 | 单测覆盖，需要真实地图包验收 |
| `validate_plan` 真路径 | 证明 goal -> OctoPlanner3D -> global_path 通 | Gateway route 有，真 map 上还没验收 |
| local planner 真消费 global_path + terrain | 证明局部避障/可穿越性进入局部路径 | C++ core 有测试，runtime 链路还要跑 |
| CmdVel dry-run | 证明路径跟踪会产生命令但不发电机 | 需要 no-motion endpoint 验收 |
| C++ DDS nav endpoint | 证明 DDS goal/path/cmd_vel 可替掉 Python adapter | 二进制有，现场默认链路未完成 |

## PCD -> OctoMap 测试计划

### P0：本地二进制构建

目标：确认 `octoplanner3d_pcd_to_octomap` 能在目标 Linux 环境构建。

命令：

```bash
scripts/build/build_octoplanner3d.sh
```

验收：

- 生成 `build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap`
- 生成 `build/octoplanner3d_headless/octoplanner3d_headless`
- 生成 `build/octoplanner3d_headless/octoplanner3d_edit_octomap`

失败处理：

- 缺 PCL/OctoMap：先修依赖，不允许回退 fake converter。
- 缺 CycloneDDS 不影响这个阶段。

### P1：样例 PCD 转 OctoMap

目标：真实 PCD 写出非空 `octomap.ot`。

命令：

```bash
build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap \
  --input /path/to/map.pcd \
  --output /tmp/lingtu_octomap_test/octomap.ot \
  --resolution 0.20 \
  --free-layers-above 3 \
  --free-dilation-cells 1 \
  --frame map
```

验收：

- `octomap.ot` 存在且非空
- converter 返回码为 0
- 输出 JSON 或日志里能看到 resolution/frame

### P2：MapService build_octomap

目标：证明不是手工命令，而是产品服务能构建地图包。

命令：

```bash
export LINGTU_MAP_ARTIFACT_CONVERTER=/opt/lingtu/nav/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap
python lingtu.py map
# 或通过 Gateway:
curl -X POST http://127.0.0.1:5050/api/v1/maps/<name>/build_octomap
```

验收地图目录：

```text
map.pcd
metadata.json
map_record.json
octomap.ot
occupancy.npz
```

验收字段：

- `metadata.json.artifacts.octomap.path == "octomap.ot"`
- `map_record.json.state == "READY"` 或 active 后为 `ACTIVE`
- `/api/v1/slam/maps` 返回 `has_octomap=true`
- `/api/v1/slam/maps` 返回 `navigation_ready=true`

### P3：Web Map Workbench

目标：操作者不用命令行也能完成地图导入和构建。

步骤：

1. 打开 Web `Map Workbench`
2. 输入 map name
3. 输入机器人/Gateway 主机上的 PCD 路径
4. 点击 `Import PCD`
5. 点击 `Build OctoMap`
6. 地图列表进入 `可导航地图`

验收：

- Import 后地图为 `STALE`
- Build 后地图为 `navigation-ready`
- 不要求 Web 直接算路径

## Global Planning 测试计划

### G0：artifact gate

目标：没有 `octomap.ot` 时 OctoPlanner3D 不假装能规划。

验收：

- `validate_plan` 返回 409 或 preview 不可行
- blocker 包含 `octomap required`
- 不生成 direct/A*/PCT fallback 路径

### G1：no-motion route preview

目标：起点来自 odometry，终点来自 Web 点击或 API，规划只预览，不发运动命令。

命令：

```bash
curl -X POST http://127.0.0.1:5050/api/v1/maps/<name>/validate_plan \
  -H "Content-Type: application/json" \
  -d '{"x": 2.0, "y": 1.0, "z": 0.0}'
```

验收：

- `motion_published=false`
- `preview.selected_planner=octoplanner3d`
- `preview.path` 至少 2 个点
- `preview.fallback_reason=""`
- `global_path` 没有发给电机，只用于预检显示

### G2：真实 NavigationModule 预览

目标：不是单独调用 OctoPlanner3D，而是走 Navigation 的 preview 入口。

验收：

- Gateway `ControlCommandService.preview_navigation_plan()`
- Navigation `preview_plan()`
- GlobalPlannerService
- OctoPlanner3D runtime
- 返回同一个 `selected_planner=octoplanner3d`

## Local Planning 测试计划

### L0：C++ core 单测

目标：局部规划底层算法本身能处理障碍和可穿越性。

命令：

```bash
cd src/nav/services/plan/local_planner/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release -DLOCAL_PLANNER_CPP_BUILD_TESTS=ON
cmake --build build -j
./build/test_local_planner_core
```

验收：

- obstacle 会影响路径选择
- traversability hard cost 会阻断路径
- near-field stop 会输出停止 hint

### L1：Module runtime 链路

目标：`global_path/waypoint + odometry + terrain/esdf/traversability` 进入 LocalPlanner，输出 `local_path/control_hint`。

输入：

- `odometry`
- `global_path`
- `waypoint`
- `terrain_map`
- `terrain_map_ext`
- `boundary`
- `added_obstacles`
- `traversability`
- `esdf`

输出：

- `local_path`
- `control_hint`
- `alive`

验收：

- 有 waypoint 时输出至少 2 个 local path 点
- 障碍在近场时 `control_hint` 包含 slow/stop/recovery reason
- map frame jump 后清空旧 local path

### L2：PathFollower + CmdVelMux dry-run

目标：路径跟踪能算速度，但不直接控制电机。

验收：

- `PathFollower.local_path -> cmd_vel`
- `CmdVelMux` 选择最高优先级源
- teleop 高于 autonomy
- dry-run 模式 `driver_cmd_vel` 不发真实 driver

## DDS 测试计划

### D0：IDL / C++ type 生成

目标：所有 C++ DDS runtime 用同一份 IDL。

验收：

- `src/message/idl/lingtu_slam.idl` 生成成功
- `LivoxFrame / Imu / Odometry / PointCloud2 / Path / TwistStamped / Text` 都能编译

### D1：Livox DDS publisher

目标：传感器触发式数据用 DDS。

验收：

- raw packet 诊断约 50Hz
- scan 级 `LivoxFrame` 约 10Hz
- 每帧点数符合配置和现场数据
- IMU 正常发布

### D2：SLAM DDS runtime

目标：SLAM 直接吃 DDS Livox/IMU，输出定位和地图。

验收：

- `slam_hz >= 9Hz`
- `has_odom=true`
- `registered_cloud` frame = `body`
- `map_cloud` frame = `map`
- `map_odom_tf` 有更新
- `localization_quality` 非空

### D3：Nav DDS endpoint

目标：导航请求和输出通过 DDS，而不是 Python DDS adapter。

验收：

- DDS `goal_pose` 能到 Gateway/Navigation
- DDS `cancel` 能取消 mission
- DDS `global_path/local_path/cmd_vel` 能被外部 C++ reader 读到
- no-motion preview 不发布 `cmd_vel`

当前状态：C++ endpoint 文件和构建脚本存在，但还没完成现场默认替换。

## 迁移计划

### Phase 1：锁住地图包链路

目标：所有导航地图都按同一规则判断。

必须完成：

- `map.pcd -> octomap.ot -> metadata.json -> map_record.json`
- `/api/v1/slam/maps` 返回 `has_octomap/navigation_ready/state`
- `active` map 必须通过 artifact gate

不做：

- 不恢复 PCT/direct/A* 为产品 fallback
- 不把 Qt/ROS editor 搬进来

### Phase 2：真 converter 上板

目标：`pcd_to_octomap` 在 S100P 上可构建、可运行。

步骤：

1. 部署 C++ OctoPlanner3D runtime
2. 设置 `LINGTU_MAP_ARTIFACT_CONVERTER`
3. 用保存地图跑 `build_octomap`
4. 验证 `navigation_ready=true`

### Phase 3：OctoPlanner3D preview 闭环

目标：证明全局规划主链路真实可用。

步骤：

1. 激活地图
2. 从当前 odometry 取起点
3. Web/API 点目标
4. 跑 `validate_plan`
5. 检查 `selected_planner=octoplanner3d`
6. 检查没有 fallback

### Phase 4：局部规划 runtime 闭环

目标：全局路径能进入局部规划，局部规划能考虑地形/障碍。

步骤：

1. `global_path -> waypoint`
2. `terrain_map/esdf/traversability -> LocalPlanner`
3. `local_path/control_hint -> PathFollower`
4. `cmd_vel -> CmdVelMux`
5. dry-run 验证，不发真实电机

### Phase 5：DDS endpoint 替换

目标：把现场职责从 Python DDS adapter 迁到 C++ endpoint。

顺序：

1. Livox DDS 保持 C++ publisher
2. SLAM DDS 保持 C++ runtime
3. Nav DDS endpoint 先只做 mirror 输出
4. Gateway 状态确认一致后，切 goal/cancel 输入
5. 最后切 path/cmd_vel 输出

回滚：

- 保留 Python Module 内部链路
- DDS endpoint 按服务开关切换
- 不保留 ROS2 作为业务 fallback

### Phase 6：现场验收

目标：真实机器人无运动预检通过后，再进入低速运动。

顺序：

1. no-motion preview
2. local planner dry-run
3. CmdVelMux dry-run
4. 低速短距离跟踪
5. 障碍物近场 stop
6. teleop override
7. emergency stop

## 当前下一步

下一步只做两件事：

1. 在板子上构建并运行真实 `octoplanner3d_pcd_to_octomap`。
2. 用真实 `octomap.ot` 跑 `/api/v1/maps/<name>/validate_plan`，要求 `selected_planner=octoplanner3d` 且 `motion_published=false`。

这两件没过之前，不继续堆 Web 编辑器和更多模式。
