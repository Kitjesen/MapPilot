# Native DDS 导航执行计划

## 结论

现场主路径必须走 C++ DDS：Livox、SLAM、地图、OctoPlanner3D、LocalPlanner、PathFollower、Gateway 状态可观测链路都不能再依赖 `cyclonedds-python`。

Python DDS nav 已删除或退出现场主链路。Python 只保留 Gateway/API、任务编排、状态展示、离线测试和兼容调试入口；不能作为机器人启动、规划、局部控制或安全控制的必需依赖。

## 当前主链路通信方式

| 环节 | 当前通信方式 | 要点 |
| --- | --- | --- |
| Livox MID-360 | Livox SDK2 C++ 回调 -> C++ CycloneDDS typed topic | 发布原始点云和 IMU；不走 Python DDS。 |
| SLAM | C++/ROS2 SLAM sidecar -> typed DDS/桥接输出 | 发布 odometry、map cloud、localization health；回调只入队，避免阻塞主循环。 |
| Map | C++/Python map 服务消费 SLAM map artifact 和 traversability 输出 | 保存/使用地图仍由 MapManager/Gateway 管理；实时导航输入优先走 typed DDS 或本地 artifact。 |
| OctoPlanner3D | C++ native endpoint 内进程调用 | 从目标、里程计、地图/可通行性输入生成 global path；不通过 Gateway 轮询闭环。 |
| LocalPlanner | C++ `NavLoop` 内 `LocalPlannerCore` | 消费 global path、odometry、traversability，生成 local path。 |
| PathFollower | C++ `NavLoop` 内 `PathFollowerCore` | 消费 local path 和 odometry，生成 `cmd_vel`。默认 fail-closed。 |
| Gateway | HTTP/API/SSE + 状态快照 | 只做观测、任务入口和人工操作面；可读取 native endpoint JSON 状态，不参与实时控制闭环。 |

速度输出规则：`cmd_vel` 由 C++ endpoint 显式开关控制，现场默认 `LINGTU_NAV_PUBLISH_CMD_VEL=0`；打开前必须完成 no-motion 验证和近场安全确认。

## 任务状态

### 已完成

- C++ native DDS endpoint 已存在：`lingtu_nav_native_endpoint`。
- endpoint 已支持状态 JSON、`--publish-cmd-vel` / `LINGTU_NAV_PUBLISH_CMD_VEL`。
- C++ `NavLoop`、`LocalPlannerCore`、`PathFollowerCore` 已有基础闭环和单元测试面。
- traversability DDS 发布端已补入。
- `lingtu-nav-dds.service` 已作为现场服务面存在。
- native nav endpoint 状态快照已包含 `global_path`、`local_path` 和最后一次 `cmd_vel` 建议。
- native nav endpoint 已订阅 `/nav/semantic/instruction`，当前显式拒绝并写入状态原因，避免自然语言指令静默丢失。
- Gateway `/api/v1/navigation/dds_snapshot` 已能在 Module 缓存为空时读取 native 状态快照，回填全局路径、局部路径和速度建议。
- 文档已明确现场不要求 `cyclonedds-python`，C++ CycloneDDS C API + `idlc` 是主路径。
- 旧 Gateway-polling `nav_cyclone_endpoint.cpp` 桥接已移出主路径。

### 正在补

- 补齐 native endpoint 对 goal、odometry、registered cloud、traversability 的 TTL、过期原因和 fail-closed 状态。
- 对齐 IDL、topic 名称、`dds_topics.hpp`、`config/topic_contract.yaml`。
- 固化 no-motion 验证：路径可生成、状态可审计、`cmd_vel_published=0`。
- 限定 Python DDS adapter 为测试/兼容入口，缺包时必须清晰失败或关闭，不能 crash-loop。
- 现场 service 默认保持 `publish_cmd_vel=false`。

### 还没做

- S100P 上完整构建 Livox DDS、SLAM DDS、traversability DDS、nav native endpoint。
- 现场联调真实 map artifact -> OctoPlanner3D -> LocalPlanner -> PathFollower。
- 将自然语言/语义指令转换成明确 `goal_pose` 的 C++/Module 边界，目前 native endpoint 不直接执行语义指令。
- 打开 `cmd_vel` 后验证 CmdVelMux、安全层、Thunder sink 没被绕过。
- 把每次现场证据写入 `docs/07-testing/field-runs/<date>.md`。
- 清理所有会让产品路径隐式依赖 `cyclonedds-python` 的配置、文档和测试假设。

## P0 执行计划：清掉 Python DDS 主链路依赖

目标：机器人现场启动和导航不需要 `cyclonedds-python`。

- 保留 Python DDS adapter 仅用于开发机调试、兼容测试或显式实验模式。
- 检查 `src/runtime/adapters/dds/`、`src/message/dds.py`、TARE DDS 入口，确保缺 Python DDS 时不会影响 `nav` 主路径。
- 更新测试，锁定“缺 Python DDS 不 crash-loop、主路径仍可启动/构建”的行为。
- 文档和配置中统一表述：Python DDS nav 已删除，现场主路径是 C++ DDS。

验收：

```bash
python -m pytest src/runtime/tests/test_dds_endpoint_runner.py src/runtime/tests/test_dds_typed_wire_adapter.py -q
python -m pytest tests/contracts/test_module_first_runtime_boundaries.py tests/contracts/test_architecture_layer_manifest.py -q
rg -n "cyclonedds-python|from cyclonedds|import cyclonedds" src scripts config docs
```

## P1 执行计划：补齐 C++ DDS 导航闭环

目标：C++ endpoint 能独立完成 no-motion 导航闭环。

- `nav_native_endpoint.cpp`：补齐输入 topic、TTL、计数、状态 JSON、零速 fail-closed。
- `lingtu_slam.idl` / `dds_topics.hpp`：对齐导航 topic 类型、名称和 schema 版本。
- `nav_loop.cpp` / `nav_loop.hpp`：把 planner/local follower 失败转成零速和可审计原因。
- `CMakeLists.txt`：确保 endpoint、traversability publisher、IDL 生成物一起构建。
- `lingtu-nav-dds.service`：默认 `LINGTU_NAV_PUBLISH_CMD_VEL=0`。

验收：

```bash
cmake -S src/nav/services/endpoint/cpp -B src/nav/services/endpoint/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build src/nav/services/endpoint/cpp/build -j
cmake -S src/nav/services/plan/cpp -B src/nav/services/plan/cpp/build -DCMAKE_BUILD_TYPE=Release -DLINGTU_NAV_PLAN_CPP_BUILD_TESTS=ON
cmake --build src/nav/services/plan/cpp/build -j
src/nav/services/plan/cpp/build/test_nav_loop
LINGTU_NAV_PUBLISH_CMD_VEL=0 src/nav/services/endpoint/cpp/build/lingtu_nav_native_endpoint --help
```

## P2 执行计划：现场联调和放开速度出口

目标：先证明链路可观测、可停止、可回滚，再放开运动。

- 在 S100P 上同时启动 Livox DDS、SLAM DDS、traversability DDS、nav DDS。
- 读取 `/tmp/lingtu_nav_native_status.json`，确认输入计数递增、无 TTL 过期、`cmd_vel_published=0`。
- 用固定地图和固定目标跑 no-motion：确认 global path、local path、follower 状态都有输出。
- 确认 Gateway 只读状态，不参与实时闭环。
- 人工确认安全区后再显式设置 `LINGTU_NAV_PUBLISH_CMD_VEL=1`。
- 放开速度后验证 CmdVelMux、安全层、Thunder sink 全链路仍在位。

验收：

```bash
systemctl status lingtu-nav-dds.service lingtu-traversability-dds.service --no-pager
journalctl -u lingtu-nav-dds.service -n 80 --no-pager
cat /tmp/lingtu_nav_native_status.json
python lingtu.py --list
python -m pytest src/runtime/tests/test_thunder_deployment_entrypoints.py src/runtime/tests/test_runtime_binding_policy.py -q
```

## 风险和边界

- 不直接删除开发机测试仍在用的 Python DDS adapter；先把它降级为非主链路。
- `cmd_vel_published>0` 不是完成标准；必须证明安全层和 Thunder 控制 sink 没被绕过。
- Gateway 不是实时控制通道，只能做 API、状态、审计和人工入口。
- Windows 本地只能做文档、Python 测试和部分 CMake 检查；CycloneDDS/idlc、systemd、Livox 真实链路必须在 S100P 或等价 Linux 环境验收。
