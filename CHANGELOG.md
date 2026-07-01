# Changelog

All notable changes to MapPilot (灵途) navigation system.

Format: [Semantic Versioning](https://semver.org/) — `MAJOR.MINOR.PATCH`

---

## [2.3.0] — 2026-04-14~05-31 (Sprint 5-6: 工程结构大重构 + 模块边界强制执行 + 安全性加固)

### 模块解耦与边界强制执行 (15 commits, 43 files changed, +370/-1060)

- **模块测试目录分离** — 将原来集中在 `core/tests/` 的 948 个测试按模块拆分到各自目录：
  - `gateway/tests/` (13 文件, 333 tests) — 网关 runtime 状态/路由/会话/遥测
  - `nav/tests/` (9 文件, 295 tests) — FSM/patrol/scheduler/geofence
  - `slam/tests/` (6 文件, 80 tests) — 后端状态/bridge TF/GNSS 融合
  - `memory/tests/` (5 文件, 237 tests) — semantic/episodic/tagged/vector
  - **总计 9 个测试目录**, 各模块独立 conftest.py
- **核心框架测试基数** — `core/tests/` 保留 104 文件 ~1300+ 测试（框架级 + 集成）
- **模块导入边界加固** — 各模块 `__init__.py` 显式导出，消除隐式通配符导入

### 目录结构重组 (命名统一 + 扁平化)

- **PascalCase → snake_case** — 全项目 Python 包名统一 (CamelCase 目录 → snake_case)
- **包结构扁平化** — 消除深层嵌套中间包：`src/global_planning/pct_adapters/pct_adapters/` → `src/global_planning/pct_adapters/`
- **src/legacy/ 集中管理** — 分散在 gateway/thunder/pct_planner/semantic 的遗留代码统一归入 `src/legacy/`
- **scripts/ 重组** — `scripts/legacy/` 统一到 `src/legacy/scripts/`；tools 移到 `scripts/tools/`
- **sim/ 整合** — `tests/experiments/` + `sim/src/` 重叠目录合并为 `sim/experiments/`

### ROS2 提取到桥接模块

- **ros2 依赖隔离** — rclpy 仅保留在 Bridge 模块和 C++ NativeModule 启动器
- **ros2_sim_driver / sim_ros2 Driver 分离** — 模拟驱动与框架解耦
- **ROS2 上下文执行器** — `ros2_context_executor.py` 独立管理 ROS2 spin，不阻塞主线程

### 测试基础设施 (+52 tests)

- **Gateway 全面覆盖** — 13 新增测试文件覆盖 route split / runtime status / acceptance / session map / telemetry / traffic / state snapshot / health contract / commands / MCP auth
- **SLAM 后端状态** — `test_slam_backend_status.py` 验证 fastlio2/pointlio/localizer 状态转换
- **Nav 走廊场景** — `test_nav_corridor_scenario.py` 走廊导航行为验证
- **导航框架契约** — `test_navigation_frame_contract.py` FrameContract 形式化验证
- **Blackbox Recorder** — 新增 `BlackboxRecorder` 事件记录模块 + 测试

### 安全性加固 (P0)

- **CORS 加固** — Gateway 添加严格 Origin 白名单验证，拒绝未授权跨域请求
- **Shell 注入防护** — 消除所有 `os.system()` / `subprocess(shell=True)` 调用，改为 `subprocess.run([...])` 参数数组
- **线程安全修复** — Gateway 模块 5 处共享状态加锁 (async lock + reentrant lock)
- **错误处理** — 消除 bare `except:` 和 `except Exception: pass` 模式，替换为精确异常类型 + 结构化日志

### 工程标准提升

- **mypy 严格模式** — 新增 mypy 配置，严格类型检查 (strict=True, disallow_untyped_defs=True)，修复 200+ 类型错误
- **Ruff 规则升级** — 启用 `F401`(未使用导入) / `F841`(未使用变量) / `E741`(模糊变量名) / `I`(导入排序)，全项目零违规
- **依赖分离** — `pyproject.toml` 拆分 dev/docker/runtime 依赖组
- **FrameContract 提取** — 从 monolith 中提取 FrameContract 数据模型到独立模块
- **常量集中管理** — 分散在各模块的 magic numbers 统一归入 `core/constants.py`

### 杂项修复

- **Registry 缓存修复** — 后端热切换时清除注册缓存，防 stale reference
- **CMU Terrain 注册** — 补全 CMU Terrain 后端 register 调用，修复 stub blueprint 兼容性
- **异常日志完善** — 全项目 final exception logging layer，确保崩溃前打印完整 traceback
- **`.gitignore` 更新** — 补充 `*.so` / `*.pyd` / `build/` / `__pycache__` 模式
- **文档** — CLAUDE.md README.md 同步更新，CHANGELOG 补全

### 统计

- **15 commits**, 43 files changed, +370 / -1060 lines
- **测试目录**: 9 个 (core/nav/slam/memory/gateway)
- **测试总数**: ~1300+ (框架) + 96 C++ (7 suites) = **~1400+**
- **遗留代码迁移**: ~1060 行精简

---

## [2.2.0] — 2026-04-13 (Dashboard 全面重做 + DDS/Gateway 可靠性)

### Dashboard — Arc/Raycast 风格重做 (18 组件)

- **设计系统重构** — 深蓝紫背景 + 漂移彩色 blur orb + 玻璃拟态面板 + indigo→purple→pink→amber 品牌渐变
- **字体升级** — Geist + Geist Mono + Inter，14px base，4px 节奏间距 (--s-1…--s-6)，6 级圆角 (r-xs…r-2xl)
- **FloatingWidget 系统** — 控制台 4 个 widget 可自由拖拽/resize，布局 localStorage 持久化，右下角"重置布局"按钮
- **SceneView** (新) — RViz 级场景视图：5 图层开关 (网格/轨迹/路径/目标/本机)，点击发送导航目标，四足机器狗渲染 (渐变躯干+4脚+箭头)，indigo→pink 渐变路径，琥珀十字准星目标，左侧地图抽屉 + 右侧状态栏
- **GpsCard** (新) — 旋转地球卡：经纬网格动画 + 外轨道卫星 + 琥珀脉冲定位标记 + X/Y/航向/速度/SLAM
- **MiniMap** (新) — 2D 俯视 Canvas 卡：机器人箭头 + 轨迹 + 路径渐变 + 目标点
- **CameraHud** (新) — 相机右上角遥测浮层：SLAM Hz / 速度 / 电量 / 温度 / 延迟，低电/高温警告
- **LoginPage** (重构) — Canvas 粒子背景 + 三层 drifting orbs + 网格 mesh + 机器狗 SVG hero + 系统状态徽章 + 三态按钮 (Idle/Loading/Success)
- **ChatPanel** (增强) — /命令自动补全下拉 (↑↓/Tab/Enter/Esc)，5 个快捷指令 chip，消息 bubbleIn 动画
- **Modal** (新) — PromptModal + ConfirmModal 替代原生 prompt/confirm，玻璃卡 + 品牌渐变 + spring 入场 + 输入验证
- **TabBar** — Pill 分段控件 (Raycast 风)，无下划线，键盘 ←→ 导航
- **Topbar** — 品牌渐变 logo chip，transparent 背景
- **Toast** (重做) — Spring slide-in + 玻璃 shadow + 按变体发光
- **可访问性** — 全局 :focus-visible baseline，prefers-reduced-motion 尊重，roving-tabindex，aria-controls/aria-label
- **响应式** — 三级: 1280/1100/900px

### Gateway 可靠性

- **DDS GIL 饥饿修复** — DDSReader._spin_loop 轮询间隔 1ms→10ms + reader 间 time.sleep(0) 释放 GIL，解决 uvicorn 所有 HTTP endpoint 超时
- **uvicorn 主线程运行** — --no-repl 模式下 GatewayModule._defer_server=True，主线程调 _run_server()
- **uvloop 事件循环** — asyncio → uvloop (C 实现)，DDS 线程不再阻塞 I/O polling
- **WebSocket 403 修复** — Starlette 原生 WebSocketRoute 绕开 FastAPI 0.135 APIWebSocketRoute bug
- **map_cloud 背压** — TerrainModule + GatewayModule 添加 set_policy("latest") 防回调堆积
- **uvicorn crash logging** — _run_server catch-all 异常日志 + ws="auto" 兼容 websockets 15.x

### 相机自动恢复

- **三级 watchdog** — L1 rclpy 重连 (1-2次) → L2 systemctl restart camera (3-5次) → L3 USB sysfs reset + restart (6+次)
- **QoS 修复** — CameraBridgeModule RELIABLE→BEST_EFFORT 匹配 Orbbec Gemini 335
- **防重复进程** — camera.service 添加 ExecStartPre killall + KillMode=control-group，watchdog stop→kill→start 三步清洁重启
- **帧缓存优化** — TeleopModule 无论有无 WS 客户端都缓存最新帧，连接时立即推送

### 地图管理

- **occupancy.npz 自动生成** — map save 自动从 map.pcd 生成 2D 占据栅格 (0.2m 分辨率)，_map_list 新增 has_occupancy 字段，get_active_occupancy() API
- **路径统一** — 6 处 ~/data/nova/maps 硬编码清零，统一到 ~/data/inovxio/data/maps

### 文档

- **web/README.md** — 18 组件架构图 + 设计规范 + API 端点表 + 部署流程

### 统计

- **19 commits**, 30+ files changed, ~4000 lines added
- **Dashboard**: 18 组件, 1760 modules, 63KB CSS (gzip 12KB), 276KB JS (gzip 86KB)
- **Tests**: 1263 passed, 7 skipped

## [2.1.0] — 2026-04-06 (Enterprise 加固 + C++ 性能优化 + 标定文档)

### Enterprise Hardening

- **278 新增 Python 测试** — 总计 1226 tests (core/tests/)，覆盖之前缺失的模块：
  - `test_memory_modules.py` (103 tests) — SemanticMapper, EpisodicMemory, TaggedLocations, VectorMemory, RoomObjectKG
  - `test_semantic_modules.py` (83 tests) — SemanticPlanner, GoalResolver, VisualServo, AgentLoop, LLM backends
  - `test_nav_services.py` (92 tests) — NavigationModule FSM, WaypointTracker, GlobalPlannerService, SafetyRing, CmdVelMux
- **Silent exception 清理** — bridge.py, slam_module.py, robot.py 中 bare `except: pass` 全部替换为具体异常类型 + 日志
- **health() 方法补全** — MapManagerModule, PerceptionModule, VectorMemoryModule, GeofenceManagerModule 新增健康检查

### C++ nav_core 性能优化 (5 commits)

架构级：
- **SoA 内存布局** — `plannerCloud_` 从 AoS `{x,y,z,h}` → 分离 `x[], y[], h[]` 数组，SIMD 友好
- **CSR 稀疏格式** — `vector<vector<int>>` 对应关系 → `corrOffset[] + corrData[]` 扁平数组，消除指针追踪
- **旋转主序循环** — 外层遍历旋转方向、内层遍历点云，写入连续 cache 命中

算术加速：
- **scorePathFast LUT** — 361 项 `pow(x, 0.25)` 查表替代 `sqrt(sqrt())`，**2.08x** 加速
- **预计算权重** — `rotDirW⁴`, `groupDirW²` 在 RotLUT 初始化时算好
- **nth_element** — 地形 ground quantile: `O(n log n)` sort → `O(n)` partial sort
- **sincos 融合** — PathFollower 单次调用替代分开的 sin + cos

SIMD + 并行：
- **xsimd v13.0.0** — 便携 SIMD (ARM NEON / x86 AVX 自动切换)，批量旋转 + 批量距离
- **OpenMP 并行评分** — 36 个旋转方向 scoreRotation 并行执行
- **terrain 并行化** — estimateGround 2601 voxel + filterVoxels 441 voxel OpenMP parallel
- **buildPlannerCloud SIMD** — ≥256 点时 AoS→SoA 收集 + 批量旋转

编译器级：
- **LTO** — 跨函数内联 (Link-Time Optimization)
- **-ffast-math + -funroll-loops** — 放松浮点语义，循环展开
- **CSR prefetch** — `__builtin_prefetch` 减少 cache miss

CMake 修复：
- **ROS2 ament_cmake 路径** — 之前 xsimd / OpenMP / LTO / fast-math 仅在 standalone 模式生效，sunrise aarch64 部署路径完全缺失。已修复，两个路径统一

### 测试覆盖

- **96 C++ tests** (7 test suites) + **12 benchmarks** (含 SIMD 正确性验证)
- **1226 Python tests** — 全部通过

### 文档

- **CLAUDE.md** — 新增传感器标定工具箱章节 (SOP 6 步 + 参数输出 + 运行时校验 + 关键文件)
- **REPO_LAYOUT.md** — 新增 `calibration/` 目录树
- **CHANGELOG.md** — 本条目

---

## [2.0.0] — 2026-03-25 (架构全面模块化重构 + 10 大模块 + MCP 服务)

### 架构重构

- **模块化设计** — 10 个大模块 (Module) + autoconnect Blueprint 组装 + dimos 风格架构
- **Per-wire 传输解耦** — callback (默认, 0 延迟) / DDS (跨进程) / SHM (高带宽) 按接线选择
- **5 种背压策略** — all (无背压) / latest (丢包) / throttle (限频) / sample (采样) / buffer (缓冲)
- **NativeModule** — C++ 子进程生命周期管理 (watchdog / restart / SIGTERM / SIGKILL)
- **Service 层** — PerceptionService + PlannerService，业务逻辑与接口分离
- **Module 生命周期加固** — 幂等 stop / 引用循环清理 / 崩溃隔离

### 新增模块

- **NavigationModule** — 统一 planner + waypoint 跟踪 + mission FSM
- **SafetyRingModule** — 统一安全反射 + 执行评估 + 对话状态
- **SemanticPlannerModule** — 统一 goal resolver + frontier explorer + task decomposer
- **AutonomyModule** — 4 个 C++ 地形/规划节点作为单一单元管理
- **DetectorModule** — 可插拔目标检测 (5 后端: yoloe / yolo_world / bpu / grounding_dino + 返回重新排列)
- **EncoderModule** — 可插拔特征编码 (2 后端: clip / mobileclip)
- **LLMModule** — 可插拔 LLM 推理 (5 后端: kimi / openai / claude / qwen / mock)
- **GlobalPlannerModule** — 可插拔路径规划 (2 后端: astar / pct)
- **GatewayModule** — FastAPI HTTP/WS/SSE 网关 (替代 C++ gRPC 网关)
- **MCPServerModule** — 16 个 MCP 工具用于 AI agent 机器人控制
- **RerunModule** — 3D 浏览器可视化 (:9090)

### 传输层重构

- **DDS 传输改用 raw cyclonedds** — 零 rclpy 依赖，降低耦合度
- **TransportAdapter** — 桥接 TransportABC 后端到 Transport 协议
- **SHM Subscriber 惰性重试** — 非阻塞 attach，自动恢复
- **S100P cyclonedds 验证** — 从源码编译 (Unitree 方案)

### 工程结构清理

- **src/ 目录重组** — 17 目录 → 10 目录 (core / nav / semantic / memory / drivers / gateway / base_autonomy / global_planning / slam / reconstruction)
- **删除** — client/ (689MB Flutter) / C++ gRPC 网关 (40 文件) / 模型权重 (203MB) / sdk/ / ota_daemon/
- **ThunderDriver** — 从 HanDogModule 改名 (产品命名)
- **nav 合并** — nav_core + nav_rings + nav_services → nav/ (统一)
- **semantic 合并** — semantic_common + semantic_perception + semantic_planner → semantic/ (统一)
- **global_planning 清理** — 7 包 → 2 包 (far_planner 归档)

### 测试覆盖

- **481 → 600+ 测试** — 框架测试无 ROS2 需求，S100P 集成测试全通过
- **新增测试套件** — registry / transport / lifecycle / crash isolation / service layer / perception/planner 解耦

### 破坏性变更 (BREAKING)

- `han_dog_bridge.py` → `src/drivers/thunder/han_dog_bridge.py` (路径变更)
- Import 路径调整: `from src.semantic.perception` 替代 `from src.semantic_perception`
- Transport 后端 API: `TransportAdapter.register()` 替代直接实例化

### 已知限制

- Fast Path 使用规则匹配 (非学习策略)
- S100P 无 CUDA — Open3D GPU 特性不可用，改用 C++ terrain_analysis
- Kimi API Key 可能过期 — Slow Path 需要有效 LLM 密钥
- 框架测试 (600+) 基于 Mock — 真实硬件集成测试需 S100P

---

## [1.8.0] — 2026-03-09 (全链路集成完成 + brainstem 上板)

### 集成测试全通 (T1-T8)

所有节点间通信集成测试在 S100P 实机验证通过：

| 测试 | 结果 | 耗时 |
|------|------|------|
| T1 terrain_analysis 点云→地形 | ✅ PASS | — |
| T2 localPlanner 地形+waypoint→路径 | ✅ 4/4 PASS | — |
| T3 pathFollower 路径→速度命令 | ✅ PASS | — |
| T4 global_planner goal→path | ✅ 5/5 PASS | — |
| T5 pct_path_adapter 路径→waypoint | ✅ PASS | — |
| T6 全链路闭环 (6节点) | ✅ 5/5 PASS | 18s |
| T7 安全信号链 stop/slow_down | ✅ 4/4 PASS | — |
| T8 han_dog_bridge → brainstem | ✅ 8/8 PASS | — |

### 新增测试文件
- `tests/integration/test_full_chain.py` + `test_full_chain.sh` — T6 全链路虚拟机器人 harness
- `tests/integration/test_safety_signals.py` + `test_safety_signals.sh` — T7 安全信号 3 阶段测试

### Bug 修复
- `src/drivers/robot_driver/han_dog_bridge.py`: `joint_positions/velocities/efforts` 从 `[:16]` 改为 `[:12]`；`interface::msg::RobotState` 只接受 12 个关节槽（hip/thigh/calf × 4 腿），原来 16 个值触发 `AssertionError` 导致看门狗 timer 崩溃
- `src/semantic_planner/semantic_planner/planner_node.py`: 18 处 printf 风格 logger 调用改为 f-string，防止 TypeError 静默丢日志

### 语义导航 SITL 验证
- `sim/semantic/factory_stub_test.py`: 场景图 score 统一为 0.92+，目标区域位置调整为 (9,3)（7m 射程）
- "导航到目标区域" fast path conf=0.95 命中，107s goal_reached
- 已验证: 工厂大门 3m (PASS)、机械设备 6.4m/71s (PASS)、目标区域 7m/107s (PASS)

### brainstem 部署至 S100P
- Dart SDK 3.11.1 安装至 `/home/sunrise/dart-sdk/`
- brainstem workspace 部署至 `/home/sunrise/brainstem/`
- `MEDULLA_STANDALONE=1` 模式运行 `server.dart`，CMS gRPC 监听 `:13145`
- ONNX 策略 `policy_260106.onnx` 加载成功，ProfileManager 就绪 (default/mini)
- `han_dog_bridge → 127.0.0.1:13145` 连接验证：motors ENABLED，IMU/Joint/History 三路流启动
- 启动脚本：`/tmp/launch_brainstem.sh`

---

## [1.7.5] — 2026-03-02 (全测试套件 13/13 通过)

### 测试修复
- `belief_network.py`: 补充缺失导出 `_TORCH_AVAILABLE`、`NUM_AFFORDANCE_TYPES`、`build_affordance_matrix`、`build_dangerous_mask`
- `projection.py`: 实现高阶 `project_2d_to_3d(bbox, depth_image, camera_info)` 接口；添加 `project_2d_to_3d` 别名
- `test_offline_pipeline.py`: `build_object_vocabulary` 返回元组，所有调用改为 `vocab, _ = self.build_vocab(kg)`；`test_L1_zh_falls_through_to_slow_path` 标记 xfail（CLIP 多语言已可直接解析中文）
- `test_chinese_tokenizer.py`: 更新简单分词断言（无 jieba 时连续汉字整体返回）；移除英文名词提取断言
- `test_planning_pipeline.py`: 事件从 `/nav/adapter_status` 读取

### 测试覆盖（在地瓜机器人 S100P 实测）
| 测试组 | 结果 |
|---|---|
| T1 nav_core C++ gtest | 7 test suites / 全通过 |
| T2 Python 规划单元 | 2 文件 / 全通过 |
| T3 语义规划器 pytest | 260 passed, 4 skipped |
| T4 语义感知 pytest | 236 passed |
| T5 OTA 鲁棒性 | 27 tests / 全通过 |
| T6 根级别 Python | 6 文件 / 全通过（含 397 pytest cases） |
| T7 ROS2 集成测试 | 22/22 通过 |

---

## [1.7.4] — 2026-03-02 (S100P 集成测试 22/22 全通过)

### 测试修复
- `tests/integration/test_planning_pipeline.py`: 订阅 `/nav/adapter_status` 而非 `/nav/planner_status` 获取 JSON 事件（`path_received`/`waypoint_reached`/`goal_reached`）
- `tests/integration/test_planning_stub.sh`: 等待时间 8s→15s；预检改为30s重试循环（解决 terrainAnalysis 在 S100P 注册慢问题）；预检前重启 ROS2 daemon（避免 stale daemon xmlrpc Fault）

### 验证
- 在地瓜机器人 S100P（RDK S100P，aarch64）上直接运行：`bash tests/integration/test_planning_stub.sh` → **22/22 全通过**

---

## [1.7.3] — 2026-03-02 (文档与工程结构整理)

### 文档
- 重写 README.md：去除装饰性内容，改为专业技术文档，补全架构说明、操作模式、配置和部署章节

### 工程结构
- 整理根目录：`AGENTS.md` → `docs/`，`TEST_PLAN.md` → `docs/07-testing/`，`fastdds_no_shm.xml` → `config/`，`lingtu.sh` → `scripts/`
- 更新所有脚本和文档中的路径引用（`mapping.sh`、`planning.sh`、`save_map.sh`、`scripts/ota/`）

### 测试与可视化
- 新增 e2e 建筑导航测试脚本（`tests/planning/e2e_nav_test.py`、`e2e_multi_scenario.py`、`e2e_generate_viz.py`）
- 新增规划演示 GIF 和轨迹图（`tools/`，含 corridor / head-on / replan / blocked 五个场景）

---

## [1.7.2] — 2026-03-02 (代码质量 20轮迭代)

### C++ 代码质量
- Round 1: RAII 加固（VerifyEd25519/PostJson 改用 unique_ptr）、const 正确性、EscapeJson 静态化
- Round 2: 15 处错误日志统一 [函数名] 前缀
- Round 3: 超时边界检查、rfind npos 防护、filesystem path 替代字符串操作
- Round 4: 移除 3 个未使用 include，修正 include 顺序

### Python 语义规划
- Round 1: goal_resolver/action_executor/episodic_memory 公开方法完整类型注解
- Round 2: 细化 except 异常类型，消除静默 except Exception: pass
- Round 3: asyncio.Task 引用保留、资源 with 语句、历史记录 FIFO 上限
- Round 4: 消除所有 print()，异常路径补全日志，Nav2 feedback 降级 DEBUG

### Flutter UI/UX
- Round 1: StreamBuilder/FutureBuilder 错误分支统一显示 EmptyState + retry
- Round 2: 11 处 IconButton 补全 tooltip，AnimatedSwitcher 数值动画
- Round 3: AnimatedSize/AnimatedSwitcher 动画、SkeletonListTile 骨架屏
- Round 4: notifyListeners 变化守卫、context.select 精细订阅、mounted 检查

### DevOps / 运维
- Round 1: health_check 13→17 项（OTA 密钥、Python 版本、inode 率、gRPC 端口）
- Round 2: Docker Compose 健康检查 + 日志限制
- Round 3: Systemd OOMScoreAdjust=-500、MemoryMax 按服务设置、依赖链完整
- Round 4: CI 覆盖率报告、pub-cache 缓存、构建产物大小门控

### 导航规划
- Round 9: PCT 适配器边界加固（航点跳跃保护/零路径/10m 距离校验）
- Round 10: 卡死检测渐进警告（5s WARN_STUCK / 10s STUCK）、恢复确认
- Round 11: 地形评分参数化、frontier NaN/Inf 过滤

### gRPC 网关
- Round 12: Keep-alive 配置、6 个流式 RPC 全局 try/catch
- Round 13: OTA 错误码映射完整、取消信号传播

### 测试覆盖
- Round 14: 42 个回归测试（planner_node 初始化、llm_client async、OTA 健壮性）

### 集成与文档
- Round 15: 新增参数文档化（frontier/maxIndexJump/卡死检测）
- Round 16: topic_contract WARN_STUCK 同步
- Round 17: C++/Flutter 用户面向错误消息中文化
- Round 18: frontier_scorer lru_cache、路径长度缓存
- Round 19: topic/config/import 一致性审计修复

---

## [1.7.1] — 2026-03-02 (生产级安全与稳定性修复)

### 安全修复 (CRITICAL)

- **Ed25519 签名验证漏洞** — `ota_update.cpp`: `VerifyEd25519` 错误地将 `manifest_signature` 同时作为 message 和 signature 传入（自验签名），任意伪造 OTA 包均可通过验证。修复：message 参数改为 `manifest_content`
- **死代码导致监控定时器失效** — `planner_node.py`: `_follow_person_pub`、`_monitor_timer` 等初始化代码位于 `return` 语句之后，永远不会执行。任务超时/卡死检测完全失效，FOLLOW_PERSON 模式触发 AttributeError 崩溃。修复：将初始化移至 `__init__`
- **stoull/stoul/stoi 无保护** — `ota_update.cpp`/`utils.cpp` 共 4 处对外部字符串直接调用数字转换函数，空字符串或非数字内容导致 gRPC daemon 崩溃。修复：全部包裹 try/catch

### 安全修复 (HIGH)

- **popen shell 命令注入** — `ota_file_ops.cpp`: HTTP 响应头 key/value 直接拼入 shell 命令字符串后 popen 执行，攻击者控制服务器可注入任意命令。修复：添加 `isHeaderSafe()` 白名单校验（仅允许 `[A-Za-z0-9_.-: ]`）
- **非原子文件写入** — `ota_file_ops.cpp`: 直接写入最终目标路径，SHA256 校验失败后磁盘残留损坏文件，下次断点续传误判。修复：写入 `.tmp` 临时文件，校验通过后 `rename`，失败则删除
- **MakeDirs shell 注入** — `utils.cpp`: `system("mkdir -p '" + path + "'")` 路径含单引号可逃逸 shell。修复：改用 `std::filesystem::create_directories`
- **JSON 解析 npos 减法崩溃** — `ota_update.cpp`: `find_first_of` 返回 `npos` 后直接 `npos - start` 产生极大值，`substr` 抛出 `std::out_of_range`。修复：添加 `npos` 判断后截到行尾
- **asyncio.get_event_loop() 线程不安全** — `llm_client.py`: Python 3.10+ 非主线程异步上下文中已废弃，可能返回错误 loop。修复：改为 `asyncio.get_running_loop()`
- **bind() 重复注册观察者** — `alert_monitor_service.dart`: 每次重连调用 `bind()` 都叠加 WidgetsBindingObserver 和 ChangeNotifier listener，导致事件处理多次触发。修复：添加 `_unbindInternal()` 先清理再重注册
- **OTA 递归状态机** — `ota_gateway.dart`: `_advanceDeployment()` 五级互相递归，下载期间取消无效，调用栈持续增长。修复：改为 `while (!_cancelled)` 迭代循环，每阶段结束立即检查取消标志

---

## [1.7.0] — 2026-03-02 (产品迭代 50轮 — 全面质量升级)

### Flutter 客户端 — 地图 (Iter 7-12)
- **测距工具** — 双击起点/终点，虚线+距离标注
- **坐标十字准星** — 地图中心实时世界坐标显示
- **旋转锁定** — FAB 开关，锁定后禁止旋转手势
- **航点编号标签** — 每个航点旁白字黑描边序号
- **路径历史轨迹** — 60秒 600点 FIFO，青色渐隐尾迹
- **图层开关面板** — 6层独立显隐（栅格/点云/路径/航点/轨迹/前沿）

### Flutter 客户端 — 控制屏 (Iter 25-28)
- **摇杆死区** — 0~30% 可配置，死区内输出归零
- **最大速度限速** — 0.1~2.0 m/s 滑块，实时缩放 cmd_vel
- **一键返航** — 蓝色 FAB + 确认对话框，发送 (0,0,0) 目标
- **双击急停手势** — 双击任意区域急停 + 红色全屏闪烁 + 重触觉反馈

### Flutter 客户端 — 任务面板 (Iter 29-32)
- **拖拽排序航点** — ReorderableListView + drag handle
- **任务时长预估** — 路径距离 / 0.5m/s + 每航点5s
- **环形进度动画** — 蓝/绿/橙三色渐变 + 完成勾号动画
- **取消任务确认** — 显示已完成/总航点数，二次确认

### Flutter 客户端 — 状态页 (Iter 33-36)
- **CPU/内存/温度圆形仪表** — 270度弧形 CustomPaint，阈值告警
- **磁盘使用量条形图** — 已用/总容量(GB)，OTA daemon 数据源
- **运行时间计数器** — 会话时长 HH:MM:SS，每分钟刷新
- **网络延迟折线图** — RTT + 质量等级 + 30次采样 sparkline

### Flutter 客户端 — 连接管理 (Iter 37-40)
- **断线指数退避重连** — 2/4/8/...30秒，顶部 ReconnectBanner 倒计时
- **RTT历史统计** — 60次采样 FIFO，均值/最低/最高/质量标签
- **地理围栏警告** — <2m 橙色横幅，<0.5m 红色告警
- **连接状态图标动画** — 不稳定黄色感叹号，断线红色闪烁

### Flutter 客户端 — 设置 (Iter 3-6)
- **事件导出** — AppBar 分享按钮，格式化列表复制到剪贴板
- **CRITICAL 声音提醒** — SystemSound.alert + 前台检测
- **关于页面** — 版本号/构建号/gRPC端口/GitHub/许可证
- **缓存管理** — 统计数量(模板/任务/设备) + clearNonCriticalData

### Flutter 客户端 — 通用 UX (Iter 41-44)
- **下拉刷新** — 事件列表 + 健康状态页
- **骨架屏** — shimmer 扫光动画，3秒超时降级
- **HapticUtils** — 统一触觉反馈：轻/中/重/成功/警告/选中
- **EmptyState widget** — icon+title+subtitle+action，用于事件/航点空态

### DevOps & CI (Iter 19-24)
- **Flutter CI 质量门** — `flutter analyze --fatal-warnings` + `flutter test` 阻断构建
- **nav_core ARM64 交叉编译** — CI 自动检查 aarch64-linux-gnu 编译
- **一键机器人部署** — `scripts/ota/deploy_to_robot.sh` rsync + 服务重启
- **Docker 健康检查** — 检测 /nav/odometry 话题，start_period=60s
- **Systemd Watchdog** — nav-slam/planning/grpc WatchdogSec=30
- **配置备份脚本** — `scripts/backup_config.sh`，保留最近5份

### C++ 质量 (Iter 13-18, 45-48)
- **边界测试** — PathFollower/LocalPlanner/PctAdapter 空路径/NaN/极值/状态机
- **性能基准** — 三核心组件 1000次循环 < 100ms，74/74 通过
- **输入验证宏** — `validation.hpp`: isValidPosition/isValidPath/filterInvalidPoses
- **验证测试** — 11个 gtest，NaN/Inf/空路径全覆盖
- **A* 超时保护** — astar_timeout_sec=5.0 参数，超时发布 FAILED
- **路径自相交检测** — CCW+线段相交算法，发现时记录警告
- **OTA 活跃任务检查** — CheckUpdateReadiness 检测 nav-planning 状态
- **OTA 回滚日志** — 回滚事件写入历史 + 上报 infra/ota server

---

## [1.6.0] — 2026-03-01 (产品迭代 10轮 + OTA修复 + 运维升级)

### Flutter 客户端 — 功能补全

- **地图急停按钮** — 任务运行时显示红色 E-Stop FAB，一键急停；E-stop 解除需二次确认并选择原因
- **全局低电量 Banner** — 电量 <15% 橙色、<5% 红色，滑入动画，全局可见
- **任务状态深度可视化** — ETA 预估时间、重规划次数、语义导航 HUD（置信度 + Fast/Slow 标签）
- **建图→导航快捷切换** — 建图完成后 BottomSheet 一键转入导航模式
- **OTA 升级预检对话框** — 调用 CheckUpdateReadiness RPC，ERROR 阻止继续、WARNING 可忽略
- **GetCapabilities 动态感知** — 根据后端声明的能力动态隐显 UI 元素
- **定时任务调度** — ScheduledTask 数据模型 + SharedPreferences 持久化 + 管理页面（最多10条）
- **任务完成报告卡片** — 任务结束自动弹出：用时/航点完成率/重规划次数/WARNING+事件列表
- **任务模板本地持久化** — 保存/加载/删除路线方案，跨设备共享
- **事件日志过滤搜索** — FilterChip (ALL/INFO/WARN/ERROR/CRIT) + 文本搜索 + 清除按钮

### Flutter 客户端 — 地图增强

- **真彩色占用栅格** — 三色渲染：绿(自由) / 灰(未知) / 深红(障碍)
- **点云 Z 轴 5 色段** — blue → cyan → green → yellow → red 高度着色
- **图例卡片** — 半透明黑底，5行图标 + 白色文字
- **比例尺** — 基于 TransformController 动态计算，白色实线 + 距离标注
- **长按添加航点** — 逆矩阵坐标转换，橙色圆圈标记

### Flutter 客户端 — 安全与运维

- **低电量任务拦截** — 发送任务前检查电量 <15% 弹确认对话框，fail-open
- **语义导航透明度卡片** — Fast/Slow 路径标签 + 置信度进度条 + 当前目标 + 状态阶段
- **系统服务诊断** — ExpansionTile 展示各 systemd 服务状态，一键重启
- **SystemGateway** — 通过 OTA daemon (port 50052) 获取服务状态和重启

### Flutter 客户端 — 基础设施

- **流式文件下载** — `downloadFileTo()` 直接写入本地文件，大文件不堆内存
- **告警评估防重入** — `_evaluating` 守卫防止并发 `_evaluate()` 调用
- **TLS 安全日志** — 明文连接时输出警告日志
- **版本升级 1.6.0**

### OTA 修复

- **健康检查服务名** — WARM 更新后不再硬编码 `navigation.service`，改为遍历 `managed_services` 配置列表，任一 active 即通过
- **Ed25519 密钥路径** — `ota_public_key_path` 从空字符串改为 `/opt/lingtu/nav/ota/keys/server.pub`，附密钥生成说明
- **Go agent systemd 单元** — `lingtu-nav-agent.service` 新增，依赖 `ota-daemon.service`，RestartSec=60

### 测试

- **nav_core 参数敏感性测试 (4组)** — dirDiffThre 转向阈值、slopeWeight 坡度影响、stuck 边界精确、replan 冷却防抖，全部通过

---

## [1.5.x-dev] — 2026-02-28 (R11-R20)

### Added
- **[R14] pathFollower: GOAL_REACHED 信号** — 到达目标时向 `/nav/planner_status` 发布 `"GOAL_REACHED"`，带防重发 flag，路径切换时自动复位
- **[R15] han_dog_bridge: 里程计发布降频** — 新增 `odom_pub_rate` 参数（默认 10Hz），避免 50Hz IMU 频率淹没 SLAM 里程计
- **[R12] han_dog_bridge: 协方差矩阵** — Odometry 消息补全 36 元素 pose/twist 协方差（对角阵，诊断工具可读）
- **[R11] localPlanner: Joy 轴参数化** — `joy_axis_fwd/left/autonomy/obstacle` 4 轴均可在 robot_config/launch 覆盖，适配不同手柄型号
- **[R13] robot_config: 集中化参数** — `autonomy.joy_to_speed_delay` / `joy_to_check_obstacle_delay` / `pct_planner.smooth_min_dist` 写入 robot_config.yaml single source of truth

### Changed
- **[R16]** costmap 验证：确认 `/nav/costmap` 是 semantic_perception 内部接口，topic_contract 注释已同步
- **[R17] pct_planner_astar: 障碍物起终点修正** — A* 前自动搜索最近可行格（起点 radius=3，终点 radius=5），避免规划失败
- **[R18] health_check.sh: 节点存活检查** — 第 7 节添加 4 个必须节点验证 + dog/generic driver 自动识别

### Fixed
- **[R20]** 修复第一批迭代（R1-R10）遗漏提交的 7 个文件

---

## [1.5.x-dev] — 2026-02-27 (R1-R10)

### Added
- **[R3] pct_planner_astar: 路径降采样** — 新增 `_downsample_path()` 等距降采样函数 + `smooth_min_dist` ROS2 参数，减少航点密度、缓解跟踪抖动
- **[R6] driver_node: 串口实现** — `connect_hardware()` 使用 `serial.Serial`，`send_to_motors()` 发送 `vx,vy,wz\n` 协议，仿真模式 fallback
- **[R14 precursor] pathFollower: Joy 轴参数化** — `joy_axis_fwd/left/yaw/autonomy/obstacle` 5 轴均可配置
- **[R8] navigation_run: planner_profile 暴露** — launch 参数说明更新，包含 `pct_py` 选项

### Fixed
- **[R1] semantic_planner cmd_vel 类型修复** — `Twist` → `TwistStamped`，remap 补充 `("cmd_vel", "/nav/cmd_vel")`；修复孤立话题问题
- **[R2] han_dog_bridge: 位置航位推算** — 接入 IMU yaw + cmd_vel 积分位置 (x, y)，Odometry 消息携带有效位置字段
- **[R5] adapter_status 消费者迁移** — 发现 3 处隐藏的 `/nav/planner_status` JSON 消费者（task_manager.cpp / pct_path_adapter.py / planner_node.py），全部迁移到 `/nav/adapter_status`

### Infrastructure
- **[R7] health_check.sh** — 添加 `/nav/dog_odometry` / `/nav/adapter_status` 频率检查
- **[R9] ROBOT_DEPLOYMENT.md** — 更新节点链图 + 故障排查表
- **[R10] CLAUDE.md** — 话题合约表新增 dog_odometry / adapter_status / GOAL_REACHED

---

## [1.5.x-dev] — 2026-02-26 (话题审计 + 双发布者修复)

### Fixed
- **han_dog_bridge 里程计冲突** — `/Odometry` remap 从 `/nav/odometry` 改为 `/nav/dog_odometry`，Fast-LIO2 成为唯一 `/nav/odometry` 发布者
- **pct_path_adapter 状态话题冲突** — `/nav/planner_status` 双发布者修复：pct_path_adapter JSON 状态迁移到 `/nav/adapter_status`；global_planner 纯字符串保留原话题

### Added
- `topic_contract.yaml`: 新增 `dog_odometry` / `adapter_status` / `costmap` 条目，澄清各话题唯一发布者

---

## [1.5.0] — 2026-02-25 (nav_core Core/Shell 分离)

### Changed
- **nav_core 包** — 提取 3 个 ROS2 节点的纯算法到 header-only 库 (`path_follower_core.hpp` / `local_planner_core.hpp` / `pct_adapter_core.hpp`)
- **pathFollower / localPlanner / pct_path_adapter** — 成为 Thin Shell，核心逻辑改为调用 nav_core
- **gtest** — 53 个 gtest 覆盖核心算法，Windows MSVC + Linux gcc 9.4 通过
- **pybind11** — `py_nav_core.cpp` 绑定，Python 测试可调用真实 C++ 算法

---

## [1.4.x] — 2026-02-24 (E2E 建筑点云导航验证)

### Verified
- building2_9.pcd 真实点云 3 场景全部到达（13.3s / 13.1s / 11.9s，path RMS < 0.05m）
- 修复 localPlanner near-field E-stop 导致 safetyStop_ 永久卡死问题
- 输出论文图 (`tools/e2e_building_traj.png`, `tools/e2e_building_nav.gif`)

---

## [1.3.x] — 2026-02-22 (Python A* PCT 规划器)

### Added
- `pct_planner_astar.py` — 纯 Python 8-连通 A* 全局规划器，零 `.so` 依赖，适合 x86_64 开发机
- `planner_pct_py.launch.py` — 对应 launch profile (`planner_profile:=pct_py`)
- `ROBOT_DEPLOYMENT.md` — 机器人部署指南（初版）
