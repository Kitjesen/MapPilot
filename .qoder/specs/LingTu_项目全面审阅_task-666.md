# LingTu 项目全面审阅报告

## Context

LingTu (灵途) 是一个面向四足机器人的户外自主导航系统，运行在 S100P (RDK X5, aarch64) 平台上。本次审阅覆盖了项目的全部关键代码区域，包括核心框架、导航与安全、感知与语义、驱动与网关、C++ 后端、配置文件以及构建/部署脚本。

---

## 状态总览

所有修复（除 Task 12 MCP 速率限制已按用户要求跳过外）均已实施并通过验证。

- ✅ Task 1-11, 13-17: 已实施、已通过测试验证
- ⏭️ Task 12 (MCP 速率限制): 用户明确要求跳过

---

## 严重问题 (Critical — 必须修复)

### Task 1: `GnssSerialDriver.stop()` 未关闭串口 — 资源泄漏 + 死代码
**文件**: `src/slam/gnss_serial_driver.py`

- `stop()` 方法仅设置 `_running = False` 并 join 线程，**没有关闭 `self._serial`**（串口 fd 泄漏，重复 start/stop 后设备可能 `Device or resource busy`）
- `write_rtcm()` 中 L213-219 的清理代码（`self._thread = None` + `self._serial.close()`）位于 `return False` **之后**，是**永远不可达的死代码**

**修复**: 在 `stop()` 中添加串口关闭逻辑，删除 `write_rtcm()` 中的死代码。

### Task 2: `In._deliver` 共享可变状态缺乏线程同步保护
**文件**: `src/core/stream.py`

- `_deliver()` 方法在无锁状态下修改 `_msg_count`、`_latest`、`_last_ts`、`_rate_window_count`、`_drop_count` 等字段
- `_msg_count += 1` 是经典的"读-修改-写"模式，即使在 CPython GIL 下也非原子操作
- 多线程发布到同一 `In` 端口时统计指标会不准确

**修复**: 在 `_deliver` 入口处获取 `self._lock` 保护计数器更新段（不保护回调执行，避免性能损失）。

### Task 3: `_build_worker_mode` 本地模块生命周期缺乏错误隔离
**文件**: `src/core/blueprint.py`

- Worker 模式构建路径中，本地模块的 `setup()` / `start()` 直接裸调用，无 try/except 保护
- 对比 `SystemHandle.start()` 为每个模块提供了 try/except 隔离
- 任何模块 `setup()` 异常会导致：整个 `build()` 崩溃 + 已创建的 worker 子进程泄漏

**修复**: 为本地模块的 `setup()` / `start()` 添加与 `SystemHandle.start()` 一致的 try/except 隔离，构建失败时确保 `coord.shutdown()` 被调用。

---

## 警告级问题 (Warning — 建议修复)

### Task 4: `gateway_module.py` 中 `_voxel_downsample` 被重复定义两次
**文件**: `src/gateway/gateway_module.py`

- L1397: 包含 `len(pts) == 0` 空数组保护 + `np.sort(unique_idx)` 保序
- L1916: 无空数组保护 + 不排序
- Python 静默使用第二个定义，第一个定义是死代码；两个实现在行为上有差异

**修复**: 合并两个版本的优点（保留空数组保护 + 排序），删除重复定义。

### Task 5: Auth 登录 Cookie 缺少 `samesite` 属性
**文件**: `src/gateway/routes/auth.py`

- `/api/v1/auth/login` 设置 cookie 时未显式声明 `samesite` 参数
- 在 CORS origin 配置被放宽的场景下增加 CSRF 攻击面

**修复**: 添加 `samesite="lax"` 显式声明。

### Task 6: `Out.publish` 错误计数器在锁外更新
**文件**: `src/core/stream.py`

- 回调抛出异常时，`_publish_errors += 1` 在无锁状态下修改
- 速率限制日志的 `_last_error_log_ts` 存在竞态

**修复**: 使用原子操作或锁保护错误计数器更新。

### Task 7: `DeviceManager.stop()` 静默吞没设备关闭异常
**文件**: `src/core/devices/manager.py`

- `except Exception: pass` 完全吞没所有异常，无日志记录
- 硬件关闭失败可能意味着资源泄漏

**修复**: 添加 `logger.debug()` 记录关闭异常。

### Task 8: `angDiffDeg` 在角度差 > 540° 时返回错误结果
**文件**: `src/nav/core/include/nav_core/local_planner_core.hpp`

- 仅做单次 `d -= 360` 归约，当 `|a - b| > 540°` 时结果错误
- 在 `scoreAndSelect` 中 `rotAng` 可达 350°，`endDirPathList_` 可达 ±180°，组合后可触发此 bug
- 导致大旋转角度下的路径评分出现数值错误，可能选出次优路径

**修复**: 使用 `std::fmod` 替代单次减法：
```cpp
inline double angDiffDeg(double a, double b) {
  double d = std::fmod(std::fabs(a - b), 360.0);
  if (d > 180.0) d = 360.0 - d;
  return d;
}
```

### Task 9: `VoxelGridParams` 默认值与 `LocalPlannerCore` 内部硬编码不一致
**文件**: `src/nav/core/include/nav_core/local_planner_core.hpp` + `local_planner_full.hpp`

- 公开默认值 `gridVoxelOffsetY = 5.25`、`gridVoxelNumY = 531`
- 内部实际使用 `gridVoxelOffsetY = 4.5`、`gridVoxelNumY = 451`
- 外部调用者（可视化、调试）的体素索引与规划器内部不匹配

**修复**: 将 `VoxelGridParams` 默认值对齐为 `4.5` / `451`。

### Task 10: `downsamplePath` 总是追加终点导致重复航点
**文件**: `src/nav/core/include/nav_core/pct_adapter_core.hpp`

- 函数无条件 `result.push_back(input.back())`
- 当最后一个采样点已经是终点时，终点被重复添加
- 可能触发 `WaypointTracker` 提前到达目标检测

**修复**: 添加距离检查避免与已接受的最后一个点重复。

### Task 11: `WorkerSystemHandle` 缺少 `comm_health()` 方法
**文件**: `src/core/blueprint.py`

- `SystemHandle` 有 `comm_health()`，`WorkerSystemHandle` 没有
- 对 worker 模式系统句柄调用 `comm_health()` 会抛出 `AttributeError`

**修复**: 为 `WorkerSystemHandle` 添加对应的 `comm_health()` 实现。

---

## 改进建议 (Suggestion — 可考虑)

### Task 12: MCP Server 工具调用无速率限制
**文件**: `src/gateway/mcp_server.py` — 考虑添加 per-tool 速率限制

### Task 13: `_spawn_auto_relocalize` 使用字符串拼接构建 Shell 命令
**文件**: `src/gateway/gateway_module.py` — 建议改用环境变量传递路径

### Task 14: OTA 安装脚本 `$PACKAGE_DIR` 直接插入 Python 代码
**文件**: `scripts/ota/install_nav.sh` — 建议通过环境变量传递

### Task 15: `In.policy` 属性文档不完整
**文件**: `src/core/stream.py` — docstring 仅提及 2 种策略，实际支持 6 种

### Task 16: `_do_wire` 类型检查对 `typing.Any` 过于严格
**文件**: `src/core/blueprint.py` — `Any` 应作为通配符匹配

### Task 17: `_do_wire` 类型检查对 `typing.Any` 过于严格
**文件**: `src/core/blueprint.py` — `Any` 应作为通配符匹配

---

## 安全架构评估 (已通过)

- ✅ **依赖方向**: `nav/` 不导入 `semantic/`、`drivers/`、`gateway/` — 无违规
- ✅ **CmdVelMux**: 使用 `RLock` 保护仲裁逻辑，`_sanitize_twist` 过滤非有限值
- ✅ **SafetyRing**: 三环安全架构完备，odom/cmd_vel 超时检测、localization LOST→STOP 路径清晰
- ✅ **NavigationModule**: `_nav_lock` 保护 FSM 状态，recovery motion 有 stop_event 取消机制
- ✅ **GlobalPlannerService**: 7 阶段规划流程，`plan_safety_policy` 支持三种策略
- ✅ **认证**: HMAC 安全比较、日志脱敏、API Key 中间件覆盖 REST/MCP

---

## 验证方式

1. **框架测试**: `python -m pytest src/core/tests/ -q`
2. **C++ 测试**: 在 `src/nav/core/build/` 下运行 `test_local_planner_core`、`test_path_follower_core`
3. **手动验证**: 对 `GnssSerialDriver` 执行 start/stop 循环测试串口泄漏
4. **安全验证**: 检查 gateway cookie 设置和 MCP 端点行为
