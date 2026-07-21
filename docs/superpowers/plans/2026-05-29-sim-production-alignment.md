# Plan: sim ↔ 生产 全局规划链路对齐 (方向 C)

> **Historical plan status**: Restored from commit `df57d5b6`, the last clean UTF-8 version found in git history. Treat this as a 2026-05-29 historical planning record; re-verify all referenced paths and status claims before using it as current implementation guidance.

## 1. 目标与成功标准

**目标**：消除 sim 与生产 nav 在「全局规划层」的分叉，让两者走同一条
PCT 全局链路；`astar` 退化为「无 PCT native 时的显式 fallback」，而不是
sim 默认规划器。

**成功标准**：
- 任何 profile 选 `planner="pct"` 时：有 native → 跑 PCT；无 native（Windows/
  Mac/缺 .so）→ **自动、显式降级到 astar**（不再 RuntimeError 崩溃），并在
  status 里能查到 `active_planner` 与 `degraded_reason`。
- sim 家族在 Linux/CI 上跑的就是 PCT（与真机同构）；在 Windows 开发机上
  自动降级 astar（行为与今天等价，但语义显式）。
- `costmap` 门控行为不再 sim/真机分叉（sim 走 pct 后 `replan_on_costmap_update`
  自动与真机一致）。
- 契约文档 + 测试锁死新边界。

## 2. 范围边界

**做**：
- GlobalPlannerService 增加「主规划器不可用 → 显式降级 fallback」逻辑 + 状态暴露。
- sim 家族 profile 的 `planner` 由 `astar` 切到 `pct`（阶段 2）。
- 契约文档新增「sim↔生产对齐 + 降级语义」章节。
- 测试：降级单测、profile 快照更新、契约测试。

**不做**：
- 不改 PCT/A* 算法本身。
- 不改局部规划链（terrain→local_planner→path_follower，已对齐）。
- 不重编译 .so，不引入新依赖。

## 3. 现状与证据

- `cli/profiles_data.py`：sim/sim_gazebo/sim_industrial/sim_mujoco_live 均 `planner="astar"`；
  生产 nav/explore `planner="pct"`。
- `GlobalPlannerService.setup()` (`src/nav/global_planner_service.py:54-57`)：
  `_create_backend()` 后**无 available 检测**，PCT .so 缺失时 backend.available=False，
  plan() 返回 [] → 现状直接 RuntimeError，无自动降级。
- `_PCTBackend.available`：.so + tomogram 都加载成功才 True。
- `NavigationModule._on_costmap`：`update_map` 与重规划都在 `replan_on_costmap_update`
  守卫下；PCT 默认 False，A* 默认 True → costmap 行为分叉。
- tomogram `building2_9.pickle` 物理存在；PCT .so 有 aarch64 + x86_64 + x86_64_py312。

## 4. 实施步骤 (两阶段)

### 阶段 1 — 降级收口 (纯增强，无破坏，可独立验证)

1. `GlobalPlannerService.__init__`：新增 `_active_planner_name`、`_degraded_reason`。
2. `GlobalPlannerService.setup()`：创建主 backend 后检测可用性
   （`getattr(backend, "available", True)` —— 非 PCT backend 恒可用）。
   主不可用且 fallback 与主不同 → 切到 fallback backend，记录 degraded_reason，
   并**重算 map_artifact_gate**（active=astar 时 pct 的 tomogram gate 不再 block）。
3. 暴露状态：`active_planner` / `degraded_reason` 进 `last_plan_report` 与
   `NavigationModule` 的 mission/health status。
4. 契约文档 §4/§9 增补「PCT 不可用降级 astar」语义。
5. 测试：`test_planner_backends.py` 或新建，覆盖「PCT 不可用 → 降级 astar 可规划」
   + 「降级后 status 标注」。

### 阶段 2 — sim 切 PCT (行为变更，需 CI/真机回归，单独确认)

6. sim/sim_gazebo/sim_industrial `planner` astar→pct（tomogram 已配 building2_9）。
   sim_mujoco_live 无 tomogram：保留 astar 或补 tomogram（待定）。
7. 删除 sim 上「显式 astar」的隐含假设，依赖阶段 1 的自动降级在 Windows 上兜底。
8. 更新 `test_profile_graph_snapshots.py` 中 sim 的 planner 期望值。
9. CI（Linux）回归确认 sim e2e 在 PCT 下仍通过；不通过则定位是真问题还是
   测试基线需更新。

## 5. 涉及文件清单

- `src/nav/global_planner_service.py`（阶段1核心）
- `src/nav/navigation_module.py`（status 暴露 active_planner）
- `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md`（语义增补）
- `cli/profiles_data.py`（阶段2）
- `src/core/tests/test_planner_backends.py` / 新增降级测试（阶段1）
- `src/core/tests/test_profile_graph_snapshots.py`（阶段2）

## 6. 验收清单

- [ ] PCT 不可用时 GlobalPlannerService 降级 astar 而非崩溃
- [ ] status 暴露 active_planner + degraded_reason
- [ ] 降级后 map_artifact_gate 不误 block
- [ ] 契约文档更新降级语义
- [ ] 阶段1 测试通过 + 现有 plan_safety/profile 快照不回归
- [ ] (阶段2) sim planner=pct，快照测试更新，CI 通过

## 7. 风险与回滚

- **风险**：阶段2 在 Linux CI 上把 sim 从 astar 切 pct，可能触发 PCT 相关
  新行为/测试基线变化。
  **缓解**：阶段1先独立合入验证；阶段2单独提交，CI 红则单独回滚 profile 改动。
- **风险**：降级逻辑改变了「PCT 缺失」的失败模式（崩溃→降级运行）。
  **缓解**：用 status 显式标注 degraded，避免「静默以为在跑 PCT」。
- **回滚**：阶段1/阶段2 分提交，各自可独立 revert，不互相依赖。

## 8. 依赖与环境变更

- 无新增依赖。
- 本机 `uv run` 因 brainstem-api 要求 py≥3.13 解析失败，回归用 `python -m pytest`。
