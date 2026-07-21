# LingTu 全面优化计划 — 解耦 · 模块化 · 规范化

> **Historical / evidence status**: Restored as valid UTF-8 from commit `20c71e56`, the last clean version found in git history. The later `c6606872` copy is lossy mojibake/invalid UTF-8; treat this document as a 2026-05-31 historical audit snapshot unless each item is re-verified against the current codebase.
>
> **审计日期**：2026-05-31
> **审计方式**：5 个并行 Agent（导入边界 / 文件组织 / 测试覆盖 / 代码质量 / 架构模式）+ 综合 Synthesis Agent
> **前序计划**：[2026-05-30 解耦执行计划](./2026-05-30-module-decoupling-execution.md)（Tasks 1-2 已完成）

---

## 一、现状总评

### 🟢 健康部分

| 维度 | 评估 |
|------|------|
| **导入边界** | gateway → nav/semantic/drivers 零违规，边界测试通过 ✅ |
| **Core 框架** | module.py / stream.py / registry.py / blueprint.py 结构清晰，零循环引用 ✅ |
| **Module-First 规范** | 所有 Module 使用 In[T]/Out[T] 类型端口、@register、@skill 装饰器，模式统一 ✅ |
| **Shim 兼容层** | 3 个 `sys.modules` swap shim 正确工作，旧导入路径透明重定向 ✅ |
| **语义层隔离** | semantic/ → nav/drivers/gateway 零导入 ✅ |

### 🔴 需立即关注的（按严重度排序）

| # | 严重度 | 问题 | 位置 |
|---|--------|------|------|
| 1 | P0 | **`tests/` 目录被 pytest 排除** — 26 个 ROS2 集成测试对 `python -m pytest` 不可见 | `pyproject.toml` |
| 2 | P0 | **5 / 9 模块没有独立 test 目录** — 137 个测试平铺在 `src/core/tests/` | nav/gateway/drivers/memory/slam |
| 3 | P1 | **死代码** — `gateway_module.py:161-195` 函数定义后立即被覆盖 | `src/gateway/gateway_module.py` |
| 4 | P1 | **缺少 `__init__.py`** — `src/nav/services/` 是隐式命名空间包，脆弱 | `src/nav/services/` |
| 5 | P1 | **孤儿 `.pyc` 目录** — `src/semantic/common/semantic_common/` 只有 `.pyc` 无 `.py` | `src/semantic/common/` |
| 6 | P1 | **绕过 core.yaml_helpers** — `map_manager_module.py:967` 直接 `import yaml` 无回退 | `src/nav/services/map_manager_module.py` |

---

## 二、前序计划剩余任务（Tasks 4-7 评估）

| 计划任务 | 状态 | 建议 |
|----------|------|------|
| **Task 4**: 提取 NavigationModule ROS2 发布到 Bridge 层 | ❌ 未开始 | **降优先级**。`navigation_module.py` 的 rclpy 导入已是条件门控（`enable_ros2_bridge`），当前方案务实可行。仅在 ROS2-free 部署成为硬需求时执行。 |
| **Task 5**: 仿真证据 Freshness Gate | ❌ 未开始 | **中等优先级**。`sim/scripts/server_sim_closure.py` 已修改但未完成。下次 S100P 仿真活动前完成。 |
| **Task 6**: 真实 Planner Benchmark | ❌ 未开始 | **中等优先级**。当前 `benchmark_planner.sh` 只有 sleep 占位。声明 planner 性能改进前必须完成。 |
| **Task 7**: 导航链效率证据 | ❌ 未开始 | **低-中优先级**。结构良好但非阻塞，下次导航调试时顺手实现。 |

---

## 三、新发现问题汇总

### P0 — 立即修复

**1. 集成测试不可见**
- **文件**：`pyproject.toml` lines 88-98
- **问题**：`testpaths` 只列了 `src/core/tests` 等 3 个目录，顶层的 `tests/`（26 个集成测试）被排除
- **方案**（二选一）：
  - (a) 加 `tests/` 到 testpaths，ROS2 测试加 `@pytest.mark.ros2` + skip-if-no-ros2 fixture
  - (b) 创建 `scripts/run_integration_tests.sh` 文档化调用方式

**2. 死代码 — gateway_module.py**
- **文件**：`src/gateway/gateway_module.py` lines 161-195
- **问题**：`_apply_dynamic_filter_step1half` 函数定义（30 行）+ 整个 body，紧接着 line 196 被覆盖为 `= _map_apply_dynamic_filter_step1half`。函数体永不可达。
- **修复**：删除 lines 161-195（死定义），只保留 line 196（正确的引用赋值）

### P1 — 应该修复

**3. 缺少 `__init__.py`**
- **文件**：新建 `src/nav/services/__init__.py`
- **影响**：`nav.services` 成为 PEP 420 隐式命名空间包，任何 `services/` 目录都可能意外合并

**4. 孤儿包目录**
- **文件**：删除 `src/semantic/common/semantic_common/`
- **内容**：仅含 `__pycache__/` 中的 `robustness.pyc`、`sanitize.pyc`、`validation.pyc`，对应 `.py` 源文件不存在

**5. 绕过 core.yaml_helpers**
- **文件**：`src/nav/services/map_manager_module.py:967`
- **问题**：裸 `import yaml as _yaml` 没有 fallback，而 `core.yaml_helpers` 已提供无 yaml 时的 JSON 回退
- **修复**：替换为 `from core.yaml_helpers import ...`

**6. 测试文件仍用旧 shim 路径**
- **涉及文件**（6 个）：`test_dynamic_filter.py`、`test_map_occupancy.py`、`test_mujoco_mid360_pattern.py`、`test_nav_services.py`、`test_saved_map_artifact_gate.py`、`test_services_modules.py`
- **问题**：从 `nav.services.nav_services.X` 导入而非 `core.X`
- **修复**：更新导入为 `core.X`，然后删除 3 个 shim 文件

### P2 — 改善

**7. 8 个遗留 simplification wave 测试文件**
- **文件**：`test_simplification_wave1.py`、`test_simplification_wave2_team{A,B,C,D}.py`、`test_simplification_wave3_team{E,F,G}.py`
- **问题**：一次性代码简化项目的产物，增加 test 目录噪音
- **修复**：逐一审计 → 有覆盖价值的合并，重复的删除

**8. 3 个未文档化的 stack factory**
- **文件**：`src/core/blueprints/stacks/__init__.py` 导出 12 个，CLAUDE.md 只列了 9 个
- **缺失**：`exploration`、`lidar`、`sim_lidar`
- **修复**：补充 CLAUDE.md 文档

**9. 5 个模块无独立 test 目录**
- **缺失**：`src/nav/tests/`、`src/gateway/tests/`、`src/drivers/tests/`、`src/memory/tests/`、`src/slam/tests/`
- **现状**：137 个测试平铺在 `src/core/tests/`
- **修复**：创建 test 目录，迁移对应测试，更新 `pyproject.toml` testpaths

**10. `.gitignore` 遗漏**
- `build_nb_win/`、`*.egg-info/`、`*.pt` 模型文件未忽略
- **修复**：补充 `.gitignore`

### P3 — 锦上添花

**11. 模型文件 `yoloe-26s-seg.pt` 在源码树中**
**12. 3 个测试文件在非标准位置**
**13. Registry 热路径可加 `lru_cache`**
**14. `full_stack_wiring.py` 硬编码模块名字符串**

---

## 四、文件/文件夹规范化方案

### 4.1 目录重组

```
Phase 1（低风险 · 立即可做）
━━━━━━━━━━━━━━━━━━━━━━━━━
✅ 新建 src/nav/services/__init__.py
✅ 新建 src/semantic/perception/tests/__init__.py
✅ 删除 src/semantic/common/semantic_common/ （孤儿 .pyc）
✅ 删除 src/gateway/gateway_module.py:161-195 （死代码）

Phase 2（中风险 · 解耦收尾）
━━━━━━━━━━━━━━━━━━━━━━━━━
✅ 6 个测试文件导入路径从 shim 改为 core.*
✅ 删除 3 个 shim 文件（same_source_map_artifacts / dynamic_filter / yaml_helpers）
✅ 修复 map_manager_module.py:967 裸 import yaml
✅ 更新 .gitignore

Phase 3（高工作量 · 测试目录重组）
━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ 创建 5 个模块独立 test 目录
   src/nav/tests/          ← 7 个 test_nav_*.py
   src/gateway/tests/      ← 12 个 test_gateway_*.py
   src/drivers/tests/      ← test_driver_spec.py
   src/memory/tests/       ← 2 个 test_memory_*.py
   src/slam/tests/         ← 5 个 test_slam_*.py
✅ 迁移 3 个非标准位置测试文件
✅ 审计并清理 8 个 simplification_wave 文件
✅ 更新 pyproject.toml testpaths

Phase 4（架构优化 · 可选）
━━━━━━━━━━━━━━━━━━━━━━━━━
🔵 nav/services/nav_services/ → nav/services/ （扁平化嵌套）
🔵 合并 semantic_exploration.yaml / far_planner.yaml 到 robot_config.yaml
🔵 full_stack_wiring.py 常量化模块名
🔵 Registry 热路径加 lru_cache
```

### 4.2 命名约定

| 规范 | 当前 | 目标 |
|------|------|------|
| 所有 package 有 `__init__.py` | `nav/services/` 缺 | 补全 |
| test 目录有 `__init__.py` | `perception/tests/` 缺 | 补全 |
| 二进制文件不入源码树 | `yoloe-26s-seg.pt` 在 `src/` | 移入 `models/` 或用 git-lfs |
| 构建产物不入仓库 | `build_nb_win/`、`.egg-info/` 存在 | `.gitignore` 排除 |
| 测试从规范路径导入 | 6 个文件用 shim 路径 | 改为 `core.*` |
| 统一配置格式 | `dufomap.toml` 是唯一的 TOML | 改为 YAML 或保留（标注例外原因） |

### 4.3 目标目录结构

```
src/
├── core/                     # 框架内核 + 共享契约（不变）
│   ├── blueprints/           # Blueprint 组装（不变）
│   │   ├── full_stack.py
│   │   ├── full_stack_wiring.py
│   │   └── stacks/           # 12 个 factory（文档补齐到 12）
│   ├── tests/                # 核心框架测试（保留，逐步瘦身）
│   ├── same_source_map_artifacts.py  # 共享契约 ✅
│   ├── dynamic_filter.py             # 共享契约 ✅
│   ├── yaml_helpers.py               # 共享契约 ✅
│   └── ...
├── nav/
│   ├── services/             # nav_services 扁平化 → services
│   │   ├── __init__.py       # 🆕 显式 package
│   │   ├── geofence_manager_module.py
│   │   ├── map_manager_module.py
│   │   ├── patrol_manager_module.py
│   │   └── task_scheduler_module.py
│   ├── tests/                # 🆕 独立 test 目录
│   │   ├── __init__.py
│   │   ├── test_nav_modules.py         # ← 从 core/tests/
│   │   ├── test_nav_services.py        # ← 从 core/tests/
│   │   └── ...
│   └── ...
├── gateway/
│   ├── tests/                # 🆕 独立 test 目录
│   │   ├── __init__.py
│   │   ├── test_gateway_runtime_status.py
│   │   └── ...
│   └── ...
├── drivers/
│   ├── tests/                # 🆕 独立 test 目录
│   └── ...
├── memory/
│   ├── tests/                # 🆕 独立 test 目录
│   └── ...
├── slam/
│   ├── tests/                # 🆕 独立 test 目录
│   └── ...
└── semantic/                 # 清理 orphan common/ 目录
    ├── common/               # 保留（如实际有内容）
    │   └── (删除空的 semantic_common/)
    └── ...
```

---

## 五、优先级执行清单（排序）

### 🔴 Sprint 1：立即修复（预估 2-3 小时）

| # | 行动 | 工作量 | 文件数 |
|---|------|--------|--------|
| S1-1 | 删除 `gateway_module.py` 死代码（lines 161-195） | S | 1 |
| S1-2 | 新建 `src/nav/services/__init__.py` | S | 1 |
| S1-3 | 删除 `src/semantic/common/semantic_common/` 孤儿目录 | S | 1 |
| S1-4 | 修复 `map_manager_module.py:967` 裸 `import yaml` → `core.yaml_helpers` | S | 1 |
| S1-5 | 补充 `.gitignore`（build/egg/pt） | S | 1 |
| S1-6 | 文档化集成测试运行方式（方案 a 或 b） | S | 1-2 |
| S1-7 | 运行全部 core tests 确认零回归 | S | - |

### 🟡 Sprint 2：解耦收尾（预估 4-5 小时）

| # | 行动 | 工作量 | 文件数 |
|---|------|--------|--------|
| S2-1 | 6 个测试文件导入路径：`nav.services.nav_services.X` → `core.X` | M | 6 |
| S2-2 | 删除 3 个 shim 文件 | S | 3 |
| S2-3 | 确认边界测试通过 + 全量测试通过 | S | - |
| S2-4 | 新建 `src/semantic/perception/tests/__init__.py` | S | 1 |
| S2-5 | CLAUDE.md 补充 3 个 factory（exploration/lidar/sim_lidar） | S | 1 |
| S2-6 | 审计 8 个 simplification_wave 文件，决定去留 | M | 8 |

### 🟠 Sprint 3：测试目录重组（预估 6-8 小时）

| # | 行动 | 工作量 | 文件数 |
|---|------|--------|--------|
| S3-1 | 创建 `src/nav/tests/` + 迁移 7 个测试 | M | 7+ |
| S3-2 | 创建 `src/gateway/tests/` + 迁移 12 个测试 | M | 12+ |
| S3-3 | 创建 `src/drivers/tests/` + 迁移测试 | M | 3+ |
| S3-4 | 创建 `src/memory/tests/` + 迁移 2 个测试 | S | 2+ |
| S3-5 | 创建 `src/slam/tests/` + 迁移 5 个测试 | M | 5+ |
| S3-6 | 迁移 3 个非标准位置测试文件 | M | 3 |
| S3-7 | 更新 `pyproject.toml` testpaths | S | 1 |
| S3-8 | 全量测试 + 修复导入路径 | M | - |

### 🔵 Sprint 4：深度优化 + 前序计划收尾（按需）

| # | 行动 | 工作量 | 说明 |
|---|------|--------|------|
| S4-1 | `nav/services/nav_services/` → `nav/services/` 扁平化 | L | 影响导入路径，需全局更新 |
| S4-2 | Planner benchmark 真实化（Plan Task 6） | M | 替换 sleep → A*/PCT 实测 |
| S4-3 | 仿真 Freshness Gate（Plan Task 5） | M | 结构化 JSON + scenario 补齐 |
| S4-4 | 导航链效率证据（Plan Task 7） | M | 链级遥测 JSON artifact |
| S4-5 | ROS2 提取到 Bridge 层（Plan Task 4） | L | 仅 ROS2-free 部署需求时 |
| S4-6 | Registry 热路径 `lru_cache` | S | 性能微优化 |
| S4-7 | `full_stack_wiring.py` 常量化模块名 | M | 维护性改善 |
| S4-8 | 合并 config 到 `robot_config.yaml` | S | 减少配置碎片 |

---

## 六、不做的

| 事项 | 理由 |
|------|------|
| `nav/services/nav_services/` 重命名 | 工作量太大（影响几十个导入路径），Phase 4 可选做 |
| ROS2 完全从 NavigationModule 移除（Task 4） | 当前条件导入方案已足够务实，无 ROS2-free 部署需求 |
| 重写 agent_loop.py 工具调用 | 已通过 test_agent_loop_tool_schema.py 测试，当前实现稳定 |
| 迁移到 `src/` 以外的测试框架 | 当前 pytest + conftest.py 模式工作良好，不需要引入新工具 |

---

## 七、验收标准

每个 Sprint 完成后执行：

```bash
# 边界测试（必须 0 失败）
python -m pytest src/core/tests/test_module_boundaries.py -v

# 核心框架测试（必须全绿，当前基线 ~1226 tests）
python -m pytest src/core/tests/ -q

# 导入无循环
python -c "from core.module import Module; from core.blueprint import Blueprint; print('OK')"

# Lint 零新增问题
ruff check src/ --statistics
```

---

## 附录：文件索引

| 文件 | 审计发现 |
|------|----------|
| `src/gateway/gateway_module.py` | lines 161-195 死代码 |
| `src/nav/services/` | 缺 `__init__.py` |
| `src/semantic/common/semantic_common/` | 孤儿 `.pyc`，应删除 |
| `src/nav/services/map_manager_module.py:967` | 裸 `import yaml` |
| `src/nav/services/same_source_map_artifacts.py` | shim，Sprint 2 后删除 |
| `src/nav/services/dynamic_filter.py` | shim，Sprint 2 后删除 |
| `src/nav/services/yaml_helpers.py` | shim，Sprint 2 后删除 |
| `pyproject.toml` | testpaths 需扩展 |
| `.gitignore` | 缺 build_nb_win / *.egg-info / *.pt |
| `CLAUDE.md` | 缺 3 个 factory 文档 |
| `tests/benchmark/benchmark_planner.sh` | sleep 占位，需真实化 |
| `sim/scripts/server_sim_closure.py` | 已修改但未完成 gate |
