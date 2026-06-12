# Plan: 仓库目录架构重构 — sim/real 可读化 (架构师方案)

> Superseded note (2026-06-12): the physical split proposal for moving
> `sim/scripts/*` into separate gate/demo directories is no longer current
> guidance. The later 2026-05-31 modularization record and
> `.omx/plans/ralplan-sim-directory-reorg-20260612T084832Z.md` preserve
> `sim/scripts/<name>` as a stable public path contract and organize scripts
> through indexes and tests instead.





## 1. 目标与成功标准

**目标**：消除 sim/real 在目录层的晦涩与散落，建立 dimos 式「接口 + 可替换后端」
的对称结构，并补齐缺失的驱动契约层。

**成功标准（可读性可验证）**：
- 任何人能在 10 秒内回答：「某能力(驱动/传感器)的 real 实现和 sim 实现分别在哪」。
- 仿真场景/机器人模型只有**一个**权威存放树，不再 worlds/scenes/assets 三处重复。
- `sim/scripts` 不再混装 gate/validation/demo/launch。
- 存在显式的驱动/传感器接口契约（spec），real/sim/stub 都实现它。
- 重构后 `python -m pytest src/core/tests/ -q` 不回归。

## 2. 范围边界

**做**：目录重组、资产归一、契约层引入、import 路径修复、文档。
**不做**：不改算法逻辑、不改 Module 端口语义、不引入新依赖、不动 C++/.so 构建产物内容。

## 3. 现状诊断 (证据)

### 散落点
- `src/drivers/sim/`：mujoco_driver / ros2_sim_driver / sim_pointcloud_provider / stub（驱动后端，位置合理）。
- 根 `sim/`：engine/ + scripts/(50+) + following/ + assets/ + meshes/ + scenes/ +
  worlds/ + robots/ + robot/ + bridge/ + sensors/ + semantic/ + evaluation/ +
  datasets/ + output/ —— 职责混杂。
- `src/core/blueprints/simulation_contract.py`、`stacks/sim_lidar.py`：sim 契约在 core。
- `src/core/tests/test_sim_*`、`test_simplification_wave*`、`test_*gazebo*`、`test_*mujoco*`：sim 测试散布。

### 三大病灶
1. **sim 概念分裂**：`src/drivers/sim/`（后端）vs 根 `sim/`（引擎+工具王国），边界不清。
2. **资产重复**：场景在 `sim/worlds`、`sim/scenes`、`sim/assets/{mjcf,sdf,urdf,xml}` 三处；
   机器人模型 `sim/robot/`(单) 与 `sim/robots/`(复) 并存。
3. **缺接口契约**：`src/drivers/{thunder,sim,lidar,gnss}` 平铺，无 driver/sensor `spec`，
   sim 与 real 不是「同一接口可替换后端」。这是「欠缺的东西」。

### dimos 对照 (理想)
- `hardware/<能力>/{spec.py(Protocol), registry.py, mock/, <vendor>/}`
- `simulation/{base, engines, <engine>}`，`robot/<model>/{connection, blueprints}`
- sim/real 差异只压在最底层后端，上层栈零差异（见 `2026-05-29-sim-production-alignment.md`）。

## 4. 目标目录架构

```
src/drivers/
  spec.py            # [新] DriverAdapter / SensorSource Protocol 契约
  real/              # [新] 真实硬件后端
    thunder/  lidar/(livox)  gnss/
  sim/               # 仿真后端 (保留现有文件)
  stub.py            # mock 后端
  # real / sim / stub 均实现 spec.py，注册到 core.registry

sim/                 # 根目录三区化
  engine/            # 仿真引擎运行时 (保留)
  assets/            # [归一] worlds/ (worlds+scenes 合并) robots/ (robot+robots 合并) meshes/ mjcf/ sdf/ urdf/
  gates/             # [新] CI 门禁 *_gate.py
  validation/        # [新] *_validation.py / *_diagnosis* 评估
  demos/             # [新] demo_*.py / run_*.py / record_*.py / view_*.py
  launch/            # 保留 (launch_*.sh / *.launch.py)
  following/ datasets/ output/   # 保留 (output/datasets 应 gitignore)
```

## 5. 旧 → 新 迁移映射 (关键项)

| 旧路径 | 新路径 | 风险 |
|--------|--------|------|
| `sim/scenes/*.xml` + `sim/worlds/*.xml` | `sim/assets/worlds/` | 低(引用集中在 profile/scene_xml) |
| `sim/robot/thunder.urdf` + `sim/robots/*` | `sim/assets/robots/` | 低 |
| `sim/assets/{mjcf,sdf,urdf,xml}` | `sim/assets/<同名>/`(就地归并) | 低 |
| `sim/scripts/*_gate.py` | `sim/gates/` | 中(scripts/lingtu、CI 引用路径) |
| `sim/scripts/*_validation.py`,`*_diagnosis*` | `sim/validation/` | 中 |
| `sim/scripts/demo_*.py`,`run_*.py`,`record_*.py`,`view_*.py` | `sim/demos/` | 中 |
| `src/drivers/thunder/` | `src/drivers/real/thunder/` | **高(import 多)** |
| `src/drivers/lidar/`,`gnss/` | `src/drivers/real/{lidar,gnss}/` | **高** |
| 新增 | `src/drivers/spec.py` | 低(纯新增) |

## 6. 实施步骤 (三阶段，风险递增)

### 阶段 A — 根 sim/ 三区化 + 资产归一 (低风险，先做)
1. 建 `sim/assets/worlds`、`sim/assets/robots`，迁移 scenes/worlds/robot/robots → 归一。
2. `sim/{gates,validation,demos}/` 建立，按前缀迁移 `sim/scripts/*`。
3. 全仓 grep 修正对这些资产/脚本的路径引用（profile `scene_xml`、`scripts/lingtu`、CI、launch）。
4. 回归：`pytest src/core/tests -q` + 抽样跑一个 gate。

### 阶段 B — 驱动契约层 spec.py (中风险，纯增强)
5. 新增 `src/drivers/spec.py`：`DriverAdapter`/`SensorSource` Protocol（connect/read/write/streams）。
6. 现有 thunder/sim/stub **声明实现该 spec**（不改逻辑，加类型/注释 + registry 注册校验）。
7. 测试：契约一致性测试（sim/real/stub 端口形状一致）。

### 阶段 C — drivers real/ 对称 (高风险，单独确认后做)
8. `src/drivers/real/` 建立，迁移 thunder/lidar/gnss，批量修 import + registry。
9. 全量回归 + 部署路径核对。

## 7. 涉及文件清单 (高层)

- 新增：`src/drivers/spec.py`、`sim/{gates,validation,demos,assets}/`、本 plan、契约文档更新。
- 移动：`sim/scripts/*`、`sim/{scenes,worlds,robot,robots,assets}/*`、(阶段C)`src/drivers/{thunder,lidar,gnss}`。
- 路径引用修正：`cli/profiles_data.py`(scene_xml)、`scripts/lingtu`、`sim/launch/*`、CI、tests。

## 8. 验收清单

- [ ] sim/ 根目录只剩 engine/assets/gates/validation/demos/launch/following/datasets/output
- [ ] 场景与机器人模型单一权威树 (sim/assets)
- [ ] `src/drivers/spec.py` 存在，real/sim/stub 实现一致接口
- [ ] (阶段C) `src/drivers/real/` 与 `src/drivers/sim/` 对称
- [ ] `pytest src/core/tests -q` 不回归；抽样 gate 可跑
- [ ] 契约文档同步更新目录约定

## 9. 风险与回滚

- **移动破坏 import/路径**：每阶段独立提交；用 grep 全量定位引用后再移；CI 红即回滚该阶段。
- **部署脚本路径**：`scripts/lingtu`、robot-side 路径可能硬编码 `sim/scripts/...`，阶段A 必须同步改。
- **阶段隔离**：A/B/C 各自可独立 revert，互不依赖。先 A（收益大风险小），C 单独确认。

## 10. 依赖与环境

- 无新增依赖。回归用 `python -m pytest`（本机 uv 因 brainstem-api py≥3.13 约束失败）。

## 11. 执行记录与架构决策修正 (2026-05-30)

### 已完成
- **阶段 B**：新增 `src/drivers/spec.py`（MotionDriver/CameraSource/PointcloudSource 契约）
  + `src/core/tests/test_driver_spec.py`（6 passed）。sim/real/stub 端口同形状现已机器可校验。
- **阶段 A-1（修正版）**：资产归一**消除重复、保持层级**，而非塞入 `sim/assets/` 深层级：
  - `sim/scenes/*.xml → sim/worlds/`，`sim/robot/thunder.urdf → sim/robots/`
  - 5 处引用已改，空目录已删，零 xml 相对路径风险。
- **可读性文档**：`src/drivers/README.md`（sim/real/mock/colcon 边界 + spec 契约）、
  `sim/scripts/README.md`（脚本分类索引）。

### 架构决策修正：放弃「大爆炸式物理 mv」(原阶段 A-2 / C)
**证据**：`sim/scripts/<name>` 已是事实契约（`_external_launcher`、`simulation_contract`、
`test_profile_graph_snapshots` 断言、多个 `from sim.scripts.x import`、CI/deploy/sh）。
物理移动 = 改 30+ 处跨 profile/契约/测试/CI，连锁且易漏；drivers 物理 `real/` 因 colcon
包只能部分对称、还需改 ~24 处 import。

**决策**：当路径已被广泛契约化，物理重组成本非线性爆炸。改用「接口契约(spec) + 文档索引」
达成可读性，**不做** `sim/scripts` 三区化 mv 与 `drivers/real/` 物理迁移。实质可读性收益
的主体已由 spec.py(B) + 资产归一(A-1) + README 索引拿到，风险归零。

### 基线警示（独立于本重构）
工作区存在进入任务前的未提交改动（`base_autonomy/local_planner_module.py`、
`path_follower_module.py`、`terrain_module.py`、`nav/navigation_module.py` 等），已导致
2 个 MuJoCo 全栈移动测试 `moved=0.0`。`git stash` 到 HEAD 基线后该测试 **通过**，证明
失败源于预存改动，**与本重构无关**。建议单独排查这批 base_autonomy/nav 改动。
