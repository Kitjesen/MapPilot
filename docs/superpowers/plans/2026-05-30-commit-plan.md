# 工作区分流提交计�?(分支 codex/super-lio-localization-backend)

> 现状�?98 个未提交改动，横�?6+ 个不相关主题，行级混杂。本计划把它�?
> 拆成可独�?review 的提交单元，并标注每个单元的可行性与风险�?
> **不擅自提�?*——尤其非本人工作与已�?bug（F 破坏 MuJoCo）需各自 owner 确认�?

## 主题归类（机器分类，198 项）

| 代号 | 主题 | 数量 | 类型 | 可干净提交 |
|------|------|------|------|-----------|
| A | sim/real 重构（本次） | 23 | rename+新文�?混杂M | �?需 worktree 重建 |
| B | runtime evidence/contract/inspection | 21 | 几乎�?untracked | �?|
| C | gateway runtime dataflow + inspection view | 6 | �?untracked | �?|
| D | super-lio / SLAM 后端 | 5 | M | 分支本主�?|
| E | semantic / follow-person | 10 | M | 可能已部分提�?|
| F | base_autonomy / nav 核心 | 16 | M | ⚠️ 含破�?MuJoCo 的改�?|
| Z | cli/config/docs/scripts/web 杂项 | 117 | M+少量untracked | 需再细�?|

## 提交单元与建�?

### B �?runtime evidence（✅ 可立即干净提交�?
`scripts/real_runtime_*`、`runtime_contract_audit`、`saved_map_artifact_gate`�?
`src/runtime/{runtime_evidence,runtime_switch,runtime_validation_gates,inspection_acceptance,
gateway_runtime_acceptance,product_field_check}` + 对应 `test_*` + `cli/runtime_{audit,display}`�?
- 全新文件，自包含。建议：`feat(runtime): real-runtime evidence + contract acceptance gates`
- owner 确认�?`git add` 这些路径单独 commit�?

### C �?gateway dataflow（✅ 可立即干净提交�?
`src/gateway/services/runtime_dataflow.py`、`web/src/components/{RuntimeDataflowView,
InspectionAcceptanceView}.*`、`web/scripts/dataflow-ui-contract.mjs`�?
- 全新文件。建议：`feat(gateway): runtime dataflow view + inspection acceptance UI`
- 注意：`src/gateway/services/runtime_switch_plan.py` 也属此组�?

### A �?sim/real 重构（❌ 当前分支不可干净切，建议 worktree 重建�?
内容：`drivers/spec.py` + `drivers/real/`(11 rename) + 资产归一(scenes→worlds, robot→robots)
+ `drivers/README.md`、`sim/scripts/README.md`、`sim/assets/README.md`
+ `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md` + 本目�?plans
+ `test_driver_spec.py` + 契约 docstring/注释（full_stack_wiring, navigation_module,
  local_planner, traversability_cost_module�? 20 �?driver import 修正�?
- **障碍**：import 修正文件与别人的改动行级混杂（落�?Z）�?
- **干净方案**：从干净基线（origin/master �?stash �?HEAD）开�?worktree，按上述清单
  重建，commit 为：
  - `refactor(drivers): symmetric real/ vs sim/ backends + driver spec contract`
  - `refactor(sim): consolidate scenes→worlds, robot→robots assets`
  - `docs(nav): navigation compute contract + layout redesign`
- 已全部验证（driver 228 passed、framework 334 passed、driver_spec 6、profile 38）�?

### F �?base_autonomy/nav（⚠�?含已�?bug，先查再提交�?
`base_autonomy/modules/{local_planner,path_follower,terrain}` + `nav/*`�?
- **已确�?*：这批改动导�?MuJoCo 全栈移动测试 `moved=0.0`（HEAD baseline 通过）�?
- **不要直接提交**——先二分定位哪个改动破坏移动，修复后再提交�?

### D / E / Z �?需 owner 确认
- D super-lio：分支本主题，owner 确认完成度后提交�?
- E semantic/follow：检查是否已被最�?3 �?commit 覆盖，避免重复�?
- Z 117 项：�?cli/config/docs/scripts/web，需再细分主题或�?owner 分流�?

## 推荐执行顺序

1. **先止血**：F �?MuJoCo 回归 bug 定位修复（唯一确认的功能破坏）�?
2. **干净分流**：B、C 各自独立提交（自包含新主题）�?
3. **A worktree 重建**：把本次 sim/real 重构产出为独立、可 review �?PR�?
4. D/E/Z 由各 owner 认领�?

## 风险

- �?198 混杂改动上继续堆功能 = 债滚债，且回归无法干净归因（已�?MuJoCo 坑）�?
- 任何提交前应�?`git stash` HEAD baseline 对照，确认不固化 F �?bug�?
