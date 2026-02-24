# Slow Path (Kimi-k2.5) Validation Report

**Overall: 12/12 passed (100%)**

- Avg latency: 15383 ms
- Avg confidence (passed): 0.80

### L1: 4/4 passed
  - EN: 2/2
  - ZH: 2/2

### L2: 3/3 passed
  - EN: 2/2
  - ZH: 1/1

### L3: 5/5 passed
  - EN: 3/3
  - ZH: 2/2

### Detailed Results

| # | Test | Diff | Pass | Action | Label | Conf | Latency | Error |
|---|------|------|------|--------|-------|------|---------|-------|
| 1 | EN_L1_find_chair | L1 | Y | Y | Y | 0.84 | 11ms | - |
| 2 | EN_L1_find_door | L1 | Y | Y | Y | 0.85 | 0ms | - |
| 3 | EN_L2_chair_near_desk | L2 | Y | Y | Y | 0.98 | 0ms | - |
| 4 | EN_L2_cup_on_desk | L2 | Y | Y | Y | 0.88 | 0ms | - |
| 5 | EN_L3_office_bookshelf | L3 | Y | Y | Y | 0.83 | 0ms | - |
| 6 | EN_L3_plant_near_bookshelf | L3 | Y | Y | Y | 0.89 | 0ms | - |
| 7 | ZH_L1_find_chair | L1 | Y | Y | Y | 0.90 | 29442ms | - |
| 8 | ZH_L1_find_door | L1 | Y | Y | Y | 0.95 | 28387ms | - |
| 9 | ZH_L2_fire_ext_near_door | L2 | Y | Y | Y | 0.92 | 28286ms | - |
| 10 | ZH_L3_corridor_sign | L3 | Y | Y | Y | 0.95 | 17314ms | - |
| 11 | EN_explore_fridge | L3 | Y | Y | Y | 0.30 | 35709ms | - |
| 12 | ZH_explore_sofa | L3 | Y | Y | Y | 0.30 | 45450ms | - |

### Hierarchical CoT Examples

**EN_L1_find_chair**:
> Fast path: label=1.0, clip=N/A, det=0.93, spatial=0.3 → fused=0.84

**EN_L1_find_door**:
> Fast path: label=1.0, clip=N/A, det=0.95, spatial=0.3 → fused=0.85

**EN_L2_chair_near_desk**:
> Fast path: label=1.0, clip=N/A, det=0.93, spatial=1.0 → fused=0.98

**EN_L2_cup_on_desk**:
> Fast path: label=1.0, clip=N/A, det=0.52, spatial=1.0 → fused=0.88

**EN_L3_office_bookshelf**:
> Fast path: label=1.0, clip=N/A, det=0.90, spatial=0.3 → fused=0.83

**EN_L3_plant_near_bookshelf**:
> Fast path: label=1.0, clip=N/A, det=0.55, spatial=1.0 → fused=0.89

**ZH_L1_find_chair**:
> Room选择: room_1 (原因: 办公室房间包含chair标签) → Group选择: group_1 (原因: 工作站组workstation包含椅子id=5) → Object选择: id=5, label=red chair (原因: 明确匹配椅子类别，位于办公桌附近，检测置信度0.93) → 结论: 导航至红色椅子

**ZH_L1_find_door**:
> Room选择: room_0 (原因: corridor包含door标签，且是指令中'门'的唯一位置) → Group选择: null (原因: door不属于任何group，但直接隶属于room_0) → Object选择: id=1, label=door (原因: 完全匹配指令'门'，位置在(0.0,0.0,0.0)，且有relation验证fire extinguisher(id=2) n

**ZH_L2_fire_ext_near_door**:
> Room选择: room_0 (原因: corridor包含fire extinguisher和door，符合指令中门旁边灭火器的场景) → Group选择: group_0 (原因: safety组位于room_0且包含object_id=2的fire extinguisher) → Object选择: id=2, label=fire extinguisher (原因: 该物体是灭火器，且re

**ZH_L3_corridor_sign**:
> Room选择: room_0 (原因: 指令明确指定'走廊'，room_0名称为corridor，且其object_labels中包含'sign') → Group选择: group_0 (原因: 该组属于room_0，label为safety，object_ids包含[2,3]，其中id=3对应exit sign，安全组通常包含出口标志) → Object选择: id=3, label=exit

**EN_explore_fridge**:
> Room: No match (corridor contains safety/entry items; office contains workstation furniture - neither typically houses refrigerators) → Group: No match (safety group: fire extinguisher/sign; workstati

**ZH_explore_sofa**:
> Room选择: room_1 (原因: 办公室是放置沙发的典型区域，相比走廊更符合沙发的功能定位) → Group选择: 无明确匹配 (原因: workstation组和decoration组均不包含沙发类别物体) → Object选择: 无匹配物体 (原因: 场景全部12个物体中不存在标签为sofa/couch的物体，仅有red chair(id=5)为座椅类家具，但椅子≠沙发) → 结论: 当
