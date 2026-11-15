# MLS 后端 A/B 验收

这份文档定义全局规划后端的离线对照格式。工具只读取 JSON，不启动导航进程，不修改地图，也不把路线预览当成机器人执行成功。

工具入口是 `tools/bench/mls_backend_compare.py`。它把每个后端的证据分成四层。

| 层 | 需要回答的问题 | 典型字段 |
| --- | --- | --- |
| `surface` | 是否得到正确的多层表面 | `resolution_m`、`surface_count`、`observed_count`、`inferred_count`、`levels` |
| `graph` | 是否建立了可查询的稀疏图 | `node_count`、`edge_count`、`component_count`、`fallback_used` |
| `geometry` | 路线是否满足 LingTu 的三维执行合同 | `route_valid`、`route_reason`、`checked_points`、`checked_segments`、`sweep_valid`、`motion_diagnostic_valid` |
| `performance` | 构图和查询分别花了多久 | `cold_prepare_ms`、`cold_total_ms`、`cache_restore_ms`、`warm_query_ms` |

## 输入格式

输入必须声明 `lingtu.mls_backend_compare.v1`，每个 case 至少包含两个后端。后端名称可以是 `lingtu_cpp`、`dimos_mls`、`lingtu_rust` 或离线 PCT 适配器。

```json
{
  "schema": "lingtu.mls_backend_compare.v1",
  "evidence": {
    "map_id": "official-building2-9",
    "map_commit": "9a9cc431ea905a5878975cc6fbbce6c9618b31a4",
    "measured_backend_commit": "26144f7fd8424bdc33ab97dcd58e9497f2d413ee",
    "latest_backend_commit": "5e0ebfd422a6",
    "execution_validated": false
  },
  "cases": [
    {
      "id": "building_cross",
      "backends": {
        "lingtu_cpp": {
          "surface": {
            "resolution_m": 0.05,
            "surface_count": 24151,
            "observed_count": 24151,
            "inferred_count": 0,
            "levels": 4
          },
          "graph": {
            "node_count": 5939,
            "edge_count": 1092,
            "component_count": 1,
            "fallback_used": false
          },
          "geometry": {
            "route_valid": true,
            "route_reason": "ok",
            "checked_points": 208,
            "checked_segments": 207,
            "sweep_valid": true,
            "motion_diagnostic_valid": true
          },
          "performance": {
            "cold_prepare_ms": 32900,
            "warm_query_ms": 318.7,
            "cold_total_ms": 32900
          }
        },
        "dimos_mls": {
          "surface": {
            "resolution_m": 0.05,
            "surface_count": 0,
            "observed_count": 0,
            "inferred_count": 0,
            "levels": 0
          },
          "graph": {
            "node_count": 6095,
            "edge_count": 0,
            "component_count": 2,
            "fallback_used": false
          },
          "geometry": {
            "route_valid": false,
            "route_reason": "disconnected_components",
            "checked_points": 0,
            "checked_segments": 0,
            "sweep_valid": false,
            "motion_diagnostic_valid": false
          },
          "performance": {
            "cold_prepare_ms": 4020,
            "warm_query_ms": 14.1,
            "cold_total_ms": 4020
          }
        }
      }
    }
  ]
}
```

`surface_count`、`node_count` 和 `edge_count` 必须来自实际后端输出。工具不会从路线点数猜测表面或图规模。缺字段会进入 `PARTIAL` 和 evidence gap。

## 运行方式

```powershell
python tools/bench/mls_backend_compare.py `
  --input build/planner-compare-20261114/mls-acceptance.json `
  --output-json build/planner-compare-20261114/mls-acceptance.report.json `
  --output-md build/planner-compare-20261114/mls-acceptance.report.md
```

工具只使用 Python 标准库。输出中包含每个 case 的四层状态、性能比值、缺失证据和两类最终声明。

- `offline_route_geometry` 表示是否有路线几何复核证据。
- `execution` 只有在输入明确设置 `execution_validated: true` 时才为真。

## 四层通过条件

### Surface

通过需要有地图分辨率和表面数量。正式对照还应记录观测表面、推断表面、多高度层数量和地图 revision。推断表面必须和原始占据证据分开保存。

### Graph

通过需要有节点数量和边数量。楼梯、平台、楼层和断开的区域必须记录组件数量。区域更新对照还要保存区域更新前后受影响的节点和边。

### Geometry

只有 `route_valid=true` 且 `sweep_valid=true` 才能标记通过。`motion_diagnostic_valid` 作为额外的几何运动诊断，不替代机身扫掠结果。路线点数、终点误差或路径返回成功不能单独构成通过。

### Performance

冷准备、缓存恢复和热查询分别记录。构图失败的快速返回不能和成功路线的查询耗时放在同一平均值里。不同体素尺寸、墙距、机身高度和支撑谓词不能直接做性能排名。

## 当前证据和边界

2026-11-14 的 matched 实验已经提供 LingTu 和 DimOS 的部分路线、几何复核和性能数据。它使用 DimOS `26144f7`，而最新 DimOS `main` 已经是 `5e0ebfd422a6`。因此下一轮输入需要重新生成 DimOS 的 matched 结果，不能把旧提交结果标成最新版本。

当前已有的三类结果应该这样记录。

| 结果 | 可以证明 | 不能证明 |
| --- | --- | --- |
| Surface/Graph smoke | 静态资产结构和序列化行为 | 实时地图区域更新 |
| `octoplanner3d_verify_path` | 路线的机身、支撑、扫掠、坡度和步高条件 | SCAN 执行和物理运动 |
| MuJoCo 导航报告 | 选定仿真场景中的局部执行表现 | Go2/NX 实机通过 |

以下项目仍需要单独补证据。

- 最新 DimOS `main` 的同输入构图和热查询。
- Rust 原生 node/edge/corridor 与当前 C++ `SurfaceGraph` 的逐层 A/B；当前已完成的是 Rust surface provider 与 C++ 图的对照。
- mapd 区域变更输入和区域修复结果。
- SCAN 接入、动态障碍、Follower、Driver 的端到端执行。
- MuJoCo 完整建图、定位、全局路线、局部轨迹和到达停车。
- Go2 或 Thunder 的实机导航。

只有最后两类物理运行都通过，PDF 才能写“完整导航验收通过”。当前工具输出的默认结论是离线全局路线几何证据。

2026-11-15 的 surface 对齐和 Rust 原生 graph 摘要复测已补充到
`docs/research/mls-surface-compare-arrival-box-20261115.md`：arrival-box 和
body-depth/align 的表面、图统计以及共享 C++ 几何复核一致；upper-floor 和
batch-room 的无路线结果也一致。Rust 原生 graph 的规模已经接近 C++ 图，但
候选边还没有经过 LingTu 机身、支撑和连续扫掠准入，尚未进入生产
`PlannerArtifact`、SCAN、Follower 或 Driver。

以上是离线验收文档的历史结论。当前生产 `PlannerArtifact` 已持有 Rust graph
handle，`startPlan()` 先执行 Rust limited endpoint query，再经过 C++ 几何准入，
整条候选失败时回退 C++ graph。该变更只覆盖全局 graph provider，不能替代文档中
仍未完成的 surface/cache、MuJoCo 全链路和实机验收。
