# ThunderV4 MuJoCo 多层楼梯近景候选

更新时间：2026-07-04

## 结论

已新增一个专用楼梯测试场景，用来验证 ThunderV4 强化学习运控在跨高度平台上的表现。

本次测试通过的是：

```text
cmd_vel -> MuJoCo policy -> ThunderV4 motion
```

这不是完整导航闭环，也没有接入 OctoPlanner3D、LocalPlanner、DDS、Gateway 或真实机器人驱动。

## 测试资产

| 项目 | 路径 |
| --- | --- |
| 多层楼梯场景 | `sim/worlds/mujoco/thunderv4_stair_showcase.xml` |
| 场景注册 | `src/drivers/sim/mujoco/driver.py` |
| 录制脚本 | `sim/scripts/record_thunderv4_stair_showcase.py` |
| 侧向近景视频 | `artifacts/mujoco_thunderv4_stair_showcase_vx10_side.mp4` |
| 视频中帧截图 | `artifacts/mujoco_thunderv4_stair_showcase_vx10_side_frame_mid.jpg` |
| 指标 JSON | `artifacts/mujoco_thunderv4_stair_showcase_vx10_side.json` |

## 场景设计

场景不是平地 gait 展示，而是两段楼梯：

- 第一段：6 级台阶
- 中间平台：用于检查第一段跨高度是否完成
- 第二段：6 级台阶
- 上层平台：用于检查跨层高度是否完成
- 每级高度：0.06m
- 总平台高度：0.72m
- 黑色台阶边线：用于视频里清楚看到台阶层级

完整建筑场景 `building_scene.xml` 仍然保留，但它是复杂跨楼层环境，不适合作为第一轮 gait 近景验收，因为会把运控能力、相机视角、建筑碰撞和导航问题混在一起。

## 当前结果

命令：

```powershell
$env:PYTHONPATH='src'; python sim\scripts\record_thunderv4_stair_showcase.py --camera side --linear-x 1.0 --duration-s 18 --video artifacts\mujoco_thunderv4_stair_showcase_vx10_side.mp4 --json-out artifacts\mujoco_thunderv4_stair_showcase_vx10_side.json --frame-out artifacts\mujoco_thunderv4_stair_showcase_vx10_side_frame_mid.jpg
```

验收结果：

| 指标 | 结果 |
| --- | ---: |
| 是否到达中间平台 | true |
| 是否到达上层平台 | true |
| 前进距离 | 7.329m |
| 高度增量 | 0.725m |
| 最大高度 | 1.148m |
| 最大横向偏移 | 0.143m |
| 最大 roll | 0.067rad |
| 最大 pitch | 0.253rad |
| 机身触地事件 | 0 |
| 结果 | PASS |

## 边界发现

速度扫描显示：

- `vx=0.55, 16s`：能到中间平台，不能到上层平台。
- `vx=0.9, 20s`：能到上层平台，后段姿态开始变差。
- `vx=1.0, 18s`：当前最合适的候选，能到上层平台且没有机身触地。
- `vx=1.0, 22s`：会越过平台后翻倒，所以验收不能无脑延长时间。

因此当前候选只证明：在这个两段楼梯场景里，策略可以完成一次 0.72m 高度跨越。

它还不能证明：

- 能稳定完成真实建筑 3m 以上跨楼层楼梯。
- 能在导航闭环里按路径上楼。
- 能处理真实楼梯边缘、打滑、障碍物、定位漂移。
- 能迁移到真实机器狗。

## 后续门槛

进入完整导航演示前，还需要按顺序做：

1. `cmd_vel -> policy` 在多层楼梯场景里重复 10 次，统计通过率。
2. 加入不同速度、停止、转向、平台停留测试。
3. 接入 PathFollower，让导航输出速度，而不是固定 `linear_x=1.0`。
4. 接入 LocalPlanner，在楼梯附近生成可执行局部路径。
5. 接入完整 `Navigation -> CmdVelMux -> MuJoCo policy`，做 dry-run。
6. 最后再进入 `building_scene.xml` 的真实跨楼层路线测试。
