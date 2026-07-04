# ThunderV4 MuJoCo Policy 运控基线

更新时间：2026-07-04

## 当前状态

状态：`SMOKE_PASS / DEMO_REJECTED`

含义：

- ThunderV4 MuJoCo policy 运控可以站立、前进、转向，基础 smoke 指标通过。
- 当前视频不能作为高质量产品演示，上一版远景视频已拒绝。
- 新增的近景视频能看清模型，但展示场景仍有材质/地面干扰，仍不作为最终产品效果。
- 下一步必须对齐原始 ThunderV4 demo 的控制细节，并重做专用录制工具。

## 结论

ThunderV4 在 MuJoCo 中的强化学习运控可以作为下一阶段调试对象，但还不能声明为高质量运控演示基线。

当前已验证：

- 使用真实 ThunderV4 MJCF 模型：`sim/robots/thunderv4/mjcf/thunderv4.xml`
- 使用 ThunderV4 策略文件：`sim/robots/thunderv4/policy/pose_flat_low_kpkd_microterrain_model29600_policy.pt`
- MuJoCo 驱动模式：`drive_mode=policy`
- 策略加载成功，实际加载器是 `TorchScriptPolicyRunner`
- 站立、前进、较长时间原地转向均通过
- 四足接触稳定，没有机身触地
- 这些测试全部是仿真，不会向真实机器狗发送控制命令
- `sim/scripts/policy_nav_smoke.py` 已修正策略后端字段，`.pt/.pth/.jit` 会报告为 `torchscript`

## 范围

本页只记录 MuJoCo 中 ThunderV4 强化学习运控本身，不混入导航链路。

不包含：

- OctoPlanner3D 全局规划
- LocalPlanner 局部规划
- PathFollower 路径跟踪
- DDS / Gateway / 真机驱动
- 真实机器狗物理运动

后续导航接入前，必须先以本页作为运控基线：先确保 policy gait 稳，再把导航输出的 `cmd_vel` 接进 policy。

## 资产

| 项目 | 路径 |
| --- | --- |
| ThunderV4 MJCF | `sim/robots/thunderv4/mjcf/thunderv4.xml` |
| Policy | `sim/robots/thunderv4/policy/pose_flat_low_kpkd_microterrain_model29600_policy.pt` |
| MuJoCo 驱动 | `src/drivers/sim/mujoco/driver.py` |
| MuJoCo 引擎 | `sim/engine/mujoco/engine.py` |
| Policy runner | `sim/engine/mujoco/robot_controller.py` |
| Smoke 脚本 | `sim/scripts/policy_nav_smoke.py` |

## 验收结果

| 模式 | 指令 | 时长 | 结果 | 关键数据 |
| --- | --- | ---: | --- | --- |
| stand | `vx=0.0, wz=0.0` | 4s | PASS | 漂移 `0.00045m`，四足接触，机身未触地 |
| walk | `vx=0.15, wz=0.0` | 4s | PASS | 前进 `0.4625m`，平均速度约 `0.116m/s` |
| turn short | `vx=0.0, wz=0.4` | 4s | 未达阈值 | 转角 `0.2516rad`，稳定但转向响应偏弱 |
| turn long | `vx=0.0, wz=0.45` | 6s | PASS | 转角 `0.5912rad`，漂移 `0.0465m` |

站立、前进、转向都没有出现 NaN / Inf。脚接触统计显示四个足端均参与支撑，`non_foot_ground_contacts=0`。

## 录制

Policy gait 视频：

`artifacts/mujoco_thunderv4_policy_gait.mp4`

该视频已拒绝，原因：

- 相机太远，机器狗太小。
- 使用 `open_field` 旧测试场，墙体和障碍物干扰画面。
- 混入导航小窗和文字，不适合评估 gait。
- 低速 `vx=0.15` 不适合作为效果展示。

新的近景候选视频：

`artifacts/mujoco_thunderv4_policy_gait_showcase_vx08_clean.mp4`

该视频只作为临时检查，不作为最终验收视频。

对应摘要：

`artifacts/mujoco_thunderv4_policy_gait.json`

视频摘要：

- `drive_mode=policy`
- `policy_loaded=true`
- 前进距离：`0.5818m`
- `z_min=0.3930`
- `z_max=0.4788`
- `roll_abs_max=0.0050`
- `pitch_abs_max=0.0137`
- `lidar_points_in_inset=724`
- `pass=true`

## 执行命令

```powershell
$env:PYTHONPATH='src'; python sim\scripts\policy_nav_smoke.py --world open_field --direct-only --direct-mode stand --direct-duration 4 --max-stand-drift 0.05 --json-out artifacts\mujoco_policy_stand_current.json
```

```powershell
$env:PYTHONPATH='src'; python sim\scripts\policy_nav_smoke.py --world open_field --direct-only --direct-mode walk --direct-duration 4 --linear-x 0.15 --min-direct-motion 0.20 --json-out artifacts\mujoco_policy_direct_current.json
```

```powershell
$env:PYTHONPATH='src'; python sim\scripts\policy_nav_smoke.py --world open_field --direct-only --direct-mode turn --direct-duration 6 --angular-z 0.45 --min-turn-yaw 0.35 --max-turn-drift 0.18 --json-out artifacts\mujoco_policy_turn_6s_current.json
```

## 当前判断

ThunderV4 MuJoCo policy 运控可以继续作为导航接入目标，但在接导航前必须先完成运控演示质量整改。

下一步不应该直接把完整导航塞进来，而应分两段：

1. `cmd_vel -> MuJoCo policy -> ThunderV4 运动`
2. `Navigation -> PathFollower -> CmdVelMux -> cmd_vel -> MuJoCo policy`

这样可以区分问题来源：如果第一段失败，是运控策略/模型/坐标问题；如果第二段失败，是导航输出、速度限幅、路径跟踪或安全层问题。

## 已知问题

- 当前 MuJoCo 展示场景质量不达标：地面材质、灯光、相机、文字叠层都需要重做。
- `open_field` 不能再用于 policy gait 展示，只能作为老传感器/障碍测试场。
- 需要写专用 `record_thunderv4_policy_gait.py`，不要复用导航录制脚本。
- 需要逐项对齐 `sim/robots/thunderv4/mujoco_him_keyboard.py` 原始 demo 的 IMU、action、warmup、命令范围和轮/足控制逻辑。
- 4 秒短转向样本响应偏弱，转向验收建议使用 `6s, wz=0.45` 作为当前基线。
- 当前视频是 policy gait 直连速度指令，不是完整导航闭环视频。
- 当前结果仍是仿真，不代表真实机器狗已经移动。

## 后续代码修正

已修正：

- `sim/scripts/policy_nav_smoke.py` 的 `policy_backend` 字段现在根据策略文件后缀报告：
  - `.pt/.pth/.jit` -> `torchscript`
  - `.onnx` -> `onnx`
  - 未加载 -> `unloaded`
- `.pt` 策略 metadata 不再误走 ONNXRuntime 解析，因此不会再输出误导性的 `onnx_error`。
