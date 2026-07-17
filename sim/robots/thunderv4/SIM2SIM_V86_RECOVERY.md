# Thunder V4 V86 Recovery Sim2Sim Contract

## Scope

This document covers the V86 MUJICA recovery policy exported from
`model_27000.pt` to ONNX.  It was trained in IsaacLab against `THUNDER_CFG_V4`
and the Thunder V4 URDF.  The MuJoCo check is a simulator transfer evaluation,
not a substitution of a legacy robot asset.

## Training Contract

| Item | V86 value |
|---|---|
| Task | `PoseRoughV4MujicaRecoveryFinalHeightLockV86Kp80Kd10P3O` |
| Environment class | `ThunderV4PoseMujicaRecoveryFinalHeightLockV86Kp80Kd10P3OEnvCfg` |
| Physics / policy timing | `0.005 s` physics, decimation `4`, `0.020 s` policy |
| Policy input | six 60-dimensional frames, step-major, ONNX shape `[1, 360]` |
| Default pose | FR/RL `[-0.10, -0.80, 1.80]`; FL/RR `[0.10, 0.80, -1.80]`; wheels `0` |
| Action scales | hip `0.125`, thigh/calf `0.25`, wheel velocity `5.0` |
| Wheel action term | raw action clipped to `[-3, 3]`, yielding target velocity `[-15, 15] rad/s` |
| Controller | leg `Kp=80`, `Kd=10`; wheel velocity `Kd=1` |
| Torque limits | legs `120 Nm`, wheels `17 Nm` |
| Hardware speed limits | legs `17.48 rad/s`, wheels `44 rad/s` |

The actor uses the V43-compatible `mujica_action_mean_scale=4.0` and runner
`clip_actions=4.0`.  The exported ONNX already produces this bounded action;
the MuJoCo player must not multiply it by four a second time.

## MuJoCo Transfer Rules

The dedicated player is
`tools/rl_eval/mujoco_v86_recovery_play_aligned.py`.  It:

1. resolves qpos, qvel, and actuator slots by V4 joint name;
2. uses the training `5 ms x 4` timing rather than the MJCF's general-purpose
   `1 ms` default;
3. writes torque through the named actuator slots;
4. uses root angular velocity transformed from world frame to the V4 base frame;
5. preserves contact physics while rendering collision primitives transparent;
6. applies a speed-torque roll-off only when a joint is already moving in the
   direction of commanded torque.  Braking torque is retained and qvel state is
   never force-clipped in the default run.

## Exact Evaluation

Run:

```powershell
$env:MUJOCO_GL = 'wgl'
python D:\inovxio\tools\rl_eval\mujoco_v86_recovery_play_aligned.py `
  --policy <results-root>\20260709_recovery_v86_mujoco\policy.onnx `
  --xml D:\inovxio\brain\lingtu\sim\robots\thunderv4\mjcf\thunderv4.xml `
  --out <results-root>\20260710_v4_mujoco_sim2sim_v86_5ms_exact `
  --duration 8 --cases side_left side_right supine prone --history-format step
```

The 2026-07-10 result folder is named
`20260710_v4_mujoco_sim2sim_v86_5ms_exact` under the local results root.

| Initial state | Final height | Uprightness | Last 2 s leg vel mean/peak | Last 2 s wheel vel mean/peak | Result |
|---|---:|---:|---:|---:|---|
| side left | `0.408 m` | `1.000` | `0.041 / 0.057 rad/s` | `0.131 / 0.186 rad/s` | recovers |
| side right | `0.408 m` | `1.000` | `0.044 / 0.061 rad/s` | `0.134 / 0.191 rad/s` | recovers |
| supine | `0.409 m` | `1.000` | `0.035 / 0.049 rad/s` | `0.115 / 0.177 rad/s` | recovers |
| prone | `0.201 m` | `-0.962` | `6.388 / 13.787 rad/s` | `14.990 / 15.840 rad/s` | fails |

The legacy file names `supine` and `prone` are misleading. Both start fully
inverted with uprightness near `-1`: `supine` is a 180-degree roll around the
body X axis, while `prone` is a 180-degree pitch around the body Y axis. The
evaluation therefore shows successful roll-axis inversion recovery and failed
pitch-axis inversion recovery; it is not a conventional supine-versus-prone
comparison.

`isaac` term-major history was deliberately tested on side-left and supine.
Both remained inverted, while step-major recovered.  This is behavioral
evidence that the exported policy consumes the existing step-major layout.

## Remaining Gap

The transfer is normal for side-left, side-right, and supine resets, but it is
not full recovery coverage: prone remains inverted and high-motion.  Do not
call V86 a four-pose sim2sim success until the prone reset distribution and
contact behavior are traced against an Isaac rollout using the same reset
state.  The strict V86 final gate is also tighter than the stable MuJoCo result:
height `>= 0.38 m`, joint error `<= 0.24 rad`, leg velocity `<= 0.05 rad/s`,
wheel velocity `<= 0.08 rad/s`, orientation error `<= 0.045 rad`.
