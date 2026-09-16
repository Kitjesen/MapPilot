# Thunder Flat G2 `model_4998` 部署说明

## 1. 结论与状态

这是从 G1 `model_999.pt` 全状态续训 4000 轮得到的第二阶段横移专项模型。checkpoint 与 ONNX 的结构、数值一致性导出均通过，但历史固定协议评估显示横移落脚峰值恶化到约 `598–617 N`。

**状态：固定评估失败，禁止真机部署。**

G2 只适合作为离线研究样本，不能因为累计轮数更多而替换 G1/S4。它也没有单向 hip 防内夹或运行时 target clamp。

- 任务：`DOSO-Isaac-Velocity-Flat-Thunder-Terrain-Inward-v0`
- run：`2026-09-01_23-27-27_lateral_smooth_s4_7996_g2_v2`
- 开始：`2026-09-01 23:27:27 CST`
- 最终 checkpoint：`2026-09-02 00:32:58 CST`
- 阶段用时：约 `65 分 31 秒`
- 训练量：在 G1 后继续 `4000` 轮，4096 env，seed `42`，learning rate `2e-4`
- 继承方式：完整恢复 G1 actor、critic、optimizer 与 iteration
- actor 训练谱系：约 `33,000` 次更新（20k 基线 + 8k post20k + 1k G1 + 4k G2）
- policy：单帧 `53 -> 512 -> 256 -> 128 -> 16`，ELU，无 observation normalization
- ONNX：opset 11，动态 batch；输入 `obs [batch,53] float32`，输出 `actions [batch,16] float32`

## 2. 相对 G1 修改了什么

G2 同时加强了随机化和多个奖励，不能把结果直接归因于 action-rate 或任意单项：

| 项目 | G1 | G2 | 变化倍数/方向 |
| --- | ---: | ---: | --- |
| payload | `0–2 kg`，70% 零载荷 | `0–4 kg`，50% 零载荷 | 更重、更常见 |
| 其他刚体质量缩放 | `[0.90,1.10]` | `[0.85,1.15]` | 范围扩大 50% |
| reset 外力 | `±5 N` | `±10 N` | 2× |
| reset 力矩 | `±1.5 Nm` | `±3 Nm` | 2× |
| interval push XY | `±0.2 m/s` | `±0.3 m/s` | 1.5× |
| flat orientation | `-1.0` | `-2.0` | 2× |
| 腿加速度 | `-3e-7` | `-5e-7` | 1.67× |
| 轮加速度 | `-5e-10` | `-1e-9` | 2× |
| hip 绝对角惩罚 | `-5.0` | `-20.0` | 4×；阈值仍 `0.28 rad` |
| mirror | `-0.10` | `-0.15` | 1.5× |
| action-rate | `-0.015` | `-0.020` | 1.33× |
| wheel slip | `-0.05` | `-0.20` | 4× |
| 模式化 clearance | `-5.0` | `-100.0` | 20× |
| 最少 2 轮接触 | `-0.5` | `-1.0` | 2× |

命令混合保持为站立 10%、纯 yaw 20%、纯横移 30%、混合 40%；摩擦、hip reset 和训练 Kp/Kd 也不变。

## 3. 固定协议评估结果

评估时间：`2026-09-02 00:58:39 CST`。条件：seed `20260901`，每场景 256 env，warmup 200 step，采样 600 step，payload `0 kg`，Kp/Kd `90/6.93`，关闭 domain randomization。

| 场景 | G1 接触峰值 | G2 接触峰值 | 增幅 | G1 总 RMSE | G2 总 RMSE | 增幅 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 左横移 `+0.6 m/s` | `272.15 N` | `616.53 N` | `+126.543448%` | `0.03614` | `0.06417` | `+77.56%` |
| 右横移 `-0.6 m/s` | `286.14 N` | `597.57 N` | `+108.839435%` | `0.04408` | `0.06496` | `+47.38%` |

G2 横移实际主轴速度为 `+0.5520/-0.5496 m/s`，接触 duty 降为 `0.5696/0.5766`，late hip abs max 仍为 `0.3085/0.3107 rad`。顶层 `passed=false` 的直接触发项是右横移 `late_hip_stuck_fraction=0.061249997`，超过恢复门槛 `0.05`；`616.53/597.57 N` 接触峰值则同时独立违反既定 `450 N` 质量门槛。它虽然改善了零命令主漂移（`0.003303 -> 0.000598 m/s`）和左 yaw 跟踪（`0.5487 -> 0.6073 rad/s`），仍不具备部署价值。

由于 G1→G2 同时修改了 13 组因素，这一对历史样本只证明“G2 组合与重踩强关联”，不能证明是 action-rate、clearance、slip、hip 或某一项单独造成。根因必须依靠等训练量、同 parent、同 seed 协议的单因素/消融实验判断。

## 4. 导出验证与完整性

- checkpoint SHA256：`0c796d666994bcf3faae3389e0f9b134851ffd265d4ad4dd33bd428e602ef0a0`
- ONNX SHA256：`72736f3a59902801535ea7e26032b66b34c631495c1744676ff444a73dd39065`
- 验证 batch：`1 / 3 / 17 / 256`，共计算 277 组（前缀重复）的 contract-shaped 输入
- 最大绝对误差：`1.52587890625e-05`
- 平均绝对误差：`1.4850770639895927e-06`
- 硬门槛：最大绝对误差 `<= 5e-5`，通过

导出器同时绑定 checkpoint、`agent.yaml`、`env.yaml` 的 SHA256，并把观测布局、动作关节顺序和 scale 写入 ONNX metadata。数值一致性只证明 ONNX 忠实复现 checkpoint actor；它不会把失败的控制策略变安全。导出记录见 `exported/export_verification.json`，全部源文件与原始归档 hash 见 `PROVENANCE.json`。

在包目录执行 `python verify_package.py` 可核对 checkpoint、ONNX、YAML、训练/评估记录、归档和源快照。

## 5. 包内容

| 文件 | 用途 |
| --- | --- |
| `exported/policy.onnx` | 53→16 原始 actor；仅供离线分析，不含安全 clamp |
| `model_4998.pt` | 原始 RSL-RL checkpoint |
| `params/env.yaml` / `params/agent.yaml` | 训练时解析后的环境与 PPO 真源 |
| `exported/export_verification.json` | IO、hash 与 PyTorch/ORT 误差 |
| `export_g2.py` | CPU-only 可重复导出及验证脚本 |
| `verify_package.py` | 仅依赖 Python 标准库的整包 hash 校验 |
| `evaluation/` | 历史固定评估 JSON、CSV、日志 |
| `training/` | 启动脚本、训练日志、进度记录 |
| `PROVENANCE.json` | 时间、谱系、hash、核心指标与禁止部署状态 |
| `source_snapshot/` / `source_snapshot.tar.gz` | 训练源代码快照 |
| `remote_raw_artifacts.tar.gz` | 从 `44087` 服务器回传的原始 run 归档 |
| `git/doso-train.diff` | run 保存的工作树记录；不是完整源码 patch |

包内已恢复精确原始启动脚本 `evaluation/doso_eval_smooth_original.sh`（SHA256 `326656afd6802adda850b75e1a3ca5e8419ebb233136c5687929315784da318c`），可确认 G2 的固定评估调用。评估器本身当时未被 Git 跟踪、也未记录执行时源码 hash；评估前 staging 快照 `evaluate_torque_tracking_recovered_pre_eval.py` 早于评估且输出 schema 与历史 JSON 一致，是高置信度候选，但不冒充形式化的字节级原件。完整边界见 `evaluation/REPRODUCTION.md`。

## 6. 观测与动作契约

53 维观测顺序：base-frame angular velocity 3 维（×0.25）、projected gravity 3 维、速度命令 3 维、12 个腿关节相对默认位置、16 个关节速度（×0.05）、上一周期网络原始 action 16 维。

没有 4 个轮关节位置槽位。动作顺序是：

```text
FR hip/thigh/calf, FL hip/thigh/calf,
RR hip/thigh/calf, RL hip/thigh/calf,
FR/FL/RR/RL wheel
```

腿位置 target：`q_default + action * scale`，hip scale `0.125`，thigh/calf `0.25`；轮速度 target：`action * 5.0`。训练 policy/control 为 50 Hz，腿 Kp/Kd `90/6.93`，轮 Kp/Kd `0/1`。

以上契约用于可复现实验和离线回放，不是对 G2 真机部署的授权。

## 7. 明确禁止事项

- 不得把 `exported/policy.onnx` 接入真机动作链路或落地使能。
- 不得把 `hip_abs_limit_l2` 当成单向机械限位；它只是双向软奖励。
- 不得仅凭 G2 更新更多、站立漂移更小或 yaw 更好就替换 G1/S4。
- 不得用 G1/G2 两点比较给单个奖励项定责。
- 不得在缺少逐腿符号验证、物理 target clamp、关节/电流/力矩门控、watchdog 与急停时运行任何同谱系模型。

若未来只为故障复现实验加载 G2，也应限定在隔离仿真，并保留其原始 hash 与 `passed=false` 标签。
