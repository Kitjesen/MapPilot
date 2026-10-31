# 人物识别、跟踪与遮挡恢复：开发参考

核对日期：**2026-10-20**。范围：2024–2026 年与 LingTu 人物跟随直接相关的工作，重点补充 2025–2026 年的新结果。本文是论文与源码对照、开发依据及待验证方案；**不是新模型实测，也不是功能已经实现的声明**。

历史回放、已复现反例和经典基线见 [person-follow.md](person-follow.md)。本文维护较新的论文依据与实施映射；后续实现、PR 和实验记录可引用下面的稳定编号，如 `SEL-01`、`MEM-01`、`DEV-03`。每次引用同时写清采用了哪一项机制，不能只写“参考某论文”。

上层用户目标、对话功能范围和端到端验收见 [对话控制与架构计划](plan.md)。本文负责人物观测/身份改进，不将算法回归通过等同于“对话后即可可靠跟随或搜寻”的整机能力。

## 1. 开发判断

近期优先修复已经确认的实现错误，然后分别比较关联、身份特征、参考更新和定位。**没有证据表明必须先训练新的 VLA，才能改善当前链路。** 新论文提供了更合适的参照，但不替代已有反例和本机评测。

2026-10-20 按用户确定的近期范围：**不采用需要 V100 级资源的方案，先修改现有代码，再选择可独立接入的核心算法。** COPE 等仅有论文 GPU 测量、尚无选定目标平台证据的新增模型退出近期接入清单，保留文献机制参考。RexSeek 也不列为当前部署依赖；它是 IDEA Research 的《Referring to Any Person》（ICCV 2025）中的描述选人模型，3B/7B 表示约 30 亿/70 亿参数，不承担现有完整跟随链。

必须区分以下问题：

| 问题 | 输出应表达什么 | 不能由什么直接推出 |
| --- | --- | --- |
| 检测 | 画面中有哪些人及候选框 | 检测为 person，不代表是用户要找的人 |
| 描述选人 | 符合指令的零个、一个或多个候选 | 画面只有一个人，不代表符合描述 |
| 短时关联 | 当前框与哪条轨迹连续 | 三帧位置连续，不单独证明长期身份相同 |
| 身份确认 | 是否与原始目标及可信参考一致 | 相同衣服、mask 连续、余弦高分不等于身份保证 |
| 当前定位 | 有效源时间下的当前三维观测 | 历史保留坐标和内部预测不是新观测 |
| 跟随和搜索 | 可提交给原生导航评估的目标候选 | 看见人或模型生成 waypoint，不代表允许运动 |

本项目保留现有原始身份锚点、经核验多视角、UNKNOWN/OTHER、来源时间和上下文隔离。修改应接到现有 owner，不新增并行的身份系统、地图 owner 或运动控制入口。原生三维规划、碰撞、地面支撑、运动约束与取消事务继续生效。

本文中“论文事实”来自原文；“作者实现”来自公开代码的指定版本；“LingTu 建议”是尚需验证的工程判断。未核实的发布、权重、许可或平台性能明确写未知。免下游训练不等于没有预训练模型、不用加速器或 CPU 实时。

## 2. TrackVLA++ 是否训练、是否需要 GPU

**VLA-01 — TrackVLA++: Unleashing Reasoning and Memory Capabilities in VLA Models for Embodied Visual Tracking**。arXiv v1：2025-10-08；[作者目录](https://pku-epic.github.io/)列 ICRA 2026。[原文](https://arxiv.org/html/2510.07134v1)、[项目页](https://pku-epic.github.io/TrackVLA-plus-plus-Web/)。

| 项目 | 已核实事实 | 对我们意味着什么 |
| --- | --- | --- |
| 作者训练 | 基于 NavFoM，含 Qwen2-7B、SigLIP 和 DINOv2；§V-A 使用 8 张 H100 约一天，共 192 GPU 小时 | 确实训练了模型；这不是使用者每次部署都需重训的费用 |
| 实机推理 | Go2 四路 RGB 发往远端 RTX 4090，返回推理 token 和 waypoint | 论文实测方案依赖 GPU；GPU 在远端，未要求机器人必须内置同卡 |
| 速度 | §V-B/Table III 的 4.8 FPS 是视觉识别对比实验结果 | 不把它当含采集、网络、导航与执行的机器人端到端频率 |
| 身份记忆 | TIM 为 4 tokens，按推理 token 熵控制更新，invalid 时冻结 | 可以借鉴可信更新原则；DINO 参考图库不是 TIM 的完整复现 |
| 发布及平台 | 本轮未核实到 ++ 的可用 checkpoint 入口，也没有目标 CPU/BPU/NX 实时证据 | 前作 TrackVLA 的仓库不能作为 ++ 权重已发布的证据 |

判断：复现整套时，应按 GPU 方案预算。不能断言其任何实现都绝不可能在 CPU 运行，但目前没有支持 CPU 实时部署的证据。近期只迁移可独立验证的身份记忆原则，VLA 动作头不接管 LingTu 原生导航。

### 2.1 速度和新增算力补充

CUDA 是作者选用的 NVIDIA 加速接口；“官方示例使用 CUDA”本身不能推出“大模型”“必须大显卡”或“算法不能在 CPU 实现”。下表的结果来自不同任务、模型和硬件，不能直接作为排名。FPS 是相应测量范围内的吞吐率，其倒数不能直接当作单请求或机器人端到端延迟。

| 方法 | 已公开速度与平台 | 计时边界及算力判断 |
| --- | --- | --- |
| RexSeek | 本轮未在作者论文/补充、README 或 3B model card 核实到官方可复现 FPS | 官方 3B 与论文 7B 都不能凭版本号推速度。TrackVLA++ Table III 对 RexSeek 报 1.1 FPS，是另一团队的识别对比，具体版本/硬件范围不完整，不套用到发布的 3B。自回归生成和外部 proposals 都有成本 |
| TrackVLA++ | Table III 识别对比 4.8 FPS；实机部署远端 RTX 4090 | 7B VLA 属于需要单独算力预算的候选；表中 FPS 不代表完整实机链 |
| COPE | 单 V100、256×128 人体 crop；Occluded-Duke 整库无 PSS 42.30 秒、含 PSS 51.42 秒 | 论文与作者实现测的是整库特征提取、检索及评测，当前实现 test batch=256；不是 batch=1 响应时间，不换算成直播 FPS。图像编码与整库后处理的成本需分开测 |
| TrackTrack | MOT17 161.52 FPS、DanceTrack 408.37 FPS；i7-11700K＋RTX 3090 实验环境 | 只计跟踪关联，检测/特征预加载，不能说完整 GPU 系统有该帧率。关联策略没有新增神经网络，适合先复用已有检测与特征做 CPU 对照 |
| OA-SORT / BoostTrack++ | 前述核对未取得可用于本项目的完整实时帧率 | 新增机制主要是几何、运动不确定度、遮挡关联，不要求新增大语言模型；完整 detector/ReID 的开销仍然存在 |
| SENTRY＋SAM2-L | A100：32.8 FPS、5.7 GB；同表 SAM2-L 为 44.0 FPS、5.1 GB | 完整方案有分割及反向验证成本；借鉴“写入前验证”不等于必须引入整套 SAM2。输入分辨率未核定，不能外推板端速度 |
| SAM2Long，P=3 | RTX 3090 24 GB：19 FPS、5.3 GB | 有界多路径增加计算；最终视频路径选择另有非因果边界，不作为在线跟随速度承诺 |
| EdgeTAM | 1024×1024：A100/torch.compile 150.9 FPS；iPhone 15 Pro Max/CoreML CPU＋NPU 15.7 FPS | 有移动加速器证据；手机数字不是纯 CPU，也不是任意 NPU 的结果。公开 CoreML 示例与完整 video memory 的区别仍见 MASK-01 |

表中来源分别见 VLA-01、SEL-01、REID-01、MOT-01/02/03、MEM-01/02 和 MASK-01；RexSeek 的外部测速见 [TrackVLA++ Table III](https://arxiv.org/html/2510.07134v1)。COPE 的时间见 [正式论文 Table 4/7](https://openaccess.thecvf.com/content/CVPR2026/papers/Sun_COPE_Consistent_Occlusion_and_Prompt_Enhancement_Network_for_Occluded_Person_CVPR_2026_paper.pdf)及 [processor_test.py](https://github.com/Cecoming/COPE/blob/13ba061721953653f4aba078d19354f181ea63b0/cope/processor_test.py#L30-L54)。

LingTu 近期算力安排属于工程建议，尚非板端性能结论：检测、短时关联和有效位置更新持续运行；身份参考复用已算特征，按可信条件提交；较重的语义/身份复核按初始化、歧义或重入事件触发，并保留必要的周期审查。跳帧或降低复核频率会改变错人风险，必须与覆盖率和恢复时延一起评测。SENTRY 反向分割、人体 mesh、生成式姿态增强及 7B VLA 各自独立做离线候选，不在首轮全部叠入运行链。

## 3. 描述选人与人物身份

### SEL-01 — Referring to Any Person / RexSeek / HumanRef

**ICCV 2025**。[正式论文](https://openaccess.thecvf.com/content/ICCV2025/html/Jiang_Referring_to_Any_Person_ICCV_2025_paper.html)、[作者代码](https://github.com/IDEA-Research/RexSeek)、[发布的 RexSeek-3B](https://huggingface.co/IDEA-Research/RexSeek-3B)。

- 论文事实：按描述选择零个、一个或多个实例；HumanRef 含目标不存在的 Rejection 子集。它比只报告 Rank-1 的检索基线更贴近“红衣人不在画面里”的问题。
- 训练与部署：作者做了多阶段训练；仓库发布 3B，论文主结果包含 7B，不能混用其成绩。[作者运行代码](https://github.com/IDEA-Research/RexSeek/blob/b85d8e26a29790cf843e4ea5fe475938b37fc0e4/tests/test_local_load.py#L55-L70)走 CUDA；未找到机器人 CPU/NPU 实时证据。loader 有 device/量化选项，不能据 demo 断言模型逻辑必需 CUDA；官方 3B/7B 的可复现速度本轮未取得。
- LingTu 建议：首先修掉唯一候选绕过描述的分支，再加入目标缺席、同属性多人及关系描述反例。空候选是正常结果；单帧选人不承担后续长期身份保持。
- 复用边界：[IDEA License 1.0](https://github.com/IDEA-Research/RexSeek/blob/b85d8e26a29790cf843e4ea5fe475938b37fc0e4/LICENSE#L7-L13)仅授非商业研究，商业需另行授权。近期优先借鉴拒识任务与评测设计。

### SEL-02 — Tackling Alignment Ambiguity in Person Retrieval through Conversational Attribute Mining（CECA）

**CVPR 2026**。[正式论文](https://openaccess.thecvf.com/content/CVPR2026/html/Zou_Tackling_Alignment_Ambiguity_in_Person_Retrieval_through_Conversational_Attribute_Mining_CVPR_2026_paper.html)、[作者代码](https://github.com/sugelamyd123/CECA)。

- 论文事实：通过 MLLM 对话挖掘服装、鞋、背包等属性，进行细粒度图文对齐，并降低低质量回答的训练权重；使用 Qwen2-VL-7B-Chat 与 CLIP ViT-B/16。
- 部署陷阱：不能说“只训练时用大模型、推理纯 CLIP”。[当前评测实现](https://github.com/sugelamyd123/CECA/blob/5926087dbd2a547b5e322504fd25d5011dcd8c09/2026-CVPR-CECA/utils/metrics.py#L79-L87)仍读取每张图库图像的属性文本，以 0.7 图像＋0.3 文本融合。直播新 crop 取得属性文本的成本不能隐藏。
- 发布情况：本轮未看到可用 checkpoint 链接；实现依赖 CUDA，README 残留合并冲突与本机路径。README 自述 Apache 2.0，但未核实独立 LICENSE。没有 CPU/NPU 实时证据。
- LingTu 建议：先表达属性的“支持、矛盾、不可见”，避免把不可见当明确不符；完整模型仅作后续离线候选。其 Rank-k/mAP 不证明缺席拒识或长期跟踪。

### REID-01 — COPE: Consistent Occlusion and Prompt Enhancement Network for Occluded Person Re-identification

**CVPR 2026**。[正式论文](https://openaccess.thecvf.com/content/CVPR2026/html/Sun_COPE_Consistent_Occlusion_and_Prompt_Enhancement_Network_for_Occluded_Person_CVPR_2026_paper.html)、[作者代码及权重入口](https://github.com/Cecoming/COPE)。

- 论文事实：训练时给不同身份加入相同遮挡，减少共同遮挡物导致的假相似；以前景增强减少背景干扰，另有 prompt 相似度修正。完整方法需人物 ID 与人体 parsing 监督。
- 训练与部署：CLIP ViT-B/16，作者训练使用 V100；已有 trained weights 入口可免本地重训，但本轮未下载。[作者推理实现](https://github.com/Cecoming/COPE/blob/13ba061721953653f4aba078d19354f181ea63b0/cope/processor_test.py#L22-L38)使用 CUDA。没有机器人 CPU/NPU 延迟证据；README 图库总耗时不是单帧 FPS。
- 重要边界：作者额外发布免训练 NPSS 邻域后处理，但它不是拒识器。[评测代码](https://github.com/Cecoming/COPE/blob/13ba061721953653f4aba078d19354f181ea63b0/utils/metrics_NPSS.py#L64-L69)跳过 query 身份不在 gallery 的样本，Rank-1 不证明缺席场景可靠。未核实仓库 LICENSE，不能默认可商用。
- LingTu 建议：作为遮挡/背景污染的主要新参照。先比较整框 DINO 与可见人体区域；图库更新考虑遮挡、模糊和背景比例。NPSS 如做离线比较，只使用已核验身份的参考，避免现场路人互相增强。

### REID-02 — From Poses to Identity: Training-Free Person Re-Identification via Feature Centralization（Pose2ID）

**CVPR 2025**。[正式论文](https://openaccess.thecvf.com/content/CVPR2025/html/Yuan_From_Poses_to_Identity_Training-Free_Person_Re-Identification_via_Feature_Centralization_CVPR_2025_paper.html)、[作者代码](https://github.com/yuanc3/Pose2ID)。

通过同身份姿态生成和互近邻特征中心化改善检索。“免训练”指不重新训练用于特征增强的 ReID 底座；姿态生成器仍经过训练（L20 48 GB，约 20 小时），作者已有权重，运行实现含 CUDA。许可需连同生成底座分别核对。

LingTu 近期不把生成的人像写成可信身份观测。可先借鉴真实多视角特征汇聚；互近邻增强在少人、同衣着、目标缺席场景可能强化错误。它不能替代开放集拒识。

## 4. 短时多目标关联与有偏观测

### MOT-01 — Focusing on Tracks for Online Multi-Object Tracking（TrackTrack）

**CVPR 2025**。[正式论文](https://openaccess.thecvf.com/content/CVPR2025/html/Shim_Focusing_on_Tracks_for_Online_Multi-Object_Tracking_CVPR_2025_paper.html)、[作者代码](https://github.com/kamkyu94/TrackTrack)。

高分、低分及 NMS 丢弃的高分框共同竞争，轨迹参与重复初始化抑制。关联本身不需新训练，但论文整体使用在目标数据集训练的 YOLOX-X、FastReID。完整复现还需 NMS 前候选。

论文 Table 8 的 161.52/408.37 FPS 不能当端到端：RTX 3090＋i7-11700K；[run.py](https://github.com/kamkyu94/TrackTrack/blob/ee7f1c5fcbdcac48ed8bfab38d52c0006bf304da/3.%20Tracker/run.py#L58-L66)在预加载检测和特征后只计 tracker update，排除检测、特征与后处理。对 LingTu 先修弱框入口、恢复旧轨迹与新建轨迹的语义，再做固定输入关联对照。

### MOT-02 — BoostTrack++: using tracklet information to detect more objects in multiple object tracking

**Filomat 39(16), 5685–5702, 2025**；arXiv v2：2025-08-20，首发于 2024。[正式论文](https://www.pmf.ni.ac.rs/filomat-content/2025/39-16/39-16-21-25196.pdf)、[原文 v2](https://arxiv.org/html/2408.13003v2)、[作者代码](https://github.com/vukasin-stanojevic/BoostTrack)。

以形状、运动不确定度和缓冲 IoU 提供软支持，避免单用 IoU 抬分导致新 ID/切换。新增机制不需训练网络；完整基准仍用已训练检测器、ReID、相机补偿及离线 GBI。

作者硬件为 i9-13900K＋RTX 3080；[main.py](https://github.com/vukasin-stanojevic/BoostTrack/blob/fb5bfc3a8f067476565e753b3a73df4d757c9d03/main.py#L98-L108)在 detector 之后开始 tracker 计时。不能据此声称 CPU/NX 全链实时。LingTu 可比较轨迹支持下的弱框恢复，保留原始检测分数；已被 detector 删掉的框无法靠关联恢复。离线 GBI 不用于在线运动决策。

### MOT-03 — Occlusion-Aware SORT: Observing Occlusion for Robust Multi-Object Tracking（OA-SORT）

**CVPR 2026**；arXiv v2：2026-03-09。[正式论文](https://openaccess.thecvf.com/content/CVPR2026/html/Li_Occlusion-Aware_SORT_Observing_Occlusion_for_Robust_Multi-Object_Tracking_CVPR_2026_paper.html)、[原文](https://arxiv.org/html/2603.06034v2)。

估计遮挡并修改关联与 KF 更新，减少部分框偏移污染状态；无需额外 tracker 训练，仍依赖已训练检测器。作者实验用 V100＋Xeon 6130；本轮未核实作者仓库、可迁移 FPS 或机器人 CPU/NPU 实测。

最有用的原则是**遮挡会令观测产生偏差，不能等权写入滤波和身份参考**。其框底边近远排序有平面地面与视角前提；起伏相机、楼梯和多层地面不满足直接套用条件。LingTu 应用真实 RGB-D、姿态及可见人体区域作证据。

## 5. 参考更新、mask 与长期恢复

### MEM-01 — SENTRY: SAM2-Enhanced Neighbor-Aware and Temporally Reasoned Memory for Visual Tracking

**ECCV 2026**（arXiv 与作者页标注）；arXiv v2：2026-06-24。[原文](https://arxiv.org/html/2606.24449v2)、[项目](https://hamadya.github.io/SENTRY/page/)、[作者代码](https://github.com/HamadYA/SENTRY)。

写入 memory 前将候选短窗反向跟踪，与目标及邻居轨迹检查循环一致性，避免单帧高分误写。无需重新训练 SAM2 底座。作者报告 A100 上 SAM2-L＋SENTRY 32.8 FPS、5.7 GB；本轮未核实该测速输入分辨率，不能作为本机完整 RGB-D 链时延。

发布版本需区别于论文概述：[固定 default.yaml](https://github.com/HamadYA/SENTRY/blob/dd4486c7eeadd7e7022854e29e95e3101390ce65/configs/sentry/default.yaml#L1-L25)实际为 `amg_enabled: false`、`association_mode: simple`、`memory_policy: severe_rescue`，不能说下载默认配置就复现全部论文。SENTRY 自身 Apache 2.0，捆绑 tracker 各有许可。

LingTu 建议：利用已有候选多帧和可信多视角，在提交新参考前检查时间、身份、可见区域和竞争者；UNKNOWN/OTHER 不更新。先验证该原则，只有出现现有证据无法区分的失败，再评估更昂贵的反向重跟踪。三帧候选自洽不是独立身份真值。

### MEM-02 — SAM2Long: Enhancing SAM 2 for Long Video Segmentation with a Training-Free Memory Tree

**ICCV 2025**；核对 arXiv v3：2025-07-29。[正式论文](https://openaccess.thecvf.com/content/ICCV2025/html/Ding_SAM2Long_Enhancing_SAM_2_for_Long_Video_Segmentation_with_a_ICCV_2025_paper.html)、[原文](https://arxiv.org/html/2410.16268v3)、[作者代码](https://github.com/Mark12Ding/SAM2Long)。

以有界多路径 memory 保留遮挡歧义，按质量和目标存在性剪枝，不额外训练 SAM2/2.1。Table 6：RTX 3090 24 GB，P=3 为 19 FPS、5.3 GB。最终最优路径在视频结束后选出，不能直接拿最终轨迹充当因果在线控制结果。多数代码 CC-BY-NC 4.0，原 SAM2 权重和训练代码的 Apache 2.0 不能覆盖它。

LingTu 可借鉴“歧义时延迟换身份”，或用于离线困难片段对照；不将所有假设写入可信图库，也不直接移植完整 memory tree。

### MASK-01 — EdgeTAM: On-Device Track Anything Model

**CVPR 2025**；arXiv 首发 2025-01-13。[原文](https://arxiv.org/html/2501.07256v1)、[作者代码](https://github.com/facebookresearch/EdgeTAM)、[权重](https://huggingface.co/facebook/EdgeTAM)。

轻量编码器＋2D Spatial Perceiver 压缩 memory，作者通过蒸馏训练模型，使用者可用预训练权重。论文 1024×1024 输入：A100/torch.compile 150.9 FPS，iPhone 15 Pro Max/CoreML CPU＋NPU 15.7 FPS。**存在无需 CUDA 的移动部署证据，但不是任意 NPU 或 LingTu 板端证据。**代码、权重 Apache 2.0。

开源 CoreML 示例只导出 image/prompt/mask 三模块；[逐帧示例](https://github.com/facebookresearch/EdgeTAM/blob/7711e012a30a2402c4eaab637bdb00a521302c91/coreml/inference_example.py#L190-L196)使用保存的点重新分割，不是完整 video memory 传播。其 synthetic timing 也不能替代论文 VOS 测速。

LingTu 可离线比较“可见 mask/人体部位→深度采样”，针对已知前景椅子污染。mask 连续不证明身份；有效深度、RGB-D 对齐、边界污染和跨区域一致性仍需验证。

### ROB-01 — Follow Everything: Goal-Aware Adaptation and Graph-Based Planning Towards Arbitrary Leader Following

核对 **arXiv v5，2026-06-06** 的 PDF/HTML 题名。[PDF](https://arxiv.org/pdf/2504.19399v5)、[HTML](https://arxiv.org/html/2504.19399v5)。摘要页仍显示旧副标题 *A Leader-Following and Obstacle Avoidance Framework with Goal-Aware Adaptation*；这是同一 arXiv 记录，不是两篇。v2 曾 withdrawn，后续已有 v3–v5，不能据此说整篇撤回。正式 venue 未核定。

距离分层图库为不同距离区间保留高置信参考，缓解近距离局部身体、出视野边缘 crop 污染整个记忆。实机使用 Intel i7＋RTX 3070 笔记本；没有 CPU 独立实时证据。对 LingTu 只比较距离/尺度与观测质量分层，保留原始锚点；不照搬其 costmap/graph planner 替换原生三维导航，也不把分割置信度当身份确认。

## 6. 丢失后搜索与评测

### SEARCH-01 — RPF-Search: Field-based Search for Robot Person Following in Unknown Dynamic Environments

arXiv v2：**2025-07-11**，记录标为 IEEE/ASME Transactions on Mechatronics 接收。[原文](https://arxiv.org/html/2503.02188v2)、[摘要与版本](https://arxiv.org/abs/2503.02188)、[作者项目](https://medlartea.github.io/rpf-search/)。

按墙角/地形遮挡和动态行人遮挡选择不同搜索策略；利用最后轨迹、地图可达路径及行人运动产生观察候选。原方法包含在线 SVR 拟合，不应称完全无学习。未核实 CPU/GPU 全链运行条件。

LingTu 借鉴“最后位置、预测、搜索区域分别表达”，搜索候选受真实时间与可达性约束；通过已有原生三维地图与规划预览。搜索中的邻居仅是遮挡/运动证据，不是允许换跟的目标；论文的随流或超越行为需要单独产品验证。

### SEARCH-02 — OA-NBV: Occlusion-Aware Next-Best-View Planning for Human-Centered Active Perception on Mobile Robots

**2026 预印本**，核对 arXiv v2：2026-09-10。[摘要与版本](https://arxiv.org/abs/2603.11072)、[原文](https://arxiv.org/html/2603.11072v2)。

根据人体完整性、图像占比和遮挡评价下一观察位。原实机 Orin 负责视点生成/导航，分割和人体 mesh 处理在远端 Ryzen 9＋RTX 4090 Laptop GPU，不能说整套板载独立运行。实机观察对象包含 mannequin，不能用其结果证明动态人物身份恢复。作者代码入口本轮未核实。

可借鉴视点效用，用于已有候选观察位选择；人体重建不是当前必须依赖。视点改善与身份确认分别验收，地面/高度图方案不替代本项目三维地面支撑与碰撞判断。

### BENCH-01 — TPT-Bench: A Large-Scale, Long-Term and Robot-Egocentric Dataset for Benchmarking Target Person Tracking

arXiv v2：**2025-07-09**。[原文](https://arxiv.org/html/2505.07446v2)、[项目/数据入口](https://medlartea.github.io/tpt-bench/)。该版本 comments 为 Under review；本轮未另核定后续正式发表状态。

48 序列、5.3 小时、571,982 帧目标框；包含里程计、LiDAR、IMU、全景与 RGB-D。人推传感器车跟人采集，适合移动视角、长时遮挡和重入；不是自主跟随成功证据。新增数据尚未下载、许可未完整核定，不能写进已下载清单或假定带三维人物真值。

建议用它补充现有 MOT17/本地 RGB-D。同衣着交叉可用 [DanceTrack](https://dancetrack.github.io/)，机器人自运动与二维/三维人体可参考 [JRDB](https://jrdb.erc.monash.edu/)；后两者入口及限制见历史研究记录。数据可得性、许可、标注适用性与本地路径在实际获取时另记。

## 7. 更新但暂不优先引入的工作

| 编号与工作 | 新结果及来源 | 不直接引入的具体原因 |
| --- | --- | --- |
| WATCH-01 DOA-SORT: Directional Occlusion-Aware Multi-Object Tracking with Distributional Observations | [arXiv v1，2026-09-19](https://arxiv.org/html/2609.22706v1)，方向遮挡、多观测假设和关联 margin | 未核实作者代码/硬件；局部验证中 HOTA 提高不总伴随 IDSW 降低，底边排序假设仍在。先比较较成熟的 MOT-01/03，不先移植五分量模型 |
| WATCH-02 AR2-4FV: Anchored Referring and Re-identification for Long-Term Grounding in Fixed-View | [CVPR 2026](https://openaccess.thecvf.com/content/CVPR2026/html/Yan_AR2-4FV_Anchored_Referring_and_Re-identification_for_Long-Term_Grounding_in_Fixed-View_CVPR_2026_paper.html)，长期离开/重入与恢复时延指标 | 明确 fixed-view/稳定背景假设；不适用于移动相机的背景锚点坐标。可借鉴恢复指标，未核实可直接运行的作者代码 |
| WATCH-03 SAM 3: Segment Anything with Concepts / SAM3.1 Object Multiplex | [SAM3 原文](https://arxiv.org/html/2511.16719v1)，2025-11；[3.1 官方发布](https://github.com/facebookresearch/sam3/blob/7f473cfd983d5eb43040ff608f66bff0d7bdcb53/RELEASE_SAM3p1.md)，2026-03-27 | 概念匹配可返回所有同描述目标，不证明唯一身份；3.1 的约 7 倍来自 H100/128 对象，不是单人端侧速度。[官方环境](https://github.com/facebookresearch/sam3/blob/7f473cfd983d5eb43040ff608f66bff0d7bdcb53/README.md#L69-L71)要求 CUDA 环境；权重需访问申请及自定义 SAM License |
| WATCH-04 SAMURAI: Motion-Aware Memory for Training-Free Visual Object Tracking With SAM 2 | [TIP 2026 DOI](https://doi.org/10.1109/TIP.2026.3651835)；[旧 arXiv](https://arxiv.org/html/2411.11922v2)题名不同；[作者代码](https://github.com/yangchris11/samurai) | 可参考运动辅助 mask 选择/记忆质量筛选。作者实现未提供 live/streaming 接口；本轮未核实可给定硬件与分辨率的测速。作者页面的月份与索引冲突，因此只引用 DOI 年份 |

## 8. 当前代码缺口到实施项的映射

源码位置以本次核对时的函数/模块为准，后续行号可能变化。下表记录审查时的缺口与最小修法，**实施状态以第 9 节为准**。纸面机制不构成修复完成。

| 编号 | 已有事实与缺口 | 最小改动方向 | 依据与验收反例 |
| --- | --- | --- | --- |
| DEV-01 描述选择 | [visual_servo.py](../../src/decision/modules/visual_servo.py) `_try_select_follow_target`：无视觉服务、无参考且唯一人物连续确认时，可绕过描述直接锁定 | 对含外观/关系约束的指令保持描述判断；支持零个和多个符合者 | SEL-01/02；指令红衣、仅蓝衣时不锁人、不发跟随目标；普通无属性“跟随这个人”的合法初始化单独保留 |
| DEV-02 首次位置与取消 | [person.py](../../src/decision/vision/person.py) 位置提交会把重置后的零占位混入 EMA；VisualServo 状态锁内同步编码阻塞取消 | 新上下文首次有效测量直接初始化；慢编码在锁外计算，回交检查现有 intent/context 代次 | 已复现实现反例，无需新论文；首测 x=10 不输出 x=4；受控阻塞期间取消仍能推进，过期编码结果不提交 |
| DEV-03 弱候选入口 | [backends.py](../../src/perception/backends.py) 默认 confidence=0.3；BPU 直接默认 0.25；[BPUTracker](../../src/perception/tracking/bpu_tracker.py) low=0.1，而 detector 已先过滤 | 区分候选、建轨与发布阈值；保留低分框供旧轨迹恢复 | MOT-01/02、ByteTrack；默认 `[0.1,0.25)` 入口缺口需修，显式 confidence=0.1 的配置不在此结论内；弱框不能任意独立建跟随目标 |
| DEV-04 时间与动态状态 | [native_byte_tracker.py](../../src/perception/tracking/native_byte_tracker.py) 每次 predict 按一步加 velocity；工作线程可能覆盖中间帧；三维 person 有静态融合问题 | 用实际 source_ts 推进运动和过期；区分原始观测、滤波状态及预测，人物不沿用静态累积均值 | MOT-01/03 是机制参考，真实 Δt 适配是本项目工程建议；不规则采样与掉帧测试，同轨迹稀疏采样不产生无理由高速/早过期；不把语义实例分裂说成 Follow 原始 track ID 自动分裂 |
| DEV-05 恢复身份 | [person.py](../../src/decision/vision/person.py) `_motion_reacquire` 在无 DINO 分支可仅凭唯一邻近候选三帧恢复；已有可信多视角与 UNKNOWN/OTHER 应保留 | 时间/运动提供候选，长期换轨另需身份依据；根据质量区分不确定与可靠否定 | SEL-01、REID-01、MEM-01；交叉后邻人占原位置不被确认，原人换视角/部分遮挡仍有恢复机会 |
| DEV-06 特征模型对照 | [DinoEncoder](../../src/perception/encoding/dino.py) 为通用 DINOv3；[reid.py](../../src/decision/vision/reid.py) 开发 OSNet 的 `model_path=""` 不能证明专用 ReID 权重已加载 | 固定候选、裁剪和初始化，对比明确训练来源的 checkpoint；记录真实预处理与后端 | REID-01/02；DINO 保留基线，不能预先宣称 OSNet 更好，也不能推断 BPU 文件来源；分别看误接受和正确覆盖 |
| DEV-07 可信参考提交 | 已有初始参考、经核验多视角及候选帧，并非缺少整个 memory | 先比较低质量观测不写入、真实视角多样性；只有证据表明有益时再加入距离分层或短窗反向校验 | MEM-01、ROB-01、VLA-01；遮挡者 crop 不污染图库，UNKNOWN/OTHER 后锚点不变化，背面回归仍能恢复；不为候选另建一套通用框架 |
| DEV-08 三维位置 | 历史已接受帧仍有前景椅子深度污染，身份正确不等于位置正确 | 可见人体部位/mask 内采样，检查边缘、有效深度及多区域一致性 | MASK-01、MOT-03；椅子挡住躯干时不把椅子深度作为人物位置；只有缺深度时不可伪造当前位置 |
| DEV-09 检测与共享实例 | 固定类别检测器回退后按 prompt 索引重标，以及 [InstanceTracker](../../src/perception/tracking/instance_tracker.py) 容量裁剪重绑共享字典 | 保留真实模型类别语义；共享字典原地裁剪 | 已复现实现问题；通用 `door.chair.person` 回退不把 COCO car 标 person，查询层与更新层始终看到同一实例表 |
| DEV-10 匹配回退 | `_find_match` 有几何/语义失败后的距离补配；零点云重叠或低外观分数未必可靠否定 | 基于真实时间、轨迹状态、质量与可靠身份冲突约束回退；不无条件删除运动/IoU 补配 | MOT-02/03 及历史 DeepSORT/BoT-SORT 对照；移动导致零重叠的同人可继续，明确异人冲突不能靠距离覆盖 |
| DEV-11 丢失后观察 | 当前保留位置与恢复证据不能等同完整主动搜索 | 先用原生地图和已有观察位产生接口表达清楚的搜索候选，身份恢复后再正常跟随 | SEARCH-01/02；转角丢失不换跟遮挡者，未知/不可达位置不授权移动，搜索超时/取消遵循原任务 |

DEV-02/09 是确定性实现修复，不应包装为论文创新；DEV-04 的真实时间适配也不是照抄按帧 tracker 即可完成。相机运动补偿只作用于所用图像状态，不能在 map 系位置再重复施加。

## 9. 执行顺序与勾选规则

本清单只跟踪本次人物链研究与开发，不替换 [项目交付 roadmap](../roadmap.md)。完成一项时附实现位置、最窄回归结果和证据等级再勾选；模型下载、进程启动和引用论文均不能算功能完成。

- [x] 核对 TrackVLA++ 的训练、推理部署与测速范围。
- [x] 补查 2025–2026 的描述拒识、遮挡 ReID、关联、记忆、mask 和搜索工作。
- [x] 建立论文编号、代码缺口、实施项与验收反例的映射。
- [x] 第一步：DEV-01/02/09，修确定性反例，分别验证原有合法行为与取消语义；完成范围和本地证据见下。
- [ ] 第二步：DEV-03/04/10，固定候选输入修弱框入口、真实时间和动态更新；保留既有部分赋值修正。
- [ ] 第三步：DEV-05/07，明确恢复的身份依据与参考提交条件，回放相似人交叉和长时重入。
- [ ] 第四步：DEV-06/08，独立比较专用 ReID 与可见部位定位；确认选定平台算力后再决定模型接入。
- [ ] 第五步：DEV-11，在原生三维链上验证观察位/搜索，完成取消与不可达反例后才进入监督实机验证。
- [ ] 补充 BENCH-01 评测数据获取记录，按人物/序列划分验证和测试，形成可复跑基线。

### 第一批代码修复记录：2026-10-20

- [x] DEV-01：唯一人物快速入口只允许明确的通用人物描述；红衣等属性描述仍需匹配，保留“the visible person”已有无模型行为。
- [x] DEV-02 位置：无有效空间上下文时，首次有效三维位置直接初始化；恢复 x=10 不再输出 x=4，机器人 x=8 时 waypoint 为 x=8.5。后续 EMA、源时间速度、跳变与重复帧拒绝保持。
- [x] DEV-02 取消：CLIP/OSNet/DINO 与长缺失多候选描述排序先在状态锁外准备；按 intent generation、观测 sequence 和新鲜度检查后提交。预计算失败不在提交锁内重试。显式 ID、观测回调与 VLM 安装目标均已接线；旧计算不能恢复已取消任务或覆盖更新观测。
- [x] 编码并发配套：`DinoEncoder` 用独立推理锁串行化加载、推理和销毁，避免支持的 TensorRT 引擎共用执行上下文时重入。此锁不属于取消/跟踪状态事务。等待 VLM 选人期间不重复计算丢弃的特征；持续 DINO 跟踪保留当前 ID 早返回与可达性过滤/批量处理。
- [x] DEV-09 类别：实际 owner 为 [yoloe_detector.py](../../src/perception/detection/yoloe_detector.py)。固定类别回退使用自身 names 和 class_id，按 prompt 过滤；正常 YOLO-E 仍使用开放词汇索引，真实提示词/推理失败继续向上传递。
- [x] DEV-09 实例：容量裁剪原地删除，保留查询器/场景图共享的字典引用。

验证：跟随、人物、检测回退和实例关联六个相关测试文件 **209 passed**；独立 DINO 编码器文件 **18 passed**，其中三个 fake TensorRT 并发用例修复前均失败。关键选人、取消、长缺失排序、首帧位置和类别/实例反例均有先失败再通过的证据。十个本次改动的 Python 文件 Ruff 通过。测试使用受控编码器和 fake native 入口，**没有运行真实 TensorRT 或实机运动，也没有得到新的板端 FPS**。

后续优先接入：DEV-03 的 ByteTrack 弱候选恢复入口；DEV-04 的真实源时间动态状态；DEV-05/07 的身份恢复与可信参考提交条件。先固定输入逐项比较，再决定是否引入 TrackTrack/OA-SORT 的额外关联策略。现有跨帧 VLM 首选参考的 freshness、长时间换轨仅凭运动等问题仍在后续范围，不能把本批取消修复解释为全部恢复问题已解决。

## 10. 实验与开发记录要求

每次比较只改变一个主要因素，保持检测输入、人物初始化和测试划分一致；调阈值使用验证集，测试集冻结。图库来自因果过去；需要完整视频才能得出的结果只能离线报告，不倒灌给在线链。

| 必测输入 | 需要同时报告的结果 |
| --- | --- |
| 描述目标不存在、唯一不符者、同属性多人 | 错误接受、正确拒识、歧义处理、错误目标下发次数 |
| 同衣着交叉、背面、部分遮挡、离开重入 | 正确可见覆盖率、IDSW、恢复成功率及耗时、图库错误写入；可辅以 HOTA/AssA/IDF1 |
| 转动相机、变帧率、重复/丢失源帧、上下文切换 | 状态/预测偏移、来源帧龄、过期处理和跨上下文结果是否提交 |
| 前景椅子/人体部分深度、上下楼与多层地面 | 在正确身份上统计三维误差、位置可用率和错误深度接受，说明真值来源 |
| 编码延迟、服务超时、目标消失时取消 | 感知至目标下发的 P50/P95 延迟、取消时效、旧结果是否覆盖新意图；物理停止另需 native 证据 |

单看已接受帧准确率会隐藏大面积缺失；“零错人但长期拒识”不能当可靠跟随。原始余弦、重排序分数和置信度都不是身份概率。多参考取最大值时，还应固定或记录参考数，防止只是图库变大导致误接受增加。

后续每条实验在 [person-follow.md](person-follow.md) 记录：引用编号与具体改法、源码版本、模型/实际权重/预处理、输入与划分、平台/加速器/分辨率、是否含检测/编码/网络/导航的计时边界、主要指标与保留失败、许可和部署结论。文献核对阶段未修改算法；随后代码修复见第 9 节。两阶段均未安装依赖或下载这些新增模型/数据；所有外部 FPS 都不是本机验证结果。
