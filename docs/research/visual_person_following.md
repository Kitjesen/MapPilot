# 视觉指定人员跟随：论文、代码与推进计划

状态：研究快照，不是运行时合同
观察日期：2026-08-29
适用范围：LingTu `tracking` Product，S100P/RDK X5，native CycloneDDS 与 native navigation

## 结论

可以找到论文，也能找到官方代码，但不能把“有 GitHub 仓库”等同于“可以集成”。当前资料可以分成四类：

1. ByteTrack、OC-SORT、BoT-SORT、MOT-RPF 有明确 MIT 许可，可用于算法对照或局部借鉴。
2. Mono-RPF、OCL-RPF 有真实实现，但没有声明代码许可证，而且依赖旧 ROS、CUDA 和较重的研究栈，只能学习方法。
3. Apple Human Following、Active Target Search 没有发现作者公开的正式实现，适合参考系统结构。
4. ReferTrack、TPT-Bench、JRDB 和 Follow-Bench 更适合做研究基准；其中 ReferTrack 的 4B 模型不适合直接部署到 RDK X5。

对 LingTu 最重要的判断不是更换一个新的 tracker，而是先把当前链路收口：

```text
BPU detector
  -> 一个短期 MOT
  -> Detection3D(map frame)
  -> 一个指定人员身份状态
  -> 跟随位置
  -> native nav
  -> driver
```

VLM 只负责首次从候选人中选中目标；实时闭环继续使用检测、MOT、轻量 ReID 和三维状态。视觉层不直接发布速度。

## 调研范围与方法

本次聚焦三个问题：

- 论文如何维持“仍然是同一个人”，而不是只维持任意人员轨迹。
- 论文如何处理部分遮挡、短时完全丢失和持续丢失。
- 官方代码是否存在、是否有许可证、是否适合 S100P/RDK X5。

检索来源包括 CVF/ECVA 论文页、arXiv、作者项目页和官方 GitHub。只把论文原文、作者项目或论文明确链接的仓库作为依据。第三方复刻、只有摘要的转载和无法确认来源的项目不进入采用结论。

这不是 PRISMA 系统综述。它是一份面向工程决策的定向调研，目标是找到可以验证 LingTu 真实业务问题的论文、代码和数据。

## LingTu 当前基线

当前正式链路已经完整：

```text
PerceptionModule
  -> BPUTracker(请求 BoT-SORT，缺失时回退 native ByteTrack)
  -> 2D 检测与深度/位姿投影
  -> Detection3D(map frame)
  -> VisualServoModule
  -> PersonTracker
  -> map-frame standoff goal
  -> native navigation
```

`tracking` 现在是这条链路的正式 Product，不再表示“跟踪一个显式地图点”。
操作员可在 Web 的当前人员列表中按稳定 ID 选择，也可通过 SDK 调用
`follow_person(target_id)`；两者都进入同一个 VisualServo 入口。

对应代码：

- [`module.py`](../../src/perception/module.py) 创建检测与跟踪输出。
- [`bpu_tracker.py`](../../src/perception/tracking/bpu_tracker.py) 适配 BoT-SORT、ByteTrack 和本地 fallback。
- [`native_byte_tracker.py`](../../src/perception/tracking/native_byte_tracker.py) 是无 CUDA/ROS 依赖的 ByteTrack 风格实现。
- [`visual_servo.py`](../../src/decision/modules/visual_servo.py) 负责选人、更新跟随目标和取消视觉任务。
- [`person.py`](../../src/decision/vision/person.py) 保存人员位置、速度、外观和丢失时间。
- [`goals.py`](../../src/nav/services/goals.py) 把视觉目标交给 native nav。

已经做对的部分：

- VLM/CLIP 用于首次选择目标，没有进入逐帧速度闭环。
- 人员位置已经转换到 map frame。
- 目标最终经过 native nav、局部规划和 driver，不绕过运动控制边界。
- 感知过期或目标丢失会取消导航目标。

当前缺口：

- 正式 BPU 路径的 BoT-SORT 配置关闭 ReID；短期 track ID 不等于长期人员身份。
- `enable_osnet_reid()` 和 `set_osnet_encoder()` 没有正式调用者，不能算已交付能力。
- 跟随目标默认以 1 Hz、0.25 m deadband 更新；是否导致急转时滞后尚未用真实数据证明。
- 当前 MuJoCo 只完成静止人员的接线与目标提交证明；连续运动、转弯、遮挡和物理跟随距离尚未动态验收。
- Host 的视觉跟随状态尚未进入 MCAP，因此还不能用一次录制统一分析感知、选人和 native 运动时间线。

## 论文与代码核验

### 可用于代码对照

| 项目 | 论文 | 官方代码与许可 | 运行条件 | LingTu 结论 |
| --- | --- | --- | --- | --- |
| ByteTrack | [ECCV 2022](https://www.ecva.net/papers/eccv_2022/papers_ECCV/html/315_ECCV_2022_paper.php) | [FoundationVision/ByteTrack](https://github.com/FoundationVision/ByteTrack)，MIT，核验提交 `d1bf019` | 官方栈以 Python、YOLOX、PyTorch 为主 | 已在 LingTu 中实现其两阶段关联思想；用于基准，不搬完整运行栈。 |
| OC-SORT | [CVPR 2023](https://openaccess.thecvf.com/content/CVPR2023/html/Cao_Observation-Centric_SORT_Rethinking_SORT_for_Robust_Multi-Object_Tracking_CVPR_2023_paper.html) | [noahcao/OC_SORT](https://github.com/noahcao/OC_SORT)，MIT，核验提交 `8462e7e` | 官方仓库以 Python 为主，也给出 C++ 部署入口 | 优先做离线对照，判断其遮挡重更新是否优于当前关联；数据没有证明前不新增正式 backend。 |
| BoT-SORT | [论文](https://arxiv.org/abs/2206.14651) | [NirAharon/BoT-SORT](https://github.com/NirAharon/BoT-SORT)，MIT，核验提交 `2519854` | YOLOX、FastReID、PyTorch/CUDA | LingTu 已请求该后端；先确认真实启用率和无 ReID 配置的表现，不再复制一份。 |
| MOT-RPF | [AIEA 2024](https://doi.org/10.1109/AIEA62095.2024.10692403) | [hyzhu1999/MOT-RPF](https://github.com/hyzhu1999/MOT-RPF)，MIT，核验提交 `287d282` | Jetson Xavier NX、OC-SORT、手势选人 | 可参考运行期重新选人和轻量实机组织；不能接管 LingTu native nav。 |
| ReferTrack | [2026 预印本](https://arxiv.org/abs/2607.20061) | [MedlarTea/referTrack](https://github.com/MedlarTea/referTrack)，Apache-2.0；已发布评估代码和 checkpoint，尚未发布训练代码 | Qwen3-4B checkpoint 约 7.6 GB，Habitat、PyTorch/CUDA | “先 referring、后 tracking”与 LingTu 方向一致；只在工作站做研究对照，不部署到 X5。 |

### 有实现但不应复制

| 项目 | 论文 | 代码状态 | 限制 | 可借鉴内容 |
| --- | --- | --- | --- | --- |
| Mono-RPF | [ICRA 2023](https://arxiv.org/abs/2302.02121) | [MedlarTea/Mono-RPF](https://github.com/MedlarTea/Mono-RPF)，真实实现，核验提交 `7c4e2cf`，未声明许可证 | ROS1、CUDA 10.2、YOLOX、AlphaPose、Ceres | 部分人体仍可见时，让有效关节分别贡献位置观测。先用 RGB-D 有效深度做轻量实验，再决定是否需要姿态模型。 |
| OCL-RPF | [RA-L 2024](https://arxiv.org/abs/2309.11727) | [MedlarTea/OCL-RPF](https://github.com/MedlarTea/OCL-RPF)，真实实现，核验提交 `3fd06f7`，未声明许可证 | ROS1、catkin、PyTorch 1.11、CUDA 11.3、mmtrack/mmcv | 短期与长期外观记忆值得研究；第一版不引入在线训练。 |
| Koide 单目跟随 | 作者仓库说明 | [koide3/monocular_person_following](https://github.com/koide3/monocular_person_following)，核验提交 `ed7f384`，未声明许可证 | ROS1、TensorFlow、Jetson，源码较旧 | 学习相对位置估计、UKF 和目标恢复结构，不复制代码。 |
| Follow-Bench | [2025 预印本](https://arxiv.org/abs/2509.10796) | [MedlarTea/follow-bench](https://github.com/MedlarTea/follow-bench)，仓库根未声明统一许可证，核验提交 `202d164` | Python 仿真与多个规划器，不是视觉 tracker | 借用拥挤、门口、交叉口和不同跟随距离的场景与指标，不移植规划器。 |

### 只有论文、占位代码或数据集

| 项目 | 可获得材料 | 当前状态 | 用途 |
| --- | --- | --- | --- |
| Apple Human Following | [Apple 官方研究页](https://machinelearning.apple.com/research/human-following) | 没有发现作者公开的官方实现 | 学习多角度登记、脸部/躯干 ReID、运动预测和丢失搜索的产品结构。 |
| Active Target Search | [论文](https://arxiv.org/abs/1809.08793) / [作者页](https://nickwalker.us/publications/kim2019/) | 没有发现作者公开的官方实现 | 学习“先停车，再规划到可观察位置”的找回逻辑，不引入其完整行为树。 |
| RPF-Search | [论文](https://arxiv.org/abs/2503.02188) / [仓库](https://github.com/MedlarTea/RPF-Search) | 仓库无许可证，README 仍写待发布代码；当前不能视为实现 | 区分墙角遮挡与人群动态遮挡，作为后期显式搜索设计参考。 |
| TPT-Bench | [论文](https://arxiv.org/abs/2505.07446) / [工具](https://github.com/MedlarTea/TPT-BENCH-TOOLS) | 数据与工具已发布；LICENSE 文本为数据集 CC BY 4.0，工具仍依赖 ROS Noetic | 很适合做机器人视角、长时指定人员跟踪基准；写独立数据转换器，不把 ROS 工具带入 Product。 |
| JRDB | [论文](https://arxiv.org/abs/1910.11792) / [官方数据集](https://jrdb.erc.monash.edu/dataset/) | 机器人视角 RGB、RGB-D、LiDAR 和 2D/3D 人体轨迹；数据为 CC BY-NC-SA 3.0 | 用于研究与离线评估；非商业许可限制需要与产品训练数据分开。 |

## 证据综合

### 1. MOT 与指定人员身份是两个问题

ByteTrack、OC-SORT 和 BoT-SORT 主要解决短时轨迹关联。它们能减少遮挡时的轨迹断裂，但 track ID 本身不是长期身份。Apple Human Following、OCL-RPF 和 ReferTrack 都把“确定目标是谁”单独处理：先登记或 referring，再用外观与运动维持目标。

LingTu 应保持同样分工：`BPUTracker` 负责当前画面中每个人的短期轨迹，`PersonTracker` 只负责从这些轨迹中维持指定人员。

### 2. 论文对遮挡的处理不是一个统一动作

文献共同指向三种不同情况：

- 部分可见：继续利用仍可靠的身体区域、关节或深度点更新位置。
- 短时完全丢失：停止发布新目标，保留短时间的运动预测用于重新关联。
- 持续丢失：默认结束或等待；只有用户明确允许搜索时，才通过地图和 native nav 选择观察位置。

这不需要新框架，只需要让现有目标状态能区分“仍有观测”和“只剩预测”。

### 3. VLM 的价值主要在首次选人

本次核验到的传统机器人跟随系统仍以检测、MOT、ReID 和状态估计作为实时闭环。ReferTrack 证明了语言描述和跟踪可以端到端结合，但其 4B 模型、CUDA 与训练资源不符合 X5 产品链。LingTu 当前“VLM/CLIP 选一次，轻量跟踪持续运行”的方向更适合现场平台。

### 4. 当前最大未知不是模型，而是链路性能

在替换算法前，需要先回答：

- 急转时掉队主要来自检测频率、1 Hz 目标更新、定位延迟，还是局部规划响应？
- 遮挡后认错人主要来自短期 MOT、缺少外观身份，还是深度位置跳变？
- 固定前向相机的视场是否已经成为主要瓶颈？

没有这些数据，新增模型只会让链路更重，无法证明业务收益。

## 推进计划

### 已完成（2026-08-29）

- `tracking` Product 已收口为 camera/perception/VisualServo/native-nav 主链，去掉与跟人无关的 LLM、Memory 和第二运动链。
- Web 可从 SceneGraph 人员列表按稳定 ID 选人；Gateway、同步 SDK 和异步 SDK 使用同一个 `target_id` 入口。
- Perception 的现有图像编码器已通过公开接口交给 VisualServo，不再各建一份。
- 只有一人时可在同一 track 连续两帧后直接锁定；多人仍要求 CLIP 或视觉 LLM。
- 正式 follow 已使用 PersonTracker 的短时位置预测，结果仍作为 `map` 目标交给 native nav。
- Detection3D 的源时间用于排序和速度估计；重复或倒退观测不会推进人员状态。
- 当前帧找不到指定人员时立即取消 VisualServo 自己的导航目标；进入跟随距离后不后退，朝向仍面向人员。
- Gateway 状态已包含 track id、地图位置、速度、置信度和最后观测时间。
- PersonTracker 内无人调用的 FusionMOT/qp_perception 第二路径已删除。
- Thunder MuJoCo 已有 tracking 专用 preset、静止人员实体和稳定 ID 静态闭环测试。

当前未完成的是动态 MuJoCo 跟随、S100P 实机跟随验收、多人场景所选 ReID 后端的部署决定，以及视觉状态的 MCAP 主题；现有 MCAP 不能自动记录 Host 内的 `servo_status`。

### 第一批：建立当前基线

先使用 Host `servo_status` 与已有传感器录制建立基线；如需统一写入 MCAP，再单独增加视觉状态 DDS 主题，不假定当前已经存在：

- detection/track ID 连续性；
- 指定人员是否发生身份切换；
- map-frame 三维位置跳变；
- camera 到 `visual_goal_request` 的延迟；
- 目标丢失、取消目标和重新出现的时间；
- 跟随距离、最小净空和停车延迟。

覆盖五个真实业务场景：直行、90°转弯、相似衣着人员交叉、1–3 秒遮挡、持续遮挡。

完成条件：能区分问题出在检测/MOT、身份、三维投影还是 nav，而不是只得到一个总成功率。

### 第二批：收口正式跟踪路径

涉及文件：

- `src/perception/tracking/bpu_tracker.py`
- `src/decision/vision/person.py`
- `src/decision/modules/visual_servo.py`
- `src/lingtu/assembly/wires/semantic.py`

调整原则：

1. `BPUTracker` 是短期 MOT 的唯一所有者。
2. `PersonTracker` 只保存指定人员的身份、位置、速度、置信度和最后观测时间。
3. 从正式路径去掉没有调用者的 FusionMOT 分支；OSNet 只保留一个明确注入点，是否启用由 X5 实测决定。
4. 首次选人保存少量高质量外观样本；逐帧不调用 VLM。
5. 不改变 native nav 与 driver 的最终控制权。

完成条件：正式运行只有一套 MOT；同一段录制的结果可重复；删除备用分支后现有选择、跟随、取消测试仍通过。

### 第三批：预测与遮挡恢复

1. 先测量端到端延迟，再决定是否让正式链路使用已有的 `get_follow_waypoint()`。
2. 对 1 Hz、3 Hz、5 Hz 目标更新做同一录制回放，选择最低但能稳定跟随急转的频率。
3. 部分遮挡先尝试 bbox 内有效 RGB-D 深度和人体区域稳健统计；只有它无法解决问题时，再评估轻量 keypoint 模型。
4. 完全丢失立即取消新目标；短期预测只用于重新关联，不用于盲目继续前进。

完成条件：90°转弯的跟随误差下降，身份切换不增加，丢失后机器人仍能及时停止。

### 第四批：外部基准与可选研究

- 在独立工具目录中读取 JRDB/TPT-Bench，不把 ROS 数据工具接入 Product。
- 离线比较当前 tracker、native ByteTrack 和 OC-SORT；胜者必须同时改善身份连续性和 X5 预算，才进入正式候选。
- 在工作站运行 ReferTrack 视频评估，研究语言选人与遮挡场景；不作为 X5 部署依赖。
- 只有持续丢失成为主要失败原因，才设计显式 `search person` 行为，并继续通过 native nav 运动。

完成条件：每个新增候选都有相同数据、相同指标和明确停止条件；没有收益的候选直接放弃。

## 不做的事情

- 不引入第三套正式 MOT。
- 不复制无许可证的研究源码。
- 不把 ROS1、catkin 或 rosbag 运行时重新带回 Product。
- 不让 VLM、在线训练或 4B VLA 模型进入 X5 实时闭环。
- 不让视觉模块直接发布 `cmd_vel`。
- 不在默认 follow 中偷偷搜索丢失人员。

## 推荐顺序

```text
现有链路基线
  -> 唯一 MOT + 唯一指定人员状态
  -> 延迟/急转优化
  -> 部分遮挡
  -> 离线外部基准
  -> 按真实失败决定是否做主动搜索
```

第一阶段代码推进应从基线与正式路径收口开始，而不是先移植论文代码。

## 调研限制

- 维护状态按 2026-08-29 可见仓库内容和最后源码推送判断，后续可能变化。
- 预印本 ReferTrack、TPT-Bench、RPF-Search 与 Follow-Bench 的结论需要和后续正式出版版本对照。
- 没有许可证不代表作者反对研究，但代表 LingTu 不能默认获得复制、修改和分发源码的权利。
- 论文报告的 GPU/机器人结果不能替代 S100P/RDK X5 实测，也不能证明 LingTu field readiness。

## AI 辅助说明

本报告使用 AI 辅助完成定向文献检索、官方来源核验、代码仓库筛选和 LingTu 代码映射。论文与代码链接均指向论文原文、作者页面或官方仓库；实施判断仍需通过 LingTu 自有录制、仿真和现场证据验证。
