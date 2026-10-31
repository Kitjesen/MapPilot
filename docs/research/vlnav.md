# VL-Nav 复现记录

更新：2026-10-17（产品搜索接线；下文实验日期不变）。主参考固定为 **arXiv:2502.00931v7（2026-07-14）**，
对应作者标注的 IROS 2026 工作，避免混用 2025 年旧版方法。

- [本地原文](papers/vl-nav.pdf)
- [作者项目页](https://sairlab.org/vlnav/)
- [固定版本方法与公式](https://arxiv.org/html/2502.00931v7)

目标是用户说“帮我找一瓶水”，系统根据实际图像找目标，未发现时主动探索，
发现候选后规划到观察位置，再核验是否符合指令。当前已在 Habitat 中跑通
**移动—重观测—更新对象—继续探索/接近候选**的实验闭环，但仍未接入 Qwen3-VL、
四足动力学和实机运行。产品代码已增加原生语义观察视点查询桥和无候选搜索状态，
并通过本地 native/DDS 与受控生命周期验证；它仍不是完整产品闭环或论文完整复现。
本页是后续实现和验收的入口；其他论文作为参考，不再同时追三套系统。

## 方法依据与产品实现约束

2026-10-16 重新核对本地 PDF 的版本标记和 III-A / III-B / III-C。后续实现必须说明
采用的论文环节、代码位置、与原方法的差异及验证结果，不能以论文引用代替实现证据。

| 论文方法 | LingTu 对应位置 | 当前边界与必须保留的区别 |
| --- | --- | --- |
| III-A：场景图、对象最佳观察图像支持 Qwen3-VL 规划；子任务完成后重规划 | [Agent](../../src/decision/modules/agent_planner.py)、[TaskPlan](../../src/decision/tasks/decomposition.py)、[Memory](../../src/memory/README.md) | 当前三者未连成论文的完整规划器；规则解析和独立工具循环不能作为该方法已经实现的证明 |
| III-A：先筛候选，再用图像与邻接场景信息核验，选用观察位姿 | [目标解析](../../src/decision/goals/resolver.py)、[观察位置](../../src/decision/semantic_navigation/observation.py)、[核验](../../src/decision/semantic_navigation/verification.py) | 当前按对象位置生成周围观察候选并查询原生可达性；不等同于论文直接使用保存的最佳观察位姿 |
| III-B/C：视觉方向证据、实例优先及前沿排序 | [选择内核](../../src/decision/frontiers/vlnav.py)、[产品搜索状态](../../src/decision/semantic_navigation/search.py) | 公式实验与产品搜索分开：产品运行时向原生地图服务请求观察视点，再做 3D 预览；本地 native/DDS 和受控搜索状态已验证，但这不是论文地图前沿模块的逐行复刻或正式地图实测 |
| FAR Planner 接收目标并执行几何导航 | [原生命令接口](../../src/nav/commands/README.md) | LingTu 保留 OctoPlanner3D + SCAN，是明确的产品适配；投影前沿只能提出候选，不能替代三维碰撞、支撑与运动约束 |

论文采用 Qwen3-VL。使用已配置的 Kimi 或其他模型属于后端替换实验，必须记录实际模型、
输入场景/图像/历史、结构化响应、耗时和最终任务结果；不能仅因 API 格式兼容就声称效果等价。
任务 ID、取消、旧回复隔离和 Product 生命周期是 LingTu 的工程契约，不归因于论文。

“产品级”按以下事实验收，不以新增类、接口数量或一段成功视频判断：

- **完整执行：** 固定场景和指令，真实模型、感知、目标选择、原生运动、结果核验属于同一次任务；先覆盖视野内目标，再覆盖未见目标需要搜索及关系约束。
- **正确结束：** 目标不存在、误候选、路径受阻和模型不可用有明确结果；导航到达与对象确认分别记录，不能把检测类别或受理回执当成功。
- **生命周期：** 取消、任务替换、旧模型回复、相机重启和地图切换保持已有归属与失效规则；停止证据由原生执行者提供。
- **可复跑：** 固定数据、任务集、地图、模型/参数和初始状态，保留失败案例；报告成功率、误报完成率、路径效率（具备参考路径时）、耗时与模型调用量，并与固定基线比较。
- **分层资格：** 本地契约、Habitat 算法实验、原生物理仿真、实机静止和受监督运动分别记录；按照 [Testing](../testing.md) 验收目标平台。设备性能预算在运行前固定，不用论文设备的帧率代替板端结果。

以上是后续实现约束，不是新通过的验收。本轮产品接线没有运行机器人；独立模型 API
连通性检查也不能替代同一次任务中的感知、规划与运动证据。

### 产品搜索适配（2026-10-17）

目标不在共同观测中时，产品路径不再依靠 Python `FrontierScorer` 的二维投影授权运动。
`nav.commands.query_semantic_views` 通过原生 typed DDS 向正式地图端点发送地图身份、frame
epoch、相机范围/FoV 与实际观察历史，返回候选观察位姿。SemanticPlanner 对每个候选继续
调用原生三维 plan preview；只有可行候选才建立自己拥有的搜索导航任务。

搜索任务到达只表示新的拍摄位置可用。系统必须等待晚于到达时间的新共同观测，重新选择
对象，并在接近目标后完成既有视觉核验，才能报告成功。指令替换、取消、相机 generation
变化、地图身份变化和迟到视点结果均不能推进旧搜索。目标在搜索途中出现时，先定向取消
自己的搜索子任务并等待原生终态，再进入对象接近。

这一步借鉴 VL-Nav 的“未发现目标则选择观察方向、移动后重观测”分工，同时保留 LingTu
的原生三维地图与运动仲裁。它没有复刻论文的 Qwen3-VL 场景图重规划或 FAR Planner，
也没有提供真实检测、仿真运动或实机证据。当前本地证据包括 Python bridge 37 项、
WSL 原生客户端构建、真实 CycloneDDS 关联与 120 ms 无响应超时、既有候选算法退出码 0、
搜索状态 7 项和相关受控生命周期回归。

## 已实现到哪里

| 环节 | 已有内容 | 仍缺什么证据 |
| --- | --- | --- |
| 文本理解 | 普通 HTTP/MCP 指令优先创建 Agent run；模型配置和单次调用时限已对齐；执行任务需领域成功证据 | 带真实图像记忆的 Qwen3-VL 式任务重规划；当前 `TaskPlan` 仍未驱动 AgentLoop |
| 目标感知 | Habitat 连续图像上的 YOLO-World + FastSAM；通用对象词表、RGB-D 投影、CLIP 对象图像特征 | 已发现手提包误报为瓶子；类别核验、实机输入仍缺 |
| 对象记忆 | 保存点云/512 维特征/ID/多视角证据；重载后继续关联；独立实例评测已接通 | 实测有对象拆分和串轨，身份稳定性尚未解决；实时 Product、跨次定位仍缺 |
| 探索评分 | v7 式 (3)—(8) 和 III-C 选择规则已驱动 Habitat 循环；产品原生视点桥与搜索状态已通过本地 native/DDS 和受控回归 | 正式保存地图 endpoint、连续真实 RGB-D、拒绝后的真实模型任务重规划 |
| 几何执行 | Habitat NavMesh 代理体实际移动；产品搜索候选必须再过 OctoPlanner3D 预览后才能派发 | 产品代码尚无四足动力学、SLAM 现场和实机轨迹证据 |
| 目标核验 | 接近后重检候选；关系查询要求实际候选图边；明确不授权运动 | 重检同一错误标签仍可能假成功；Qwen3-VL 语义核验、错误候选回退未完成 |

### 本轮代码

[`src/decision/frontiers/vlnav.py`](../../src/decision/frontiers/vlnav.py)：

- `vl_score`：置信度加权高斯混合 × 视野中心权重，截断至 `[0, 1]`；
  σ 使用论文给出的 `0.1 rad`，视野外不给视觉分数。
- `cue_from_box`：用检测框中心和针孔相机内参生成 `Cue`；水平角向左为正，
  `bearing = atan2(cx - (x1+x2)/2, fx)`，置信度保留检测器输出。此适配要求校正后的
  水平相机；倾斜相机须先用完整外参把像素射线转到导航水平面，不能直接套用。
- `unknown_ratio`：有界 BFS 计算未知单元比例，不能穿过障碍；
  BFS 的邻域半径是实验参数，论文未给出的数值不冒充作者配置。
- `observe_frontiers`：复用现有 `FrontierScorer` 提取的簇，保留传入的参考高度，
  在当前相机视野内生成探索候选。它在**簇代表点**上筛视野，区别于论文在
  **聚类前逐单元**筛视野；因此这只是现有提取器的适配，并非逐行复刻全部地图模块。
- `rank_candidates`：先选置信度过门限且未到达的实例，实例按视觉分数排序；
  无可用实例时按 `w_dist/(1+d) + w_VL*S_VL*(1-exp(-k*ratio))` 排前沿。
  支持当前搜索中的已拒绝实例排除，全部候选不可用则返回空列表（保持原地）。

视觉分数由候选生成时保存；选择时重新计算距离，调用方更新未知比例。
实例坐标是物体中心，不能直接当机器人目标位姿。距离评分遵循论文的水平距离，
调用方须先按当前楼层/高度层筛选，最终路径仍交给原生三维导航。

选择器目前是**可调用算法**，尚未接入 `SemanticPlannerModule` 实时运行。
现有 planner 内部的 `FrontierScorer` 没有地图订阅/更新/提取链路；不能因为它
有 `get_best_frontier()` 就宣称主动探索已经生效。原生 `ViewQuery/ViewResult`
可作为下一步候选接口，保留 map identity、frame epoch、参考高度和相机观察历史。

[`tools/datasets/habitat_probe.py`](../../tools/datasets/habitat_probe.py)：
记录官方场景的八个同步 RGB-D 视角、相机位姿和内参；保持 Habitat Y-up 坐标并
明确轴约定。这是传感器数据准备，不执行找物、检测或机器人运动。

[`tools/datasets/habitat_cues.py`](../../tools/datasets/habitat_cues.py)：
读取上述记录，通过现有 registry 构造 `YOLOWorldDetector`，实际检测目标提示词，
输出逐帧检测框、检测置信度、`Cue` 和整个水平 FoV 的视觉分数曲线。
输入 RGB 图片在调用现有检测器前转为 BGR；检测框坐标对应原图，使用原图内参。
工具不读取官方语义标签、不使用深度、不生成地图前沿、不派发运动目标。
框中心方向是当前适配；尚未引入论文中的 FastSAM 分割区域。

### Qwen3-VL 与 Cue 的分工

论文 III-A 的 Qwen3-VL 负责将抽象任务拆成 `exploration` / `go to`，结合场景图、
对象最佳观察图像选择目标，并在子任务结束后重规划。它不是每帧方向评分器。
III-B 的方向证据来自开放词汇检测；当前桥接使用 YOLO-World 检测框中心和置信度。
检测词的文本编码由 YOLO-World 的 CLIP 编码器完成，不是生成式 LLM 输出 `Cue`。

对于候选方向 `theta`，当前视觉分数为：

```text
mu_i = atan2(cx - box_center_x_i, fx)  # positive left
alpha_i = detector confidence
S(theta) = clip(sum(alpha_i * exp(-0.5 * (angle_diff(theta, mu_i) / 0.1)^2))
                * cos(pi * theta / hfov)^2, 0, 1)
```

视野外为零。分数是启发式排序值，不是经过校准的目标存在概率。
`Cue` 可在不使用深度时计算；对象三维位置和可通行观察位置还需要深度/几何链路。

本轮模型输入使用官方 `yolov8s-worldv2.pt` 与 CLIP ViT-B/32 权重；
来源为 [Ultralytics 模型文档](https://docs.ultralytics.com/models/yolo-world/)。
此处 CLIP 是检测器的文本编码组件；后文 2026-10-08 的补充实验另用 CLIP 图像编码器
生成对象裁剪图特征。二者都不是 Qwen3-VL。服务器先用 CPU 验证数据链，
时间记录包含每次运行首帧的提示词编码，不能作为机器人部署帧率。

服务器检测复跑命令（`source/` 是上传的本仓库 Python 源码）：

```bash
cd /root/autodl-tmp/lingtu-vlnav
PYTHONPATH=source/src:/root/autodl-tmp/models/clip-vit-base-patch32 \
YOLO_AUTOINSTALL=false vision/bin/python source/tools/datasets/habitat_cues.py \
  apt_0 --weights yolov8s-worldv2.pt --prompt chair --device cpu --output cues-chair
```

CLIP 的标准权重放在 `weights/clip/ViT-B-32.pt`，复用服务器已有的 CLIP/ftfy
源文件路径；安装和推理仅涉及独立实验环境，没有更新产品依赖锁文件。

## 可复跑验证

提交范围为选择内核、测试、采集工具、本记录、decision README 和参考论文。
`build/vlnav/` 中的数据集、图像、深度与日志是本地生成结果，不随 Git 提交；
下文指向这些结果的链接仅在已运行实验的工作区可用。

```powershell
.venv\Scripts\python.exe -m pytest tests/decision/test_vlnav.py `
  tests/decision/test_frontier_scorer.py -q --junitxml=build/vlnav/tests.xml
```

本轮结果：**29 passed**（10 个新测试 + 19 个现有前沿测试）。
覆盖公式数值、FoV 边界、墙体阻隔、实例优先、误检排除、到达距离、视觉线索改变
前沿排序，以及真实 `FrontierScorer` 输出适配。全部是本地算法测试，数据为合成
fixture；不是 Habitat episode，也不是感知准确率或论文 SR。

`build/vlnav/selection.json` 留存了实际调用内核的三个合成输入结果：
探索时相关前沿得分 `0.560786`、最近前沿 `0.1`；出现过门限实例后优先核验实例；
将该实例加入拒绝集合后回到相关前沿。该文件明确记录 `model_executed=false`、
`robot_motion_executed=false`，不能当成模型识别或运动证据。

### 2026-10-06 — 实际检测到 Cue 的连接

在同一服务器、同一批 `apt_0` 的 8 张 Habitat 原始图像上，运行本仓库注册的
`YOLOWorldDetector`，使用 `yolov8s-worldv2.pt`、置信度门限 `0.25`、CPU 推理。
输入仅为图像和英文检测词，没有读取官方语义标签。结果已下载到本地：

| 提示词 | 每帧检测数（00—07） | Cue / 视觉分数 |
| --- | --- | --- |
| `chair` | 0, 0, 0, 2, 5, 3, 0, 0 | 10 条逐帧 Cue，不代表 10 个不同对象 |
| `water bottle` | 0, 0, 0, 0, 0, 0, 0, 0 | 全部为空，所有方向分数为 0 |

`05` 帧一个检测框横向范围为 `[361.114, 430.253]`，中心 `u=395.683`。
使用 `fx=cx=320`，计算 `bearing=-0.232243 rad=-13.3065°`（右侧），
检测置信度 `0.697474`。同帧三个 Cue 的高斯贡献分别为
`0.001024 + 0.697474 + 0.174859`；再乘视野权重 `0.799329`，得到该方向
`S_VL=0.698099`。这不是 LLM 生成的数值。

- chair 检测框与分数曲线 (`build/vlnav/cues-chair/05.jpg`)
- chair 原始检测和 Cue 记录 (`build/vlnav/cues-chair/report.json`)
- water bottle 空检测记录 (`build/vlnav/cues-water-bottle/report.json`)
- chair 推理日志 (`build/vlnav/server/cues-chair.log`)

逐帧视检可见远处软包座椅区域也被标为 `chair`，还没有细粒度类别核验。
水瓶的空检测只能说明这批输入和这个模型/门限下未检出，不能证明整个场景无水瓶。
本次未调用 Qwen3-VL、未运行 FastSAM、未使用深度，也没有地图前沿选择或运动执行。
因此新增证据是 **Habitat RGB → 实际检测 → Cue → 视觉方向评分**，并非找物闭环。

相关本地测试合计 **51 passed**：原评分与前沿测试、检测器测试、像素投影和记录
适配测试。新测试覆盖左右符号、缩放内参一致性、RGB/BGR 转换、本地权重传递、
空检测分数归零。测试里的检测返回值是 fixture；上述服务器结果才是真实模型推理。

## 平台与复现顺序

论文模拟实验是 DARPA TIAMAT Phase 1：HabitatSim 的两个公寓、IsaacSim 的营地和
工厂；仿真本体为 Spot、五个 RGB-D 相机。项目页另有 Rover、Go2、G1 实机演示。
当前未找到作者发布的完整 VL-Nav 源码仓库链接；因此先按论文实现，并记录工程差异，
不能声称已运行作者原始程序。

1. 在独立服务器环境加载官方 ReplicaCAD，实际输出 RGB-D 和标定。该数据与论文
   TIAMAT 场景不同，用于先验证数据链。已有 `apt_0` 标签无 water bottle，第一例
   用真实存在的目标；不把其他物体改名成水瓶。
2. 运行开放词汇检测/分割，把当前帧检测投影为实例，记录最佳视图及观察位姿。
   数据集真值只用于评测，不能传给搜索策略充当模型识别。
3. 接原生探索候选 → 本轮评分器 → 三维预览 → 目标受理 → 到达后新帧核验；
   失败回到搜索，预算用尽明确结束。记录每步输入、候选分数、选择、执行和核验。
4. 在含水瓶的可用官方场景做固定起点 episode，再测试多房间和复合关系指令。
   输出原始第一视角录像与真实轨迹，成功率和路径长度按 episode 统计。
5. 获取与论文相同的场景、任务和实验设置后才比较 SR/MTUR/SPL；在此之前只报告
   LingTu 自己的复现指标。Habitat 动作导航与 Go2 动力学/实机运动分别验收。

## 服务器记录

已重新登录用户提供的服务器，检查到 RTX 5090（32607 MiB 显存）、约 179 GB
剩余数据盘空间。检查时 GPU 利用率 98%，不修改正在使用的 `isaaclab` 环境。

`/root/autodl-tmp/lingtu-semantic-vlm` 有旧的 CLIP 图像分类与几何产物；检查到的
Python 环境没有 `habitat_sim`、`habitat`、`ultralytics`，Qwen 下载目录为空。
旧 CLIP 分类不能充当 YOLO 检测、Qwen3-VL 推理或完整语义导航证据。

独立安装目录：`/root/autodl-tmp/conda/envs/lingtu-vlnav`。
Habitat-Sim 0.3.3 的可用 Conda 构建要求 Python 3.9，首次 Python 3.10 解依赖失败，
已改用独立 Python 3.9 **渲染环境**；LingTu 产品 Python 版本保持原约定。
安装日志放在 `/root/autodl-tmp/lingtu-vlnav/`，安装和场景渲染结果以实际日志为准。

本轮安装已成功，`import habitat_sim` 返回 `0.3.3`。安装了 `headless` 与
`withbullet` 构建，精确环境清单保存到 `build/vlnav/server/environment.txt`。
独立 vision venv 安装了 Ultralytics `8.4.173`；安装成功不代表模型已推理。

服务器直连 Hugging Face 下载超时；本机通过官方仓库 API 和固定 `v1.6` resolve
地址下载 601 个非 Git 元数据文件，0 个失败，总数据约 157 MB，保存于
`build/vlnav/replica_cad/`。完整资源包括此前子集缺少的灯光、关节对象和配置。
服务端目标目录为 `/root/autodl-tmp/lingtu-vlnav/data/replica_cad/`。

### 已完成的真实渲染

Habitat-Sim 0.3.3 在服务器 RTX 5090 的 EGL/OpenGL 后端成功加载 `apt_0`，
固定随机种子 `7`，生成 8 组 `640 × 480` RGB-D、相机位姿及内参。
水平 FoV 为 90°，相机相对 agent 原点高 1.2 m，八个方向间隔 45°。
逐帧有限且大于零的深度比例为 67.4%—100%；这只是有效数据比例，不是深度精度。
部分方向面对近墙，黑色区域和无深度像素必须作为无观测处理。

- 八方向原始视图 (`build/vlnav/apt_0/views.jpg`)
- 客厅方向原始帧 (`build/vlnav/apt_0/04.png`)
- RGB-D、位姿和证据范围记录 (`build/vlnav/apt_0/report.json`)
- 实际渲染日志 (`build/vlnav/server/probe.log`)

官方配置中的默认 navmesh handle 未解析，因此显式加载同一官方数据包的
`navmeshes/apt_0.navmesh`。没有修改原始场景；其他场景的缺失资源警告保留在日志。
本次采样点来自官方 navmesh，尚未验证四足本体的足迹、支撑和碰撞条件；不能把
这个视点当作已验证的机器人起点。场景没有加载像素级语义注释，后续识别须来自模型。

服务端复跑命令：

```bash
/root/autodl-tmp/conda/envs/lingtu-vlnav/bin/python \
  /root/autodl-tmp/lingtu-vlnav/habitat_probe.py \
  --dataset /root/autodl-tmp/lingtu-vlnav/data/replica_cad/replicaCAD.scene_dataset_config.json \
  --scene apt_0 \
  --navmesh /root/autodl-tmp/lingtu-vlnav/data/replica_cad/navmeshes/apt_0.navmesh \
  --output /root/autodl-tmp/lingtu-vlnav/apt_0
```

这些输出是固定位置的传感器渲染，**不是找物轨迹、模型识别结果或导航视频**。
运行记录明确标记 detector 与 robot motion 为 false。生成数据在 `build/` 下，
可按命令重新生成；复跑工具、算法和本文留在源码目录中。

## 2026-10-07 — RGB-D 对象投影与跨帧关联

本轮新增 [`tools/datasets/habitat_objects.py`](../../tools/datasets/habitat_objects.py)，
分为服务器 `segment` 和本地 `build` 两步。不是 VL-Nav 官方代码，属于本项目适配。

1. 使用前一轮真实 YOLO-World 检测框，以官方 `FastSAM-s.pt` 做框提示分割。
   每帧只进行一次全图分割，复用官方 `predictor.prompt` 为各检测框选择掩码。
2. 掩码与已保存的同帧深度配对，调用现有 `mask_to_pointcloud()`。
   深度按米读取，取 0.3—15 m 范围，每条观测最多保留 512 点。
3. 将光学相机坐标转为 Habitat 世界坐标，再以 `[x,y,z] -> [x,-z,y]` 转为
   **右手 Z-up 实验坐标系 `habitat_z_up`**。此坐标系未绑定原生导航地图，
   不可与旧离线数据适配器的 `[x,z,y]` 反射变换产物直接混合。
4. 调用现有 `InstanceTracker` 跨帧关联，距离回退门限为本次实验的 0.35 m；
   未生成 CLIP 实例特征，关联依赖已有几何重叠/同类距离逻辑，不是语义特征验证。
5. 保存关联记录、对象点云、最高检测分数对应的原始图像及相机位姿；使用现有
   tracker 保存/重载入口，比较对象 ID、位置和点云。

### 实际结果

- 输入为同一位置八个朝向的真实 Habitat RGB-D，以及模型产生的 10 条检测。
- 10 条观测均完成分割和深度投影，被现有跟踪器关联为 **5 个候选实例**。
- 每个候选关联到 2 帧；这仍是同一位置转向观察，不是机器人平移探索。
- 两次观测点云质心的距离：对象 0 为 3.84 cm，1 为 2.34 cm，2 为 34.40 cm，
  3 为 2.20 cm，4 为 13.73 cm。它是跨帧质心差，不是相对于真值的定位误差。
- 对象 2 为软包座椅区域；部分预测掩码混入背景，尚未有真值核验或 VLM 细分类。
- 重载的 5 个对象 ID、位置及点云一致。尚未验证重新启动后继续跟踪的行为。
- 本轮没有使用官方对象类别/实例坐标来生成对象，也没有执行 Qwen3-VL 或运动。

证据目录：`build/vlnav/objects-chair/`。

- 掩码与关联 ID (`build/vlnav/objects-chair/05.jpg`)
- 三维点云图 (`build/vlnav/objects-chair/map3d.png`)
- 俯视图 (`build/vlnav/objects-chair/map.png`)
- 对象、最佳图像引用、关联与重载记录 (`build/vlnav/objects-chair/objects.json`)
- `clouds.npz`：背景观测点和各实例点云；`tracker.json`：现有跟踪器持久状态；
  `views/object_*.png`：各候选的最佳原始观察图像。
- `build/vlnav/masks-chair/`：服务器实际分割结果；
  `build/vlnav/server/masks-chair.log`：推理日志。

服务器分割命令：

```bash
cd /root/autodl-tmp/lingtu-vlnav
YOLO_AUTOINSTALL=false vision/bin/python habitat_objects.py segment apt_0 \
  --cues cues-chair/report.json --weights FastSAM-s.pt --output masks-chair
```

下载 `masks-chair/` 后，本地构建命令：

```powershell
.venv\Scripts\python.exe tools/datasets/habitat_objects.py build build/vlnav/apt_0 `
  --masks build/vlnav/masks-chair/masks.json --output build/vlnav/objects-chair
```

新增 4 个测试通过，现有投影 4 个测试通过，Ruff 通过。测试涵盖坐标手性、轴方向、
两个旋转视角投影到同一已知点、米制深度与掩码、关联后的最佳原图保存及点云重载。
这一步完成了**已有 RGB-D 观测中的三维对象构建与关联实验**；下一步仍需处理掩码
污染、移动视角验证和原生地图绑定，再接观察位置规划。不能把这份产物直接当运动地图。

## 2026-10-08 — Habitat 连续移动找物实验

本轮新增三个实验工具，把上一节的静态观测改成连续 episode：

- [`habitat_world.py`](../../tools/datasets/habitat_world.py) 在 Habitat-Sim 0.3.3 中
  驱动相机代理体沿 NavMesh 转向/前进，并返回每次动作后的 RGB-D 与位姿。
- [`habitat_vision.py`](../../tools/datasets/habitat_vision.py) 常驻加载
  `yolov8s-worldv2.pt` 和 `FastSAM-s.pt`，逐帧检测、分割并投影三维点。
- [`habitat_episode.py`](../../tools/datasets/habitat_episode.py) 更新占用观测、对象跟踪和
  候选排序；每次移动后必须重新观测，再决定继续探索或接近候选。

运行时没有读取 ReplicaCAD 的对象语义标签。路径几何使用官方 NavMesh，位姿直接来自
模拟器；这两项是实验 oracle，不能当作 SLAM 或 LingTu 原生三维规划已经接通。
场景中的物体保持静止，“动态”指代理体连续运动、视角变化和在线更新对象，不代表动态
物体或人员跟踪。

### 检测、分割和对象关联依据

[VL-Nav v7](https://arxiv.org/html/2502.00931v7) 明确使用 YOLO-World 与 FastSAM，
但没有公开从框提示掩码到 RGB-D 清理、跨帧关联和融合的完整实现细节。本轮不能称为
作者代码复刻。为了处理 FastSAM 掩码带入远处背景的问题，本项目按深度连续性提取连通
区域，并保留距离相机最近的显著区域；这是 LingTu 的工程选择，不是论文原文算法。

[ConceptGraphs（ICRA 2024）](https://arxiv.org/html/2309.16650) 及其
[官方代码](https://github.com/concept-graphs/concept-graphs) 提供了更完整的对象地图参考：
掩码与深度投影、体素化/DBSCAN 去噪、几何重叠与 CLIP 相似度关联、再融合对象几何。
此轮实验只使用了掩码投影、深度区域清理和现有跟踪器的几何/同类距离关联，未接
CLIP 实例特征。下文补充实验接通了对象裁剪图特征；DBSCAN 和 ConceptGraphs 的
完整融合链仍未实现。
原文已下载为 [conceptgraphs.pdf](papers/conceptgraphs.pdf)。

本轮还修正了两个会直接改变对象地图的错误：默认贪心关联现在保证同一帧的一对一匹配，
不会让两个检测同时领取同一个已有或刚创建的对象；投影入口会拒绝非有限或非正的
`fx/fy`，不再静默替换成 `1`。

### 实际运行结果

| episode | 结果 | 观测 / 动作 | 路程 | 对象 | 接近证据 |
| --- | --- | ---: | ---: | ---: | --- |
| `chair`, seed 7 | 接近后重新观测候选 ID 2 | 28 / 27 | 3.600 m | 4 | 选中后移动 0.200 m，固定目标距离减少 0.193 m；最终水平距离 1.318 m，18 次观测 |
| `chair`, seed 8 | 接近后重新观测候选 ID 3 | 15 / 14 | 1.800 m | 4 | 选中后移动 0.200 m，固定目标距离减少 0.194 m；最终水平距离 1.267 m，13 次观测 |
| `water bottle`, seed 7 | 预算耗尽 | 35 / 34 | 2.400 m | 0 | 0 检测、0 对象；没有虚构成功或重载通过 |

三轮的 `collision_microsteps` 均为 0，只表示 NavMesh 相机代理体没有报告微步碰撞。
椅子 episode 的终止条件要求：选定固定对象后至少平移 0.15 m、与该固定位置的距离至少
减少 0.10 m，并在接近位置再次检出候选。它证明循环确实发生过，不能换算成论文 SR、
语义真值准确率或四足机器人可执行性。

seed 7 在运行中 `step=5` 保存 2 个对象；`step=6` 没有检测，`step=9` 从新的真实视角再次
匹配对象 ID 1，观察计数增加。seed 8 在 `step=5` 保存 4 个对象，`step=6` 再次匹配
ID 2、3，计数增加。两轮最终保存/重载后的对象几何一致。水瓶没有产生对象，所以重载
关联结果记录为 `null`，不把两个空集合相等算成验证通过。

RGB-D 反投影使用实际栅格像素中心 `cx=319.5, cy=239.5`。与 Habitat 独立的逆图形
矩阵参考比较，seed 7/8 的最大差异分别为 `6.75e-7 m` 和 `4.50e-7 m`；这只证明两种
数值计算一致，不是物理标定或语义定位精度。旧的 `cx=320, cy=240` 写法在相同检查中
最大相差约 `0.00952 m` 和 `0.01143 m`，已修正。最早一次 seed 7 随机落在桌面小型
NavMesh 孤岛，无法形成地面探索；失败记录保留在服务器 `episode-chair-seed7`，正式
episode 改为从最大可通行岛采样。

证据已下载到本地，录像均为可解码的 H.264/YUV420P：

- seed 7： 搜索叠加录像 (`build/vlnav/episode-chair-live7/search-h264.mp4`)、
  第一视角录像 (`build/vlnav/episode-chair-live7/first-person-h264.mp4`)、
  摘要 (`build/vlnav/episode-chair-live7/summary.json`)、
  逐步记录 (`build/vlnav/episode-chair-live7/steps.json`)、
  运行中检查点 (`build/vlnav/episode-chair-live7/checkpoint.json`)、
  投影检查 (`build/vlnav/episode-chair-live7/geometry-checks.json`)、
  日志 (`build/vlnav/episode-chair-live7/episode.log`)。
- seed 8： 搜索叠加录像 (`build/vlnav/episode-chair-live8/search-h264.mp4`)、
  第一视角录像 (`build/vlnav/episode-chair-live8/first-person-h264.mp4`)、
  摘要 (`build/vlnav/episode-chair-live8/summary.json`)、
  逐步记录 (`build/vlnav/episode-chair-live8/steps.json`)、
  运行中检查点 (`build/vlnav/episode-chair-live8/checkpoint.json`)。
- 水瓶负例： 搜索叠加录像 (`build/vlnav/episode-bottle-live7/search-h264.mp4`)、
  第一视角录像 (`build/vlnav/episode-bottle-live7/first-person-h264.mp4`)、
  摘要 (`build/vlnav/episode-bottle-live7/summary.json`)、
  逐步记录 (`build/vlnav/episode-bottle-live7/steps.json`)。

搜索录像分别为 28、15、35 帧（3 fps），第一视角分别为 117、61、158 帧
（12 fps）。固定帧率只方便查看，不代表含 CPU 模型推理的端到端实时速度。

服务器复跑示例：

```bash
cd /root/autodl-tmp/lingtu-vlnav
PYTHONPATH=source:source/src:/root/autodl-tmp/models/clip-vit-base-patch32 \
YOLO_AUTOINSTALL=false vision/bin/python source/tools/datasets/habitat_episode.py \
  --dataset data/replica_cad/replicaCAD.scene_dataset_config.json \
  --navmesh data/replica_cad/navmeshes/apt_0.navmesh \
  --habitat-python /root/autodl-tmp/conda/envs/lingtu-vlnav/bin/python \
  --yolo-weights yolov8s-worldv2.pt --fastsam-weights FastSAM-s.pt \
  --clip-weights weights/clip/ViT-B-32-state.pt \
  --output episode-chair-clip7 --seed 7 --max-steps 120 --observe-labels

ffmpeg -hide_banner -loglevel error -y \
  -i episode-chair-clip7/search.mp4 -c:v libx264 -preset fast -crf 20 \
  -pix_fmt yuv420p -movflags +faststart episode-chair-clip7/search-h264.mp4
```

模型和 Habitat 位于独立实验环境，没有修改 LingTu 的产品依赖锁文件。直接相关的
投影、关联、模型桥接和 episode 测试共 **57 passed**，涉及的 Python 文件通过 Ruff。
本轮没有运行 Qwen3-VL，没有使用数据集语义标签来搜索，也未接入原生三维规划器、SLAM、
四足动力学、动态物体或人员跟踪。两个椅子 episode 证明的是连续移动感知链已经可运行；水瓶仍需换用
确实含目标、且允许公平评测的官方场景或数据集继续验证。

### FastSAM-s 实测速度（2026-10-08）

在同一服务器 Intel Xeon Gold 6459C 上，以 CPU 4 线程、FP32、batch=1 测试
`FastSAM-s.pt`。输入为本轮 episode 的四张实际 640×480 RGB 图像，`imgsz=640`、
`retina_masks=True`，先预热 5 次，再测量 24 次。全图分割调用包含预处理、网络和
掩码后处理，平均 **53.64 ms**、中位数 **50.48 ms**、P90 **69.00 ms**，等效约
**18.64 帧/秒**。网络自身平均 40.35 ms；单独对两个框选择掩码并转成 NumPy，
平均额外 26.07 ms。首个分割调用约 1.20 s，以上稳态结果不包含该冷启动。

这是 FastSAM 分割部分的短测，不包含 YOLO-World、深度清理、三维投影、对象关联、
渲染、文件读取和导航。不能把视频回放帧率或该模型帧率当作完整感知/导航频率。
原始数据 (`build/vlnav/fastsam-speed.json`) 和
测速脚本 (`build/vlnav/fastsam-speed.py`) 已保存。

官方论文常见的 40 ms/图则是 YOLOv8-x 主干的 FastSAM、1024 输入、RTX 3090/PyTorch
测试，约 25 帧/秒，不能与本次较小模型的 CPU 测试直接比较。
[论文](https://arxiv.org/pdf/2306.12156)、
[官方模型与速度说明](https://github.com/CASIA-LMC-Lab/FastSAM#model-checkpoints)。

## 2026-10-08 — 补齐对象 CLIP 图像特征

此前 `habitat_vision.py` 为每条检测写入空的 `features`，因此上述连续移动
实验并未执行视觉特征关联。本次接通的是这条实际模型链路：

```text
Habitat RGB → YOLO-World 框 → FastSAM / 深度清理 → 有效对象观测
                         └→ 原始 RGB 对象框裁剪 → CLIP ViT-B/32 → 512 维单位向量
有效观测的点云 + 图像特征 → InstanceTracker 关联 → 特征 EMA / 点云融合 → 保存与重载
```

[`habitat_vision.py`](../../tools/datasets/habitat_vision.py) 通过现有 registry
构造 CLIP 编码器，每个 episode 只加载一次；每帧对成功投影的对象框批量编码。
Habitat RGB 先转为现有编码器要求的 BGR，编码器内部再转回 RGB 后裁剪。
重复掩码、深度无效和投影失败的观测不进入特征批次；无检测时不调用图像编码。
模型如果未返回有效特征，实验会报告错误，不能静默退回空特征。

这使用的是对象框内的图像编码，不是 YOLO-World 提示词的文本向量，也不是 LLM
输出的分数。当前未启用全图/裁剪/掩码三来源特征融合。跟踪器保留原有的语义强且几何
有重叠优先、双向几何强匹配、同类距离回退逻辑，以及原有质量加权 EMA；
本次没有重调关联门限。新建和重载的对象特征也统一归一化。
相似椅子的 CLIP 特征可能很接近，视觉相似度不能独立证明物体身份。

### 实际模型与连续运行结果

服务器使用 CPU 4 线程、OpenCLIP 3.3.0 和已有 OpenAI CLIP ViT-B/32 权重。
`open-clip-torch` 已在项目的 `perception` 可选依赖中声明；仅在独立实验环境
安装它及缺少的传递依赖 `timm`、`ftfy`，没有改动产品依赖或锁文件。

| episode | 观测 / 动作 | 路程 | 有效图像特征观测 | 最终对象 | 特征保存重载 |
| --- | ---: | ---: | ---: | ---: | --- |
| `episode-chair-clip7` | 28 / 27 | 3.600 m | 44 | 4，均为 512 维 | 维度和值一致，单位范数 |
| `episode-chair-clip8` | 15 / 14 | 1.800 m | 31 | 4，均为 512 维 | 维度和值一致，单位范数 |
| `episode-bottle-clip7` | 35 / 34 | 2.400 m | 0 | 0 | `null`，没有对象可验证 |

两个椅子案例均为 `candidate_reobserved_at_approach`。seed 7 在第 5 步重载，
第 9 步在新视角续接 ID 1；seed 8 在第 5 步重载，第 6 步续接 ID 2、3。
后续观测与关联对象更新前特征的余弦相似度分别记录了 40 次和 27 次，
范围为 **0.7881—0.9881** 和 **0.7837—0.9862**。
这是实际生成的图像特征相似度，不能解释为身份正确率或匹配概率。

seed 7 候选 ID 2 的第 10、27 帧原始对象裁剪图，其两个检测特征余弦相似度为
**0.869899**；裁剪图、维度和范数记录已保存。加入 CLIP 后，这两个固定种子的
路径、对象数和接近结果与此前一致。因此证据是缺失链路已接通并可持续运行，
不是准确率提升或 ConceptGraphs 完整复现。水瓶仍未检出，本轮没有实现“找到水”。

- seed 7：裁剪图对照 (`build/vlnav/episode-chair-clip7/clip-crops.png`)、
  对照数值 (`build/vlnav/episode-chair-clip7/clip-crops.json`)、
  搜索录像 (`build/vlnav/episode-chair-clip7/search-h264.mp4`)、
  第一视角录像 (`build/vlnav/episode-chair-clip7/first-person-h264.mp4`)、
  摘要 (`build/vlnav/episode-chair-clip7/summary.json`)、
  逐帧完整特征 (`build/vlnav/episode-chair-clip7/steps.json`)、
  保存对象状态 (`build/vlnav/episode-chair-clip7/tracker.json`)。
- seed 8：搜索录像 (`build/vlnav/episode-chair-clip8/search-h264.mp4`)、
  摘要 (`build/vlnav/episode-chair-clip8/summary.json`)、
  逐帧完整特征 (`build/vlnav/episode-chair-clip8/steps.json`)。
- 水瓶负例：搜索录像 (`build/vlnav/episode-bottle-clip7/search-h264.mp4`)、
  摘要 (`build/vlnav/episode-bottle-clip7/summary.json`)、
  逐帧记录 (`build/vlnav/episode-bottle-clip7/steps.json`)。

全部保存/重载与连续关联检查使用本仓库的编码器、跟踪器和序列化代码。
相关本地测试 **75 passed**，覆盖注册工厂本地权重参数、RGB/BGR 与裁剪顺序、
归一化、语义关联、远处同特征对象不误合并、保存重载和 episode 回归；涉及文件通过 Ruff。
这些测试是本地契约证据，上表另为实际 Habitat 模型运行证据。仍未接入 Qwen3-VL、
实机输入、SLAM、原生三维规划或四足动力学，也未做数据集实例真值评测。

```powershell
.venv\Scripts\python.exe -m pytest tests/perception/test_clip_encoder.py `
  tests/perception/test_perception_backends.py tests/perception/test_instance_tracker.py `
  tests/perception/test_detection_projection.py tests/tools/test_habitat_vision.py `
  tests/tools/test_habitat_episode.py -q
```

三轮的六份 H.264 视频均在本地成功解码首帧和末帧，帧数与观测/微步记录一致；
视频检查记录 (`build/vlnav/clip-video-checks.json`) 留存了实际结果。

### 本地权重格式与复跑

服务器已有的 `weights/clip/ViT-B-32.pt` 是官方 TorchScript 文件。
OpenCLIP 当前的本地加载接口需要普通 state dict；直接加载曾先后报
`weights_only=True` 和三个额外元数据键错误。导出参数并移除这三个非学习参数后运行成功，
模型权重来源没有改变。导出脚本和两次失败日志保存在本地 `build/vlnav/` 中，
以下命令保留可复跑的处理方法：

```python
import torch

model = torch.jit.load("weights/clip/ViT-B-32.pt", map_location="cpu")
state = model.state_dict()
for name in ("input_resolution", "context_length", "vocab_size"):
    state.pop(name, None)
torch.save(state, "weights/clip/ViT-B-32-state.pt")
```

按上文服务器命令传入 `--clip-weights weights/clip/ViT-B-32-state.pt`。
seed 8 使用 `--seed 8 --output episode-chair-clip8`；
水瓶负例增加 `--prompt "water bottle" --max-steps 35 --output episode-bottle-clip7`。
完整原始 RGB-D 留在服务器；本地下载了摘要、完整逐帧特征、对象状态、点云、日志、
视频和选定视角的原始 RGB-D。固定视频回放帧率仍不代表含 CLIP 的端到端实时速度。

## 2026-10-09 — 通用对象观察、关系证据与独立身份评测

本轮将观察从单个任务类别扩展为任务目标加 `chair, table, cabinet, shelf, sofa,
bed, bottle, cup, door, refrigerator, sink, television`。这是可配置的有限观察词表，
不是已经识别所有物体。每个有效对象都经过同一条检测、分割、深度清理、三维投影、
CLIP 图像编码和关联链。上下文对象进入地图；只有与任务目标匹配的类别参与方向线索、
接近候选和终止判定，不能用看见桌子代替找到水瓶。

- [`habitat_vision.py`](../../tools/datasets/habitat_vision.py)：组合观察提示词，批量编码所有有效对象。
- [`habitat_episode.py`](../../tools/datasets/habitat_episode.py)：目标过滤、移动后重观测、视图记录、保存重载和实验输出。
- [`habitat_scene.py`](../../tools/datasets/habitat_scene.py)：对象特征/观测证据导出、少量几何关系和关系查询。
- [`habitat_identity.py`](../../tools/datasets/habitat_identity.py)、
  [`habitat_world.py`](../../tools/datasets/habitat_world.py)：独立实例图及关联评测。

### 对象图实际保存的内容

`tracker.json` 保留本仓库跟踪器的持久状态；`scene.json` 额外输出每个对象的地图位置、
512 维融合图像特征、检测分数、观察计数和最多三个有区别的真实 RGB 视图。
新视图相对所有已存视图至少平移 0.3 m 或旋转 20°，避免把相同相机位置的连续帧
当成多视角证据。每条视图记录包含原图路径、框、帧序号、分数和相机位姿。
对象几何的坐标系为 `habitat_z_up`，原始相机位姿标为 `habitat_y_up`。

目前的 `description` 是“检测类别、已存视图数量与帧号”的可核对摘要，
`vlm_caption=false`。本轮没有运行图像描述 VLM，也没有生成颜色、材质、用途等外观描述。
不能把视图摘要称为多视角 VLM 理解。

关系复用 `SceneGraphBuilder` 的 `near`：三维近似包围盒间隙小于 1.5 m。
对象 extent 仍是原有二维框/深度估计，不是完整三维表面包围盒，因此关系也只是
几何候选证据。每个对象按盒间隙、中心距离选择最近两条已有 `near` 边，然后取并集；
一个热门参照物最终可以超过两条边。它是便于检查的稀疏图，不是所有关系的穷举。
没有添加未经证明的 `on`、`inside`、房间或功能关系。

查询复用现有 `GoalResolver.fast_resolve`，没有调用 LLM，也尚未启用查询词与
对象图像特征的文本—图像相似度。关系指令必须有目标、
参照物及连接二者的实际图边；返回的 candidate 也必须绑定到该边。
`grounded` 只表示在**检测候选图**中绑定成功，所有结果同时明确输出：

```json
{
  "grounding_scope": "detector_candidate_graph",
  "semantic_verification": "not_performed",
  "motion_authorization": false
}
```

缺少保留的关系边表示当前图没有足够依据，不表示真实场景里不存在该关系。
这些查询仅供检查，不派发运动目标。

### 两轮实际 Habitat 运行

场景为官方 ReplicaCAD `apt_0`，使用 Habitat-Sim 0.3.3 与前述 YOLO-World、
FastSAM-s、CLIP ViT-B/32 权重。没有把官方类别或实例 ID 送入检测、关联、探索评分或运动选择。

| episode | 观测 / 动作 | 路程 | 有效特征观测 | 候选对象 / 预测类别 | 多视角对象 | 保留 near 边 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `episode-general-chair7` | 28 / 27 | 3.600 m | 69 | 9 / 6 | 5 | 9 |
| `episode-general-cup8` | 31 / 30 | 4.346 m | 303 | 35 / 8 | 21 | 46 |

第一轮预测类别为 bed/chair/shelf/sofa/table/television，第二轮为
bottle/cabinet/chair/cup/shelf/sink/table/television。类别列表是模型预测，不是正确类别统计。
两轮均在接近后重新检出目标候选，且对象几何与特征保存重载一致；所有持久特征都是
512 维单位向量。第二轮有 **4 个 Habitat 碰撞微步**，不能称为无碰撞执行。

原始密集关系图有 9 / 219 条边。两轮运动完成后，使用当前导出器从相同保存状态重新
导出稀疏图；这没有重新运行感知或运动。原图和原查询保留为 `scene-dense.json`、
`queries-dense.json`。稀疏化使第二轮的错误瓶子候选不再通过已有查询，但**没有修复**
该候选的错误类别或关联算法。

### 几何对齐和对象关联分别验收

几何投影与 Habitat 独立逆图形矩阵参考的最大差异分别为 **6.75e-7 m**、
**7.47e-7 m**。另有两个相机平移后投影同一已知静态点的本地测试。
这证明在给定精确仿真位姿、内参和轴约定时，点能进入同一地图坐标系；
没有验证真实 SLAM 漂移、时间同步、相机/LiDAR 外参或跨次地图定位。

对象关联不再以“再次出现相同 ID”作为唯一证据。仿真器为每个刚体配置独立实例 ID，
**在跟踪更新之后**读取同位姿的实例图。只有一个刚体覆盖预测掩码至少 60% 的像素时，
才给这条观测归属；其余记为 ambiguous。静态场景部分没有同样的实例标注，
因此覆盖率不是关联准确率，未归属观测也不自动算对或错。

| episode | 可归属 / 全部观测 | 轨迹内 oracle 身份变化 | 混入多个对象的轨迹 | 被拆成多个 ID 的真实对象 |
| --- | ---: | ---: | ---: | ---: |
| chair7 | 67 / 69 | 0 | 0 | 1 |
| cup8 | 103 / 303 | 4 | 3 | 5 |

`identity_switch_count` 统计同一轨迹内相邻可归属观测的真实对象 ID 变化，
不是宣称已经实现标准 MOT 指标。chair7 的显示器
`frl_apartment_monitor_:0000` 被屏幕/下部支撑观测拆成 ID 0、3。
cup8 的 ID 3 混入椅子和 beanbag，ID 27、32 在两个相邻 kitchen utensil 之间切换。
当前实验使用逐观测匹配及同帧一对一约束；库内另有 Hungarian 分配模式，但本轮
没有启用或验证它。提高 CLIP 相似度、合并邻近同类对象都不能直接证明身份问题已解决。

### 一个必须保留的错误语义查询

cup8 第 12 步，候选 **ID 22** 被 YOLO-World 标为 `bottle`，分数 **0.256675**，
只有一次观测。实例图的 220 / 307 像素归属 oracle `1000052`，
对应 `frl_apartment_handbag_:0000`；原始裁剪也能看到手提包及提手。
原密集图却让“找到桌子旁边的水瓶”绑定到它，fast resolver 排序分数为 **0.86285**。
该分数由标签和空间关系等启发式组合得到，不是目标存在概率。
这是错误标签传播到关系查询的反例，绝不是找到水的成功案例。

cup 候选 ID 17 的 13 条独立归属记录始终对应同一个
`frl_apartment_kitchen_utensil_06_:0000`。官方配置的类别 46 是宽泛的
`kitchen_utensil`；图像可见小型容器，但不能仅凭这个类别或低分辨率裁剪确认 cup 子类。
官方 bottle/cup 类别分别为 14/28，当前已安装 ReplicaCAD 资产中没有 bottle 类对象。
水瓶真实性与瓶内有水都仍未证明。

### 结果入口与复跑

- 地图、真实多视角及反例拼图 (`build/vlnav/general-evidence.png`)
- chair7：录像 (`build/vlnav/episode-general-chair7/search-h264.mp4`)、
  摘要 (`build/vlnav/episode-general-chair7/summary.json`)、
  对象图 (`build/vlnav/episode-general-chair7/scene.json`)、
  身份评测 (`build/vlnav/episode-general-chair7/association.json`)。
- cup8：录像 (`build/vlnav/episode-general-cup8/search-h264.mp4`)、
  第一视角 (`build/vlnav/episode-general-cup8/first-person-h264.mp4`)、
  摘要 (`build/vlnav/episode-general-cup8/summary.json`)、
  逐帧证据 (`build/vlnav/episode-general-cup8/steps.json`)、
  稀疏对象图 (`build/vlnav/episode-general-cup8/scene.json`)、
  原密集图的错误查询 (`build/vlnav/episode-general-cup8/queries-dense.json`)、
  候选身份与分数核对 (`build/vlnav/episode-general-cup8/findings.json`)、
  身份评测 (`build/vlnav/episode-general-cup8/association.json`)。

四份 H.264 视频已完整解码；搜索录像为 28 / 31 帧，第一视角为 117 / 128 帧。
检查记录 (`build/vlnav/general-video-checks.json`) 保存了实际帧数和首末帧尺寸。
固定回放速度不代表模型实时频率。

```bash
cd /root/autodl-tmp/lingtu-vlnav
PYTHONPATH=source:source/src:/root/autodl-tmp/models/clip-vit-base-patch32 \
YOLO_AUTOINSTALL=false vision/bin/python source/tools/datasets/habitat_episode.py \
  --dataset data/replica_cad/replicaCAD.scene_dataset_config.json \
  --navmesh data/replica_cad/navmeshes/apt_0.navmesh \
  --habitat-python /root/autodl-tmp/conda/envs/lingtu-vlnav/bin/python \
  --yolo-weights yolov8s-worldv2.pt --fastsam-weights FastSAM-s.pt \
  --clip-weights weights/clip/ViT-B-32-state.pt \
  --output episode-general-chair7 --seed 7 --max-steps 60 \
  --query 'find chair near table' --query 'find cup near table' \
  --query '找到桌子旁边的水瓶'
```

第二轮改为 `--prompt cup --seed 8 --max-steps 40 --output episode-general-cup8`。
默认观察完整词表；`--observe-labels` 后不跟值可复跑此前单目标对照，或显式传入少量
上下文类别。模型权重、原始完整 RGB-D 和实例图留在服务器；摘要、完整特征、保存状态、
点云、视图引用所需 RGB、录像及评测已下载到本地。

```powershell
.venv\Scripts\python.exe -m pytest tests/tools/test_habitat_vision.py `
  tests/tools/test_habitat_scene.py tests/tools/test_habitat_identity.py `
  tests/tools/test_habitat_episode.py tests/perception/test_instance_tracker.py -q
```

本轮直接相关测试 **57 passed**，涉及源码和测试通过 Ruff；只有既有 jieba 的
`pkg_resources` 警告。测试包括目标/上下文分离、两个相似相邻椅子换检测顺序、
已知平移投影、视图去重复、图边绑定及保存重载。
没有新增项目依赖。通用观察和图像特征链已补齐；可靠身份、VLM 外观核验和真正找到水
仍是未完成项。下一步先修同一对象的零件拆分、相邻对象串轨和弱错误类别被查询放大，
再增加更复杂关系；不应把更多文字描述当成身份可靠性的替代。
