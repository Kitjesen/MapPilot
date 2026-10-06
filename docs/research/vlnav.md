# VL-Nav 复现记录

更新：2026-10-06。主参考固定为 **arXiv:2502.00931v7（2026-07-14）**，
对应作者标注的 IROS 2026 工作，避免混用 2025 年旧版方法。

- [本地原文](papers/vl-nav.pdf)
- [作者项目页](https://sairlab.org/vlnav/)
- [固定版本方法与公式](https://arxiv.org/html/2502.00931v7)

目标是用户说“帮我找一瓶水”，系统根据实际图像找目标，未发现时主动探索，
发现候选后规划到观察位置，再核验是否符合指令。当前**未完成整条闭环**。
本页是后续实现和验收的入口；其他论文作为参考，不再同时追三套系统。

## 已实现到哪里

| 环节 | 已有内容 | 仍缺什么证据 |
| --- | --- | --- |
| 文本理解 | LingTu 的任务分解、物体查询、LLM 接口 | 带真实图像记忆的 Qwen3-VL 任务重规划 |
| 目标感知 | YOLO-World 检测与深度投影代码 | 官方场景中实际推理、误检/漏检记录 |
| 对象记忆 | SceneGraph、持久实例相关实现 | 最佳观察图像、位姿、置信度的完整读写与 VLM 检索 |
| 探索评分 | 本轮实现 v7 式 (3)—(8) 和 III-C 选择规则 | 实时地图输入、候选保留/去重、拒绝后的闭环重选 |
| 几何执行 | 原生观察点查询、路径预览、目标受理和状态回传 | 与此评分器连接，真实仿真轨迹 |
| 目标核验 | 现有接近后 VLM 核验与有限次换视角 | 模型在真实渲染图像上的判定与端到端终止记录 |

### 本轮代码

[`src/decision/frontiers/vlnav.py`](../../src/decision/frontiers/vlnav.py)：

- `vl_score`：置信度加权高斯混合 × 视野中心权重，截断至 `[0, 1]`；
  σ 使用论文给出的 `0.1 rad`，视野外不给视觉分数。
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

- [八方向原始视图](../../build/vlnav/apt_0/views.jpg)
- [客厅方向原始帧](../../build/vlnav/apt_0/04.png)
- [RGB-D、位姿和证据范围记录](../../build/vlnav/apt_0/report.json)
- [实际渲染日志](../../build/vlnav/server/probe.log)

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
