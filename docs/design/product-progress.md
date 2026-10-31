# 灵途产品推进计划与完成状态

更新：2026-10-23。以可复核的结果标记完成，不把模型预览、单元测试、真实仿真和安装包混为一谈。

## 当前主线：实际工厂到同一个 Product

- [x] 实际生产线首段 27 个静态组件、122 个原始凸几何；整组件保留，未用 AABB 或平面替代。
- [x] 精确迁移 99 个材质／网格／贴图文件约 438.8 MiB；独立 UE 关卡已保存、重载并核对。
- [x] 实际 Thunder 静态挂载／观察位置与源几何核对；拒绝与输送线相撞的观察草案，保存真实原始采样。
- [x] 完整工厂 4.9 s 零指令物理观察及原始接触记录；保留瞬态，不等同于驾驶或出生资格通过。
- [ ] 最终 World/出生/渲染验收与 Product session 接入。
- [x] 恢复 Windows mapd 已有管理端点；原 graph 125 项与新接线 3 项通过，Linux 未改变。
- [x] Windows PGO 链接修复：独立同版本静态 GTSAM SDK，当前源码新 exe；原共享 SDK 未改。4 个优化器原生测试通过，Windows RunPlan 显式绑定新程序。
- [x] 原生合成地图保存／重载：真实 PGO、SaveMapEngine、3D OctoMap 构建、重开存储并严格加载；失败替换保留原地图。129 项配置回归、30 项传输回归、原生保存回归通过。
- [ ] 真实工厂建图、保存、重载，使用返回的真实地图身份进入 Product 导航。上面的合成夹具不是这项验收。
- [ ] 同一个 Product 原生导航到点、停稳、匹配采集、原始数据、三态判断与原生任务终态。
- [ ] 障碍停车、人工接管／释放、统一现有传感器控制；之后扩展新模态。

最新文件、实际参数、失败记录与未完成原因见
[工厂 README](../../sim/tools/worlds/factory_environment_collection/README.md) 和
[传感器交付台账](sensor-delivery-status.md)。以下保留首页／旧 UI 及前期清单的历史记录。

## 已完成

- [x] **默认工厂首页（Editor 运行版）**：`LingTuFrontEnd` 为 GameDefaultMap；黑色展台不再是默认入口；实际首页截图与正常退出已验证。
- [x] **工厂背景接入**：使用已领取 Factory Environment Collection 的干净 UE 渲染图；不是概念图，也没有为首页加载整套原始工厂。
- [x] **机器人／地图两步选择组件**：分类、搜索、机器人三维查看、地图预览、返回与精确配置选择已有实现；不得把模型可预览等同于可驾驶。
- [x] **画面／操作／界面／声音设置基础**：实际 Slate 控件、修改反馈、保存、撤销、恢复默认、未保存返回决策已接入；不代表所有 3A 设置选项均已完成。
- [x] **背景音乐与界面音效基础**：两首 32 秒原创电子氛围草稿、三个短提示音；主／音乐／界面音量与静音分开；首页播放启动已见真实日志。听感和最终配乐仍待打磨。
- [x] **素材来源记录与 D 盘存放**：`sim/library/SOURCES.md` 记录来源、许可与接入边界；完整工厂原始包留在 D 盘外部素材库。

上述完成证据主要见 `factory-front-end-and-music.md`。旧独立打包版尚未重做。

## 当前推进

- [x] **发布入口／资源清单同步**：Shipping 显式地图列表已改为新首页与已有运行地图，不再显式 Cook 旧黑色展台；资源登记测试包含新背景与选择页缩略图。入口、资源、发布模块合计 **33 项测试通过**；并未执行真实 Cook。
- [x] **新首页后的 UI-only 选择流程验收**：2026-10-21 同一次真实 UE 启动完成首页、机器人、分类／三维查看、地图选择截图；原生 QA 驱动推进页面，未提前启动物理、未确认会话／写出会话意图，success.json 与自然 exit0 齐全。不是人工输入全流程或运行场景验收。
- [x] **运行入口排除制作工具**：选择器和 Editor-backed runtime 共用六项插件排除参数，保留渲染插件；正常编辑器制作和 packaged 路径不变，相关 **118 项测试通过**。
- [ ] **快速启动**：新一轮初始化 109.80 秒，低于前一轮 191.51 秒，但仍不达快速启动要求；内存、DDC 与编辑器模块加载继续定位，不将全部改善归因于插件修复。
- [ ] **选定工厂生产线接入**：以 Factory Environment Collection 为目标，不冒用旧 FactoryPark 或 factory_workshop。迁移范围、三点巡检、权威、证据与下一步统一记录于 [工厂 README](../../sim/tools/worlds/factory_environment_collection/README.md)。
- [ ] **FactoryInspectionBay 选区正式验收**：已完成 59 个源组件／184 个源碰撞几何的 enclosed candidate、UE 三视图自然退出与 Thunder 10.1 s 零指令站立稳定性检查；`factory-enclosed-render-03` 已比前一版改善曝光，但选区外墙／背景仍未闭合，且尚未生成正式 World/session。
- [ ] **扩大为完整巡检纵切片**：已离线生成 40×36 m 路线范围，459 个源组件、1064 个原始几何，六个候选站位（起点、Control22、Engine、第二控制台、工业机器人单元、终点）；物理候选已单独生成并列出 13 个视觉阻塞组件，UE 路线关卡已保存并 reload 验证，新增 252 个资源、约 1.34 GB 未 Cook 数据，仍未获得出生、导航或 World 资格。
- [x] **工厂 StaticMesh 选区实测**：真实源地图输出 458 个候选 Actor、100 个不同网格及 117 个跨边界项；源地图未保存。该清单不含完整骨骼／灯光／特效／依赖，尚未迁移或资格化。
- [x] **巡检底层定向验证**：178 项 Python 巡检测试、121 项传感器合同、4/4 原生 C++ 测试目标通过；新增三点采集、三轮重跑、迟到证据与中点失败回归。均非新工厂联合实跑证据。
- [x] **传感器实际接入记录**：[运行传感器 README](../../sim/runtime/sensors/README.md) 已区分五条已有流、GPS／热成像／声学缺口、UE 30 Hz 与 headless 1 Hz 相机及 MID-360 噪声开关；所有传感器统一噪声控制仍未完成。

### 2026-10-21 最新验证

`build/unreal-tests/product-progress-20261021/runtime-plugins-flow/result.json`：passed=true、
exit_code=0、natural_exit=true，整轮 136.641 秒。1920×1080、30 FPS 上限、保留声音；
五张 UI 截图齐全，地图和三维查看截图已人工检查。没有启动 MuJoCo，没有真实运动或
巡检完成证据。更早失败记录如下保留，不代表最新 UI-only 流程仍失败。

源选区清单位于 `build/unreal-tests/product-progress-20261021/factory-bay-intake-api-corrected`：
真实读图、自然 exit0、183.141 秒，源地图未保存；候选并非完整关卡或碰撞世界。
本轮最终组合回归 **120 passed**，Ruff／相关 diff whitespace 通过，相关进程最终扫描为空。

本轮首个 UI-only 验证保留于 `build/unreal-tests/product-progress-20261020/selection-flow`：
测试选用了缺少嵌入式素材审阅目录的旧选择目录，原生驱动明确报出
`asset_review_catalog_unavailable_or_empty`，因此没有成功截图。UE 自然退出，外层进程
exit0 不代表验收通过；脚本随后因没有 success.json 报错。补齐目录后使用独立输出重验，
不覆盖首次失败现场，也不改变生产端可用性检查。

补齐目录后的真实 UI-only 验证仍**未通过完整流程**：

- 输出目录：`build/unreal-tests/product-progress-20261020/selection-flow-complete-catalog`。
- 实际截图：`sim/runtime/visual/RobotSimUE/Saved/Automation/LingTuSimUI/FrontEnd/product-flow-complete-20261021`，
  已有 1920×1080 首页 `login.png`、机器人选择 `asset-library.png`，以及两个机器人展示图。
  机器人选择截图已人工查看；它不是机器人运动证明。
- `Unreal.log` 记录 Engine Initialization **191.51 秒**；随后静态网格编译预算
  **1345.01 MiB > 1132.69 MiB**，并出现一次音频 buffer starvation。
  日志最后处于 G1 展示／分类阶段，没有地图截图与 success.json。
- 外层总限时 **240 秒**触发；finally 关闭本次 owned tree，因此不是自然成功退出。
  未启动 MuJoCo、未确认会话、未生成 selection.intent.json，没有运行 Cook／UAT／项目构建。
- 首次收尾扫描无相关进程；最终复核仍无 UE、ShaderCompileWorker、UBT/UAT、dotnet、
  cl/link、MuJoCo。本轮 UE PID 67972 已不存在。后来另见 MSBuild.exe PID 47816，
  创建时间 2026-10-21 00:36:36，晚于本轮 UE 最后日志 00:16:26；命令为 VS2022
  `MSBuild.exe /noautoresponse /nologo /nodemode:1 /nodeReuse:true /low:false`，
  父 PID 73500 已不存在。它不属于本轮启动的 owned tree，未操作／终止；
  因此不将最终全机状态描述为“MSBuild 也全部清空”。

接下来先分开统计引擎初始化与展示资源加载，检查 DDC 命中、内存和同步加载点；
不把扩大等待时间当作性能修复，也不将本次局部截图计为完整交互验收。

## 待完成，按顺序推进

### 2026-10-24 当前三步闭环

1. **选区渲染、碰撞、出生点和 World/session**：渲染与碰撞候选已有证据；视觉边界、正式出生点、WorldImporter qualification 和 session binding 仍未完成。
2. **同一 Product 建图保存／重载**：尚未启动；必须等待本工厂正式 World/session，禁止复用旧 FactoryPark 或零坐标 compile fixture。
3. **原生导航到一个设备点并巡检采集**：尚未启动；必须使用本次保存地图，原生导航停稳后再保存匹配 RGB／热像／声学原始数据并输出正常／异常／不可判定结果。

当前唯一有效的推进证据目录是 `build/inspection-tests/20261023/factory-enclosed-bay-01/`、`factory-enclosed-render-03/`；它们明确区分了候选、渲染检查和物理站立检查，不能直接当作 Product 运行成功。

传感器本轮增量（2026-10-21）：

- [x] MATRiX 传感器清单、三条局部机器码检查与 REA/Ghidra 定向伪代码恢复；初轮超时、
  后续 procedure metadata 超时、GPS 尾部截断和首次关闭警告均保留，未将其当成完整程序或测温证明。
- [x] GNSS 实验测量模型、显式噪声／失锁开关和 SensorRuntime 端点／证据合同。
- [x] GNSS Catalog／rig／schema／可选生产端点、native DDS Fix/Status；现有 Thunder
  实际 MuJoCo 快照 → 独立 DDS 订阅器通过，覆盖 FIX、NO_FIX、reset 和自然关闭。
  联合回归 217 passed／30 schema skipped；随后现有 Miniconda 环境补跑 Catalog/schema
  68 passed／0 skipped，覆盖跳过项（套件有重叠，不累加）。原生实际链路单独 1 passed；默认导航未改变。
- [x] GNSS 正式 Product 配置编译／feeder 源码接线：显式 session 复用认证 IMU 连接，
  无新增进程；原生端会话、时间和流校验通过；最终 Python→原生 DDS 实收 5 fix＋6 status。
  201 项直接回归通过；真实 MuJoCo 动态测量测试修正了 post-step 位姿滞后；独立复审 CLEAR。
- [ ] GNSS 整套 Product 运行验收、Gateway/UI、定位融合与新工厂实跑；端点通过不等于巡检验收。
- [ ] 热成像原始温度测量、设备热属性与巡检证据；声学压力采样／传播与巡检证据。

详见 [MATRiX sensor investigation](../../research/matrix/SENSORS_20261021.md) 与
[当前传感器状态](../../sim/runtime/sensors/README.md)。

| 优先级 | 内容 | 现状 | 完成条件 |
| --- | --- | --- | --- |
| P0 / 1 | 快速启动＋首页→选择→加载→运行→返回 | UI-only 流程已通过；初始化仍为 109.80 秒，完整运行／返回未验收 | 优化缓存／加载；只经 ProductControl 启停；真实 readiness 才可驾驶；失败可返回；退出无残留 |
| P0 / 2 | 可驾驶工厂区域 | 已选 Factory Environment Collection 生产线；StaticMesh 选区清单已输出，跨界裁剪与其他依赖未完成 | 先选一个车间／走廊，迁移所需依赖；同源 UE 视觉与 MuJoCo 碰撞、尺度／坐标对齐 |
| P0 / 3 | 现有 Thunder 真正运动与碰撞 | 控制和同步基础已有；不能用首页或展示动画证明运动 | 同一次运行提供位移、转向、接触、松键停止与重置真值；UE 不另算一套机器人动力学 |
| P0 / 4 | 最小产品性能门 | 新界面能显示，不等于高保真工厂性能达标 | 在目标机器测 1920×1080 主视口、RTF、帧时间；所选 RGB/Depth 流保持 640×480@30，不靠降低资格规格过关 |
| P1 / 5 | 巡检任务完整闭环 | 原生任务、引导、结果有基础链路 | 原生任务驱动目标／路线／距离／进度；自主巡检可人工接管；停稳采集、下一点、完成或失败有真实报告 |
| P1 / 6 | 传感器逐项验收与噪声控制 | 各类实现成熟度不同，需按实际源代码和样本复核 | RGB、Depth、LiDAR、IMU、GPS、热成像、声学分别列支持程度；噪声统一显式开关、参数、可重复样本；不以 UI 音乐冒充声学传感器 |
| P1 / 7 | 可交互场景资产 | 素材库有模型／部件运动基础，不等于物理合格 | 每项明确静态碰撞、可推动刚体、铰链／旋转部件或车辆；真实关节／碰撞接线；保留许可与代理来源 |
| P1 / 8 | 资产／机器人／地图管理 | 本地记录、目录与选择 UI 已有；完整管理器未完成 | 导入、校验、预览、依赖与版本、可运行状态；不合格资产不能伪装可用 |
| P1 / 9 | Windows 独立分发 | 已有 UE 发布工具和策略；新版本尚未 Cook、安装或完整运行 | 实际 Cook/Stage；测量 Pak/IoStore 和插件大小；补齐外部 MuJoCo／DDS／Bundle 依赖；独立启动、返回与卸载验收 |
| P2 / 10 | 更多机器人与动态行人／车辆 | 模型数量不代表控制器／行为可用数量 | 逐机器人接控制器和传感器；逐场景接真实动态状态、碰撞与任务，不只播放动画 |

## 体积结论与约束

以下为本机源文件统计，不是最终压缩安装包大小：

| 项目 | 字节 | 约占空间 | 是否本轮新增运行媒体 |
| --- | ---: | ---: | --- |
| 工厂背景 PNG | 3,042,859 | 2.90 MiB | 是 |
| 两首导入后的 SoundWave | 2,268,649 | 2.16 MiB | 是 |
| 新入口地图 | 6,421 | 6.27 KiB | 是 |
| 上述合计 | 5,317,929 | 5.07 MiB | 最终 Cook 后大小尚未测量 |
| 原始音乐制作文件／说明 | 6,145,017 | 5.86 MiB | 制作源文件；不作为独立运行媒体登记 |
| 当前项目 Content 全部 | 535,531,820 | 510.72 MiB | 大部分是已有资源，不代表都会 Cook |
| 其中机器人展示库 | 150,543,338 | 143.57 MiB | 已有三维选择资源，不随意删除支持的模型 |
| 外部完整工厂原始缓存 | 9,349,903,643 | 8.71 GiB | 不在 RobotSimUE 的 Content 中 |

采用“轻量首页背景＋按需迁移的可玩区域”，不采用“首页加载完整工厂”。
控制包体的重点是显式地图与运行资源、依赖裁剪、合理贴图／LOD和实际 Cook 统计。
不删除作者源文件、下载缓存或用户模型来伪造包体变小，也不把 DDC、Intermediate、Saved
和制作源文件的工作区占用当成产品安装体积。

最终包体尚未测量。不能从上述 5.07 MiB 推出“整个游戏只有 5 MiB”，也不能将外部
8.71 GiB 缓存直接当成这次首页增加了 8.71 GiB。
