# Unitree Go2

### 2026-09-16 当前建图迁移与部署边界

源码中的 real map Product 已声明原生 LiDAR/IMU、SLAM、mapd、导航仲裁与驱动；
Host/Gateway 负责生命周期请求、适配及网页服务，不以此宣称整个建图产品已完成现场验收。
下面 `.31` 的安装记录是 9 月 15 日已验证的版本。9 月 16 日新增的“地面建模不受显示
点数上限影响”、静态点云可见点拾取及最新简化 Web 仅完成本地修复/构建，尚未安装 NX。
本次只读复核中 `127.0.0.1:12218` 和 `15052` 均拒绝连接，笔记本
`192.168.66.95:22` 超时，因此未读取到 NX 当前发布目录或当前运行状态。

局部投影的低位分位种子仍可能把多数低平台当作起始支撑面；此项尚未修复。
局部表面拟合不等于地面真值、完整碰撞检查或全局闭环质量；不能据静止数据更新和
本地回归宣称大范围建图、保存再加载、跨层或动态环境已完整验收。
`15176` 是明确标注示例数据的本地 UI 验证页，不是机器人实机页面。
当前证据见 `build/terrain-ground-model/mapping-ui-acceptance.md`。

### 2026-09-15 原生局部表面模型升级

已安装到 NX 当前 `.31` 发布目录：mapd、原生客户端、Host 数据接线和 Web。
新独立地形程序及对应 systemd 参数已同步，当前 map Product 不启动地形进程，
因此它的导航效果仅完成原生回归，尚未进行运动验收。安装保留 SLAM PID 3736 与
`product-4b18eea72695480d9df0fdd187b0fdb6` 建图会话。
回退文件：NX `/home/unitree/terrain-ground-install-20260915-230923`。
安装后 13 帧完整 SSE 场景持续更新约 2.03 Hz，三个诊断网格与投影坐标、代次一致；
这是 Web 场景发送频率，不是导航控制频率。浏览器已刷新并实际点格确认出现
“局部表面拟合 · 高度 / 残差 / 细 XY 支撑”，连接及实测关节信息正常。
浏览器最终版本已核对：选中局部投影时底栏显示“支撑候选 / 障碍回波 / 未确认”
与当前观测范围，修正了仍显示点云点数及点云缓存状态的错误标签。
新增 mapd 参数在启动时拒绝非正残差或小于 3 的支撑数量；NX 命令行错误分支已验证。
最后的选项/图例更新备份在 `/home/unitree/terrain-ground-final-20260915-231800`。

本轮用共享 C++17 表面拟合替换 mapd 的最低点高度参考，以及原生 TerrainCore
的邻格高度分位数估计。共享的是几何证据：mapd 负责建图投影，独立地形进程负责
导航地形输出，最终运动仲裁仍由导航持有。没有引入 ROS 或新的依赖。

- 每个 20 cm 建模格内按 5 cm XY 列统计支撑，每列一票；重复回波与竖直墙面点数
  不直接增加支撑票数。至少 3 个不同 XY 列且分布不能退化成一条线。
- 选择低位且具有面积支撑的高度簇，再做局部平面拟合。无效低簇不会阻止检查较高的
  合格簇；同列高度起伏保留在残差统计中，不被取平均隐藏。
- 只融合相互平面预测一致的邻格；台阶两侧不会通过平均变成假坡面。
  最终局部拟合残差 RMS 不超过 4 cm，几何拟合坡度上限 35°。
  这两个值是建模质量限制，不是允许 Go2 行走或爬楼的能力声明。
- 导航请求 5 cm 网格时，在至少 20 cm 区域建模，只投回实际观察到的小格。
  雷达盲区、支撑不足及残差超限保留未知，不以机身位置或邻域预测制造地面点。
- mapd 投影从机器人附近低处表面建立连通参考，沿局部坡面预测检查连续性。
  绿色表示有连通支撑候选，红色表示其相对高度 12–80 cm 内存在当前占据体素，
  灰色表示未确认。该显示不等同于导航全包络可通行判断。
- 保留原有三维占据、射线清除和地形历史机制；没有删除机器人周围的占据来消除红格。

Web 点选局部投影可查看同一 MapScene 的拟合高度、残差及细 XY 支撑数。
新增 `ground_height`、`ground_roughness`、`ground_support` 三个原生网格，
通过 DDS → C ABI → HostBus → Gateway → Web 传输；缺失或跨代数据不拼接。
原始 `maps.elevation` 的观测最低高度语义保持不变。MapScene ABI 为 3，
客户端 ABI 为 10，必须与 Host 一起安装。

验证证据：

- Windows 地面/投影回归，TerrainCore 12 项回归通过；覆盖低处杂点、坡面、
  台面与地面共存、线状退化、同列粗糙度、缺扫、细网格、盲区及障碍保留。
- NX/aarch64：4 组 maps 测试、MapScene 客户端回环测试、TerrainCore 12 项通过；
  mapd、客户端共享库和独立地形程序编译成功。测试 DDS 使用隔离测试域。
- Python 接线 17 项、Web 查询 13 项通过，TypeScript 与 Web 生产构建通过。
- 原生 MapScene 实测快照：176,520 个 voxel 点，10 m 滚动窗口；完整落在窗口内的
  建模网格为 49×49、20 cm。1,778 格有合格局部表面拟合，包含高处表面，
  不能把这个比例称为地面覆盖率。五次 NX 回放平均 22.19 ms，拟合残差中位数
  2.06 cm、95 分位 3.49 cm，均不是定位精度或地图对真实环境的测量误差。

代码：`src/maps/include/lingtu/maps/layers/ground_surface.hpp`；
证据与复现程序：`build/terrain-ground-model/`。
[实际点云拟合诊断图](../../../../build/terrain-ground-model/terrain-ground-before/terrain-ground-review.png)。

参考 [CMU terrainAnalysis](https://github.com/jizhang-cmu/ground_based_autonomy_basic/blob/noetic/src/terrain_analysis/src/terrainAnalysis.cpp)
的地形历史与障碍相对高度思想，以及
[ANYbotics elevation_mapping](https://github.com/ANYbotics/elevation_mapping)
的高度证据、估计质量和可见性处理原则。本轮没有移植后者的位姿协方差传播或完整概率融合；
`variance_m2` 只是空间拟合残差方差。只有台面而没有地面观测时，仅靠局部平面不能确定
它是否适合落脚，仍需机身高度参考、连通性、完整碰撞检查及现场验证。
真实坡面、台阶、移动遮挡和绕行运动尚未在本轮做现场验收；不能据静止回放宣称成熟度已等同参考系统。

### 2026-09-15 局部投影脚下障碍显示复查（待浏览器现场核对）

用户截图显示密集黄色格，视觉上与机器人重叠。本次只读采样 generation 6400：
`maps.surface_projection` 为 20 cm 网格，雷达位姿 XY 对应格为支撑候选 0；
不能将这一单格结果扩展为完整机身净空，也不能用晚于截图的数据否定现场现象。
服务端当前首页加载 `app-BRbB38ZX.js`，该文件选择 surface_projection，障碍色为红色；
截图的细黄色格与此不一致。已请求在停车后刷新核对，尚未确认旧标签页或其他叠加图层
是否为原因。浏览器检查接口不可用，不能宣称网页现场已验收。

同帧对比：[原始占据与当前地面相对投影](../../../../build/go2-surface-projection/footprint-projection-comparison.png)。
输入：`build/go2-surface-projection/footprint-live.sse` 和 `footprint-scene.json`。
本次没有修改分类阈值、清除机身周边占据、重启服务或发送运动目标；问题仍待定位，
不能记录为已修复。

### 2026-09-15 当前建图与参考项目对比

22:18–22:22 只读采样确认 Product 为 map，session.mode 为 mapping，
session.active_map 为空。地图目录保留的旧 active 名称不代表已进入旧图导航。

当前输出应区分：

- SLAM 几何点云：用于描述三维表面；完整保存 PCD 与网页显示缓存不是同一个数据产品。
- 网页点云缓存：本次 `/api/v1/map/points` 抽样返回 80,000 点，包含地面附近和高处回波；
  不能从点数、颜色或截图推导建图完整率与定位精度。
- mapd 局部投影：本帧 51×51 格、20 cm 分辨率，307 格支撑候选、233 格高度带占据、
  2061 格未确认。约 10 m 的滚动窗口不代表整趟建图范围；绿色不是运动许可。

实测图：[当前点云与投影](../../../../build/go2-surface-projection/mapping-current-views.png)。
左侧点云时间 22:22:10，右侧投影时间 22:18:02，不能按同帧逐点配准比较。
输入与绘图脚本在 `build/go2-surface-projection/comparison-*` 和
`plot_mapping_comparison.py`。这次没有保存、切换地图或发送运动目标。

| 参考 | 可借鉴内容 | LingTu 当前差距与边界 |
| --- | --- | --- |
| [FAST-LIO](https://github.com/hku-mars/FAST_LIO) | 几何点云累计、保存与高度/强度着色 | 已有几何点云链路；应清楚区分完整保存图、显示缓存和当前扫描。彩色展示不等于 RGB 重建 |
| [ANYbotics elevation_mapping](https://github.com/ANYbotics/elevation_mapping) | 局部地面高度与不确定性融合、可见性清理 | 当前 surface_projection 是低位表面及连通性启发式，没有同等的高度方差融合；该参考仓库已停止主动维护，借鉴算法而非直接引入整套依赖 |
| [CMU Go2 自主栈](https://github.com/jizhang-cmu/autonomy_stack_go2) | 地形、障碍、候选路径和规划结果分层展示 | 需要让用户明确区分地面候选、实时障碍与导航通行条件；其 L1/Point-LIO 配置与本项目传感器链路不同，不能直接照搬阈值 |

下一步优先补全整次建图的累计视图和保存前后核对，再核查近地面观测覆盖、
地形分类与可信度表达。墙面厚度、重复经过的重影、闭环误差需要记录和现场参照，
不能依据不同场景的公开演示截图宣称精度相当。当前图像也不能单独证明缺点来自
雷达盲区、滤波、标定或显示抽样中的哪一项。

最新审查：[2026-09-15 新增 10 项代码缺陷](review-20260915-ten-bugs.md)。
包含目标高度、坐标重置、遥测及点云/相机恢复；本轮仅审查与本地复现，尚未修复或部署。

### 2026-09-15 地面相对高度投影（已安装，静止数据验收）

语言和职责：

| 环节 | 实现 | 本次职责 |
| --- | --- | --- |
| mapd | C++17 | 地面参考、支撑连通、障碍高度分类；生成 surface_projection |
| native client | C++ / DDS / C ABI | 接收一致快照，将原生数据交给 Host |
| HostBus / Gateway | Python | 数据校验、序列化、HTTP/SSE 发布；不重新做地面分类 |
| Web | TypeScript / React / Three.js | 解码网格、显示颜色、交互查询 |

算法虽在 `.hpp` 中定义，仍由 C++ 编译器编译进 mapd；不是 Python 点云算法。
当前图层只是显示分类，现场地面缺点与动态导航验收仍是独立待办，不能说所有问题已解决。

#### 同日 C++ 遍历优化（22:10 已安装，NX 静止验收通过）

优化 `ProjectSupportSurface` 的占据查询：跳过没有地面参考的 XY 列，
只读取相对地面 12–80 cm 的高度带；一个显示格已确认占据后不再遍历该格其余列。
保留原始高度边界判断，分类阈值、输入地图和导航安全逻辑不变。没有新增依赖。

本地 Windows / GCC 15.2 / `-O3` 验证：

| 合成场景（200×200×100） | 原实现中位耗时 | 新实现中位耗时 |
| --- | --- | --- |
| 连续地面与高处回波 | 21.256 ms | 9.209 ms |
| 部分区域缺少地面参考 | 17.937 ms | 3.121 ms |
| 加入分散占据回波 | 24.234 ms | 5.359 ms |

三个场景各 12 次，共 36 次新旧网格逐格相同；各场景先两次预热，再统计十次。
原生投影回归通过，新增障碍高度带两端内外边界测试。
这只是此函数的本地合成基准，不代表 NX 速度或端到端控制延迟改善了同样比例。
证据在 `build/go2-surface-projection/performance/`，包含原实现快照、比较程序及测量结果。

首次上传时，本机切换到 172.20.10.x 网络，127.0.0.1:12218 转发退出，笔记本
192.168.66.95 也不可达。用户恢复网络后，本机回到 192.168.66.62，已重新通过笔记本
恢复 NX SSH（12218）与网页（15052）转发。这是远程维护通路；笔记本直连 NX 的建图操作
仍使用 `http://192.168.123.18:5050/`，不依赖开发电脑的转发。

NX 上原生 projection / mapd_engine 两组回归通过，36 次新旧分类结果逐格一致。
同样的 400 万格合成场景在 NX 上测得：

| 场景 | 原实现中位耗时 | 优化后中位耗时 |
| --- | --- | --- |
| 连续地面与高处回波 | 12.822 ms | 4.531 ms |
| 部分区域缺少地面参考 | 12.121 ms | 1.513 ms |
| 加入分散占据回波 | 13.793 ms | 2.312 ms |

这是函数合成基准，不是完整导航的控制延迟。记录：
`build/go2-surface-projection/performance/nx-benchmark.txt`。

2026-09-15 22:10 安装到当前 `.31` 发布目录，仅重启 lt-maps；SLAM PID 和 Product
current record 保持一致，lt-maps / lt-host / lt-slam 均 active。本轮未发送运动指令。
安装后 8 帧 generation 306–339 的 surface_projection 为 live，state 返回 mapping / map、
localizer_ready 和 pose_fresh 均 true。场景仍包含真实未知区域，不能据此宣称运动避障验收完成。
证据：`build/go2-surface-projection/optimized-live.sse`、`optimized-live-stats.json`。
回退二进制：`/home/unitree/lingtu-surface-update/backups/mapd-20260915-221051`。

另备有 `build/go2-surface-projection/go2-surface-optimization-direct-nx.zip`，可在随行
Windows 笔记本直连 NX 后使用，无密码、无联网下载。原生构建/验证/安装已在 NX 执行；
PowerShell 包装脚本通过语法检查，但尚未在笔记本上一键完整执行，日常操作不必重复安装。

#### 已安装分类修复的实现与证据

修正：建图“局部投影”现在读取独立 `maps.surface_projection`，不再把
`maps.occupancy` 的宽高度带直接解释为障碍。新增算法位于
`src/maps/include/lingtu/maps/layers/surface_projection.hpp`，由原生 mapd 执行。

- 在地图坐标上固定对齐 20 cm 显示格，避免 5 cm 滚动窗口移动时粗格边界整体平移。
- 以已观测低表面的下四分位高度形成候选，默认 5 cm 输入时每格至少 3 个观测列。
  在机身周围 2 m 搜索参考表面（机身下方 0.15–1.0 m），以 8 cm 邻格高差连接。
  这是几何支撑候选，不是经过可通行性验证的地面标签。
- 只将参考表面上方 12–80 cm 的当前三维占据回波标红。连通候选蓝绿；
  无地面依据、断开的高处表面、部分边界格保持灰色。邻格参考只用于标红，不能填绿空洞。
- 完整三维点云、原始占据、三维碰撞与 ESDF 不因显示分类而被删点或放宽；
  该图层不参与运动授权，也不取代 traversability。
- `MapScene` 新增 `surface_projection`；map-scene C ABI version 更新为 2。
  mapd、`liblingtu_nav_client.so`、Python ABI/HostBus/Gateway 必须一起更新，网页只接受新语义。
  没有新增 ROS 或第三方依赖。

参考 [CMU terrain analysis](https://github.com/jizhang-cmu/ground_based_autonomy_basic/blob/noetic/src/terrain_analysis/src/terrainAnalysis.cpp)
的相对地面高度/低分位表面思路，以及
[ANYbotics elevation mapping](https://github.com/ANYbotics/elevation_mapping)
对观测表面与地形信息的区分；没有声称移植其完整地形、不确定性或通行算法。

验证：NX 的 surface_projection、mapd_engine、mapd_dds 三组 C++ 测试，以及 native client
`--case map_scene` 通过；Python 59 项、网页投影 12 项、触及网页文件 ESLint 和
TypeScript/Vite 构建通过。Vite 仍有既有大 chunk 提示。
覆盖平地不染红、真实高处障碍、天花板、缺地面、凸起表面、厘米级高度噪声、
5 cm 窗口滚动对齐、近场 1.3 m 无地面，以及新图层的 DDS/ABI/Host/Gateway 传输。

实机安装在 `.31` 发布目录内打补丁，只重启 mapd/Host；SLAM PID 3736 与原
`product-4b18eea72695480d9df0fdd187b0fdb6` 会话保留。本轮未发送运动指令，未改写保存地图。
15052 首页已返回 `app-BRbB38ZX.js`，新图层实时更新。
浏览器检查接口不可达，尚未完成页面截图和运动中的视觉验收。

现场补充发现：首次 0.8 m 参考范围内没有任何地面候选，导致全灰；已据实测扩大为 2 m，
并补近场缺地面的回归。最终连续 14 帧：51×51、0.20 m，蓝绿 31–36 格、红 84–91 格、
灰 2477–2485 格；相邻帧改变 3–18 格。新旧分辨率和语义不同，不能用格数差宣称误检率下降。
**灰色和不连续仍会存在：当前近处地面没有实际观测依据。** 后续需在受监督移动补扫时核对
地面覆盖与动态障碍；不能通过填平灰格来声称可以导航。

现场证据：`build/go2-surface-projection/live-final.sse`、`final-stats.json`、
`projection-comparison.png`（同一帧的原始投影与新分类对照）。
NX 源码与备份：`/home/unitree/lingtu-source-20260910/build/surface-projection/`；
`backup/` 保存安装前 mapd、native client、Host 文件和网页。

### 2026-09-15 局部投影空地着色与跳变核查（修复前）

- 实机当前为 `map` Product、mapping；SSH 与 15052 HTTP/SSE 可读。
- 网页“局部投影”取 `maps.occupancy`，不是导航可通行图。
  `LiveMapEngine::ProjectOccupancy` 把传感器相对 Z [-1, +2] m 内各高度
  按 XY 合并：任一 occupied -> 100，否则任一 free -> 0，否则 unknown。
  没有地面分类；地面、桌面等表面都会参与投影。0 也仅说明列内有部分空间被射线经过。
- 连续六帧 6373–6378：200×200、0.05 m、网页降采样 factor=1，
  原点均为 (-7.95,-4.05,0)，相邻帧改变 85/131/150/94/83 格。
  这段跳变来自源投影状态变化，不是本段窗口滚动或网页降采样。
- 同帧最低回波位于机身下方 0.15–0.50 m 的列约 6199–6228 个，
  其中 3984–4016 个投影为 occupied。此项支持低处表面与占据着色混合；
  最低回波不是地面分类标签，同列也可能有高处障碍，不能把这些列全部认定为误检。
- 完整 3D 占据中保留地面是正常的；把宽高度带压平后当障碍图阅读会误导。
  导航另用 `maps.local_collision` 三维碰撞和 traversability，
  不能从投影着色直接推出实际碰撞查询已错误拒绝地面。
- 本轮只读采集，未修改阈值、过滤地面、重启服务或发送运动。
  后续展示应区分已观测地面支撑、地面上方障碍与未确认区域；
  不通过抹平颜色或全局 Z 截断伪造可通行性。
  证据：`build/go2-audit-20260915-ten/projection-elevation.sse`。

`model.yaml` describes the Go2 and its MID-360 configuration at
`sensors/mid360_fastlio2.yaml`. Robot selection does not choose `real` or
`sim`; the adjacent `robot.yaml` supplies the physical configuration when the
selected Env is `real`.

`robot.yaml` intentionally does not contain Gateway ports, Product policy,
local-planner tuning, map settings, or lifecycle controls. Those values belong
to the Host/deployment or `config/runtime_graph/`. Its small
`perception.default_classes` entry is the only current per-robot Host override;
all other perception values use the typed runtime defaults.

| Area | Owner |
| --- | --- |
| Motion | `src/drivers/real/motion/robots/unitree/go2/` |
| Physical configuration | `config/robots/unitree/go2/robot.yaml` |
| MID-360 / Fast-LIO2 | `config/robots/unitree/go2/sensors/mid360_fastlio2.yaml` |
| Deployment | `scripts/deploy/` |
| Products | `config/runtime_graph/products/` |

`unitree/go2 + real` has runnable configuration. The simulation environment
does not yet bind a Go2 session and robot assets, so `unitree/go2 + sim`
currently fails for that missing input rather than because the model forbids
`sim`.

## 2026-09-15 随行 Windows 直连与大范围建图准备

### `.31` 已安装：笔记本建图遥控与机载录制

网页紧凑布局补丁（同日，已安装）：移除重复“现场”标题和独立工作区标签，
状态提示移至地图工具栏下的 36 px 固定高度栏，保留恢复控制入口，避免提示变化推动地图。
只读/仿真标识并入工具栏。8 项布局/交互契约、ESLint、TypeScript/Vite 通过；
NX 已返回新版静态首页，无服务重启。浏览器检查接口不可达，本次未完成更新后截图验收。

后续历史点残留修正（同日，已安装）：仅关闭整列清除/衰减仍不完整。
用户报告机器人被累计点包围，当前 body 扫描在诊断盒内无点，而历史显示层存在点。
新增批量查询 `RollingOccupancyGrid::ObservedFreeVoxels`，体素层只清除整个体素
都由射线观测为空的历史表面；未知、窗口外、最新未被后续射线穿过的命中继续保留。
查询只读取独立占据层，不改变其碰撞状态、射线或导航参数；在新表面写入前清理，
避免旧空闲历史吞掉本帧命中。NX 三组 maps 原生测试通过，覆盖清除旧点、保留其他高度、
旧空闲空间的新命中，以及原占据回归。网页近处点上限为 5 个渲染像素，降低地图点放大倍率。
网页点大小四项回归、ESLint、TypeScript/Vite 通过。

安装仅重启 `lt-maps`，SLAM PID 与 Product current record 保持不变。
安装后连续 12 帧，按网页机身 yaw 坐标诊断的 X ±0.35 / Y ±0.20 / Z ±0.12 m
盒内为 0 点，机身相对 Z [−0.45, −0.15) m 保留 4,038～4,536 点。
此盒仅用于分析，不是新增机器人过滤包络。安装前后机器人位置改变过，实时地图服务也重启过，
这些数据仅证明新运行的静止表现，不能声称在同一原卡点清除了所有旧点。
模拟观测序列覆盖实际清除逻辑；持续运动和动态物体退场仍待现场验收。
旧地图文件未改写，网页显示累计表面也不等于可通行判定。
备份 `mapd-before-ghost`、`dist-before-ghost.tar.gz` 在同一 v31 验证目录。

后续姿态与快照发布修正（同日，本地通过；20:00 后已安装，见下方实测）：

- Gateway `services/odometry.py` 原先仅发布 yaw，丢掉里程计的 roll/pitch。
  现转发同一条里程计的归一化四元数；网页 SSE 与状态快照均保留它，
  map<-odom 只组合一次完整姿态，模型在统一 Three.js 坐标基底中旋转，
  相邻实测姿态使用最短弧四元数插值。未提供完整姿态的来源仍使用原 yaw。
- `CloudViewerService.should_publish_view_cloud` 原先在 mapping/exploring
  用点数增长决定发布，完整快照点数不变或减少时也要等默认 1 s 强制刷新。
  现完整快照只受既有最小发布间隔约束（默认上限 4 Hz），不再要求点数增长；
  增量累计模式保留原策略，空快照仍立即清屏。
- 新增回归先在旧代码复现：完整姿态缺失、等量替换/删点后旧快照仍在显示。
  修复后 Gateway 32 项、Web 22 项通过；受影响 Ruff/ESLint/TypeScript 和
  Vite 构建通过，Three.js 大分块提示仍存在。
- 源码核对 `toDdsMapObservation` 将 registered body scan 与
  `state_estimation_at_scan` 放入同一观测；mapd 从该观测转换到 map，
  未发现这里使用另一条最新位姿替换扫描时刻位姿。此项为代码证据，
  不是外参、SLAM 配准或运动期间时间同步的完整实机验收。
- NX SSH 经原隧道与新建笔记本隧道均超时，Windows 笔记本 SSH 可达。
  以上两项本轮尚未安装，不能计入已部署 `.31`；先前历史点清理已安装。
  构建包 `build/go2-release-20260915-v31/attitude-update.tar.gz`，
  恢复 NX 后备份并安装 Gateway 两文件与 Web，重启 Host 后核对完整姿态
  和快照发布；继续现场运动观测墙面稳定性、旧点清除及地面保留。

20:00 后开机复验（同日，已安装）：

- NX 重启后 `lt-host/lt-maps/lt-slam` 均为 inactive；先安装完整姿态/完整快照
  发布补丁，再由 ProductControl 启动 `map --variant camera`，事务成功，
  session 为 `product-4b18eea72695480d9df0fdd187b0fdb6`。助手未发送非零运动指令。
- 实机 `/api/v1/state` 里程计已包含四元数，连续 15 份均存在；HTTP 首页
  返回 `app-CrnLKEty.js`。本次使用源码/接口与几何回归核验，未取得浏览器截图。
- 另发现 HTTP `/api/v1/map/points` 把每次请求时间用作点云 `stamp_s`，
  缓存未变化也显得新鲜。已保存缓存对应的源时间，更新发生在 WS 限频之前；
  HTTP 的 `ts` 仍表示响应生成时间，`stamp_s` 表示数据时间，重读不改源时间。
  新增回归先红后绿，含源不更新、WS 限频时缓存更新、清空后的重复读取。
  点云/健康/采样相关 49 项通过，Ruff 通过；已安装并仅重启 Host，
  SLAM PID 与 Product current record 保持不变。备份 `cloud-viewer-before-time.py`。
- 15 次只读采样的 map source-to-Gateway-emit 为 0.869～1.263 s，registered
  scan 为 0.119～0.262 s。此值不是端到端运动延迟，也不是单独的网络延迟。
  现场 Host 累计点云配置上限为 2 Hz；本轮未改高频率，也未把约一秒差值
  宣称已全部消除。完整地图快照生成与转发开销仍需进一步拆分测量。
- 最后 10 帧显示云在完整四元数机身诊断盒 X±0.35/Y±0.20/Z±0.12 m 内均为
  0 点，机身下方 Z[-0.45,-0.15) m 有 5,580～5,703 点。HTTP 源龄约1.219 s，
  已不再冒充当前请求时刻。仅为原地采样，尚未验证持续运动时墙面重影/配准误差。
- mapd ready，无容量拒绝、无效观测或 DDS 发布失败；一次状态中处理队列深度1。
  相机 color 29 Hz/depth 28 Hz，JPEG HTTP 200；不据此推断相机已用于导航避障。
- 累计层另有 `accumulated_column_carving`，它与本轮已修复的网页 voxel 表面
  是不同数据层，不应把表面保留回归推及所有层。保存地图及导航运动验收未在本轮重做。

点云断流与恢复检查（2026-09-15 后续，网页更新）：

- `useBinaryCloud` 原本只在 WS 首次打开时设置无帧超时，首帧解码后清除，
  后续停止发帧却不关闭连接时不会恢复。现每次有效解码都续期 2.5 s 定时器，
  超时地图启用既有 HTTP 备用，scan 无备用则清空旧帧；恢复有效帧停止备用轮询。
- 恢复时原先继承 `prev.connected`，会沿用备用通道留下的 false；现在读取当前
  WS OPEN 状态。建图页保留过期地图参考，但以真实源时间标注“缓存画面”。
- 新测试执行真实 hook 的转译代码，模拟 WS、Worker、时钟，旧代码首帧后断流的
  两项用例先失败；修复后含恢复/续期/卸载及解码协议 16 项通过。
  受影响 ESLint、TypeScript、Vite 通过；Three.js 分块体积提示仍存在。
- 静态更新不重启 SLAM/mapd/Host，不发送非零运动；此回归不是断开真机网络的
  运动验收。累计地图约一秒 source-to-emit 延迟仍在，没有据此宣称已消除。

16:20 后续修正（已安装至 `.31`）：

- 遥控改为场景顶部的“遥控模式”，开启后直接使用 WASD / Q、E；
  不再在右侧展开方向按钮。唯一可调参数是限速，默认 0.50 m/s，并受后端上限约束。
  松键、失焦、切换限速会清除输入；Space 保持，Esc 或“退出遥控”释放控制。
  操作菜单会暂停输入，回到场景须重新按键。刷新后默认不启用遥控。
- 实机 mapd 旧体素层仍使用 `column_carving=true`、`decay_rate=0.01`：
  当前帧命中同一 XY 列的顶部/墙面，会清除列内未重新命中的低处表面；
  单次稀疏命中也会被衰减。网页采用 `maps.voxel_cloud`，因此低处点难以累积。
  已安装本地体素表面保留修正：关闭该层整列清除和时间衰减，按机器人周围
  30 m、相对传感器 Z −3～5 m 窗口保留，容量仍有上限。
  独立实时碰撞占据层与其射线清除逻辑保持原行为；此图层不代表可通行空间。
- NX ARM 构建及 `lingtu_maps_mapd_engine_test`、`lingtu_maps_voxel_layer_test`
  均通过，覆盖地面→同列顶部观测、衰减、容量恢复和碰撞占据一致性。
  仅重启 `lt-maps`，SLAM PID、Product current record 不变。
  网页低高度带 Z [−0.45, −0.15) m 由修正前约 240 点恢复到
  连续十帧 4,963～5,058 点（网页总数上限 60,000）。这是表面保留验证，
  不是整场地地面覆盖率或导航运动验收。
- 独立保存的诊断地图 `go2_ground_check_20260915_160301` 有 15,448 点，
  同高度带保留 1,773 点；更低的 4 点不作地面认定。清理移除了同高度带 210 点，
  因而不能说保存阶段没有任何低处点被剔除，但并未删空地面。
  `map_optimization.json` 记录 `performed=false, code=pgo_timeout`；
  保存成功不代表完成闭环优化，也未激活该图导航。
- 网页 34 项相关回归及 ESLint、TypeScript/Vite 通过。
  实际浏览器已核对模式入口、不遮挡、限速切换和 Esc 退出；未发送非零命令。
  地图点为绿色，当前扫描为蓝色；修复主题切换临时把地图点变白的问题。
  在线图仍是滚动局部表面，整趟完整地图需保存后预览，未知地面不能补画成可通行。

当前已安装文件在 `/opt/lingtu/current`：新版 `bin/mapd`、速度适配器和 `web/dist`。
备份与记录在 NX `/home/unitree/lingtu-validation-20260915-v31`，
源码同步到 `/home/unitree/lingtu-source-20260910`；后续打包须使用这些更新后的源码。

15:23 速度遥测补丁：`src/localization/adapters/status.py` 原先只从 C++ 状态快照
转发 pose，`Odometry.twist` 保持默认零，导致网页“实测前进速度”恒为零。
现将快照已有的 `fastlio_velocity` 旋转到 odometry 的 child/body 坐标轴再转发，
使用同一快照 odom 朝向，不使用 map 对齐后的朝向。这是 Fast-LIO 线速度估计，
不是遥控请求速度；本补丁不添加同步角速度，也不改变 native 导航与驱动链路。
本地 40 项适配器测试及 Ruff 通过，新增前后/侧向/停车、四种朝向与倾斜的
适配器 → Gateway 缓存/SSE 线速度回归。已备份并更新 NX `.31` 的该 Python 模块，
只重启 Host，确认 SLAM PID 和 Product current record 不变；同步更新 NX 源码。
安装后 state API 的 vx 已非固定零，本地网页实际显示过 0.01 m/s。
这是静止时的小幅估计波动，尚待操作者短距离运动核对变化幅度和停车回落。
重启前正常完成旧录制，15:23:10 续开一小时录制
`go2_laptop_mapping_20260915_20260915T072310Z_69973_d61dc0c0`。

NX `/opt/lingtu/current` 已切换到 `v2.3.0-go2.20260915.31`，`.30` 和旧地图保留。
通过 ProductControl 停止旧 nav，启动 `map/camera`，会话为
`product-ddac12b76d754035b3631e9c57e24c85`。这是新建图，不加载旧导航地图。
本版补齐网页 `map` 遥控入口；初装沿用 `.30` native，后续 mapd 更新见上面的 16:20 记录。

笔记本打开 `http://192.168.123.18:5050/`，刷新后点击场景顶部“遥控模式”，按键开始。
速度上限为 0.50 m/s；W/S 前后、A/D 侧移、Q/E 转向，松键停止，Space 保持。
建图是直接遥控，需要操作者绕开障碍，不是规划绕障。结束先停稳，再保存地图并确认成功。

安装后发现机载录制启动失败：sudo ProductControl 创建的锁是 `root:root 0644`，
Host 的 `lingtu` 用户无法写入。修复部署源 `scripts/deploy/thunder/lingtu-runtime.conf`，
在 NX `/etc/tmpfiles.d/lingtu.conf` 持久化锁文件 `lingtu:lingtu 0600` 规则；
不删除锁、不跳过锁、不重启当前建图。实测修复保留同一 inode，root 持锁时 Host 被阻挡，
释放后 Host 可以获取。该部署配置补丁独立于 `.31` 发布包，后续安装服务使用修正后的部署源。

14:18（北京时间）通过网页同一 API 启动 3600 秒的 sensors 原始录制，
会话 `go2_laptop_mapping_20260915_20260915T061833Z_42413_18c13be0`；
约 11 秒已写入 53.8 MB，状态 healthy/recording。录制不含相机视频，
不等于已保存可导航地图。超出一小时须另开录制，结束仍须保存 SLAM 地图。

验证：网页遥控 13 项本地测试、受影响文件 ESLint、TypeScript/Vite 构建通过；
已安装网页显示建图遥控、0.50 m/s、录制中，彩色/深度均报告 30 帧/秒。
零输入连接及保持 ACK 已确认，验证客户端已断开，没有发送非零运动命令。
这不代替笔记本现场松键/失焦/断线停车、短环路保存及离开 Wi-Fi 的验收。
在线点云仍为局部表面预览，整趟累计建图视图尚待完善。

15:14 复验：随行笔记本直接请求 NX，返回同一 map 会话和新鲜定位；开发电脑通过
笔记本的临时 SSH 转发打开 `http://127.0.0.1:15052/?observe=1`，作为只读监看端。
SLAM 约 10 Hz，相机约 30 Hz，开启雷达叠加后已配准扫描年龄约 0.1 秒。
无人订阅时扫描编码按需暂停，不能把旧编码帧的年龄当作 SLAM 停止。
本地监看依赖开发电脑到笔记本的网络，离开原 Wi-Fi 后可能断开；笔记本网线直连
NX 的建图与操作不依赖这条监看转发。
旧原始录制正常完成，新一小时录制于 15:14:55 开始，会话为
`go2_laptop_mapping_20260915_20260915T071455Z_66146_96fb35c0`。
SLAM 会话未重启、内存地图未清空，没有发送运动命令；短环路及保存验收尚待现场进行。

后续已远程修正指定任务中的笔记本 `DESKTOP-17EDVN1` / Dz：WinHTTP 的
`thunder.lan:7890` 开发电脑依赖移除，CLI 改用已验证独立运行的本机 Clash `7898`，
NX 加入 NO_PROXY；有线配置为 `.123.100/24`，无默认网关。
从笔记本直接请求 NX navigation/status 成功，路由为以太网 on-link；本机代理 HTTPS 204。
浏览器系统代理原本已使用本机 `7898` 且绕过 `192.168.*`，不是全部程序均依赖旧隧道。
这次同时修复了旧 CLI/WinHTTP 代理残留和以太网仅有 APIPA 地址的问题。
没有发送机器人命令，也没有主动断开 Wi-Fi；离开原 Wi-Fi、重启后的复验仍待现场完成。

笔记本连接说明和两个网页快捷方式见 [laptop-direct/README.md](laptop-direct/README.md)。
它们不包含凭据、不安装程序、不切换 Product，也不发送运动命令。

现场条件：开发电脑不能移动；另一台 Windows 笔记本未安装 LingTu，准备通过网线
直接连接机器狗/NX。公网或原 Wi-Fi 可能中断。NX 承担计算和机载数据保存，
随行笔记本仅作浏览器显示和操作端，不复制开发电脑的 SSH 私钥、不安装 ROS 或 LingTu。

### 连接另一台 Windows

1. 网线接机器人对外网络口，保持 NX 到 Go2 主控、MID-360 的内部网络连接。
   不要拔掉内部雷达或主控线来腾出网口。
2. Windows 按 Win+R，输入 `ncpa.cpl`，选择实际接线的以太网适配器，
   属性 → Internet 协议版本 4。设置同网段空闲地址，例如确认未占用的
   `192.168.123.100`，子网掩码 `255.255.255.0`；网关和 DNS 留空。
   不使用 NX `.18`、雷达 `.20`、Sunrise `.99` 或主控 `.161`。
3. 浏览器打开 `http://192.168.123.18:5050/`；只查看时加 `?observe=1`。
   `127.0.0.1:15050` 仅是原开发电脑的 SSH 转发，不能在新电脑上照抄。
4. 若打不开，PowerShell 执行 `Test-NetConnection 192.168.123.18 -Port 5050`。
   TCP 不通先检查地址和接线；TCP 通但网页异常，再读取 NX Host 状态。
   完成机器人直连用途后，按原网络要求把该适配器恢复为自动获取地址。

本轮已确认 NX `eth0=192.168.123.18/24`、Gateway 监听 `0.0.0.0:5050`，
从 Sunrise 的机器人内网直接访问主页返回 HTTP 200；后续也已在随行笔记本验证直连 API。
NX 磁盘可用约 380 GB；这不是建图时长或覆盖面积保证，仍须检查录制增长和 SLAM 内存。

### 断网与数据保存的区别

- 公网/Wi-Fi 断开但网线正常：局域网浏览器仍可连接 NX，不需要互联网服务。
- 笔记本到 NX 的网线断开：网页会断流，网络遥控须停车；机载 systemd 建图进程
  不以浏览器存活为生命周期，但实际拔线后的持续采集、重新连接仍待现场验收。
- NX/机器人断电或 SLAM 重启：不能当作普通网页掉线。内存地图不保证保留。
  出发前开启已有机载原始数据录制、检查文件持续增长；结束必须明确保存地图并检查产物。
  原始录制可用于重放，不等于已经自动生成可导航地图。

### 当前阻挡完整随行流程的缺口

1. `.31` 已补齐 `TeleopPanel.tsx` 和 `SceneView.tsx` 的 `map` 遥控入口，
   沿用现有 bootstrap 与 native operator-motion 接口。还需在笔记本现场验收松键、
   失焦、断线停车和重连不恢复旧输入；不能以本地测试代替真实运动验收。
   原配遥控器接管尚未完成验收，不应同时与 LingTu 争夺运动控制。
2. 在线“空间点云”主要是局部表面预览，不能表达整趟路线的完整累计覆盖。
   需要独立于滚动局部碰撞图的机载累计建图产物，以及分块、有界传输的全程预览；
   不在浏览器或 Gateway 另建权威地图，不通过提高定位工作图密度拖慢 SLAM。
3. 建图工作区应同时呈现累计地图、当前扫描、真实行走轨迹、相机、采集/录制/保存状态，
   提供“跟随/全图”、定位质量及闭环结果。局部观测面积不能冒充全图完成百分比，
   地图空白不能被解释成已知可通行；未标定相机只能预览，不能宣称 RGB 融合地图。

先在原地完成笔记本直连与上述入口验收，再用短环路验证保存和重连，随后扩大范围。
建图使用正式 `map --variant camera`，不加载旧导航地图；该切换会冷重启当前 Product，
不应在用户仍测试导航时隐式执行。普通 map 直接遥控不等于 SCAN 辅助避障。

## 2026-09-15 `.30` 安装与导航静止检查

NX 已安装 `v2.3.0-go2.20260915.30`，`/opt/lingtu/current` 已指向该版本。
本次仅发布下节的碰撞保持和自主输出新鲜度修复及相关测试；不代表其他本地修改全部部署。
NX ARM 上实际编译、链接 mapd/navd，地图 2 项、导航 7 项 CTest 通过，
包括四向新障碍保持与真实自由射线解除、SCAN 制动以及自主输入失效检查。

操作者确认在旧场地建图起点附近。通过 ProductControl 启动 `real + nav/camera`，
加载 `go2_expanded_20260914_0350`；初始局部匹配失败后，自动全局定位恢复，
随后地图跟踪通过，无人工放宽配准阈值。会话为
`product-601db795f59f4522b4a30e19b7537dc6`。

13:10–13:12（北京时间）静止检查：7 个服务 active，Gateway 定位新鲜且可接受目标，
无活动任务、无遥控客户端；native InputGate ready，定位 TRACKING，
collision live/complete，分辨率 0.05 m、数据年龄约 71 ms。
native 配置控制频率 100 Hz，空闲循环均值约 10.09 ms，健康状态 healthy；
最终速度为零。这些是无运动证据，不是带规划负载或真实障碍制动验收。
相机彩色/深度约 29.9 fps；相机外参仍未验证，深度未融合到避障。

电脑原 15050 转发进程已退出，重新建立 SSH 转发到 NX 5050 后，
`http://127.0.0.1:15050/api/v1/state` 恢复可读。观察入口为
`http://127.0.0.1:15050/?observe=1`。此次未发送运动目标。
下一步由操作者监督，以 0.20 m/s、软质测试物验收截停和移开恢复，再验收绕行。
禁止用人作障碍；尚不能宣称动态避障或实际制动距离已通过。

## 2026-09-15 自主导航碰撞反馈：实时障碍与最终输出修复

用户确认碰撞发生在网页发送目标后的自主导航，不是建图直接遥控。此反馈说明实机
动态避障尚未通过验收。前一轮本地修复时网页 `/api/v1/state` 返回 503，NX SSH 超时，无法读取
事故时刻的扫描、碰撞格和最终指令；以下是本地已确认并修复的缺陷，不是事故归因，
也不是已经验证不会撞人的声明。后续安装与静止检查见上节；没有发送运动目标。

### 已修复的两个缺口

1. **新障碍可能下一帧就消失。** `rolling_occupancy.cpp` 中，一个此前已经多次被
   射线确认空闲的格子，单次新命中不足以克服历史空闲概率，原来只靠 `current_hit`
   进入碰撞位图。下一帧即使看向别处、没有射线穿过该格，也会清掉此保护。
   现在改为 `unresolved_hit`：在当前 field 不启用时间衰减的配置下，未再观测或被
   遮挡时保持阻挡；后续射线实际穿过，且同帧没有端点命中时，才撤除端点保护。
   显式衰减、窗口滚动移出和重置仍保留原有语义。历史概率、膨胀半径和显示表面均不因此改变。
   因而薄物体或噪声的单次命中也会保留到有清除证据；不能用未观测清空来减少卡顿。
2. **自主导航发布前遗漏了输入新鲜度复查。** 遥控已有发布前复查，自主导航以前
   直接发布规划结果。现对自主非零命令在 DDS 发布前重新评估 InputGate；若点云、
   碰撞图、定位或驱动输入已过期，调用现有 `holdEndpointMotion` 发零并暂停执行，
   保留导航目标与路线。停车发布失败仍按原机制锁住输出，不能继续发非零。该检查
   在规划返回后执行，不等于能在耗时的同步计算内部抢占停车。

代码在 `src/maps/cpp/layers/rolling_occupancy.cpp` 和
`src/nav/cpp/endpoint/nav/runtime/loop.cpp`，没有增加地图所有者或旁路驱动命令。
历史概率层暂时显示 Free 时，未解除的命中仍可能令实时碰撞层 Blocked；实际运动
应以 `/maps/local_collision` 为依据，不能以概率层或显示点云推断一定可以通行。

### 已验证与尚未验证

- 本地地图回归先在旧代码复现失败，修复后 rolling occupancy、mapd engine 两个测试通过。
- 新增 `test_live_obstacle_braking`：实际 RollingOccupancy 的膨胀位图接 SCAN 制动
  几何，验证前后左右四向的“原本空闲 → 新障碍单次命中 → 后续未再观测仍阻挡 →
  实际射线确认空闲后释放”。这是 native 组件联动，不包含 DDS 调度和真实电机制动。
- SCAN 新障碍截停已有样条、清障后恢复，以及实际位置/旋转/转弯扫掠等相关回归通过。
- 自主控制器 C++ 回归及两条 Python 输出接线合同通过；覆盖输入在规划期间失效、
  零命令仍可发布、停车失败不能放行，并保留当前任务。
- 实际 `navd` Windows Release 完整编译及链接通过，包含本轮修改的 `loop.cpp`。
  依赖仍有原有 SCAN/LBFGS 与 MSVC 告警；这不是 NX ARM 构建或实机验收。
- 当时尚未安装到 NX；后续 ARM 构建、安装和无运动证据见上节。监督运动未验收，事故时序也尚未取到。

### 当前能力边界与下一步

实时检测链路已经存在：`/slam/map_observation → mapd → /maps/local_collision →
SCAN`；最终指令还按实际机身位置、实测速度和配置制动参数检查扫掠空间。因此无需
等到全局重新规划完成才停车，界面显示旧轨迹也不能授权继续运动。

但“对当前障碍反应”与“预测行人将进入哪里”是两件事：当前 MotionLayer 会生成
预测点，自主 SCAN 有权威 collision bitmap 时会跳过 legacy 点云融合，SCAN Task
也不消费该预测数组。**默认 SCAN 的行人运动预测尚未接通**，不能把已有预测生产代码
当作此能力已完成。后续预测应作为短期、带时间的避让约束，不应写死进保存地图。

Go2 的 `sensors/mid360_fastlio2.yaml` 当前设 `lidar_filter_num: 2`、
`lidar_min_range: 0.5`，这些过滤在导航使用的 SLAM 观测上游发生。0.5 米是相对于
雷达的端点过滤范围，不是机器人表面到障碍的净距，也不是实测制动距离。D435i 预览
和内置雷达尚未补进该碰撞链。应先用近场原始扫描和机身模型区分自身回波、外部物体
与真实盲区，再决定独立保留近场避障点或融合补盲传感器，不能盲目把最小距离改为零。

下一轮先恢复连接、安装并做不运动的数据验收，再由操作者监督，用软质固定障碍和
可远程移入的测试物验证截停/移开恢复，禁止拿人作障碍。记录同一时刻的扫描源时间、
collision 代数/年龄、规划输出、最终命令和实测速度；先测停止距离与响应延迟，再
确定可用速度。规划加速度或指令变化率不是实测制动能力，不能据此宣称高速避障通过。

参考 [Nav2 Collision Monitor](https://docs.nav2.org/rolling/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/)
将碰撞检查放在速度处理链末端的做法；本项目继续由 native nav 仲裁、driver 执行，
不为此引入 ROS 或第二个速度发布者。

## 2026-09-15 项目适配判断与导航故障应对

本节是当前源码与官方实现的适配审查，不是新增机载验收。选型目标是操作者能明确
区分建图、辅助遥控和自主导航，机器人能可靠到点、停车并从可恢复故障中继续任务。
点云更密、更干净只是一部分条件，不能替代定位、通行判断、控制执行和恢复验收。

### 哪些参考能力确实需要

| 能力 | 判断 | 采用范围 |
| --- | --- | --- |
| 地面与薄障碍保护、可靠的自由射线清除 | 当前必需 | 完善现有 native mapd / prune；参考 ANYbotics 和 Nav2 的证据处理，不新增竞争地图所有者 |
| 进展检测、按失败原因选择等待/重规划/脱困 | 当前必需 | 完善现有 native nav，参考 Nav2 恢复编排；已有机制优先验收和修正 |
| Patchwork++ 分区域地面识别 | 候选，需离线比较 | 先用于地面标注及保存清理；不能把地面标签直接当作可通行，也不照搬车载参数 |
| ERASOR / ERASOR2 / Removert 保存地图清理 | 有价值，排在基础一致性之后 | 用录制扫描与对应位姿离线对照；不能替代实时避障，不能把当前行人从碰撞地图中删掉 |
| STVL / Dynablox 整套地图与跟踪框架 | 暂不整体接入 | 参考寿命管理、自由空间历史和动态判别；当前没有证据证明重建整套框架比修复已有链路更合适 |
| 相机/第二雷达补盲区 | 有条件需要 | 先完成外参、时间对齐与各传感器射线原点处理；出图不等于已融合进导航或保存地图 |

官方依据：[Nav2 恢复树](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)
把路径跟踪、目标更新、恢复适用性、等待/旋转/后退及重试上限组合起来；
[进展检查器](https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/plugins/simple_progress_checker.cpp)
用实际位姿变化和时间判定是否有进展。这些控制思路适合参考，但默认后退行为只支持
X 方向，不能直接覆盖 Go2 全向侧移恢复；清空 costmap 也不能在本项目中被解释为
把真实障碍和未知空间变成安全区域。

### 当前模式能力必须讲清楚

- `map`：录制建图与操作者移动。正式 map 的直接遥控不等于 SCAN 辅助避障。
- `teleop_avoid`：不要求保存地图，以操作者输入为引导做局部避障；当前 Product
  `recovery.max_attempts=0`，禁用通用自主恢复预算。但 SCAN 仍有沿输入方向的边界
  离开候选及局部重新生成轨迹，不能说完全没有恢复。
- `nav`：在保存地图中定位并执行目标；配置启用有界平移/旋转恢复。具有代码机制
  不等于所有原卡点已实机通过。

nav 还已有持续局部堵塞升级为全局改道、搜索忙时替换待处理目标，以及定位/点云/
碰撞图失效后的输入门控。`ActivePathBlockagePolicy` 等待持续阻挡和新观测，局部
已有可执行绕行时不抢占；`InputGate` 要求必需输入实际更新后才推进恢复。项目也有
预测障碍输入，但默认 SCAN 的 collision bitmap 路径尚未消费预测点，不能据此宣称
SCAN 已接通行人运动预测。本轮建议是验收和修正这些已有能力，
不是另建一套全局重规划或控制权状态机。

因此持续 W、前方阻挡且只有后方能退时，当前辅助模式不会擅自把输入改成自主后退。
若要辅助模式也自动后退，需要明确允许的动作范围及持续操作者输入约束；松键停车
不能被恢复动作覆盖。自主导航则应在正确定位与有效碰撞数据下选择可行的恢复方向。
模式声明见 `config/runtime_graph/products/{map,teleop_avoid,nav}.yaml`，具体执行分支
在 native nav 的 `Executor::tickIntent` 与自主执行路径。

### 导航常见问题及我们需要的处理

以下是实施与验收要求，不能把表格全部读成已完成能力。

| 表现 / 场景 | 需要区分的原因 | 处理与通过条件 |
| --- | --- | --- |
| 明明有路却判受阻 | 真障碍、噪点、自身回波、旧残影、支撑不足 | 显示真正阻挡的格子及来源；回放 hit/miss，验证薄障碍保留、残影可清。不得单靠缩小包络放行 |
| 行人或物体临时横穿 | 短时阻挡或持续封路 | 先减速/停车；条件恢复后继续原任务，持续封路才选绕路。当前物体仍参与碰撞，静态地图清理不负责实时让行 |
| 有轨迹但机器人不动 | 最终指令被拒绝、失去控制权、底层低速响应、打滑 | 对齐请求速度、最终速度与实测位姿；按实际进展判失败，不能以已发送指令认定执行成功，也不一律增大速度 |
| 窄通道和起点已贴近障碍 | 中心可过但机身扫掠体不通过；原地旋转空间不足 | 分别检查前后左右及旋转扫掠空间，选择有界侧移/后退或换向。无可验证动作时明确终止并提示所需移动方向 |
| 反复重规划、左右摇摆 | 地图噪声、频繁目标更新、候选来回切换、轨迹衔接差 | 保留仍有效的路径；必要时重规划并保持速度衔接。新目标生效后旧结果不得覆盖它；恢复失败不应无限重复同一动作 |
| 重启/搬动后定位错误 | 旧初值、地图匹配失败、位姿跳变 | 明确进入定位等待，核对初值并重定位；有效前不执行旧轨迹。UI 显示定位故障，不伪装成路径不可达 |
| 点云/地图延迟或控制卡顿 | 传感器时序、队列积压、计算超时、网络失联 | 保持最终运动检查与输入超时停车；按端到端数据年龄定位瓶颈，先削减预览/保存等非关键负载，而不是仅提高标称频率 |
| 地面缺失、楼梯或多层空间 | 没有观测、支撑误判、净空不够、机体能力不符 | 未知保留未知；补扫/标定后融合；分别验证地图、路径及 Go2 步态。3D 搜索成功不能证明能上下楼梯 |
| 暂停/切换模式后状态混乱 | 生命周期、任务、控制权与界面不同步 | 保留唯一生命周期和最终指令所有者，明确显示阻止运动的原因与可执行操作；模式切换不自动重放旧运动命令 |

### 应先修的本库一致性问题

本次追加确认两个公开 OctoMap 构建模式仍有语义差异：
`src/maps/cpp/build/pipeline.cpp::BuildNativeOctomapInDirectory` 的 native 路径对所有
占据点做 XY 支撑扩展并在其上方生成 free；
`src/nav/cpp/planning/global/octoplanner/pcd_to_octomap.cpp` 的 external 路径先提取
水平支撑，限制桌面向低地面侧向扩展，并只从支撑生成上方自由层。
`native_octomap` 是公开支持模式，但默认 `build_mode` 是 `external_pcl_converter`。
这是需要统一的本地代码缺口，未据此断定当前现场卡点来自 native 路径。

优先顺序：先统一支撑/净空与地图转换语义、核对实际阻挡来源；再验收自主恢复和
轨迹衔接；最后完善保存地图去残影并评估额外算法。基础验收至少覆盖空地连续行走、
绕纸箱到点、物体移开后继续、窄处平移/旋转、行进中换目标、断输入停车、重启后
同地图定位。每项同时记录现场现象与 native 的地图、指令和实际进展，不能仅看网页轨迹。

## 2026-09-14 建图下部点云缺失：现场数据与局部表面保留

用户在 `.29` 的 `map/camera` 中看到高处黄色点密集、地面附近蓝色点稀疏。
在不重启、不切换 Product、不发送运动命令的情况下，采集约 12 秒、50 组原生
`lidar_scan / registered_cloud / map_cloud` 快照，以及同期 12 组网页表面快照。
采集期间位姿变化各轴小于 5 mm；去畸变 body 点云经扫描位姿变换后，与原生 map 点云
的最大逐点误差约 0.016 mm。原始 LiDAR 与 registered 快照最多差一帧约 101 ms，
因此原始点数对比属于同期统计，不能称为严格逐点过滤归因。

按地图 Z < −0.15 m 统计低处回波（高度带不等于语义地面）：

- 原始雷达每帧平均 1,387 点；SLAM 去畸变输出每帧平均 672 点，低处回波确实存在。
- 同期网页 `maps.voxel_cloud` 低处点为 176–984 个，平均 349 个；Z > 1.8 m 的点平均
  15,204 个。当前总点数约 1.6–1.9 万，未触发 Gateway 的 6 万点上限，也未命中网页
  Z 范围过滤，不能把这次缺失归因于浏览器抽稀。
- 将已采集扫描按 5 cm 体素合并，在机器人周围 5 m、Z = −0.45～−0.15 m 内得到
  3,898 个实测体素；同期网页某帧同范围仅 207 个。前者是诊断合并、后者是滚动快照，
  差值不能直接当作丢点率，也不是保存地图的质量验收。
- 当前原始扫描中，地面高度带的最近回波距机身水平约 0.95 m。脚边以及没有回波的
  方向仍需从其他位置、朝向补扫；局部图保留修复不能证明这些区域已观测。

已确认代码原因：mapd 表面体素继承了 `VoxelLayerConfig.column_carving=true`。
`src/maps/cpp/layers/voxel.cpp` 把任意高度新回波的 XY 格加入已观测列，再清掉这些列中
LiDAR 原点下 0.7 m 至上 1.8 m 的全部旧体素，然后插入当前帧。高处回波不能证明同列
地面已空闲；这项整列替换会删掉本帧未扫到的旧地面，而高度带外的上部点继续累积。
NX 实际进程参数已确认 `--carve-min-z -0.7 --carve-max-z 1.8 --decay-ms 250`。
另有既有保留限制：每 250 ms 命中数乘 0.99，小于 1 时删除，单次命中会很快消失。

本轮修复范围为 mapd 的观测表面体素默认配置，关闭其整列替换；独立体素库既有默认值、
导航 occupancy 射线更新、accumulated 层及保存时清理流程保持原有行为。
表面点还用于 `maps.elevation` 的观测高度投影，因此这不只是浏览器渲染改动；
导航碰撞快照仍来自独立 occupancy，而不是该表面层。

使用生产 `VoxelLayerCore` 回放这 50 帧、按源时间执行 48 次既有衰减，整列替换开→关后，
最终低处体素从 637 增至 1,323，中层从 2,032 增至 4,296，高处仅从 5,982 增至 6,129；
容量拒绝和快照遗漏均为 0。该采样约 4 Hz，不能重建原来 10 Hz 输入的实时缓存，
只证明整列替换对不同高度的保留存在明显偏置。关闭衰减的诊断对照中，低处保留为
2,991→4,740；第一阶段只改整列替换，后续完整保留修复见下文。

保存地图由 SLAM 自己的地图和扫描 patch 生成，不以网页 voxel 快照作为原始地图；
本次没有保存或重启当前建图，不能据这份诊断宣布完整保存地图或导航已验收。

新增生产 `LiveMapEngine` 两帧回归在旧配置下先失败；修复后
`lingtu_maps_mapd_engine_test` 通过（1/1）。测试同时对比碰撞 bitmap、二维 occupancy、
accumulated 输出不变，原显式列清理高度带测试仍保持启用，既有容量与 epoch 测试通过。
本轮修改文件的 diff 空白检查通过。以上为本地 C++ 证据，不是机载安装或在线效果验收。

现场数据、可复算脚本和对比图保存在 `build/go2-ground-observation-20260914/`。
本节为本地修复记录，当前实机 `.29` 尚未安装该表面保留改动。

后续完整保留修复已在本地实现：mapd 表面层取消命中计数衰减，并在接收新扫描前
回收空间窗口外体素。窗口沿用现有发布 ROI：水平半径 30 m，当前 body 扫描位姿
高度下 3 m、上 5 m；缓存仍受 500,000 体素上限约束。epoch 改变仍清空旧数据。
这解决了单次稀疏回波快速消失及只裁发布、不裁缓存的问题。历史动态物体表面可能
保留到离开窗口，因此该层是局部观测历史，不是已清理静态地图或实时碰撞裁决。
accumulated 层仍使用既有更新策略，不能把这项修改说成所有地图层均已改为窗口缓存。

### 同链路追加审查：容量状态与多楼层显示

- `src/maps/cpp/mapd/engine.cpp` 在任一 voxel/accumulated 容量拒绝后将
  `capacity_limited_` 累积置为 true；衰减释放空间不会清除，只在 epoch 重置时清除。
  `cpp/mapd/dds.hpp::EvaluateReadiness` 据此持续返回 `map_capacity_limited`。
  这是代码可达的恢复状态缺项，当前 12 秒实机采样和回放没有触发容量拒绝。
  已改为新一次非空观测被完整接收后恢复当前容量状态，历史拒绝计数保留；
  仅等待时间流逝或输入完全被过滤不会清除故障。碰撞快照完整性仍单独判断。
  恢复状态不代表历史漏收区域已补扫。
- `web/src/workers/cloudDecoderCore.ts` 按世界 Z 的固定 `[-20,20]` 范围过滤每一个点，
  没有使用机器人所在高度或用户选择楼层。直接调用生产 decoder，完全相同的合法
  PCLD v2 单点帧在地图 Z=0/19/20 时显示 1 点，Z=21/−21 时显示 0 点。
  这是多楼层显示缺陷，不是当前低于 4 m 场景缺失的原因。已删除该绝对高度裁剪，
  并修正 HTTP 点云回退的同类 ±20 m 裁剪，以及保存地图默认 ±50 m 裁剪。
  非有限坐标仍拒绝，用户显式选择的高度过滤仍有效。
  复算入口为 `build/go2-ground-observation-20260914/check-height-bound.mjs`。

验证：C++ `mapd_engine / voxel_layer` 2/2 通过；Web 17 项通过，TypeScript 构建及
修改的生产 Web 文件 ESLint 通过。回归覆盖单次命中保留、窗口移动先释放后接收、
容量故障恢复、无有效输入不恢复、epoch 清空，以及地图高度平移后显示一致。
`window-replay/` 使用生产体素实现回放同一 50 帧，最终保留低处体素 4,740 个、
总计 41,785 个，无容量拒绝或发布遗漏；本机 Windows 更新均值 2.20 ms、最大
4.97 ms。这不是 NX 性能证据，也不覆盖满缓存规模。当前 `.29` 尚未安装这些改动，
未重启或保存用户正在进行的建图会话。

### 残留杂点：现有处理与参考项目

- 输入已有有限值、距离、雷达标签过滤及下采样。下采样用于降密度，不能等同去噪。
- 实时碰撞层使用命中/穿过射线更新占据概率及滚动窗口回收；当前命中立即参与碰撞。
  目前没有通用统计离群点或半径邻域过滤。孤立有效回波可能造成短时占据，但不能仅凭
  稀疏就删除，以免移除椅腿等薄障碍。显示表面稳定不等于实时碰撞已消除杂点。
- 保存时已有原生 prune 清理，依赖 `poses.txt` 和扫描 `patches/*.pcd`，按地面高度和
  多帧/多次命中保护点。缺少记录且清理非必需时可能跳过，须看保存任务清理报告。
  当前移动实例评分只写报告、不参与删除，尚无完整的可见性自由射线反证；因此不能
  保证反复出现的杂点或动态残影全部滤除。默认 body Z ≤ −0.45 m 的地面保护阈值也
  不等于 Go2 的实际地面分类，需结合站姿和实测地面验证，未盲目放宽。

参考官方实现：

- [FAST-LIO](https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp)
  用移动局部地图窗口删除远处空间块，支持本次有界空间保留方向。
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping)
  的 visibility cleanup 利用可见性/射线更新，适合继续研究历史残影清理，不能用
  “本帧没扫到”代替确实观察到空闲。
- [ERASOR](https://github.com/LimHyungTae/ERASOR)
  针对保存地图动态残影；本项目现有自主实现仅覆盖部分流程，不等同官方完整算法。
- [PCL RadiusOutlierRemoval](https://pointclouds.org/documentation/classpcl_1_1_radius_outlier_removal.html)
  和 [StatisticalOutlierRemoval](https://pointclouds.org/documentation/tutorials/statistical_outlier.html)
  可作离线候选滤波。对这次 46,888 个诊断合并体素做等价半径规则探测：10 cm、
  至少 5 个其他邻点会移除 5,164 点，其中 609 个低处点（低处点的 12.85%）；
  20 cm、5 个邻点移除 209 点，其中低处 45 点。没有真值标签，不能把删除量称为
  去噪收益。结果和可复算脚本为 `filter-probe.json / filter-probe.py`。

后续杂点验收需分别标注静态薄物体、有效地面、移动物体离开后的残影，验证误删和
残留；先离线回放，再验证机载耗时。未把批量清理或未经标注验证的邻域过滤接入
实时避障，也未新增依赖。

### 进一步检索：官方代码、适用位置与新发现（2026-09-14）

本轮是公开源码审查与本库对照，不是这些算法在 Go2 上的效果排名。
参考代码未加入产品依赖，未改动实机运行模式。以下为本次读取的官方分支，
后续集成应固定实际使用的版本。

| 官方项目 / 已读入口 | 实际做法 | 对 LingTu 的判断 |
| --- | --- | --- |
| [Nav2 VoxelLayer](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/voxel_layer.cpp) | 障碍标记与自由射线清除分开，`raytraceFreespace` 调用体素射线清理 | 可对照实时碰撞层的命中/清除顺序和量程，不需要更换原生 DDS 架构 |
| [Nav2 DenoiseLayer](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/denoise_layer.cpp) | `denoise` 对单格做快速去除，较大阈值走连通分组 | 官方配置放在障碍层之后、膨胀之前；不能按膨胀后的块大小识别原始孤立噪点。5 cm 栅格中的细障碍可能仅占单格，应先离线评估 |
| [ANYbotics ElevationMap](https://github.com/ANYbotics/elevation_mapping/blob/master/elevation_mapping/src/ElevationMap.cpp) | `visibilityCleanup` 根据传感器到观测点的射线生成高度上界；只处理超出扫描保留时间、且考虑方差后仍与可见空间矛盾的旧高度 | 优先参考“可见空闲证据 + 时间 + 不确定度”。其单层高度图不能直接替代我们多层 3D 地图 |
| [STVL](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) | 按时间衰减，结合传感器视锥加速清除，底层 OpenVDB | 可参考动态历史的寿命管理；视锥内不等于每处都被射线实际扫过，不照搬清除时间或增加 OpenVDB 依赖 |
| [ERASOR](https://github.com/LimHyungTae/ERASOR) | 扫描与地图的伪占据差异、局部地面保护用于动态残影清理 | 更接近保存时批处理问题，地面和坐标系参数必须适配 Go2 |
| [ERASOR2](https://github.com/url-kaist/ERASOR2/blob/main/src/erasor2/erasor2.cpp) | `setScanAndPose / setSubmap` 保留地面与实例标记，后续区域比较及实例判别参与清理 | 本库命中计数和 XY 格评分不能等同官方实现；已有可选参考路径可用于离线对照，不直接变成机载默认 |
| [Removert](https://github.com/gisbi-kim/removert) | 基于配准扫描和多分辨率距离图做离线动态点去除 | 可对照漏删/误删；该仓库 README 仍将 revert 步骤列为待补，不能仅凭论文标题宣称公开代码已完整实现恢复阶段 |
| [Dynablox](https://github.com/ethz-asl/dynablox/blob/main/dynablox/src/processing/ever_free_integrator.cpp) | `blockWiseMakeEverFree` 要求已观测空间及邻域在多帧内保持空闲；结合占据历史、聚类与跟踪检测动态物体 | 值得参考独立帧证据和稀疏扫描缓冲；未知体素不当作自由空间。整套 TSDF/voxblox 集成不是本轮的轻量修复 |
| [Patchwork++](https://github.com/url-kaist/patchwork-plusplus/blob/master/cpp/patchworkpp/src/patchworkpp.cpp) | 分区域地面估计、反射噪声处理、地面候选恢复 | 用于地面识别与保护，不是通用动态点清理器；地面标签仍不等于机器人可通行 |

参数与接口上的具体约束：

- [Nav2 去噪文档](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/denoise/)
  默认最小连通组 2、8 连通。此处“组大小”是栅格数，和 PCL 的点邻居数不是同一个量。
  对当前 5 cm 网格，薄椅腿/线缆是否仍可检出必须独立测量。
- ANYbotics 的 `add` 保存最低观测高度与传感器位置；清理使用扫描时间和 3σ 高度边界。
  借鉴时应保留三维射线、正确传感器原点、端点及遮挡后方保护，不能退回整列删除。
- [Patchwork++ 参数源码](https://github.com/url-kaist/patchwork-plusplus/blob/master/cpp/patchworkpp/include/patchwork/patchworkpp.h)
  包含 `sensor_height=1.723 m / min_range=2.7 m` 的默认值，直接用于 Go2 会漏掉关键近处区域。
  RNR 实现还同时使用下视角、低于预计地面的高度和 intensity，不是简单的低强度过滤。
  我们 SDK 流保留 uint8 reflectivity 为浮点值，不能假定它已经归一化到 0–1。
- 官方 [MID-360 产品说明](https://www.livoxtech.com/mid-360) 说明角分辨率会随积累时间改善。
  因此短帧没有点不代表该处空闲，等待多帧也不能让被遮挡或视场外区域自动变成地面。
- 本轮查看的 ERASOR2 主分支已支持独立 CMake 程序；不能再笼统称它运行必需 ROS。
  但其 [输入流程](https://github.com/url-kaist/ERASOR2/blob/main/USAGE.md) 仍依赖地面、实例
  预处理和对应位姿，且参考实现按其许可证隔离，不纳入默认产品。

#### 对照本库后追加确认的缺口

1. **雷达质量信息没有充分使用。** `src/localization/slam/cpp/fastlio.cpp::toPclCloud`
   只检查 `tag & 0x30`，接受该字段为 0/1；SDK 流原样传递完整 tag。
   [MID-360 协议](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)
   还定义 bit 2–3 的微粒质量和 bit 0–1 的相邻物体间异常回波质量。例如 `0x08 / 0x02`
   可以通过当前 tag 条件。代码缺口已确认，但尚未取得本次现场各 tag 的计数与空间分布，
   不能宣布它就是当前杂点来源。先核对实际设备/固件标签语义并统计，再选择过滤策略；
   不直接丢弃所有非零 tag 或所有低反射率点。
2. **高命中保护不要求跨帧。** `src/maps/prune/cpp/core/evidence.cpp::isProtected`
   用 OR 组合地面命中、至少 2 帧、至少 3 次命中；`cleaner.cpp` 对每一个 patch 点计数。
   同一帧同一 20 cm 体素中的 3 点即可受保护。这是明确的现有规则，能保留单帧稠密
   结构，也会保留部分噪点，不是稳定静态物体的证明。后续应让命中与自由空间反证
   一起决定是否清理，不能仅把 OR 改成 AND 后任由有效单帧地面被删。
3. **地面保护仍是 body Z 阈值。** `local_pt.z <= -0.45` 不包含局部法向、平面残差、
   重力方向或支撑连通性。它可能漏保护真实地面，也可能保护低于地面的反射点。
   这些是规则的局限；具体现场误删量尚未验证。应在保存清理的地面标注阶段修正，
   通行/台阶能力仍由 traversability 与导航约束裁决。
4. **能力文档曾比实现更乐观。** prune README 把 label/submap/protect 写为 done，
   但 `core/flow.cpp` 明确写 partial，本轮已使表格和源码一致。部分现有迁移测试只
   断言文本/符号存在，并不能证明动态残影已清除或有效地面没有被删。

另一个核对结果：当前 rolling occupancy 并非完全不清除旧障碍。按默认参数，从
饱和 log-odds 3.8918 降到占据阈值 logit(0.8)=1.3863，需要至少 3 次纯 miss 更新
（每次减 0.8473），且当前命中必须消失。这是公式推算，不是现场时延实测，也不是
“任意再来 3 帧就能清掉”：射线未穿过、区域被遮挡或重复出现噪声时，条件并未满足。
应优先记录真正阻挡路径的格子的 hit/miss/当前命中，区分观测不足与清除逻辑错误。

没有贸然修改的疑点：协议将 `time_interval` 描述为首末点时间差，而本库 SDK 流按
点数 N 分配间隔；但本次读取的 [官方驱动](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/comm/pub_handler.cpp)
也使用除以 N。资料与实现存在口径差异，目前不能据此断言应改为 N−1，更不能将它
当成大范围建图残影或控制卡顿的已证实根因。保留现有行为，后续用原始包时序核对。

#### 建议实施顺序与验收证据

1. **优先核对阻挡格子的来源。** 对现有带时间与位姿的扫描回放，输出 tag 分布、
   hit/miss 与当前命中贡献、旧点年龄，以及过滤前后的低处/薄物体点保留；日志统计
   放在拥有该数据的 native lidar/mapd，不在 Web 另造占据地图。
2. **补历史表面的可见性清理。** 复用原生射线和位姿，窗口内旧表面只有在收到有效
   穿过证据时才进入清理判断；端点、遮挡后方、未知区域不按空闲处理。清理负载与
   碰撞发布时延分别测量，不能用表面点减少替代避障正确性验收。
3. **补保存时的地面与反证判断。** 比较局部分区域地面拟合和现有高度阈值的差异；
   保留现有原图备份及 removed 输出，对有效地面、薄物体和实际移走物体做标注。
   先完善当前 prune 所需证据，再评估是否值得引入完整实例级方法。
4. **再决定邻域/连通组去噪是否接入。** 无人静态场景、移走纸箱、薄椅腿、桌面下
   地面、坡面/台阶、机器人转身各自验收误删和残留；NX 上测更新耗时及清除延迟。
   当前只有近静止 12 秒数据，不能代替移动物体移开后的真值验收。

以上为本地研究和实施建议，未声称完成这些后续算法，也未将官方桌面性能转述为 NX 性能。

## 2026-09-14 周期定位与全局恢复修复（`.29`）

修正 `.28` 实机场景暴露出的调用链：保存/显式位姿继续严格 `verifySeed`，
已有地图对齐的周期预测改走既有 `MapIcp.refine`，连续三次失败和显式全局请求通过
`RelocalizationSearch::Global` 真正进入全局搜索，不再自动补回旧初值。
已有对齐的全局结果仍受原门控限制，未放宽 0.15 米、2 度偏航、5 度倾斜等 Go2 限制。

本地及 NX ARM 的 `messages_fastlio2_mock_flow / messages_track_seed /
messages_relocalization_gate` 均为 3/3 通过；本轮 Python 路由回归 4 项通过，
Ruff 与独立审阅通过。一个既有宽入口测试仍因旧 `runtime_contract.py` 文本断言
找不到 `slam_relocalization_request` 失败，未改动该旧诊断目录。
`.29` 已正式安装到 `/opt/lingtu/releases/v2.3.0-go2.20260914.29`，安装器通过
ProductControl 恢复 `map/camera`，7 个 unit 全部 active。安装后的会话为
`product-1676b0ff211741dbb8e0d1780e0901d6`；HTTP 确认为 `mapping`、位姿新鲜、
`authority=none`、无活动任务、执行 `IDLE`、零运动读数。网页已连接。

同源数据回放使用从当前 NX 导出的 `map.pcd`（23,740 点，没有同目录语义地图）
和 7,430 点去畸变 body 扫描。严格种子核验得到 `2325/3706 = 62.74%`，被拒绝；
局部细化虽收敛，但候选相对已有对齐变化 `0.885 m / 7.995° yaw`，原门控仍拒绝跳变。
本地 Windows/PCL 的细化耗时约 118–123 ms，不能当作 NX 时延或实机恢复证明。
这说明实际初始对齐仍需结合机器人所在位置核对，不能把调用链修复等同导航通过。
未发送导航目标；`.29` 尚未通过异位旧地图定位及导航验收，当前仍保留可用建图 Product，
等待更明确的现场初始位置。已向用户询问相对原建图起点的距离和朝向，或回到原起点。
详细回放和原始证据在 `build/go2-release-20260914-v28/localization-replay/`。

## 2026-09-14 导航启动与失败回退修复（`.28` 机载结果）

用户确认继续使用旧地图 `go2_expanded_20260914_0350`，机器狗当前位于原场地的其他位置。
`.27` 的正式 `map/camera -> nav/camera + scan` 切换读入了 23,740 个地图点，
但 BBS3D 首轮全局配准未收敛/超时，SLAM 在启动就绪期限内仍为
`LOCALIZING / map_tracking_initial_alignment_pending`。这次没有发送导航目标或非零运动命令。

本轮修复三个实际启动链路问题：

- `src/localization/slam/cpp/cyclone_runtime.cpp`：ProductControl 的无初值
  `track-against-map` 请求不再清掉启动时读取的同地图完整位姿种子；换图不继承旧种子，
  显式初值仍优先。种子仍须经过现有配准与重力一致性检查，不直接授予定位成功。
- `src/lingtu/real/switch.py`：旧 nav 已确认停止、目标 nav 尚未启动的中途失败，
  不再因不存在的目标导航端点无法停车而阻断旧 Product 恢复；实际存活的端点仍必须止动。
- `src/lingtu/control.py`：失败 JSON 保留完整 `phases/cleanup`，不再只输出最初的
  SLAM 错误而隐藏回退失败原因。

本地验证：实际 C++ 种子边界测试 1/1、相关部署入口 2 项、CLI 11 项、回退 9 项通过，
修改的 Python 文件 Ruff 通过。NX 的 ARM 构建和 `messages_track_seed`、
`messages_relocalization_gate`、`messages_icp_diagnostics` 三项测试通过；`.28` 已正式安装。
用户已经换位置，清种子问题不是此次配准失败的唯一原因。

`.28` 无运动请求实机重试结果：旧种子核验三次失败后，BBS3D 在 6.163 秒内找到候选，
经后续配准与门控产生过一次有效对齐。但是周期复核连续失败，15 秒后定位降级，最终
Host readiness 以 `http_data_not_ready / http_non_motion_safe_false / 503` 拒绝就绪。
完整切换报告为 `failed_rolled_back`，包含 `map:restored`、`previous_processes:active`、
`previous_session:active`；网页已恢复 `mapping` 和新鲜位姿。没有发送导航目标。
该现场回退发生在目标 nav 已启动后，验证了完整失败报告和后期失败恢复；
“目标 nav 尚未启动”的修复目前仍由针对性回归覆盖，不能混称同一个实机场景。

新定位根因：现场同源 body 扫描的固定初值核验内点比为 `2344 / 3638 = 0.64431`，
低于既有 `0.80` 门槛，尽管均方误差约 `0.012598`、空间支持比为 `1.0`。
当前周期复核错误地走了适用于可信初值的固定姿态核验；连续失败后运行时清空种子，
但后端又由已有 `map_odom` 补回初值，实际没有进入日志声称的全局搜索。
正在按显式初值核验、周期漂移修正和全局恢复分别使用既有算法入口修正；不降低质量门槛。
证据在 `build/go2-release-20260914-v28/`：完整切换报告、SLAM journal 和配对扫描。
扫描是当前同源去畸变 body 云，和对应 odom 时间差小于 1 微秒；不是失败异步任务的精确输入归档。

## 2026-09-14 当前发布：`.27` 建图主视图语义与只读验收

`v2.3.0-go2.20260914.27` 已通过正式 release 安装到
`/opt/lingtu/releases/v2.3.0-go2.20260914.27`。ProductControl 已成功启动
`map --variant camera`，RunPlan 的 7 个 unit 均为 active。准备安装时发生的 Sunrise/NX
网络中断已经恢复，不再是当前阻塞。`.25`/`.26` 的现场检查证明了数据传输、uint64 身份、
关节和相机链路，但只核对了图层是否到达及编号是否一致，漏查了两个地图层各自的实际用途。
下节 `.26` 记录因此明确作为历史版本验收。

本轮确认的根因和 `.27` 修复：

1. `maps.occupancy` 把雷达原点 Z−1 至 Z+2 m 的整个高度带压到二维格：带内任一 occupied
   体素就显示红色，只有部分射线经过且没有 occupied 才显示蓝绿。地面、桌面和物体都可能
   为红，蓝绿也不证明整列为空、地面有支撑或规划可走。因此它不适合作为建图默认主视图；
   `.27` 只把它保留为可选“局部投影”，使用单像素实色，不再为每格绘制高频斜线。
2. `.26` 默认点云误选了 `maps.accumulated_cloud`。该层实际是用于碰撞观察的占据点，存在
   列清理和时间衰减；当前样本中 82% 的残留点高于 1.8 m。Gateway 随后又做一次 0.15 m
   体素抽稀，使它更不像操作者期望的建图表面。这不是 SLAM 完整地图，不能靠改颜色把它
   解释成完整建图。
3. `.27` 改用 mapd 已有的 `maps.voxel_cloud` 5 cm 局部观测表面，每帧只采用一个完整原生
   快照，取消 Gateway 的 0.15 m 二次体素抽稀，继续遵守最多 60,000 个网页展示点的预算。
   Gateway 不在 Python 中另建或累计地图。网页默认打开近距三维“空间点云”，按真实 Z
   高度着色并称为“局部地图点/局部表面”；空白保持未确认，完整 SLAM 建图须保存地图后查看。

这些修改只纠正网页数据源、显示层级和文字语义，不删除或改写原始碰撞占据数据，不改变
mapd、SLAM、规划图、局部避障或导航控制输入。21 项受影响的前端测试和 23 项后端显示源与
快照测试通过。

`.27` 的现场只读证据记录在
`build/go2-release-20260914-v27/live-verification.json` 和
`build/go2-release-20260914-v27/joint-age-distribution.json`：

- 20.062 s 内收到 183 帧关节 SSE，即 9.12 Hz；每帧 12 组 q/dq/tau 均有效。源龄平均
  47.12 ms、P95 81.35 ms，但有 1 帧达到 784.25 ms，超过网页现有 500 ms 阈值并被正确
  显示为过期，不能把本轮描述为全部低延迟。
- HTTP 地图返回 14,761 个点，点云和健康元数据的 sequence、reset epoch 完全一致，来源为
  `maps.voxel_cloud`。另采集 39 帧 occupancy，身份、epoch 和未知/射线经过/表面回波三类
  数据均有效。
- 两次 JPEG HTTP 请求均返回 200，序列前进且内容不同。原生彩色与深度传感器采集率为
  29–30 Hz；该数值不是网页 JPEG 帧率，网页实际显示的是彩色图像与深度采集状态。

浏览器实机验收中，建图页默认以近距“空间点云”显示 17,496–17,620 个局部地图点，机器人
保持近景并使用真实关节姿态，点云按高度着色。切换“局部投影”后没有密集斜线摩尔纹，界面
明确说明它不是地面或通行图；相机实际彩色图像也已显示。局部表面仍不等于完整全局 SLAM
地图，画面空白也不证明该处已经建好或可走，完整建图须保存地图后查看。本轮只读取状态和
切换显示，没有发送运动目标，也没有进行运动或导航验收。

## 2026-09-14 历史版本验收：`.25`/`.26` 实测关节与地图传输

`v2.3.0-go2.20260914.26` 已通过正式 native release 安装，当前链接为
`/opt/lingtu/releases/v2.3.0-go2.20260914.26`；ProductControl 已正式恢复
`map --variant camera`，RunPlan 的 7 个 unit 均为 active。安装和本轮只读检查没有发送
非零运动命令。`.25` 暴露的下列三个问题已随完整的 driver、native client、Host/Gateway
和 `web/dist` 更新进入 `.26`；后续发布仍不能用下文只替换少数文件的历史 `prepare_*`
补丁包。

### `.25` 现场问题与 `.26` 修复状态

1. mapd 的实际 `reset_epoch=117269136217079808` 是 uint64。`.25` 将它作为 JSON number
   传到网页后超过 JavaScript 安全整数范围，观测图因此显示“观测图已切换”并拒收数据。
   `.26` 的 Gateway 在 JSON 边界把 map scene、occupancy 和 elevation 的 reset epoch
   作为规范十进制字符串精确传输；网页把它当作不透明身份精确比较。旧数字只接受安全整数，
   不用舍入后的 number 弱化同源检查。generation 和 observation sequence 保持数字。
   实际网页已显示三色观测图，没有再出现 epoch 拒收提示。
2. `.25` 的 `map/camera` 会话已启动，但 NX Python 环境缺少发布配置声明的 OpenCV，JPEG
   relay 因此无法生成网页画面。NX 已补装并锁定 `opencv-python-headless==4.11.0.86`，
   原有 `numpy==1.26.4` 未改变。`.26` 网页已显示实际办公室彩色画面；相机状态中的彩色、
   深度传感器采集率均约 30 Hz，但网页 JPEG 推流配置为 10 Hz，网页没有显示深度图。
   两次 HTTP JPEG 请求均返回 200，大小分别为 20,512 和 20,532 bytes，内容不同；相机序列
   从 3296 增至 3339，确认不是重复旧帧。
3. `.25` 的 Gateway 在每次导航状态投影时同步执行完整 session/map 查询，阻塞了同一事件
   处理链上的关节 SSE，网页不能稳定跟随原生关节频率。`.26` 的高频状态路径改为复用轻量的
   最新运行事实，不在每条状态事件中执行完整地图查询。`.26` 在 20.109 s 内收到 259 帧
   关节 SSE，即 12.8798 Hz；`.25` 对照只有 2.43 Hz。259 个 source stamp 均不同且递增，
   每帧 12 组 q/dq/effort 全部有限，源数据年龄平均 35.48 ms、最大 168.21 ms。

本次发布解决两类直接影响操作者判断的问题：

- Go2 driver 从官方 `rt/lowstate` 读取 12 个腿部关节的实测 `q/dq/tau_est`，经 typed DDS、
  HostBus 和 `joint_state` SSE 到网页官方 URDF。网页只接受完整、唯一且有限的 12 关节样本；
  首帧直接对齐，后续按相邻接收间隔做有界插值，不外推、不生成假步态。
- 源时间戳用于排序，后端 `source_age_s` 与浏览器单调接收时间共同判断新鲜度，避免用两台
  机器的墙钟比较。超过约 500 ms 或连接中断时冻结最后实测姿态并明确显示过期；从未收到
  有效数据时才显示固定的“展示站姿”。关节事件走独立的小型状态流，不让整个 App 随最高
  30 Hz 的遥测频率重绘。关节遥测是可选显示数据，不参与运动许可或 Product 就绪判定。
- `map` Product 默认使用俯视“观测图”：蓝绿表示该高度带被观测为空，珊瑚红表示存在
  占据回波，灰色表示当前滚动窗口未知。蓝绿不等于地面有支撑或规划可走；窗口外也不等于
  从未建图，格数统计不能当作全图完成度。点击观测格只显示坐标和语义，不发送目标。
- 累计点云、当前扫描和 occupancy 使用各自缓存与采样节流，避免图层按帧互相替换；累计层
  默认提供稳定参考，“实时雷达”独立开关。超出网页预算时保持全范围均匀抽样，occupancy
  缩图保留占据优先和未知语义。没有有效栅格、数据过期或身份不一致时，不用稀疏点云或
  空画面臆测“无地面”或“没有建图”。
- occupancy 是高度带的二维 XY 投影，`origin.z=0` 不是测得地面。有效 Go2 位姿下，网页
  只把二维观测图、通行图和参考网格显示到默认站姿脚下的参考高度，并明确标注
  “二维投影 · 非地面高度”；点击射线使用同一显示平面。实际点云、高程、规划查询与导航
  目标 Z 均保持原值。

当前证据边界：本地前端关节、断流、地图投影和关联显示 27 项测试通过；typed DDS 往返、
driver/native client 2 项和 Host/Gateway 62+4 项相关测试通过。`.25` 在 NX 已验证原生链路
以约 25 Hz 发布 12 个真实关节，全部位置、速度和估算力矩为有限值；此前只读探针也曾在
5 秒内收到 2494 条 LowState。它证明真实关节源和原生发布可用，不证明网页持续刷新、
行走时 URDF 姿态对应或实机导航效果。`.26` 的只读网页已经显示“实测 · 12 个关节”、
“定位有效”、`0.00 m/s`、“无控制者”和“运动保持”；空间视角中四腿完整，二维投影位于
脚下且没有穿过机身。40 帧 occupancy 的精确字符串身份一致，每帧 40,000 格正常；末帧为
200×200、分辨率 0.05 m，unknown/free/occupied 分别为 14,064/22,533/3,403。网页已显示
实际彩色画面，相机状态显示彩色和深度采集约 30 Hz；该数值不是网页帧率，网页 JPEG 配置
为 10 Hz 且没有显示深度图。本轮未做行走姿态、运动指令或实机导航验收。

`.26` 安装前回归中，reset epoch JSON 边界、elevation 缓存和 SSE 重放相关 35 项通过，
包含上述真实 uint64 及相邻值不能混淆；前端相关 22 项、TypeScript 和生产构建通过；
Gateway 高频状态路径性能相关 112 项及 OpenCV 缺失依赖 3 项通过。另有 1 项历史
map-save 断言未纳入本轮结论，并已确认同样失败存在于 `.25` 基线，不把它写成本轮修复结果。

### 建图与相机正式启动（`.26` 已只读复核）

`map --variant camera` 在实时建图中增加原生相机预览，不需要保存地图，也不切换到导航定位。
普通 `map` 仍选择 `standard`；MuJoCo 的 `map`、`nav` 当前只声明支持 `standard`，
实机 `nav --variant camera` 保持可用。Go2 相机变体要求完整发布包包含
`bin/realsense_capture` 和 `bin/lingtu_camera_dds`。

安装后在目标控制器上通过正式适配器调用 ProductControl：

```bash
bash /opt/lingtu/current/scripts/lingtu --robot unitree/go2 --env real switch map --variant camera
bash /opt/lingtu/current/scripts/lingtu --robot unitree/go2 --env real status --json
```

适配器执行 `python -m lingtu.control`。NX 使用已配置的 Python 3.10 环境；不能误用系统
Python 3.8。建图不传 `--map`。启动包含原生零命令握手和就绪检查，不自动站起、不发送
非零运动目标。`.26` 已完成 NX 部署和 Product 激活，网页已回读实际连续彩色画面及深度
采集状态；网页没有显示深度图。这不包含相机外参精度、深度融合或导航运动验收。

### `.26` 浏览器只读验收

以下检查只观察或切换网页显示，不开始导航、不发送遥控、恢复、停止或其他运动命令：

1. 打开实际 App 并保持观察模式，确认页面连接到新 release；机器人出现时，“关节姿态”应为
   “实测 · 12 个关节”，机身和四腿没有周期性假动作。若数据自然断流，应冻结最后姿态并显示
   “数据过期”，恢复后再平滑跟随新样本。
2. 在 `map` Product 中确认首屏为俯视“观测图”，三类格清楚可辨；点选蓝绿、红、灰格后，
   文案分别说明观测为空、占据和未知，并持续说明观测不等于可走。
3. 切换“点云”及“实时雷达”，确认累计图稳定、当前扫描可独立显隐，图层不会交替闪空；
   数据缺失或过期时页面应说明来源状态，不把空白解释为未知地面。
4. 机器人有效定位时，二维投影应显示在机身下方且带“非地面高度”说明；真实点云位置不应随
   二维底图偏移。俯视与空间视图中的点选 XY 应一致。
5. 若当前 Product 明确包含 camera 变体，只观察真实帧和新鲜度状态；没有实际帧时保留
   “未加载/已暂停/数据过期”的事实，不用历史画面冒充现场。检查状态、目标反馈和相机提示
   切换时，主要画布和操作卡不应跳动或闪烁。

## 2026-09-13 建图与自主导航现场验收

### 2026-09-14 点云密度、覆盖与展示链路核查

#### 建图观测图与点云显示修复（本地）

`map` Product 的网页默认俯视“观测图”，数据直接来自 mapd 的
`maps.occupancy` 高度带投影，不从稀疏点云生成自由区域：

- 蓝绿：该高度带有自由空间观测，且没有占据体素；不等于已确认地面支撑。
- 珊瑚红：该高度带存在障碍回波。
- 灰色斜线：当前窗口内未知，需要改变位置或朝向补扫。
  窗口外不代表从未建图，面积统计也仅限当前窗口，不代表整图完成度。

点击只检查观测格、位置和补扫建议，不会发送运动目标。“点云”切换到累计观测表面；
“实时雷达”独立叠加当前扫描。导航模式保留保存地图的静态通行图与目标预览/确认流程。
没有有效观测栅格时只展示带说明的点云参考；数据过期、身份或坐标不一致时不保留旧色块。

Gateway 现在有界传送 occupancy 的 `int8` 格值及坐标、来源、序列、原始时间。
超大格网缩小预览时，任意占据仍为占据、全部自由才为自由、其余保持未知；
该处理只影响显示，不回写 mapd/规划器。建图点云每帧只采用一个原生完整快照，
累计层优先，避免 live/voxel/accumulated 共用频率阀导致内容交替；导航模式优先 live，
兼容扩展地图层关闭。HTTP 回退也改为稳定全范围抽样。

本轮未连接机器人、未部署到 NX，保存 PCD 和 SLAM 参数未改变。
前端、Gateway 需要一起发布；此前 `ui-camera-source-20260914.tgz` 不包含本轮修改。

本地验收：9 项观测格解码/身份/时效/坐标/渲染测试，以及受影响的 24 项网页地图、
图层尺寸、状态布局回归通过；后端 occupancy、单一快照、语义/SSE 回归通过。
前端 TypeScript、ESLint、生产构建通过；构建仍提示现有 Three.js chunk 超过 500 kB。
实际 App 的离线交互样例验证了三类格点击、全未知窗口、过期清图、独立雷达开关、
黑白主题和 390 px 窄屏。该样例使用明确标记的合成地图，所有 API 写请求均被拒绝，
不能当作本轮实机建图证据。旧状态布局测试因提示条改为固定占位而过时的断言已同步。

本地预览：`http://127.0.0.1:15173/@fs/D:/inovxio/brain/lingtu/build/go2-live-validation-20260913/mapping-observation-replay.html?observe=1`。

本轮为本地源码、已下载地图和历史记录分析；用户此前确认设备已关机或拔线，
没有重连设备、修改 SLAM 参数或发送运动指令。

直接读取 `build/go2-live-validation-20260913/expanded.pcd`：23,740 个 XYZ 点，
其中 23,020 个不同的 20 cm 三维体素；这份保存地图本身已经很稀疏。
较早的 `bidirectional.pcd` 与 `bidirectional-clean.pcd` 均为 14,842 个相同 XYZ 点，
不能把那一轮的稀疏解释成保存清理删点。点数本身不证明覆盖质量。

确认的链路与限制：

- Go2 Fast-LIO 配置：每 2 点取 1，距离 0.5–20 m，扫描体素 25 cm、地图体素 20 cm。
  `LidarProcessor::incrCloudMap` 用扫描降采样后的点维护 ikdTree；原始 `saveMap`
  直接 flatten 这棵定位内部地图。它不是持续原始扫描的稠密累计图。
- `stationary_thresh=0.08` 只检查雷达原点距上次入图位置的平移。未达到 8 cm
  时仍处理定位、发布新扫描，但跳过内部地图增量；没有按时间、朝向或新增覆盖更新。
  因此不能承诺原地停住就持续补密保存地图。原地转身会因雷达前偏带来原点位移，
  不能笼统说所有旋转扫描都会被跳过。
- patches 记录预处理、去畸变后的扫描，默认至少间隔 1 秒且平移 20 cm 或 yaw 5°。
  PGO 成功时可按优化姿态合并这些 patches；静止未记录的新扫描无法由 PGO 恢复。
  上述限制不替代下文历史起点缺口证据：该处 patch 点数已经为 0，缺观测早于保存。
- 当前网页 registered scan 默认再经 15 cm 体素；部署模板最多发送 5,000 点。
  在线地图为有范围与点数上限的预览缓存，不能当作完整磁盘地图。
  场景点尺寸和透明度也使点之间的空隙明显；不能用调大点遮盖未知空间。
- SCAN 使用 mapd 的完整局部碰撞格；上述网页采样不回写规划地图。

已修复的本地显示缺块：

- `/ws/scan` 与 `/ws/cloud` 超过发送上限时，旧的整数步长再截断会遗漏点集后段；
  改为覆盖整个点集的均匀索引采样，保持原有发送上限。
- 地图管理页 PCD 预览原先只解析 ASCII 前 30 万点或 binary 前 50 万点；
  改为覆盖完整文件的有界预览。当前 expanded 地图小于该上限，故此问题不是
  这张小地图稀疏的原因。

本地验证：9 项 Gateway 采样/配准扫描回归、4 项 PCD 预览回归通过；
相关 Python Ruff、前端 ESLint 和 TypeScript 项目检查通过。用超过预算且首尾属于
不同空间区域的点云检查覆盖范围，仍保持点数上限。此处新增修复未部署，
也尚未加入此前任务准备的 `ui-camera-source-20260914.tgz`，后续发布需从当前源码重打包。

后续建图改进应分开推进：先保留定位工作图的实时预算，评估独立保存密度和有界的新覆盖
采纳条件；用相同已去畸变扫描回放比较地面覆盖、障碍保留与处理耗时，再选择参数。
网页默认应以稳定的保存地图/可通行层为主，实时扫描单独开关，并明确未知与无支撑。
原始观测没有覆盖的地面需要改变观察位置和朝向；深度补点还需要外参和时间对齐。

上文后续观测图修复已处理三个在线地图层共用缓存和 HTTP 随机抽样问题。
另有已定位但尚未修改的限制：场景页固定请求 8 万 XYZ 点，可能超过 mapd 查询的
1 MiB JSON 响应上限；这是加载容量问题，不归因为雷达测量错误。

### 2026-09-14 网页选点、状态闪烁与相机预览审阅

用户截图目标 `(0.40, -0.33, 0.10)` 的只读预览返回 `empty_path`，
起点 `(-0.4777, 0.0277, 0.0109)`，本次搜索约 1000.46 ms。
它证明这次搜索未找到路径，不能单凭该错误证明现场物理不可达。
网页现在说明“未找到连接当前位置的路径”，保留选点，允许重新预览。
绿格表示目标所在高度层满足静态通行条件，仍需从起点连通；红格表示受阻，
灰格表示缺少支撑，实时雷达和局部风险不是该静态图的替代品。

发现并修复的交互问题：

- App 用上一次一秒计时值校验刚收到的 SSE，误判时间来自未来，导致授权提示闪烁。
  现在使用两个同源浏览器时间中的最新值，保留过期、断链和原生运动许可判断。
- 只读预览不再被运动授权拦住；开始导航仍检查当前运动许可及明确回执。
- 恢复控制保留目标，等待页面收到原生恢复状态再结束等待；异常在原位置说明，
  避免接口查询已恢复而 SSE 还显示暂停时反复点击。
- 选点时停止视角跟随；反馈集中在固定尺寸目标卡，避免重复 toast 和按钮跳动。
  起点平移超过 0.25 m 或高度变化超过 0.15 m 时，旧预览失效但保留目标。
- 目标卡移到左下角，相机缩小让出操作区；窄窗口的次要视角按钮保留图标和提示，
  工具栏允许换行，避免“空间视图”被逐字挤成竖排。
- 通行图轮询不再跟随瞬时诊断可用性反复重启；请求失败保留上一帧至 10 秒有效期。
  保存地图默认俯视，局部风险改为可选图层，避免与静态红绿格重叠误导。
- 正常替换目标不再当作障碍报警；已完成/取消任务卡 8 秒后收起，失败和暂停仍保留。
- 相机常驻入口可收起；JPEG 解码完成后再替换画面，3 秒无新帧显示暂停。
  JPEG 不再等待缺席的可选 WHEP 服务 8 秒；新增实际帧率/可用状态接口。

相机缺失的直接原因：当前 `.24` 导航运行配置没有 camera 进程及 Host 预览接线，
`/api/v1/camera/snapshot` 返回 503 `camera_not_loaded`。
已在代码中补齐下节的 `nav --variant camera` 配置，并保留相机外参未验证状态。

本地验证：39 项相关 Web 回归、152 项 ProductControl/真实与仿真切换/相机接口回归通过；
Product 编译用例也通过。相关 ESLint、Ruff、TypeScript/Vite 构建及启动/安装脚本语法检查通过。
浏览器已核对选点预览和新版离线反馈；本轮没有发送机器人运动目标。
检查期间 NX、Sunrise 均断开，用户确认已关机或拔线，因此新版尚未安装到 NX。
相机采集器此前真实 RGB-D 验证通过，不等于本轮网页持续出图已验收。
离线浏览器分支测试使用真实 App、保存地图与历史相机帧，截断所有机器人 API/WS 请求：
`empty_path` 只出现一次；可达响应允许确认；暂停时禁止开始但保留目标；恢复确认后可继续；
移位 0.4 m 使旧预览失效；停止图传 3 秒后隐藏旧帧并显示暂停。
760 px 窗口中恢复前后画布均为 703×365 px、顶部 330.14 px，目标卡均为
440×210.80 px、顶部 472.34 px，提示切换未产生布局位移。
分支中的可达路径为接口夹具，只验证交互，不作为算法或实机可达证明。
待重新连接后：安装新版、由 ProductControl 激活 camera 变体、检查定位和实际 RGB-D 帧率，
再验收网页连续画面与现场目标预览。不能把这些本地修复宣称为导航卡住已全面解决。
待传输文件已整理为 `build/go2-live-validation-20260913/ui-camera-source-20260914.tgz`，
包含本轮源码及已构建的 `web/dist`，文本统一 LF；这是源码更新包，尚非 NX 原生发布包。
重新连接后在 NX 源码目录解包，通过 `package_native_release.sh` 生成下一版本，再走正式安装器。
本地公开 CLI 的 `nav --variant camera --dry-run` 已返回 `planned`，没有启动任何进程。

### 2026-09-14 D435i 相机代码接入

网页接入配置：`nav` 的 `camera` 变体由 ProductControl 一起管理原生 camera
进程、Host SHM 相机适配器和 JPEG relay。`standard` 变体保持纯雷达导航。
Go2 的采集驱动与序列号来自本目录 `robot.yaml`；相机外参仍未验证，深度不融合进导航。

```bash
python -m lingtu.control switch nav --robot unitree/go2 --env real \
  --variant camera --map go2_expanded_20260914_0350 --local-planner scan \
  --set scan_planner.max_acceleration_mps2=1.0 --json
```

网页“现场”工具栏的“相机”控制画面显隐，`/api/v1/camera/status` 报告实际帧率和
数据是否过期，`/api/v1/camera/snapshot` 和 `/ws/camera` 复用现有 JPEG 链路。
切换配置不发送运动目标；发布安装器保留当前 Product 变体。

重启后的首次出图验证已通过：NX 枚举 `8086:0b3a`，D435i 序列号
`419522073370`，视频节点 video0–video5。实际 SDK 报告 RGB 为 Inverse Brown
Conrady，但五个系数均为零；修复仅允许该恒等畸变情况通过，不把非零逆畸变
错误当成正向 Brown。NX 内参回归通过，随后录制 30 组 RGB-D 成功（退出 0）。
90 条 LTOB 记录完整，640×480，RGB/深度/内参同组时间戳一致，深度为 uint16 毫米。
主机接收时间计算约 31.32 组/秒、最大间隔 33.92 ms（短段含启动缓冲，非硬件帧率验收）；
末帧有效深度占比 97.07%，有效深度中位数 2.613 m。导出彩色图已人工检查。
证据 `build/go2-live-validation-20260913/d435i-boot-validation.json`、
`d435i-live-color.png`、`d435i-live-depth.png` 和 `d435i-boot-test.records`。
这是相机真实采集验证；RGB-D 空间精度、相机外参和导航融合尚未验收。

新增 `src/drivers/real/camera/impl/realsense/` 原生 librealsense2 采集程序，复用
`lingtu_camera_dds` 的 LTOB v2 / SHM / DDS 发布链路。RGB8、对齐彩色视点的
uint16 毫米深度、彩色内参使用同一主机接收时间戳；不把设备开机时间当成 UTC。
保留零深度空洞，按 SDK 深度单位转换，处理图像行跨度。D435i IMU 未接入 SLAM。

构建：`bash scripts/build/build_realsense_native.sh`，需要目标系统安装官方
librealsense2 开发包及运行库。正常发布打包在检测到已构建的 realsense_capture 后
包含该可执行文件。相机启动脚本新增 `LINGTU_CAMERA_DRIVER=realsense_native` 和
`LINGTU_REALSENSE_SERIAL_NUMBER`，保留 Orbbec 默认行为，不向 D435i 传 Orbbec 参数。
具体采集、SDK 路径及启用方式见 `src/drivers/real/camera/impl/realsense/README.md`。
设备清单新增禁用的 D435i 条目；仅修改清单不会为 Product 添加相机角色。

验证：WSL g++ 使用官方 librealsense v2.56.5 头文件，以 -Wall -Wextra -Wpedantic
-Werror 完成采集代码语法/类型编译检查；相机启动参数分流测试通过，相关 shell
语法检查通过。随后完成 SDK 链接、NX ARM 构建；RGB-D 对齐精度尚未验收。
保留 camera calibration=unverified，不把尚未标定的深度融合到导航地图。

2026-09-14 NX 运行验证：librealsense v2.56.5 和 nlohmann/json v3.11.3 源码
由本机下载后传入 NX，SDK 使用 FORCE_RSUSB_BACKEND=ON、关闭 CUDA/图形示例/固件下载，
低优先级双任务原生构建成功。NX 无法下载 JSON，SDK 构建工作副本的 external_json.cmake
改为引用本地同版本源码，并将 include 放到 SDK 要求的 build/third-party/json 下。
这只是离线依赖路径调整，没有修改 SDK 算法或项目相机协议。

SDK 安装位置：`/home/unitree/lingtu-deps-20260910/realsense-install`。
采集程序：`/home/unitree/lingtu-source-20260910/build/realsense_native/realsense_capture`。
file 确认 ARM aarch64；ldd 中 librealsense2.so.2.56 正确解析到该 SDK 安装目录，
没有 missing library。SDK 与采集程序的构建及链接退出码均为 0。
历史记录（本次重启前）：root 下 `--list-devices` 退出 0 但设备列表为空；`--max-frames 30` 退出 1，
错误为 `D435i not connected`，输出文件 0 字节。USB 设备树仅有 tegra-xusb 根集线器，
无 D435i 节点，因此不能归因于普通用户权限，也不能声称已经出图。
本次重启后的识别和采集结果见本节开头。

本地证据目录 `build/go2-live-validation-20260913/`：realsense-configure.log、
realsense-build.log、d435i-capture-build.log、d435i-link-check.txt、d435i-capture.err。
连接被识别后，先运行采集程序的 `--list-devices`，再用
`--serial-number SN --max-frames 30` 录制；验收真实 RGB-D 后才接入 ProductControl 的相机角色。

### 2026-09-14 轨迹生成后的执行等待侧录

用户报告画出轨迹后长时间不移动。初次 HTTP 状态已是上一任务 SUCCESS，随后原生
快照捕获用户新任务：全局重规划 67.014 ms、局部轨迹 193 点，但
`spline_execution_frozen`，朝向误差 -0.875222 rad，执行时间 0，最终指令
vx=vy=0、wz=-1 rad/s，驱动回执 command_accepted。这属于行进轨迹朝向对齐，
不同于上一轮已修复的缺省终点 yaw。闭环跟踪器在朝向误差超过配置阈值时只转向，
暂停轨迹时钟；该快照不能量出整个对齐持续时间或实际转速。

随后 10 秒原生+SLAM 侧录又捕获 `recovery_translation_active`，指令约 0.15 m/s，
再转 `persistent_path_obstruction_replan`，约三秒后 `replan_budget_consumed`。
这说明等待还涉及脱困和全局重规划的切换，尚未隔离具体障碍及是否提前中断恢复。
不能简单归因于全局 A* 慢或加速度小，也没有因此放宽碰撞检查。
证据：`build/go2-live-validation-20260913/delayed-start-native.json`、
`delayed-start-samples.jsonl`、`delayed-start-nav.log`。本次仅采集用户操作，未发运动目标。

### 2026-09-14 网页选点无法替换旧任务与终点旋转

现场任务 `nav-task-1789362509105895264` 的原目标为 (-3.960805, 0.502858)，
已进入 `aligning_goal_yaw`，平移输出零、旋转输出约 0.186 rad/s；截图中新目标
(-2.10, 0.14) 尚待确认，预览接口实际返回 `navigation_busy`。这次不能归因为
新目标没有空间。已通过公开停止接口取消旧任务，确认 QUIET / CONFIRMED。

两处修复：网页通过通用 GoalRequest 发送 map_click，未填 yaw 却被 schema 默认值
变为 0；目标构造现检查实际提供字段，地图点未填朝向保持无朝向约束，显式 yaw=0
或其他角度仍保留，普通 coordinate 请求保持原行为。另一方面，独立预览任务不再
因存在 active_task_id 被拒绝，仍在全局规划计算或排队时返回 busy，保留预览互斥、
地图/坐标系时效检查；预览不下发运动目标。相关 Gateway 11 项回归通过。
原生 ARM navd 构建通过，正式安装 `.24`，ProductControl 使用保存的完整 seed
重新启动成功。现场无运动检查：IDLE / ACCEPTING / CLEAR / QUIET；目标
(-2.10, 0.14, -0.10) 预检通过，路径 1.908801 m、16.165 ms。
记录 `build/go2-live-validation-20260913/map-click-preview-online.json`。
未自动重放旧任务，尚未验收修复后的实机行走或执行中换点；不能将空闲预览通过
当作执行中预览的实机证明。

### 2026-09-14 状态、脱困与通行图本地修复

本轮已在重启后的 NX 编译、通过正式安装器部署为 `v2.3.0-go2.20260914.22`。
ARM 停车屏障、通行图写出、14 项 RecoverySequence 与 7 项 Executor 回归通过。
通过 ProductControl 启动 `nav + scan`，加载 `go2_expanded_20260914_0350`，
保留用户此前采用的 `scan_planner.max_acceleration_mps2=1.0`。

启动首次定位失败；保存 seed 为约 (-5.84, 0.81)，上次导航记录约 (-4.15, 0.72)。
用户确认在上次终点附近后，以该记录重试；seed 配准仍失败，随后全局回退匹配成功，
连续跟踪位姿约 (-5.81, 0.82, -0.064)，匹配重叠比例约 0.998。不能把初值直接当作定位。
当前通行图在线返回 97×171、0.2 m，962 格可通行、1,849 格受阻、13,776 格未判定，
与离线同地图同高度层回放一致。地图 epoch 为 1789328865258，原图未修改。

用户确认现场通行空间与立即停车条件后，执行目标 (-4.85, 1.10, -0.10)，
预览路径 1.177 m、12.51 ms，指令限速 0.20 m/s、到点半径 0.30 m。
任务 `state-recovery-supervised-20260914-1789361228` 约 9.3 秒报告 SUCCESS；
最终速度为零、QUIET、停车 CONFIRMED，控制租约已释放。现场主观反馈待补充。
记录：`build/go2-live-validation-20260913/state-recovery-motion-trial.jsonl` 与同名前缀 console。
此短路线未触发脱困，不能代替前阻后通的实机验收。另捕获到到点停车期间
`navigation_state_stale` 的短暂公开状态，必须继续处理；此次没有因此放宽状态时效。

- 状态：`gateway/navigation/projection.py` 的公开 `resume_required` 包含操作者接管锁；
  `motion.reason` 优先解释当前运动保持原因，停稳确认不再遮盖它。网页只在状态已知、
  恢复锁解除且 `motion.permission=CLEAR` 时提示恢复成功。Gateway 21 项、Web 10 项回归通过。
- 脱困：`src/nav/cpp/navigation/recovery.cpp` 无安全候选时，原来每个控制 tick 都消耗
  一次尝试，同一帧可能耗尽 3 次预算。现在第一次失败后等待新的 cloud 和 traversability
  generation。新增三项回归补丁前失败、补丁后通过；Recovery/Executor 共 25 项通过，
  包括 SCAN 从封闭起点等待新观测、后方出现唯一出口后输出有界后退指令。
  `teleop_avoid` 仍遵循操作者方向，不因这次修改擅自后退；碰撞判断没有放宽。
- 通行图（历史功能，2026-09-24 源码已移除，尚未部署；见[当前交接](../../../../docs/operations/go2-offline-mapping.md)）：原生全局规划器曾输出 `nav.status.json.planning-map.json`（实机为配置的
  nav status 路径追加 `.planning-map.json`）。同源查询地面支撑和机身净空，单一高度层，
  不合并上下楼层。0=缺少支撑/未判定，1=满足静态通行条件，2=占据/净空受阻。
  完整投影在后台计算，同地图、同高度层复用；超过格数预算明确不可用，不抽样冒充完整图。
  Gateway 只读 `/api/v1/navigation/planning_map`，核对当前 session、地图版本和导航状态时效。
  Web 默认显示通行图，可切回点云；断链和身份变化不沿用旧图。选点仍需路径预检，绿色不代表
  已与当前位置连通，也不代表动态障碍安全。
- 选点：新点击立即替换待确认点并清除旧预览，旧回执不能覆盖新点击或取消。预览失败保留原因；
  尚未预览的目标先显示“预览路径”，不会直接作为已验证目标发送。点击使用显示高度层，避免斜视图
  中固定 Z=0 拾取平面引起的位置偏差。

真实地图离线回放：`go2_expanded_20260914_0350/octomap.ot`，0.2 m、97×171，
当前 Z=-0.1 m：13,776 格缺少支撑/未判定，962 格满足静态通行条件，1,849 格受阻。
此前成功短路径的两个端点都落在可通行格；这些数字属于离线记录，不是当前现场。
原生投影平地、墙、净空、缺支撑、上下层隔离、预算/取消及写盘缓存测试通过，navd 构建通过。
离线程序加载、投影、缓存和两次写盘总计约 0.16 秒（WSL x86，不能当作 NX 性能）。
最终相关 Gateway 回归 51 项、Web 回归 27 项通过；类型与相关 lint 通过。浏览器使用正式
Scene3D 和该实图侧录验证红/绿格点击坐标及原因。点云视图也沿用当前权威高度层拾取；
重连需要新的通行图响应，不能先回显断链前的缓存。
离线交互预览位于 `build/go2-live-validation-20260913/planning-map-preview.html`，
需使用本地 Vite 预览，页面明确标注未连接机器人，不存在运动命令入口。

待实机验收：部署并确认当前版本；停止→恢复状态一致；同一障碍帧不会耗尽脱困；前阻后通时
有界退出并重新规划；网页可走/受阻格与现场障碍一致；选点、预览、到点停车完整闭环。
原始地图保持不变。

### 2026-09-14 后续网页目标反复受阻与提示栏跳动

后续用户网页任务出现 `replan_budget_consumed`，再次选点后长期在
`local_plan_pending` / `scan_actual_motion_blocked` 之间切换，并曾执行安全检查通过的
0.25 rad/s 脱困旋转。已通过公开停止接口取消当前任务，停车确认成功，急停保持有效。
这不能与此前 1.18 m 短路线的成功混为一项验收。

保存失败位图 `state-recovery-scan-failure.json`，机身地图位置
(-4.443469, 3.570156, -0.101332)，yaw=-1.238782。前查询点占据、后查询点空闲。
直接调用原生 Grid/RecoveryPlanner 对同一完整位图回放：16 个方向中 8 个有安全出口，
选中 bin 11，固定朝向机身位移 (-0.133939, -0.323358, 0)，长度 0.35 m，
`verified=true`、TranslationReady、无需旋转。记录与回放工具保存在
`build/go2-live-validation-20260913/` 下 `state-recovery-*` 和 `replay_recovery_bitmap.cpp`。
此为真实位图离线回放，尚非脱困实机运动通过。

代码缺口：`AutonomyTickController` 对最终 `scan_actual_motion_blocked` 只调用
`stopLinearMotion` 重置局部规划，没有将制动拒绝送回 Executor 受阻计时；
下一次成功规划又会清除规划器自身受阻计时。新增 `reportFinalMotionBlocked`，
独立保留最终制动拒绝的持续时间，达到原 blocked_interval 后触发现有安全脱困。
安全命令获准或新任务清除该计时；仿真反馈转换到执行时钟。没有改变占据格、
脱困速度或候选路径碰撞检查。两项新增 Executor 回归及两项相关安全回归在 Windows 通过，
ARM navd 构建、同四项 Executor 回归和 AutonomyTickController 测试均通过，
正式安装为 `v2.3.0-go2.20260914.23`。卡点脱困的实机运动验收仍待完成。

界面根因是条件渲染整块 sceneAlert，显示/隐藏改变地图容器高度。现固定 48 px 状态栏，
文字单行、省略长提示，窄窗口隐藏辅助说明；无告警时使用中性连接状态。
TypeScript/Vite 构建与定向 ESLint 通过，已更新 `.22` 网页，HTTP 确認返回新样式。
同一前端已包含在 `.23`；目前尚缺用户现场确认页面不再跳动。

`.23` 首次重启用显式 `(x,y,z,yaw)` 初值失败。用户确认一直原地站立，没有移动。
保存的 `track_seed.json` 保留完整四元数，位姿约 (-4.441, 3.573, -0.100)，
四元数 (-0.027014, 0.023147, -0.575394, 0.817102)。随后使用同一 ProductControl
启动命令、去掉 `--initial-pose`，恢复使用完整保存姿态，启动成功；连续四次配准通过，
重叠比例 0.997862，地图位姿约 (-4.446, 3.573, -0.097)。未修改 ICP 阈值或地图。
此对照支持姿态初值影响配准，但尚未通过同一扫描离线对照把影响完全隔离。
原地重启优先使用完整保存姿态，不应用只有 yaw 的人工初值覆盖它；机器人搬动后
仍须重新核对定位，不能把旧 seed 当作当前真值。
当前导航 IDLE、ACCEPTING、无控制者、无需恢复、运动许可 CLEAR，未重放旧目标。
记录：`final-braking-full-seed-slam.json`、`final-braking-ready-navigation.json`，
位于 `build/go2-live-validation-20260913/`。仍需处理停车确认期间的状态发布短暂过期，
并单独验收前阻后通时的安全脱困；启动恢复不等于运动验收通过。

### 2026-09-14 扩大地图首轮自主导航通过

“恢复控制无效”已现场复现：`operator_takeover_latched=false`、
`control_loop_hold=false`，但 `resume_required=true`。原生恢复请求只传前两项，
`MotionStopBarrier` 走 `autonomy_already_ready` 提前返回，HTTP 成功且网页提示
成功，独立恢复标志却未清除。修复为自治和遥控恢复请求都携带独立的
`resume_required`，进入原有零输出、停车确认、清除旧指令及恢复标志流程。
网页恢复接口还需观察实际 `control.resume_required=false` 才提示成功；
超时错误保留在暂停提示栏。原生 ARM 停车屏障回归（含独立标志的成功和
超时分支）、9 项控制循环连接契约、20 项网页回归及网页构建/lint 通过。
原生构建在未修改的 `control/autonomy.cpp:17` 仍有 unused-function 告警。

部署状态：`.20` 已暂存，但安装器恢复 nav 时两次遇到
`native map verify rejected: artifact_gate_failed:map write in progress`；
回滚 Product 也被同一错误阻塞。现场锁的 owner 为 `check-map-activation`、
PID 属于 mapd，随后锁自行消失，说明地图健康只读查询与最终验证发生争用，
并不是原始地图被删空。最后确认 current 链接回到 `.19`；不能据此声称
旧 Product 已恢复就绪。用户随后确认机器狗已关机或拔线。

本地 `FieldBackend.commit_map` 已在原 10 秒预算内重试这个精确的地图锁忙错误，
保持同一激活 token 和地图身份；缺失/无效地图立即失败，持续忙仍超时失败。
30 项 real-switch 测试（含短暂忙、持续忙、无效地图）及相关 Python lint 通过。
最终待安装版本为 `.21`，准备脚本在本地
`build/go2-live-validation-20260913/prepare_resume_release.py`；最后一次向 NX
上传新版 backend/准备脚本失败，必须重新上传后再组包，不能直接重跑旧脚本。
重连后先核对 current/session 与原地图仍在，用正式安装器及 ProductControl
恢复 `nav + scan`、`go2_expanded_20260914_0350`，再做恢复回执与实际状态一致性
验收。此轮修复未发送运动目标，最终现场恢复验收尚未完成。

后续网页选点验收发现：任务 `nav-task-1789333987398281984` 到点后仍显示
`resume_required`。NX 日志在 2026-09-13 21:13:48 UTC 附近记录停车确认失败：
零指令收到接受回执，但连续静止里程计只有 2/8；43 个回执后样本中 35 个
曾判为运动。随后 21:13:51 UTC 已确认 8/8 静止，保持状态需要显式恢复。
本次未取消停车证据门槛，也没有把负载告警直接当作该事件的原因。

地图显示核对：`/api/v1/maps/go2_expanded_20260914_0350/points` 实际返回
23,740 点，小于默认 30,000 上限；页面“雷达约 4,300 点”是当前扫描。
保存 `occupancy.npz` 为 100×173、0.2 m，14,856 格 unknown、2,444 格 occupied，
没有 free 格。该辅助二维投影不从无射线来源的 PCD 臆造自由空间；当前 nav
全局规划消费 `octomap.ot`，不能拿此二维辅助图代替三维支撑/净空判断。
仍缺少直接供操作者阅读的、与全局规划判定一致的地面/净空投影视图。

网页现默认隐藏保存地图上的实时点云叠加和旧行走轨迹，实时雷达可独立开启；
当前任务结束后隐藏旧规划路径。显示原生局部风险栅格，但其空白不代表可走。
待选点为黄色“待确认目标”，圆圈使用选定到点半径（旧圈固定 0.4 m），
保留候选目标高度；可用的原生预览路径显示黄色虚线，并给出距离。
以上仅更新显示，不重建/修改地图，不发送运动。

风险图层现场验证另发现 NX/Gateway 比 Windows 快约 2.20 秒，网页直接用
`Date.now()` 检查原生时间戳，超过 1 秒未来容差后错误隐藏新栅格。
现将 `/api/v1/state` 的时间戳与其浏览器接收时刻配对，按本地已流逝时间
估算 Gateway 当前时间。辅助请求/SSE 保活不更新这对时钟，断流后数据仍会
正常过期；未放宽风险栅格 2 秒时效或 1 秒未来容差。地图图例改为纵向排列，
避免与局部风险图例重叠。相关 39 项网页回归、构建和修改文件 lint 通过。
更新后的 NX 网页已实看：静态地图默认开启、实时雷达默认关闭，局部风险
正常显示；俯视点击显示黄色“待确认目标”和 1.2 m 距离，保持状态下
“开始导航”仍禁用。演示选点已取消，全程没有恢复控制或发送运动目标。

网页手动导航入口是 `http://127.0.0.1:15050/`。带 `?observe=1` 的地址
只读，不接收选点指令；已在只读顶栏增加“进入导航”链接。进入现场后，
确认当前地图与定位，点击“全图”查看保存地图或“俯视”查看机器人周围，
滚轮放大，点击地面选点，确认速度后点击“开始导航”。0.20 m/s 与 0.50 m/s
已加入选项。若显示“控制已暂停”，先由操作者点击提示旁的“恢复控制”，
接口只解除保持、不重放旧目标，随后重新选点。停止按钮仍保留在顶栏。

保存地图点颜色原先被深色 PointsMaterial 再次相乘而变暗，现改为中性色
乘数并加大保存地图点；不修改原始 PCD 或规划地图。全图按完整点云范围取景，
离群点也包含在内，图上空白不代表可通行。网页静态资源已在 `.19` 上单独更新，
没有重启原生进程；原 index 备份在 NX 验证目录的
`navigation-ui-previous-index.html`。相关 20 项网页回归、构建与修改文件 lint 通过。

地图为 `go2_expanded_20260914_0350`，原始文件保留在
`/var/lib/lingtu/maps/go2_expanded_20260914_0350/`。
用户已把机器人从建图结束处移回起点附近，因此旧终点初值配准失败，
后续全局定位回到约 `(-0.25,-0.09,0.02)`，用户确认位置变化。
不能把这次约 7.5 m 的坐标变化当成原地定位跳变。

实机使用 `.16` release、`nav` Product、SCAN 局部规划。
在线预览从当前位置到 `(-2.10,0.10,-0.10)`：1.934 m、3 个路径点、
12.48 ms。用户确认现场可立即停车后发送任务
`expanded-supervised-20260914-1789332003`，指令限速 0.20 m/s，
到点半径 0.30 m；约 29.2 s 后原生状态 REACHED/SUCCESS，
最终指令为零，运动 QUIET、停车 CONFIRMED，控制租约已释放。
用户反馈“已到点停稳，过程基本正常”。期间出现旋转和平移恢复状态，
这一次通过不等于所有障碍布局和长路线均已验收。

最终平移指令采样最大 0.200001 m/s（状态序列化误差量级），
SLAM 实测速率采样峰值 0.265 m/s。这里修复的是**指令上限传递**，
不宣称底层实测速度严格不超调；当前采样也不能代表连续峰值测量。
现场原始记录、速度曲线、摘要在
`build/go2-live-validation-20260913/expanded-motion-*`。

本轮修复：

- Gateway 原先把目标变成 PoseStamped 后丢掉 `max_speed_mps` 与
  `acceptance_radius_m`。现在目标服务、原生客户端、DDS、全局规划激活、
  重规划和执行器保留这些约束；最终命令仲裁在平滑之后再执行任务上限。
  新任务恢复 Product 默认上限。原生命令 ABI 从 8 升至 9，需整包更新。
- 图优化对接近水平姿态的秩 4 信息矩阵采用无主元消元，误拒绝合法半正定矩阵。
  改为对称对角主元消元。原图重放从第 4→5 帧的 `invalid_graph_information`
  前进到第 234→235 帧的 `insufficient_planar_correspondences`；
  后者仍需处理，**尚未完成闭环优化**，未降低匹配门槛或构造虚假边。
- Gateway 把导航事件设为 64 条批处理，巡检/探索事件设为 512 条批处理，
  但处理函数接收单条事件；因此机器人已到点，网页却显示 UNKNOWN。
  改为 `all` 逐条即时处理，单条 REACHED 能立即生成 SUCCESS。

验证：目标 Python 链路 178 项通过；NX 命令 DDS 实传和停止优先级 2 项、
规划激活/重规划 2 项、图优化 3 项通过；本地 CMU/SCAN 执行器限速 2 项通过；
Gateway 订阅与状态投影共 23 个不同用例通过。Python 全组首次出现取消超时
断言时序失败，单独复查和全组重跑均通过，保留间歇性测试记录。

保存图在 NX 无运动回放：短途、反向短途、回起点三条路径均可达，
12–166 ms，路径高度 -0.1..0.1 m，未绕到高处。导航仍需按现场障碍、
定位和局部碰撞图逐次验证，未完成闭环校正不能被“可激活”替代。

部署注意：ABI 8→9 更新时，新版 `lingtu_nav_control` 无法给旧 navd
发送停止请求。第一次安装已正式回退；随后用旧 release 的
`python -m lingtu.control stop` 确认退出，再安装 `.16`。
安装脚本须在 Go2 的 Python 3.10 虚拟环境 PATH 下运行，系统 Python 3.8
不支持当前 dataclass 的 `slots`。`.17` 追加网页事件修复后在无初值全局定位
阶段超过启动等待时间，安装器回退到 `.16`；明确停车后安装 `.18`，
以本次到点位置 `(-1.80551,0.029022,-0.019617,-2.972224)` 初始化，
导航再次就绪。后续应修复常规升级丢弃定位初值/重新全局搜索的问题，
不能把这次显式初值启动说成冷启动可靠性已验收。

命令消息格式更新还涉及 `lingtu_driver` 的停止请求发布者、
`lingtu_explore_dds` 的目标发布者以及原生 DDS 录制/回放工具。
这些必须与 navd/client 同步重编译，不能只替换导航程序；
`.19` 包含它们和上述网页事件修复。录制 CDR 回归通过。

最终 `.19` 已安装并通过 ProductControl 恢复 `nav + scan`，加载上述扩大地图，
定位 TRACKING、位姿新鲜、输入门控与 driver 均 ready，任务 IDLE、目标入口
ACCEPTING、运动 QUIET、最终三轴指令均为零。本轮没有再次发送运动目标。
仍存在 `deadline_miss_ratio_high`：最终 600 个循环样本中 48 个超过 10 ms
周期（8%），循环 p95 11.04 ms、最大 17.14 ms。不能把待命就绪或本次
短途导航通过说成循环负载问题已解决。只读状态保存在
`build/go2-live-validation-20260913/expanded-final-ready.json`。

### 2026-09-14 扩大范围地图已保存

用户告知建图完成后，回读遥控请求和最终输出均为零，经正式 Gateway/mapd
保存为 `go2_expanded_20260914_0350`，操作
`go2-expanded-save-20260914-0350` 为 SUCCEEDED。NX 路径为
`/var/lib/lingtu/maps/go2_expanded_20260914_0350/`，没有覆盖原走廊地图。

保存 PCD 为 23,740 点、278 个关键帧，OctoMap 约 310 KiB，occupancy、ESDF、
traversability 均已生成；元数据确认 `external_pcl_converter`、分辨率 0.2 m。
关键帧位置覆盖约 10.04 × 4.83 m，累计 XY 路径约 40.69 m；终点约
`(-6.838,3.746,-0.142)`，距起点约 7.80 m。这些是定位记录，非外部测量。

仍有地图质量限制：保存优化 `performed=false`、`sequential_chain_incomplete`，
`edge 4->5: sequential_registration_rejected: invalid_graph_information`，loop_count=0。
完整点云 Z=-9.62..4.48 m，存在远离主要表面的离群点，未擅自删除。
不能据保存成功或 can_activate=true 宣称闭环校正、地图质量或导航已验收。
当前保留 map 会话，尚未激活该新图为导航地图，也未发送自主运动目标。

本地点云、关键帧、优化报告和图位于
`build/go2-live-validation-20260913/expanded*`。
`expanded-map-topview.png` 仅显示 Z=-0.1..1.0 m 切片便于看通道；
`expanded-map-overview.png` 的侧视图展示完整高度范围，均不修改保存地图。

### 2026-09-14 03:42 扩大范围建图已启动

通过 ProductControl 切换 `map`，会话
`product-218b027699014b4fbee9f6ae6750e8fe`，继续使用 `.14` release。
Fast-LIO mapping/定位新鲜/地图可保存均已回读；DDS domain 0，驱动 ready，
控制循环 healthy，启动时最终输出为零。原走廊地图保留。

已打开 `start-wasd.ps1 -DomainId 0 -Speed 0.30 -TurnRate 0.25`，
随后确认 `go2-keyboard` 输入被接纳，用户输入的 -0.30 m/s 与 0.25 rad/s
到达原生最终指令；这不是实测速率验收。`teleop_local_planner=false`，
建图直接遥控不提供 SCAN 绕障，操作者负责绕开物体。
10 分钟只读诊断采集位于 NX
`/home/unitree/lingtu-validation-20260913/mapping-034208/samples.jsonl`；
诊断结束不会自动停止建图，也不会自动保存地图。

用户完成覆盖并回到起点附近停稳后，再保存新地图、检查起点/转角地面支撑、
定位连续性和正反向规划。后续自主运动前仍须修复上一轮目标限速传递问题。

### 2026-09-14 03:20 走廊实机运动：已到点停车，限速与状态仍待修复

用户在机器人旁允许测试。使用 `.14` release 与
`go2_corridor_fixed_20260914`，通过 Gateway `/api/v1/navigate/click`
提交地图目标 `(1.268291,-0.195619,0.034089)`。目标所在地图位于 NX
`/var/lib/lingtu/maps/go2_corridor_fixed_20260914/`。

- 起点约 `(3.411,-0.369,0.051)`，定位记录位移约 2.15 m；约 19 s 后
  原生导航报告 `SUCCESS / REACHED`，随后 `QUIET / CONFIRMED`，测试控制租约已释放。
  终点定位约 `(1.269,-0.195,0.028)`；这只是定位内的误差，不是外部测量精度。
- 约第 6–19 s 进入过 `RECOVERING / BLOCKED`，还不能宣称连续平顺通过。
- 请求 `max_speed_mps=0.2`、`acceptance_radius_m=0.3` 被回执回显，但
  `ConstructedGoal.pose_stamped()` 仅传位置/朝向，未把这两个约束送进执行链路。
  本次 SLAM 平面估计速度峰值约 0.745 m/s；**低速验收未通过，暂停追加运动测试**。
  后续必须修复目标约束传递并验证最终输出，不能仅依据回执宣称限速生效。
- 底层已到点，而 Gateway 任务状态仍为 `UNKNOWN / task_status_unavailable`，
  中途还出现状态过期。成功任务的清理取消返回终态冲突；机器人保持停车。
- 开始两次测试脚本误将 task_id 与 request_id 设为同值，被接口拒绝且未运动；
  已修正为独立请求编号。上述运动记录来自修正后的单次有效目标。

证据：`build/go2-live-validation-20260913/corridor-motion-trial.jsonl`、
`corridor-motion-console.txt`、`corridor-motion-summary.json`、`corridor-motion-map.png`。
现场是否碰撞、是否明显卡顿仍需操作者反馈；本条更新优先于下面“尚未运动验收”的历史记录。

### 2026-09-14 走廊路径修复已部署：在线规划通过，尚未运动验收

用户确认原 4 m 路段全部为平地，两侧有物体。本轮修复两个进一步定位的问题：

- 全局 footprint 支撑查询将 0.43 m 半径在 0.2 m 栅格上向上取整为 0.6 m，
  查询到了物理包络边缘之外的格子。改为查询半径边缘所在格（本例偏移 0.4 m），
  不改机器人半径或机身碰撞包络。NX 新回归先复现失败，再验证正常地面通过、
  实际边缘缺支撑仍拒绝；地图替换按新快照对象测试，避免同一对象缓存影响测试。
- 原始 PCD 在 `(2.1,-0.1,0.3)`、`(1.3,-0.5,0.3)` 没有占据，转换器却把
  相邻物体顶部扩展到这些位置，下方原本都有 z=-0.3 m 地面。支撑扩展现在检查
  原始目标列：已有更低超过 1 格的占据表面时，不额外生成上层支撑。原始障碍
  及上层地面点均保留。新增台面/低地面场景在旧实现失败，修复后通过。

真实地图仍保留 14,390 个原始占据格（14,842 点），扩展后 15,830 格。修正后，
原参数正反向均沿走廊规划成功，约 28–29 ms，路径 z 为 -0.1..0.1 m；不再走
之前升至约 0.9 m 的侧面路线。另做的 body-height=0.3 m/cylinder 和无台阶参数
组合未通过，仅属离线诊断，未采用为实机标定。0.25 m 同层高度变化限制下正反
回放也通过，但该选项不在当前 ProductControl `--set` 参数表中，**未写入在线
RunPlan**；在线仍保留原全局参数，不能声称已限制所有后续目标的攀爬。

验证：NX 支撑/栅格/台阶/邻域三项回归通过（0.86 s），转换器两项通过（0.14 s）；
Windows 对应五项通过（2.96 s）。实机 `navd` 已重新链接全局规划库。
通过现有原生 release 安装器安装 `v2.3.0-go2.20260914.14`，替换 `navd` 与
`octoplanner3d_pcd_to_octomap` 及其兼容构建路径；其他二进制沿用 `.13`。
原始地图 `go2_bidirectional_20260914_0044` 未修改，复制后以正式
`lingtu-mapctl build --build-mode external_pcl_converter` 生成新地图
`go2_corridor_fixed_20260914`。必须指定 build-mode；单独传 converter 不会切换
mapctl 默认的内置构建模式。新副本首次默认构建结果已在尚未激活时重建替换。

随后通过唯一生命周期入口恢复：

```bash
"$LINGTU_PYTHON" -m lingtu.control switch nav --robot unitree/go2 --env real \
  --map go2_corridor_fixed_20260914 --local-planner scan \
  --set scan_planner.max_acceleration_mps2=1.0 --json
```

ProductControl 返回 active，session `product-1650ccb40f9741908c7c755e786c5a91`，
包含 map_committed、goal_acceptance_ready。Fast-LIO TRACKING，重定位完成，
对保存地图连续跟踪成功，无 LiDAR/IMU 丢帧；观测时 IMU 约 200 Hz、雷达约 10 Hz。
已恢复电脑 `http://127.0.0.1:15050/?observe=1` 只读监控隧道。

在线只读 `validate_plan`：起点约 `(3.398,-0.372,0.056)`，目标
`(1.268291,-0.195619,0.034089)`；返回 `ok=true, feasible=true`，5 点、
2.585 m、29.76 ms，`source=native_nav`、`motion_published=false`。
这证明已部署地图的在线全局规划通过；**未下发自动目标，局部轨迹执行、真实绕障
及到点停车仍须现场运动验收**。页面中的 mode=navigating 是 Product 模式，
本轮观测时实际任务 ID 为空，执行状态 IDLE。

证据仍在 `build/go2-live-validation-20260913/`：`corridor-live-preview.json`、
`corridor-live-state.json`、`corridor-forward-plan.json`、`corridor-reverse-plan.json`、
`footprint-before-test.txt`、`shelf-before-test.txt`、`route-hit-audit.txt`。
NX 对应目录另有 `corridor-switch.json`、`corridor-external-build.json`、
`route-release-install.txt`。本段覆盖下方早期“正式服务未恢复/原路线无路径”的状态。

### 2026-09-14 重启后 NX 静态验证：修复支撑误判，完整路径仍未通过

本轮 NX SSH 已恢复。检查时只有 `lingtu-go2-compute.service` 处于 active/exited，
LingTu Product 服务未运行，5050 无响应；`/opt/lingtu/current` 仍指向
`v2.3.0-go2.20260913.13`。本轮在 NX 源码构建目录编译与验证转换器，未替换正式
release、激活旧地图或发送电机运动指令。网页此时没有实时数据不能解释为导航已就绪。

首先在 aarch64 复现旧支撑算法缺陷：两面相隔 0.8 m 的竖直墙被共同判为水平
支撑（诊断样本 286 个支撑种子）；新增的平行墙测试在 PCL/no-PCL 两个入口均失败。
真实地图 `go2_bidirectional_20260914_0044` 的目标列 `(1.268291,-0.195619)`
在 z=-0.1..0.7 m 没有原始占据点，但邻近障碍误判为支撑后，1 格扩展填入了这些格子。
因此先前“目标机身高度有占据”不能全部归因于雷达、残影或真实物体。

已修正转换入口的 `horizontalSupportKeys`：排除上方紧邻占据格的内部点，只在
直接相邻 XY 格中检查露出的表面，允许 1 格高度差，至少满足三个方向。不再跨
2–4 格并带横向容差寻找支撑。没有满足支撑条件的原始点仍保留为占据，不删除障碍。
平行墙诊断支撑种子降为 0；真实目标列上述新增占据消失，地面 z=-0.3 m 保留。

NX 两项 CTest 通过（约 0.12 s），覆盖 ASCII/binary/no-PCL、扩展 0/1 格、单点
体素地面、细小障碍、自由包络中的障碍保留、独立/平行墙、台阶和空结果拒绝。
同一补丁的 Windows/PCL 构建与两项 CTest 也通过（约 1.87 s）。
真实输入 14,842 点对应 14,390 个原始占据格；旧支撑扩展后 28,748 格，新规则
扩展后 18,455 格。减少的是错误/不再满足支撑条件的扩展格，不是再次过滤原始点。

**仍未通过完整路径回放**：NX 用原实机规划参数从 `(3.373184,-0.373739,0.049748)`
到 `(1.268291,-0.195619,0.034089)`，起终点吸附改善为 `(3.5,-0.5,0.1)` 与
`(1.1,-0.1,0.1)`，但仍 `empty_path`（约 182 ms）。基础阶段 553 次迭代，后备阶段
1529 次迭代，均耗尽可达节点；仍有地面支撑/机身占据/运动边检查拒绝。
停车位置 y 负方向约 0.6 m 的检查列确有原始占据，不能全部移除。
这次结果证明转换误扩展的修复，不证明完整地图可导航或现场运动通过。

本地证据：`build/go2-live-validation-20260913/` 中 `support-before-tests.txt`、
`support-after-tests.txt`、`converter-audit*.txt`、`bidirectional-arm-fixed.ot`、
`bidirectional-arm-plan-after.json` 与 `.log`。NX 同名证据在
`/home/unitree/lingtu-validation-20260913/`。正式部署和 Product 恢复尚未执行。

### 2026-09-14 正反向建图：漏扫改善，发现并本地修复空 OctoMap

用户完成正反向运动后，确认停稳并关闭 WASD，保存新地图
`go2_bidirectional_20260914_0044`（操作 `go2-bidirectional-20260914-0044`
SUCCEEDED）。地图含 14,842 个有限点、113 个 patch 姿态，展开航向范围
约 -13.18°..196.65°，这次确实转向反方向。
以半宽 0.3 m、z=-0.5..-0.1 m 检查，原起点 `(0,0)` 有 11 个地面候选点，
停车点 `(3.373,-0.374)` 有 13 个，路段 `(1.268,-0.196)` 有 9 个。
这证明原先的地面漏扫已有改善，不等于完整机身范围均满足支撑约束。

加载 nav/SCAN 时失败：`native map stage rejected: artifact_gate_failed:octomap
artifact is empty or unreadable`，ProductControl 报告 rollback_failed。随后 NX
断开，用户确认已关机或拔线。没有提交自动运动目标；再次上线需恢复 Product，
不能沿用此前服务 ready 的状态。

已在 NX 用部署转换器独立复现：14,842 点 -> 每体素至少 3 点过滤后 38 体素 ->
最小 4 体素连通簇过滤后 0 体素，仍退出成功并写出 129 字节、size=0 的 OctoMap。
SLAM 保存地图已降采样，原始点数密度不能再当作观测置信度。
已修改 `src/nav/cpp/planning/global/octoplanner/pcd_to_octomap.cpp`：ASCII/PCL
输入共用保留已采样占据体素的构建流程，去除第二次点数/连通簇过滤；仅从支撑
表面生成上方自由空间，无有效占据体素时报错。保存清理继续由原有 pruning 负责。

本地 Windows/PCL 编译通过；两项 CTest 回归通过，覆盖 ASCII/binary/no-PCL
读取、单点体素地面、细小障碍保留、非支撑点不生成自由空间、空结果拒绝。
用本轮真实点云重建得到 28,748 个占据体素（含原配置的支撑扩展），已不为空。
修复尚未部署 NX，尚无 aarch64 新版本编译或实机运动验收证据。

**完整路径仍未通过本地回放。** 使用现有实机参数，从
`(3.373184,-0.373739,0.049748)` 到走过的 `(1.268291,-0.195619,0.034089)`，
起点/目标分别吸附到 `(3.3,0.1,0.5)`、`(1.5,-0.5,0.3)` 后 A* 无路径。
直接栅格查询显示原停车点中心列已有 z=-0.3 m 支撑，但完整 footprint 支撑检查
仍失败；目标列还存在 z=-0.1..0.5 m 占据点。需继续区分现场物体、点云残影和
支撑取样问题，不删除这些点或放宽安全约束来强行通过。
证据位于 `build/go2-live-validation-20260913/bidirectional*`。

本轮全局图优化也未完成：`edge 8->9: sequential_registration_rejected:
invalid_graph_information`，该问题单独保留，未作为空 OctoMap 的原因。

### 2026-09-14 地面补扫已保存，路径预览仍失败

用户告知「补扫完成」后，确认请求、最终指令和遥测速度为零，关闭本轮 WASD，
另存为 `go2_ground_rescan_20260914_0035`；操作
`go2-ground-rescan-20260914-0035` 为 SUCCEEDED。保存点云 13,563 点、137 个
patch 姿态，轨迹 x=-0.233..3.976 m、y=-0.776..0.433 m，航向约 -23.41°..17.71°。
原起点 `(0,0)` 和当前停车位置附近 `(0.7102,-0.1438)`，半宽 0.3 m、
z=-0.5..-0.1 m 区域的保存地面点仍均为 0。遥控采集中确有 Q/E 对应的
±0.25 rad/s 请求及最终输出，但未形成面向来路的转身；不能表述为完全没有转向输入。
用户随后确认「主要前后移动，还没有转身面向起点」，与本轮航向记录一致。
下一轮采用人工走开后单独 Q/E 转身、面向起点停留补扫的步骤，不据此认定 Q/E 故障。
已重新通过 ProductControl 启动 map 会话
`product-4b25e6fdc4a14ce4b30bb544cd86372f`，回读 mapping ready、保存支持、
输入 ready、teleop_local_planner=false、最终零指令；重新打开 0.5 m/s、
0.25 rad/s WASD 窗口。该新一轮仍待操作者补扫完成，未保存或验收。

随后加载新地图进入 nav/SCAN，保留 max_acceleration_mps2=1.0，会话
`product-0ae1a5ee29f94d46977268332b24d27e`，重定位 completed。
对实测走过的 `(2.133366,-0.377192,0.017104)` 进行无运动路径预览：
起点约 `(0.745882,-0.117709,-0.016275)`，feasible=false，
`start_snap_exhausted`，23.16 ms，motion_published=false。
当前仍未通过自主导航验收，未发送自动运动目标。
本次 map_optimization performed=false，`sequential_chain_incomplete`，
`edge 17->18: sequential_registration_rejected: invalid_graph_information`，
该图优化问题尚未修复，不能混同于起点实际缺少地面观测。

用户要求开始录制可用于导航的地图，已通过 ProductControl 切换正式 `map`，
会话 `product-b5cf18f008b54a6ca0178df335f0832b`。回读 mapping/SLAM ready、
map_save_supported=true、teleop_local_planner=false、driver command_accepted、
控制循环 healthy；旧地图 `go2_walk_20260913_2346` 保留。新 WASD 窗口为
0.5 m/s、0.25 rad/s，已收到零输入保活，最终指令全零，尚未确认本轮人工运动。
这轮仍使用 MID-360，不启用内置雷达融合。正式 map 为直接遥控，不提供 SCAN 绕障。
NX 10 分钟只读采集位于 `/home/unitree/lingtu-validation-20260913/mapping-003215`。
操作者应离开起点后转身面向原起点补扫，并换朝向覆盖预定路线；完成后停稳告知，
再另存地图、检查地面覆盖和原失败类型的路径预览。结果见本节开头；导航仍未验收。
后续切换 nav/SCAN 时仍使用此前用户确认的 `scan_planner.max_acceleration_mps2=1.0`。

### 2026-09-14 内置雷达补地面：接入验证，尚未启用融合

用户认为嘴部雷达为 L2。NX 原生 Unitree SDK2 只读采集已确认
`rt/utlidar/cloud`、`rt/utlidar/cloud_base`、`rt/utlidar/cloud_deskewed`
均有真实 PointCloud2 样本，12 秒采集各 184 帧，约 15.36 Hz；对应 frame 分别为
`utlidar_lidar`、`base_link`、官方 `odom`。LidarState 的 error=0、
software=`1.0.0.38`，firmware/sdk 字段为空，不能据此确认 L1/L2 型号。
工具为 [`tools/diagnostics/go2_lidar`](../../../../tools/diagnostics/go2_lidar/README.md)，
使用已安装的 Unitree SDK2，不增加正式运行的 ROS2 依赖、不发布运动或地图消息。

机身坐标点云的初步单帧检查，在 |x|、|y|<0.6 m、z=-0.5..-0.1 m 内有
344 个近地候选点；|x|、|y|<0.3 m 内有 8 个，最近候选点水平距离约 0.323 m。
这是按高度筛选的观测，尚不是经过外参验证的地面分类，也不能证明原缺口中心已覆盖。
内置雷达有补充价值，但机身遮挡依然存在。原始帧和诊断数据位于 NX
`/home/unitree/lingtu-validation-20260913/builtin-lidar`、`builtin-native-01`。

**当前融合阻塞项是时间基准与外参验收。** `builtin-native-clock-01` 中，NX
接收时间减内置点云 header 时间为：cloud 5.1327 s、cloud_base 5.1340 s、
cloud_deskewed 5.1046 s。两次 MID-360/内置快照对比也相差约 5 s。
NX `timedatectl` 回读 NTP=yes、NTPSynchronized=no。这些值包含设备时钟差和传输/
处理延迟，不能直接称为点云缓存了 5 秒，也不能直接减去固定 5 秒当作同步。
未修改系统时钟、生产外参或地图。需确认发布端时钟与 NX 的同步关系，并验证运动
中的扫描时刻；否则会用错误时刻的机身位姿放置补充点云。

静止候选地面拟合（高度筛选、0.65–2.5 m 水平范围、3 cm 平面残差筛选）中，
两路平面在各自机身原点的高度相差约 1–2 cm；两路采集时间未对齐，不视为标定。
此前对不同采样范围取整体 z 中位数得到约 8 cm 差值，不能据此修改外参。

融合实施边界：

1. 保留 MID-360/Fast-LIO 的统一位姿与重定位来源。官方 `cloud_deskewed` 的
   `odom` 不等于 LingTu 的 `odom`，不能直接混入。
2. 校准内置雷达与 LingTu body 的变换并核对扫描时刻；验证地板、墙面重合及
   机身自返回过滤后，才允许写入地图。单个地面平面不能确定完整六自由度外参。
3. `MapObservation` 当前每包只有一个射线原点，mapd 也使用统一的 epoch/sequence
   和一个最新样本槽。不能让第二个独立发布者直接写 `/slam/map_observation`，
   也不能把两颗雷达的点简单拼接后共用 MID-360 原点进行清空。
4. 保存链路 `SaveCoordinator` 请求 SLAM 快照，Fast-LIO `saveMap` 保存自己的地图
   与 patches。只补 mapd 实时图不会自动补进保存地图；补充观测必须接入同一保存
   事务和地图坐标，并通过保存后端点支撑/路径预览验收。

不融合的现场修复：保留旧地图，通过 ProductControl 进入正式 `map`，在操作者
控制下离开起点约 1.5–2 m、转身面向原起点，从多个角度补扫地面，覆盖预定路线
后另存新地图。应按实际地面点覆盖验收，不能仅按走过路线或定位 ready 验收。
原地看向同一个方向或保持朝向倒退不能保证补到脚下地面。补扫前先确认实际地形；
没有观测的区域不直接填成平地，不关闭支撑检查。保存后重新预览原失败端点，
通过后才进行现场监督的自主导航。目前未向实机发送自动补扫或导航运动指令。

本次 MID-360 可见性回放：以当前标称 +13° 安装外参、145 个保存姿态，以及
起点地面 z=-0.25/-0.30/-0.35 m 假设计算，缺口中心 `(0.475,-0.291)` 的
FOV/距离合格姿态均为 0；对照目标 `(1.61,-0.23)` 为 85–87 个。记录航向仅
约 -33.93°..16.55°。这是未建模遮挡、未经现场外参标定的几何推算。
MID-360 水平 360°、垂直 -7°..52°，并非能看到脚下所有地面。
[Livox 官方规格](https://www.livoxtech.com/mid-360/specs)。

本轮地图已于用户告知「建图好了」后保存为 `go2_walk_20260913_2346`，保存操作
`go2-walk-20260913-2346-save` 返回 SUCCEEDED。保存点云 16,738 点，全部坐标有限，
OctoMap、occupancy、traversability 等产物已生成。采集前 163 帧（约 163 秒）无读取
错误且定位均报告 ready；轨迹范围约 x=-0.006..6.862、y=-1.864..0.016 m。
这不等于地图几何质量或实际导航通过。

已加载该地图进入 `nav + SCAN`，会话 `product-995e8f5232c34cf5bb995a5833419c07`，
保留 `scan_planner.max_acceleration_mps2=1.0`，初值来自建图停稳后的实时位姿：
`--initial-pose 0.433112 -0.309277 0.028601 -0.0527642612`。
重定位 completed、精配准 3186 个内点、输入 ready、目标 ACCEPTING，当前没有导航
目标、最终零输出。旧 WASD 已关闭。

**首条自主导航尚未通过无运动预览，未发送运动目标。** 对刚才走过的两个目标
`(1.609908,-0.230175,0.006913)` 和 `(1.667665,-0.753757,0.055802)`，NX 预览均返回
`feasible=false / goal_not_reached`。同一 OctoMap、相同规划参数的本地 C++ 回放显示：
起点约 `(0.475,-0.291,-0.018)` 吸附到 `(0.9,0.9,-0.1)`，第一目标吸附到
`(1.1,0.9,0.1)`，目标误差 1.243 m，超过 0.5 m 容差。未放大容差、未关闭支撑检查。

原生栅格查询将两个原始端点归因为 GroundSupport；起点正下方的 z=-0.7..0.3 m
列均 unknown，目标中心 z=-0.3 m 有占据支撑，但周边机身范围的支撑不足。
机器人半径 0.43 m、栅格 0.2 m 下，现有五点支撑检查的周边采样使用 ceil，距中心
0.6 m；这与半径值不同，尚未改动该取样规则。
进一步在 NX 上逐帧转换保存的 patches/poses，检查两个端点附近半宽 0.3 m、
z=-0.5..-0.1 m 的区域：起点 patch 点 0/帧 0/保存点 0；目标 patch 点 1180/帧 87/
保存点 13。patches 是 Fast-LIO 预处理/去畸变后的逐帧记录，并非未经处理的雷达报文。
因此起点缺地面在保存清理之前已存在，不能归咎于保存后的清理或 OctoMap 转换；
仍需确认现场地形以及雷达覆盖/进入 patches 前的过滤。

保存过程的 `map_optimization.json` 另记录 performed=false、
`sequential_chain_incomplete`，原因是 `edge 4->5: sequential_registration_rejected:
reverse_insufficient_planar_correspondences`。原始地图已保存，不能称为完成了全局图优化。
下一步先核对/补齐地面观测，再重复端点和完整路径预览，通过后才进行现场监督运动。

用户反馈 SCAN 加速度从 0.5 提至 1.0 m/s² 后顺畅很多，随后要求开始建图与自主导航。
该反馈是现场主观改善，不表示所有安全停车或端到端延迟已通过验收。
已通过 ProductControl 切换正式 `map`，会话
`product-7903c98f11b649838bcf72df4f54e863`；回读 native_control_mode=teleop、
teleop_local_planner=false、Fast-LIO mapping ready、地图保存能力 true、DDS domain=0、
输入 ready、driver command_accepted、循环 healthy、初始最终零输出。
重新打开 WASD（速度上限 0.5 m/s，转速 0.25 rad/s）。正式 map 是直接遥控，
不提供 SCAN 辅助绕障，需要操作者主动避开障碍。

本次验收原定按以下顺序推进；地图已保存，但自主导航目标尚未提交：

1. 操作者覆盖一个房间或一段走廊，经过门口、转角和通道后回到起点附近停稳。
2. 告知「建图完成」后，检查采集中的定位连续性，保存新地图并检查点云与导航产物。
3. 用同一地图切换 `nav --local-planner scan --set scan_planner.max_acceleration_mps2=1.0`，
   核对定位、路径预览和目标准入，不将 mapping 的 ready 当作导航就绪。
4. 现场监督下先验收短距离空地到点停车，再验收固定障碍绕行、执行中换目标。

本轮启动 10 分钟、1 Hz 的只读定位与控制状态采集；采集不发送运动命令。
原有地图保留，本轮地图已另存为本节开头记录的新地图。

## 实时安全范围显示（本地 Web 预览）

2026-09-13 加速度对比试验：用户要求提高加速度，通过已部署环境的
`python -m lingtu.control switch teleop_avoid --robot unitree/go2 --env real --local-planner scan --set scan_planner.max_acceleration_mps2=1.0 --json`
建立会话 `product-aecda0a71edb447e8a8118d1d7eeb7dc`，没有修改 Product 默认值。
回读当前 RunPlan 确认 SCAN 加速度 1.0 m/s²（此前 0.5），遥控速度上限仍为
0.5 m/s，`LINGTU_NAV_SMOOTHER_ENABLED=0`。这也纠正下文的通用链路说明：当前
会话没有启用速度平滑器，不能把本次顿挫归因于平滑器的缓升速；最终安全停车和
SCAN 重新起步仍可能造成停顿。六项服务 active，输入 ready、driver command_accepted、
控制循环 healthy、最终零输出后，重新打开 0.5 m/s、0.25 rad/s 的 WASD 窗口。
这是待现场对比的会话参数，不代表运动验收通过；恢复时用同一 ProductControl
命令将 `--set scan_planner.max_acceleration_mps2` 改回 `0.5`，不能直接改已发布 RunPlan。

起步延迟待验收：2026-09-13 NX 的 `start-delay-231458/samples.jsonl` 连续采集
75 秒、375 帧，接口读取无错误，但未收到非零遥控请求，因此不能据此给出起步延迟。
采集包含 nav 路径/跟踪/最终指令、driver 回执及 Fast-LIO 速度，采样周期 0.2 秒；
这些是状态快照，并非同一时刻的同步硬件测量。代码中没有路径生成后固定等待几秒的
定时器，朝向对齐、速度重规划和最终碰撞检查仍可阻止平移。此前保留的
`scan_actual_motion_blocked` 只能证明发生过最终拦截，不能代替本次起步复现。

2026-09-13 继续核对「出路径后顿一下」：当前 RunPlan 的 SCAN 轨迹加速度上限为
0.5 m/s²，通用路径跟踪器的上限为 1.0 m/s²；两者不是同一层参数，也不是实测
机身加速度。按恒定 0.5 m/s² 从零升至 0.5 m/s 约需 1 秒，但这不等于先等待
1 秒再运动。普通重规划由 SCAN Backend 保留已提交轨迹；Spline follower 换入
新轨迹后直接读取其起点速度，重新计时本身不要求零速度。
最终制动碰撞检查拒绝候选时，`FinalControl` 会停车并清零速度平滑器，随后
teleop 请求重规划；这条「拦截 → 停车 → 重规划 → 重新起步」链路会造成顿挫。
当天记录中存在 `scan_actual_motion_blocked` 和 `collision_at_trajectory_start`，
但尚未证明每次停顿都来自它们。第二轮 `start-delay-232324/samples.jsonl`
同样为 75 秒、375 帧、无读取错误、全零遥控输入，仍不能给出运动延迟结论。
验收需对齐同一次动作的轨迹 ID、跟踪候选、最终安全判定、驱动回执和定位速度；
确认连续非零指令下只是提速缓慢后，才调整加速度，不能用提速掩盖反复停车。
本地当前代码的跟踪器 4 项、执行器 3 项、规划器 3 项针对性回归通过，覆盖新轨迹首拍非零输出、
暂停计时、朝向对齐、安全前缀续行、持续低速输入和降速重规划。空地图、理想运动
积分的 0.2 m/s 用例初始化约 66 ms，开始跟踪后 0.5 秒输出约 0.161 m/s、1 秒
约 0.190 m/s，4 秒内未重新启动轨迹；这不是 Go2 实测响应。
规划器用例另验证重规划保留旧轨迹，以及非零起始速度、加速度的边界保持。
本次 MSVC 构建成功，但仍有上游类型转换、未使用参数及 L-BFGS 的 C4701/C4702
告警；未改动这些源文件，也未将构建成功表述为无告警。

若网页能打开但显示「实时数据连接中断」，先区分本地 Vite（15173）与 NX 监控隧道
（15050）：15173 正常而数据 API 返回 502、15050 无监听，表示监控传输不可用，
不能据此认定机器人关机。2026-09-13 曾在电脑接入 `hongsen`、获得 `172.20.10.3`
时复现；原 Sunrise `192.168.66.65` 的 SSH 超时。应恢复可到达 Sunrise 的网络，或确认
其新地址后修改本机 `~/.ssh/lingtu-go2.conf`，再运行 `start-monitor.ps1 -NoBrowser`。
此脚本只恢复只读隧道，不启动运动。

默认使用「机身高度切片」，只保留与当前机身中心高度相交的抽样膨胀格，避免不同高度的
格子在透视下叠加遮挡；「图层 → 安全范围 → 三维诊断」可以恢复所有高度及诊断标记。
尺寸和解释默认折叠。切片仍来自原接口的 640 个样本，**并非完整局部切片**，也没有改变
后端按占据排序抽样所导致的帧间跳变；不能据此判定空闲通道。切片过滤、三维恢复及
已有几何/时效回归共 5 项通过，Web 构建通过，已在实时预览核对。

2026-09-13 的本地 Web 已支持在「局部安全诊断（采样）」图层显示蓝色双圆柱包络、
白色前后查询点和红色膨胀占据格，并提供「俯视」按钮。包络尺寸读取当前 RunPlan，
跟随与机器狗模型相同的插值位姿；格子使用 native 输出的 map 坐标与实际分辨率，
不会再膨胀一次。当前 Go2 为半径 0.25 m、前后偏移 ±0.18 m、上下各 0.10 m。

占据数据来自 `/api/v1/navigation/dds_snapshot` 的 `local_map.collision`。
当前接口只返回 640 个抽样占据格（完整图约 46 万占据格），**空白不能作为自由空间证据**；
半径已计入占据格，蓝色包络与红格重叠也不能当成第二次碰撞判定。
不展示过期、非 map 坐标或不完整的碰撞图。此项属于只读可视化，不构成避障运动验收。

本地预览使用 `ROBOT_HOST=127.0.0.1:15050` 启动 Vite，访问
`http://127.0.0.1:15173/?observe=1`；15050 仍是 NX 监控隧道。
本次未替换 NX 的 `.13` 发布包。几何/坐标/数据时效 4 项回归及 Web 构建通过。

## `.13` 已部署：遥控主动转向同步引导目标

2026-09-13 重启后已发布 `v2.3.0-go2.20260913.13`，运行代码提交 `d392c14`。
NX Release 编译及 18 项直接相关回归通过，经 ProductControl 启动
`teleop_avoid + SCAN`，会话为 `product-f126f5e9994f4de4b0c8ee15272893e8`。
45.17 秒零输入采集共 223 帧，全部 input gate ready、控制循环 healthy；
导航和驱动非零输出均为 0。定位 age p95 为 0.102 s、碰撞地图 age p95
为 0.141 s，地图处理增加 420 帧、替换 0 帧、队列最多 1，驱动持续就绪。
原厂避障关闭已由驱动日志确认。已打开 0.5 m/s、0.25 rad/s WASD 窗口，
运动验收结果待现场反馈；上述零输入检查不能证明运动流畅或绕障成功。
NX 证据目录：`/home/unitree/lingtu-validation-20260913/steering-guide/`。

随后用户实际操作了 WASD。300 秒只读采集共 1,499 帧，其中非零请求 760 帧，
这 760 帧的控制循环全部 healthy，但有 233 帧最终输出为零；遥控 reason 中
`scan_initialization_failed` 169 帧、`scan_actual_motion_blocked` 33 帧、
`local_intent_pending` 20 帧、`odom_stale` 1 帧。计数是 5 Hz 抽样帧数，
不是独立事件数，也不能把有障碍时的正常停车全部算作故障。
保存的 67 份失败快照中：轨迹起点碰撞 42 份、时间调整后碰撞 12 份、
动态约束仍不满足 7 份、碰撞重启次数耗尽 6 份。
因此本轮尚未通过连续运动/绕障验收，不能将问题归因于控制循环负载，
也不能把转向引导修复等同于所有规划失败已修复。转向释放后的主观手感仍待反馈。
数据在上述目录的 `operator-trial/motion.jsonl`、`analysis.json` 和
`failure-*.json`；驱动状态更新慢于采集，实际速度与规划状态并非同步采样。

用户随后报告“看着有路却不走、没有自主脱困、延迟严重”。进一步核对：
42 份 `collision_at_trajectory_start` 中有 31 份碰撞时间恰为 0；该 reason
在源码中涵盖第一个样条时间间隔，不能把其余 11 份也解释为当前机器人起点占据。
`failure-001492.json` 按 Grid 的坐标变换和索引公式查询，前部圆柱中心落在
占据格 `[102,102,48]`，后部 `[94,102,48]` 为空闲。沿机身后方 0.05、0.10、
0.35 m 的前后查询点抽样均空闲；右侧 0.35 m 终点空闲，但途中 0.05、0.10 m
仍有前部占据。这里只是点查询，不是完整扫掠轨迹或实物净空验收。

当前 Executor 对 SCAN 排除了通用旋转恢复；Product 的 `recovery.max_attempts`
也为 0。仍存在的边界退出仅沿操作者输入方向尝试 0.35 m、速度不超过 0.15 m/s，
不会自主搜索后退/其他侧向出口。Grid 的边界退出只豁免最初所在格，后续占据格
仍拒绝，因此“较远终点空闲”并不足以触发退出。不能仅把 max_attempts 改成 1
就宣称 SCAN 已具备自动脱困，更不能跳过实际几何碰撞检查。

同轮 760 个非零请求采样的导航端输入 age p50=0.011136 s、p95=0.040265 s、
max=0.041467 s。它测的是导航端收到输入后的年龄，不是按键到机身运动延迟。
结合循环全部 healthy，重复规划拒绝/零输出/重新起步是现有证据支持的顿挫来源；
仍需分别核对占据真实性、规划耗时和实际速度响应，不能断言网络绝无延迟。

2026-09-13 本地回归复现了 `teleop_avoid` 的共享目标生成问题：平移期间
主动输入 Q/E，Follower 响应转向，但参考线仍锁定旧方向；松开 Q/E 后
SCAN 还会恢复旧朝向。测试中机身已转过 0.12 rad，释放转向输入时旧实现
输出 -0.18 rad/s，形成回拉。这是本地控制输出证据，不是新的实机测量。

Executor 现按操作者主动转向更新参考：相对上一次引导方向转过 10° 时，
从当前位姿重建 3.5 m 引导；释放转向输入时补齐最后不足 10° 的转角，
保持新的朝向。此类更新递增参考代数，但不重置 Follower 或局部规划器；
已有轨迹仍须通过原有可行性与碰撞检查。没有转向输入时，继续保留稳定参考，
避免规划器自身的绕障转向不断带偏目标。平移方向改变仍按原逻辑处理。

新增两个回归先在旧实现失败、修复后通过，覆盖 SCAN/CMU 共享引导以及
SCAN 释放转向后的最终角速度。另有 16 项直接相关回归通过，覆盖连续低速、
平移换向、侧移朝向、稳定参考、绕障回归和碰撞停车。Release 构建通过；
上游 `lbfgs.hpp` 仍有原有 C4702/C4701 编译告警，本次未修改该优化器。
此修复已部署，尚未完成实机运动验收；
它不代表短绕障目标末速度为零、感知占据或全部卡顿问题已解决。

## `.13` 已生效：Go2 竖直范围上下各 0.1 m

用户要求将竖直膨胀与官方默认对齐，已将本地 `robot.yaml` 的
`collision_clearance_below`、`collision_clearance_above` 都改为 0.10。
由既有 RobotConfig → native_nav → mapd 映射统一下发；没有在进程启动脚本中
增加第二份参数。此项也会同步影响使用同一净空参数的导航碰撞约束。
本地配置/驱动契约 43 项及真实 RunPlan 编译用例 1 项通过；显式不对称净空的
映射用例仍验证“机身下方净空映射为障碍向上膨胀、上方反之”。
此前 NX 连接中断时未部署；本次 `.13` 重新发布并经 ProductControl 启动后，
已回读新 RunPlan 的导航上下净空均为 0.1，mapd 的向上、向下膨胀也均为 0.1。
没有直接修改旧 `.12` RunPlan 或在启动脚本内覆盖 RobotConfig。

`teleop` 是直接速度控制；以下参考目标逻辑属于 `teleop_avoid`：
按键生成机身坐标 vx/vy，Executor 按当前朝向建立长度 3.5 m 的参考线；
输入方向变化超过 10° 重建参考，同方向参考目标移动累计 0.875 m 后更新。
遇阻先试前进短目标（距离依次减半，常规下限 0.5 m），再试左右各
30°/60°/90°侧目标，必要时补 0.4/0.3/0.2 m 短侧目标。
候选按顺序选择，使用膨胀栅格直线段检查；选定后通常保留至接近或失效。
绕行目标末端速度为零，再由带当前运动边界条件的 B-spline 优化判定可执行性。
因此目标几何上可达不保证当前速度下轨迹可行；频繁选短目标和零末速可能影响
连续性，但不能据此断言全部卡顿都由目标点造成，仍需区分地图占据与执行跟踪。

## 2026-09-13 `.12` 局部范围与卡顿核对

随后用户请求前方目标点绕障测试。只读构造前方 2 m、速度上限 0.5 m/s 的候选，
`teleop_avoid` 下预览返回 `active_octomap_not_configured`。关闭键盘、确认控制权
释放及零输出后，经 ProductControl 尝试 `nav + go2_static_check_20260913`：
地图加载成功，但 BBS3D 候选后的 ICP 精配准失败，`map_tracking_initial_alignment_failed`，
导航启动未通过。未发布运动目标，随后恢复 `teleop_avoid`；不能把此结果记为绕障通过。
NX 证据目录：`/home/unitree/lingtu-validation-20260913/forward-goal-preview/`。

上游固定提交的 `advanced_param.xml` 默认竖直膨胀上下各 0.1 m；当前有效 mapd
参数为向上 0.25 m、向下 0.35 m。水平半径 0.25 m、圆柱中心偏移 0.18 m、
前视 3.5 m 和加速度 0.5 m/s²相同。竖直范围差异需要结合机身与传感器坐标验证，
不能直接缩小，也尚未证明它是当前全部截停的原因。

实机当前 mapd 局部碰撞体积为 200×200×100 格、分辨率 0.05 m，
即 10×10×5 m；占据射线范围配置 5 m。遥控前视距离 3.5 m，
允许相对平移输入左右各偏转 90°。局部地图覆盖周围空间，不表示按 W 时会主动倒车。
mapd 水平膨胀 0.25 m，SCAN 前后圆柱查询中心偏移各 0.18 m；
网页抽样点云不能直接代表这张完整的膨胀碰撞地图。

`.12` 会话保留的失败快照 sequence=18（collision generation=41407）显示：
起步速度约 0.287 m/s，侧向目标距起点 0.5 m；失败阶段为
`retimed_collision_validation / collision_after_refine`。重建保存的三次 B-spline，
在 t=0.43 s 前部查询格 [106,101,47] 空闲，t=0.44 s 前部查询格
[107,101,47] 占据，格中心约 (1.125,0.025,-0.025) m，均为快照地图坐标。
起点与目标的前后查询格均空闲；失败段距起点约 0.12 m，并非超出地图范围。
这证明拒绝依据存在于保存的碰撞地图，尚不能证明该占据来自真实物体还是感知误差。
截图的 `scan_local_target_blocked` 与上述保留的优化失败是两种不同诊断，不能等同。

读取时本会话最终安全检查触发重规划累计 18 次，最近原因为
`scan_actual_motion_blocked`。初次 120 秒采集共 600 帧、非零输入 0 帧，
控制循环健康 600 帧；静止数据不能验收用户报告的运动卡顿。
证据目录：NX `/home/unitree/lingtu-validation-20260913/jerk-after-default/`。
此轮仅采集分析，未放宽碰撞判断、修改速度或宣称运动修复。

后续 `held-input` 300 秒采集共 1,499 帧，其中 336 帧请求非零，336 帧的控制
循环均健康；13 帧 `scan_initialization_failed`、6 帧 `local_intent_pending`、
3 帧 `scan_actual_motion_blocked`、1 帧 `scan_emergency_stop`，合计 32 帧非零
请求对应最终零输出（计数是 5 Hz 抽样帧数，不是独立事件数）。输入在导航端的
age_s 为 p50=0.0101、p95=0.0403、max=0.0488，不能代表 PC 按键到机身运动的全链路延迟。

同会话 10 秒序号增量核对：导航循环与最终输出均约 99.43 Hz，驱动按自身状态
时间戳测得 50.00 Hz；operator 样本约 30.56 Hz。键盘轮询为 20 Hz，NX
`teleop-stream --rate-hz 20` 同时支持新输入立即发布和 50 ms 到期补发保活，
因此 DDS 样本数不等于新按键样本数。SCAN FSM 的定时器为 100 Hz、未来碰撞
检查定时器为 20 Hz，均不等于完整轨迹优化吞吐量。频率证据为同目录
`frequency-check.json`，运动统计为 `motion-timing-analysis.json`。

## 原厂避障默认关闭（LingTu 接管期间）

Go2 原生驱动接管控制时，先确认 `SportClient::StopMove()`，再用
`ObstaclesAvoidClient::SwitchGet / SwitchSet(false)` 关闭原厂避障并回读确认。
原厂已关闭时只查询，不重复设置；常规控制和 readiness 刷新不再查询此接口。
停止并释放控制或断线后的重新接管会重新确认，覆盖机器狗重启后开关恢复的情况。
查询、设置或回读失败会保持未就绪并按驱动现有重连间隔重试，不向 SportClient
转发非零速度。日志确认项为 `Go2 factory avoidance disabled (confirmed)`。

这是 LingTu 驱动的接管默认行为，不是修改 Unitree 固件的永久启动设置；
未启动 LingTu、或使用厂商应用重新打开开关时，不能假定原厂避障仍关闭。
LingTu 的 `teleop_avoid`、SCAN 和最终碰撞检查保持开启；不会通过关闭自身避障提速。

`tests/drivers/real/motion/test_go2_acquisition.cpp` 使用可控 SDK 替身执行实际
Go2 适配器，覆盖调用顺序、开关已关闭、RPC 失败、回读仍开启、停止后重获控制、
状态过期后重连，以及普通刷新不调用原厂设置接口。实机是否运动流畅仍需独立验收。

2026-09-13 已安装 `v2.3.0-go2.20260913.12`（运行时代码 `5a4600a`）。
本机与 NX ARM 的接管/驱动核心/状态三项 CTest 均通过，Python 驱动契约 13 项通过。
正式部署后驱动日志出现上述关闭确认，独立 `SwitchGet` 返回 `0 / false`；
`teleop_avoid` 的 `check_obstacle`、`teleop_local_planner` 均仍为 true，
输入门控与驱动就绪，零输出确认。新会话为
`product-d165c5a3fe8e477197b10b250871da19`，已恢复 0.5 m/s / 0.25 rad/s 键盘窗口。
NX 证据目录：`/home/unitree/lingtu-validation-20260913/factory-default/`。
本次未重启机器狗硬件、未发送自主运动指令；实际运动改善尚未验收。

## 2026-09-13 下午 teleop_avoid 卡顿：初步实测

随后用户要求将测试窗口提高到 0.5 m/s。已关闭原 0.2 m/s 窗口，确认控制权释放、
最终与驱动输出均为零，回读厂商避障仍为 false 后，打开 0.5 m/s / 0.25 rad/s
窗口；未提高 Product 的 0.5 m/s 上限，未发送自动运动指令。新采集目录为 NX
`/home/unitree/lingtu-validation-20260913/teleop-speed05/`，该调整不代表低速问题已验收。

重规划操作要区分输入：当前 `?observe=1` 是只读页面，不发送点击目标；
`teleop_avoid` 本身也不接受自主导航目标（`goal_not_allowed_in_teleop_avoid`）。
在 WASD 窗口改变平移方向超过 10° 时，Executor 会重置旧局部规划并建立新参考，
无需先到旧局部目标。持续同方向时，SCAN 按进度、速度变化、未来碰撞检查及轨迹结束
等条件更新轨迹，并非每个控制周期都重新优化。地图点击换终点应在已定位的 `nav`
及可操作页面中另行验收；不能把只读点击无动作当成 SCAN 拒绝重规划。

本次 `.11` 重启后正式启动 `teleop_avoid + SCAN`，会话
`product-2450856b1b844b36891605af568eaf7a`，键盘设置 0.2 m/s、0.25 rad/s。
用户反馈明显卡顿后暂停试验，保留 2,202 行约 5 Hz 记录。采集源不同步；
驱动约 1 Hz 状态去重后只用于区间统计，不当作指令与响应逐帧同步证据。

- 多个持续输入片段里最终指令非零、驱动也收到指令，但实际运动很小。例如连续 W
  6.01 秒片段，平均最终 vx=0.1507 m/s，平均驱动实测 vx=0.0502 m/s；另一
  4.40 秒片段分别为 0.1168 和 0.0096 m/s。不能仅归因于全局或局部规划耗时。
- 纯旋转片段中最终 vx/vy 为零、wz=0.25 rad/s，但实测旋转仍偏慢；执行层因素
  需要单独验证。另有少量 odom_stale/recovering 和碰撞停车事件，不能用总体循环健康抹去。
- 本次开测漏做了重启后的厂商避障状态回读；事后 SDK SwitchGet 返回 enabled=true。
  已在请求/最终/驱动指令均为零时，按用户既有要求 SwitchSet(false)，返回 0，
  再次回读 false。LingTu 的 check_obstacle/teleop_local_planner 保持 true。
- 这只修正了测试条件，尚未证明厂商避障是卡顿的唯一原因。下一轮保持同样的
  0.2 m/s 输入做空地对照，先确认实际响应改善，再评估规划轨迹和固定障碍绕行。

证据：NX `/home/unitree/lingtu-validation-20260913/teleop-afternoon/`；
本地 `build/go2-live-validation-20260913/teleop-afternoon/summary.json`。
开关回读及修改记录为 `vendor-before.jsonl`、`vendor-change.jsonl`。

## teleop_avoid 无线操作与绕障验收计划（2026-09-13，进行中）

本阶段沿用 Windows WASD → Wi-Fi → Sunrise → NX；Sunrise 到机器人仍保留网线。
尚未接入原配双手遥控器，也未完成高速避障验收。所有项目必须在
`teleop_avoid + SCAN` 下执行，不能把 `map` 的直接遥控结果算作辅助避障通过。
该 Product 使用实时雷达/定位构建局部碰撞图，不要求先加载保存地图。

### 分工和执行边界

- 操作者在现场准备固定纸箱或泡沫障碍，观察机器人和场地，并持有已验证可用的独立停止手段；Space 是网络控制停止，不能替代断网情况下的现场停止手段。
- Agent 可以自动执行只读预检、同步采集、回放、指标分析和回归；取得本轮现场确认后，可通过现有 native operator-motion 接口发送有速度/时长上限的试验指令。
- 自动运动首轮最多 1 秒；后续短段最多 3 秒，每段末尾零输入并确认实际停车。测试脚本需保留超时停止和控制权释放。人工键盘与自动试验不能同时持有控制权。
- 第一轮现场确认只用于约定试验，不代表允许无人看护地持续运动。碰撞、方向反转、遥测过期、控制异常或停车未确认时立即终止，不自动重试、不提高速度跨过失败。
- 当前测试速度上限保持 0.5 m/s；原厂避障状态须读回记录，LingTu 最终碰撞检查保持开启。不得用关闭检查、缩小机身包络或忽略真实外部障碍来换取通过。

### 场地与记录

先在空地验证方向和停止，再放置一个能被 MID-360 明确看到的固定纸箱。
建议初始纸箱距机身前缘约 2.5 m，两侧各留至少 1.5 m 通道，绕过后也有停车空间；
这些是初始布置建议，仍需结合实际制动距离和场地确认。人站在运动范围之外，
移动障碍前必须确认停车。不用人迎面靠近来做首次避障实验。

每轮记录：键盘/脚本请求、NX 接收时间与序号、局部候选及选中轨迹、规划/拦截原因、
最终 vx/vy/wz、Go2 实测速度与姿态、原始/配准扫描、碰撞图年龄、定位质量、
循环耗时和控制权状态。跨机器延迟只有在时钟对齐后才计算；否则分别报告本机发送间隔
与 NX 内部处理延迟，不相减制造“端到端延迟”。

| 阶段 | 动作与速度 | 通过条件 / 失败后处理 |
| --- | --- | --- |
| 0：无运动预检 | 正式启动 teleop_avoid，零输入观察 30 秒 | Product/参数一致、传感器和定位新鲜、输入就绪、循环健康、最终零输出；否则先修启动/数据链路 |
| 1：方向与停车 | 空地 W/S/A/D 分别 0.2 m/s、1 秒；每次停车后再下一项 | 前后/侧向符号正确，A/D 无持续意外转向；记录响应延迟及停车时间/距离。异常时对照请求、最终指令、实测运动定位责任层 |
| 2：纯旋转 | 空地 Q/E 分别 0.25 rad/s、1 秒，再按结果扩到 2 秒 | 两方向均有响应；请求与最终 vx/vy 为零；实测平移漂移单独记录，松键停车。近障碍旋转另做扫掠空间检查，正常拦截不记为卡死 |
| 3：空地速度响应 | 依次 0.2、0.35、0.5 m/s；每段不超过 3 秒 | 前一级通过才升级；比较最终与实测速度、起步曲线、网络断流。尚未达到速度稳定段时不宣布速度跟踪通过 |
| 4：单障碍绕行 | 先 0.2 m/s，再逐级到 0.5 m/s；纸箱两侧均可通行，随后分别只留左/右通道 | 实际进入可通行侧并绕过、无接触、无持续左右反复换向；只停车不计绕行通过。缺少安全空间时停车是正确结果，记录具体阻断证据 |
| 5：恢复与连续操作 | 停车后移走纸箱，再重新给输入；人工验证 W→松键→A/D→Q/E，及失焦/断流 | 障碍清除后可以恢复，无残留目标/控制权阻塞；失焦/断流停止，重连不恢复旧非零输入，重新松键解锁后才可运动 |

### “丝滑”的判定

先测基线，再固定阈值，不能在看到失败后偷偷放宽：

- 方向、控制权、无接触和停止必须每次通过；有效非零输入被拒绝必须有可解释、可复现的原因。
- 暂定交互目标：本机按键采样到 NX 最终指令更新的 P95 ≤ 200 ms；只在有可对齐时间证据时判定。实际机器人起步、加减速、停车分别报告，不能把指令变化率当成实测加速度。
- 在空地稳定段比较请求、最终和实测速度；若持续慢，先找第一处速度被压低的位置。尚未取得稳定段、场地不足或被障碍限速时，该项记为未验收。
- 每种方向和每种绕行布置至少 3 次，逐次记录通过/失败；成功绕行的轨迹应连续，无持续停走或左右摇摆。偶尔成功不能覆盖其余失败。
- 0.5 m/s 的上述项目稳定通过、停车距离和感知延迟明确后，另立更高速度的场地/停车余量试验；本计划不授权直接增加现有速度上限。

下一次实际运动前需要现场提供：障碍尺寸及距机身前缘距离、左右/后方可用空间、
可立即停止的方式。先运行阶段 0，再确认阶段 1 的首个 0.2 m/s、1 秒动作。
本节是待执行计划，不是已有实机验收结果。

## 2026-09-13 建图到导航：本轮验收

用户休息期间关闭了 WASD 窗口，确认控制权释放和零输出；没有发送非零指令、
站起指令或自主移动目标。此前 Q/E 采集窗口没有收到纯转向输入，不能宣称实机
Q/E 和连续绕障已验收。原机器地图列表为空，不能把实时局部地图当成已保存地图。

本轮按真实启动流程新增定位和修复：

| 问题 | 原因与修复 | 证据 |
| --- | --- | --- |
| 建图启动后驱动始终不就绪 | nav 启动脚本按默认 CMU 名称检查路径库，未区分是否真正启用局部规划；nav 先退出，driver 随后找不到命令发布者。脚本与 navd 使用相同的规划器启用条件 | 真机启动失败日志；本地旧实现 map/teleop 两项失败，修复后启动回归 8 项通过 |
| 雷达建图强制依赖外接相机 | `map` Product 未声明视觉建图功能却要求 camera 进程及关键模块；移除这项依赖，Host 也不再装配相机。NX USB 枚举没有相机 | 原配置回归失败；Product/启动合计 9 项通过 |
| 无法保存的模式仍显示可保存 | native mapd 只允许 `map` Product 保存，session 却沿用 SLAM 后端静态能力。修正会话能力，网页在非建图模式显示“请先启动建图模式”并禁用保存 | 旧实现 teleop_avoid/nav 两项失败；会话相关 8 项、网页地图流程 15 项通过 |
| 建图启动参数不一致 | 未启用的 CMU follower 仍覆盖 RunPlan 参数，1.0 加速度被报告为 2.0、0.3 前视距离变为 0.5、0.2 到达阈值变为 0.3。仅真正启用 CMU 时应用其配置 | Windows/ARM 旧实现配置回归均复现失败，修复后均通过；`.6` 正式建图启动成功 |
| 建图运行正常，网页却显示未就绪 | 综合就绪错误沿用了导航目标准入条件，mapping 被 `navigation_session_inactive` 拦截。直接遥控 Product 的运行就绪不要求导航会话，目标准入仍保持关闭；保留原生状态过期、急停等阻断 | 本地就绪测试 47 项通过，含 map/teleop/nav 及缺失原生状态、急停案例 |
| 保存成功的地图无法加载 | 默认转换器输出完整 `.ot`，地图校验和原生编辑却只调用 `readBinary`。统一按文件头读取完整或压缩二进制 OcTree，保留空树/错误文件拒绝 | 真机原图 6,515 节点仍被旧读取器误拒绝；ARM 旧用例失败，修复后的 store/activation/save 三套测试通过，覆盖两种格式和完整格式编辑 |
| 实机导航启动互等，仿真没有暴露 | real 禁止 slamd 自行启动地图匹配，ProductControl 却等所有进程就绪之后才通过 Gateway 发起重定位。mapd 等 `map←odom` 点云，Host 又在 mapd 之后启动，形成循环等待。改为 SLAM 前端就绪后，通过已有 `slamctl track-against-map` 原生 DDS 接口开始匹配并等待有效定位，再启动下游；回滚恢复旧导航也经过同一顺序 | `.8` 原图已通过激活，但 mapd 无观察帧，SLAM 只有里程计；本地原有 44 项加 6 项阶段顺序/失败回滚回归通过，`.10` 已进入真实地图匹配，原循环等待不再出现 |
| 冷启动导航先等待不存在的 mapd | `stage_map` 在进程启动之前发 DDS 激活请求，停机状态没有接收者，实际复现 `map_activation_timeout`。先用已有 `mapctl prepare` 离线准备准确地图身份，启动 SLAM 并对齐后启动 mapd，再由 mapd 正式激活，随后才允许 nav/driver/Host 继续启动 | 本地生命周期及部署入口合计 103 项通过；Windows 跳过的 3 项原生 Bash 用例已在 NX 直接执行通过。`.9` 只打包未安装，冷启动修复合入 `.10` 后统一现场验收 |

| 当前机载包缺少无初值全局定位 | BBS3D 在旧构建缓存中为 NOTFOUND，冷启动返回 `initial_pose_required`；项目已有该依赖及构建脚本。使用既定提交 `41529a34a2fb9618b5ff560fb3c2363f1615666d` 离线构建 CPU 版本，启用 `LINGTU_REQUIRE_BBS3D=ON`；CMake 安装同时携带动态库和相对运行路径 | 现场失败扫描 7,486 点对原保存地图离线全局搜索约 4.4 秒，后续配准成功；5 项 ARM 定位回归通过；安装目录 `ldd` 确认使用随包 BBS3D，未遗漏动态库 |

`.20260913.5` 建图被配置不一致拦截；`.6` 已成功通过正式建图启动，并完成
`go2_static_check_20260913` 保存（约 1.19 秒）。此静止地图的首次导航加载
因完整 OctoMap 格式误判而回滚到 map，没有发送导航目标。`.7` 的网页就绪修复
已生效，但打包安装目录遗漏更新 mapd，仍带旧二进制；已用 CMake 正式安装新
mapd/mapctl 到打包目录并重新发布 `.8`。`.8` 已安装，安装后的 mapd 与打包源
逐字节一致；原图无需重建即可通过激活检查，建图 ready/data_ready/motion_ready
均为 true。`.10` 已修复冷启动两处顺序阻塞，但无初值定位因缺少 BBS3D 被拒绝；
给定简单零位姿也未通过点云检查，不能据此放宽配准阈值。NX 运行 venv 没有 pytest，
Python 用例是在 Windows 跑的，不能记为 ARM pytest 通过；ARM 原生配置用例单独执行。
证据目录：本地 `build/go2-live-validation-20260913/overnight/`，NX
`/home/unitree/lingtu-validation-20260913/overnight/`。


**本轮最终状态（2026-09-13 08:46，机载 `.20260913.11`）**：

- 已从停止状态经 ProductControl 完成无初值 `nav + SCAN` 启动，真实地图匹配、mapd 激活、native nav 就绪及目标准入均通过。连续地图匹配成功，未使用伪造定位或放宽配准门槛。
- 导航静止采集 45.08 秒、222 个样本：输入门全部 ready、循环全部 healthy，导航和驱动非零输出计数均为 0；里程计年龄 P95 90.58 ms、碰撞图年龄 P95 90.59 ms，地图处理 445 帧、替换 0 帧。
- 同一静止地图的 6 个只读路径预览：后方 0.5 m、左右各 0.5 m、左侧 1 m 可达，耗时 0.33–2.48 ms；正前方 0.5 m/1 m 返回 `goal_not_reached`。这两点不能记为通过，也不能仅凭该返回值断定是实物阻挡；还需要完整采图及现场位置对照。所有预览均 `motion_published=false`。
- 已正式切回 `map`，当前会话 `product-19690506534e4e119c05933e2c41a87e`；ready/data_ready/motion_ready 均 true，允许保存地图。Windows 生产 NativeStream 的 15 秒零指令试验通过（239 次发送，最大间隔 94 ms），退出回执为 `release_zero_published`；无控制权持有，无全局/局部执行路径，最终输出为零。
- 非运动 doctor：17 pass、0 fail、4 warn。警告为 mapping 不接收导航目标，以及 3 项未安装相机的观察结果；不影响当前雷达建图，但不能用这份报告宣布相机或运动验收通过。
- **仍需人员在场**：WASD/QE 实测方向与响应、行走中点云/定位质量、完整区域采图、固定障碍绕行与到点停车。静止地图只覆盖当前位置可见区域；本轮未发送非零运动指令。

机载源版本 `fb8751f`，BBS3D 按已有构建脚本固定提交离线编译。复建时需先完成
`bash scripts/build/build_3d_bbs.sh`，再以 `LINGTU_REQUIRE_BBS3D=ON` 构建 SLAM；
随后 `cmake --install build/slam_core --component lingtu_runtime --prefix ...`，
确保重新编译的 slamd 与 `libcpu_bbs3d.so` 都进入实际打包目录。

醒来后，在本仓库的 PowerShell 执行以下已有入口即可打开专用键盘窗口：

```powershell
.\config\robots\unitree\go2\start-wasd.ps1 -DomainId 0 -Speed 0.5 -TurnRate 0.35
```

当前 `map` 为直接遥控建图，**不提供 SCAN 辅助绕障**。先现场确认按键方向、松键停车，
再遥控覆盖区域并保存；转导航后先检查定位和路径预览，再进行有人监督的导航验收。
网页观察入口为 `http://127.0.0.1:15050/?observe=1`；键盘窗口在休息期间保持关闭。

操作流程遵循既有 ProductControl：

1. `switch map`：现场遥控采集需要覆盖的区域，再在网页“地图”中保存。
2. 保存操作成功后，确认该地图的产物完整；静止测试地图只能用于链路验收。
3. `switch nav --map MAP_NAME`：加载同一地图并等待定位及目标接收就绪；无初值启动要求安装带 BBS3D 的版本。
4. 先做无运动路径预览，再在人员监督下验收短距离导航、绕障和到点停车。

在已加载机载环境的 NX sudo 终端，入口为：

```bash
python -m lingtu.control switch map --robot unitree/go2 --env real --json
python -m lingtu.control switch nav --robot unitree/go2 --env real --map MAP_NAME --local-planner scan --json
```

Windows 的 `start-wasd.ps1 -DomainId 0 -Speed 0.5 -TurnRate 0.35` 仅连接已就绪的
控制链路，不启动 Product。`map`/`teleop` 是直接遥控，`teleop_avoid` 提供辅助避障；
`manual_mode=0` 本身不等于开启局部规划。这里的 domain 0 必须对应当前 RunPlan。
重启后仍由 ProductControl 显式启动；不会自动恢复上一次非零指令或导航目标。

## SCAN 官方、Thunder 仿真与 Go2 实机的验证边界

[SCAN 官方说明](https://github.com/wuyi2121/SCAN-Planner#-important-functions)
明确默认参数针对 Unitree Go2，公开操作入口是目标点、关键点和参考路径。
我们使用的 WASD 方向辅助避障是 LingTu 的 Product 行为，不能当作上游已验收的
接口。上游局部规划器也不包含我们使用的独立 Unitree SDK2 驱动与运动仲裁。

| 环节 | 官方 SCAN | LingTu 当前验证范围 |
| --- | --- | --- |
| 输入 | RViz 目标、关键点、`/initial_path` | `teleop_avoid` 将方向输入转为局部参考，需单独验证受阻后的目标选择 |
| 雷达与定位 | 实机接入 LIO 的机身/传感器位姿与点云 | MID-360 原始帧/IMU → Fast-LIO2 → native DDS，外参、时序与动态配准需实测 |
| 局部地图 | 内置滑动 3D 占据地图和双圆柱机身检查 | 独立 mapd 输出碰撞地图，由 SCAN 与最终运动检查消费 |
| 执行 | 上游 controller 输出 `cmd_vel`，仿真有 Go2 运动学模型 | 既有 MuJoCo 验证为 ThunderV4 策略；Go2 实机由 SDK2/固件步态执行 |

官方真实/仿真输入定义见
[`run.launch`](https://github.com/wuyi2121/SCAN-Planner/blob/main/src/planner/plan_manage/launch/run.launch)，
运动学仿真见
[`go2_kinematic_sim.cpp`](https://github.com/wuyi2121/SCAN-Planner/blob/main/src/planner/plan_manage/src/go2_kinematic_sim.cpp)。
本地仿真绑定在 `sim/sessions/products/doso/thunder_v4/teleop_avoid.yaml`；
Go2 雷达外参在相邻 `sensors/mid360_fastlio2.yaml`，不得用 Thunder 模型响应替代 Go2 实测。

后续按同一条证据链逐段验收：同步记录原始雷达/IMU、LIO 位姿/配准扫描、局部碰撞图、
操作者请求、规划轨迹、最终指令和驱动实测速度。先验证静止地图与空地直行/横移，
再验证固定障碍物下的方向辅助绕行。每次找到第一处偏差后修改其所属环节，
不能只凭点云画面、进程在线或仿真通过就宣布 Go2 绕障通过。

## 2026-09-12 再次开机：地图候选 .9

NX 开机后恢复可连接，仍安装 `.8`，Product 服务未启动。通过 ProductControl
启动 `teleop_avoid + SCAN`，基线会话 `product-d8e4a065425b4fe6b7634e3dcf7ed27a`。
45.18 s 的 223 个静止采样全部 input ready / loop healthy，导航和驱动零输出；
碰撞图年龄 P95 90.57 ms、控制工作 P95 最大 1.548 ms，地图处理 441 帧，
无 pending 替换。motion 只读预检通过。厂商避障重启后为 true，按既有要求关闭
并回读 false；LingTu 碰撞检查保留。实机 Web 隧道恢复到本机 15050。

取回 `.8` 的 `motion-v8/`：16:05:12–16:20:12 共 4455 条导航状态，操作请求和
最终指令全部零，SCAN 未产生运动规划。这段记录没有捕获到用户描述的运动卡停。
其中 SDK 机体反馈曾变化、后续高度下降，不能把“指令为零”等同于“物理全程静止”，
也不能凭此确定外部操作或底盘行为的原因。

地图修正已经在 NX 的 Release 构建中通过 32 项占据图、16 项 mapd 用例，断言启用。
候选差异只有完整射线清理、终点单次计票、对应回归和衰减测试时序修正。
首份传输补丁因混合换行符不能应用，未修改源码；更正为保留原始行尾的补丁后
应用并重新编译、测试通过。首轮旧程序测试不作为修复证据。

已通过正式安装器部署 `v2.3.0-go2.20260912.9`，源码 `7742196`，会话
`product-72b18ec5329f4bcfbe171afa9b1bffb2`。有效包在 NX `field-v9/release-final/`；
最初暂存包的源码元数据早于提交，未用于安装。mapd 与 navd 均从本次源码重建，
其余已验证的原生程序沿用 `.8`。导航实际无 domain 参数或环境覆盖，使用声明的 domain 0。

部署后 45.17 s / 223 个样本全部 input ready / loop healthy，导航和驱动非零输出均为 0；
定位年龄 P95 100.57 ms，碰撞图年龄 P95 90.56 ms，控制工作 P95 最大 1.532 ms。
地图处理 440 帧，pending 替换为 0，motion 只读预检通过。与更新前静止基线相比，
本次没有出现地图刷新变慢或控制循环负载恶化；这不是运动负载的验收结论。

开始有界 15 分钟只读采集后，打开 domain 0 的 0.50 m/s WASD 窗口，由用户操作。
采集包含导航请求/规划/最终指令、driver、SLAM、mapd 及失败快照；最多 5 Hz，
driver JSON 约 1 Hz，来源不同步，不能当作逐指令端到端时延测量。助手未发送非零指令。
先验收空地起步、松键停车，再进入固定障碍绕行。

本轮证据在 `build/go2-live-validation-20260912/field-v9/`；NX 对应目录为
`/home/unitree/lingtu-validation-20260912/field-v9/`。

本次现场验收未通过，用户反馈“仍然很慢或容易停住”。最终 15 分钟记录共 4495 行，
按唯一导航状态筛选有 270 个新鲜非零请求采样，其中 24 个最终输出全零：
8 个等待局部意图、11 个初始化失败，其余为真实运动碰撞检查、生成轨迹、紧急停车或
局部目标受阻。活动采样的输入门全部 ready，SLAM 全程 TRACKING、mapd 全程 ready。
SDK 线速度峰值 0.514 m/s 不能证明起步和连续运行正常。相同 W/S 输入内也出现短轨迹
末端减速、换轨后接近零速；状态文件约 5 Hz，driver 约 1 Hz，不据此推断逐指令延迟。

冻结失败 588 定位到非零初始加速度的时间调整固定只执行三次：起始速度约 0.340 m/s、
加速度约 0.218 m/s² 本身可行，三次后轨迹加速度控制点上界仍为 0.5202 m/s²，超过
0.5 限制而被拒绝。继续保持相同起终点位置、速度、加速度调整可以收敛，冻结全图
扫掠检查无碰撞。修复为有界收敛，并保留原速度、加速度及完整轨迹碰撞检查。

冻结失败 597 的实际起点空闲，首段约 0.24 s 时前圆柱进入相邻占据格，不能描述为
“机器人已经在障碍物内”。旧候选最短 0.5 m，遗漏附近 0.2–0.4 m 的侧向空间。
保持原候选顺序，穷尽后补充 0.4/0.3/0.2 m 短侧向候选；相同输入回放由持续失败变为
第 12 tick 生成 0.3 m 安全侧移，261 段扫掠零碰撞，最大速度约 0.188 m/s、加速度
0.492 m/s²。侧移相对原参考线定义，与当前机身相差约 20.75°，带约 0.106 m 当前
机身后向分量；这只是局部安全挪动，不能称为已经完成向前绕障或实机验收。

旧的 33 份冻结数据由 29 份可行变为 30 份，原可行的 29 份没有丢失；一个仍可行案例
改选较短轨迹，一个搜索从 27 增至 39 tick，不能只用通过总数推断规划速度或路线完全不变。
曾发现短候选抢在原参考优化之前造成退化，已修正为原参考兜底也失败后才启用。
若根本没有可用参考终点，可直接尝试短侧向候选。新增用例覆盖左右侧及参考终点
空闲/占据的 12 种短口袋组合。

持续输入的顿挫还对应第三处逻辑：重规划进度门为 1 m，而短局部段仅 0.3–0.5 m，
终速设为零，原先要等整段结束才续规划。MotionIntent 的非最终临时段现在按段长
提前触发续规划；长段和最终目标到达条件保持。续接边界沿用旧样条当时的 P/V/A，
不能称为实测加速度；若下一段不可行仍停车，不保证狭窄空间里始终不停。

三处修复的 Windows SCAN 回归 69 项通过，588 新测试已先在旧实现复现失败。
续接回归使用 External 测试时钟比较精确边界，验证短段耗尽前换轨、P/V 连续且保持
非零速度；这没有修改实机的运行时钟。`.9` 的现场失败结论保持。详细分析见
`field-v9/stable-motion-v9-report.md`、`dynamic588/` 和 `short-side-fix/`。

## 2026-09-13 .4 已部署：Q/E 控制修复，现场验收待完成

三处修复已通过 Windows 与 NX ARM 原生遥控控制器回归，并通过正式安装器部署
`v2.3.0-go2.20260913.4`，运行源码提交 `0286e1c`。Product 会话为
`product-7a0f9282f115474799cc8cacc1c3be7e`，DDS domain 0，navd 来自该发布目录。
安装后 45.17 s / 223 个静止样本中，导航和驱动非零输出均为 0，输入门均 ready，
控制循环均 healthy；定位年龄 P95 100.62 ms，碰撞图年龄 P95 90.56 ms，
控制工作 P95 最大 1.563 ms，地图处理 449 帧、pending 替换 0。运动只读预检通过。
这只证明本地回归和实机静止准备状态，不证明持续转向、绕障或所有停滞问题已解决。
新版 WASD 从零输入启动，平移上限 0.50 m/s、转向 0.35 rad/s；等待用户操作 Q/E
并结合请求、最终指令和实际角速度验收。不会自动发出非零运动指令。
实机证据目录：NX `/home/unitree/lingtu-validation-20260913/field-v13/`。

重新开机后已连接 NX，并通过 ProductControl 恢复 `real / teleop_avoid`（SCAN、
DDS domain 0），启动时零请求、零最终指令，输入门 ready。当前 navd 环境和参数
均未覆盖最小平移阈值，实际采用源码默认 0.03 m/s；因此以下零阈值缺陷不能作为
当前配置下实机 Q/E 不转的已确认根因。重新读取 `.3` 的持久记录，共 1,586 行有效、
1 行关机后无效记录，只有 2 个纯 Q 样本，均最终输出 +0.35 rad/s；样本不足以
验收持续转向。驱动代码也会将纯 yaw 非零指令传给 SDK Move，而非当成 StopMove。

进一步复现了第三处反馈问题：从平移规划失败切换到纯旋转时，导航主循环没有收到
清除旧局部诊断与路径的结果，会继续展示旧的 blocked/轨迹。现在直接控制取消
平移规划时明确清空局部诊断并发布空路径；当前旋转结果仍由 teleop/final 状态表示。
新回归先触发平移失败再切 Q，旧实现未清除而失败；最终原生回归验证其清除行为。

此前用户反馈 Q/E 无法旋转时，NX 已不可达；Sunrise 的 eth0 为 DOWN，用户确认已关机
或暂时断开。该次故障没有同步请求与最终否决记录，不能宣称现场根因已确认。
旧 `.2` 记录有一份纯 Q 输入 `(0,0,+0.35)`，最终同值、reason=accepted，只证明
此前曾有旋转请求通过，不能替代这次验收。键盘映射保持 Q 左转、E 右转。

本地控制器回归独立复现并修正两个问题：

- 当公开支持的平移最小阈值设为 0 时，纯旋转因 `0 >= 0` 误入平移规划，执行器
  认为没有平移意图而返回空闲。辅助平移规划现在显式要求非零平移；Q/E 仍通过
  最终速度整形、足迹刹停检查及唯一驱动链路，不以伪造路径或关闭避障放行。
- 旋转后松键，原先先整形再停止，可在该帧发布残留角速度。零请求现在直接走
  零指令和停止平滑器的分支，避免这一帧多转。

以上配置分支故障尚未证实就是用户这次实机故障。后续开机要分别采集纯 Q / 纯 E
的请求、最终角速度、实际角速度和否决原因；若为旋转扫掠碰撞，需要检查对应占据格。
新增原生回归使用真实平滑器，覆盖 ±0.35 rad/s、默认/零平移阈值、松键、超时，
并确认最终碰撞否决仍然生效；键盘映射、组合键及松键相关 8 项 Python 回归通过。
Windows 原生遥控控制器整组回归通过（退出码 0）；新增回归的零阈值误路由和
松键残留角速度在旧实现分别复现失败。ARM 构建及部署现已完成，实机 Q/E 验收待完成。
本地构建与回归结果见 `build/go2-live-validation-20260913/qe-control/`。
上述三处代码已进入 `.20260913.4`；默认 0.03 m/s 阈值下的现场不转根因仍需新记录确认。

## 2026-09-13 .3 已部署：短绕行目标保留与实测速度续接

`.2` 的现场低速、停走验收失败，定位到短绕行续接状态机，不归因于键盘轴映射。
本轮修复只改 SCAN FSM：

- 新候选仍至少距离起点 0.2 m；已选目标按实测距离与现有到达阈值判断是否完成，
  保留原有方向范围、规划半径和整段碰撞检查。
- 计划位置已进入已选目标的最后 0.2 m、机器人实际尚未到达时，延后仅由进度触发
  的重规划。正常长段续接保留，碰撞回调仍可立即重规划或停车。
- 短段名义结束后，下一段从实测位置、实测速度和零初始加速度开始；不再把旧样条
  终点零速度覆盖到仍在运动的机器人上。该变化仅适用于 MotionIntent 的已结束段。

针对性回归注入 0.13 m 跟踪滞后和左右通路变化：旧实现提前换侧，目标偏离原目标
3.635 m；只修目标后，续接初速度仍与实测相差 0.1 m/s。两处均有独立失败记录。
测试还覆盖实际距目标 0.15 m 时续接同一目标、实际到达后可选新目标，以及新图
阻断旧尾段时必须触发碰撞重规划/停车，不能靠保留目标继续撞向障碍。
Windows 与 NX ARM 原生的 71 项 SCAN 回归均通过（分别 15.11 s / 5.82 s）。
已通过正式安装器部署 `v2.3.0-go2.20260913.3`，运行源码提交 `393e1a7`；
当前 Product 会话 `product-a496cd31d64149c2ba96c8452074ff3d`，DDS domain 0，
运行 navd 的路径已确认来自该发布版本。

安装后 45.17 s / 223 个静止采样中，导航和驱动非零输出均为 0，输入门均 ready，
控制循环均 healthy；定位年龄 P95 91.74 ms、碰撞图年龄 P95 90.57 ms，控制工作
P95 最大 1.487 ms，地图处理 446 帧、pending 替换 0。运动只读预检通过，无阻断项。
SDK 仍报告最高约 0.0197 m/s 的微小实测速度，不能把零输出等同于物理上绝对静止。
旧键盘窗口已关闭，新 0.50 m/s WASD 窗口从零输入打开；正在等待用户复测反馈。
本轮实机连续运动和绕障尚未验收通过，不能用静止预检与回归替代。
证据：`build/go2-live-validation-20260913/field-v12/`。

## 2026-09-13 .2 已部署，现场复测仍慢且反复停车

已通过正式安装器部署 `v2.3.0-go2.20260913.2`，源码提交 `306e77f`，当前
`real / teleop_avoid` 会话为 `product-548634c7a11f419e879ab40641cb306c`。
本次更新 navd，保持真实障碍、速度/加速度约束和最终运动检查。

- 方向：先尝试沿请求方向的安全短段，再尝试较长的侧向绕行；保留辅助绕障能力。
- 时间调整：用有效曲线的严格速度上界代替包含首尾外推控制点的过度保守上界。
  065/071 的独立旧实现失败、新实现通过；边界位置、速度和加速度不变。
- 卡点退出：SCAN 当前足迹占据且普通规划不可用时，接通既有首格退出检查；仅沿
  当前用户平移方向检查 0.35 m 直段。速度不超过 0.15 m/s 及当前请求上限。
  有旋转请求、进入第二个占据格、末端不空闲或最终实测/刹停检查不通过时拒绝。
  松键、改向、输入超时或地图否决时撤销；没有开启自主寻找方向的随机恢复。

Windows 与 NX 的 70 项 SCAN、3 项速度边界回归通过。Windows 82 项执行器整组
通过后，低速请求补充与相关 3 项定向复测通过；NX 整组 82 项通过、1 项新增低速
用例未等异步初始化而失败，该用例改为等待规划结果后，相关 3 项重测通过。
两平台的辅助遥控控制器测试通过。测试等待的修正没有改变生产超时或运动限制。

34 个冻结场景保留原有全部 31 条可用轨迹，11,369 段扫掠零碰撞；原 3 个不可用
场景仍不可用。197 通过实际 Executor → Teleop → Final 的离线回放：W 持续退出，
A、松键、无输入归零；仅更新同一碰撞图的版本时，初始化后 37 tick 连续非零，
最高 0.15 m/s。该版本更新用例没有注入新障碍，不冒充真实动态避障验收。

安装后 45.17 s / 223 个静止采样的导航及驱动非零输出均为 0，输入门均 ready、
循环均 healthy；定位年龄 P95 91.75 ms、碰撞图年龄 P95 90.57 ms，地图处理
445 帧、pending 替换 0，motion 只读预检通过。旧控制窗口已关闭，新 domain 0 /
0.50 m/s WASD 窗口已从零输入打开，并开始 15 分钟只读采集。

现场复测未通过。676 份去重导航状态中，190 份新鲜纯 W 输入有 44.7% 的最终
平移速度低于请求的 20%，15.3% 三轴为零。稳定 W 的 8.21 秒中，最终安全
重规划计数从 0 增至 8，原因均为 `scan_actual_motion_blocked`；随后的初始化
碰撞失败与重新起步形成停走循环。采样未观察到首格退出，不据此宣称实机退出通过。

已定位提前换侧：旧绕行段长 0.2458 m，计划推进 0.1271 m 后触发半程重规划。
旧目标距新的计划起点只剩 0.1187 m，触发了新候选的 0.2 m 最小距离判断，
但距相邻实测位姿仍有 0.2436 m。旧目标被过早丢弃，新目标换到另一侧。
154 冻结碰撞图中，旧右侧连接及其完整刹停扫掠仍空闲；新左侧曲线在约 0.15 s、
起点后 1.64 cm 碰到前圆柱占据格。真实外部障碍保留，最终运动检查的停车有依据。
这证明需要区分“新目标最小距离”和“已选目标实际到达”，不能通过放宽安全检查处理。
5 Hz 状态不能证明重规划由进度回调还是碰撞回调最先触发；新目标选择中的距离误判
以及当时实际仍在向旧侧运动均有记录支持。修复与新一轮现场验收仍在推进。

证据：`build/go2-live-validation-20260913/control-direction/`；NX 发布与运行证据：
`/home/unitree/lingtu-validation-20260913/field-v11/`；本次实测分析：
`build/go2-live-validation-20260913/field-v11-analysis/`。

## 2026-09-13 三处 SCAN 修复已部署，现场复测仍未通过

已安装 `v2.3.0-go2.20260913.1`，源码 `d0edccb`，Product 会话
`product-b3759ce6a53d4783814790582b355ecb`。通过正式发布包和安装器切换，只有
navd 在 `.9` 的基础上更新；mapd、SLAM、驱动和传感器标定保持前一版。
NX 原生 Release 的 69 项 SCAN 回归通过；597 全图回放 12 tick 得到短侧移轨迹，
261 段扫掠零碰撞，单 tick 最大约 3.48 ms。该数字是离线 ARM 调用耗时，不是实机
绕障完成耗时。

部署后 45.17 s / 223 个静止采样的输入门全部 ready，控制循环全部 healthy，导航和
驱动非零输出均为 0；定位年龄 P95 100.62 ms、碰撞图年龄 P95 90.59 ms，控制工作
P95 最大 1.453 ms，地图处理 436 帧、pending 替换 0。motion 只读预检通过。
SDK 的微小速度反馈不能替代现场观察，不把“输出全零”写成物理上绝对没有移动。

现场复测仍未通过：用户报告向左输入却向前走、速度慢、容易卡停。重新打开的
0.50 m/s WASD 窗口采集了 750 份去重导航状态，其中 134 份有新鲜非零请求。
40 份 A 输入均正确请求 `(vx=0, vy=+0.5)`，其中 27 份最终指令以前向为主。
第一段 A 的参考方向约 90.14°，局部轨迹方向约 0.14°，机器人朝向仅偏转
0.002–0.041 rad；错误发生在绕行目标选择，不能归因于键盘轴交换或机器人转了 90°。
当前允许 ±90° 绕行，且长侧向候选优先于请求方向的短距离进展。

134 份活动状态的输入门均 ready，但 52 份最终平移速度低于请求的 20%，20 份
三轴最终指令为零。较长的 S 输入约 12.90 s 内观察到 17 次轨迹变更；第一段 A
也存在换轨时保持约 0.5 m/s 的实例，因此不能把每次换轨都等同于停车。

新失败 065/071 还复现了时间调整误判：边界恢复后的首尾外推速度控制点被直接当作
有效曲线速度上界。065 保存曲线的实际峰值速度约 0.499825 m/s、加速度约
0.411013 m/s²，均在 0.5 限制内，却被判超限。071 的同类反复放大使间隔异常增大，
最终被有限数检查拒绝，没有发布该坏轨迹。增加迭代次数不能解决这类误判。

另有 8 份新失败在轨迹 t=0 已碰撞。197 的完整碰撞图经原生 Grid 查询确认：
机身中心和前圆柱空闲，后圆柱处于占据；这与旧 597 的起点空闲、稍后首段碰撞不同。
用户随后确认该处靠近外部物体。7 组同序列原始观测/碰撞图中，5 组后圆柱占据均有
当前回波落入实际膨胀核，另外 2 组无该回波且后圆柱空闲；保留这些真实障碍。

197 冻结图按现有 Grid 边界退出规则，正前方 0.35 m 是唯一通过的 16 方位出口：
后圆柱前进约 2.93 cm 后离开初始占据格，之后前后圆柱经过的格均空闲；其余 15 个
方向仍被拒绝。以 0.15 m/s、0.35 s 反应时间、0.5 m/s² 减速度及当时实测速率
代入最终运动检查也通过。但 SCAN 的辅助遥控未接通该出口：通常恢复排除了 SCAN，
teleop 的恢复器只旋转，而且没有向最终检查传递已验证平移标记。这是另一处接线缺口，
不是删除障碍或扩大穿越范围的理由。退出必须仅沿当前按键方向，并在松键、改向、超时
或新地图否决时停止。

本轮证据在 `build/go2-live-validation-20260913/control-direction/`，原始采集在 NX
`/home/unitree/lingtu-validation-20260912/field-v10/keyboard-retest/`。旧版静止预检与
回放通过的结论仍成立，但不代表本版实机方向、持续运动或绕障已经验收通过。

## 2026-09-12 验收链路补齐与地图修复候选（当时尚未部署）

当前正式 Product 的 Thunder MuJoCo RunPlan 默认使用
`sim_mid360_slam.yaml` 和原生 Fast-LIO2；过去独立 `teleop_avoid.json`
组件配置却默认 `mujoco_navigation_fixture`、瞬时扫描与真值位姿输入，
并由夹具补充地面覆盖。这两种仿真结果不能混作同一条传感器链的证据。
默认组件配置现改为 Fast-LIO2、physical rolling 和关闭真值 prior；
显式 fixture 诊断仍可用，但报告明确只验证真值定位之后的导航部分。

| 环节 | 当前正式 Thunder 仿真 | Go2 实机 |
| --- | --- | --- |
| 雷达 / IMU | MuJoCo 生成与原生 DDS 发布 | MID-360 实际采样与原生 DDS 发布 |
| 定位 | 原生 Fast-LIO2，Thunder 仿真外参与 IMU 配置 | 原生 Fast-LIO2，Go2 MID-360 标定配置 |
| 地图到指令 | native mapd → SCAN → follower → 最终运动检查 | 相同核心模块，独立实机配置和时序 |
| 执行 | MuJoCo driver bridge → Thunder 4998 策略 / 物理步进 | Go2 driver → Unitree SDK2 / 固件步态 |

这意味着共享规划算法，但没有完整共享传感器误差、地图观测、底盘响应和运行时负载。
Go2 的 `sim` 资产/会话仍未绑定。不能声称已有 Go2 仿真到实机的闭环验收。

本轮还补强了附加避障验收：原来只读取 `teleop.output`，最终仲裁清零时仍可能
报告局部输出正常。现在要求顶层 `final_cmd_vel` 已发布且非零，并从当前
`slam_runtime` 状态检查沿操作者方向的真实里程计推进，复用既有前进距离阈值。
完整 Product wrapper 原先已另有 MuJoCo 物理位移/停车检查，本轮没有把它的既有
能力遗漏或冒充新增。相关 dispatcher 与 runner 回归共 151 项通过。

地图候选修复保留完整射线清理，并纠正终点重复投票：终点的命中或范围裁剪空闲票
已经单独记录，不能在遍历时再次计为空闲。新增“多个真实端点与一条穿过射线”及
“裁剪端点与真实端点落在同格”的旧补丁失败用例。Windows Debug 的 32 项占据图
及 16 项 mapd 用例通过；衰减集成测试改为读取同一状态快照，避免把合法的独立
衰减误判为初始化失败。保存的真实点云回放仍使两个穿过格由 0.98 降至约 0.6231，
两个未穿过对照格仍为 0.98，点序变化不影响这些结果。ARM 在线耗时尚未验证。

本轮重建 Windows 原生 navd、mapd、slamd 后，以正式 ProductControl 运行一轮
Thunder 固定障碍场景，定位来源为 Fast-LIO2 估计器，没有真值 prior。0.5 m/s
前进请求持续 35 s，MuJoCo 物理前进 12.480 m，最大侧向绕行 2.025 m，障碍净距
最小 0.397 m，回归原走廊的侧偏 0.095 m，停车与清理均通过。343 份导航采样
均 input ready，其中 330 份有已发布非零最终指令；里程计沿指令方向推进
12.296 m。该轮分类为 `component_e2e`，`product_acceptance_passed=false`，
没有冒充完整 Product 场景矩阵或实机验收。首次启动因二进制旧于源码被拦截，
保留失败报告；重建后第二次通过。证据与精简结果见
`build/go2-live-validation-20260912/sim-chain/summary.json`、`result-run2.json`。

`.8` 对既有 33 份实机失败输入重新进行独立冻结回放，29 份可解；11718 个轨迹
采样/扫掠检查零碰撞，峰值约 0.49994 m/s。剩余 4 份分别为动力学不可行、两份
时间调整后碰撞、一份优化重启上限；不能通过放宽碰撞约束将其记为通过。
这不是实机连续运行成功率。证据：`build/go2-live-validation-20260911/detour-replay/isolated-v8.jsonl`。

用户已确认机器人关机或网线断开；Sunrise Wi-Fi 在线，但 eth0 无载波，NX 不可达。
本节地图候选、验收代码尚未安装到 NX，最近确认的实机版本仍为 `.8`。
下一次开机先取回 `motion-v8/` 的操作记录，正式部署候选并完成无运动地图刷新检查，
随后现场验收空地 W/S/A/D 响应、固定障碍绕行、松键停车与换向恢复。

## 2026-09-11 SCAN 绕障修复：实机数据回放，现场尚未验收

本轮处理 `teleop_avoid + SCAN` 的局部目标选择、短距离运动时间调整，
以及候选轨迹末段碰撞漏检。已通过正式发布流程安装 `.5`，现场运动验收尚未通过。

- `backend.cpp` 把 MotionIntent 已声明的最大偏离角传给 FSM。
  `SCANReplanFSM::getLocalTarget()` 在直线参考目标或到它的连接被占用时，寻找可达的左右目标。
  默认在意图方向两侧 90 度以内，以不超过 30 度的角度间隔、逐级缩短的距离搜索；
  3.5 m 视野最多增加 24 次机身扫掠查询。目标沿用至到达、失效或原参考重新直达，
  避免每次重规划左右切换。新参考重置目标，松键仍由现有执行器释放与停车。
- 某个替代目标整次规划失败后，下一个周期尝试下一个固定几何候选；占据图更新
  不会改变候选编号。候选耗尽后保留原参考路径的规划机会，再开始新一轮。
  同一周期不增加优化调用次数。更换目标时重新初始化，不能沿用上个目标的
  失败随机种子；成功目标保持，松键、换向和新参考仍清理对应状态。
- 替代目标需要从当前起点到目标的整段机身净空通过，随后继续进行 B-spline 优化、
  动力学可行性与最终运动检查；没有缩小机身、清空起点占据或关闭碰撞检查。
  RouteTarget 的目标语义保持不变。按 W 可以向左右绕，但没有新增自动向后脱困。
- `bspline_optimizer.cpp` 的 rebound 接纳与 refine/时间调整后的复核原来只检查
  候选轨迹前 2/3，已改为检查整条候选轨迹，含末段到点段。周期性 FSM 前视仍是
  执行中检查，不能替代发布前的完整候选验证。
- `planner_manager.cpp` 原来只做三次时间拉伸；保持非零起步速度时，边界控制点
  会随时长改变，三次拉伸不保证满足加速度限制。实测约 0.331 m/s 初速、0.5 m
  侧向目标在空地图上也失败：三次后加速度控制点上界约 0.692 m/s²，超过 0.5。
  对零端点加速度，现由控制点表达式 `Q(h)=B+hL` 计算可行时间上界，再保留可行
  区间进行细化；使用调用者真实初速、初加速度，不从拟合多项式反推边界。
  非零端点加速度保留原有通路，调整后仍复核完整轨迹碰撞，未提高任何运动上限。

本地 Release SCAN 65/65 通过；NX ARM Release 的 SCAN 65/65、executor 80/80、
local-planner core 50/50 通过。新增覆盖左右通道、中段受阻但端点自由、角度限制、
目标保持、动态地图下候选切换与初始化顺序、末段碰撞，以及带初速的短侧移。

上轮保存的 33 份真实地图、位姿、速度与方向输入均保留。每份从独立 FSM 开始，
固定自己的随机种子，最多运行 64 次规划回调，覆盖 24 个几何候选各自的确定性/
随机初始化和参考回退。逐案隔离随机状态，避免前一场景的重试次数影响后一场景。
这不是历史 FSM 的逐帧复现，也不是在线绕障成功率。

本地同一二进制的受控对照：启用侧向候选时 28/33 可解，仅使用参考路径时 9/33
可解；后者的 9 份全部保留，新增 19 份可解。28 条轨迹的 11525 个采样及相邻
机身扫掠段均无碰撞，速度不超过输入上限。剩余 5 份保持拒绝：1 份初速约
0.50769 m/s 已超过请求 0.5 m/s，3 份时间调整后碰撞，1 份优化重启达到上限。
早期共享随机状态的 15/33、26/33 等阶段结果不能用作逐案严格 A/B。

NX 上相同逐案隔离对照也是 28/33 对 9/33，9 份参考可解输入全部保留。
28 条成功轨迹检查 11702 个采样/机身扫掠段，零碰撞且未超输入速度上限；
最大单次规划回调约 22.79 ms，成功场景最多使用 19 次回调。这是离线 ARM 回放
耗时，不等于实机在线控制周期或人机交互延迟。失败分类与本地一致。

本轮还复现了发布前只检查前 2/3 的漏检：一条已返回 Ready 的旧候选有 64 个
末段碰撞采样，修复后该候选不会再被直接接纳。没有用降低碰撞或动力学标准来
清除失败。现场固定纸箱绕行、速度跟踪、松键停车仍需单独验收。

本地证据：`build/go2-live-validation-20260911/scan-accepted.xml`、
`detour-replay/isolated.jsonl`、`detour-replay/reference-only.jsonl`；
NX 证据在 `/home/unitree/lingtu-validation-20260911-bRrcju/detour/`。
下面 `.4` 的日志说明缺陷出现时的条件，不代表本轮补丁已通过现场运动验收。

### .5 部署与静止检查

已安装 `v2.3.0-go2.20260911.5`，源码快照 `b0a5bb5`。正式安装器校验通过后，
由 ProductControl 启动 `teleop_avoid + SCAN`，会话
`product-3794d98a3d4241778933bc7a62f70e6d`。当前 navd 没有 domain 参数或环境覆盖，
使用其声明的默认 DDS domain 0。实机监控为 `http://127.0.0.1:15050/?observe=1`。

45.17 s 静止采集共 223 个样本：输入门全部 ready、driver/maps 全部 ready，
导航和驱动非零输出均为 0；定位/碰撞帧龄 P95 为 82.24/90.57 ms，
驱动回执帧龄 P95 为 11.26 ms。地图处理 445 帧、无 pending 替换。
但 83 个样本报告 `p95_utilization_high`，控制工作耗时 P95 最大 9.29 ms。
motion 只读预检被 `motion.control_loop` 和 `motion.correlated_driver_ack` 拦截，
所以本次没有打开新的运动验收窗口。ACK 检查仍有异步快照固定序号容差问题，
负载观测中 `motion_update_last` P95 为 7.92 ms，障碍快照生成 P95 为 1.71 ms。

按用户既有要求，重启后厂商避障从 true 设置为 false，SDK 返回 0 且回读 false；
LingTu `check_obstacle=true` 保持。没有发送非零运动请求。证据见 NX 上述目录的
`deployed/static-summary.json`、`deployed/readiness.json`、`deployed/factory-avoid.jsonl`。

### .6 输入消费修正与 .7 诊断等待修正

`nav/main.cpp` 仅对 `TeleopAvoid + SCAN` 关闭 navd 自身的 registered-cloud reader。
此模式的局部规划、轨迹检查和最终制动都消费 mapd 的 `local_collision`，原来的
MotionLayer 点云处理与障碍快照没有运动消费者。mapd、SLAM、Web 独立的 DDS
订阅继续运行，其他模式和规划后端也保留原点云输入。没有降低碰撞标准或负载门槛。

只读 motion 预检按实际输入所有者检查：SCAN 要求有序、新鲜且完整的碰撞图，
CMU 仍要求注册点云。控制回执固定一次 nav 输出的 producer/sequence，再在现有
诊断快照新鲜度窗口内等待 driver 接纳该序号或后续序号；不再比较两个异步采样的
“最新序号相差不超过 2”。无法覆盖、回执拒绝、producer 不同或状态过期仍失败。
等待时间计入已有快照年龄，不把开始等待时的新鲜度当作返回时的新鲜度。

2026-09-12 重启后确认 `.6` 仍安装，但 `/run/lingtu/current.json` 与原生状态文件
不存在，通过 ProductControl 重新启动 `teleop_avoid + SCAN`。45.17 s 采集的 223
个样本全部 input ready / loop healthy，导航与驱动非零输出均为 0，工作耗时 P95
最高 1.433 ms（之前 `.5` 最高 9.294 ms）。定位和碰撞帧龄 P95 均约 90.6 ms，
mapd 处理 450 帧，无 pending 替换。不能据此声称运动时已绕障通过。

`.6` 的诊断等待仍错误使用了 0.35 s 的实时 DDS 回执门槛，而 driver JSON 文件
在 `src/drivers/real/motion/main.cpp` 中每 1 s 才发布一次。`.7` 仅把文件采样等待
改用既有 `status_max_age_s`（默认 3 s）；实时 DDS 回执超时、最终运动仲裁保持。
覆盖序号、同 producer、快照老化与超时拒绝均保留。候选脚本在重启后的 NX 上
只读 motion 预检 37/37 通过，证据在 `/home/unitree/lingtu-validation-20260912/reboot/`。
NX 运行 venv 未安装 pytest；本地回归与 NX 只读实测分别记录。

`.7` 已正式安装为 `v2.3.0-go2.20260912.7`，源码 `b5d85bb`，运行会话
`product-06ae924323bd44cdb7bbd196fc9aa65b`。安装后首次 motion 预检在输入门
`recovering` 时拒绝，保留该报告；随后 45.17 s 的 223 样本全部 ready/healthy，
导航与驱动始终零输出，滑窗工作 P95 最大 1.494 ms。地图处理 446 帧，未替换
pending 帧；稳定后的 motion 预检 37/37 通过。本地针对性回归 97/97 通过。
Web 扫描流 8 s 收到 31 个连续非空帧，每帧 2708–2909 点，frame 为 map。
重启后厂商避障回读 true，按既有要求关闭并回读 false，LingTu 碰撞检查保留。
随后打开 0.50 m/s 用户 WASD 窗口，助手未发送非零指令。空地响应、固定纸箱
绕行和松键停车仍等待本次现场反馈，不属于上述静止/只读检查的通过项。
证据：NX `/home/unitree/lingtu-validation-20260912/deployed/` 与 `motion/`。

### 2026-09-12 运动后停滞：短直行候选遗漏

用户报告 `.7` 能移动，但容易停滞。读取当时仍保留的失败快照（sequence 16，
同一 `.7` 会话）发现请求为后退 180°：机身占据查询为自由，远端目标查询也自由，
但二者的连接被占据格挡住。完整 Grid 查询显示后方约 0.65 m 可达，左右候选
方向通常只有 0.05–0.35 m 净空，不足最短 0.5 m 候选。以上距离属于当时已经
膨胀的碰撞图，不是原始雷达到物体的测距，也不能证明占据格是真障碍还是残影。

原 FSM 在左右目标耗尽后回到 3.5 m 参考终点，因为终点本身自由，不会把连接
缩短到障碍之前。独立完整地图回放连续 64 次仍 `rebound_restart_limit`。
修复在原有左右候选之后加入同一请求方向的逐级短目标，最多多 4 次扫掠查询；
同样要求整段双圆柱净空，继续优化、动力学与最终运动检查，未增加自动反向动作。
这是避免漏掉近处可行目标，不代表能够穿越前方障碍或自动从任何窄处脱困。

本地 66/66 SCAN 回归通过。保存的相同完整快照修复后第 4 次规划返回 Ready，
轨迹 255 个采样及相邻扫掠段零碰撞，峰值速度约 0.379 m/s（输入上限 0.5）。
NX ARM 66/66 回归也通过；相同快照得到相同轨迹与采样结果。修复源码快照
`7b9f986` 已通过正式安装器发布为 `v2.3.0-go2.20260912.8`。
部署后会话 `product-8e8a75ef090f46bcbb45d8f69c8bac96` 的 45.17 s / 223 个
静止样本全部 ready/healthy、导航和驱动零输出，工作 P95 最大 1.523 ms；mapd
处理 441 帧，无 pending 替换，motion 预检 37/37 通过。随后重新打开 0.50 m/s
WASD，并在 `motion-v8/` 启动有界 15 分钟只读采集，现场动作仍待用户复测。
本轮最初的 240 s 采集发生在用户运动之前，1187 个样本均零输出；不能将它作为
用户刚才运动的历史记录。随后保存了当前状态和失败快照，并重新启动采集。
证据在 NX `/home/unitree/lingtu-validation-20260912/stalled/`，本地
`build/go2-live-validation-20260911/stalled-replay/`。实机运动效果仍待更新后复测。

## 2026-09-11 上一版 .4 现场验收：Web 已部署，绕障未通过

该次安装的 release 为 `v2.3.0-go2.20260911.4`，发布源码快照 `7b30d4d`。
通过正式 native release 安装器及 ProductControl 恢复 `teleop_avoid + SCAN`，
DDS domain 0，会话 `product-442d6d7051ac4942a2ab5681a50af89d`。
本版部署 Web/Gateway/SDK 的免登录工作台；原生导航、地图、SLAM 和驱动二进制
复用 `.3`，没有把本地射线融合补丁冒充已部署。账户凭据不写入本文件。

实机入口为 `http://127.0.0.1:15050/?observe=1`。无凭据访问 health、bootstrap、
导航状态和定位状态均返回 200；浏览器实际显示已连接、Go2 mesh、配准扫描和有效定位。
`15173` 是未连接 NX 的本地 Vite 预览，不能用它判断实机状态。

安装后的无运动采集持续 45.05 s，共 222 次状态采样：输入门全部 ready、循环全部
healthy，导航和驱动输出均为零。里程计/碰撞帧龄 P95 分别为 90.61/90.60 ms，
导航报告的循环工作耗时 P95 最大 7.91 ms；地图处理 436 帧，无 pending 帧替换。
另一次 5 s 原生 DDS 订阅测得雷达约 9.99 Hz、IMU 200.26 Hz、SLAM 10.20 Hz、
局部碰撞约 7.25 Hz、速度输出 99.62 Hz。这些是静态证据。

专用 motion-stage 只读预检本次 37/37 通过，但保留了启动前两份
`motion.correlated_driver_ack` 失败报告：独立 JSON 快照的采样相位及固定两序号
容差仍会误判。没有修改原生门槛。安装后 222 个样本均收到同 producer 的新鲜、
已接受回执，221 对相邻样本中，后一回执均达到或超过前一输出序号；这证明持续
接纳新输出，不证明每条中间序号逐一送达，也不是精确端到端延迟测量。

重启后厂商避障回读为 `true`。按用户既有要求调用 `SwitchSet(false)`，返回 0，
回读 `false`；LingTu 的 `check_obstacle=true`、`manual_mode=false` 保持有效。
重新打开 0.50 m/s WASD 窗口，由用户操作，助手没有发送非零运动请求。

随后实际操作者输入采集 239.76 s、1188 个样本，其中 391 个有非零有效输入。
全部输入门仍 ready，非零输入的最大请求帧龄约 43.91 ms，最终平移指令峰值
约 0.50 m/s。实际运动期间出现 51 个 `scan_local_target_blocked`、61 个
`scan_initialization_failed` 和 2 个 `scan_emergency_stop` 样本；这些是状态采样
次数，不是独立故障次数。保存了 33 份优化失败快照，包含时间调整后动力学超限、
调整后碰撞和重复碰撞重启达到上限。循环另有 22 个负载告警、60 个期限未满足
告警样本；最大报告规划耗时 37.94 ms，循环工作耗时 P95 最大 12.87 ms。
末帧请求、最终输出、驱动指令均为零，SDK 平移速度为零。

**结论仍为绕障未通过，不能因为能够移动而关闭问题。** 用户现场反馈容易卡住。
本轮包含转向、前进、侧移和后退，不是严格的单次空地 4 s 直行实验，不能据此
给出稳态速度跟踪验收。当前记录排除了这段采样中键盘超时或输入门关闭，不能
排除规划/最终检查限速与底盘跟踪误差。

### .4 为什么局部目标受阻不等于四周无路

- `src/nav/cpp/navigation/executor.cpp` 将持续方向输入变成约 3.5 m 的两点直线
  参考，并沿原方向推进目标；这不是先在局部地图中求得的绕障引导路径。
- `SCANReplanFSM::getLocalTarget()` 只在这条参考曲线上向前/向后寻找自由目标。
  没有可用点时直接置 `localTargetBlocked_`；`Backend::run()` 返回
  `scan_local_target_blocked`，该次不会进入 rebound/DynAStar 优化。
- `.4` 的 SCAN 接收的 `maxDirectionDeviationDeg=90` 只参与有效性/参考身份判断，
  没有用于展开左右替代目标；CMU 的方向搜索不能当作 SCAN 已实现的能力。
  因此仅把配置改成 180/360 度不会产生全向目标搜索。
- 当前 Product 的 `recovery.max_attempts=0`，执行器的通用遥控恢复还明确排除
  SCAN。机器人支持侧移、已有弯曲轨迹，不代表存在可用的自动后退脱困策略。

上述缺陷需要 MotionIntent 受阻后的替代目标/绕行引导，并保留真实机身碰撞、
动力学约束和松键停车。前进意图内绕行与向后脱困应分别定义，不能用放宽碰撞
判定或忽略操作者方向代替。当前配准扫描不是完整碰撞栅格；仅凭网页上点云稀疏
不能证明目标附近自由。此次 local-target 受阻发生在优化之前，不会产生新的
优化失败快照，邻近时刻快照不能冒充该时刻碰撞格子的确证。

### 射线漏清修复的验证边界

本地生产源 `rolling_occupancy.cpp` 已取消同端点跳过和共享体素截断，保留每帧
概率更新、真实命中阻挡和原有占用阈值。30 项 rolling-occupancy 用例在 Windows
Debug 及 NX 独立 ARM 编译中通过。NX 保存点云回放的两处实际穿越格由历史
0.98 降至约 0.6231，无穿越对照格仍为 0.98；原始/逆序/穿越优先结果一致。
四帧回放平均约 40 ms/帧，旧实现约 30 ms/帧，不能当作在线 Mapd 性能验收。
该补丁尚未进入 `.4`。Windows mapd-engine 集成用例曾在初次写入后的衰减断言
失败，带状态输出的控制台复测 16 项通过；该时序不稳定仍需处理，不能写成稳定
集成验收已通过。

本轮原始证据在 NX `/home/unitree/lingtu-validation-20260911-bRrcju/`，下载的状态、
DDS/预检报告、ARM 回放与运动失败快照在
`build/go2-live-validation-20260911/field-evidence/`。部署与验收结论以本节为准，
下文保留之前各次实验的时间和边界。

## Web 工作台与官方 Go2 模型（2026-09-11，已随 .4 部署）

新版直接进入实时场景，主 Gateway HTTP、SSE、WebSocket 不再要求 API Key 或
登录 Cookie，即使旧环境中仍设置了相关密钥也不拦截 Web。登录页、登录/检查路由、
接口声明与 SDK 登录方法已删除；独立 MCP 服务的认证不在本次 Web 改动范围内。
只读监控的禁止写入规则、急停和原生运动仲裁仍有效。

工作台统一为中性黑白主题，支持浅色、深色与跟随系统，设置和浮动工具使用毛玻璃。
完整功能通过以下入口使用，不再要求 `debug_nav`：

- 顶部为 logo、现场、地图、巡检任务、更多、连接状态、停止和设置。
  控制台、定位诊断、数据诊断和规划参数保留在“更多”内。
- 场景采用一个 3D 画布与一个右侧面板，右侧按“状态 / 图层 / 操作”切换。
  地图、常用位置、定位工具在同一面板展开，返回后恢复操作入口，不再增加第三栏。
  实测速度、规划与雷达在状态面板查看；位姿数值、控制明细和传感器频率按需展开。
  规划阻断提示在面板外显示，控制权与运动许可在面板底部保留。
- 图层旁明确区分扫描/地图点云与局部安全采样诊断；画面空白不等同于碰撞栅格可通行。
  断开时显示等待连接，缺失速度显示为 `--`；过期导航状态不作为当前控制状态展示。
- 设置只保留外观和系统两类：主题、语言、重置布局、真实模块健康与诊断包下载。
  假 OTA 检查、硬编码版本和未实现占位项已移除。

折叠的是技术详情和低频操作，实际功能入口仍保留。失败、数据过期、运动阻断与
等待停车确认不会藏进详情。页面区分“运行模式”和“巡检任务”，地图就绪检查
也不再声称会切换导航服务。地图卡片的重命名、删除合并到“更多”。

菜单、面板和悬停使用约 160–180 ms 的短过渡，并尊重减少动态效果设置。
外点关闭工作区菜单的那一次点击不触发导航选点或重定位；菜单、设置和普通按钮
获得交互时暂停键盘遥控，按键先松开再重按才会重新输入。遥控面板中的 WASD、
松键保持、失焦保持和 Space 停止继续有效。工作区菜单保持在三维画布上方。

本轮工作台改版通过 30 项针对性回归、TypeScript/Vite 构建和改动范围 ESLint。
浏览器核对了浅色、深色、390 px 窄屏、工具返回、键盘切换与只读入口。
设计规范见 [`web/DESIGN.md`](../../../../web/DESIGN.md)。这些是本地 UI 证据，
当时没有部署 Go2、改变控制参数或执行实机运动；后续现场结果见本页最新验收节。

地图列表现在区分加载中、成功为空和读取失败；失败时保留旧列表并注明数据来源，
不会把连接失败解释为没有地图。地图、巡检、诊断页面继续按需加载。

Go2 使用 [Unitree 官方完整 URDF 与 mesh](../../../../web/public/assets/robots/go2/README.md)，
固定上游版本 `7d6075f7f58588b189b940130e3edab3c839b2df`，原始文件和 BSD 许可随包保留。
包含 42 个 link、41 个 joint、17 个 visual；七个 DAE 网格在一次场景加载中各解析一次，
重复腿部共享几何。模型保持原始米制尺寸、关节原点与轴，场景背景跟随明暗主题。
原模型不包含外加 MID-360 和扩展电脑，不能用它替代实际外参标定。

Go2 型号由运行时 bootstrap 提供；机身跟随真实 XYZ 与朝向，保留真实轨迹采样，
不再次拟合曲线。没有关节遥测时使用明确的固定展示站姿（hip 0、thigh 0.8、calf -1.6 rad），
不用虚构步态代替实测。定位断流隐藏机器人，SLAM 重启或地图坐标跳变时清除旧位姿插值和历史轨迹。
关节遥测接入与其独立的断流处理见下节。

### 2026-09-14：Go2 实测关节显示

库内增加只读关节链路：Unitree SDK2 `rt/lowstate` → Go2 driver → 原生 DDS
`/robot/joint_states` → 现有 HostBus → Gateway `joint_state` SSE → Web 官方 URDF。
不发布 `rt/lowcmd`，不修改运动许可、步态或电机命令。

- 读取 12 个腿部电机的 `q`（rad）、`dq`（rad/s）、`tau_est`（估算力矩，N·m）；
  索引为 FR 0–2、FL 3–5、RR 6–8、RL 9–11，各腿依次 hip、thigh、calf。
  来源：[官方电机索引](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/go2/low_level/unitree_legged_const.py)。
- 驱动只发布新鲜新样本，发布频率上限 30 Hz，接收时刻保持原样；遥测不成为运动就绪条件。
  HostBus/SSE 实际频率还受各层轮询与传输限制，这个上限不是实机测量结果。
- 网页首个有效样本直接对齐实测角；连续样本按接收间隔做 16–80 ms 线性插值，
  没有角度外推。每次按 URDF 原点和轴重建旋转，避免多次更新累计角度。
- “关节姿态”显示“实测 · 12 个关节”；超过 500 ms 无新鲜数据显示
  “数据过期 · 保留姿态”。初次未收到数据时显示“展示站姿 · 等待数据”。
  新鲜度使用后端源数据年龄加浏览器单调接收时间，不直接比较两台机器的时钟。

本地交互样例（明确使用合成角度，不连接机器人）：
`http://127.0.0.1:15173/@fs/D:/inovxio/brain/lingtu/build/go2-live-validation-20260913/mapping-observation-replay.html?observe=1&joints=1`。
顶部“关节流”按钮用于检查断流冻结和恢复；生产网页只使用实际遥测。
此项尚未在 NX 安装与实机逐关节对照验收；上线需同步更新 driver、native client、Host 与 Web。

本次验证：

- 真实 Unitree SDK 的 `lingtu_driver` 在 WSL 编译、链接通过；Go2 acquisition 与
  joint telemetry 原生测试 2/2 通过。新增本机 DDS → Client → C ABI 往返测试通过。
- 关节桥接、SSE 队列、时间戳及相邻接口 62 项 Python 测试通过；随后补齐可选
  topic 端口声明与筛选订阅，real map/nav/teleop 和 sim nav 的 4 项集成测试通过，
  既有 odometry 筛选订阅回归通过。可选遥测不会变成 Product 必需条件。
- 前端关节映射、连续更新、断流恢复、二维投影高度及关联显示 27 项测试通过，
  TypeScript、生产构建与修改文件 ESLint 通过。关节事件通过独立显示数据流更新，
  不让整个 App 随关节频率重绘。保留既有 Three.js 分包体积告警。
- 2026-09-14 开机后，NX 的 `eth0` 只读探针 5 秒收到 2494 条 LowState、2491 个
  不同 tick，12 组 q/dq/tau_est 均为有限值。使用
  [go2_joint_probe.cpp](../../../../tools/diagnostics/go2_joint_probe.cpp)，没有发送运动命令。
  该证据证明官方关节源可读，不代表新版完整 Product 已安装运行。

二维底图穿过机身的显示问题也已修正：原生 occupancy 是 XY 投影，origin Z 默认 0，
不能据此认定地面就在地图 Z=0。Web 对有效 Go2 位姿将二维观测图、通行图与参考网格
显示到默认站姿的脚下参考高度（由原始 URDF 推得约 0.297 m 的足部中心偏移），
已有更低的投影保持原位。这只是显示参考，不是测得地面或新的支撑数据。
点击射线使用相同显示平面，仍返回原始 XY；规划查询高度、目标高度、实际点云和高程不改动。
离线样例则明确以机身 Z=0.34 m、合成地面 Z=0 m 建立相对位置。

以下是此前界面改版的历史验证，不作为新增关节链路的验收证据：
前端 187 项回归全通过，无跳过，包含工作区入口、只读限制、地图失败状态、
URDF 结构、位姿插值、菜单关闭时阻止选点穿透、菜单与遥控按键隔离。
TypeScript/前端构建和修改组件 lint 通过；应用入口约 253 kB，Three.js 分包约
563 kB（均为压缩前大小），后者仍触发大于 500 kB 的体积提醒。
此前同一更新的 Gateway 相关 190 项用例、SDK 60 项及 24 个子测试已通过；本轮视觉调整未改动这些后端文件。
浏览器已检查免登录入口、黑白主题、毛玻璃设置、工作区菜单在画布上方完整展示。
之前一轮已检查恢复的地图工具和官方 Go2 实际 DAE 渲染。
模型演示明确标为本地演示，未连接机器人。

本机开发预览为 `http://127.0.0.1:15173/`（需要本地 Vite 开发服务）；模型独立预览位于
`build/go2-web-refine-20260911/model-preview.html`，不作为产品入口发布。
此预览服务未连接 NX，所以状态显示离线，不能据此执行实机验收。

改版完成时尚未切换：当时 Sunrise 可 SSH 登录，但其 eth0/eth1 都为 `NO-CARRIER`，
NX 不可达，本机 15050 转发返回 503。没有修改网络或重启设备。
当时机器人侧为下节记录的 `.3`。之后物理链路恢复，已通过正式 native release /
ProductControl 流程部署 `.4`，免登录和实机数据连接结果见本页最新验收节。

## 实时监控

机器人已经启动 Product 后，在开发电脑运行：

```powershell
.\config\robots\unitree\go2\start-monitor.ps1
```

入口为 `http://127.0.0.1:15050/?observe=1`。脚本复用既有 Go2 SSH 配置，
只把本机回环端口转发到 NX Gateway；不启动 Product、不获取运动控制权。
新版 Web 无需密钥登录；机器人侧当前 `.4` 的部署状态见最新验收节。
只建立连接、不打开默认浏览器时加 `-NoBrowser`；端口冲突时可指定 `-LocalPort`。

监控页默认进入场景，并集中显示当前配准扫描、规划轨迹、地图坐标下的机器人
位置和朝向、雷达/IMU/定位频率、请求速度、最终输出和规划状态。
配准扫描通过原生 SLAM 同一扫描时刻的位姿转换到地图坐标；它是当前帧，
不是累计地图，也不是未经 SLAM 过滤的全部原始回波。
当前 Product 没有输出累计地图点云时，仍可单独查看配准扫描。

轨迹以原生导航快照为准；没有有效轨迹时显示等待或失败原因，不用旧轨迹冒充
当前执行路径。断流/过期数据会明确提示。`observe=1` 页面不提供点击导航、
重定位、地图编辑或遥控入口；鼠标仅用于视角操作。运动试验使用独立 WASD 窗口。
只读是本页面的交互约束，不改变原生运动控制边界。

2026-09-11 已部署到 `v2.3.0-go2.20260911.3`（源码快照 `57cb01f`）。
验证包括后端 44 项、前端 20 项测试、前端构建和 NX 上 8 项无运动检查；
现场只读订阅连续收到 2843、2839、2816 点的 `registered_scan`，坐标为
`map`，序号 79→80→81，源时间推进，帧龄约 0.11–0.33 秒。定位为
`TRACKING`，处理扫描约 10 Hz、IMU 约 200 Hz，严格 teleop_avoid 就绪检查通过。
当时没有运动请求或有效轨迹，最终速度为零；这些证据只证明监控数据接通，
不证明绕障验收通过。现场证据位于 NX 源码目录
`build/go2-live-monitor-evidence.json` 和 `build/go2-live-monitor-preflight.json`。

此次还修正了现场地图查询连接地址：mapd 实际监听
`/run/lingtu-mapd/mapd.sock`，Host 曾按 `LINGTU_SESSION_ROOT` 推导成
`/run/lingtu/mapd.sock`，使地图操作查询返回 503 并阻挡 Product 切换。
已在 Host 与 mapd 共读的 `/opt/lingtu/config/go2-native.env` 持久化
`LINGTU_MAPD_QUERY_SOCKET=/run/lingtu-mapd/mapd.sock`。确认原生地图保存任务为空后，
通过 ProductControl 停止并正式安装、重启；地图操作查询恢复 200，任务数为零。
本次 Product 会话为 `product-f0e37ad2a53e418ba800fd902a93b2e8`。

## 连接 NX 与网络核对

Go2 扩展 Linux/NX 电脑是 LingTu 与外接 MID-360 的正式运行主机，
Sunrise 只是临时开发跳板。历史连接拓扑如下：

```text
Windows
  -> Sunrise: Wi-Fi 192.168.66.65 / 有线 192.168.123.99/24
  -> NX: unitree@192.168.123.18 / eth0
```

[go2 历史任务](codex://threads/01a0149a-ca4f-7a90-a1b1-60571f7eaad4)
在 2026-08-22 记录了从 Sunrise 成功 SSH 登录 NX。`unitree` 是这台
NX 的历史账号，不是所有 Go2 的通用账号；密码使用这台机器交付时的凭据。
这条历史记录不代表设备当前在线。

2026-09-10 本轮已实际登录 `sunrise@192.168.66.65`，再从 Sunrise
成功登录 `unitree@192.168.123.18`。Windows WLAN 为 `192.168.66.62/24`。
早期 Sunrise 密钥认证曾被拒绝，当时通过已有设备密码完成两跳登录。
随后已把既有 `inovxio_deploy` 的公钥加入 Sunrise 和 NX 两个账户，
非交互 SSH 与 SCP 双跳均验证成功；当前不再依赖该次密码登录流程。
私钥仅保存在开发电脑的 `.ssh` 目录，凭据不保存在本文或仓库中。

当时不能连接 NX 的原因是 Sunrise `eth0` 有载波，但 DHCP 配置没有获得
IPv4 地址；访问 `.123.18` 的路由错误地走向 Wi-Fi 默认网关。
本轮先通过 NetworkManager 的内存连接 `lingtu-go2-check` 临时恢复
`eth0=192.168.123.99/24`，设置 `ipv4.never-default=yes`，没有设置网关，
Wi-Fi 连接保持正常。随后用 `sudo nmcli connection modify` 将该连接改名为
`lingtu-go2-eth0`，启用 `connection.autoconnect=yes`，自动连接优先级设为
`100`，保留 manual IPv4、`.99/24`、never-default、无网关和禁用 IPv6。
配置已写入
`/etc/NetworkManager/system-connections/lingtu-go2-eth0.nmconnection`；
执行 `connection reload` 后，已再次成功从 Sunrise 登录 NX。

Sunrise 后续复用已有的 `lingtu-go2-eth0` 配置，不再创建临时连接。
以下命令仅适用于已确认 `eth0` 连接机器人内网、且 `.99` 仍由该跳板使用的现场。
若现场配置仍保留原名 `lingtu-go2-check`，先执行
`sudo nmcli connection modify lingtu-go2-check connection.id lingtu-go2-eth0`
完成改名，再修改并启用已有配置：

```bash
sudo nmcli connection modify lingtu-go2-eth0 \
  connection.interface-name eth0 connection.autoconnect yes \
  connection.autoconnect-priority 100 \
  ipv4.method manual ipv4.addresses 192.168.123.99/24 \
  ipv4.gateway "" ipv4.never-default yes ipv6.method disabled
sudo nmcli connection reload
sudo nmcli --wait 8 connection up lingtu-go2-eth0
ip route get 192.168.123.18
```

正确路由应为 `dev eth0 src 192.168.123.99`。这里是开发跳板的 `.99`，
NX 本身继续使用 RobotConfig 中的 `.18`。NX 原有的持久化配置同样名为
`lingtu-go2-eth0`，使用 `eth0`、manual `192.168.123.18/24`、
`autoconnect=yes`、`never-default=yes`，没有设置网关；两个同名配置分别属于
Sunrise 和 NX，不能把两台主机的地址互换。

| 本轮实测 | 结果 |
| --- | --- |
| NX 主机 | 用户 `unitree`，Ubuntu 20.04.5，`aarch64`，`eth0=192.168.123.18/24` |
| NX → Go2 主控 `.161` | 2/2 ping 收到，平均约 0.22 ms |
| NX → MID-360 `.20` | 2/2 ping 收到，平均约 2.49 ms |
| `/opt/lingtu/current` | 2026-09-11 已确认指向 `/opt/lingtu/releases/v2.3.0-go2.20260911.3`，发布源码 `57cb01f`；实时监控与现场连接修复见上节 |
| ProductControl | 新版 `teleop_avoid + SCAN` 已启动，DDS domain 0，session `product-79162a3b12514c9da00c865eb9b7ac84`；严格 motion 只读门通过、blockers 为空。随后现场 W 验收失败：用户报告不移动，42 条有效输入快照的最终指令全部为零，当次起点碰撞导致停车；实际绕障与速度跟踪未通过 |
| 本轮新版部署 | `.2` 包含真实测量射线清空修复、持续 W 失败后恢复及键盘超时后的松键重新解锁；本地地图 28 个用例、键盘 33 项，以及 NX 地图 2 个测试入口、SCAN 7 项针对性回归通过。此前 `.1` 的 426 项完整导航回归是历史证据，不等同于本轮现场运动验收 |

本轮已完成开发跳板网络持久化、运行依赖安装、DDS 运行配置落盘和新版 release 切换。
`.8` 已完成完整 Product 启动；其 22 个原生程序与通过 120 秒静止检查的 `.7`
逐字节一致。未发送非零运动指令，也未调用 StandUp。
网络连通和登录成功不等于运动 readiness 通过。

| 地址或参数 | 用途与来源 |
| --- | --- |
| NX `192.168.123.18/24`、`eth0` | 当前 `robot.yaml` 的机器人侧静态地址、Go2 SDK2 网卡和 MID-360 host 地址；现场按 NX 实际网卡核对 |
| Go2 `192.168.123.161` | `driver.probe_ip`，用于连通性探测，不是 SDK2 的显式运动目标地址 |
| 外接 MID-360 `192.168.123.20` | `lidar.lidar_ip`，提供点云和包内 IMU，与 Go2 内置雷达分开 |
| `sunrise@192.168.66.65`、`192.168.123.99/24` | 本轮已登录的跳板管理入口；机器人侧有线地址已持久化到 `lingtu-go2-eth0`，后续仍按现场核对 |
| `192.168.66.204` | 历史 Go2 Wi-Fi 地址，不是 NX SSH 地址 |
| `http://127.0.0.1:15050/` | 历史本机 Web 隧道入口，转发目标与隧道存续未确认，不是机器人 IP 或固定 Gateway 端口 |

开发电脑能直接到达机器人子网时：

2026-09-13 核对：NX 只有 `eth0`，NetworkManager 未列出 Wi-Fi 设备，USB/PCI
枚举也未发现无线网卡；开发电脑当时只有 WLAN。因此直连还需实际的有线接入或无线桥接，
不能仅删除 ProxyJump。不要改动 NX 的 `eth0=192.168.123.18/24`，它还承担雷达和
Go2 SDK2 通信。

最短路径是电脑 USB 以太网适配器连接机器狗的外部网络口。适配器使用同网段空闲地址
（例如先确认未占用的 `192.168.123.100/24`），网关和 DNS 留空，电脑 Wi-Fi 可继续上网。
无线方案需给 NX 增加已支持的无线接口，或在机器人网络侧接无线网桥；其供电及网口接法
需现场确认。Go2 自带 Wi-Fi 地址不能直接当作 NX 的地址。

本机已准备独立别名 `lingtu-go2-nx-direct`，沿用原密钥与主机校验，明确禁用跳板。
原 `lingtu-go2-nx` 跳板入口保留。以下入口在直达网络接好后使用；创建别名不代表连通验收：

```powershell
ssh -F "$env:USERPROFILE/.ssh/lingtu-go2.conf" lingtu-go2-nx-direct
.\config\robots\unitree\go2\start-monitor.ps1 -SshTarget lingtu-go2-nx-direct -LocalPort 15051
```

单独使用 15051 避免误复用已有的 Sunrise 隧道。确认当前 Product 和 DDS domain 后，
`start-wasd.ps1` 同样接受 `-SshTarget lingtu-go2-nx-direct`；不可把旧会话 domain 当作当前证据。

```powershell
ssh unitree@192.168.123.18
```

开发电脑已持久化 SSH 配置
`C:/Users/99563/.ssh/lingtu-go2.conf`，别名为 `lingtu-go2-sunrise` 与
`lingtu-go2-nx`，后者通过前者跳转。后续连接优先复用这份已验证配置：

```powershell
ssh -F "$env:USERPROFILE/.ssh/lingtu-go2.conf" lingtu-go2-nx
```

连接资料保留在本节和开发电脑 `.ssh` 中，不复制到构建或验收 artifacts。
需显式指定历史账号和地址时，可使用：

```powershell
ssh -J sunrise@192.168.66.65 unitree@192.168.123.18
```

也可先 `ssh sunrise@192.168.66.65`，登录后再执行
`ssh unitree@192.168.123.18`。Sunrise 的有线接口历史上出现过 `eth0` /
`eth1` 变化，应在该机器上用 `ip -br -4 addr` 和 `ip route` 确认；
不要把 NX 的 `.18` 地址配置给开发电脑。

登录 NX 后，先执行只读的主机、网络与 Product 状态核对：

```bash
hostname
whoami
uname -m
cat /etc/os-release
ip -br -4 addr
ip route
ping -c 3 192.168.123.161
ping -c 3 192.168.123.20
sudo bash -c '
  set -a
  . /opt/lingtu/config/go2-native.env
  set +a
  cd /opt/lingtu/current
  bash scripts/lingtu --robot unitree/go2 --env real status --json
'
```

最后一条命令要求 NX 已安装 LingTu release。本轮普通 `unitree` 用户读取
`/run/lingtu/current.json` 被权限拒绝，使用 sudo 后成功；不需要放宽文件权限。
`go2-native.env` 必须在 sudo 之后的 Bash 中加载，并用 `set -a` 导出给
ProductControl 子进程，避免 `LD_LIBRARY_PATH` 被 sudo 剥离。后续
`switch`、`stop` 也沿用这个加载方式。
NX 登录时若出现旧 ROS 选择菜单，可以 `Ctrl+C` 跳过，本项目仍走原生 DDS。
Ping 只能检查 IP 连通性，
不能证明 DDS、传感器或 Product 已就绪。SDK2 使用
`driver.network_interface` 选择网卡并进行 DDS 发现，不使用目标 IP；
不要直接运行 SDK 运动示例来检查网络。

地址和接口以 [RobotConfig](./robot.yaml) 及机器现场配置为准，
SDK2 接线见 [Go2 Driver](../../../../src/drivers/real/motion/robots/unitree/go2/go2.cpp)。
Product 启停、服务诊断、历史调试结论和无运动／运动验收边界见
[运维说明](../../../../docs/operations.md#go2-connection-and-debug-records)。
连接信息保存在本 README；凭据不写入仓库，地址不复制进 Product YAML。

## 2026-09-10 部署准备与修复记录

NX 已持久化 `/opt/lingtu/config/go2-native.env`，内容为：

```bash
LINGTU_CYCLONEDDS_PREFIX=/home/unitree/cyclonedds_ws/install/cyclonedds
LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib:/opt/lingtu/current/lib
LINGTU_MAPD_QUERY_SOCKET=/run/lingtu-mapd/mapd.sock
```

以下九个非 Driver 服务均已写入
`/etc/systemd/system/<unit>.d/90-go2-native.conf`，用
`EnvironmentFile=/opt/lingtu/config/go2-native.env` 读取该文件：
`lt-lidar.service`、`lt-slam.service`、`lt-maps.service`、`lt-terrain.service`、
`lt-nav.service`、`lt-camera.service`、`lt-explore.service`、`lt-gnss.service`、
`lt-host.service`。`systemctl daemon-reload` 已完成；写入这些配置不代表服务已经启动。
Go2 Driver 继续使用 Unitree SDK2 的 DDS 库，没有套用上述非 Driver 配置。

本轮以真实 `lingtu` 账号对安装后的 `slamd` 执行 `ldd`，确认加载的是
project CycloneDDS 路径中的库。此前空环境会错误选择另一套
`/usr/local/lib` 中的库。终端中的环境变量不会自动传给 systemd，CLI
也需按上节方式在 sudo 后加载并导出配置。

新 venv 位于 `/home/unitree/lingtu-venv-20260910`。已离线安装当前
`uv.lock` 的 core + gateway 共 18 个运行时 wheel，`pip check` 和导入检查通过，
以 `lingtu` 服务账号执行也通过。统一服务安装器已将
`/opt/lingtu/config/python.env` 写为这个新 venv 的 `bin/python`，后续 CLI 与
Host 使用此持久化路径。

部署前备份保存在 `/opt/lingtu/backups/20260910-before-native-release`，
包含旧机器配置、systemd units 和 previous current 记录。原
`/home/unitree/lingtu-20260826-teleop-session` 与上一版 release 保留用于回退；
旧 Product 服务已由统一服务安装器替换或停用，地图与现场标定保留。
本轮发布基于本地 `c37e9e0` 加当前工作区修改，NX 隔离源码快照为
`7213047`（`.8`）。后续 `.9=bbf3653`，本次 real 修复在其上形成
`.10=b5763a1`，Host 查询性能修复为 `.11=82c9895`；本轮 `.12` 的发布 metadata
确认源码为 `40c8b93c0a20b4f075b0370f03f9a4c1502fc4a6`。这些是现场隔离源码快照，
不是 GitHub 已合并版本的声明。

| 本轮修复 | 当前证据与范围 |
| --- | --- |
| 停止态 release 升级 | 修复已有 `current` 链接但无当前 RunPlan 时错误拒绝安装的问题；保留失败回滚与不启动 Product 的行为 |
| 网页打包 | 补入被 Git 忽略的已构建 `web/dist`，避免只打包网页源码 |
| Livox Release 测试 | 保留 assert 测试断言，修复 `NDEBUG` 消除断言后触发的未使用变量／参数编译错误 |
| Rolling occupancy | 修正 aarch64 上 `int64_t` 与 `long long` 混用导致的 `std::min` 类型不匹配 |
| Camera DDS | Linux 目标补充 `librt` 链接，解决 Ubuntu 20.04 的 `shm_open`／`shm_unlink` 未定义符号 |
| 保存时 PGO | 该入口迭代预算由 30 提高到 200，并修正小角度 Taylor 展开；NX 对应 3/3 测试通过 |
| `teleop_avoid` 验收器 | 移除旧 traversability 必选约束，匹配当前 SCAN/CMU；相关 gate 与 collector 共 82 项本地测试通过，保留趴卧、控制就绪和零输出门槛 |
| SCAN 浮点绝对值 | ARM 上旧整数 `abs` 导致死循环，gdb 确认 `fcvtzs`；改为 `std::abs` 后，原卡住超过 7 分钟的 NX 测试在 0.34 秒通过，相邻轨迹测试在 0.11 秒通过 |
| CMU 绕障方向 | 使用已有引导目标重新投影到当前机体系，避免绕障转向后持续偏离最初接纳的走廊；原物理净空与回归走廊测试通过，NX 导航整套 410/410 通过，用时 55.70 秒 |
| 导航启动残留参数 | 删除 wrapper 对 `LINGTU_TELEOP_SLOW_DISTANCE_M` / `LINGTU_TELEOP_STOP_DISTANCE_M` 的陈旧必填检查；编译器与 C++ 已移除这两个旧参数，继续保留真实会话、机器人几何与规划器检查 |
| 驱动开机自启 | Driver 与其他 Product 进程统一由 ProductControl 启动；安装器清理旧 canonical unit 的自启链接，不停止正在运行的 Product，避免重启后缺少 `/run/lingtu/session.env` 而报错 |
| Host 状态路径 | 实机也设置 `LINGTU_SESSION_ROOT=/run/lingtu`，不能据此推断为仿真；SLAM 与导航状态读取仅在 `LINGTU_ENV=sim` 时使用会话内文件，实机默认路径继续匹配原生服务，显式路径配置优先 |
| Host 避障就绪契约 | `use_traversability_cost` 与当前 RunPlan 严格比对，修复把 SCAN/CMU 辅助遥控一律判为必须开启该代价的旧约束；保留点云避障、输入门和控制权检查 |
| 最终启动检查接口 | ProductControl 改为读取 `/api/v1/readiness` 中的原生就绪判断；DDS 原始快照没有 `ok`、`status_available`、`blockers` 字段，不能当作判断结果使用。超时报错附带真实阻塞与循环健康信息，保留控制循环健康门槛 |

本轮从新编译目录执行的 MID-360 无运动验收已通过：原始点云约 9.99 Hz、
IMU 约 200.27 Hz、里程计约 10.07 Hz、配准点云约 9.97 Hz，SLAM 为
`TRACKING`，`rt/nav/cmd_vel` 样本数为 0。该证据只覆盖新编译的传感器与
SLAM 链路，不代表真机运动验收通过。

从 `/opt/lingtu/current/bin` 运行安装包内的相同无运动验收也已通过：
原始点云 10.00 Hz、IMU 200.28 Hz、里程计 9.98 Hz、配准点云 9.98 Hz，
SLAM 为 `TRACKING`，`rt/nav/cmd_vel` 样本数为 0。
NX 还通过 Go2 Driver 3/3、录制 14/14、保存优化 3/3、Livox 5/5 测试。
安装后的 22 个 ELF 可执行文件已按各自 DDS 库环境、以 `lingtu` 账号验证动态库解析。

优化前启动回放（NX `build/go2-start-readiness-f.jsonl`）确认：Host 的
`ready/data_ready/motion_ready` 均为 true、reasons 为空；机器人状态为
`standing`，零速度回执已接受，导航为 `IDLE/NONE/CLEAR/QUIET`。
这仍不能替代最终运动验收：100 Hz 循环的 600 周期窗口中超期约
9.2%–9.8%，工作耗时 p95 约 11.9–12.2 ms，超过 10 ms 周期。
点云到障碍层更新最近一次约 8.2–9.9 ms，障碍快照约 2.7–2.9 ms，
全局规划耗时为 0。当时性能阻塞发生在静止时的传感器处理，不能归因于正在寻路。
不降低健康门槛，也不以 Host 单独 ready 作为放行运动的依据。

`.5` 发布包的最终启动结果保存在 NX
`/home/unitree/lingtu-source-20260910/build/go2-product-start-g.json`：
状态读取和原生契约已通过，当时阻塞为 `deadline_miss_ratio_high`。
最近 600 个周期中 63 个超期（10.5%），工作耗时 p95 为 13.22 ms、
p99 为 16.71 ms。这是优化前的历史结果，后续 `.6` 的结果如下。

### 控制时序优化与静止验收

`MotionLayer::snapshot` 对每个候选障碍点缓存一次本次快照的距离，
避免 `partial_sort` 在比较时反复读取点并计算距离；仍保留最近的障碍点，
位置变化后重新计算距离。NX 上同一批 267 帧实测点云、同一频率配置下，
快照 p95 从 2.01 ms 降到 1.16 ms，约减少 42%。新增位置变化、三维距离、
高度保留与无上限输出回归；导航 410/410 测试通过，用时 53.01 秒。

`.6` 的 ProductControl 完成 `native_nav_ready`、`slam_ready`、
`goal_acceptance_ready`、`motion_output_ready` 并提交 active 会话。
随后 120.46 秒、121 次静止采样全部通过：

| 指标 | `.6` 实测 |
| --- | --- |
| 控制周期 | 100 Hz，原 p95 利用率 0.90、超期比例 0.05 门槛保持不变 |
| 600 周期滑窗中的最高工作耗时 p95 | 8.675 ms |
| 最高滑窗超期比例 | 1.0% |
| 里程计 / 配准点云 | 9.936 / 9.903 Hz |
| 采样中的导航和驱动输出 | 全为零，驱动 ready 且 ACK accepted |
| 目标 / 指令请求 / 遥控输入计数 | 全为 0 |
| 原生导航进程 | 同一 producer，未重启 |

证据在 NX `build/go2-product-start-cached.json` 和
`build/go2-idle-soak-cached/summary.json`，相对路径根目录为
`/home/unitree/lingtu-source-20260910`。
这证明静止运行，不证明实际行走、绕障或紧急停车已经验收。

严格辅助遥控验收随后指出两个问题，已在 `.7` 修正：诊断原来把 SCAN 判成必须接收
CMU 的 cloud gate，并额外要求原生 TeleopAvoid 未启用的第二层定位健康门；
同时 idle teleop 与通用空闲保活每周期重复发送零速度，导致约 199 Hz
输出和过大的回执序号差。修正匹配 `inputGateConfig` 的原生契约：
SCAN 检查新鲜的局部碰撞栅格，CMU 检查点云；两者均保留里程计、
驱动与输入就绪检查。状态补充栅格必需标志、年龄和时效门槛。
空闲保活只在本周期没有成功发送零速度时补发；验收器 71 项测试通过。

`.7` 的严格 `teleop_avoid` motion-stage 只读门禁通过，blockers 为空。
该检查读取状态与回执，不会发送非零命令；名称中的 motion 不代表已完成行走测试。
更新后的 NX 导航 410/410 回归通过，用时 52.94 秒。
第一轮 120 秒采样出现一次 `/ready` HTTP 503，当时保存的原生控制健康和驱动回执
仍正常；原采集器没有保留该次响应体，不能据此确定原因或声称已修复。
补充失败响应体记录后的第二轮 120.42 秒、121 次采样全部通过：
最高滑窗 p95 8.809 ms、最高超期比例 1.3333%，里程计 9.676 Hz、
配准点云 9.651 Hz，输出 99.410 Hz，输出数 / 控制周期数恰为 1.0。
输出仍全为零，目标和操作指令计数为 0。证据为
`build/go2-idle-soak-detail/summary.json`；单次 503 原因仍需更长时间采样定位。

综合 doctor 检查另修正两项工具问题：从已有 `LINGTU_API_KEY` 环境变量
发送 `X-API-Key`，避免受保护 API 的 HTTP 401 被错误解释为多项机器人故障；
SLAM 就绪依据实际速率和里程计，网页显示用的点云缓存数量仅作诊断信息。
网页无人订阅时缓存为 0 不代表原生 mapd 或 SLAM 没有数据。地图处理与发布仍由
辅助遥控 preflight 的 mapd 计数、generation、DDS 发布和容量检查负责。
修正后的 source doctor 与 `.8` 安装包内 doctor 均在 NX 通过，
0 fail、3 个未选择相机的可选 warning；
工具相关测试共 102 项通过（辅助遥控 71 项、doctor/collector 31 项）。

最终证据：`build/go2-product-start-release8.json`、
`build/go2-service-ready-release8.json`、`build/go2-doctor-release8.json`、
`build/go2-final-native-equivalence.json` 与 `build/go2-deployment-final-summary.json`。
最终当前状态保存在 `build/go2-product-status-release8.json`。
`.8` 相对 `.7` 只更新诊断工具和说明，原生导航测试与持续静止证据可对应到同一程序。
持久化设置已落盘；Product 会话不持久化，整机重启后仍需按上文通过 ProductControl
显式启动。下一阶段是现场监督下的低速直行、侧移、松手停车、失联停车、绕障和急停，
以及延长就绪状态采样以查明此前单次 503。

使用现场已保存的凭据运行只读诊断，不在命令参数或输出中打印密钥：

```bash
sudo bash -c '
  set -a
  . /opt/lingtu/config/go2-native.env
  . /etc/lingtu/gateway.env
  set +a
  export PYTHONPATH=/opt/lingtu/current/src
  /home/unitree/lingtu-venv-20260910/bin/python -m diagnostics.field.doctor \
    --env real --non-motion --json --strict
'
```

### NX 性能配置持久化

本机已核实为八核 NVIDIA Orin NX（tegra234），CPU 有 policy0 / policy4
两个调频域，现有功耗模式为 MAXN。以下是本机部署配置：

| 进程 | CPUAffinity |
| --- | --- |
| LiDAR / SLAM / maps / Host | 0–3 |
| nav | 4–6 |
| driver | 7 |

六个 unit 的亲和性写入 `/etc/systemd/system/lt-<name>.service.d/91-go2-cpu.conf`。
原 `/run/systemd/system/.../95-go2-cpu-test.conf` 临时文件已删除。
`/opt/lingtu/config/go2-compute.sh` 将 policy4 设为 `performance`，并按
本机 NVIDIA `jetson_clocks` 的 EMC 设置方法锁定内存频率：取 `max_rate`
与正数 `emc_iso_cap` 中较小值，本轮为 3,199,000,000 Hz。CPU 仍受当前
1,984,000 kHz 上限约束；没有更改功耗模式、GPU 设置或温控策略。

`/etc/systemd/system/lingtu-go2-compute.service` 已 enable，开机应用硬件设置。
nav / driver 通过 `Requires` 和 `After` 确保先加载配置，配置失败时不会
直接启动这两个服务。已在 Product 停止时恢复原调频值，再重启此配置服务，
确认重新得到 `performance`、EMC lock=1 和预期频率；没有执行整机重启。
这是 NX 机器设置，不属于 RobotConfig 的物理几何，也不属于 Product 模式声明。

```bash
systemctl status lingtu-go2-compute.service --no-pager
systemctl show lt-nav.service lt-driver.service -p CPUAffinity
cat /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
sudo cat /sys/kernel/debug/bpmp/debug/clk/emc/rate
```

本轮新增启动与状态回归：nav wrapper/config 32 项、服务安装 8 项及
7 个 Bash 升级场景、Gateway 状态路径 20 项、SLAM adapter 7 项、
Gateway 就绪投影 69 项、ProductControl 最终启动检查 4 项均通过。
网络、Python、DDS 路径与服务配置已经持久化；Product 会话仍是运行时
状态，重启后通过 ProductControl 显式启动，不自动让机器人进入运动。

## 2026-09-10 扩展验收

14:09 用户重新开机后已恢复连接：Sunrise `eth0` 自动恢复为 `192.168.123.99/24`，
carrier=1，NX ping 2/2。14:11 登录 NX，运行约 6 分钟，当时仍为 `.9`，
ProductControl 返回 `stopped`；driver/nav/slam 均 inactive，driver 仍为 disabled。
计算服务已随启动成功应用，nav CPU 4–6、driver CPU 7、CPU performance、
EMC 3,199,000,000 Hz 均保持，证明这次重启后的相关配置生效。
这确认的是当前停机状态，不补造上次断电前的正常停机证据。
已找到旧 `.9` 的 120 s 接口 soak：13 次样本，首个 capabilities 请求超时导致
整体失败；其余样本接口恢复，未记录导航控制故障。300 s 采集没有 summary，
保留 166 条完整样本、实际覆盖约 165 s，最后完整时间为 08:51:47，末尾 3,377 字节
为未写完的零字节。完整样本的原生控制/驱动/零命令正常，Host 有 2 次请求超时；
164 份可用停车状态均为 `NOT_REQUESTED`，不能认定采集完成或历史停车确认成功。
原始文件和离线统计保存在 `build/go2-deploy-20260910/reconnected-release9/`。
本次发布基于 NX 实际干净快照 `bbf3653` 比对，仅集成 real 的导航、SLAM、地图代码
及测试/文档，保留现场独有的 Livox Release 断言修复，未覆盖不完整的仿真目录。
`.10` 的 NX ARM 构建已完成：导航 410/410（51.67 s）、SLAM 13/13（1.17 s，
包含真实 DDS TF 发布）、地图 24/24（10.36 s）、驱动 3/3、录制 14/14 通过。
构建日志位于 NX 源码目录 `build/go2-release10/`。完整标准发布包校验、安装预演和
安装均通过，current 已切换到 `.10`，没有原地覆盖旧 release。
`.10` 专用 `teleop_avoid` 启动与 37 项只读检查通过，doctor 为 18 pass / 3 个
未启用相机 warn / 0 fail。接口持续测试仍失败：首次 capabilities 请求超过 3 s，
另两条契约失败由缺失该响应引起；14 次样本中 Host 平均 CPU 为 90.71%，多个
状态接口平均约 0.5–1.4 s。定位最大 XY 漂移约 0.0055 m，不能据此推断运动能力。
本轮发现 NX 旧 `build/idle-soak.py` 固定采 121 行，忽略传入的 300 s；实际覆盖
120.06 s，不能算 5 分钟完成。121 条原生循环/驱动回执/零输出均正常，Host 有
2 次超时，整体失败。已上传支持时长参数的采集器，保留本次短测作为失败证据。
ProductControl 最终 stop 和 status 均为 `stopped`，6 个所属服务 inactive、MainPID=0；
此项只证明生命周期停止，不证明运动后的物理刹停。没有发送非零指令或 StandUp。
原始证据在 `build/go2-deploy-20260910/release10-evidence/`。
离线 NX 性能定位确认：每次 `load_runtime_graph()` 重新解析 YAML，连续四次耗时
0.302–0.308 s；状态边界又构造接口用不到的完整 manifest，耗时 0.337–0.399 s。
随后完成 Host 静态声明复用与按需投影：
`src/gateway/services/app_bootstrap.py` 在 Gateway 实例复用静态 Runtime Graph，
当前 env/backend/session/RunPlan 可用性仍每次计算；
`src/gateway/services/runtime_status.py` 直接使用原契约的 frames/frame_links/data_sources，
不再构造接口用不到的完整 manifest。全局 YAML loader 保持原行为，实时定位、
输入门及控制权不缓存。NX 同输入前后返回值完全一致，边界查询降至 0.53–0.71 ms，
能力声明缓存命中约 17–38 μs，首次仍需加载。旧版 4 项新回归失败，修复后 5 项通过；
94 项相关本地契约最终全通过，另补齐一个旧 RunPlan 测试夹具缺失的 `has_process`，
保留原断言。NX 运行 venv 未安装 pytest，没有把本地契约写成 NX pytest 通过。

`.11` 标准发布包校验、安装预演和安装通过，源码快照为 `82c9895`，没有改动
`.10` 的原生 C++ 程序或配置门槛。新版实机无运动验收结果：

| 检查 | `.11` 实测结果 |
| --- | --- |
| ProductControl 与专用 gate | `teleop_avoid` / SCAN 完整启动，37 项只读检查通过，未取得操作者控制权、未发布测试运动命令 |
| doctor | 18 pass / 3 个未启用相机 warn / 0 fail |
| 完整零输出采样 | 300.019 s、301 条完整样本、错误 0；导航和驱动命令均为零，Host 全部 ready，原生控制与驱动回执正常 |
| 输入与循环 | odom 10.086 Hz、registered cloud 9.862 Hz、输出 99.396 Hz；每控制周期一次零输出，未发生导航重启 |
| 控制耗时 | 最大窗口 work p95 为 8.969 ms，最大 deadline miss ratio 为 1.833%；原健康门槛下通过 |
| API 持续测试 | 120.098 s、57 次多接口样本、0 违规、0 超时；沿用原 3 s 超时门槛 |
| 延迟变化 | 定位接口均值 1045.45 → 8.73 ms，就绪接口 921.74 → 12.29 ms，bootstrap 1365.04 → 20.89 ms |
| Host CPU | Linux `ps %CPU` 采样均值 90.71% → 30.71%，口径为单核 100%，不是整机占用比例 |
| 停止与清理 | ProductControl stop/status 均 `stopped`，6 个所属服务 inactive / MainPID=0，validation/stop 退出码均 0 |

冷请求仍有开销：首次 capabilities 最大 2408.9 ms；并行 `/ready` 采集最大
1868.1 ms、p95 18.88 ms。不能把均值提速写成所有请求都小于 20 ms。
通用 service inventory 仍列出未选中的 camera/terrain/explore 以及未运行的泛化 DDS
检查；它不替代这次 Product 的 37 项专用 gate，也没有因此取消真实必需检查。
上述零输出批次只证明这一段运行与生命周期就绪；该批次没有发出非零速度或
StandUp，未执行 Go2 运动后刹停、自主导航、实体绕障或行人横穿验收。
原始报告在 `build/go2-deploy-20260910/release11-evidence/`，本地增量补丁及回归日志
为同一 build 目录的 `gateway-status-performance.patch` 和
`gateway-status-performance-final-contracts.log`；`.9/.10` 的失败证据均保留。

| 验收范围 | 当前结论 |
| --- | --- |
| Go2 部署与监督 WASD | `.13` 已安装、启动与专用只读就绪通过，用户确认 WASD 已移动、松键后停车；负载告警与避障效果仍待进一步验收。`.12` 的 120 s 零输出与 `.11` 的历史记录保留 |
| Go2 单次监督前进 | 用户指定 0.5 m/s、4 s；已运动并停止，SDK 净位移 0.554 m；期间出现局部初始化失败和循环负载告警，稳定前进验收未通过，详见下文 |
| Go2 参数、实采点云的 ARM 离线 SCAN | 12 个场景通过；不包含实机运动或完整全局导航 |
| Thunder / 4998、真值定位隔离导航 | 直行、绕货架、运动中换目标、到点停车通过 |
| Thunder / 4998、实际 Fast-LIO2 完整 Product | 12 已实际到点并停车，TF 坐标阻断已解除；整体仍因输入就绪与 LiDAR 丢帧门槛失败 |
| Thunder / 4998、实体行人横穿 | 未通过；最新 04 仍有 58 个物理接触步，后续到点与停车不能抵消碰撞 |
| 楼梯、Go2 现场自主运动 | 本轮未验收；静态货架通过不覆盖这些行为 |

### 2026-09-10 单次监督前进：0.5 m/s、4 s

用户明确指定速度与时长，并确认现场有人看护、前方至少 3 m 无人员或障碍、
具备已验证的立即停止手段。仅执行一次，保持 `teleop_avoid` / SCAN，
`manual_mode=false`，未调用 StandUp 或关闭避障。

预演发现 `lingtu.operator_drive` 强制要求 RunPlan 显式包含
`LINGTU_DDS_DOMAIN_ID`，但正常 real/systemd RunPlan 没有该字段，其 nav
合法默认域为 0；这次预演在发出非零指令前失败。随后从当前会话新鲜 nav
状态读取实际 `domain_id=0`，使用已有 `lingtu_nav_control operator-motion`
输入链执行；没有直接调用 Go2 SDK 绕过导航。该 Python 工具的域解析缺陷
仍待修复，另有 sim 进程环境/argv 优先级需要对应实际启动规则。

本次 50 Hz 发布 200 条前进意图，vx=0.5 m/s、vy=wz=0，持续 4 s，
随后 hold、release 和原生 stop 均返回成功，再由 ProductControl 正常停止。
6 个所属服务均 inactive / MainPID=0。268 条状态采样显示：

- Go2 SDK 前后里程计 XY 为 (-0.113792, 0.209229) → (0.164244, 0.688788)，
  净位移 **0.554329 m**；指令积分 2 m 不是实测行程，也不能只用最后一个
  `last_completed_motion_run` 代表全部运动，本次输出中断分成了 6 段。
- 输入门全程 ready；局部状态出现 `scan_actual_motion_limited` 4 次、
  `local_intent_pending` 7 次、`scan_initialization_failed` 8 次，控制循环健康
  中有 8 次 `p95_utilization_high`。这些是采样条数，不是独立故障次数。
  不能把本轮未走满归为加速度一项，也不能把初始化失败称作正确避障。
- native stop 证据为 `CONFIRMED`、驱动 ACK observed/accepted；当前实机契约
  为 `driver_ack`，quiet odometry 要求和计数均为 0，不能声称该门禁要求了
  连续静止窗口。另采到停车确认后的 9 个不同时间戳、新鲜 SDK 状态，最大
  平移速度 0.01113 m/s、角速度 0.028762 rad/s，低于 0.03 / 0.08 阈值。

指令发送、实际运动、停止与进程退出已有证据；**稳定按给定速度向前运动尚未通过**，
自主到点、实体绕障与动态行人避障仍不能由此认定通过。未追加运动或自动重试。
用户随后现场确认：“确实向前移动，现已停稳”。局部规划失败与负载告警各自对
速度响应的影响仍需分析本次采样，不能仅凭同时出现就认定为全部原因。
证据在 `build/go2-deploy-20260910/go2-forward-05-4s-evidence/`；保留原始
operator 日志、前后驱动状态、完整采样、stop ACK 与 ProductControl 退出结果。

#### 同日离线分析：停车、时序与历史点云

已取回本次 NX journal、最终 SLAM/Mapd 状态及部署源 `82c9895`，保持六个
Product 服务停止，没有追加运动。此次分析明确区分两件事：**路程损失的执行链
已经确认；初始化失败的底层唯一原因尚未闭环**。

- 本次 200 条前进意图对应 396 个遥控处理 tick，其中 113 个计为停车，约
  28.5%。驱动没有新增拒绝、断连、watchdog 或 backend 错误。
  非零指令的 144 个追踪样本中，命令平移速度平均为 0.265466 m/s，观测平均
  为 0.170975 m/s。该统计排除了零命令，不是完整四秒均速；它表明实际给底盘
  的输入没有连续维持 0.5 m/s，起停过程中的跟随响应也进一步减少了位移。
- 去重后只有 73 个 nav 状态和 15 个 driver 状态；不能把 268 行读取当成
  268 次独立故障。有效前进段记录到两帧 `local_intent_pending` 和两帧
  `scan_initialization_failed`，这四帧最终速度均为零，循环健康均正常。
- `p95_utilization_high` 对应两帧：600 tick 滚窗的工作耗时 p95 为
  9.077760 / 9.016160 ms，略超 10 ms 周期的 90% 提醒线。两帧仍输出非零，
  `control_loop_hold=false`。因此纠正“负载告警导致此次停车”的推断；本次证据
  不支持该因果，也不支持初始化花了数秒。两帧初始化失败对应 worker 单次
  调用分别仅 1.361440 / 1.163200 ms，不能以它们代表所有调用的最大耗时。
- 唯一可见 `scan_actual_motion_limited` 将约 0.500 m/s 缩到 0.494 m/s，
  缩放比例 253/256；这是制动包络碰撞限速，不是“底盘达不到速度”的故障码。
  该帧的小幅限速不足以解释全部路程损失；较低频状态可能遗漏其他制动事件。
- 最终安全门停止后，会经 `replanTeleop` 重置旧轨迹，从当前位姿重新生成。
  本次多次轨迹 ID 回到 1、执行时钟重启与六段运动记录一致。但尚未记录每次
  reset 的首个停止原因，不能直接把重置判成不必要 bug 或删除此安全行为。
- 确定的诊断缺口是 `scan_initialization_failed` 合并了 rebound 优化失败、
  时间调整/refine 失败和最终速度/加速度超限；已有 debug 中的搜索/优化细项
  未从核心填入，默认零不代表没有搜索、碰撞或优化。对应实现：
  `src/nav/cpp/planning/local/scan/backend.cpp`、
  `upstream/plan_manage/planner_manager.cpp`（相对同一 scan 目录）；停止后重置
  接线在 `src/nav/cpp/endpoint/nav/control/teleop.cpp` 与
  `src/nav/cpp/navigation/executor.cpp`。

本次使用实时 SLAM + Mapd 滚动碰撞图，不依赖已保存全局地图。状态显示
`planner_map`/`active_octomap` 为空、`saved_map_points=0`、扩展累计地图层关闭。
现场保存地图目录未找到地图资产，常规录制目录不存在，`/run/lingtu/blackbox`
为空；本次测试脚本也没有开启原始 DDS 录制。已保存的几何是每帧 640 个最近
传感器 XY 的**已膨胀占据格中心**，原完整图约 50.17–61.21 万个占据格，
全部标记 `occupied_points_truncated=true`。`complete=true` 只说明底层图完整，
不说明这 640 点导出完整。预览可用于局部回看，不能重建当时的完整激光点云、
复算全部碰撞，或确定某格来自人、地面还是机器人自身。停机后重新采集也不能
代替未保存的历史帧。

已有几何和命令时间线可在本地
[历史回看页面](../../../../build/go2-deploy-20260910/forward-analysis.html) 查看；
页面不含控制接口。分项分析为同一 build 目录的
`forward-analysis-initialization.md`、`forward-analysis-timing.md` 和
`forward-analysis-behavior.md`；追加 journal 在 `forward-analysis-evidence/`。

下次应先补齐失败分支、首个碰撞位置/时间、实际速度/加速度超限值，以及
触发重置的首个安全原因。复现输入需完整碰撞 bitmap、同步位姿/速度、引导与
失败轨迹；原始传感录制可复用 `sensors` 预设，但该预设并不自动包含碰撞
bitmap。先做离线回放，再进行新的监督实机验收；不以降低门槛、删除安全停止
或提高速度代替定位。本轮仅完成分析与资料更新，未部署算法修改。

#### 有人走到机器狗前方时会怎样

| 运行状态 | 当前行为 |
| --- | --- |
| Product 已停止（本轮结束状态） | LingTu 不产生动作 |
| `teleop_avoid + SCAN`，无有效运动意图 | 保持零速度，不会因人靠近而自动生成后退目标 |
| `teleop_avoid + SCAN`，持续前进意图 | 尝试局部轨迹、限速或停车；没有专门的后退避人策略 |
| `nav`，有目标且持续受阻/无进展 | 可尝试经过检查的平移、旋转恢复；后退只是候选方向之一 |

`teleop_avoid` 的 `recovery.max_attempts=0`，SCAN 遥控路径也明确排除通用
遥控恢复分支。跟踪控制器允许短暂负 vx 纠正位置，本次确实记录到一帧
-0.022682 m/s，但恢复状态为 inactive，不能称为“主动后退避人”。
当前 SCAN 最终制动使用实测占据和机器人运动，没有把已有动态簇预测接入该
权威碰撞路径。动态行人横穿仍未通过仿真验收，也未通过 Go2 实体验收，不能
承诺人靠近就会可靠退让；静态停车/绕行与主动退让必须分别测试。

**以下为此前断线时的记录；14:09 已恢复连接，当前状态见本节开头。** NX 已完成 `.9` 安装与
ProductControl 启动，安装包内 doctor 为 18 pass / 3 可选相机 warn / 0 fail，
严格辅助遥控就绪检查通过，且明确 `read_only=true`、`command_published=false`。
随后开始 300 秒静止采样和并行 120 秒多接口 soak，但 SSH 失去响应，未取回结果。
Sunrise Wi-Fi 仍通，实际 `eth0` 为 `NO-CARRIER` / `carrier=0`，NX `.18` 与
Go2 主控 `.161` 均不通。`lingtu-go2-eth0` 的 `.99/24`、autoconnect 和
never-default 配置仍已保存；不能据此认定持久化配置丢失，也不能确定是网线还是供电。
11:57 再次 SSH 登录 Sunrise 核对，`eth0` 仍为 DOWN / `carrier=0`，NX ping 0/2。
**本轮未发送非零运动命令或 StandUp，但不能宣称断线后的最终停机已确认。**
恢复现场连接后，先通过 ProductControl 读取状态并核对停机，再取回 NX
`build/go2-idle-soak-release9/summary.json`、`build/go2-formal-soak-release9.json`
和 `build/go2-soak-release9-exits.log`。本地记录为
`build/go2-deploy-20260910/release9-link-loss.json`。驱动 unit 已更新为
`SuccessExitStatus=143`，保留 CPU 7 和禁用开机启动；正常 SIGTERM 不再应被误标为
失败，真正 watchdog/启动无心跳仍失败。4 项针对性回归通过，现场最终停机需恢复后验证。

恢复连通后的无运动核对命令（在 NX 执行，不启动导航）：

```bash
sudo bash -c 'set -a; . /opt/lingtu/config/go2-native.env; set +a; bash /opt/lingtu/current/scripts/lingtu --robot unitree/go2 --env real status --json'
sudo bash -c 'set -a; . /opt/lingtu/config/go2-native.env; set +a; bash /opt/lingtu/current/scripts/lingtu --robot unitree/go2 --env real stop --json'
systemctl show lt-driver.service -p ActiveState -p SubState -p Result -p ExecMainStatus
```

本轮将真机静止、Go2 参数的离线规划和 Thunder MuJoCo 运动分开验收。
**尚未完成 Go2 实机自主导航避障验收。** 当前 NX 的
`/var/lib/lingtu/maps` 没有现场保存地图；`teleop_avoid` 的就绪不能替代
`nav` 所需的保存地图、重定位和全局路径。正式保存地图只允许 `map` Product，
站立采一小片点云也不能替代现场覆盖建图。

已取得的真机静止证据：

| 检查 | 结果 |
| --- | --- |
| `.8` 连续静止 | 900.45 秒、901 次采样，全部通过 |
| 原生输出 / 里程计 / 配准点云 | 99.416 / 10.404 / 9.881 Hz |
| 最高滑窗工作耗时 p95 / 超期比例 | 8.619 ms / 1.1667% |
| 输出与控制周期之比 | 1.0，全为零速度，目标和操作指令计数均为 0 |
| `/ready` 延迟 | p95 551 ms，最大 586 ms；本轮没有 503 |
| 严格配对的现场点云 | 10 秒内保存 41 帧，云/扫描位姿最大时差约 0.48 微秒，位姿位移约 5.5 mm |

但后续 60 秒、多接口正式 soak **未通过**：驱动自行发送了
`lingtu-driver-safety-1` 停车请求，原生导航返回
`stop_confirmation_timeout_stop_remains_latched`，网页显示 OPERATOR 和
停车确认 FAILED。没有用户运动输入；原生输出仍为零。
当时驱动累计 `backend_errors=2`、`safety_stops=1`、`watchdog_stops=0`，
不能把原因写成命令看门狗超时，不能用前面 15 分钟的通过覆盖。
已保存故障状态，并通过 ProductControl 停止真机 Product 后进行离线分析。

停车确认超时已定位到 `src/nav/cpp/endpoint/nav/main.cpp` 的同步等待：
等待回执期间正常输出循环暂停，原来只发一次零命令；驱动的 3 秒重连期间
会丢弃收到的命令，恢复后没有新的零命令可确认。修复在原 4 秒期限内按既有
控制频率持续发布零命令，并只接受本次等待实际发布成功的精确序号/时间戳回执。
自主停车仍要求回执后的连续静止里程计证据。NX ARM 导航 410/410、驱动 3/3
回归通过，独立审查无阻断问题；测试覆盖丢弃前 150 条零命令后恢复、错误回执
拒绝及不延长原超时。这不等于已完成真机故障注入或运动停车验收。
最初触发驱动故障的 SDK 返回码或状态过期原因当时被后续成功命令覆盖，仍未确定；
驱动新增故障边沿日志保留原原因，后续必须据此诊断，不能把停车回执修复当成
初始通信故障也已解决。

IMU 初始化修复已通过 NX ARM 全部 12 项 SLAM 回归。完整流程测试另修正了
模拟三面墙被 `lidar_filter_num=3` 抽成单面墙的输入错误；保持全部健康断言，
修正后同一测试连续 3 次通过。补丁保留测试也改为先验证有效观测确实产生，
再检查未超限完整与实际丢弃后不完整；没有修改生产地图保存判断。

NX 证据相对于 `/home/unitree/lingtu-source-20260910`：
`build/go2-idle-soak-900s/`、`build/go2-formal-soak.json`、
`build/go2-after-formal-nav.json`、`build/go2-after-formal-driver.json`、
`build/go2-stop-after-soak-fault.json`、`build/go2-scan-capture/`。
原先偶发单次 503 的原因仍不能仅凭本次未复现而认定已修复。

测试工具已补齐 Gateway 鉴权，并修正网页点云缓存为零的误判；真实 SLAM、
里程计、时效、静止状态和漂移检查仍保留。监督验收脚本使用已有
`LINGTU_API_KEY`，不将其写入报告。源码修复和不可变 `.8` 安装包的状态分别记录，
不在安装包内就地覆盖文件。

Go2 几何与实采点云的离线 SCAN 检查已在 NX ARM 完成。它直接调用生产
RollingOccupancyGrid、一次碰撞膨胀和 SCAN，不链接 DDS 或 Driver：

| 场景 | 结果 |
| --- | --- |
| 合成空地、可绕过的有限前墙 | 均生成轨迹，2 ms 采样无碰撞，终点误差约 0.11 mm |
| 合成全封闭 | 明确拒绝，没有可执行轨迹 |
| 41 帧现场云、8 个方向 | 6 个方向生成安全局部轨迹，2 个方向明确拒绝 |
| 现场云叠加合成前墙 | 明确拒绝，没有穿障轨迹 |

现场 6 条轨迹中有 3 条是缩短的局部轨迹，不能说已到达原始目标。
冻结地图不验证在线时效或机器人跟踪，空白栅格也不表示已经测绘的自由空间。
证据是 NX `build/go2-offline-scan-result/summary.json` 和逐轨迹 CSV。
合成绕障轨迹的峰速约 0.739 m/s，未运行控制器，不能据此声称 0.5 m/s
实际速度限制已经验收。

Thunder V4 / 4998 的 MuJoCo 结果另行记录：

- 发现并修复旧验收使用园区地图、实际 Product 使用工厂世界的配置错配；
  当前点击 fixture 和命令文档统一为 `factory_workshop_v2`。
- 真值定位隔离实验的 2 m 直行通过。绕货架复测实际行程 9.183 m、净位移
  6.038 m、到点误差 0.187 m；6,038 个仿真步观察无实体接触，到点零命令回执和清理通过。
- 该绕障 fixture 没有跟踪精度门槛。实测到最新局部路径的几何距离 p95 为
  1.809 m、最大 2.233 m；原生状态自己的位置跟踪误差 p95 也约 2.07 m。
  这不能仅解释为统计方式差异：一条重规划轨迹起点与当时机器人相距约 2.23 m。
  已定位为隔离 runner 以 RTF=0.5 推进物理仿真，却没有接入 simulation execution
  clock，轨迹执行/物理时间实测比为 1.981。修复后同条件复测通过：该比值为
  1.0069，原生时间对齐的跟踪误差 p95 为 0.0186 m、最大 0.0269 m，路径几何
  距离 p95 为 0.0154 m；5,913 个物理步无实体接触，到点误差 0.2096 m，
  物理零命令回执、控制权清空和进程清理通过。证据为本地
  `build/go2-deploy-20260910/scan-rack-sim-clock-run/report.json` 及同名前缀分析文件。
- 修正旧采集器只统计 `acceptance_*` 几何导致普通货架碰撞漏记的问题；
  现在按仿真步复用实体接触分类，缺少证据或存在实体接触均不能通过。
- 相关 Python 回归 217 项、原生导航回归 103 项通过。目标替换在原生契约测试中覆盖，
  后续增加真实运动中换目标实验：实际位移 0.546 m、速度 0.511 m/s 时接受新目标，
  旧轨迹已失效；但短暂碰撞栅格过期后，输入恢复仍长期停车，最终距新目标 3.335 m，
  因而该轮整体失败。10,570 个物理步无实体接触，停车和清理通过。
  已修复同步停车等待丢弃 TF/碰撞栅格等传感批次，以及新目标把短暂过期当成永久失败的错误；
  恢复后基于新的停车位姿重新规划。中间复验虽已到点，但仍因延后激活未同步
  `last_plan.accepted` 而失败。修复此记账遗漏及恢复重规划期间永久输入故障悬挂后，
  最终同场景 **全部 12 项通过**：实际运动 0.582 m、速度 0.527 m/s 时换目标，
  新目标误差 0.0993 m，3,597 个仿真步无实体接触，局部几何跟踪误差 p95 为
  0.0150 m，输入就绪 100%，目标/路径计数均为 2，旧目标不再控制，物理停车和清理通过。
  4 项原生回归通过（4.58 s）。新结果保存在独立 `scan-rack-mid-motion-replacement-final-run`，
  未覆盖之前失败报告。证据为
  `build/go2-deploy-20260910/goal-replacement-stale-fix.md`。
- 同源 Fast-LIO2 Product 启动出现静止发散。直接采集 DDS IMU 发现首次校准
  的 240 个样本含落地冲击，均值模长约 11.427 m/s²，旧代码由此固定约 0.859
  的加速度比例，使后续静止产生约 1.39 m/s² 的虚假重力残差。初始化窗口修复
  已通过原生回归。第 04 轮还暴露 feeder 冷启动占满共同阶段就绪预算，已修复为
  feeder typed ready 后一次性开始 provider 有界等待，保持 DDS readers 先启动；
  34 项测试通过，2 项平台跳过。第 05 轮在 Popen 后 41.003 s 输入就绪、
  57.181 s 配准成功、75.204 s Product 提交，证明原启动截断已解除。
  但点击仍因 `localization_not_tracking` / `native_input_gate_not_ready` 返回 409。
  243 次去重更新中 197 次退化拒绝、19 次后验协方差对角非正、2 次信息矩阵非正，
  25 次接受；原先竖直发散未再出现。已在静止约束测试中复现协方差截断只改对角线、
  保留旧相关项导致矩阵非正定；改为同步缩放行列，保留既有上限，Windows SLAM
  12/12 通过。06 未再记录到这两种协方差拒绝。原生离线重放精确复现首个退化拒绝，
  实际匹配的法向矩阵条件数约 7.52、三个方向均非零；混合弧度/米的完整位姿 Hessian
  却因旋转杠杆数值较大，把三个平移方向全部划为低于最大特征值 1% 的弱方向。
  现已按单一旋转特征长度统一尺度，并正确回映射信息、状态和协方差投影；保留原
  1%/最大退化数量/条件数和物理运动门槛。Windows SLAM 12/12 通过（3.92 s）。
  同一 06 原始数据的原生重放由 181 次更新中 75 次接受，改善为 175 次接受；
  仍拒绝 2 次位置增量超限与 4 次被判为 4/5 方向退化的更新，协方差/LDLT 拒绝为 0。
  最大估计速度由 0.3777 降至 0.0648 m/s，完整协方差最小特征值约 2.09e-8。
  这证明已复现的尺度问题修正有效，尚不能宣称完整在线定位已验收。
  05 向已停止 mapd 请求 restore 的错误也已修复：按真实存活进程选择先恢复地图或
  先恢复上一 Product 的 mapd，再用原 token 恢复地图。76 项回归通过，06 实际注入
  故障后上一身份、子进程、地图恢复、journal 清理全部通过；正常停机和清理也通过。
  06 完整导航仍失败：点击返回 `recovering; native_input_gate_not_ready`，IMU
  184.34/200 Hz，LiDAR 202/213 帧发布、8.72/10 Hz，均不能通过原频率/丢帧门槛。
  原始 DDS IMU 首段 4,269 条与 feeder 发布数一致，物理跨度 21.34 s，源时间戳却
  跨度 23.1487 s，确认固定物理采样被盖上调度唤醒时间。已统一 IMU/瞬时 LiDAR
  的物理采样时基，并修正瞬时点云错误提前 100 ms 的帧时间；保持 0.6 s 过期门槛，
  计入物理步、快照、排队和投射耗时。39 项直接回归和独立复审通过，07 隔离验证
  此项效果，没有同时改变退化算法。07 主运行 IMU 12,004 条完整发布、199.47 Hz，
  但 LiDAR 600 次调度仅发布 416 帧，仍未通过；启动最终因 `odom_stale` 超时，
  未进入目标场景。该轮报告未验证生命周期/停车/回滚，不能用 06 的通过代替。
  随后的受管零命令性能诊断也未达到 active，不能算正式验收：52.37 s 冷加载后，
  60.293 s 内 9,693 个物理步/IMU 样本，LiDAR 484 次调度仅发布 9 帧。
  插桩精确记录 474 次在投射前源数据已超过 0.6 s，1 次在投射后超时，队列替换为 0。
  支撑面查询累计 15.729 s、p50 1.375 ms、p95 2.990 ms；物理步累计 28.302 s，
  普通睡眠仅 0.252 s。说明主线程已追不上固定周期，不能归咎于全局规划或只猜测雷达慢。
  支撑射线现先排除仅包含视觉外壳或机器人自身的几何组，再沿用原生精确相交与混组排除。
  同模型对照还发现旧查询会在跳过共面的视觉面时越过真实顶面：起点返回路面下方的
  地基，净空误报 0.65 m，新查询命中实际路面为 0.43 m；部分台阶旧报 0.57 m，
  实际踏面应为 0.45 m。因此不能称新旧数值全部等价，旧错误值的对照证据保留，
  已知物理顶面的回归已补齐，共 5 项支撑查询测试通过，Ruff 通过。
  同模型起点查询 p50 从 1.076 ms 降至 0.475 ms，但无遮挡台阶会因构造分组掩码
  多耗约 0.13–0.16 ms，不能称所有位置都提速。此值仅用于 feeder 姿态稳定检查和
  高度证据，不输入策略、SLAM 或规划；全部频率/时效门槛保持不变。
  证据见 `build/go2-deploy-20260910/feeder-profile-diagnostic-01/findings.md`。
  08 未插桩正式复测仍失败：首段在 60 s feeder 就绪截止时被正常停止，只有 3 个
  物理步，IMU/LiDAR 均未发布，未进入点击场景。实际支撑高度 0.539–0.544 m、
  跨度约 0.005 m，均在既有门槛内，退出不是姿态异常。该轮独立回滚准备运行中，
  31.325 s 发布 6,078 条 IMU（194.03 Hz），LiDAR 303 次调度仅发布 78 帧
  （2.49 Hz），最大调度迟滞 1.288 s；也未达到地图就绪，未执行故障注入。
  实采 IMU 相邻源时间为 5 ms，但接收时已延迟 p50 625.74 ms、p95 1266.89 ms，
  确认时基正确仍不等于实时吞吐足够。08 去重采样的 47 次 SLAM 更新中，35 次接受，
  11 次位置增量超限、1 次速度增量超限；退化数量及协方差/LDLT 拒绝均为 0。
  这些在线样本支持尺度修复效果，但最终旧点云距状态已超过 19 s，不能算连续定位就绪。
  不能把正常清理、零命令 ACK 或之前 06 的回滚通过写成 08 完整验收通过。
  之后针对无窗口、无相机且明确使用 CPU LiDAR 的路径，在复制/编译前排除
  没有外部引用的 worldbody 直属纯视觉 group 2。工厂 7,118 个实体保持，
  142 个视觉几何和约 968 MiB 无引用资产被跳过；动态 body、机器人惯量、
  共享资源、传感器引用及窗口/相机路径保留。121 项回归与独立审查通过。
  09 实测冷启动约 27.4 s，在原 60 s 内完成，全部 Product 模块与定位已启动；
  控制段 IMU 198.11 Hz，接收延迟 p50 16.63 ms / p95 54.08 ms，已无秒级积压。
  但 LiDAR 238 次调度只发布 205 帧（8.51 Hz），仍未通过丢帧门槛。
  点击任务实际已进入 planning，随后验收工具把 Gateway 的字符串 native_state
  当成整数枚举而崩溃。现已区分原生状态与公开生命周期枚举，保留原响应；
  11 项使用真实 Gateway 投影的回归通过，覆盖规划、执行及成功/失败/取消的结果保存。
  09 正式启动、停机、清理通过，但点击场景未完成、回滚未通过前置就绪，整体失败。
  09 控制段 156 个采样更新中 152 次接受，2 次速度增量超限、2 次 5 方向退化拒绝，
  协方差/LDLT 拒绝为 0。原始报告在 `scan-click-night-09`，未覆盖旧失败。
  10 启动、停机、清理及实际注入回滚全部通过，冷启动 26.21 s；IMU 199.96 Hz，
  但 LiDAR 仅 6.54 Hz、丢帧 34.48%，完整验收仍失败。点击落在 map 激活后的
  短暂 recovering 窗口而返回 409；源代码的启动就绪本来已检查 input gate，不能
  说它只检查进程存活，但此前通过不保证稍后点击时输入仍就绪。附着验收现按相同
  会话/新鲜度/定位/input/control 条件，在固定 18 s 内等待连续 1 s 就绪再提交一次；
  409 不重试，60 s 运动预算和原频率/碰撞/到点门槛保持，启动采样另存。
  状态及就绪等待 14 项、附着点击与生命周期 26 项回归通过，独立复审通过。
  10 控制段 117 个采样更新中 101 次接受，仍有 14 次退化和 2 次速度增量拒绝，
  未观察到协方差/LDLT 数值拒绝；该轮无非零运动，不能宣称到点验收通过。
  11 使用修正后的 MID360 采样窗口，冷启动 24.833 s，启动、停车、清理及实际
  注入回滚均通过。IMU 199.992 Hz；LiDAR 880 次调度发布 682 帧，7.746 Hz、
  丢帧 22.5%，仍高于原 1% 门槛。607 个采样 SLAM 更新中 605 次接受、2 次速度
  增量拒绝，未采到退化或协方差/LDLT 拒绝，不能据此取消运行时健康检查。
  点击稳定就绪后已 ACK，17.178 s 全局规划后激活路径；但起点约 (-0.006,-0.022)
  与目标 (61,18.5) 混在同一路径，而碰撞栅格的 map 范围约为 [56,66] x [11.45,21.45]。
  SCAN 反复报告 `recovery_collision_map_roi_uncovered`，物理非零命令数为 0；
  验收观测就绪比例 95.849% 也低于 98% 门槛：其中 8 次未读到 SLAM 快照、3 次
  未读到 nav 快照，262 次可读 nav 快照均就绪；缺失观测不能写成 11 次真实输入故障。
  完整报告在 `scan-click-night-11`。
  坐标链随后定位到 `src/localization/slam/cpp/cyclone_runtime.cpp`：`TfMessage`
  按值返回，但 DDS 序列指针仍指向返回前的局部数组。实际 MSVC Release `/O2`
  正常调用已复现存储归属错误，没有人为禁用返回值优化或构造特殊移动操作。
  当前 `DdsRuntime::writeTf` 接收持有数组的消息，并在 `dds_write` 前重绑存储，
  没有在规划层重复乘坐标变换。直接使用生产转换/发布函数的隔离 DDS 回归通过：
  连续两次非零平移和正负 90° 旋转的父子帧、源时间戳与各数值全部匹配。
  在同一正式测试中移除最终绑定，0.18 s 即失败；恢复后 SLAM 13/13（3.06 s）
  和直接相关 Python 契约 5/5 通过，独立审查无阻断。没有修改真机 MID360 输入。
  12 的完整链路已验证实际效果：首个 sensor origin 回到约 (60.948,16.494,0.453)，
  规划起点约 (60.926,16.497,0.453)，原目标不变；全局路径由 11 的 4 点缩为
  2 点，规划时间从 17.178 s 降为 5.203 s。机器人实际走过 2.144 m、净位移
  2.115 m，终点约 (61.177,18.609)，误差约 0.208 m，原生状态 `REACHED`，
  213 条非零命令、735 个非零物理步，实体接触为 0，随后输出零速度。
  TF 不动阻断已解除，12 的启动、最终停车、实际故障注入回滚和所属进程清理均通过，
  冷启动 23.795 s。但总体仍失败：LiDAR 调度 423 帧、发布 349 帧，8.247 Hz，
  丢帧 74 帧（17.49%）超过 1% 门槛；就绪比例 68/81=83.95% 未到 98%。
  13 次未就绪中，10 次为真实 SLAM scan age 超过 0.6 s（其中 2 次 nav 也报告
  `odom_stale`），另有 2 次空 SLAM 快照、1 次空 nav 快照，不能全部归为文件读取问题。
  控制段采样 287 个 SLAM 更新全部接受、弱方向为 0，但这仍不能代替连续时效验收。
  实采 15,792 条 IMU 和 666 个完整点云与三段运行发布总数一致；测试所属 native、
  feeder、采集进程全部退出，只保留普通空闲 supervisor。结果及 11→12 对照在
  `scan-click-night-12/report.json` 和 `findings.md`，此前失败没有覆盖。
  原始证据为 `scan-click-night-06/imu.csv`、`lidar.ltlivox`、
  `feeder-timing-findings.json` 和 `report.json`（均位于本地本轮 build 目录）。
  完整雷达导航仍未通过，真值绕障不能替代它，更不能替代 Go2 行走。
  上述后续修改未追加部署到 `.9`；重连后 real 的 SLAM、地图和导航修复已集成
  `.10`，仿真专用修改仍在本地工作区。
  验收报告另修正“传感器检查抛错，但子项仍标 ok=true”的错误；未测现在为 null，
  检查失败为 false 并保留原因，4 项带生命周期清理的回归通过。没有修改验收阈值，
  该工具仍有 3 条既存未使用 noqa 的 ruff 诊断，本次新增修改无新诊断。
  详细报告位于本地
  `build/go2-deploy-20260910/navigation-acceptance-agent.md`。

新增实体动态行人测试复用已有胶囊碰撞模型、真实射线点云与逐物理步接触计数，
不向地图注入合成障碍；记录实际 `mjData.xpos`，不能把发出位置指令当成行人运动。
直接受影响 Python 回归 199 项通过。判据已删除“横漂超过 0.25 m 就算绕行”的
假通过分支：4998 自身横漂不足以证明主动避障，当前只认可明确障碍停车与恢复。
`scan-dynamic-pedestrian-01` 无实体接触并到点，但未观察到确证避让，保持失败。
第二轮调整横穿时机后，雷达观察 104 次，行人实际移动 6 m，确实停车、恢复并到点，
但 **3,382 个物理步中 121 步发生行人实体接触**，最近中心距约 0.656 m，整体失败。
原始报告在 `build/go2-deploy-20260910/scan-dynamic-pedestrian-02/report.json`。
最终停车与清理通过不能抵消碰撞。轮胎在首次记录碰撞时仍位于 SCAN 轮廓内，
输入全程就绪，雷达已看到行人；原生 Mapd 对记录点云的回放确认，历史占据先验
需要同一 5 cm 栅格多次命中，而 1 m/s 行人每帧移动约 10 cm，导致真实新命中
未及时进入硬碰撞图。去掉人工地面点仍复现，不能只修改测试输入来规避。
实时碰撞种子现为“当前帧真实命中或历史占据”；概率、历史导出和保存地图
保持原算法，同一命中集合不反复清除重建。26 个原生用例通过，点云回放中的
制动触发从仿真 8.7 s 提前到 8.1 s，但这不能替代实体实验。
第三轮同参数实验仍失败：54 步实体接触，精确首碰为仿真 8.600 s，
`FL_wheel` 与行人胶囊穿入约 4.42 mm，机器人位置约 (41.977, 7.552, 0.456)。
输入就绪仍为 100%，之后被行人推离路线，触发 `replan_budget_consumed`，未到点。
新命中延迟问题已修正，剩余的移动障碍制动/恢复问题尚未验收；没有修改速度、
安全边距、横穿时机或碰撞门槛来让第三轮通过。证据在 `scan-dynamic-pedestrian-03`。
首碰时指令仍为 0.60 m/s、实速约 0.604 m/s，首次遥测零指令出现在 8.8 s，
因此不能把首因写成“已发刹车但 4998 停不住”。当前制动沿机器人速度预测，
不预测行人横向运动；接触前部分帧的行人返回只有 z=1.03–1.65 m，未覆盖当前
机身检查高度。需要区分动态预测和稀疏竖向观测的影响，不以扩大垂直膨胀代替推断，
以免重新破坏货架和楼梯净空。
进一步核对没有发现 LiDAR 倒装或坐标翻转：站点高度约 0.650 m，MID360 图案
垂直视场约 -7.21° 至 52.16°。但仿真代码把每帧 4,000 点预算同时用作图案推进量，
相对于 10 Hz、每帧 20,000 物理采样，把扫描图案时钟放慢了五倍。现已按快照的
物理采样时刻选择完整扫描窗口，再有序降采样；丢帧也不会冻结扫描相位。
159 项接线测试、18 项实际 CPU 射线/快照测试及独立复审通过；瞬时点云仍使用
全零点内时间偏移，没有虚构运动补偿。该修复覆盖本次执行的 formal feeder 和
native DDS MuJoCo 路径。另一条 `sim/runtime/sensors/mid360.py` 的
`Mid360PatternCursor` 仍按点数预算推进，丢帧也会冻结相位，已记录为待修问题；
本轮没有运行它的 `Mid360EndpointFactory` / raycast extractor 路径；该路径仍用于
coordinated Editor/playable，并非死代码，不能宣称已一并修复。本次也未改变真机 MID360 驱动。
同参数第四轮实体复测仍失败：4,631 步中 58 步接触，首次接触为 8.595 s、
`FL_wheel` 与行人穿入约 4.28 mm。原先 8.4/8.5 s 的机身高度点云缺口已消失，
对应高度带内有 122/149 个返回，但 8.5 s 命令仍为 0.6 m/s，8.6 s 遥测才记录
零命令，实速仍约 0.604 m/s。命令遥测只有 10 Hz，不能用它与逐步首碰相差约
5 ms 来断言精确的制动/接触先后；可以确认此次未取得足够的无碰撞刹停提前量。
之后到点误差 0.0629 m，精确停车、控制权清空和清理通过，但碰后恢复不算成功避障。
原速度、行人时机、碰撞几何和验收门槛均未改变，03/04 失败报告均保留。
证据为 `build/go2-deploy-20260910/scan-dynamic-pedestrian-04/report.json` 和
`dynamic-obstacle-collision-review.md`。
项目已有 `MotionLayer::snapshotPredictedDynamic` 动态簇外推，不能说完全没有
预测代码；但当前 SCAN 使用 Mapd 权威碰撞栅格时跳过这组预测点，最终保护也只
使用该栅格。当前 0.6 m/s 下，以反应时间 0.35 s、配置减速度 0.5 m/s² 计算的
前推制动距离为 0.57 m；这些是配置假设，不能当成测得的 4998 或 Go2 制动性能。
后续先用第四轮点云回放验证现有簇估计的方向、速度和可靠识别时刻，再把通过验证的
动态预测接到实际 SCAN 保护路径。既有 1 s 预测时域短于上述配置下 1.55 s 的
反应加刹停时间，不能仅启用旧预测开关就宣称修复。需保留实测地图的归属和机身净空语义，
再用原横穿场景及静态货架/台阶回归确认提前避让，不能靠调低速度或扩大垂直膨胀过关。
新的 Mapd 已另通过原静态货架场景：10.394 m 路径、5,899 个物理步零接触，
到点、精确停车、控制权清空及进程清理通过。直接受影响 Maps 套件 9/9 通过；
exploration 的 Release 测试缺少启用断言选项也已补上，启用后复验通过。

本轮 `.9` 之后的选定源码文件归档为本地
`build/go2-deploy-20260910/post-release9-source.tar.gz`，清单为同名 `.json`。
它是选定文件的源码归档，不是可安装发布包，也不证明所有依赖差异已覆盖。
重连后已取回中断采样、确认重启后停止状态，对照 NX 实际源码并仅集成 real
原生修复、对应测试与文档，保留现场 Livox 修改。该子集已完成 ARM 构建、测试及
`.10` 完整发布安装；不能称归档内 40 个文件全部部署，更不能把仿真通过写成实机通过。

下一次 Go2 自主导航验收必须先监督完成现场建图与保存，通过地图覆盖/关键帧检查，
再用 ProductControl 启动同一地图的 `nav`，确认重定位与无运动路径预览。
实际直行、松手停车、绕障、运动中换目标和到点停车仍需现场人员监督。

### 2026-09-10：前进中反复停车的代码修复与验收边界

本节是上述 0.5 m/s × 4 s 现场证据之后的修复，已作为 `.12` 完成 ARM 构建和安装，
`current` 链接与发布 metadata 均已核对。ProductControl 已启动
`teleop_avoid`，就绪与静止检查通过，尚无本版追加实机运动的验收结果；
不能把原生回归和安装成功当成现场避障已通过。

**不是始终没有路径。** 记录中多次出现 `teleop_assist_spline_ready`，之后进入
零输出、`local_intent_pending`、`scan_initialization_failed`，再生成新轨迹。
问题落在“执行安全否决后的重新初始化”这段链路；两次可见初始化失败分别只用了
约 1.36/1.16 ms，不支持把它们解释成长时间搜索超时。两次 p95 告警出现时仍有
非零输出、`control_loop_hold=false`，也不能将告警直接当作停车原因。

已确认的输入错误位于
[`fastlio.cpp`](../../../../src/localization/slam/cpp/fastlio.cpp) 与
[`cyclone_runtime.cpp`](../../../../src/localization/slam/cpp/cyclone_runtime.cpp)：
滤波器的世界系速度原先被直接填入 `/slam/odometry`、`/slam/state_at_scan` 的
`child_frame_id=body` twist。导航按机身系消费后再旋转，机器人转向时会得到错误的
运动方向。例如朝向 90° 时，世界系 `[0, 0.5, 0]` 应发布为机身前进
`[0.5, 0, 0]`，旧实现却发布成侧移。SLAM 原先还没有填充角速度，制动包络因此
无法正确使用实际转动。

修复在 SLAM 输出边界完成：使用同帧滤波速度、同步 IMU 去偏置角速度及安装变换，
把 IMU 原点的速度换算为 body 原点/机身系 twist；已有世界系诊断值保留。
缺少本帧有效速度或角速度时，不用合成零刷新导航里程计，仍可输出 SLAM 诊断位姿；
由现有 odometry age 门在 `teleop_avoid` 中阻止继续运动。这一约束不等于项目显式
`manual_mode=true` 也强制使用所有传感器门。

此错误已由非零朝向回归复现，但**尚未证明它是历史两次初始化失败的唯一原因**。
那次现场朝向接近零；完整失败碰撞位图、候选曲线和安全重规划首因当时未记录，
只有每帧最多 640 个膨胀体素预览，不能用它们反推完整可通行空间。

本轮同时补齐现有诊断通路：

- SCAN `last_scan_attempt` 区分 `initialization`、`rebound_optimization`、
  `refine_optimization`、`dynamic_feasibility`，保留实际优化返回码、碰撞检查
  位置/时间/占据状态，或速度/加速度实值与门槛。未尝试、未测量的字段写 `null`。
  碰撞位置是被拒绝段的起点或控制点，**不是精确接触点或命中障碍体素**。
- `teleop.last_safety_replan` 保存真正触发 `replan_motion()` 的原因、计数、
  steady 时间、平滑前规划候选与最终指令，后续 ready/pending/idle 不覆盖它。
- 首个最终尝试仍失败的 FSM tick 捕获完整不可变输入；同 tick 内部重试最终成功
  不算失败事件。保存失败曲线控制点、间隔、起终点导数、参考路线、机器人状态、
  参数与完整膨胀 bitmap，供复核候选碰撞和动态约束；这不是完整 FSM/随机历史录像。
- 完整输入异步写到 `<nav status_file>.scan-failure.json`，默认
  `/dev/shm/lingtu/nav_endpoint_status.json.scan-failure.json`。坐标明确为
  `request_planning_frame`，另含 grid-from-planning 变换；位图编码为
  `packed_lsb0_hex`，线性索引 `(z * size_y + y) * size_x + x`。
  每段连续失败只捕获一次，下一段替换；普通状态只带摘要和写入计数/失败数。
  `/dev/shm` 不跨重启，下一次取证应在停机前连同 nav/driver 状态一起复制出来。
- 原来没有接入真实测量的 SCAN 搜索/优化细分耗时、展开数和重启数改为 `null`，
  避免把默认零误读成“没有搜索或优化”。

仍需单独推进的边界：`/odom_prior` 现有生产者/消费者沿用世界系速度约定，不能只改
一端成 body 系，否则会二次转换；本轮保持这一内部契约。导航平面模型对 roll/pitch
的速度投影也需要在坡面验收中核对。本轮没有修改碰撞阈值、放宽安全停车或启用自动
后退。正式验收顺序为 ARM 构建与无运动输入核对，再在现场监督下用同一前进场景
取新的失败证据；通过后才进行动态障碍接近/绕行验收。

本地验证证据：

- `test_scan_planner` 完整 49/49 通过，含新增的碰撞/速度/加速度失败归因、
  完整地图所有权、异步 reset 保留、成功后重新捕获失败等 6 项回归。
  日志为 `build/go2-deploy-20260910/scan-attempt-debug-full-tests.log`。
  之后新增的取消归属用例先红后绿；最终 7/7 诊断回归通过，见同目录
  `scan-attempt-debug-cancel-green.log`。已确认取消的规划尝试不会延迟冒充新请求失败，
  此小修之后未重复执行此前完整 49 项。
- SLAM 的 `messages_odometry_publication`、`messages_initial_body_origin`、
  `messages_fastlio2_mock_flow` 3/3 通过；新增坐标测试和异常速度归零测试均先在
  旧逻辑失败，再由修复通过。证据为同目录 `odometry-body-twist-tests.xml`。
- 最终集成后的 `test_nav_status_publisher`、`test_teleop_tick_controller` 2/2
  通过，验证真实安全重规划原因保留、完整位图异步保存、不重复写入和 reset 后摘要
  保留；优化失败中的非有限数值序列化为 `null`。证据为同目录
  `scan-endpoint-final-tests.xml`。
- 无新 Odometry 的本地探针确认既有 0.25 s 时效配置在 0.26 s 报 `odom_stale`，
  新鲜数据恢复后重新开放。它验证输入门行为，不证明实机刹停距离。
- Windows/MSVC 的 `navd` 与 `slamd` 已构建；上游数值转换和 L-BFGS 等既有
  编译告警仍在，本轮新增诊断没有新增对应编译告警。此构建不是 ARM 发布包。

随后完成的 NX ARM 发布验证：SLAM 三个直接相关测试 3/3 通过（0.65 s），
整个 `nav_endpoint` 构建及完整导航测试 417/417 通过（26.62 s）。GCC 9 / aarch64
在 `test_scan_planner` 链接时出现 LTO partition 编译器内部错误；仅对该测试目标
添加 `-flto-partition=none` 后通过，生产程序的 IPO/LTO 配置未修改。
这些原生产物已进入 `.12` 标准发布包并完成安装。随后 ProductControl 启动
`teleop_avoid + SCAN` 成功；本次原生状态确认 DDS domain 为 `0`。
专用 motion 只读门通过，120 s / 121 帧静止采样无错误：点云约 9.91 Hz，
里程计约 10.62 Hz，命令输出约 99.42 Hz，导航和驱动始终为零，
未出现控制保持、重启或非零输入。最大窗口工作 p95 为 7.74 ms。
证据位于 `build/go2-deploy-20260910/forward-fix-ready-2.json` 与
`forward-fix-idle-summary.json`。第一次只读门因独立状态快照的回执序号差被拒绝：
nav=4827、其内嵌 ACK=4824，153.6 ms 后的 driver=4840，同一 producer 且均为零。
后续检查通过；这是待修的诊断快照关联误报，不能据此放宽原生运行安全门。
本轮未自动发送非零指令或调用 StandUp，现场动作与绕障尚未验收。

### 监督键盘操作入口

本地 [`src/lingtu/operator_keyboard.py`](../../../../src/lingtu/operator_keyboard.py)
已完成 22/22 测试与独立审查；这不代表键盘已完成实机运动验收。
同目录 [`start-wasd.ps1`](./start-wasd.ps1) 提供专用 Windows 控制窗口。
本轮已实际打开窗口；NX 确认 `active_source_id=go2-keyboard`、
`authority_reason=sample_accepted`、`input_gate_reason=ready`，初始导航与驱动输出均为零。
启动前须由 ProductControl 使 `teleop_avoid` 就绪；启动器不负责 Product 启停。
`DomainId` 必填，使用本次 RunPlan 的实际 DDS domain，不能猜测为默认值。
先将 `$domainId` 设置为该实际值，再从仓库根目录执行：

```powershell
.\config\robots\unitree\go2\start-wasd.ps1 -DomainId $domainId
```

默认 W/S 前后、A/D 左右侧移，平移速度为 0.2 m/s；Q/E 转向为 0.35 rad/s。
松开运动键或窗口失焦发送零命令；Space 发送零命令并解除键盘使能；
启动、失焦或显式停止后，操作者须让该窗口重新取得焦点并松开全部运动键，
再按键才能恢复运动。Esc 请求停止、等待原生停止确认后退出；确认失败会报告错误，
不能据窗口退出认定已停稳。
命令固定 `manual_mode=0`，经过原生导航避障与最终安全仲裁。没有新增自动退避，
也不保证行人主动接近时一定避开；现场人员仍须监督并保留可执行的停车手段。

### 键盘输入超时与 Sunrise Wi-Fi 省电

本轮用户只按 W 后出现 `LT_TELEOP_STREAM_TIMEOUT_STOP_V1`，原生 hold、release、
stop 完成；随后实机状态确认控制权已清空、导航与驱动为零。这个错误表示远端
超过 350 ms 没有及时处理新输入，不表示 SCAN 找不到路径。看门狗只在最后意图
非零或 manual 模式启用，因此此前长时间零输出不能证明按键链路不会断流。

沿同一 Windows → Sunrise → NX SSH 链路发送时间戳（无 DDS、无机器人运动）复现：

| 同条件 90 s / 1800 帧 | Windows 最大发送间隔 | NX 最大接收间隔 | 接收间隔超过 350 ms |
| --- | --- | --- | --- |
| Sunrise Wi-Fi 省电开启 | 94 ms | 1114.6 ms | 16 次 |
| Sunrise Wi-Fi 省电关闭 | 79 ms | 123.9 ms | 0 次 |

Sunrise 原来 `iw dev wlan0 get power_save` 返回 on，NetworkManager 连接沿用
系统 `wifi.powersave=3`。已在 Sunrise 执行以下修复，未重启 Wi-Fi 连接：

```bash
sudo iw dev wlan0 set power_save off
sudo nmcli connection modify TopSpeed5GCPE-5525 802-11-wireless.powersave 2
iw dev wlan0 get power_save
nmcli -g 802-11-wireless.powersave connection show TopSpeed5GCPE-5525
```

当前输出分别为 `off` 与 `disable`，后者持久化到该 Wi-Fi 连接；机器人侧有线
路由仍为 `eth0 src 192.168.123.99`。现场更换无线连接时须核对实际连接名。
对照支持省电模式是这次链路断流的重要原因，不保证无线永远不再抖动。
350 ms 停车保护与断流后必须重新启动客户端的规则保留，不能延长旧速度保活掩盖断流。

同时修复 Windows 客户端的另一处可达阻塞：原采样线程发送后同步打印，注入
550 ms 控制台输出阻塞后，松键零指令间隔实测变为 610 ms。用户未选中文字，
不能据此认定本次现场由 QuickEdit 引起。客户端关闭 QuickEdit、移除采样循环打印，
按截止时间调度 20 Hz，落后时不补发旧时槽，并在退出后汇报最大发送间隔与阻塞耗时。
本次修复证据在 `build/go2-deploy-20260910/keyboard-link-probe-90s.json`、
`keyboard-link-powersave-off-90s.json`、`keyboard-console-stall-probe.json`；
连接配置与操作说明仍以本 README 为入口。实际按键运动与避障需要继续现场验收。
客户端本次 22/22 回归及独立审查通过（`keyboard-console-fix-tests.log`）。
修复后已替换旧控制窗口，NX 再次确认 `go2-keyboard` 的新控制权与样本被接纳，
输入门为 ready、循环健康、初始导航与驱动均为零；这项连接确认不替代后续按 W 的现场验证。

### 低速遥控轨迹起步过慢：`.13` 修复与监督验收

无线修复后，用户反馈按 W 仍看不出移动。此时 `go2-keyboard` 的控制权和样本持续
接纳，输入门 ready，Go2 状态为 standing、motors_enabled，驱动无命令拒绝。
保存的最后一次运动窗口持续约 1.66 s，83 个驱动样本的命令积分仅约 7.97 mm，
实际里程计位移约 0.23 mm。不能把它解释成键盘未连接或电机未使能。

真实 SCAN preview 有 598 个轨迹点，每步 0.05 s，时长约 29.8 s；
前 1.649 s 的目标偏移仅约 3.44 mm，轨迹切线速度约 0.0096 m/s。
0.2 m/s 在此链路中作为规划和跟踪的速度上限，并非直接恒速指令；
固定 3.5 m 参考线的一整段低速多项式初始化，把加速分散到很长时间内。
配置 0.5 m/s² 也只是加速度上限，不保证起步必须以该加速度上升。

问题位于 `src/nav/cpp/planning/local/scan/upstream/plan_manage/planner_manager.cpp`
的初始轨迹时间分配与 B-spline 采样：原始采样间隔随 `ctrl_pt_dist / max_vel`
增大，0.2 m/s 时约 1.2 s，无法细致表达短加速阶段。最终曲线而非初始化 seed
必须通过起步速度、速度/加速度上限和碰撞检查，才能证明修复有效。
相同方向的持续 W 不会每条指令都 reset FSM；该假设已由代码链路排除。

落键后的 `last_local` 保留历史曲线/命令，但 tracking 被 stopLinearMotion 清空，
故其 `trajectory_id=0` 不能反推运动时一直重置。现场证据在
`build/go2-deploy-20260910/keyboard-no-response.json`；后续只读 90 s 窗口全部为零
请求，不能当作新的按键运动验收。随后完成 MotionIntent 初始化修复与回归，
保留参考线、碰撞视野、原生安全仲裁与按键速度上限，修复已随 `.13` 部署。

低速回归还确认了时间调整中的独立错误：`UniformBspline::checkFeasibility`
把未超限加速度的哨兵 `-1` 参与 `sqrt(abs(value) / max_acc)`。恒速
0.051 m/s、限速 0.05 m/s、实际加速度为零时，正确时间倍率为 1.02，
原代码却返回 `sqrt(2)`，额外减慢了遥控响应。回归已先复现失败；修复仅让
实际发现的超限量决定倍率。该数值不是机器人实测加速度。

候选修复为 MotionIntent 的完整参考线生成加速、巡航、减速初值，并细化加速段
采样。低速增加控制点后，用现有 Eigen SparseQR 对同一最小二乘矩阵一次分解、
求解三轴，避免重复构造和分解稠密矩阵；没有增加依赖。优化后最终样条仍须
重新检查速度与加速度，不能只验证初值。仅反复调用原有软约束细化仍会重新
产生超速，已在 Windows 与 NX 同样复现，故该中间候选没有安装。

新候选仅对 MotionIntent 固定首末位置、速度和加速度，再做时间调整并恢复
这些物理边界，避免清零实际起始状态。最终整条曲线使用导数控制点的三维范数
上界检查速度与加速度；调整后复用原有前 2/3 区间的机身轮廓、线段和朝向碰撞
检查，后段仍由原 FSM 随执行推进检查。固定目标规划保留原有细化路径。
NX 上数学回归 3/3 已通过（176 ms），这不代表整条遥控链路或实机运动验收
通过；前进、侧移、反向起速及低速输入的结果与安装状态将在验收后记录。

最终候选在 Windows 与 NX 上均通过四组遥控链路回归：持续低速意图、0.05 m/s
输入、六种方向/初始速度的实际发布样条导数与边界、遇墙停止或重规划。
NX 在不运行现场 Product 的条件下，0.2 / 0.05 m/s 的初始化分别为
34.32 / 52.36 ms（单次观测）；1 s 输出分别为 0.190071 / 0.046891 m/s。
0.2 请求旧版同条件 1 s 仅输出 0.005896 m/s。这些数据使用理想运动积分，
不是 Go2 的实际位移或实际加速度。命令变化率与参考样条加速度分别记录，
不能把规划器的 0.5 m/s² 上限当作所有控制输出的硬变化率限制。
非零起始速度和加速度、已达到请求速度的样例也通过了直接规划器验证。

完整 NX 导航回归首轮 422/424 通过。剩余两项旧测试在 trajectory 首个 ready
时刻要求正速度，此时执行时间恰为零，精确的静止 P/V/A 应产生零输出。
测试改为先验证零时刻，再明确推进执行时钟后验证原有的恢复、方向和限速合同；
未修改生产代码去制造非零首帧。七项同类测试收尾在 Windows 与 NX 上均 7/7
通过，因此 424 项导航回归均有通过证据（首轮 422 项、修正后重跑受影响七项，
不是第二次完整 424 项运行）。运动后的侧移允许优化曲线有微小另一轴分量，
以 1° 方向误差、正确左右符号、零转向和请求速度上限验证；实测最大误差
0.509°，没有强制投影避障输出。证据为 `keyboard-response-arm-suite.log` 和
`keyboard-response-arm-direction.log`。新发布包安装状态仍以本 README 顶部实测表为准。

`.13` 通过标准发布脚本打包与安装，ProductControl 启动 `teleop_avoid + SCAN`
成功，实际 DDS domain 为 0。专用 motion 只读门通过，Go2 为 standing、
motors_enabled，初始导航和驱动均为零；整体诊断列出的未启用相机、探索等项
不属于此 Product 的验收要求。新窗口随后以 `go2-keyboard` 获得控制权，
输入门 ready，现场按键时导航输出约 0.15 m/s、驱动已接纳；这些是命令值，
不能等同于实测机器人速度。用户随后明确确认“已移动，松键后停车”。
本轮助手没有自动发出非零运动指令。

运动期间仍观察到 `p95_utilization_high`，因此本次只关闭“WASD 无响应及松键
停车”的问题，不宣称长时间实时性、静态绕障或行人动态避障已通过。
发布、就绪和连接证据分别为 `keyboard-response-start.json`、
`keyboard-response-ready.json`、`keyboard-response-initial.json` 和
`keyboard-response-reopened.json`，统一位于 `build/go2-deploy-20260910/`。
后续 30 s 只读采样保存为 `keyboard-response-live.jsonl`，共 295 帧，末帧遥控请求、
导航和驱动命令均为零，SDK 报告平移速度为零、姿态 standing。采样中 128 帧
带 `p95_utilization_high`，这是状态采样数量，不是 128 次独立故障。

## 2026-09-11 绕障漏段修复与机载复核

本轮针对“遇到障碍停住但不绕开”，确认 SCAN 初始化的一处代码缺陷：
`initControlPoints` 只扫描前 2/3 控制点，若在这段内进入障碍、到后段才退出，
碰撞段没有闭合，DynAStar 就没有获得这段的引导任务。修复仅继续扫描已经
进入的碰撞段直至退出，不扩大新碰撞段的启动范围，也不放松最终碰撞检查。
新增回归在旧代码上得到零条引导；修改后，有限长障碍的 MotionIntent 最终
轨迹能绕行，完整曲线每 2 ms 的线段检查通过，导数控制点范数满足
0.2 m/s 和 0.5 m/s² 上限。本机 SCAN 56 项通过；NX 完整导航回归
426/426 通过（63.44 s），包括两项新增绕障回归。
这些是原生离线证据，不是实机绕障运动验收。

历史失败快照 sequence 16 属于 180° 后退请求，不能冒充新的 W 前进记录。
完整 50 万字节位图可精确复现旧失败。固定机身朝向、原高度与 ±90° 边方向
条件下，5 cm 和 2.5 cm 平面搜索均未找到通路；取消方向限制才得到需反向
移动的 16.84 m 绕路。离散搜索失败不证明连续空间绝对无路，也不能把放宽
条件的路线作为原请求通过。此快照在修复后仍正确拒绝候选，没有强行放行。
另需后续补齐的合同是 SCAN 尚未像 CMU 一样使用 `maxDirectionDeviationDeg`
限制搜索；上述 ±90° 是诊断额外施加的条件，不是已经验证的 SCAN 运行约束。

同时合并 `MotionLayer::updateFromScan` 中重复的点云遍历与体素键计算，保留
每点命中计数、高度代表点和射线清除规则。41 帧历史雷达扫描的全部快照、
逐点查询状态和预测输出，在本机及 NX 上与修改前完全一致；交叉射线、同体素
重复命中与 InputProjector 回归通过。NX 无导航服务时，451 个计时样本的
处理总耗时中位数为 4.055 → 3.966 ms，p95 为 5.012 → 5.108 ms：
没有证据声称尾延迟改善，因此循环负载告警仍待现场复核，不能沿用本机较大
降幅宣传机载性能。`motion_update_last` 和 `obstacle_snapshot_last` 保留的是
最近一次云处理阶段耗时，不代表每个 100 Hz 周期都执行这些工作。

证据统一在 `build/go2-deploy-20260910/`：`avoidance-replay/` 保存原失败、
`avoidance-replay/topology/` 保存连通性诊断与图，`avoidance-replay/fixed/`
保存修复后对原失败的回放，`motion-replay/comparison-arm.json` 保存机载
性能及结果一致性对照。上述代码已随 `v2.3.0-go2.20260911.1` 安装，六个
Product 服务 active、SLAM TRACKING，专用 motion preflight 通过、blockers
为空；`avoidance-release-start.json`、`avoidance-release-ready.json` 和
`avoidance-release-initial.json` 记录发布与就绪证据。

### 新 W 失败、实际速度与厂商避障状态

新版现场快照 `avoidance-acceptance-failure-99.json` 的 sequence 9 是 W 前进，
不是前述历史 S 请求。冻结完整碰撞位图后可复现相同失败，94 个控制点的最大
差异为 1.34e-14 m；约 2.96 s 的候选位置触碰栅格，最后为
`rebound_optimization / collision_restart_limit`。直接初始化引导探针扩展
477 个节点后得到零条引导。固定高度和机身朝向的全 ROI 5 cm 离散搜索，
有或无 ±90° 限制均没有找到连通路。这说明该快照中不是简单调优化权重就能
保证绕过；仍须核对现场通道和地图，不能据离散结果断言连续空间绝对无路。
证据在 `avoidance-forward-replay/`，图为其 `topology/collision-plane.png`。

重新打开标题为 `LingTu Go2 - WASD obstacle avoidance` 的窗口后，90 s
只读采集得到 447 个导航状态：171 个 `teleop_assist_spline_ready`、2 个
`local_intent_pending`、274 个零输入状态；这次没有初始化失败，但并不代表
机器人跟上了轨迹。完整的第一次前进 run 25 持续 6.781 s，指令积分
1.219 m，SDK 速度积分 0.505 m，里程计净位移 0.480 m，平均净移动速度
约 0.071 m/s；第二次前进 run 26 持续 6.661 s，指令积分 1.000 m，
净位移仅 0.020 m。此前进度中的 5.76 s / 0.48 m 是 run 25 尚未结束的
截面，完整数据以此处为准。现场是否接触障碍仍未得到明确确认，不能把第二次
数据称作已验证的空旷平地速度实验。

独立只读 SDK 订阅器直接观察 `rt/api/sport/request` 与 `rt/sportmodestate`：
运动段存在约 0.2 m/s 的 API 1008 Move 请求，周期约 20 ms；例如持续 W
第 4–5 秒仍为约 0.20 m/s，而 SDK 实际平移速率约 0.08、0.058 m/s，
这两秒内没有 API 1003 StopMove 插入。该证据排除了这段由键盘断流或统一
指令缩放造成慢速，不能排除机器人本身、厂商限制或现场受阻。厂商原始
mode/gait/error 固定回报 0/0/100，含实际运动期间；其数值含义未核实，
不据此推断机器人没进入步态或硬件故障。原始证据为
`sport-observe-wasd-long-20260911.jsonl`，导航与完整运动段汇总为
`avoidance-wasd-20260911.jsonl`、`avoidance-wasd-analysis.json`。

SDK `ObstaclesAvoidClient::SwitchGet` 实测成功返回 `enabled=true`，
见 `sport-query-20260911.jsonl`。厂商避障是当前真实存在的执行条件，但开关
开启不等于已经证明其正在限速；本轮没有调用 SwitchSet、切换步态或关闭保护。
排查工具只订阅数据、查询开关和 ServiceList，未发布非零运动请求。

该次导航状态中仍有 75 个 `p95_utilization_high`、13 个
`deadline_miss_ratio_high`，这是滚动状态采样数量，不是独立故障次数。
上述持续 Move 请求与实际速度差不能仅归因于循环负载。最新点云采集得到
15 个严格时间匹配帧（4 s，未达到脚本 20 帧的完整采集门槛），只用于几何
诊断；主要地面高度约在 body 下方 0.30–0.35 m，目前无证据宣布整片地面
被误挡，也不能拿这批新点云替代旧失败时刻的原始点云。

接下来须在已确认空旷、有人监督且可立即停车的条件下，对齐请求、最终指令、
厂商接收请求和实测运动，验证起步、持续速度、侧移、松键停车；然后用有明确
两侧通道的固定障碍验证点云、绕行轨迹和实际执行。当前不提高速度下限、
不放松碰撞判定，也不把“样条 ready”当作物理绕行完成。

## 2026-09-11 厂商避障关闭、0.5 m/s 与键盘超时恢复

用户明确要求关闭 Go2 自带避障，并把 WASD 测试请求调至 0.5 m/s。
先结束旧键盘会话并确认 native `zero_barrier_complete`、无控制权和零输出，
再调用 SDK `ObstaclesAvoidClient::SwitchSet(false)`；设置成功后
`SwitchGet` 回读 `enabled=false`，测试后再次查询仍为 false。
只改厂商开关，LingTu 仍为 `teleop_avoid + SCAN`、`check_obstacle=true`、
`teleop_local_planner=true`、`manual_mode=0`，最终碰撞和制动检查仍执行。
没有切换步态、没有绕过 `lingtu-driver`，助手未自动发送非零运动请求。
未验证该厂商开关跨重启的保持行为，后续开机应查询实际值。

本次窗口使用当前已核实 domain 0 的启动参数：
`start-wasd.ps1 -DomainId 0 -Speed 0.5`；这是测试窗口请求上限，不是实测速度，
也未修改默认 0.2 m/s 或增加最低速度补偿。窗口新增显示实际启动的速度参数。
两次 motion preflight 的唯一失败是嵌入回执落后 3 个输出，而检查最多允许 2；
驱动 50 Hz、导航 100 Hz 的相位采样可产生该差异。3 s 零输入连续观察得到
1–3 的变化，后续完整新鲜 preflight 在原检查条件下通过、blockers 为空，
没有放宽回执检查。较慢更新的独立 driver JSON 不能与最新导航序号强行逐帧
比较。证据为 `factory-off-speed05-ready*.json`，通过样本为 `*-ready-live.json`。

用户随后现场操作的 run 39 持续 5.941 s，指令积分 2.329 m，SDK 速度积分
1.848 m，里程计净位移 1.696 m，净移动平均约 0.285 m/s；相较此前存在改善，
但本次同时改变厂商开关和速度请求，且包含转向、反向与障碍附近运动，不能
据此单独认定此前慢速由厂商避障造成，也不是静态绕障通过。采集中仍有
初始化失败、局部目标阻挡和实际制动停车，均继续保留为待解决项。

该窗口后来触发原生 350 ms 输入超时，用户日志为 1,287 次发送，
最大发送间隔 1,078 ms、最长 SSH 写入 16 ms、一次间隔超过 350 ms。
现有证据把主要停顿定位在发送操作之外，尚不能区分 Win32 按键查询与 Windows
调度/唤醒延迟。现场最后状态确认零输出、控制权释放、SDK 平移速度为零。
退出后的 `Press Enter` 中连续 `wwww...` 是控制台缓冲文字，不是仍在发送运动。

本机键盘客户端现保留 350 ms 机载超时与停车规则，并补齐恢复逻辑：

- 只在收到明确的 `LT_TELEOP_STREAM_TIMEOUT_STOP_V1` 且旧进程成功清理、
  正常退出后重连；普通 SSH 故障、未确认停车仍退出，不自动重连。
- 每次连接从零输入和未解锁的键盘状态开始；所有运动键释放后再按才会运动。
  本地采样/发送间隔达到 350 ms 时丢弃这次非零采样并重新要求松键，
  不重放暂停前的按住命令。Ctrl+C 中断清理时仍由 finally 完成停车确认。
- 额外记录最大按键采样耗时、唤醒延迟以及最差间隔的发送序号/会话时间，
  只在退出后打印，不在控制循环内输出。

`tests/lingtu/test_operator_keyboard.py` 33 项通过，覆盖上述恢复和中断情形，
相关 Ruff 检查通过；这证明本地恢复合同，尚未证明 Windows 一秒停顿已消失。
更新后已重新打开 0.5 m/s 窗口，等待实际使用验证。所有记录位于
`build/go2-deploy-20260910/`，包括 `factory-avoid-disabled-20260911.jsonl`、
`factory-off-speed05-query-after.jsonl`、`factory-off-speed05-analysis.json`、
`keyboard-timeout-recovery-tests.xml`；新窗口遥测为
`keyboard-recovery-speed05-20260911.jsonl`。

## 2026-09-11 持续 W 慢启动与起点碰撞的进一步定位

本轮必须区分两种表现，不能都归因于键盘延迟或规划计算慢：

- 较早的 `factory-off-speed05-20260911.jsonl` 首段连续 W，轨迹编号始终为 1，
  执行时间正常推进，没有冻结或反复重置。首次采样后约 0.20 / 1.01 / 2.01 /
  2.22 s，最终平移**指令**模长约 0.011 / 0.212 / 0.484 / 0.500 m/s。
  轨迹从零加速度平滑起步，0.5 m/s、0.5 m/s² 参数对应至少 1.5 s 的种子加速段，
  优化后的整条曲线又统一调整时间以满足速度上限。这里的 0.5 m/s² 是上限，
  不是实测加速度，也不保证 1 s 达到目标速度；这些采样没有证明重复重置缺陷。
- 最新 `held-w-current-20260911.jsonl` 的 40 条非零 W/S 状态中，请求为
  ±0.5 m/s，但最终输出全部为零。其中 39 条是 `scan_initialization_failed`，
  1 条是生成轨迹的等待状态；失败诊断为 `collision_at_trajectory_start`。
  失败 75 和 83 的完整位图离线重放均复现 t=0 碰撞，单次约 3 ms，
  不是等待数秒后才完成一次搜索。

失败 83 的位姿为 `[-0.189454, -0.060114, -0.004600]`，航向约 -0.005586 rad。
精确索引得到前圆柱中心栅格 `[-1,-2,-1]` 为空，后圆柱中心 `[-8,-2,-1]`
被占用（位图索引 1900091）。现有严格边界退出检查的 360 个方向均未找到
合格出口，因此不能靠提高速度或绕过起点碰撞使它运动。

随后取得 35 帧、约 8 s 的扫描位姿匹配点云，位置基本不变；对应两个中心
栅格均无新的命中贡献。但点云晚于失败约 471 s，且 Go2 的 FAST-LIO 配置
会去掉距雷达 0.5 m 内的回波，其中一个可能的历史占用来源处于该范围。
所以当前没有回波不等于已经证明安全，也不能据此删除占用、缩小机身范围，
或开启超时消障。数据未证明重复膨胀、普通地面误挡或机器人自反射。

本轮确认并修改的代码问题：

1. SCAN 在达到重复失败上限后清除持续 MotionIntent 的目标，外层仍保持相同
   参考路径，导致清障后持续按住 W 也可能等不到新目标。现在停稳后保留仍有效
   的 MotionIntent 并重试；松键/取消由原有 Executor 重置路径清除，Route
   的原有终止语义不变。本地 9 项针对性回归通过；NX ARM 构建及 7 项回归通过。
2. `RollingOccupancyGrid::TraceRay` 用端点与雷达的**栅格索引差**代替真实
   空间方向，导致清空射线偏离测量线段。当前数据中已观察到实际穿过的单元漏清，
   以及测量线段未穿过的单元收到 miss。已改用连续坐标差，正确处理零方向轴、
   雷达边界终止及边角相交，不再清空只被射线边角碰到的旁侧单元；
   命中优先、每帧去重、膨胀和占用保留策略不变。该缺陷本身已确定，但尚未证明
   它是后半身占用的唯一来源；还存在当前射线完全看不到的候选来源。本地地图
   28 个用例通过，旧射线实现会失败；NX ARM 的 rolling occupancy 与 mapd engine
   两个测试入口均通过。

用户重启后已恢复连接，完成 ARM 构建、打包和安装。`/opt/lingtu/current`
现指向 `v2.3.0-go2.20260911.2`，源码快照 `cd83253`。通过正式
`scripts/lingtu switch` 入口启动 `teleop_avoid + SCAN`，严格 motion 只读门通过、
blockers 为空；该检查没有取得控制权或发布命令，导航与驱动均为零输出。
重启前订阅到的最新碰撞位图（generation 82959）与失败 83 为同一 epoch，
同一后半身单元仍被占用；这排除了仅仅查看过期失败文件的解释，但没有确定
具体物体。地图修复的 ARM 验证和版本安装已完成，但随后现场运动验收失败，
尚未解决起点碰撞停车，实际绕障与速度跟踪未通过。
助手本轮未自动发送非零运动请求。

`.2` 安装后的 `release2-acceptance-20260911.jsonl` 完整 119.987 s 共 596 条状态中，
42 条有有效操作者输入：29 条 `scan_initialization_failed`、12 条
`local_intent_pending`、1 条 `scan_generate_trajectory`，最终指令全部为零。
首次 W 的 0.5 m/s 请求在 `1789071741.838459` 已新鲜到达，控制权和键盘链路
有效；最大请求年龄 41.844 ms，没有不健康循环状态或驱动拒绝。规划报告
`collision_at_trajectory_start`，
碰撞位于 t=0，单次计算约 1.36 ms。用户同时确认机器人没有移动。因此本次失败
不能归因于 W 未送达或规划耗时数秒，修复没有使这次现场试验通过。汇总为
`release2-acceptance-summary.json`。

随后实时位图中的残留占用来源候选格为 `[-8,3,7]`。在同一张位图上查询新采集的
32 个位姿，机身 z 范围为 `[-0.002626,+0.001494] m`，后圆柱中心 18 次为空、
14 次占用；末帧 z 约 -0.000003 m，距离分格边界仅 3 μm。因此起点并非稳定
空闲，也不能把过去的失败文件当作当前状态。242,544 条新测量射线既未命中、
也未穿过该来源格，不能用没有新回波证明它已清空；来源是否为机器人自身尚未确定。

当前实时位图与末帧位姿的离线重放可输出 1.187 m 轨迹段，最大速度约
0.4896 m/s，1,948 个完整点/段检查样本无碰撞。它不是完整 3.5 m 路径，也不证明
现场绕障通过。第二轮 120 s 采集 `release2-retry-20260911.jsonl` 共 595 条状态，
没有非零输入或非零输出，用户尚未完成第二次运动试验；现场验收结论仍为未通过。

本次重启把厂商避障开关恢复为 `true`。按用户先前要求，通过只读核对与
`SwitchSet(false)` 再次关闭，返回码为 0，回读为 `false`；LingTu 规划与最终
安全检查继续启用。因此厂商开关不能视为跨重启持久化配置，每次现场验收前
需读取实际状态。NX 证据为源码目录下
`build/go2-release-20260911-2-vendor-disable.jsonl`。

将这 35 帧后采集的点云从空地图离线重建后，W 方向的 MotionIntent 在
0.5 m/s 上限下可生成轨迹，4,871 个全轨迹采样均无碰撞。这只证明该批可见
点云上的离线结果，不能证明之前未再观测到的占用已经消失。8 个方向中，
6 个全轨迹检查通过、1 个明确受阻；另 1 个方向虽生成轨迹，但后段有 884 个
占用采样，首次出现于 15.541 s，位于现有近端 2/3 检查范围之后。该方向需继续
验证滚动重规划能否在进入后段前更新路径，不能宣称所有方向的整条轨迹均已验收。

证据位于 `build/go2-deploy-20260910/`：`held-w-current-20260911.jsonl`、
`held-w-current-failure-{1,571}.json`、`held-w-replay/`、`held-w-latest-replay/`、
`held-w-cloud-current/`、`held-w-map-analysis/` 和 `held-w-liveness-20260911.patch`。
现场继续读取当前占用与原始命中/清空证据，再验证实际绕障和松键停车，
不能把重建后出现轨迹直接等同于真机避障通过。

### 雷达图与安装参数核对

`held-w-map-analysis/collision-contributors.png` 是排查图：叠加失败时栅格、
晚约 471 s 的点云和可能的历史命中格子，不是同时刻实时地图，不应用它判断
雷达整体误差。独立的 `held-w-cloud-quality/single-frame-floor.png` 只显示一帧
点云与该帧地面拟合，没有叠加旧栅格。

已核对 NX 正在运行的 `slamd`（PID 5314，DDS domain 0）通过 `--config` 使用
`.2` release 下的 `config/robots/unitree/go2/sensors/mid360_fastlio2.yaml`。
该安装配置仍使用 Go2 MID-360 标称支架参数：`T_body_lidar` 平移
`[0.16143, 0, 0.12262] m`，俯仰 +13°。FAST-LIO 的 `r_il=I`、
`t_il=[-0.011,-0.02329,0.04412] m` 是 MID-360 内置 IMU 与雷达之间的参数；
`navigation_body_from_imu_*` 与其组合后恰好得到上述雷达在机身坐标系中的位姿。
FAST-LIO、MapObservation、mapd 的变换链未发现重复应用安装外参。
`esti_il=false`，没有在本轮根据占用图猜测新的安装角。

`calibration.slam.status=verified` 对应 2026-08-23 的静态验证，不能解释为
当前支架外参已重新实测。本轮对重启前 03:09:53–03:10:01 保存的 35 帧数据
重新拟合可见地面：16,684 个支撑点，合并 RMS 5.37 mm，95% 绝对残差
10.27 mm，倾角约 0.827°；单帧 RMS 3.84–6.59 mm。机身各轴移动仅数毫米，
重启后 04:25:38–04:25:46 CST 又采集 32 帧、242,544 个点，地面拟合 RMS
5.669 mm、倾角 0.73985°，单帧图和分析位于 `release2-cloud-quality/`。
这些结论限于静止局部地面的一致性，不证明绝对地图精度、运动漂移或完整外参。
不要把地面倾角直接当成安装角修正量。

### 2026-09-11 建图链路复核：已复现射线去重漏清，未完成卡点修复

这次是本地源代码检查及已保存实机点云回放，没有连接机器人、发送运动、清地图，
也没有修改生产算法或硬件参数。结论不能写成实机已恢复。

仿真与实机不是两套完全独立的导航算法。完整 MuJoCo Product 和 Go2 的主要链路
都包含原生 SLAM、Mapd、SCAN、跟踪与最终安全检查；差异在传感器/驱动实现、
RobotConfig 和所运行的 Product。此前通过的真值隔离实验绕过了真实 Fast-LIO
估计，点击到点的 `nav` 也不等于持续 W 的 `teleop_avoid` 执行分支。
因此“都使用 SCAN”不能证明完整输入与恢复行为相同。

| 检查位置 | 本次确认的事实 | 结论边界 |
| --- | --- | --- |
| SLAM 输出 | `fastlio.cpp` 将本次处理的点云转换为 body，`cyclone_runtime.cpp::toDdsMapObservation` 用扫描时刻 body 位姿及 `map<-odom` 组合点云变换，射线起点另用物理雷达安装位姿 | 当前代码链未发现二次安装变换；不代表当前支架已经实测标定 |
| 原始点过滤 | `toPclCloud` 按 `lidar_filter_num=2` 取样，并按 `lidar_min_range=0.5` 排除近点；Mapd 接收的是这条链的已处理点云 | 0.5 m 是软件过滤，不是已测定的硬件盲区；远处有效射线仍可能穿过近处格子 |
| 仿真自体处理 | `sim/compat/engine/mujoco/lidar.py::_exclude_robot_geoms` 在射线投射前排除机器人所有子体的碰撞/视觉几何，正式 feeder 经 MuJoCo runtime 使用此雷达 | 无法覆盖真实自体返回与机身遮挡；不能由此断言历史占用就是自反射 |
| 地图保留 | SCAN profile 明确将 `OCCUPANCY_DECAY_AFTER_S` 设为 0；历史占用靠测量射线 miss 或滚动窗口移出清理 | 正确的保守保留与漏清缺陷会叠加；没有新回波不是自由空间证明 |
| 机身净空 | Go2 上下净空仍为 0.35/0.25 m，缺少当前载荷的实测依据 | 必须量测后校准，不能为消除停车而缩小轮廓 |

**新确认的缺陷在 Mapd 的射线融合，不需要假设 Fast-LIO 输出错误。**
`src/maps/cpp/layers/rolling_occupancy.cpp::Update` 有两处提前截断：
同一端点体素的后续射线直接跳过；逆向射线碰到本帧已遍历体素时立即 `break`。
两条测量线段在一个体素内相交并不保证余下的体素路径相同；因此这类去重会漏掉
后续可见空闲单元。当前 `TraceRay` 在这些判断之前已经生成整条射线，截断发生在
证据合并阶段，不是 Fast-LIO 求解阶段。

独立 C++ 程序直接编译当前 `rolling_occupancy.cpp`，读取已保存的
`held-w-cloud-current/frame-0000.xyz64`（7,763 点）和同帧物理雷达起点：

| 同一测量输入的处理方式 | 单元 `[-11,-4,3]`（3 条射线穿过） | 单元 `[-11,-5,3]`（2 条射线穿过） |
| --- | --- | --- |
| 当前生产代码、原始点序、空地图 | 未观测，无 miss 更新 | 未观测，无 miss 更新 |
| 当前生产代码、将穿过这两格的点移到前面 | 空闲 | 仍未观测 |
| 实验副本取消两处提前截断，原始/逆序/穿越优先 | 空闲 | 空闲 |

为单独验证历史占用清除，还在上述两格及两个无穿越射线的对照格中人为预置
0.98 的占用概率，再回放同一帧四次。当前代码保持四格 0.98；实验副本将有
真实穿越的两格降到约 0.6231、低于 0.80 占用阈值，无穿越的两格仍为 0.98。
该预置仅用于检验清空行为，不是恢复了当时的真实历史命中。实验没有调低占用阈值、
改变安装外参、缩小轮廓或通过超时删除障碍。

这些点云比故障 83 晚约 471 s；本次不能证明那一时刻的原始输入及调度顺序。
后来现场仍可能存在射线根本看不到的占用来源，因此修正漏清也未必单独解除卡点。
回放耗时只来自当前 Windows 开发机，不作为 NX 性能结论。

复现程序、显式实验副本和完整结果在 `build/go2-map-audit-20260911/`：
`prepare.py`、`audit.cpp`、`current.jsonl`、`no-early-exit.jsonl`。
`no_early_exit` 仅为诊断用编译目标，未接入发布包。下一步正式修复需确保保留
真实端点阻挡、每帧概率更新与边界语义，并补全相交射线/同端点体素的窄回归，
再验证静态障碍、动态清除及 NX 帧处理耗时。现场还需同时采集原始命中、miss、
位姿及碰撞 bitmap，确定当前起点占用的真实来源；不通过重建空地图冒充修复。

## 建图时查看局部与整图

现场的实时地图是 mapd 的移动局部窗口；`局部地图` 和 `局部投影` 都不代表
整趟建图记录。点移出窗口后可能不再显示，不能据此断定 SLAM 没有保存该区域。

- **保存地图**：输入一个新地图名，等保存完成后自动打开该地图的整图快照。
  该操作保存当前 SLAM 数据并生成地图产物，不切换 Product、不自动启动导航。
- **已存地图**：打开地图库，点击地图名即可查看，列表随即收起；点击顶部地图名
  可重新展开列表，地图的重命名、删除等操作收在每行的 **…** 中。
  默认 **2D** 俯视检查平面覆盖，**3D** 查看空间结构，**显示整图**重新取景，
  **现场**回到实时地图。顶部 **已保存** 展开快照说明。
- 建图现场默认展开地图画布，**视图**收纳图层与视角，**状态**展开详细面板；
  实测速度、数据缓存/不可用提示仍常驻显示。
- 整图预览按高度着色，颜色不等于可通行分类。预览会对整个 PCD 均匀采样，
  不是仅显示机器人附近；点数是展示点数。空白处仍需结合实测确认，不能作为无障碍证据。
- 整图快照不会随着继续行走自动更新。补扫后再次保存一个新名字即可对照；
  保存会占用 SLAM 处理时间，宜停稳后按需保存，不作高频自动刷新。

当前没有持续发布的、尚未保存的完整 SLAM 全局点云接口。保存的完整范围也以
SLAM 当时保留的数据为准，并非无限历史：Go2 Fast-LIO 配置为 `cube_len=1000`、
`max_map_points=1000000`。不要用 Python 另建累计地图来冒充 native 保存结果。

## Navigation collision geometry

The real Go2 RobotConfig keeps each horizontal safety quantity under one owner:

- Physical envelope: `0.76 x 0.31 m`, using Unitree's larger published crouching length.
- CMU and final safety: the physical rectangle plus one `0.10 m` hard margin per side.
- SCAN: the Go2-tuned heading-aligned double cylinder, `radius=0.25 m` and
  `offset=0.18 m`, with no additional footprint padding.
- `live_obstacle_inflation_radius_m` belongs to active-path blockage overlay
  diagnostics. It is not added to the robot hard footprint.

The vertical clearances are defined relative to LingTu's `body` origin. The
current `0.35 m` above and `0.25 m` below values match the Thunder/runtime defaults;
no measurements of this Go2 and its installed payload support them yet. A fixed
bare-Go2 model does not represent that payload and cannot justify shrinking the
clearances. Upstream SCAN obstacle Z inflation does not validate those dimensions.

The deployment host address is site state and is intentionally not part of the
model. SDK2 motion discovery uses `driver.network_interface` from
`robot.yaml`, not a configured Go2 target IP.

## Navigation deployment gates

Before starting the `nav` Product on the expansion Linux computer:

1. The host must have a reachable interface on the Go2 wired subnet; set
   `driver.network_interface` to that host's actual interface name.
2. Unitree SDK2 must be installed and the Go2 motion implementation must build on the
   target architecture.
3. Validate the nominal body-to-LiDAR composite derived from the official Go2
   URDF plus the archived MID-360 interface mirror. Update both
   `robot.yaml` and `sensors/mid360_fastlio2.yaml` if the installed
   bracket differs, and change `calibration.status` only after the calibration
   check passes.
4. Verify the MID-360 host/device addresses and obtain a fresh map with the
   point-cloud, occupancy, and OctoMap artifacts required by `nav`.
5. Install the `lt-*` units, run a no-motion readiness check, and only then run
   a supervised bounded-motion acceptance.

## Go2 EDU + external MID-360 field target

The current first field target is:

- Sunrise is a temporary development bridge only.
- The Go2 expansion Linux/NX computer is the production runtime host.
- `teleop_avoid` uses the external MID-360 point cloud. Fast-LIO also consumes
  the MID-360 package IMU; the camera is not an input to this first acceptance.
- Operator input is physical velocity (`m/s`, `rad/s`) over `/ws/teleop`, then
  typed DDS operator motion, native local planning/final safety, and finally the
  Go2 SDK2 motion adapter.

The Go2 configuration uses the nominal Unitree companion MID-360 bracket pose
from an archived interface-document mirror, composed with the public Go2
base-to-IMU transform. It is intentionally separate from both the Go2 built-in
L1 radar and Thunder V4's front-nose mount. The adjacent `robot.yaml` records
the verified MID-360 result and keeps the unmeasured camera separate.

There is also no official public default SSH username/password for a Go2 EDU
expansion Jetson. NVIDIA Jetson images create or pre-provision the account; use
the credentials delivered with this particular NX image. Do not add guessed
credentials to the repository.

Official references:

- [Unitree Go2 specifications](https://www.unitree.com/go2/)
- [Unitree Go2 accessories: D435i and MID-360](https://www.unitree.com/go2/charger/)
- [Unitree Go2 body/front-camera xacro](https://github.com/unitreerobotics/unitree_ros/blob/daadf41ee9afce8f90fdc09a98506012691fa122/robots/go2_description/xacro/robot.xacro#L95-L102)
- [Unitree MID-360 SLAM selector example](https://github.com/unitreerobotics/unitree_slam/blob/1e49bfa4dca2c992a566ee7d1d8480d4102149b8/unitree_slam_example/demo_mid360.cpp#L158-L188)
- [NVIDIA Jetson account provisioning](https://docs.nvidia.com/jetson/l4t/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html)

The executable field sequence is documented in
[`docs/operations.md`](../../../../docs/operations.md).
