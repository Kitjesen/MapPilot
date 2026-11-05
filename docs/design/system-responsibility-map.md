# 系统职责分工与待整理问题

核查：2026-11-01。本文按当前工作区的代码和配置梳理职责，供排查与后续整理使用。
产品选择以 `config/system/` 和实际 RunPlan 为准；运行结果以实际状态和测试记录为准。
设计目标不等于功能已实现，本表不代表任何现场设备已经部署或验收。

## 先看三条主线

```text
启动：Product + env + 机器人配置 → ProductControl 解析一次 RunPlan
      → real 或 sim 的进程管理 → native 服务 + Python Host

运动：目标 → 全局路线 → 局部轨迹 → 跟踪速度 → 最终仲裁
      → lingtu-driver → 平台运控

地图：传感器 → SLAM 位姿/点云 → mapd 实时地图 → 局部碰撞查询
                                    ↓ 新鲜空闲证据可纠正保存图的旧占据
      mapd 保存事务 → 保存 OctoMap → 准备全局表面和路线索引
```

## 产品、运行环境和硬件

| 问题 | 职责和代码入口 | 不负责 |
| --- | --- | --- |
| 产品有哪些能力 | `config/system/products/*.yaml` | 不启动进程、不决定硬件标定 |
| real/sim 具体用什么实现 | `config/system/envs/*.yaml` | 不执行产品切换 |
| 机器人几何、传感器和标定 | `config/robots/<vendor>/<model>/robot.yaml` | 不决定当前 Product |
| 解析生成 RunPlan | [`assembly/compile_plan.py`](../../src/lingtu/assembly/compile_plan.py) | 不启动原生进程；运行快照的发布和提交由 ProductControl 事务负责 |
| 发布运行快照与产品启停事务 | [`control.py`](../../src/lingtu/control.py) ProductControl | 负责提交、启动、切换和停止，不实现导航或 SLAM 算法 |
| real/sim 进程与各自回滚 | [`lingtu/real/`](../../src/lingtu/real/) / [`lingtu/sim/`](../../src/lingtu/sim/) | 不重新选择 Product；两者不强行合并回滚 |
| 实机服务目录与诊断 | [`runtime/service_catalogs/real.py`](../../src/runtime/service_catalogs/real.py) | 通用 Product unit 来源是 `real.yaml`；运行时以已解析 RunPlan 为准。目录提供安装和就绪元数据，不选择机器人型号或启动 Product |
| Python 进程内模块与连接 | [`runtime/host.py`](../../src/runtime/host.py) 和 [`assembly/host/`](../../src/lingtu/assembly/host/) | 不拥有原生进程生命周期 |
| Web/API 传输和状态展示 | [`gateway/`](../../src/gateway/) 与 `web/` | 不实现地图、规划或硬件运动 |
| 硬件速度转发和设备协议 | [`drivers/real/motion/`](../../src/drivers/real/motion/) | 不规划路线；只转发获准命令 |
| 仿真机器人物理与传感器 | [`sim/`](../../sim/) | 仿真结果不能当作实机反馈 |

Host 仍在使用：它承载 Gateway、Agent 和所选 Python Modules。Host 内装配和
ProductControl 的进程事务处于不同层，不应仅因两者都叫“启动”就合并。

## 定位、地图和地形

| 职责 | 当前实现及产物 | 边界 |
| --- | --- | --- |
| 激光和 IMU 采集 | [`drivers/`](../../src/drivers/) → 实测数据 | 设备 I/O，不负责地图或路径 |
| 连续位姿估计与去畸变 | [`localization/ltlio/`](../../src/localization/ltlio/) | 输出里程计和配准点云，不负责产品切换 |
| SLAM 服务与重定位结果提交 | [`localization/slam/cpp/`](../../src/localization/slam/cpp/) | 接收传感器和重定位结果，发布状态/位姿与地图快照 |
| 在线历史图优化 | [`localization/online_mapping/`](../../src/localization/online_mapping/) | 在线调度和后端共同处理关键帧图及校正快照，不取代高频 LTLIO |
| 保存时图优化 | [`localization/map_optimization/`](../../src/localization/map_optimization/) | 处理保存作业；点云和位姿文件读取与在线回放共享 |
| 地图实时占据、自由与未知 | [`maps/cpp/mapd/`](../../src/maps/cpp/mapd/) | mapd 是现场实时地图状态所有者，不发运动命令 |
| 地图保存事务与地图包 | [`mapd/save.cpp`](../../src/maps/cpp/mapd/save.cpp)、MapStore | mapd 协调 SLAM 快照和保存；ProductControl 负责激活地图与产品切换 |
| 保存时点云清理 | [`maps/prune/`](../../src/maps/prune/) | 用历史扫描的可见性反证和局部表面证据分流原始点；不负责实时占据或运动许可 |
| 全局多层表面与推断支撑 | [`octoplanner/surface.cpp`](../../src/nav/cpp/planning/global/octoplanner/surface.cpp) | 从准备的保存地图建立索引；推断面不写回原始占据，也不是足端承重证明 |
| 局部实测地形和机体接触查询 | [`collision/terrain.hpp`](../../src/nav/cpp/collision/terrain.hpp)、SCAN Grid | 用于实际碰撞/接触解释；分类用拟合不能直接当运动放行 |
| 地形风险代价 | [`endpoint/traversability/`](../../src/nav/cpp/endpoint/traversability/) | 独立生产风险信息；当前 nav+SCAN 不消费其代价作为通行许可 |
| 展示点云或占据 | `web/src/components/scene3d/` | 显示大小/合并不改变导航地图；实际重建是单独后端操作 |
| 语义目标、对象和记忆 | [`perception/`](../../src/perception/)、[`decision/`](../../src/decision/)、[`memory/`](../../src/memory/)；地图绑定注释由 mapd 保存 | 分别处理观测、任务意图、语义记忆；不复制权威占据或最终运动仲裁 |

prune 的表面保留、SurfaceIndex 的候选支撑、局部 Terrain 的碰撞解释和
traversability 风险都是“地面”相关功能，但输出含义不同，不能互相冒充。
未接入产品的 Patchwork++ 地面分割和增量清理实验已移到
[`research/maps/`](../../research/maps/README.md)，不属于当前实时建图或保存链。

**2026-11-01 决定：SCAN 暂不使用独立二维 traversability 风险评分。** 当前 `nav`
Product 已关闭该评分且不启动独立 traversability 进程；SCAN 的配置、请求和恢复路径
也不消费该二维评分。保留三维占据、地面接触解释及已接入的支撑判断；这不表示
固定站姿、坡面或楼梯的真实步态已通过验证。`tracking`、`explore` 等其他产品仍有
各自的 traversability 接线，不能因 SCAN 暂不用就删除它们。

## 导航与停车

| 职责 | 当前实现 | 输出及边界 |
| --- | --- | --- |
| 目标接收、取消、全局重规划生命周期 | [`endpoint/nav/app/goal/`](../../src/nav/cpp/endpoint/nav/app/goal/) | 管目标身份和全局规划请求，不负责跟踪 |
| 全局路线 | [`planning/global/octoplanner/`](../../src/nav/cpp/planning/global/octoplanner/) | 从保存 OctoMap 生成 XYZ 引导，使用三维圆柱通道和支撑检查 |
| 路线推进和局部目标/恢复协调 | [`navigation/executor.cpp`](../../src/nav/cpp/navigation/executor.cpp) | 组织局部规划、跟踪与恢复；终点和恢复旋转的速度由跟踪器生成，不构造 SCAN 网格，也不控制硬件 |
| 局部轨迹 | [`planning/local/scan/`](../../src/nav/cpp/planning/local/scan/)；CMU 为另一后端 | 结合路线、实时几何与预测生成轨迹，或报告 pending/blocked |
| 跟踪 | [`tracking/follower.cpp`](../../src/nav/cpp/tracking/follower.cpp) | 从局部轨迹和状态产生期望速度，不拥有最终控制权 |
| 已有阻塞恢复 | [`navigation/recovery.cpp`](../../src/nav/cpp/navigation/recovery.cpp)、`recovery_search.cpp` | 根据阻塞/停滞选择候选，当前不是完整的静止主动避让 |
| 输入状态、动态观测和预测 | [`endpoint/nav/input/`](../../src/nav/cpp/endpoint/nav/input/) | 组织坐标、新鲜度、障碍与预测，不直接选择产品 |
| 最终控制权与命令仲裁 | [`endpoint/nav/control/final.cpp`](../../src/nav/cpp/endpoint/nav/control/final.cpp) | `FinalControl` 从输入状态取得当拍数据，负责速度整形、限速、SCAN 实测/候选制动及内部停车状态；`main.cpp` 负责构造和接线 |
| 已检查命令的硬件适配和停车确认 | `lingtu-driver` | 处理命令新鲜度、限幅、实际驱动与停止确认 |

规划器说“轨迹被挡住”、跟踪器冻结参考、最终仲裁停止命令、驱动确认实际停车是
不同职责，不因它们都可能产生零速度就删减。最终仲裁中的实测速度制动与下发速度
制动也有不同输入，不能仅按名称判为重复。

普通目标的全局重规划由 `GoalPlanController` 保留任务目标并提交全局搜索；局部轨迹
重规划由 SCAN 负责。重试协调器负责等待和衔接；在途替换规划的状态观察与发起重规划
时的请求接收现已分开：后者交由 `GoalPlanController::replanActive()`，不再重复调用
在途替换检查。输入暂不可用时沿现有运行状态流程等待恢复。

终点转向仍是独立的阶段，Executor 提供目标朝向；跟踪器计算角速度，局部规划器
检查转身扫掠。满足 SCAN 避障配置时，最终仲裁按运行参数检查实测运动和下发速度的制动；
局部转向不再用固定 `0.1 / 0.5 / 1.0` 另做一次制动判断。部分阶段衔接和遥控动作使用
局部规划器的几何查询。自主 SCAN 平移恢复使用正式局部规划和样条跟踪；旋转恢复仍
使用原有转身扫掠检查与角速度生成。

恢复旋转期望速度由跟踪器生成，Executor 负责发起恢复、协调临时目标和报告状态。
CMU 平移保留原有恢复路径和跟踪参数；SCAN 平移使用其正式样条跟踪器。物理验收
结果单独记录。

### 局部规划与受阻恢复

2026-11-03 的自主 SCAN 平移恢复使用下列职责划分。

```text
Recovery    保留原任务，选择一个短距离临时目标，按实测位置判断到达或重试
Executor    将临时 guide、reference 和身份一起交给已有局部规划接口
SCAN Task   用正式 FSM 求解、更新碰撞、重规划并返回样条
Follower    跟踪这条样条，返回候选速度
FinalControl 检查和仲裁最终命令
```

Recovery 从原目标方向附近选择有限的侧向、后向候选位置。候选只是规划输入，只有
SCAN 返回可执行轨迹，执行输出才标记 `recovery_verified`。SCAN 平移分支不再调用
独立的恢复格搜索，也不再另行验证一条几何路径后直接跟踪。机身朝向固定为此次恢复
开始时的实测 yaw，临时参考及其 generation 跨 tick 保留，避免异步求解被每帧重启。

临时目标保留原 `Route` 与任务身份。测得抵达后，下一次局部请求恢复原路线；SCAN
明确无路时换候选，持续没有位置进展时使用既有恢复超时与次数上限。取消、目标替换、
坐标系变化和人工接管沿原执行清理流程撤销临时请求。候选完成不等于用户任务完成。

恢复旋转保留原有三维转身扫掠检查和角速度生成。CMU 保留原路径搜索与几何路径
跟踪，它们与 SCAN 样条接口的差别在局部规划能力边界表达。辅助遥控中已有的边界
脱离分支仍单独使用几何检查，不能统称为 SCAN 求解的恢复轨迹。

实现位于 [`executor.cpp`](../../src/nav/cpp/navigation/executor.cpp)、
[`recovery.cpp`](../../src/nav/cpp/navigation/recovery.cpp) 与
[`recovery_search.cpp`](../../src/nav/cpp/navigation/recovery_search.cpp)。
定向测试覆盖正式异步 SCAN 的临时轨迹复用、实测抵达后恢复原路线、待求解时替换
目标和取消；编译、测试结果和物理动态障碍结果分别记录，软件接线不能代表实机通过。

**2026-11-02：正式 navd 暂停楼梯自动接入。** 路线不再读取楼梯标注并附加楼梯段，
也不再发送楼梯模式切换或等待其恢复。地图和标注保留，试验原型仍供离线研究；普通三维
规划不等于楼梯识别、上下楼运动模型或实际通行已经验证。

最终制动现在优先使用当前路线的机身，没有路线机身时使用配置机身，和局部路线使用的
机身一致；局部另有可选的实测关节几何输入，现场路径当前没有接入该输入。运行参数不是
实测制动能力。此集成不是上游 SCAN 默认行为，参照固定版本
`348e8a590a50a5a6bbab8d8c6dcfd171f009be26`。本地回归以同一动态障碍检验：旧固定制动
否决、配置制动允许；修改后局部保留扫掠判断，最终控制在同一条件允许，并在配置制动
距离不足时限制命令。这是代码级证据，不是物理制动测试。

2026-11-02 的全局边界对照：修改前 `start_evidence_smoke` 中同一张合成保存图的
空闲目标，加入实时占据查询后规划失败；修改后全局请求不再接收该查询，同一静态目标
可规划，静态墙和终点连接障碍测试仍通过。局部输入测试另确认实时占据仍在碰撞图中；
SCAN 的障碍重规划、停车与清图恢复组件测试通过。该对照不是动态物体的物理联调。
在原 `903room_v6_corridor` 保存图上，五个固定目标各查询一次均生成路线；单次查询
约 91–363 ms，另有约 27.4 s 的地图读取和准备。结果保存在
`build/global-static-routing-20261102/results.jsonl`，不是同条件性能基准或实机验收。

## 已确认需要整理的地方

| 分类 | 当前事实 | 影响与下一步 |
| --- | --- | --- |
| 功能缺口：实时地面反证 | 实时占据和自由空间由局部使用。准备阶段的 `SurfaceIndex` 及其推断支撑不随每帧直接增删 | 确定地面被新观测否定时由 nav 如何处理支撑；mapd 继续拥有观测事实。不能把单个自由格等同整块悬崖 |
| 功能缺口：静止后有人靠近 | 现有制动检查覆盖反应和停下过程；普通恢复按 blocked/stalled 触发 | 尚无“当前位置未来能否等待”完整行为；可选主动避让要另行设计、仿真验收并保留最终仲裁 |
| 产品模型边界 | 全局是圆柱通道；局部使用所选 Body。Go2 为 17 部件，Thunder 为 8 部件。早期 Thunder 38 部件地面误拒为历史实验 | 不能称所有层已用同一逐部件模型；先修已复现的接地与低障碍区分，再谈完整步态覆盖 |
| 地形接口生效范围易误读 | 产品关闭 traversability cost，SCAN 解析/请求也不使用它 | 文档应表述为独立能力；不能说打开一个开关就获得低物可跨的 SCAN 行为 |
| 保存来源命名不清 | 状态仍使用 `native_slam_dds_control`，但保存提交由 mapd 协调；SLAM 负责提供快照 | 区分“能产快照”和“当前 Product 可提交保存”；该字段可能为外部接口，改名须先核查消费者和契约，不把名称残留当作已证实双写 |
| 在线与离线建图 | 在线调度与 backend 均在 `online_mapping/`；离线优化使用 `map_optimization/` | 两者共用文件读取，不是两套并行的在线估计器 |
| 目标状态估计 | Gateway 的就绪状态用于展示；普通目标是否接收由 navd 回执决定 | 缓存状态不再挡住普通目标；控制租约和保存位置地图身份仍在 Gateway 边界核对 |
| 动态障碍与全局重规划 | 初次规划、预览和重规划只接收保存地图几何。SCAN 与最终控制处理实时占据 | 长期封堵触发带替代通道偏好的全局规划尚未接通，不能认为已经实现自动跨通道绕行 |
| 控制权表述与入口 | HTTP 客户端租约、WebSocket 连接占用和原生运动租约分别存在；WebSocket 不查询 HTTP 租约 | 明确网页客户端占用与原生控制权各自的作用，再统一对外行为，不能直接删除原生租约 |
| 局部与最终机身上下文 | 最终制动与局部路线统一使用当前路线机身；无路线时使用配置机身 | 可选实测关节几何尚未接入现场控制；不能声称已逐拍使用实时关节姿态 |
| 显示缺口 | Web 主要显示原始占据，实际推断支撑面尚未作为同一权威图层接入 | 后续应显示原生使用的结果，不能由浏览器另算一套地面 |

文档中的旧全局 Body 和 endpoint 姿态描述已按当前工作区修正；地图保存 API 的
说明使用“请求地图保存”，不再把 SLAM 描述成永久地图事务所有者。

用户已确认第一版允许多个页面同时查看，同一时刻只允许一个页面控制机器人。HTTP 与
WebSocket 如何共享该占用、如何与原生控制租约衔接仍需实现和验证，不能将当前行为
描述为已经满足此要求。

## 明确的未完成验证

该分类依据源码和现有记录，不证明新的 Product、仿真或实机通过。下一步按
[地图交付计划](map-delivery-20261031.md) 与
[机体碰撞模型](navigation-body-model-and-motion-posture.md) 中已有证据推进；
读取目标机实际 RunPlan 后才能判断现场选用了哪些模型和功能。
