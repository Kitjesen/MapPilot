# Factory Environment Collection：巡检场景接入

更新：2026-10-23。当前主线是**用户选定的生产线车间**，不是再建一个简化工厂。
本目录负责选区、依赖和坐标的制作端接入；不拥有 Product 生命周期、导航或物理。

## 当前首段接入结果（2026-10-23）

下面是最新状态；后面的旧清单与旧缺口按其日期保留，不代表已经全部解决。

- [x] **真实首段选区**：规范坐标 X=[50,66]、Y=[-6,4]、Z=[-1,6] m；
  27 个源静态组件、122 个原有凸几何。范围只用于选择和观察，不是通行许可或新造围墙。
  相交组件整件保留，不截断地板、梁或机器；保留几何实际范围可以大于该 ROI。
- [x] **曲线与骨骼明确处置**：两项记录是 `SplineComponent` 制作曲线，不是漏导的
  `SplineMeshComponent`。其生成的 Engine 静态组件使用组件世界变换，不能使用 Actor 根变换。
  六项骨骼组件及空碰撞网格不在本次 ROI；没有重建或默认为已支持。
- [x] **实际依赖迁移**：99/99 文件，460,121,362 bytes（约 438.8 MiB），保留真实材质与贴图；
  未复制整包、源地图、BuiltData 或演示玩法。该数值不是 Cook 包大小或运行内存。
- [x] **UE 实际保存及重载**：独立关卡
  `/Game/RobotSim/Worlds/FactoryInspectionBay/FactoryInspectionBay_StaticInitialPose`，
  27 个组件的原始 mesh、材质槽、完整 transform、可见性及 NoCollision/关闭模拟已读回核对。
  `factory-bay-authoring-02` 自然 exit0，136.891 s。源项目未保存；默认入口未改变。
  这次是 NullRHI 制作验证，**不是 RGB 渲染、完整工厂照明或视觉效果验收**。
- [x] **同源物理选区**：`factory-operating-area-physical-02/world.xml` 实际编译为 122 geom，
  mesh 顶点逐项一致；世界位置最大差 2.22e-16 m、旋转矩阵差 0；不添加地面。
  合入 Thunder 后 181 geom、0 plane。Control22/Engine 的测量绑定随原组件保留。
- [x] **真实模型放置及观察检查**：初始 8 个候选通过中心支撑射线／接触检查；不是动力学或
  swept-path 通过。观察草案 [58.8,-3.8,0.615] 与原输送线相撞，明确拒绝，不进入路线。
  改为 Control22 [59,-3,0.615]、yaw0、光学 pan50/tilt25°；Engine [52.3,-3.5,0.615]、
  yaw-90、pan0/tilt40°。两处在选区和原 1065-geom 静态候选内均无放置阻挡。
  64×48 下分别有 463/301 个目标有效热像／类别像素，声学各 160 个有效 Pa 样本。
  与全量静态源逐射线比对，目标像素没有因删掉源遮挡而虚假露出；背景仍有 302/253 条射线差异。
  这些是 **静态挂载检查，不是实际导航到点、云台电机或巡检结果**。
- [x] **真实物理零指令短窗**：公开 `build_engine` + 默认 locomotion policy + 完整 1065-geom
  工厂，4.9 s 积分（含 0.1 s warmup）、961 条保存状态；有限、无 solver warning、无非轮部位承重。
  warmup 后净 XY 漂移 7.43 mm，最大 roll/pitch 0.716/0.511°。0.605 s 有接触峰值，
  四个采样时刻有单轮卸载，不能改写为全程四轮承重；没有调增益或再次动态重跑。
- [x] **实际源碰撞策略读取**：只读 source-profiles commandlet 自然 exit0，223.328 s，
  27/27 源组件覆盖。Custom `SM_AssemblyLine71` 的 Pawn、PhysicsBody、Vehicle、Visibility 等
  已读回为 BLOCK；源项目默认为 `CTF_UseSimpleAndComplex`。九项 Python 暴露通道均保留原始字段。
- [ ] **选区世界资格／出生最终批准**：`SM_SignWetFloor7` 源上确实启用物理模拟；本版明确冻结其静态姿态，
  不是已实现可推动标牌。静态视觉与碰撞变换已对齐，渲染检查、最终出生与运动验收未完成。
- [ ] **本工厂的真实地图与路线**：不能使用旧 `factory_workshop`／FactoryPark，不生成假 map_id。
- [ ] **同一 Product 巡检与异常路径**：原生到点停稳、同代 RGB/热像/声学/位姿、原始证据及
  正常／异常／不可判定、任务终态；之后障碍停车、接管／释放和统一旧传感器控制仍待验收。

关键证据均位于 `build/inspection-tests/20261023/`：

| 目录／文件 | 可复算内容 |
| --- | --- |
| `factory-operating-area-02/selection.json` | 所有组件处置、完整 geom ID、制作 ROI 与 blocker |
| `factory-visual-operating-area-02/visual-manifest.json` | 原组件变换、材质槽、准确依赖与拷贝文件计划 |
| `factory-bay-authoring-02/` | launch、Unreal.log、result、saved-level-readback、success |
| `factory-operating-area-physical-02/` | prepare.py、选区 XML/manifest、测量绑定、placement-evidence |
| `factory-mounted-views-03/` | 真实挂载 NPZ、位姿与光学参数、逐射线全量静态源比对 |
| `factory-zero-command-01/` | 唯一积分窗口的实际配置、961 条状态/接触、结果与录制复核 |
| `factory-source-profiles-01/` | 27 项源碰撞响应与原始物理模拟开关；未保存源图 |
| `factory-operating-area-focused.xml` | 48 passed，零 skip；三份作者工具、显式 World 加载及原合并/初态合同 |

`factory-bay-authoring-01` 因命令行 AssetRegistry 未索引 Engine 材质函数自然失败；文件实际存在，
已补明确资源同步扫描，未用默认材质替代。失败日志不改写。`factory-mounted-views-01` 的原始 NPZ
保留，JSON 因无效样本 NaN 统计失败；后续只统计有效声压。`factory-mounted-views-02` 保留被拒站位。
三份工具 Ruff/Mypy 已通过；独立审阅批准静态 author/reload 范围，不等于发布 World 或批准 Product。
另一次旧 asset snapshot 回归有四个 fixture 在进入 engine 前因缺 `policy_profiles` 失败，
没有越界修改或算入本轮 48 项通过数；既有 pytest `cache_dir` 配置警告仍保留。

建图前查出的 Windows mapd `--disable-query` 已从实现声明移除；现有 AF_UNIX 管理协议重新开放，
Linux 声明未改变。原 runtime graph **125 passed**、新增离线 query wiring **3 passed**；
这是配置／解析验证，未启动 map Product 或验证真实地图保存。

**较早的 PGO 阻断记录（已在本日后续修复）。** 七份本地旧 exe 均早于当前重构源码，未选用。采用现有 SDK、
Cargo offline，从当前源码配置 canonical `build/map_optimization`；Rust 内核及 C++ 静态库完成，
`lt_pgo` 链接失败。当前 GTSAM 4.3 Windows SDK 导出 `std::vector<size_t>` 的四项方法，与正常
consumer 对象重复，LNK2005/LNK1169；/MD、Release、import 宏均已独立核对，不是已发现的
运行库选错。未加 `/FORCE:MULTIPLE`、未改共享 SDK、未用旧 exe。日志在 `map-save-native-01/`。
当时的原始 RED 与目标测试源码保留；当时没有把生产配置指向缺失的 PGO。

后续修复已完成：从现有同版本源码构建独立静态 SDK，未覆盖 SLAM 共享 SDK，未忽略链接错误。
当前 `build/map_optimization/Release/lt_pgo.exe` 已构建，4 个原生优化器测试全部通过。
Windows RunPlan 显式绑定新 exe；PE 依赖检查没有 GTSAM 或其他第三方 SDK DLL，依赖系统／VC 运行库。
配置／传输回归为 129 + 30 passed，原生 SaveMap 回归通过。新增真实 PGO 保存测试用 25 帧
合成闭环点云，经过原生保存和 3D OctoMap 构建、重新打开存储、严格加载，以及失败替换保留原图。
测试 PATH 只有 System32；静态夹具明确关闭独立去动态清理，不冒充 Product 清理链验收。
证据在 `map-save-native-01/real-save-reload-02/`；01 次因测试快照序号错误被拒，记录原样保留，
未放宽生产校验。可复用命令见 `src/localization/map_optimization/README.md`。

当前选区仍缺 Product 可用的合格 World/session；真实工厂地图尚未创建／重载。
不增加第二个生命周期框架，不关闭 patch 保存，也不编造地图身份。根关节绝对 z=0.615
对应 keyframe z=0.6 加 spawn 平移 z=0.015，不能再次叠加 0.615。

较早选区工作的进程后检保存在 `factory-operating-area-postflight.json`：2026-10-23 16:05:08 +08:00，
列明的 UE／构建／原生服务及 Product/MuJoCo Python 入口清单为空。UE 两轮成功命令均自然退出；
失败的 PGO 构建已结束，其明确拥有的孤立 MSBuild 复用节点经身份确认后清理。未启动 Product、
未写地图身份、未修改共享 GTSAM SDK，未关闭其他任务的进程。

## 2026-10-22 同源几何进展（尚未发布世界）

- `bay_export.py` 已从实际源关卡导出 772 个选区 Actor 的组件、变换、实例、骨骼引用和
  依赖闭包；94 个静态网格、3 个骨骼网格，依赖查询 442 个 package。源关卡未保存。
  证据为 `build/unreal-tests/factory-bay-export-20261022-01/`。
- 原 Python AggGeom 接口不能读取 264 个 convex 的顶点，因此没有使用视觉 AABB 代替。
  `bay_collision_mesh_export.py` 使用已安装的 GeometryScript，将原有 simple collision
  复制到临时 DynamicMesh，逐连通片导出，元素变换已烘入资产局部厘米坐标。
- 首次真实补充导出 `factory-bay-collision-export-20261022-01` 为 **incomplete**：
  16 个网格因未引用顶点检查被拒；98.235 秒自然失败退出。旧失败报告保留。
  本机 UE 源码证明复制路径允许保留未引用顶点，不能据此判定边界缺失；修复后保留
  全部原始顶点、三角形及这些点的索引，不放宽后续闭合／凸性／体积检查。
- 独立新目录 `factory-bay-collision-export-20261022-02` 实际导出 **94/94 个资产、325 片、
  6278 个三角形**，19.141 秒自然 exit 0。没有构建、保存源资产或启动 Product。
- `build/inspection-tests/20261022/factory-collision-candidate-02/verification.json`：
  325 个源片经实例变换形成 **1065 个静态 MuJoCo geom**，全部通过当前候选闭合凸体检查，
  MuJoCo 实际编译通过，`nq=nv=0`。466 个未引用点均在相应资产的边界凸包并集内
  （容差 `1e-4 cm`）；未宣称恢复了原 AggGeom 元素索引。

以上仍是 **candidate / qualified_world=false**。尚有 116 个跨选区组件、181 个越界
碰撞体、2 个 spline、6 个骨骼组件记录（包含选区外记录）及项目默认 collision trace
策略待处理；大型 Cube_2 等不能静默截断。尚未迁移视觉依赖、发布 WorldPackage、冻结
导航地图／出生点／设备测量属性，更没有本工厂的实际机器人行驶或三点巡检结果。
后面的 2026-10-21 记录是历史证据，不应将旧 StaticMesh-only 缺口误读为本次未做清点。

## 测量属性作者链：版本候选与真实采样（2026-10-23）

`measurement_authoring.py` 从上述真实 Fab 碰撞候选读取稳定源组件路径，不借用
`factory_workshop` 或 `FactoryPark`。目前明确选择：

- `SM_AssemblyLineControl22_81.StaticMeshComponent0`：生产线控制台，不改称电柜。
- `BP_SplineMovableV2_0.NODE_AddStaticMeshComponent-1`：已导出的 Engine 组件。

二者保留原有 6 个物理 geom，整份候选仍保留全部 1065 个 geom。缺失设备／空几何、
缺失或重复 geom、XML 与候选名称不一致均失败，不补空 `measurements: {}`，不按视觉
AABB 新造设备。未绑定的表面仍未知，不变成零温度、正常设备或自动标签。

作者明确假设如下，**均非现场测绘、素材自带属性、设备铭牌或已验证故障阈值**：

| 源设备 | 表面温度 | 发射率 | class / instance | 表面校正模式异常阈值 | 有限声源 |
| --- | --- | --- | --- | --- | --- |
| Control22 | 330 K | 0.85 | 101 / 2101 | 345 K | 440 Hz，0.02 Pa 峰值包络，参考距离 0.1 m |
| Engine | 365 K | 0.9 | 102 / 2102 | 360 K | 180 Hz，0.1 Pa 峰值包络，参考距离 0.1 m |

声源均为 16000 Hz、3200 samples、0.2 s Hann 窗有限波形，从仿真时间 0 开始，不循环。
异常阈值仅保存在作者假设中，不冒充传感器诊断结果；默认热相机输出**表观亮温**，
不可直接拿它与表面校正阈值比较。类别含义、波形、测量原点、源 ID、未知项及完整候选
provenance 均保存在 `authoring-report.json`。1 ms / implicitfast / Newton / 100 iterations
和 -9.81 m/s² 重力同样为本候选明确声明的仿真策略，不改变源几何。

可复现的离线命令（输出目录必须不存在；不启动 UE、Blender、Product、DDS 或运动）：

```powershell
.venv/Scripts/python.exe -m sim.tools.worlds.factory_environment_collection.measurement_authoring `
  --candidate build/inspection-tests/20261022/factory-collision-candidate-02 `
  --output build/inspection-tests/20261023/factory-measurement-authoring-03 --sample
```

输出只在指定的新目录内：

- `packages/factory_environment_collection_measurements/0.1.0/`：已有源几何及非空
  measurement 声明；**未发布的实验 World 候选**，不写入 `sim/packages/`。
  此处为 ImportDraft 根目录，`draft.json` 的 state 为 `candidate`，qualification_path
  为 null；实际包在 `package/`，其 `source/` 保留原 MJCF、stable-ID 映射和作者假设。
- `session.yaml` 与 `bundle/`：显式额外 catalog root 经现有 CatalogResolver 编译，
  thermal/semantic/acoustic 的 `scene_measurements` 直接来自这个新版本候选。
  复用 `thunderv4_headless_multimodal@1.0.0` 的传感器配置，不改旧 World 或旧 Session。
  Session 中零坐标 spawn 仅用于编译，**不是已确认的出生点或可运行巡检配置**。
- `measurements.npz` 与 `sampling-evidence.json`：三个 64×48 最近命中热／语义视图、
  16 kHz / 160 samples Pa 块、有效掩码、stamp、标定配置、源位置、遮挡路径及单位。
  样本使用**编译后的原始配置与未过滤的双声源场景**；无 geom、无关节测试接收器仅
  用于观察，不代表实际机器人安装标定。正常视图、隔墙未知/无效、50 ms 双源块以及
  1 s 有限片段结束后的零值分别保留。9216 条独立标量最近射线核对热有效掩码。

**公开 WorldImporter 候选入口已贯通，视觉／导航资格化仍未完成。** 作者模块现在调用
`WorldImporter.import_mjcf_candidate(request, draft_root=...)`，不再调用私有编译函数。
该入口复用 SourceIntake、provenance、锁、staging、冲突处理与 ImportDraft；不会走旧
heightmap 的 qualified 成功路径，不生成高度图、spawn、视觉投影或资格化记录。
现有 CatalogPromoter 拒绝 state=candidate，入口也拒绝向活动 `sim/packages/` 写草稿。

请求没有新 schema 标记，使用显式字段 `package/source/units/visual/mjcf/source_bindings/
measurements`；前四项复用原有结构，两个相对路径位于 source.path 内。
`source_bindings` 是包含 source_id / component_source_id / mjcf_geom 的非空 JSON 列表，
必须一对一覆盖全部实际 geom，不允许缺失、重复或不存在的映射。首版明确只接静态内联
MJCF：拒绝 flexcomp/composite/replicate/attach、joint/freejoint、actuator、mocap、
外部文件/include 和 fusestatic；声源必须绑定
有源几何的 body。测量属性复用既有 validate_scene 严格数值与名字校验，三类均须非空。

为了不伪造高度图投影，也不改 schema，候选使用既有 World v1 的非投影 envelope；
完整来源记录由 ImportDraft.provenance_path 指向包内 `provenance/world.provenance.json`，
包含原 source intake、许可证来源声明、stable-ID 文件路径、candidate 状态及 pending 项。
`source-notice.md` 原样复制仓库已有的 Fab 领取来源记录，**不是新造许可证正文或再分发授权**。
作者报告的 importer_status 为 `candidate_imported_unqualified`。

仍需完成 Fab 视觉迁移／对齐、导航出生点、跨区、spline、骨骼和碰撞策略复核后才能
谈世界资格化。当前 visual level 仅记录源 `/Game/Maps/Demonstration`，不表示 RobotSimUE
已可加载。原 `import_world` heightmap 请求／默认产物、旧 World/Session 与所有 schema 未改。

专属回归：`tests/sim/test_factory_measurement_authoring.py`。许可素材本地导出不在 CI
时会明确 skip；skip 不算真实 Fab 证据。代码没有改 `formal_feeder`、`formal_visual`、
Gateway 证据或 perception，也没有产出路线通过、Product 同跑或现场巡检成功声明。

公开入口补齐前的 `factory-measurement-authoring-02` 产物已真实生成并用 `allow_pickle=False` 回读
28 个原始数组：Control22 / Engine / wall 热及语义有效像素为 **492 / 2025 / 0**。
50 ms 未过滤双源块 160/160 samples 有效，峰值 **0.001989828888 Pa**；独立逐源延时、
插值和距离衰减回算一致，两个源都有非零贡献。Engine 观察点的双源声学块因另一个源
受挡而全无效，原样保留；1 s 片段结束样本有效且为零。全部 1065 geom 的稳定源 ID、
名字与本次编译索引映射均保留；零关节、零物理步。测试支架分别查询 0 / 50 ms / 1 s
时间戳，最后 fixture 时间为 1 s，不把它记成经过动力学运行。

专属最终复验 **9 passed，未跳过**；与设备绑定、碰撞候选及 measurement catalog 的组合
回归 **160 passed / 30 skipped**。30 个既有 schema 测试缺少本机 `jsonschema` 而跳过，
没有安装依赖，不宣称通过完整 JSON Schema 检验；CatalogResolver 的实际校验及编译通过。
Ruff 通过。Pytest 仍有禁用 cacheprovider 后的既有 `cache_dir` 配置警告。`-01`、`-02`
产物保留，不覆盖旧证据；`-03` 使用公开候选 intake 重新生成并采样。

公开入口最终验收：新增 `test_world_mjcf_candidate_intake.py` **17 项通过**，与作者链的
9 项、旧 world importer/promotion、设备绑定、碰撞候选及 measurement catalog 一起为
**213 passed / 30 skipped**。旧 heightmap 默认产物／晋级回归通过，schema 文件未改。
`-03` 只读回验确认 4 个作者源文件经 intake 后字节一致、1065 条 stable-ID 映射保留，
新 draft 为 `candidate`、qualification_path=null，28 个原始测量数组回读通过；有效像素和
16 kHz 声压结果与上述数值一致。剩余 pending 明确为 visual_alignment、navigation_spawn、
world_qualification，不将公开 intake 完成等同于真实工厂可运行／巡检通过。

### Focused GREEN 收口与最终重生成（2026-10-23，-04）

本轮未新增实现分支，先独立重跑 focused 命令：

```powershell
.venv/Scripts/python.exe -m pytest tests/sim/test_world_mjcf_candidate_intake.py tests/sim/test_factory_measurement_authoring.py -q -p no:cacheprovider
```

结果 **26 passed，0 skipped**，Ruff 和已跟踪 importer 的 whitespace 检查通过。
随后经同一公开入口新生成 `build/inspection-tests/20261023/factory-measurement-authoring-04/`，
未覆盖 -01/-02/-03。对 -04 的独立只读回验确认：

- 仅 **1 个 candidate root / 1 份 World manifest**：
  `packages/factory_environment_collection_measurements/0.1.0/`；state=candidate、
  qualification_path=null，未做 production promotion。
- 4 个输入源文件与导入后副本字节一致；1065 条 stable-ID 映射与原候选逐项一致。
  World measurements、SensorPlan scene_measurements 和作者声明一致。
- 28 个数组以 allow_pickle=False 回读；热／语义有效像素仍为 492 / 2025 / 0，
  双源 16 kHz 块 160/160 有效、峰值 0.001989828888 Pa，9216 条独立射线检查通过。
  Engine 全双源及隔墙无效样本保留，片段结束样本有效且归零；零关节、零物理步。

本作者链／公开 candidate intake 已 focused GREEN 收口。以下是**仍阻止世界资格化的
工作项**，不是本轮已验证通过的项目：

1. 视觉制作端：Fab 选区依赖／材质迁移、同源投影与 RGB／物理对齐未完成。
   source level 引用不等于 RobotSimUE 可加载。
2. 几何制作端：原候选保留 116 个跨区源组件和 181 个越界碰撞 geom；462 个组件带
   project_default_trace_requires_review。还须处理 collision profiles/response channels、
   2 个 spline、6 个 skeletal 组件、2 个 empty AggGeom；这些计数有交叠，不能相加当成
   独立设备总数。815 个其他 SceneComponent 未重建、62 项位于选区外，需按选区用途
   明确处置，不把它们全部误判为缺失碰撞。
3. 世界／导航验收：尚无本 Fab 批准的出生点、导航地图和可执行路线；Session 零坐标
   是编译 fixture，静态接收器是采样 fixture，均不是机器人可达站位或运动／接触证明。
4. 传感器／巡检验收：K、发射率、Pa、类别及异常阈值仍为作者假设；没有机器人实际安装
   标定、真实站位的全模态有效采集、Product 同跑或三点原生巡检证据。Engine 视点的全
   双源声学块仍因另一源遮挡而无效，必须在后续站位设计／验收中处理，不能删源掩盖。

机器 pending 仍明确为 visual_alignment / navigation_spawn / world_qualification。
本轮未启动 UE、Blender、Product、DDS 或 native 运动，也未安装依赖。

### 2026-10-24 选区渲染与站立检查推进

本轮建立了新的 enclosed candidate，而不是覆盖源 `FactoryPreview` 地图：

- `build/inspection-tests/20261023/factory-enclosed-bay-01/prepare-result.json`：59 个源组件、184 个源物理几何，99 个字节完全一致的既有依赖复用；候选地图为
  `/Game/RobotSim/Worlds/FactoryInspectionBay/FactoryInspectionBay_EnclosureCandidate`。
- `build/inspection-tests/20261023/factory-enclosed-bay-01/physics-result.json`：实际 Thunder MuJoCo 组合为 184 个选区几何＋59 个机器人几何；1 ms `implicitfast`／Newton／100 iterations；10.1 s 零指令站立，最大 XY 漂移 0.010958724 m，最大绝对 roll/pitch 0.7544/0.5245 deg；这是碰撞／出生候选检查，不是导航或 Product 验收。
- `build/inspection-tests/20261023/factory-enclosed-render-03/`：UE 自然退出、三张截图完整、source map 未修改。此版本改用手动物理相机曝光（EV100=9、ISO=100、f/8、5600 K），四盏巡检顶灯各 6000 lm；相较 `-02` 已消除地面大面积过曝，设备和输送线可辨认。

渲染结果仍不是正式视觉资格化：选区边界外仍能看到黑色外部背景，说明当前选区没有完整迁入厂房外墙／环境背景；这不改变已选碰撞几何，但在正式 World 绑定前必须补齐或明确批准该视觉边界。`capture-result.json` 的 `product_run=false`、`spawn_approved=false`、`qualified_world=false` 保持原样，不能因截图成功而改写。

下一步收口顺序固定为：

1. 用同一候选证据补齐视觉边界与正式出生点验收；不改源地图、不把视觉 AABB 当碰撞代理。
2. 通过现有 WorldImporter 的正式资格化路径生成本工厂独有的 World/session；当前 MJCF candidate 仍明确拒绝 promotion，不能手写 `qualified_world=true`。
3. 只在正式 World/session 产生后启动同一个 Product：真实建图、保存并重载本次 `map_id`／`map_content_epoch`，再接一个设备点的原生导航与 RGB／热像／声学巡检采集。

### 2026-10-24 扩大为可巡检路线

上一版 16×10 m 的局部设备选区不足以形成巡检场景。基于已有源工厂导出，离线扩大到
X=[30,70]、Y=[-20,16]、Z=[-1,15] m，范围约 40×36 m；`bay_layout` 实际得到 **459 个源组件、1064 个原始几何**。结果保存在
`build/inspection-tests/20261024/factory-route-layout-01/selection.json`。

该扩大结果同时发现 13 个 `blocked_missing_source_collision` 组件，包含骨骼设备、线缆和
没有源碰撞的移动／设备组件。它们不能直接进入可通行物理；后续必须把“视觉可见”和“可碰撞／可导航”分开复核。路线草案保存在
`build/inspection-tests/20261024/factory-route-layout-01/route-plan.json`，包含起点、Control22、Engine、第二控制台、工业机器人单元和终点六个站位。

Control22 `[59,-3,0.615]` 与 Engine `[52.3,-3.5,0.615]` 继承既有静态支撑／清障检查；其余站位仍需支撑、清障、视觉对齐和传感器视场验证。该文件是路线输入草案，不是 map_id、导航路线或 World 资格证明。

`split_route_physics.py` 已按该契约生成
`build/inspection-tests/20261024/factory-route-layout-01/physics-candidate-02/`：保留 459 个完整源组件和 1064 个已有碰撞几何，单独列出 13 个视觉阻塞组件；没有新增碰撞、没有裁剪几何、没有批准出生点，也没有发布 World。

`prepare_route.py` 随后生成了 UE 路线视觉候选
`build/inspection-tests/20261024/factory-route-layout-01/visual-candidate-01/`，目标关卡为
`/Game/RobotSim/Worlds/FactoryInspectionBay/FactoryInspectionBay_RouteCandidate`。本次新增复制 252 个资源、1,336,472,668 bytes，143 个完全相同资源复用；这是未 Cook 的制作资源统计，不是最终安装包大小。候选仍未包含 Thunder runtime presentation、Runtime UI 或 Product session。

路线关卡随后经独立 UE reload 验证：
`build/inspection-tests/20261024/factory-route-layout-01/verify-result.json` 报告 saved level 存在、459 个视觉组件、37 个源灯光、所有视觉组件 NoCollision，验证 runner 自然 exit0。作者 runner 的首次 900 秒 timeout 原样保留在 `run-result.json`；它没有覆盖保存成功事实，不能改写成 timely success。

### Parfit 独立 review 修复（2026-10-23，-05）

原 focused 26 项没有覆盖以下四个缺口；本轮分别复现 RED 后修正：

- 编译不变量：真实 flexcomp 在 ngeom 不变时产生 nq=nv=24；replicate count=1
  在数量不变时把 hot/machine 改为 hot0/machine0。现在拒绝不支持的生成 tag，并对
  实际 MjModel 再检查 nq=nv=0、全部 geom/body 名称（含数量）与源一致，重新校验
  measurement 名字及声源 body 的实际几何绑定。候选 provenance 保存实际名字与
  thermal/semantic geom ID、acoustic body ID；旧 heightmap 编译报告默认字段不变。
- 截断 XML：解析点把 ET.ParseError 转为 ImportFailure(MODEL_INVALID)，回归确认
  staging 清理、quarantine 诊断落地及原输入不变。
- source 根 reparse：_source_path 不再先 resolve；以 lexical absolute 路径交给既有
  SourceIntake，先检查所有路径组件再解析。真实 Windows junction 的绝对／相对入口
  回归均通过，源数据未改变；旧 heightmap/import-promotion 回归通过。
- 类型检查：异构字典显式标注 dict[str, Any]；worldbody 只取一次并先做 None 收窄，
  没有新增 type-ignore。mypy 复现的 7 条错误清零。

最终组合 **70 passed / 0 skipped**：candidate intake **25**、作者链 **9**、旧
world importer/promotion **36**；其中还包含绕过 XML 前置校验、以真实 MuJoCo 编译结果
验证最终不变量的两项回归。Ruff 和 whitespace 检查通过。

新生成 `build/inspection-tests/20261023/factory-measurement-authoring-05/`，仍只有一个
candidate root，state=candidate、qualification_path=null。实际编译 nq=nv=0、1065 个
geom 名称与两个设备 body 一致，6 个热／语义 geom ID 和两个声源 body ID 校验通过。
28 个数组回读、492/2025/0 有效像素、双源 160/160 有效及 0.001989828888 Pa 峰值保持。
未覆盖旧产物，未晋升；视觉对齐、导航出生点与世界资格化仍按上述 pending 项保留。

严格 schema 独立补验：使用现有 `C:/Users/99563/miniconda3/python.exe` 的 jsonschema
4.26.0 / Draft202012Validator，并以 referencing.Registry 解析仓库已有外部 schema 引用，
对 -05 的实际 WorldPackage、MeasurementScene、SensorPlan **3 份产物全部通过**。
未知表面属性、负温度两项负对照均被拒绝，不是 importer 的 fallback 子集校验。
结果保存在 -05/strict-schema-validation.json；没有安装依赖，也没有运行 Product。
这次是已有 Miniconda 对实际产物的独立检查，不将此前 venv 中跳过的测试改写为已执行。

Parfit 修后独立复审结论：**APPROVE，原四项问题均关闭，无新增阻断**。
其单独执行 8 项定向回归（0 skipped）、两文件 Mypy/Ruff，并只读回验 -05 的源文件字节、
1065 条映射、编译不变量与原始数组；未重复组合套件、重生成、采样或运行 Product。
该结论仅覆盖候选 intake／测量作者链，不关闭视觉、出生点、世界及实际巡检资格化 pending。

## 来源与用途

### 设备绑定与测量切片（2026-10-22）

`bay_device_bindings.py` 通过显式设备 ID／源组件 ID 绑定原有 geom，只重分组并调整局部
坐标，不增加碰撞几何或改变世界位置；35 个专属测试通过。初版高层 Switchboard 并非
地面可达目标，后续切片选择地面的 `SM_AssemblyLineControl22_81` 和已选 Engine，
未移动高柜，也未将控制台改称电柜。

`build/inspection-tests/20261022/factory-device-bindings-01/physical-extents-readonly.json`
独立复算了 1065 个 geom。Cube_2 的巨大 Z 范围来自源组件缩放及源视觉 bounds，
不是转换错误；它是高大薄实体而非水平地板。未静默删除或截断。

`build/inspection-tests/20261022/factory-device-measurements-01/` 保存真实静态测量切片：

- 保留全部 1065 个原 geom，仅重分组其中 6 个；新增三个无 geom rig，零关节、零运动。
- 64×48 控制台、Engine、隔墙三个视图，共 9216 条独立最近命中射线比较通过。
  热／语义有效像素分别为 1101、3036、0；未赋热属性的墙不是零温度。
- 表面温度人为声明为 330／365 K，发射率 0.85／0.9；现有灰体模型输出
  325.2289／359.5510 K 表观亮温。这些不是素材作者属性或真实设备实测。
- 五个双通道×512 samples、8 kHz Pa 块保留有效、混合源受挡及隔墙无效情况。
- `measurements.npz` 内含原始数据、掩码、单位、假设标定、stamp 和 geom/source 映射；
  `run_slice.py --verify` 只读回验通过。没有 RGB、UE、DDS 或巡检成功输出。

结果仍为 **DRAFT / SYNTHETIC / qualified_world=false**。静态 rig 位姿不等于机器人
实际安装标定；同源 WorldPackage、真实站位／导航、RGB 对齐和 Product 同跑仍待验收。

### 资产来源

- 原工程：`D:/epic/Projects/FactoryPreview/FactoryProject.uproject`。
- 原地图：`/Game/Maps/Demonstration`；已领取来源和使用边界见
  [`sim/library/SOURCES.md`](../../../library/SOURCES.md)。受限原始素材不上传公开仓库。
- 首页已经使用这张地图的真实渲染图，但首页 PNG **不是可运行场景**。
- `factory_park_hf` 是已有的旧园区；`factory_workshop@2.0.0` 是另一份原生校园。
  二者都不等于本次 Fab 生产线，不复用它们的地图身份或巡检坐标。

## 当前完成状态

- [x] 来源领取、D 盘缓存、独立工程和实际 UE 画面已确认。
- [x] 产品默认首页已使用工厂背景；旧黑色模型展台不是默认入口。
- [x] 运行入口跳过六项制作插件；正常 UE 编辑入口仍保留制作能力。
- [x] 同次真实 UE 启动的首页、机器人、分类、三维查看、地图选择截图通过。
- [x] 选区工具离线测试：只保留相交 Actor，标出跨边界几何，不修改输入。
- [x] 真实源地图 StaticMesh 选区清单：458 个 Actor、100 个不同网格、117 个跨边界项；含原始引用、Actor 变换与视觉范围。
- [ ] 机械臂等 SkeletalMesh、灯光、特效、Spline／实例数据和依赖的补充清点；StaticMesh 清单不是完整关卡导出。
- [ ] 按选区迁移网格／材质／贴图依赖；排除整图烘焙光照和旧演示玩法。
- [ ] 建立同源碰撞及导航地图；校准坐标、比例、出生点和传感器视点。
- [ ] 真实 Thunder 短距离驾驶、接触、松键停止和重置验收。
- [ ] 三点原生巡检、人工接管、证据审阅和接受闭环。
- [x] 三点采集底层回归：真实 C++ Executor 连续三次重跑、旧证据拒绝、第二点失败不访问第三点；输入为合成导航／传感器回执，不是本工厂实跑。
- [ ] 独立 Windows 包启动、加载与运行性能验收。

巡检／传感器最新检查：**178 项巡检 Python、121 项传感器合同、4/4 原生 C++ 测试目标通过**。
传感器接入矩阵、两个相机路径与噪声开关边界见
[`传感器 README`](../../../runtime/sensors/README.md)。GPS／热成像／声学未接入本运行链，
不可把菜单声音或真值里程计当作相应传感器。

## 已知加载问题与本次改动

上一轮 Editor 初始化 191.51 秒，启动时可用物理内存约 3.4 GiB；DDC 测试随机读
2.48 MB/s，日志出现慢读。展示网格的编译估计需求 1345.01 MiB，高于当时
1132.69 MiB 预算。最大日志空档约 52.58 秒，位于 DirectoryWatcher 与 MeshUtilities
加载记录之间；**日志空档不能证明该模块独占了这些时间**。

本轮在选择器和 Editor-backed runtime 命令中共享一份制作插件排除参数：
`ModelContextProtocol,AllToolsets,Terminal,Tripo3DUEBridge,PythonScriptPlugin,EditorScriptingUtilities`。
不修改 `.uproject`，不关闭 LCC 场景渲染，不影响 packaged executable 或正常编辑器制作。

实测结果：初始化 **109.80 秒**；全程 **136.641 秒**，包括五张 UI 截图、三维查看
和自然退出。窗口为 1920×1080、帧率上限 30，保留声音。没有启动物理仿真、
没有确认会话或写 selection intent。该结果仍不达成快速启动要求。
前后缓存与可用内存不同，不将全部差值归因于两项新增插件排除，更不承诺固定提速比例。

证据：

- `build/unreal-tests/product-progress-20261021/runtime-plugins-flow/result.json`
- 同目录 `launch.json` 与 `Unreal.log`
- `RobotSimUE/Saved/Automation/LingTuSimUI/FrontEnd/runtime-plugins-flow-20261021/`
  下 `login.png`、`asset-library.png`、`category-humanoid.png`、
  `robot-inspection.png`、`map-selection.png` 和 `success.json`。
- 启动命令相关回归 **118 passed**；选区工具及实际发现的 Paths API 错误
  分别经历 RED→GREEN，当前 **2 passed**。
- 最终组合回归 **120 passed**，Ruff 与所触及跟踪文件的 diff whitespace 检查通过。
  Pytest 仍有禁用 cacheprovider 时的既有 `cache_dir` 配置警告。

后续先查缓存／磁盘与内存竞争，再统计同步资源加载点。真正减少编辑器模块和
运行时派生资源构建的交付路径是 Cook 后的游戏包，不是永远让用户启动 Editor。
制作端预热与运行包首次加载分别计时，不清空缓存或降低画质来伪造比较。
初步体验目标为目标机器上热启动可操作首页不超过 10 秒、冷启动不超过 30 秒；
这是待验证的产品目标，不是当前实测，也不直接改动安全或启动超时来假装达标。

首次源选区命令保留于 `build/unreal-tests/product-progress-20261021/factory-bay-intake`：
98.593 秒后自然失败退出，脚本把 UE Paths 的 `get_project_file_path()` 写成了
不存在的 `project_file_path()`。依据本机 `BlueprintPathsLibrary.h` 修正并补充回归，
采用独立输出重验；没有读取／保存源地图，也没有产出可用选区清单。

修正后真实命令输出于
`build/unreal-tests/product-progress-20261021/factory-bay-intake-api-corrected`：
`result.json` 为 passed=true、exit0、natural_exit=true、183.141 秒；
`bay-intake.json` 从源地图 8,639 个带静态网格的 Actor 中筛出 458 个，
对应 100 个不同网格，117 个 Actor 跨出制作边界。源地图未保存、未迁移到 RobotSimUE。
100 个网格中 99 个是项目资源、1 个是引擎 Cube；99 个 `.uasset` 合计
124,806,204 字节（约 119.0 MiB），**未计材质、贴图、骨骼、BulkData 或 Cook 依赖**，
不能作为最终包大小。

这次清单成功不代表源地图无错误：`Unreal.log` 仍有旧 `PW_ForkLift`／PhysX 依赖
及组件加载告警。该叉车玩法不纳入首个可玩区域。工具只登记带 StaticMeshComponent 的
Actor，不完整重建 SkeletalMesh、Spline、实例、灯光或特效；这些仍须补充清点。
下一步从跨边界清单选择保留／裁剪／替代项，再计算真正需要迁移的依赖，不能直接
把 458 个 Actor 变成已资格化世界。
本轮结束时精确进程名扫描为空（UE／ShaderCompileWorker／UBT／UAT／dotnet／
cl／link／MSBuild／MuJoCo）。没有继续留下编辑器或仿真进程。

## 首条巡检路线

先完成一个生产单元及其相邻走廊，不装入整座 11,872 Actor 工厂。
本轮提出的选区是已审阅镜头周边 `X=[3000,7000]、Y=[-2000,1600]、Z=[-100,1500] cm`，
约 40×36×16 m；这是**待复核的制作范围**，不是可通行区或碰撞边界。
跨区的大地板、墙体、屋顶不能因为与选区相交，就整件当作局部关卡迁入。

| 节点 | 目的 | 第一版可验证内容 | 仍需接入 |
| --- | --- | --- | --- |
| 起点／待机区 | 就绪检查、明确控制权 | 状态和启动条件 | 实测地面、出生位姿、零命令稳定 |
| P1 机械臂单元 | 从安全距离观察设备 | 停稳后的 RGB 概览与实际位姿 | 原生目标、机台／围栏代理；暂不启动机械臂运动 |
| P2 电控柜 | 验证面板可见性与到点采集 | RGB 证据、时间和点位绑定 | 摄像机视角与光照；不声称已有热成像或故障诊断 |
| P3 通道安全 | 检查通道、消防／维护位置 | RGB 概览与人工复核 | 真实障碍几何、导航通行检查 |
| 终点／结果 | 停车、任务完成与证据接受分开 | 原生任务终态、缺失证据提示 | 同一任务报告和实际停车证明 |

任务循环：准备→原生导航→到点停稳→`capture:overview`→核验证据→下一点→结果审阅。
首轮先做正常路线；受阻后人工接管／释放／显式继续是下一条验收，不以 UE overlap
或动画触发点位完成。点位数值、map_id、map_content_epoch、route_revision 尚未冻结，
不创建一个指向旧地图的“可运行”配置。

## 权威和迁移约束

- ProductControl 启停固定 env 的 RunPlan，UE 不起另一套进程控制器。
- 原生 inspection executor 推进点位；Gateway 负责证据绑定和结果投影。
- MuJoCo 管动力学、接触和激光；UE 管视觉、相机、界面和输入意图。
- WorldPackage 同源实体保留稳定 ID 和规范变换。视觉几何与单独的解析／合格凸代理
  对齐，UE 对共享物理实体用 NoCollision，不能拿视觉 AABB 当合格碰撞。
- 拷贝原 PhysX 叉车蓝图不等于 UE5.8 或 MuJoCo 已有可驾驶叉车。
- 先统计选区依赖和 Cook 大小，再迁移；不复制整个 Demonstration_BuiltData。

## 选区工具

`bay_intake.py` 必须在独立 FactoryPreview Python commandlet 中执行，输出目录由
`LINGTU_FACTORY_BAY_INTAKE_DIR` 指定。它只读取源世界，输出 `bay-intake.json`，不保存地图、
不导入 RobotSimUE、不生成碰撞、不发布包、不发送运动指令。
视觉 AABB 超出选区会标为 `crosses_bay_boundary=true`，需要人工决定裁剪或替代。

运行后的结果与下一步持续记在本 README 和
[`产品推进计划`](../../../../docs/design/product-progress.md)，不能仅靠脚本存在标记完成。
