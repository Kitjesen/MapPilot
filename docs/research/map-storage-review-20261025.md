# 地图数据结构与持久化：原始实现核查

日期：2026-10-25。核查对象：当前LingTu工作树。本轮完成实现核查、私有Cell字段
重排和主场景高度配色；没有更换地图格式或部署机器人。WSL GCC对真实头文件进行了
改前/改后布局编译，并运行原有rolling occupancy测试；这不是板端性能测试。

## 结论

现有主线值得保留：本地滚动三维概率占据处理实时观测，完整OctoMap保存全局导航
地图，关键帧点云/位姿保留重建来源，浏览器使用受点数预算约束的展示数据。它们的
访问方式和信息量不同，不能为了“统一结构”全部改成一个位图、一个PCD或一个数据库。

需要深入改进的是三个已经存在的问题：长时间采集的重建来源会被内存窗口淘汰；
正式source commit会将map.pcd写成只有XYZ的格式；保存地图展示每次仍全量解析PCD。
实时Cell还有一个可明确量化的布局改进空间。优先处理这些，不引入新DB、ESDF或
另一套地图持久化权威。

## 从哪些原始入口查起

| 原始入口 | 真实职责 |
|---|---|
| `src/message/idl/messages.idl:520`，MapObservation | 包含scan、map_sensor、sensor_origin、epoch/sequence和位姿质量；不是现成的射线数组 |
| `src/maps/cpp/mapd/engine.cpp:889`，LiveMapEngine::Process | 点云变换到map，用真实传感器原点更新实时occupancy，另维护展示/建模层 |
| `src/maps/cpp/layers/rolling_occupancy.cpp:1025` | 端点和原点形成射线，记录hit/miss并更新概率 |
| `src/maps/cpp/mapd/save.cpp:132` | 正式SaveMap向SLAM请求冻结的map.pcd和patch bundle |
| `src/localization/slam/cpp/ltlio.cpp:739` | 冻结数据的实际落盘：地图、关键帧、位姿、轨迹、外参与优化证据 |
| `src/maps/cpp/save.cpp:2025`、`:2053` | source commit后生成导航包，再按既有事务发布 |
| `src/maps/cpp/build/pipeline.cpp:1066` | 读取map.pcd，优先重放saved rays构造octomap.ot |
| `src/maps/cpp/service.cpp:645` | 保存地图的点云展示输出 |

## 实时数据结构

### 1. RollingOccupancyGrid：有界稠密概率网格

源：`src/maps/include/lingtu/maps/layers/rolling_occupancy.hpp:159,233`。

- 主数据是`std::vector<Cell>`。Cell保存`double log_odds`、`uint16_t hits/misses`、
  `int64_t last_observed_ns`、`bool observed`、`bool unresolved_hit`。
- 两个`uint32_t`数组保存射线投票计数；五个`uint64_t`位集加速observed、raw occupied、
  known free、collision seeds、inflated查询；`uint16_t inflation_counts_`处理膨胀重叠。
- ring坐标让局部窗口滚动，不需要每次把整块数组搬移。移出窗口的历史会丢出实时窗口；
  该窗口不是永久全局地图。
- 未观测与自由不同。`StateFor()`在未观测时返回unknown；raw occupied/free都没有
  置位时，Body查询不会把它当成已验证自由。当前未解决的hit也可以保持这种未确认状态。
- 默认200×200×100、0.05m来自类默认值，实际Product/环境可以覆盖；不能把默认值
  当作每台设备都已读取验证的配置。

### 2. Snapshot是导出，不能反推内部只有1bit

源：同头文件`:65,84`；`rolling_occupancy.cpp:1222,1252`。

| 结构 | 实际内容 | 用途/信息边界 |
|---|---|---|
| RollingOccupancySnapshot | 每体素uint8 state和int16 log_odds_q8 | 通用概率/状态导出；Q8在导出时量化，不是内部Cell精度 |
| RollingInflatedSnapshot | 三份vector<uint8_t> packed位图：inflated/raw occupied/known free | 每层1bit/体素，共3bit/体素，另有几何/计数元数据；没有内部连续log-odds |
| nav LocalCollisionMapView | 字节指针和共享不可变storage、尺寸/分辨率/坐标变换/时间 | 消费已导出证据，不重新建立概率地图 |

`engine.cpp:1103`只在collision generation改变时重新导出位图；这一复用已经存在。
`codec.cpp:426`仍需从DDS消息复制到接收端共享storage，因此不是全链路零复制。

### 3. VoxelLayerCore：稀疏表面表示

源：`src/maps/include/lingtu/maps/layers/voxel.hpp`。

- `unordered_map<VoxelKey,float>`记录体素位置和累计量；有范围、衰减与max_voxels预算。
- `ClearObservedFree()`用真实occupancy自由证据清表面；展示的最近点预算与建模用
  `SnapshotXyz()`分开，不能把少显示的点解释成地图删掉。
- 这是表面/展示和建模层，不包含完整自由空间。它不能替代rolling occupancy或
  OctoMap给运动放行，也不能按哈希表条目大小直接宣称精确RSS。

### 4. PersistentBlockGrid：分块累计层，但不是当前正式地图包的存储权威

源：`src/maps/include/lingtu/maps/layers/block_grid.hpp:12,135`。

- 默认block_size=16；外层`unordered_map<BlockKey,Block>`，块内`vector<Cell>`和
  `vector<uint8_t> occupied`。Cell保存float log-odds和uint16 hit/miss。
- 名称中的occupied在该块内为成员/存活单元标记，不应脱离实现把它解释成整张地图
  只有一个占据布尔值。缺失单元与已有概率状态仍有区别。
- 有runtime cells/blocks和persisted cells上限。块内稠密布局适合连续观测区域，
  但稀疏命中一个块也需承担块分配；没有现场分布测量前不能证明比逐体素hash必然更省。
- 类提供SaveBinary/LoadBinary，但本轮检索`src/maps`没有找到正式SaveMap调用这些
  接口。正式地图包仍走PCD/patches/OctoMap。不能因为类名和一个方法存在，就称现场
  已经自动恢复累计BlockGrid；也不建议为了“用上它”再加一份同权威地图文件。

## 唯一布局测量与内存估算

探针只include真实`rolling_occupancy.hpp`，使用当前WSL GCC13的`-fdump-lang-class`。
修前结果位于`build/map-storage-20261025/layout.cpp.001l.class:4172`：

```text
RollingOccupancyGrid::Cell
size=32 align=8
base size=26
```

已将私有Cell字段排列调整为double、int64、两个uint16、两个bool；类型、默认值、
公开格式和按名称访问的算法不变。改后同一真实头文件的GCC布局位于
`build/map-storage-20261025/layout-after.class`：`size=24 align=8 base size=22`。
这证实移除了填充，不证明射线积分或规划同比加快。

对默认4,000,000体素，下面是由实测Cell大小与源码元素类型计算的主要常驻数组
payload，**不是进程RSS、峰值或板端实测**：

| 数组 | 计算 | 字节 |
|---|---|---:|
| cells_ | 4,000,000×32 | 128,000,000 |
| 两份ray计数 | 4,000,000×2×4 | 32,000,000 |
| inflation_counts_ | 4,000,000×2 | 8,000,000 |
| 五份bitset | 5×ceil(4,000,000/64)×8 | 2,500,000 |
| 合计 | 不含其它层/临时内存/容器开销 | **170,500,000，约162.60MiB** |

改后按同一默认体素数计算，cells_为96,000,000字节，以上主要数组合计138,500,000
字节，约132.08MiB。相较改前减少32,000,000字节，约30.52MiB；这是数组payload
差额，不是进程RSS实测。相比直接把double改成float/Q8，这个调整不改变概率精度。
三层packed导出只需1,500,000字节，但不能据此说地图服务内存只有1.5MB。

## 正式持久化与可重建来源

### 保存链实际保留什么

当前ltlio的`writeCapturedMapFiles()`保存以下文件，再由pipeline辅助列表随source
commit事务保留。关键代码：`ltlio.cpp:519,569,588,739`；`pipeline.cpp:498`。

| 产物 | 保存的含义 | 不能声称的能力 |
|---|---|---|
| map.pcd | 冻结/修正后的地图表面 | 单独不能重建观察射线或unknown/free |
| patches/*.pcd | 选择的关键帧局部点云，实际写XYZ+intensity | 不是每一帧原始LiDAR包，也不是完整IMU记录 |
| poses.txt | patch文件名和修正后位姿；patch索引用qw qx qy qz顺序 | 不是带完整时间/协方差的因子图数据库 |
| poses.raw.txt | 修正前对应patch位姿 | raw在这里指未图优化位姿，不指原始传感器packet |
| trajectory.txt / trajectory.raw.txt | 带时间的修正/未修正机身位姿；轨迹格式qx qy qz qw | 当前窗口受历史长度约束，不自动等于整段任务 |
| keyframes.timestamps.txt | patch文件名和采样时间 | 不能替代完整帧时序 |
| scan_origin.txt | LiDAR原点在patch坐标系中的外参 | 不应丢掉它再用机身中心伪造射线原点 |
| patch_bundle.manifest | complete、dropped_count、sequence范围、patch_count | 完整性描述，不会补回已经淘汰的观测 |
| sam_loops.json / map_optimization.json | 当前保存分支的回环和优化证据 | 不是保存了所有原始优化输入的保证 |
| clean/removed/preclean（处理生成时） | 清理结果和被移除表面证据 | 文件存在/体积大小不能代替清理质量评估 |

必须区分两个索引的四元数顺序；直接按文本“八列一样”重放会错。

### 已存在的长采集限制

- `ltlio.cpp:89`默认`max_patch_snapshots=3000`；`:2825`超过上限淘汰队首并增加
  dropped_count。采样还有最小间隔/平移/转角条件，所以3000不是3000个原始LiDAR帧，
  也不能简单说恰好能录50分钟。
- `ltlio.cpp:1224,1904`对pose_history使用10000条上限，保存的trajectory可能只包含
  最近部分。不能在论文中无条件称作完整全程真值/估计轨迹。
- `io/patch_bundle.cpp:128`的HasSavedRays要求manifest complete、没有丢sequence且
  poses与patch对应；不完整bundle不会冒充完整saved rays。这个判断已经存在，不需
  再加另一套哈希/鉴权/检查框架。

### OctoMap构建实际使用什么

源：`build/pipeline.cpp:1066–1165`、`io/scans.cpp:12`、`octomap/rays.hpp`。

1. 读取最终map.pcd作为保留表面集合。
2. 如果HasSavedRays成立，读取patch、poses、scan_origin，按修正位姿将端点和真实
   ray origin变换到map，重放自由/占据证据；对最终保留表面做匹配和掠地面保护。
3. 如果只有PCD，则执行sampled_points分支；该分支的support/free dilation是推断，
   不能描述为真实观察自由空间。metadata记录evidence_source和navigation_ready来源。
4. 输出`octomap.ot`与metadata。保存和编辑沿用既有staging/commit事务与content epoch，
   这些是来源/派生物一致性的现有基础。

重要边界：`pipeline.cpp:212`明确navigation_ready是来源诊断，**不是激活或运动gate**。
当前MapStore激活核对.ot/full_log_odds等工件要求，不能说所有sampled_points地图
都已经被禁止激活。是否允许使用导入/人工补证地图必须由产品含义、操作者审阅和
规划可行性共同说明，不能把来源标签偷偷升级成新的硬限制。

### .ot与.bt的选择已经明确

`include/lingtu/maps/octomap/io.hpp:29`明确：默认.ot保存累计log-odds；.bt只保存
occupied/free的maximum-likelihood结果，使用const writer避免修改内存树。
`storage/store.cpp:868`对正式激活要求full OcTree .ot，metadata也应为full_log_odds。

建议继续让.ot作为正式导航派生物，.bt仅兼容/明确降级导出。两者都能表达树中已有
occupied/free与缺失unknown的区别；不能误说.bt完全没有free，也不能说.bt保留了
原概率。所谓压缩不能靠删掉概率或unknown语义伪装成无损优化。

## 点云字段和浏览器输出的真实缺口

### map.pcd的intensity在source commit中丢失

ltlio的`writeContractPcdBinary()`（`:550`）输出XYZ+intensity，但正式
`save.cpp:2025`调用CommitSavedSourceJson，后者LoadPcdXyz/FilterPcdPoints后在
`pipeline.cpp:2120`附近用WriteBinaryXyzPcd重写map.pcd。
`io/pcd.cpp:517`输出明确只有float32的x/y/z（12字节/点payload）。

因此：当前保留的patch仍可以有强度，正式map.pcd不保证有强度。不能把网页未来的
“强度配色”做成所有地图都显示可用，也不应把高度颜色称为传感器RGB。

推荐用现有PCD及source commit流程修复字段保留：明确最终几何source保留哪些原始
属性，过滤/下采样时如何给属性赋值；无传感器RGB就不编造RGB。不为此新增一套
平行点云数据库。论文数据包保留原patch和清理结果，不把展示色写成实验测量。

### 保存地图的30k展示预算没有限制原文件解析成本

`service.cpp:645`的GetMapPointsJson先LoadPcdXyz完整文件，再按stride导出最多指定
点数，最终为JSON XYZ；Gateway保存图入口默认30000点（`gateway/maps/routes.py:972`）。
输出小不等于读取和峰值内存小。stride也不是保边界、保细小障碍或空间均匀的LOD。

全局在线预览另走`global_map_cloud.bin/meta.json`，Gateway已支持现有PCLD二进制
传输，并有epoch/revision和点数预算（`gateway/services/global_mapping.py:72,105`）。
这是一条展示通道，不是OctoMap或保存原始观测的替代品。

推荐先复用已有map_id/content_epoch为单份展示采样结果提供复用，避免同一版地图
反复全量PCD解析；现有PCLD路径可复用时优先二进制。只有量测证明整图预览不能满足
规模/首屏需求时，再讨论按空间分块的LOD；不先造瓦片服务和新存储格式。

## 推荐推进顺序

1. **先保来源和属性。** 明确长任务需要保留的关键帧与轨迹范围，在现有patch bundle
   中完成增量落盘/封段或利用既有录制链保留任务源数据，避免把所有证据只放在有上限
   的RAM窗口。继续报告dropped/partial，不把部分观测称为完整。源数据保留不是要求
   导航热路径把全部原始数据常驻。
2. **已完成Cell字段重排。** 真实头布局已确认32→24字节，开启assert的原有rolling
   occupancy测试通过。没有量化内部概率、替换容器或改变窗口大小。
3. **在现有PCD source流程保留需要展示/研究的属性。** 先保证字段语义正确，再决定
   强度/高度等配色是否可选；清理过后仍需有可追溯来源。
4. **减少展示的重复读取。** 按已有内容版本复用展示数据；只有真实大图指标证明
   需要时才增加空间采样/LOD。展示预算不允许反向删掉导航依据。
5. **保持.ot派生物和来源同版。** 继续用既有构建事务和content epoch；验证修改/清理
   后重新构建与reload能获得一致查询。不要另加一套DB/哈希/校验清单。

## 本轮高度显示和验证

主Scene默认按地图Z高度显示蓝→青→绿→黄→红，保留单色切换。保存地图使用完整
有效XYZ的高度范围；改变显示切片不会重新定标。实时累计点云使用固定−1～2.5m
色阶，超出值用端点色并以≤/≥标注，避免逐帧伸缩；当前扫描覆盖层保留单色。
高度来自地图坐标，既不写回传感器RGB，也不写入PCD或改变占据判断。

高度配色、RGB保留、缓冲复用、多层/常高、切片、decoder和transport共45项前端
测试通过；TypeScript/Vite完整构建和6个涉及TS/TSX文件的ESLint通过。Vite仍报告
通用的包体积/插件耗时提示，本轮未扩大到打包优化。

地图语义回归直接编译原`tests/maps/cpp/rolling_occupancy_test.cpp`与当前实现，
使用`-O2 -pthread -UNDEBUG`确保assert生效，执行退出0。没有依赖Release CMake中
可能关闭的assert，也没有重跑无关导航/物理测试。重现命令与截图在
`build/map-storage-20261025/README.md`。

浏览器核验使用正式React源码、真实保留903地图的30,000点展示采样和已有本地API
fixture；切换高度/单色与全图显示已核验。相机为录制图片，所有状态/接口均为本地
fixture，不构成机器人在线、地图激活或导航运动证据。

## 未测边界

- 没有测板端RSS、CPU cache、ray更新时间、DDS实际带宽、save/rebuild耗时或浏览器
  首屏；上文数组成本均明确为计算估算。
- 没有执行长于3000关键帧/10000轨迹样本的采集，只依据实际可达的淘汰代码判断
  信息范围；未声称任何具体已存地图已经丢数据。
- 没有检查每个现存数据集的工件和强度字段；这里只证明正式路径能把字段写成XYZ。
- 高度色不是地面分类或可通行性标注。保存图的极端有效高度会压缩中间色阶；实时
  固定范围尚未为多楼层自动扩展，未以视觉处理擅自删除数据或改变地图基准。
- 没有在此验证地图几何精度、漂移、地面支撑质量、玻璃检测或真实可通行性。换一个
  存储格式不会自动修好这些问题。
- 本轮仅重排私有内存字段和修改网页显示；没有改变持久化或DDS格式。布局探针、
  窄测试二进制和前端验证日志保留在`build/map-storage-20261025/`，没有触碰共享
  CMake cache。未测板端RSS、帧率或真机效果。
