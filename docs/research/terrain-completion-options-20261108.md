# 地面补全方法：外部证据与可验证的落地方向

调研日期：2026-11-08。类型：官方文档、版本差异及上游实现综合研究。
本轮联网读取官方文档、固定 SHA 源码和版本元数据，没有安装依赖、运行外部重建器或修改产品算法。

后续实施记录：同日已将有界内部洞、局部平面拟合及保存图自由证据否定接入共享 Maps SurfaceIndex、
正式全局规划与 Web 预览。以下保留研究时的候选分析；当前实现和真实对比见
[实施报告](surface-completion-20261108.md)。逐帧区域更新仍未接入。

## 可直接复用的判断

补小洞、拟合平滑曲面、融合多次观测和生成闭合网格解决的是不同问题。
对 LingTu 当前三维多层导航，优先验证“各高度层内的有界小洞＋周围实测几何＋真实射线否定”，
再考虑局部平面或 MLS 估高。不能以整张地图变得闭合、三角形更多或视觉连续作为可走证据。
这是依据下列来源提出的候选实施路线，是否采用、接口归属和运行预算仍由产品实施负责。

其中最有价值的新发现是 `elevation_mapping_cupy` 当前 ROS2 版本只选择面积有界、
且默认不接触地图边界的无效连通区域，再在小窗口内补值。历史 ROS1 分支没有同样约束。
DimOS 则提供多高度层、区域替换、局部表面图更新的参考。二者都不能自动证明补出的地面真实存在。

## 方法对照

| 方法／项目 | 实际输出及场景 | 多层与真实边缘 | 自由空间证据 | 运行与迁移成本 |
| --- | --- | --- | --- | --- |
| DimOS MLS 表面闭运算 | 从三维占据提取可站立表面，逐 Z 补小缺格，更新表面图 | 同一XY可有多个Z；闭运算仍可能连接真实缺口，必须限制尺度和几何证据 | 本次读到的补格函数只查附近占据及净空，不是完整射线否定器 | Rust实现；可学习局部区域更新，不需整体引入DimOS |
| elevation_mapping_cupy 有界 inpaint | 在二维高程层中补内部小洞；输出派生高程层 | 单个高程层不能直接表达上下楼层；先分层再借鉴组件选择 | inpaint本身不理解三维射线；核心可见性处理另属一条链 | CuPy/CUDA＋OpenCV；借鉴有界洞筛选即可，不要求引入GPU栈 |
| grid_map InpaintFilter | 把无效高程格转换成图像mask，以OpenCV NS修补，输出独立层 | 2.5D属性层；不是同XY的任意多表面。图像连续不等于真实边缘连续 | 过滤器没有三维自由证据 | C++，core依赖Eigen；CV过滤部分依赖OpenCV和ROS filters接口 |
| 局部RANSAC平面＋限定区域采样 | 指定地面patch拟合平面，在受限小洞内产生候选高度 | 可按表面层分别做；边缘/台阶需要从实测邻域保留 | 必须另行查询补点位置的free/occupied，RANSAC没有射线语义 | 可用已有线性代数/拟合实现；不能把全图非平面点视为噪声 |
| PCL Moving Least Squares | 邻域参考平面上拟合局部多项式，平滑和重采样点云 | 可用于三维局部表面，但邻域跨台阶、薄墙或楼层会混面 | 算法接口本身不携带观测自由语义 | C++ PCL surface/search；适合离线小ROI对照，不宜先全图重采样 |
| PCL GP3局部三角化 | 依据法向和近邻生成网格，限制边长与法向夹角 | 可有多个连通部件；适合局部光滑且密度变化平缓的面；大洞会保留或错误搭桥 | 无射线证据 | 已有点和法向即可做离线网格对照；网格不是地图支撑授权 |
| Open3D TSDF融合 | 使用逐帧深度、外参和轨迹，在3D体素内加权融合并提取零交叉面 | 三维表示可保留不同高度面，分辨率/截断距离会抹平薄物和细台阶 | 有逐帧传感模型；未观测区域仍不应被当成自由／地面 | 官方入门是RGB-D；LiDAR需正确量程投影和模型，不是把PCD直接塞进RGB-D示例 |
| nvblox TSDF/ESDF | GPU增量三维重建、距离场；当前官方有LiDAR接口 | 稀疏3D块可多层，表面质量依赖输入标定/位姿和分辨率 | 支持占据/动态与freespace链，但不同C++/ROS/torch接口能力不完全相同 | C++ CUDA，Jetson可源码构建；需要核对现有NX JetPack/CUDA预算，不是免费补洞插件 |
| Voxblox TSDF/ESDF | CPU增量融合及规划距离场，经典机器人参考 | 三维体素；TSDF→ESDF传播不等于填地面缺口 | 使用扫描射线与权重，仍需要区分未知 | CPU核心、protobuf/Eigen等；ROS wrapper另属依赖。代码较老，仅作历史算法参考 |
| Open3D Poisson | 从点与一致法向生成连续隐式曲面/网格 | 可能跨越空洞并在低密度区域外推；法向错误影响大 | 没有原始传感射线约束 | 适合展示/离线对比；不能把默认闭合面直接导入导航支撑 |

## 官方文档证据

1. [DimOS官方发布记录](https://github.com/dimensionalOS/dimos/releases)说明其3D多层表面规划器在地图表面建立节点并使用缓存图。
   下节固定源码用于补充它具体如何补洞和更新区域，而不是根据“MLS”缩写推断它使用移动最小二乘。
2. [elevation_mapping_cupy当前官方介绍](https://leggedrobotics.github.io/elevation_mapping_cupy/getting_started/introduction.html)
   明确文档面向ROS2 Jazzy的`ros2`分支，`main`是遗留ROS1。
   [官方插件说明](https://leggedrobotics.github.io/elevation_mapping_cupy/usage/plugins.html#inpainting)
   明确默认只填有界内部洞，较大或连接地图边界的未知区域仍保持未知。
3. [grid_map官方README](https://github.com/ANYbotics/grid_map)把它定义为多数据层的二维/2.5D网格，
   不是任意多高度表面的三维地图；inpaint是派生层处理的一种。
4. [PCL MLS文档](https://pointclouds.org/documentation/tutorials/resampling.html)：局部参考平面上的多项式拟合，可平滑重采样。
   [PCL GP3文档](https://pointclouds.org/documentation/tutorials/greedy_projection.html)：近邻投影三角化，边长和法向角度可限制连接。
   [平面分割文档](https://pointclouds.org/documentation/tutorials/planar_segmentation.html)：RANSAC寻找支持平面模型的内点，不能判断所有点真假。
5. [Open3D 0.19 TSDF文档](https://www.open3d.org/docs/0.19.0/tutorial/pipelines/rgbd_integration.html)
   使用深度图、内参和相机位姿做体素融合，并通过marching cubes取表面。
   [Poisson官方文档](https://www.open3d.org/docs/0.19.0/tutorial/geometry/surface_reconstruction.html#poisson-surface-reconstruction)
   明确会在低密度区生成三角形并外推，可用输出density剔除低支持部分。
6. [nvblox v0.0.10核心文档](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/technical_details.html)
   区分TSDF、ESDF及占据层；[传感器文档](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/core_library_sensors.html)
   明确支持相机与LiDAR，LiDAR存径向距离而非相机Z深度。
   [安装平台表](https://nvidia-isaac.github.io/nvblox/v0.0.10/pages/installation.html)区分C++与torch在JetPack上的支持。
7. [Voxblox原论文](https://arxiv.org/abs/1611.03631)与[官方ESDF说明](https://voxblox.readthedocs.io/en/latest/pages/How-Does-ESDF-Generation-Work.html)
   证明其目标是由TSDF增量构建规划距离场，并非将全部未知空间补成表面。
8. [ANYbotics elevation_mapping README](https://github.com/ANYbotics/elevation_mapping)
   已明确停止积极维护。它仍是高度方差、观测时间和射线可见性处理的历史源码参考，不能作为新维护依赖的背书。

## 固定源码核查与差异

以下是文档之外的实现证据，均读取了固定SHA文件。源码副本及API版本元数据保存在
`artifacts/maps/cleaning-report-20261108/upstream/`，未放入产品构建。

### DimOS：补面和区域更新

`dimensionalOS/dimos@26144f7fd8424bdc33ab97dcd58e9497f2d413ee:dimos/navigation/global_planner/mls_planner/rust/src/surfaces.rs:L131-L160`
按每个Z层进行闭运算；
[固定源码](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/global_planner/mls_planner/rust/src/surfaces.rs#L131-L160)。
同文件`L207-L220`在XY±3格、Z±3格寻找占据支持，`L260-L276`保留原格并对新增格检查净空和支持。
这不是局部多项式MLS拟合，也没有在该函数中查询known-free。

`dimensionalOS/dimos@26144f7fd8424bdc33ab97dcd58e9497f2d413ee:dimos/navigation/global_planner/mls_planner/rust/src/mls_planner.rs:L287-L326`
先替换区域占据，没变化则不重建；有变化时扩展受影响区域、替换新增/删除表面并修复图。
[固定源码](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/global_planner/mls_planner/rust/src/mls_planner.rs#L287-L326)。
该项目查询日仍活跃（API默认分支最新提交2026-11-07）；此处行为以用户已讨论的固定版本为准。

### elevation_mapping_cupy：当前版本值得借鉴的是洞的边界

`leggedrobotics/elevation_mapping_cupy@d9e1c9fe42ea3d463519333a443da99f680adde8:elevation_mapping_cupy/elevation_mapping_cupy/plugins/inpainting.py:L27-L69`
默认`max_hole_area=64`、`fill_border_holes=false`，以4连通组件筛选内部洞；
`L130-L147`逐洞小窗补值，只复制选定组件。
[固定ROS2源码](https://github.com/leggedrobotics/elevation_mapping_cupy/blob/d9e1c9fe42ea3d463519333a443da99f680adde8/elevation_mapping_cupy/elevation_mapping_cupy/plugins/inpainting.py#L27-L69)。
此分支最新提交日期2026-10-03。64是格子数而非米面积，不能原样当作我们地图的物理阈值。

遗留`main@20a8a26b67a995b43eb44c23568854d1fed82a52`的插件只构造无效mask并调用OpenCV，
没有上述组件筛选。当前官方文档和遗留源码必须分开，不可把新文档的限制套给旧实现。
这份实现也没有把一个XY任意多高度层保存到同一个高程值中。

### grid_map与历史可见性：分别借鉴补值和否定旧高度

`ANYbotics/grid_map@cdd0ea25724723302fa7c1a2e30e3fb8b0a578f4:grid_map_cv/src/InpaintFilter.cpp:L63-L94`
把invalid格设mask，按地图高度范围转换到8位图，以米半径/分辨率换算像素半径，再执行NS补值。
[固定源码](https://github.com/ANYbotics/grid_map/blob/cdd0ea25724723302fa7c1a2e30e3fb8b0a578f4/grid_map_cv/src/InpaintFilter.cpp#L63-L94)。
默认分支最新提交2026-01-13，官方仍列维护中的ROS分支。这里有量化和边界语义，不能当成无损3D重建。

历史实现 `ANYbotics/elevation_mapping@f4b082c64a3e660980da53b33c7936a8f2a2ea22:elevation_mapping/src/ElevationMap.cpp:L461-L511`
沿传感器到观测点的射线形成高度上界；只对本次扫描时间范围之外的旧格，检查
`elevation - 3*sqrt(variance) > maxHeight`，矛盾成立才清旧高度。
[固定历史源码](https://github.com/ANYbotics/elevation_mapping/blob/f4b082c64a3e660980da53b33c7936a8f2a2ea22/elevation_mapping/src/ElevationMap.cpp#L461-L511)。
它说明“没看到”与“射线证明看穿”不同，也避免当前扫描的新高度立刻被擦掉；
但单层高程删除逻辑不能直接替代我们多层表面更新。该项目最后默认分支提交2024-11-04，且已停止积极维护。

## 许可证、依赖和版本边界

| 项目 | 核对的许可 | 实施时的实际成本 |
| --- | --- | --- |
| 固定DimOS源码 | Apache-2.0，已读固定SHA LICENSE；GitHub当前API显示Other不能替代文件 | 直接借鉴需保留许可/归属，不能假设全项目所有依赖同许可 |
| grid_map／elevation_mapping | BSD-3-Clause | 核心Eigen与ROS/CV层要分开，后者不是ROS-free直接替换件 |
| elevation_mapping_cupy | MIT（已读插件头和LICENSE） | CuPy、CUDA、OpenCV，ROS1/ROS2分支接口不同 |
| PCL | BSD（[1.15.1许可文件](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.15.1/LICENSE.txt)） | 在线文档标1.15.1-dev；要与本机实际版本核对后再调用，不能按网页版本宣称已装 |
| Open3D | MIT（[v0.19.0许可文件](https://github.com/isl-org/Open3D/blob/v0.19.0/LICENSE)） | 本报告是方法参考，没有新增其运行依赖 |
| nvblox | Apache-2.0，已读`5bd2d73da901ce0ba279c997a4c1f9f632b9309c/LICENSE.md` | 需CUDA、GPU内存及目标JetPack核验；C++/ROS/torch能力不能混称 |
| Voxblox | BSD-3-Clause | 经典CPU参考，默认分支最新提交2021-11-22，不作为活跃新库首选 |

上述SHA表示本次读取证据，不代表LingTu当前已安装这些版本。nvblox网页为v0.0.10文档，
API读取默认分支`5bd2d73…`最新提交2026-10-23；两者分别注明，没有声称严格同一构建。
这些库的采用/替换还需依赖评审；本报告不授权新增依赖。

## 交给实施的最小候选与验收样例

当前可先验证有界小洞组件选择和物理半径限制，保留逐层表面，不必首先替换OctoMap或安装GPU重建栈。
对被选中的小洞，再利用周围同层点拟合局部平面或低阶曲面，得到候选Z和拟合残差。
候选必须与现有三维占据/自由证据对齐：射线穿过预计地面位置时不能仅凭邻点存在继续补面；
上方自由不等于地面不存在，未观测也不等于已知自由。保留observed/inferred及原始点，
为Web显示和新观测撤销推断提供明确来源。这些是待实现语义，而非本轮已完成行为。

能检验修法是否有用的最小样例是：

- 同一真实平面挖出小内部观测缺格，恢复连通性，同时报告新增面积和拟合残差。
- 同尺寸真实台阶边、地面外沿和落差保持断开，避免只以“补了多少格”算成功。
- 同XY上下两层分别变化，只更新相应高度层。
- 新射线穿过原推断高度后撤销该推断；同帧真实命中按融合后证据处理。
- 回环修正历史位姿后，派生面与新坐标一致，旧推断不残留在错误位置。
- 手动补充记录明确高度、边界和人工来源，保存/重载后仍可检查与撤销。

研究证据足以支持以上候选方向，但未运行这些方法在test6上的补面实验。
对大范围缺测、玻璃镜面与真实坑洞，单靠闭运算或平滑没有足够证据保证正确。
