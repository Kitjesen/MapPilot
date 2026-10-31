# 地形补洞、表面重建与三维导航

日期：2026-10-30。范围：官方文档/源码核查与既有本地实验复用；本轮未修改地图或导航行为。

2026-10-31 状态更新：下文“当前代码边界”描述最初调查时的状态。后续已加入 C++
`SurfaceIndex`（默认 0.20 m 分层闭运算），见文末更新；Web 原始占据显示不包含其推断支撑。

## 结论

地形插值、补洞和平滑是常见的建模步骤，不能统称为“铺假地面”。有足够邻域几何支持的
补全可供规划使用，不应一律限制为显示。关键是保留真实观测与推断的区别，并避免把
真实落差、墙体、台阶边缘和多层结构用同一个连续表面抹掉。

上一轮 GEODE 图只绘制原始占据体素，适合检查生成结果，但完整导航表示不必止于
离散命中格。那张图不补洞是展示范围，不是所有导航算法必须遵守的规则。

## 一手证据

1. ANYbotics `grid_map` 官方
   [示例滤波链](https://github.com/ANYbotics/grid_map/blob/master/grid_map_demos/config/filters_demo_filter_chain.yaml)
   先由 elevation 生成 elevation_inpainted，再生成 elevation_smooth，并从处理后的
   层计算法向、坡度和粗糙度。它不是只做视觉着色的示例。master 于本次查阅，未在本轮
   固定整个仓库版本，也不能据此断言每台 ANYmal 的部署参数相同。
   [InpaintFilter](https://github.com/ANYbotics/grid_map/blob/master/grid_map_cv/src/InpaintFilter.cpp)
   用无效高度构造 mask，再调用 OpenCV Navier–Stokes inpaint。示例中的 0.05 m 是
   邻域算法半径，不是“只填宽度不超过 5 cm 的洞”；源码未按孔洞连通分量宽度作该限制。
2. ETH 的 IROS 2022
   [Elevation Mapping for Locomotion and Navigation using GPU](https://arxiv.org/html/2204.12876v1)
   的 II-H、II-I 讨论：射线给未观测地形高度提供上界；规划可乐观补洞或拒绝；
   提供缺失单元的边界最小高度填补，以及 Gaussian、box、median 平滑。论文中的上界
   不代表已测得精确地面，但可约束不合理推断。
3. 固定 `elevation_mapping_cupy` 提交
   `20a8a26b67a995b43eb44c23568854d1fed82a52` 的
   [Inpainting](https://github.com/leggedrobotics/elevation_mapping_cupy/blob/20a8a26b67a995b43eb44c23568854d1fed82a52/elevation_mapping_cupy/script/elevation_mapping_cupy/plugins/inpainting.py)
   支持 Telea 和 Navier–Stokes。相同版本的
   [Postprocessing.cpp](https://github.com/leggedrobotics/elevation_mapping_cupy/blob/20a8a26b67a995b43eb44c23568854d1fed82a52/plane_segmentation/convex_plane_decomposition/src/Postprocessing.cpp)
   保留处理前高度，机身平滑参考层依次补值、MORPH_CLOSE、box/Gaussian 平滑。
   这个固定版本的旧平面分割栈不应冒充当前 ROS2 分支的全部默认设施；当前官方文档
   明确区分 ROS2 与 legacy main。
4. DimOS 固定 `26144f7fd8424bdc33ab97dcd58e9497f2d413ee` 的
   [surfaces.rs](https://github.com/dimensionalOS/dimos/blob/26144f7fd8424bdc33ab97dcd58e9497f2d413ee/dimos/navigation/global_planner/mls_planner/rust/src/surfaces.rs)
   在多高度表面上按 Z 层闭运算，原表面保留；新增候选检查头顶净空和附近兼容高度的
   占据证据。此处 MLS 指 multi-level surface，不是 moving least squares。

这些来源证明存在成熟的补全机制，不证明它们采用相同补洞半径、适合所有洞或已移植到 LingTu。

## 本地已有证据：复用，不重跑

[长路线对照](../../build/long-range-20261030/README.md)使用历史真实
`903room_v6_corridor`，2026-09-30 保存，不是尚未取回的 10-29 新图。
同一 DimOS 核心、5 cm 格、0.60 m 净空、0.25 m 墙距，只改表面闭运算半径：

| 闭运算半径 | 找到路线 / 固定请求 |
| ---: | ---: |
| 0 | 0/5 |
| 0.05 m | 1/5 |
| 0.10 m | 1/5 |
| 0.20 m | 5/5 |
| 0.40 m | 5/5 |

这说明该图、该离散条件下，补全对连通性有实质影响；不是 LingTu 生产规划修改前后
成功率，也不是执行成功率。官方 Go2 的另一组 8 cm 格/0.05 m 墙距参数关闭补全仍为
5/5，所以不能把所有收益归因于填洞，也不能把 20 cm 提升为行业通用设置。

[表面数量和示意](../../build/xyz-global-20261030/README.md)显示原 316,953 格，
20 cm 新增 217,235 格（68.54%），40 cm 新增 417,324 格（131.67%）。统计包含不同
高度的地面、物体顶面等，不是新增测量点或纯地面面积。20 cm 参数在 5 cm 分辨率上
是四次膨胀再四次腐蚀，不等于洞宽上限 20 cm。相同 5/5 下没有证据优先扩大到 40 cm。

## 当前 LingTu 代码的实际边界

- `src/maps/include/lingtu/maps/layers/ground_surface.hpp::EstimateGroundSurface`
  用兼容邻域精化已有表面，但要求当前格具有自己的支持样本，因此不补空单元。
- `src/nav/cpp/planning/global/octoplanner/vendor/planner/src/global_planner.cpp::hasGroundSupport`
  在非 direct-support 模式可利用围住查询点的多个方向支撑，小范围容忍采样缺口；
  不代表完整表面修补已实现，具体是否采用该分支还取决于配置。
- saved-ray OctoMap 保存命中/自由证据，不将插值位置伪记为测量命中。
- 同日全局 XYZ 简化与 DimOS 调查已有独立记录；本轮未覆盖或重改它们。

## 推荐下一步

保留三维占据原图，从实际数据生成保留多高度的连续表面候选，在现有地图/规划边界
比较补全前后。邻域内同一平面/缓坡的小缺口允许参与路线生成；楼梯按不同踏面、坡面
按局部几何处理，不把所有 XY 压到一个高度。新实测和射线矛盾可以修正推断结果。
表面补全不同时把整根上方柱体改为空闲，也不清除原有障碍。

验证先包含连续地面、人为遮挡缺格，以及实际台阶、落差、墙和上下层间隙。可在有真实
命中的区域离线遮掉一部分点，再以保留的原始命中比较补全高度 RMSE、边缘误连数量
和路线变化。该人工遮挡试验只验证所选缺失模式，还需用后续扫描验证自然遮挡。
随后复用固定路线比较连通性、规划时间和执行表现；不以“图看起来更满”作为通过条件。

本轮无依赖安装、无新增补全实现、无部署或实机测试。

## 同日后续：先让 Web 显示真实导航体素

用户随后要求对齐 Web 与 OctoMap 的显示。已在保存地图场景增加点云/导航体素切换，
复用 mapd 文件描述符读取通道，浏览器后台解析实际 `.ot` 并画原始尺寸的占据单元。
局部 20×20 m 仅限制绘制，仍读取整文件；这一步没有补洞、修改地图或开启新导航算法。
43 项前端与 4 项 Gateway 定向测试通过，真实 GEODE 的 9,341 个区域叶节点与 C++
导出一致。历史 903room 图已在当前 Web 组件中显示；真实机器人连接、部署和运动均未
验证。[代码、截图与范围](../../build/web-octomap-20261030/README.md)。

## 后续原生补全与 Web 显示的区别（2026-10-31 核查）

`src/nav/cpp/planning/global/octoplanner/surface.cpp` 已在原生全局规划器中建立独立
支撑索引。按高度提取有上方净空的表面，逐层进行四邻域膨胀/腐蚀，新增位置检查原始
邻域占据证据及净空。普通 runtime 默认半径 0.20 m，`direct_support_only` 关闭推断；
原始 OcTree、PCD 均不改写。参数是形态学邻域半径，不是允许跨越的洞宽上限。

[原生记录](../../build/surface-native-20261030/README.md)中，历史 903room 原表面
316,953 格、推断新增 217,235 格。`final.jsonl` 固定路线 4/5 成功，剩余请求为
`start_snap_exhausted`。相同目录 `no-closing.jsonl` 也为 4/5，因此不能把这组当前
原生测试描述为补全带来 0/5 → 4/5；此前 0/5 → 5/5 是另一组 DimOS 参数实验。

当前方法尚未做连续坡面拟合；逐 Z 层闭运算不能保证修补跨多个高度层的稀疏斜坡，
也可能补掉小的真实缺口。视觉上的空白还可能对应已经自由或未观测空间。扩大体素
显示只能改变画面，不是地形重建或通行验收。下一项证据应是同区域原始点/原始占据/
推断支撑对照，以及有保留真值的缺格高度误差和台阶/真实边缘误连测试。

Web 现支持原始/2倍/4倍显示粒度（5 cm 图对应 5/10/20 cm，默认显示 10 cm），
并对主要高度或选定高度区间着色。目标表面选点仍查询原始体素。正式组件、原生接口
已接线，本地验证页仍使用历史地图和运行状态夹具；本轮未部署 NX。
[本轮代码、测试与截图](../../build/web-terrain-20261031/README.md)。
