# 保存地图点云清理（DUFOMap）

`prune` 是保存链路的 C++ 清理命令。默认实现已由旧的
`visibility_surface_v4` 替换为 DUFOMap 的 UFOMap 空闲空间分类；没有旧算法回退。
命令名继续保留，现有 mapd、保存状态、取消、超时与部署路径无需改名。

```text
保存扫描与轨迹 → 沿用或优化位姿 → DUFOMap 清理 → 构建导航地图 → 原子入库
```

它只清理持久地图，不改变实时建图预览，也不产生导航控制指令。
移动行人离开后，后续扫描形成的空闲空间证据用于分离地图中的动态痕迹。
这不是行人语义识别；分离点数不能直接当作动态点真值或清除准确率。

## 输入与保留内容

- `map.pcd`：地图坐标系的原始点云。
- `poses.txt`：按采集顺序排列的 `filename tx ty tz qw qx qy qz`。
- `patches/*.pcd`：对应扫描坐标系点云。
- `scan_origin.txt`：`lidar_origin_in_patch x y z`，必须使用标定后的雷达原点。

每帧的扫描点和雷达原点一起变换到地图坐标系。缺少原点时必须传入经过核实的
`--sensor-origin X Y Z`，不默认使用机身原点。
支持 ASCII 和未压缩 binary PCD；不支持 binary_compressed。
清理输出保留源记录顺序、所有字段和原始值，包括 intensity、normal、curvature。
只按保留/移除掩码分流，不补面、不插值、不重新拟合点的位置。
这里指清理器输出、备份和分离记录；现有地图入库步骤仍将导航用 `map.pcd` 导出为 XYZ。

## 默认参数与来源

| 参数 | 默认值 |
| --- | --- |
| 分类体素分辨率 `--voxel-size` | 0.05 m |
| 未知邻域膨胀 | 1 |
| 回波保护距离 | 0.20 m |
| 雷达近端过滤 | 0.20 m |
| 最远积分距离 | 不额外截断 |
| 并行线程 | 2 |

积分使用中心降采样、精确射线和未知空间补偿，所有扫描完成后传播并查询
`seenFree`。这些默认值与 test6 已执行清理一致。`--voxel-size` 是分类分辨率，
不会将输出点云自动稀疏为一格一点；最终地图分辨率由保存流程的输出设置控制。

上游参考 DUFOMap `9e239ddd5995136e14f5212f33382a6ebc59e518`；
正式构建固定 UFOMap `fcb20f445d257280c039c07ab756c9d5b37ebd78`。
两者为 BSD-3-Clause，许可证随安装包交付。产品构建不读取本机 research_nav 目录。

## 使用与保存保护

```bash
prune --map-dir /path/to/map-copy --dry-run
prune --map-dir /path/to/map-copy
prune --map-dir /path/to/map-copy --overwrite --apply
```

只读模式只输出 JSON，不创建目录或写点云。普通执行输出 `map.clean.pcd` 和
`map.removed.pcd`，不改源图。应用时首次保留 `map.before_clean.pcd`，已有旧名
`map.pcd.preclean` 时复用；重复清理不覆盖首份备份。
空结果、截断输入、缺失标定、输出覆盖输入等错误不能替换原图。

mapd 默认要求清理成功。清理失败、取消或超时会保留原始捕获和作业证据，
旧地图在新地图成功提交前继续可用。正式入库携带首份备份及最近一次分离点，
不额外保留 `map.clean.pcd` 中间副本。
直接运行 CLI 不会更新已有 OctoMap；正式地图重建走保存流程。

JSON 保留 `success`、`reason_code`、`applied`、输入/保留/分离点数、扫描数和输出路径，
`algorithm` 标明 DUFOMap。旧算法的表面保护统计与 `--min-free-frames` 已移除，
不把旧参数静默解释成新算法参数。保存默认命令只传 `--map-dir --overwrite --apply`，
不受影响。

## 构建与验证

使用支持 C++20 的编译器、OpenMP、LZ4/LZF 开发包；TBB 为可选构建依赖。
Ubuntu 可安装 `build-essential cmake liblz4-dev liblzf-dev libtbb-dev`。
运行时需要对应架构的 `libgomp`；构建启用 TBB 时也需要对应的 TBB 运行库。
mapd 启动前实际执行 `prune --help`，缺运行库时直接保留加载器错误并拒绝启动。
固定上游源码由 CMake 获取，
离线构建可用标准 `FETCHCONTENT_SOURCE_DIR_UFOMAP` 指定相同版本的源码目录。

```bash
cmake -S src/maps/prune/cpp -B build/prune -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/prune -j2
ctest --test-dir build/prune --output-on-failure
```

测试覆盖实际 DUFOMap 分类、标定原点与位姿、只读、首次备份、重复应用、空结果拒绝、
截断输入和原字段分流。集成证据另见
[DUFOMap 接入验证](../../../docs/research/dufomap-integration-20261106.md)。
本机运行不等于 NX 性能或全场景清理质量验收；玻璃与长期静止行人的保持/移除质量
仍要用对应场景检查。清理结果不能代替独立定位精度或导航自由空间验收。
