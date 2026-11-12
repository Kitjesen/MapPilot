# 保存清理替换为 DUFOMap：接入与验证

开始于 2026-11-06，验证完成于 2026-11-07。本轮由用户明确要求把 test6 使用的清理方案集成到项目并替换旧清理。

## 实际改动

默认 `prune` 的 `visibility_surface_v4` 分类已删除。生产入口现在使用 DUFOMap 固定上游流程，
没有可选旧 backend 或失败后自动运行旧算法。命令名保留是为了复用现有保存、取消、超时、
备份与部署契约。`--min-free-frames` 及旧表面归因统计删除，旧配置不会被静默忽略。

UFOMap 固定版本由正式 CMake 获取，不依赖被 Git 忽略的研究目录；静态链接其实现，
两份 BSD-3-Clause 许可证及 NOTICE 随 `lingtu_runtime` component 安装。

- DUFOMap：`9e239ddd5995136e14f5212f33382a6ebc59e518`
- UFOMap：`fcb20f445d257280c039c07ab756c9d5b37ebd78`
- 默认分类分辨率：double 0.05 m；未知膨胀1；回波保护0.2 m；近端过滤0.2 m；两线程。
- 使用保存位姿同时变换扫描点与真实雷达原点；最后传播节点并查询 `seenFree`。
- 原始记录按掩码分流，保全部字段及值；首次原图备份、空结果拒绝、失败恢复继续有效。
- mapd 启动前实际执行 `prune --help`，及早暴露缺运行库问题。

旧平面辅助函数只供离线增量实验使用，已搬到 `research/maps/cleaning/surface.hpp`。
该实验不是产品清理的回退方案。地图入库原有的导航 XYZ 导出没有扩展；清理器输出、
原图备份和分离点记录保留原始字段。

## 本机 test6 完整回归

平台为本机 x86_64 WSL Ubuntu、GCC 13.3 Release，不是 Go2/NX。
在采集副本上实际执行新版 `prune --apply --overwrite`，原始采集包及客户交付地图均未改动。

| 项目 | 结果 |
| --- | ---: |
| 输入扫描 | 681，未匹配0 |
| 原始记录 | 11,709,219 |
| 保留记录 | 11,258,350 |
| 分离记录 | 450,869（3.85055%） |
| 完整命令耗时 | 53.19 s |
| 峰值 RSS | 459,444 KiB（约449 MiB） |

完整命令耗时包含读取、积分、分类、清理/分离写出、首份备份和应用，不包含测试副本复制。
后台另有地图测试编译任务，以上是一轮本机实测，不是目标板耗时承诺。

新结果逐条、逐字段与已有客户清理结果完全一致，包含 XYZ、intensity、normal_x/y/z、curvature。
应用后的 map.pcd 与 clean 输出一致，首份备份与原始 capture/map.pcd 字节一致。
证据目录：`build/dufomap-integration-20261106/`，包含 `native-result.json`、`native-time.txt`、
`test6-validation.json`；测试副本在 `/home/thunder/.cache/lingtu-dufo-native-test6-20261106`。

## 验证入口

```bash
ctest --test-dir /home/thunder/.cache/lingtu-dufo-backend-20261106 --output-on-failure
ctest --test-dir /home/thunder/.cache/lingtu-dufo-map-tests-20261106 -R 'lingtu_maps_(save_map|process)_test' --output-on-failure
/home/thunder/.cache/lingtu-dufo-map-tests-20261106/lingtu_maps_save_map_test /home/thunder/.cache/lingtu-dufo-backend-20261106/prune
```

- 原生清理器13项通过：分类、位姿/非零原点、输入异常、dry-run、重复应用、空结果和全字段IO。
- 保存及进程测试2项通过；另以真实新清理器执行保存入口，验证删除结果、加载、备份、清理失败保留旧图。
- 地图清理及部署边界Python定向5项通过。
- 离线增量实验回归1项通过，maps_cleaning_replay编译通过，移走平面辅助函数没有破坏研究调用。
- 扩大的部署测试文件为56通过、3跳过、1失败；失败为已有导航控制断言
  `test_native_endpoint_accepts_only_typed_command_and_operator_motion_inputs`，涉及另一会话正在修改的
  `control_authority.activatePath()`，本轮未修改该导航实现或测试。

首次配置会获取固定上游源码。离线可传
`FETCHCONTENT_SOURCE_DIR_UFOMAP=/path/to/pinned/source`。系统需要 OpenMP/LZ4/LZF 开发依赖，
发现 TBB 时启用其并行实现。实际安装验证包含 bin/prune 与三份许可文件；本机 `ldd` 没有私有
缓存目录内运行依赖，运行时需要 libgomp 及启用时的对应 TBB 库。

## 2026-11-09 Go2 / Orin NX 部署候选

在 Orin NX（aarch64、GCC 10）构建时发现固定 UFOMap 上游的两处可移植性问题：

- 三个头文件无条件包含 x86 intrinsics。构建目录中的头文件副本按 `__BMI2__` 包含，保留原有可移植实现。
- `Octree::toKey(coord)` 先把负浮点格号转换成无符号整数，再加原点偏移；该转换超出可表示范围。
  NX 实测 `(-0.025, -0.075, -1.025)` 得到三个原点格号 `262144`，正常应为
  `(262143, 262142, 262123)`。改为先加原点偏移，再转换成无符号格号。
  同样使用构建目录头文件副本，不修改下载的固定上游源码。

修正前 NX 清理测试 7/13；修正后原有 13 项及新增正负坐标回归均通过（14/14）。
同一修正的 x86 WSL 回归也为 14/14。新增测试直接检查三轴正负格号及中心坐标往返，
现有测试继续覆盖真实清理入口的分类、位姿变换、原点标定和保存语义。
没有改变分类参数、测试预期或应用保护。排查用的串行执行和 `-ffp-contract=off` 已恢复。

这是目标板原生组件证据，尚不证明现场点云质量或 Go2 导航通过。
原始日志保存在 `artifacts/deploy/go2-20261109/` 和目标板同名部署目录。

## 原始验证边界（2026-11-07）

没有登录、更换或重启 NX 服务。目标 ARM64 编译、运行库、真实保存耗时与现场玻璃/动态质量
仍需在目标设备验收。本轮不把单次清理效果作为全场景动态点清除率、玻璃保留率或绝对精度证明。
