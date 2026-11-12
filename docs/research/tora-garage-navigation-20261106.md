# Tora 固定姿态：地下车库全局／局部导航

2026-11-06。当前原生全局规划、SCAN、跟踪器和制动几何查询在同一车库地图中完成
一次 18 秒、14.70 米的理想底盘位置导航，终点输出零速度。实体录像复放这些记录，
不是预设动画。**这不是轮胎动力学、传感器定位、DDS 全链或实机验收。**

后续[转弯轨迹诊断](../../artifacts/nav/tora-garage-complex-20261106/diagnosis/README.md)
发现旧复杂场景的 42 条局部预览中有 5 条平面自交，均在尚未执行的尾段。
还补齐了测试入口遗漏的实际命令回写与最终制动暂停；当前同算法库对照仍有自交，
且未重现旧制动窗口，不能把接线修正宣称为曲线质量修复。

进一步的[固定官方对照与分阶段定位](../../artifacts/nav/tora-curve-upstream-20261106/README.md)
在 45 秒组件切片中捕捉到 4 条实际采用的自交候选。四例均首次出现在本地
移动边界重定时循环的 refine 输出，前面的 seed、拟合和 rebound 尚无自交。
已保存真实三次样条重建、实际 preview 匹配和首次异常前后对照；这项定位不是
生产修复或实机验收。

后续[移动重定时修复](tora-retiming-fix-20261107.md)已补入真实 manager 回归，
并通过 171 项不同的原生 SCAN、求解器、三维高度和速度边界测试。修法、
固定输入证据及同车库前后记录在该文中持续标注验收范围。

## 模型、场景与实现

- 原始 Paxini Tora Double One X 1.0.13 URDF；用户确认按零位姿锁定上身导航。
- 实体显示使用全部 86 个 collision STL 的三角面与 URDF 颜色，包含 1,128,377 个
  三角形。此前点状三视图仅是抽样顶点的覆盖诊断，原始模型没有变成点云。
- 场景 18×12 米，六辆简化停放车辆、两根柱、停车线、顶灯与标牌。观察侧墙和
  天花采用剖开展示；实际障碍盒体同时生成 MJCF 和规划场景，未为录像移走障碍。
- 当前 C++ 源码编译的 `benchmark_saved_route` 使用 `PlannerSession` → `Executor`
  → SCAN → Follower，并按实际输出速度做 100 Hz 平面积分反馈；没有预设行驶轨迹。
- 保存地图及局部占据由同场景 3D 几何真值体素化，分辨率 5 cm，保留地面和已知
  自由空间。局部膨胀按 Mapd 的圆盘及上下格规则执行一次；没有模拟激光噪声或遮挡。

地图来源、底盘响应和渲染三个层次必须分开：MuJoCo 在本轮负责实体场景的显示，
机器人位置来自原生规划器命令的理想积分，没有验证摩擦、轮速控制或物理接触力。

## 接地层的同条件对照

原模型原点 `base_link` 近地，固定 `base_imu_link_0` 的名义高度为 0.178000853 m。
全部模型最低点位于它下方 0.177985443 m，非轮刚性部件最低点在下方 0.130635955 m。

| 条件 | 原始全轮包络 | 固定姿态行驶包络 |
| --- | --- | --- |
| 下部范围 | 0.18 m | 0.14 m |
| 全局体素取整后的下部范围 | 0.20 m | 0.15 m |
| 水平圆柱半径／偏移 | 0.52 / 0 m | 相同 |
| 上部范围 | 1.57 m | 相同 |
| 地面、障碍、起终点、程序和其他参数 | 相同 | 相同 |
| 全局结果 | `goal_snap_exhausted` | 成功，3 个路线点 |
| 局部结果 | 未开始 | SCAN ready，完成闭环到达 |

逐键环境比对仅有 `LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M` 和其派生的
`LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M` 不同。原全轮包络底面落在地面以下约
2 mm；新下界位于地面上方约 3.8 cm，仍覆盖全部非轮部件并留约 9.36 mm 余量。
轮胎完整水平占地保留，最低接地层不再充当刚体障碍。原始全轮网格和失败证据均保留。

原版 SCAN 固定对照为
[`348e8a590a50a5a6bbab8d8c6dcfd171f009be26`](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/launch/advanced_param.xml)。
本轮没有修改正式全局或局部算法；0.52/0.14/1.57 m 是本平台模型的本地适配，
不是上游默认值。旧实验入口的单 OBB 限定已同步改为读取当前圆柱配置。

## 实际结果

| 指标 | 本轮结果 |
| --- | --- |
| 位置到达 | 是 |
| 反馈记录 | 1801 条，覆盖 18.00 s |
| 实际积分路径长度 | 14.6998 m |
| 全局路线点 | 3 |
| 已执行局部轨迹 ID | 12 |
| 最大平面指令速度 | 1.00 m/s |
| 终点 XY 误差 | 0.000301 m，仅理想底盘数值结果 |
| 位图碰撞／无效查询计数 | 0 / 0 |
| 全轨迹圆柱外缘到场景障碍盒的最小水平间隙 | 0.0468 m |
| 结束命令 | `[0,0,0]` |
| 清除任务后命令 | `[0,0,0]` |

全局首次查询约 1.87 s（含加载／准备），缓存查询约 0.41 s；首次局部规划约
89 ms。这里只报告本机本次结果。反馈从真实请求起点开始，保留了初始等待。
全局中间路线点 Z 约 0.153001 m，来自地面体素中心加名义支撑高度；理想底盘
实际 Z 保持 0.178001 m，没有伪造沿路线抬高底盘的物理行为。

本任务是 XYZ 位置导航。输入中保留的 π/2 终点朝向没有被 XYZ 路线执行；实际
结束 yaw 为 0.401885 rad。**本轮没有通过终点朝向验收。**

## 证据与复现

### 更复杂的车库与无文字录像

随后按用户要求增加了 24×18 m 的复杂车库：12 辆车、8 根柱、两条错位停车隔离带、
两个设备柜，共 40 个三维障碍盒。中央通道净宽 2.7 m，路线必须绕行多次转弯。
渲染删除了顶部／底部文字和背景条，仅保留实体场景、路线和无文字小地图。
观察视图隐藏头顶装饰梁和灯具网格，保留灯光及全部规划障碍，避免遮挡机器人。

初次运行虽到达，但独立连续圆柱／盒体核对发现约 9.67 mm 名义包络重叠。
测试地图生成器原先只判断体素中心是否在盒内，会漏掉不对齐格子的实体边缘；
现改为 `[floor(min/res), ceil(max/res))`，保持地面顶面 z=0 上方自由。
这是理想真值测试地图的修复，没有修改生产规划器、膨胀或机器人参数。
新增边缘格／相邻自由格／地面顶面的原生自检通过，旧图与反馈保留在初次运行目录。

修正后的最终运行 `runs/native-cell-coverage` 到达：79.90 s、52.2690 m、229 个全局点、
42 个局部轨迹 ID，终点零命令。位图碰撞计数为零，但精确名义圆柱包络仍有 5.50 mm
交叠，不能称其全程严格保持 0.52 m 圆柱净距。按原始模型的 0.464488 m 外接半径计算，
实体网格到非地面盒体的保守 XY 间隙下界约 0.050013 m；这是固定姿态几何计算，
不是物理接触或实机测量。28.36～29.85 s 的一个弯角有 75 个 10 ms 零命令与
74 个限速 tick，形成约 1.49 s 的交替制动窗口。录像按原始时间保留该过程。

复杂长路线的胶囊形显示段在 MuJoCo 首次阴影渲染中造成颜色异常。Tora 回放改用
不参与阴影绘制的 `mjGEOM_LINE`，保留全部路线点和实体阴影；首帧／次帧对照恢复正常。

最终录像与数据：[`artifacts/nav/tora-garage-complex-20261106/`](../../artifacts/nav/tora-garage-complex-20261106/)。
`native-navigation` 是初次地图结果；`native-cell-coverage` 是修正地图覆盖后的最终结果。

### 初版车库证据

证据目录：[`artifacts/nav/tora-garage-20261106/`](../../artifacts/nav/tora-garage-20261106/)。

- `scene.json`、`world.xml`、`render_scene.xml`：同源场景和实体模型。
- `scene-map.txt`、`map.ot`、`map-build.log`：三维真值地图输入及构建结果。
- `runs/baseline-full-envelope/`：保留完整轮胎包络的原始失败。
- `runs/driving-clearance/summary.json`、`analysis.json`、`resolved-config.json`：实际结果、
  独立轨迹间隙核对和实际解析配置。
- 同目录 `global-route.csv`、`feedback.csv`、`feedback-previews.csv`：全局、反馈和局部路线。
- 同目录 `navigation.mp4`、`replay-report.json`：1080p、25 FPS 原速记录回放及视频检查。

正式录像为 H.264、1920×1080、25 FPS、451 帧、18.04 s，约 4.62 MB。
`ffprobe -count_frames` 和 `ffmpeg` 全帧解码通过；首、中、末关键帧已检查。
只在观察镜头隐藏了遮挡画面的近侧装饰梁，全部规划障碍保持原样。

```powershell
cmake --build build/body-execution-20261022/native --config Release --target benchmark_saved_route --parallel 3
build/body-execution-20261022/native/tests/nav/Release/benchmark_saved_route.exe --check-inflation
.\.venv\Scripts\python.exe tools/bench/tora_garage_navigation.py --run new-driving-trial --seconds 90
.\.venv\Scripts\python.exe tools/bench/tora_garage_scene.py --run artifacts/nav/tora-garage-20261106/runs/new-driving-trial
```

本轮 11 项 Python 几何、配置、场景与录像时间轴检查通过；新增及修改的 Python 文件
Ruff 通过。原生当前源码构建、膨胀对照自检、1 米闭环烟测通过。

后续尚需真实底盘 SDK、坐标／传感器标定、实际地图输入和物理运动验证；最低
3.8 cm 内的轮前障碍、门槛与路沿不在本轮验收范围内，不能由这段录像推断可通过。
