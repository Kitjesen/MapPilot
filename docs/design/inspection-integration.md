# 巡检场景运行接线

更新：2026-10-16。目标：自主巡检、允许人工接管，执行与完成仍由原生模块确认。

这份记录区分源码接入、组件测试和完整同会话运行。组件通过不等于 FactoryPark 已经完成一场自主巡检。

同日后续：[任务日志首批实现](runtime-hud.md)已接入当前任务摘要、独立的证据/验收显示及原生控制入口，相关 UI Automation 增至 83/83。它还不是逐事件历史列表；新截图明确是未绑定任务的原生界面预览，不是自主巡检实跑。

## 数据与控制链

1. ProductControl 在 `env=sim` 启动选定地图的 inspection Product，产生当前 `product_session_id`。
2. 通过既有巡检 API 保存路线、受理任务。任务冻结地图版本、路线修订、完整点索引、点坐标与 Product 会话；不是运行中重新读取可变路线。
3. 原生 inspection executor 把当前点交给既有导航链，经历导航、停稳、停留、采集结果、推进或失败恢复。UE 不检测到达、不推进任务。
4. UE 读取任务、只读 guidance 和报告。guidance 需要同地图版本的有效 map-frame 定位；任务过期、定位不可用、点位改变或会话不匹配时隐藏。
5. 暂停、继续、取消经既有任务 API 返回原生模块。按钮发送请求，不修改 HUD 中的任务真值；原生回报才改变显示。继续游戏仅关闭菜单，不等于继续巡检。

任务的 `product_session_id` 与录制会话分开保存，受理后不可重新绑定。控制请求在预留请求标识时，以及调用原生命令前，核对当前 Product 会话；UE 还显式发送 `expected_env=sim` 和原会话标识。切换会话、切到真实环境或旧任务没有会话绑定时，不把控制发给新运行。旧任务仍然可查询。

人工接管后的恢复沿既有原生控制和任务合同执行。松开移动键不能被界面解释为任务已恢复。

## 已接入的代码

| 内容 | 实现位置 | 当前证据边界 |
| --- | --- | --- |
| 冻结点坐标与 Product 会话 | `src/gateway/services/inspection_task_lifecycle.py` | Python 任务/API 测试；不是物理行驶验证 |
| 目标与定位只读投影 | `src/gateway/services/inspection_guidance.py`、既有 inspection routes | 同任务、同地图版本检查；无路径时明确 unavailable |
| 目标方位、离屏提示、地图航向尺 | `LingTuSimUI/Private/LingTuSimInspectionGuidance.*`、`SLingTuSimNavigationOverlay.*` | C++/Slate 展示；不提供运动许可或遮挡证明 |
| 阶段提示与任务菜单意图 | 现有 FieldInterface、RuntimeUIWorldSubsystem | 等待停稳、采集等待、暂停与终态分别显示 |
| 显式启动绑定 | `InspectionPresentationBinding`，供现有 UnrealProcess/协调器注入 | 默认 None 不改变旧启动参数；不是第二套 Product 生命周期 |
| 三点路线制作入口 | `sim/tools/worlds/factory_park_hf/inspection_route.py` | 输出符合公开 API 的路线，不自行启动任务或运动 |
| 沥青/混凝土法线与粗糙度 | `material_textures.py`、现有 FactoryPark UE 生成脚本 | 原创烘焙纹理，真实 UE 隔离资产导入/主材质编译；未做最终画面验收 |

## 目标显示的含义

- 距离标注是估计直线距离，不是剩余行驶里程。
- 航向以地图 +X 为零度，不宣称地理北向。
- 目标依据 map-frame 定位，在收到 guidance 时与当前仿真姿态建立局部关联；随后相机与机器人运动不会把目标当成机身附件。
- 菱形和离屏箭头表示目标方位，不证明设备当前可见，也不代替物理射线查询或避障。
- 当前公开 Path 数据未提供本接口所需的完整执行关联，guidance 返回 `path.available=false`、`reason=execution_path_unbound`。不把检查点直连为假执行路径。

## 接入条件与使用

先准备真正对应当前世界的已保存地图及其 `content_epoch`。世界名、语义检查点或随手写的 epoch 不能替代已加载地图身份。

离线生成南门、装卸区、罐区的巡检目的地：

```text
python -m sim.tools.worlds.factory_park_hf.inspection_route \
  --map-id <已保存地图ID> --map-content-epoch <实际epoch> --revision <路线修订>
```

输出经现有 `POST /api/v1/inspection/routes` 保存，再通过 `POST /api/v1/inspection/tasks` 开始。真实采集要求 inspection evidence worker 已就绪并支持 `capture:overview`，不以命令 ACK 充当证据。

将受理结果中的任务、地图和 Product 会话绑定给已有协调器：

```python
from sim.runtime.coordinator.unreal_process import InspectionPresentationBinding

binding = InspectionPresentationBinding(
    task_id=task_id,
    expected_map_id=map_id,
    expected_map_content_epoch=content_epoch,
    expected_route_revision=route_revision,
    expected_product_session_id=product_session_id,
    gateway_url=gateway_url,
)
# Inject into the existing UnrealProcess / PlayableRuntimeConfig as
# inspection_presentation=binding. Do not resolve or launch a second Product.
```

默认手动驾驶会话不会因为存在这份路线文件，就自行生成地图、启动原生服务或进入自主巡检。

## 尚未完成

1. 让原生执行路径携带可核对的任务/规划关联，再接地面路线和下一转向提示。
2. 在实际 FactoryPark 实例化地图中完成材质投影、标识和设备细节更新，并录制画面与帧时间。
3. 同一次 Product/MuJoCo/UE 运行中完成三点任务、人工接管、显式恢复、采集与最终复核。
4. 对照实际机器人相机验证观察位姿、光照和证据质量；当前概念图不提供这些证明。

## 本批组件证据

- 生产修复 UE 编译：`build/unreal-tests/ubt-inspection-generation-green-20261016.log`，Succeeded。保留了前一次因 `TSharedRef` 调用错误而失败的 `ubt-inspection-guidance-20261016-final.log`，不以修复后的结果覆盖失败记录。
- 最终测试夹具编译：`build/unreal-tests/ubt-inspection-generation-fixture-20261016.log`，Succeeded。最终 UI Automation：`build/unreal-tests/inspection-guidance-verified-20261016/report/index.json`，81 项 Success、0 Failed、0 NotRun、0 SucceededWithWarnings；包含实际 Tick 的会话、模型、重置代际回归。
- UI Automation 首个完整版本：`build/unreal-tests/inspection-guidance-final-20261016/report/index.json`，80 项 Success、0 Failed、0 NotRun、0 SucceededWithWarnings。覆盖目标绑定、边缘布局、原生阶段与菜单意图；NullRHI 测试不是最终画面验收。
- 复审发现代际清理误放在 Construct 后，新增实际 HUD Tick 回归测试。旧实现的模型代际切换断言失败，原始报告为 `build/unreal-tests/inspection-generation-red-20261016-b/report/index.json`。该轮会话断言和 reset 前置也失败，但测试夹具分别遗漏了换会话前解绑、使用了错误证据路径，不能将这两项算作有效的生产缺陷复现。夹具已修正并检查绑定返回值。生产修复把比较和清理移入 Tick，不靠等待 HTTP 刷新清除旧目标。
- Gateway 任务、guidance、API 加启动绑定、路线与材质的组合测试：213 passed、2 skipped。两项跳过需要生成后的场景资产；测试命令禁用 cacheprovider 时有一条既有 `cache_dir` 配置提示。新增/修改的相关 Python 文件 Ruff 通过。
- 独立定向复审：APPROVE，原 Product 会话控制边界问题和运行中代际清理问题均已关闭。此结论不扩展为完整自主巡检或正式地图画面验收。
- 原生 C++ inspection：`build/inspection-native-guidance-20261016`，CTest 3/3 通过。它不包含 MuJoCo 实际行驶。
- UE 材质探针：`build/unreal-tests/inspection-material-import-20261016-a/result.json`。只写入 `/Game/RobotSim/Lookdev/InspectionMaterials20261016`，没有替换正式地图材质。导入/主材质编译通过，`rendered=false`、`physics_started=false`。

材质世界尺度投影依据 [Epic 的 WorldAlignedTexture / WorldAlignedNormal 文档](https://dev.epicgames.com/documentation/unreal-engine/texturing-material-functions-in-unreal-engine)；此处用于静态场景，不应用于运动的机器人网格，以免纹理随世界固定而发生游动。
