# 山地环境候选与接入决策

日期：2026-10-18

## 结论

当前 Blender 程序化山地只保留为材质、地形和传感器布置的技术预览，不作为最终美术环境。
正式山地场景优先采用官方 Unreal 样例或已授权的完整环境包，再把 LingTu 的 MuJoCo 物理层、机器人和传感器接入进去。

## 候选

### A. Valley of the Ancient：首选技术基线

[Epic 官方 UE5.8 样例说明](https://dev.epicgames.com/documentation/unreal-engine/valley-of-the-ancient-sample-game-for-unreal-engine?lang=en-US)

- 山谷和山崖环境已经完成构图，不是零散模型拼接。
- 官方说明大约 90% 的环境静态资源来自 Moab 的 Quixel Megascans。
- 使用 World Partition 与 Data Layers 管理大场景。
- 使用 Nanite、Lumen、Sky Atmosphere、体积云和 Niagara 环境效果。
- 可作为 RobotSimUE 山地 WorldPackage 的视觉基准。
- 需要通过 Epic Games Launcher 获取样例工程；本项目不从第三方压缩包下载。

### B. Mountain Forest：最接近“可直接拿来用”的完整森林包

[Fab Mountain Forest](https://www.fab.com/listings/ed738296-730e-4f52-ae10-a2bfb57d4097)

- 发布页描述为 UE5 山地森林完整环境。
- 包含 Landscape、PCG、RVT、Nanite、Lumen、Niagara 和现成相机。
- 页面列出 UE5.5，迁移到 UE5.8 需要实际打开工程验证。
- 页面列出 41 个静态网格、7 个树木 Blueprint、12 个主材质、155 个 2K–4K 纹理和 2 个 PCG 图。
- 这是 Fab 商业资产；未购买或未获得授权文件前，不进入 LingTu 生产库。

### C. Real3D/CG港山顶天文台

[Real3D 山顶天文台地编场景](https://www.real3d.site/?type=productinfo&id=1145)

- 页面描述为 UE5.2–5.7、367 个静态网格和 4K 纹理。
- 有明确山顶／天文台语义，比当前程序化场景更适合作为任务场景。
- 当前是付费下载，且没有进入本机的授权压缩包；保持候选状态。

## 接入原则

不把外部环境包直接当作 MuJoCo 世界。接入时拆成：

```text
外部 UE 环境包
    ├── 视觉资产与材质：UE
    ├── Landscape / Heightfield：转换为 MuJoCo 地形碰撞
    ├── 静态碰撞代理：MuJoCo
    ├── 语义区域与巡检路线：统一 WorldPackage
    └── 传感器挂点与机器人出生点：SessionSpec
```

UE 只负责显示、光照、植被、材质和视觉传感器渲染；MuJoCo 负责地形、岩石、坡面和机器人接触。两边共享实体 ID、坐标、尺度和版本指纹，不让 Chaos 与 MuJoCo 同时成为同一对象的物理权威。

## 下一步

1. 通过 Epic Launcher 获取 Valley of the Ancient，确认本机磁盘位置和 UE5.8 打开结果。
2. 复制为只读参考工程，不覆盖 RobotSimUE。
3. 从其中抽取山谷、山崖、Landscape、植被和光照配置，形成 `WorldPackage/mountain_valley`。
4. 为可行走区域生成 MuJoCo heightfield，为岩壁和障碍生成低模碰撞代理。
5. 将 Thunder 放入同一坐标系，先做坡面、岩石、路面和 Reset 四项验证。
6. 只有通过 UE 预览和 MuJoCo 接触验证后，才替换当前 Blender 山地预览。
