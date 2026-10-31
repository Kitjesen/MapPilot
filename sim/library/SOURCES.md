# 场景与素材来源记录

更新日期：2026-10-20。下载、入库、UE兼容和物理交互是不同状态。

## 已选定产品背景：Factory Environment Collection

2026-10-20，用户确认实际 UE 工厂预览“很不错，作为我们产品的背景素材”。
这套工厂已用于产品默认首页，不再只是待领取候选。首页背景接入与可驾驶工厂地图是不同交付。

- 来源：Denys Rutkovskyi 的 Factory Environment Collection，
  [Fab 商品](https://www.fab.com/listings/2ee66462-8c2b-4303-892c-83f7fc0d9b3e)。
  已通过用户账号领取、下载；领取时页面标示 Standard License。
- 原始素材缓存：`D:/epic/FabLibrary/Factory Environment Collection-2ee66462`。
- 独立工作工程：`D:/epic/Projects/FactoryPreview/FactoryProject.uproject`。
- 选定画面：`/Game/Maps/Demonstration` 的生产线走廊、黄色机械臂、天窗与管线构图。
  实际 UE 5.8.1 预览见
  `D:/epic/Projects/FactoryPreview/Saved/AssetIntake/factory-demonstration-live-20261020.jpg`。
- 已确认：真实编辑器渲染；Outliner 中 11,872 个 Actor。基础资源检查登记了
  790 个 StaticMesh、62 个 Material、381 个 MaterialInstanceConstant、716 个 Texture2D。
  数量不是逐项视觉或交互验收。
- 当前用途：产品工业主题背景。首页背景表现与机器人选择页的可交互模型展示分开，
  不为背景加载整座工厂，也不把平面背景标为可驾驶地图。
- 实际接入：`/Game/RobotSim/Maps/LingTuFrontEnd` 已设为项目 `GameDefaultMap`。
  页面使用重新输出的 `Resources/FrontEnd/factory-corridor.png`，不再默认进入黑色机器人展台。
  正常默认启动实拍：`build/unreal-tests/factory-menu-20261020/home-render/home.png`。
  该次启动未使用 `LingTuDevRuntimeUIRender`，日志确认加载新入口并开始播放原创菜单音乐。
- 首次接入已输出无编辑器工具栏、无光标、无调试标签的真实 UE 场景画面；
  保留生产线工业主题，不用 AI 概念图冒充已渲染资源。
  图中现有工具栏与右下 Current Context 只属于参考截图，不进入产品 UI。
- 待处理：旧工程光照提示 18,822 个对象待重建；画面泛光和边缘色散需要收敛。
  旧叉车蓝图依赖 PhysXVehicles，尚未适配 UE 5.8，不能标记为可驾驶。
- 原始受限资源保留在 D 盘外部素材目录，不放进公开原始素材仓库。
  需要可驾驶工厂时，另行迁移选定区域及依赖、建立同源 MuJoCo 碰撞，保留 LingTu 物理权威。

Rural Australia 已领取但尚未下载，保持独立的自然环境候选，不混入本次工厂背景。
下文的 2026-10-18 获取说明是历史记录，以上状态优先。

## 山地场景预览：Mountain Outpost HF

本轮先做山地，不复用工厂院区素材。预览产物位于：
`build/mountain-outpost-hf-20261018/`。

| 层级 | 来源 | 用途 | 状态 |
| --- | --- | --- | --- |
| 山崖 | [Poly Haven Mountainside](https://polyhaven.com/a/mountainside) | 远景岩壁与山体主体 | 已下载，CC0，Blender预览已用 |
| 植被 | [Poly Haven Pine Tree 01](https://polyhaven.com/a/pine_tree_01) | 松林层次与遮挡 | 已下载，CC0，Blender预览已用 |
| 岩石 | [Poly Haven Rock 09](https://polyhaven.com/a/rock_09) | 路侧散石与近景锚点 | 已下载，CC0，Blender预览已用 |
| 地表 | [Poly Haven Rocky Terrain 02](https://polyhaven.com/a/rocky_terrain_02) | 地形颜色、法线、粗糙度 | 已下载，CC0，Blender预览已用 |
| 光照 | [Poly Haven Meadow HDRI](https://polyhaven.com/a/meadow) | 自然天空与环境光 | 已下载，CC0，Blender预览已用 |

预览脚本：`sim/tools/assets/build_mountain_outpost_hf.py`。
该场景包含确定性地形网格、贴地巡检路、山崖、岩石、松林、测量/传感器杆和一个小型设备台；它仍然是 Blender 视觉预览，不代表 UE 导入、MuJoCo 碰撞或机器人导航已经验收。

Real3D/CG港候选：[山顶天文台地编场景](https://www.real3d.site/?type=productinfo&id=1145)。页面描述为 UE5.2–5.7、367 个静态网格和 4K 纹理，但它是付费下载且当前未取得授权文件；在用户购买或提供已授权压缩包前，保持候选，不复制进生产素材库。

## 已下载并归档

| 来源 | 内容 | 本地状态 |
| --- | --- | --- |
| [Poly Haven Barrel 02](https://polyhaven.com/a/Barrel_02) | 蓝色储料桶、金属盖；作者Jorge Camacho | `industrial/ph_barrel/1.0.0`；带贴图；碰撞代理单体已测，UE未接 |
| [Industrial Storage Cart](https://polyhaven.com/a/industrial_storage_cart) | 金属双层推车；作者Jule Bielitz | `industrial/ph_storage_cart/1.0.0`；轮子与门待拆 |
| [Modular Industrial Pipes 01](https://polyhaven.com/a/modular_industrial_pipes_01) | 八个管路模块；作者Jorge Camacho | `industrial/ph_industrial_pipes/1.0.0`；阀门手轮待拆 |

三项均来自[Poly Haven官方CC0素材许可](https://polyhaven.com/license)，下载了2K完整PBR依赖。
保留来源和作者，不宣称为LingTu原创；API请求标识为LingTu-AssetLibrary。
GLB内嵌贴图，FBX与可编辑Blender工程均保留。不是从3A商业游戏中提取的资源。

## UE来源候选与历史获取记录

### Factory Environment Collection：2026-10-18 历史检查

[Fab商品页](https://www.fab.com/listings/2ee66462-8c2b-4303-892c-83f7fc0d9b3e)
2026-10-18重新核对，官方页面显示Free和Add to My Library，描述为完整工厂环境。
本次网页文本未提供可确认的许可证选项和完整UE版本清单；不沿用此前的版本上限作为当前结论。
账号是否已领取尚未核实，本机检查的项目与常见缓存目录中未找到该完整包。

[Epic官方介绍](https://www.unrealengine.com/blog/free-factory-environment-collection-now-available)
列出850+网格、430+材质、700+贴图，包含可驾驶叉车、机械臂、输送带和交互机械。
这些是发布方描述，不是本机性能或兼容性验收。

迁移方式：独立副本打开/升级验证，再按模块迁移Content；不要覆盖RobotSimUE项目设置。
先选厂房结构、货架、管线、设备细节和贴花。可借鉴交互蓝图，但与机器人接触的刚体仍由MuJoCo控制。
本项目使用UE5.8，需以实际下载版本在独立副本验证，不能把网页宣传当作兼容性测试。

### 山地首选：Landscapes Pack

[Fab商品页](https://www.fab.com/listings/c6a8fa58-84f1-4bb8-935d-a3467e9fe58d?lang=en)
2026-10-18官方页面显示Free和Add to My Library。作者DigitalTrisStudio描述含山地、雪地、草地等地形，
8K高度图/splatmap、岩石、草和地表材质。作为基地室外山地与山地驾驶地图的候选基础，
不是已搭好的机器人基地，也不是仅用于背景的图片。
尚未确认账号入库、下载、具体许可选项或UE5.8兼容性；不能标记为已安装。

### 森林细节备选：Electric Dreams Env

[Fab商品页](https://www.fab.com/listings/d79688f5-29be-4fb2-a650-2d4a813f5306?lang=en)
Epic Games官方免费示例，包含Quixel森林资产和PCG场景。
页面明确标注UE-Only Content。仅考虑UE视觉内容与示例工作流；不默认允许导出为独立MuJoCo资产包。
本轮不下载或整包启动这个较重示例，不把它混入浅色基地主题。

### 2026-10-18 执行状态与接入决定（历史）

- 主线采用Factory Environment Collection的结构/维护区，加Landscapes Pack室外地形；保持浅色现代基地主题。
- 免费商品页已核对；浏览器自动控制在Edge和内置浏览器均发生连接/导航超时，未完成领取按钮操作或下载。
  这是控制连接失败，不是账号未登录的证据。需要用户通过Fab/Launcher手动完成入库或提供已授权下载目录。
- 若首次领取出现协议，交由用户确认；本轮没有代为接受协议或购买付费商品。
- 下载后先在独立工程验证，再用UE迁移所需Content及依赖；不覆盖RobotSimUE的Config、插件或运行入口。
- 基地首页先做一个固定机位可对照验收的真实UE画面，再扩展可驾驶区域；概念图不计为实现场景。
- 与机器人发生接触的地面、墙、门等需要匹配MuJoCo碰撞表示；现成蓝图/Chaos交互不自动替代会话物理权威。
- [Fab Standard License摘要](https://www.fab.com/eula)允许将素材整合进项目，不允许独立转售/再分发原始素材；
  精确权利以该商品适用协议为准。受限源文件不进入公开原始素材仓库，UE-Only内容单独遵守其限制。

## 基地地面：本轮已下载

[Poly Haven Hangar Concrete Floor](https://polyhaven.com/a/hangar_concrete_floor)，作者Dimitrios Savva，CC0。
通过现有`polyhaven_environment_intake.py`下载4K Diffuse、OpenGL Normal、Roughness三张贴图并保存`source.json`：
`sources/polyhaven/textures/hangar_concrete_floor/4k/`。
这是源材质，不是UE材质实例或完整场景；尚未导入UE。
该扫描含磨损/裂纹，只作为维护区局部材质候选，不能整张强烈铺满浅色基地来冒充概念稿中的干净抛光地面。

### 备选：Warehouse Environment

[Fab商品页](https://www.fab.com/listings/ef0311b7-fd62-414a-b2c3-66ba95d8a21d)
发布方说明165+模块化资产、4K材质、Nanite/Lumen支持；面向较高性能机器。
历史限时免费信息不能当成当前免费。尚未核对账号报价/领取，不购买。

### 工厂建筑单体

[Industrial building plant factory 1](https://www.fab.com/listings/02d5d2ea-76bc-4536-98d4-7e8b86188ce5)
商品页显示免费，FBX/GLB等格式。尚未领取和核对具体下载许可，先保留候选。

## 材质扩展

[ambientCG](https://ambientcg.com/)提供可用于UE材质的PBR资源；[许可](https://docs.ambientcg.com/license/)为CC0。
下一批优先补混凝土、沥青、金属板与地面磨损，不用更多随机设备掩盖场景材质问题。
本轮未下载该站素材，亦未声称已建立UE材质实例。

## 接入验收

1. 先检查尺寸、UV/法线、贴图通道与许可，保留原始来源。
2. 正常应动的零件必须独立：轮子、门、手轮；贴图不能在拆分时默默丢失。
3. 视觉网格与简化碰撞代理分开；质量/摩擦无实测时标注设计假设。
4. UE真机预览、接触、驱动、受阻、Reset逐项验收后才晋升为可运行资产。
