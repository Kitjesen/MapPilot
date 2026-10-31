# MATRiX v1.1.2 Linux camera chain: actual recovery, material resources and output contracts

## Outcome / request type

2026-10-21。请求类型：发行二进制实现调查 + 公开文档/解码器核对。**没有完全闭合“配置 → 生产实例创建/挂载 → 每像素材质运算”的全部链路；不能把这项调查写成整套相机已完整还原。** 本次已实际恢复原生相机的采集、后处理挂接、GPU 回读、异步编码及输出路径，并将队列传输追至 `z_put`。配置解析和挂载辅助函数有完整原始指令证据；生产 Blueprint 工厂/挂载桥和红外材质主体仍是明确缺口。

这不是此前三个局部函数的重包装：本次证据集合含 **48 个函数、64,297 / 64,297 字节完整线性解码**；其中 **29 / 29 个函数获得 Ghidra 伪代码，procedure identity 均 resolved**。完整解码不等于所有分支语义已理解；伪代码成功也不等于无警告。另 14 函数投影的 Ghidra 导入超时，不能算反编译成功；传输尾部的 5 函数只主张原始反汇编结果。

最重要的结论：**这条 infrared 相机输出已证明是压缩图像，不是已证明的摄氏/开尔文温度帧。** 深度的米制 float32 合同不能借给红外。Semantic、fisheye、panorama、PTZ 也必须分别说明其标签、几何和控制语义。

## Scope, target and evidence authority

- 原始目标：`D:/inovxio/tools/matrix-analysis/runtime-v1.1.2/inputs/MATRiX_v1.1.2/UeSim/Binaries/Linux/UeSim` 及同目录 `UeSim.debug`。
- ELF/debug GNU Build ID 均为 `28d46d067b635be708a2e7e6ce9f5128e78bbe6c`。原始 executable SHA256 为 `ed98e648c3298d0efec44170f48b0ad528c9d3010ad743a0f0dacf20d52f7bc1`。
- 公开源码/文档保留树：`D:/inovxio/tools/matrix`，commit `55ad3fec77deaf1b589d27fa6cc4bfca4553754e`。它提供配置、协议和接收工具，不是完整 UE 相机 C++ 源码。
- 上游 [v1.1.2 build manifest](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/docs/releases/v1.1.2-build.json) 将该 executable hash 关联到 `20260923T022450476117Z-6596e3e1`、2026-09-23 10:24:50 +08:00 的重打包。**不能只凭 v1.1.2 文件名把另一版重打包视为相同。** 默认传感器集合为空；模板存在不等于已激活。
- 本次只写本报告及 `build/research/matrix-camera-chain-20261021/**`；没有改 LingTu runtime、没有启动 UE、没有服务付费/工具安装、没有子代理、没有复制资产进 runtime，也没有解密资源。
- 下文 `B-*` 是本次完整原始机器指令的本地引用，**不是 REA Evidence ID**；`E-*` 映射到真正的 REA Evidence ID。`P-*` 是经过格式解析和存储校验的包证据；`D-*` 是公开文档/工具证据。这些权威层级不能互换。

证据目录（以下相对路径以此为根）：[`build/research/matrix-camera-chain-20261021/`](../../build/research/matrix-camera-chain-20261021/)。每个 `readable/0x*.asm` 是 JSON 的可读投影；每个 `.c` 是原始 REA 返回的伪代码，不是原作者源码。原始 JSON、失败和限制均保留。

## Seven-stage coverage: what actually connects

| 阶段 | 已读到的实际实现/证据 | 结论与尚未闭合的边 |
| --- | --- | --- |
| 1. Configuration | `ParseMujocoSensorJson` `0xdd38650`（E-A）；其调用的 `ReadGenericSensor` `0xdd38cf0`（B-config，13,490 字节）读取公共字段并派生运行时属性；发行包配置与 D1/D3 交叉核对 | **解析已证实，向具体生产 camera actor 的属性传递未逐项证实。** 不把 parser 当 factory。 |
| 2. Factory / creation | `ARobotCameraSensorActor` ctor `0x11a54d30` 创建根、yaw/pitch pivots、gimbal/UDP、2D/cube capture（E-A）；P1 包含 `BP_PTZCamera`、`BP_MujocoSceneActor`；D3 明确配置驱动 PTZ Blueprint 流程 | **部分。** 原始 `.text` 对 `SpawnActor<ARobotCameraSensorActor>` 的两处候选调用都归属 `FRobotSemanticCameraRenderTest::RunTest`，不是生产工厂。未恢复生产 Blueprint 字节码中的 dispatch/spawn/property assignment，不能拿测试补这条边。 |
| 3. Mount / pose | ctor 的 `SetupAttachment` 调用；`GetMujocoMeshRelativeSpawnTransform` `0xdd739d0` 原始指令；`CaptureAndPublish` 内取得 relative transform 并调用 `URobotCameraGimbalComponent::SnapshotCapture`（E-B + B-mount） | **部分。** 证明组件局部层级、挂载辅助 API 与采集时云台快照；尚未证明生产配置 `sensor_attach` 到 camera actor 的完整绑定、单位/符号变换和动态 body-follow 每一跳。D1 的米/度/轴约定不是这一跳的反编译证明。 |
| 4. Capture / scheduling | `Tick`、`PollCapture`、`HandleRobotSensorSynchronizedTrigger`、`CaptureAndPublish`、perspective/cube submission、同步 readiness（E-B/E-D） | **原生主体已恢复。** 时间/触发信息随请求复制，队列容量影响是否接收；不是“调用次数=实测发布率”。没有运行目标、GPU、地图和同步组的动态验收。 |
| 5. Postprocess / projection | `ConfigureCaptureComponents`、`ApplyInfraredPostProcess`、`PrepareSemanticCapture`、`EnqueuePerspectiveDistortion`、`BuildCubeProjectionSample`（E-A/E-D）；红外材质路径及 P1 资源 | **挂接/投影已恢复，材质内部未闭合。** 存在颜色材质实例/曲线与 shader archive，不等于已知道温度模型、材质输入或 shader 算式。 |
| 6. GPU readback / raw format | `SubmitPerspectiveGpuReadback`、`SubmitCubeGpuReadback`、`ProcessGpuReadbacks`、render-thread closure `0x11a82540`（E-B） | **回读布局与后续像素容器已恢复。** 证明 Lock/pitch/height 检查、逐行拷贝、Unlock 和完成标志；ProcessGpuReadbacks 大函数有严重类型传播伪影，个别转换需以原始指令为准，不能按伪代码低地址全局变量字面解释。 |
| 7. Encode / publisher / wire | `QueueImagePixelsForPublish` → encode closure → completion closure → `PublishByteFrame`（E-C）；Hub → Zenoh publisher → `EnqueueCompressedImage` → CDR helper → MPSC queue → IO `FlushBatch` → `z_put`（B-transport） | **主选定图像路径已追到传输 API。** Hub 的虚调用目标需结合已读到的 publisher 构造与 override 路径理解；并非所有 sink 都强制发 Zenoh。没有远端收到包、持续吞吐、可靠性/队列饱和实测。 |

本表以用户需要的七个阶段覆盖调查范围，**不是七阶段全部完成的勾选表**。

## Recovered native chain and exact anchors

### Configuration, creation and mounting: preserve the discontinuity

`ParseMujocoSensorJson` 实际调用 `ParseJsonObjectString`、`ReadGenericSensor`、config move-assignment。`ReadGenericSensor` 原始指令引用并消费 `sensor_type/topic/frequency/position/rotation/height/width/fov/sensor_attach/frame_id/enabled/ignore_mounting_actor`、`fx/fy/cx/cy`、`K1..K4/P1/P2`、`CubeFaceSize`、PTZ ports/stabilization，以及异步/积压参数。保留 `stablization` 历史拼写与 `stabilization` 别名；运行时桥属性名包括 `GimbalControlPort`、`GimbalStatePublishPort`、`bStartGimbalOnBeginPlay`。这证明 parser 的工作，不证明后面哪个 Blueprint 确实消费了每个值。

ctor 原始调用点：`0x11a553a4 / 0x11a55408 / 0x11a55526 / 0x11a5563b` 为 `USceneComponent::SetupAttachment`；同时创建 2D/cube capture 和云台子对象。实际字段内存偏移是此构建的私有实现，不应变成 LingTu ABI。

`creation-call-candidates.json` 是 E8 字节候选扫描，不是一个全局排除证明。选定 `SpawnActor<Camera>` 候选只落在测试函数；内联、间接、UE reflection 和 Blueprint 调用不在这种搜索的否定范围。P1 定位到生产 Blueprint 是后续正确入口，**目前没有把“存在该文件”升级为已恢复其控制流。**

挂载公开约定：位置米、RPY 度、局部 +X 前/+Y 右/+Z 上，相对 `sensor_attach` body（D1）。`GetMujocoMeshRelativeSpawnTransform` 是实际存在且有函数体的辅助路径，但尚未恢复它到生产相机 spawn 节点的调用边。因此不能在本报告承诺 camera extrinsic 已全链验证。`frame_id: "map"` 仅是配置/消息字符串，不会自动使图像像素变成世界坐标测量。

### Capture → postprocess → readback

有实际调用点的主干：

```text
Tick / synchronized trigger
  -> CaptureAndPublish (0x11a595b0)
     -> perspective: SubmitPerspectiveGpuReadback (0x11a5c790)
        -> ConfigureCaptureComponents (0x11a56390)
           -> EnsureRenderTargets (0x11a55b10)
           -> ApplyInfraredPostProcess (0x11a5c250)
        -> PrepareSemanticCapture (0x11a5f1a0; non-semantic returns success)
        -> USceneCaptureComponent2D::CaptureScene
        -> EnqueuePerspectiveDistortion (0x11a5f040)
        -> pending readback request, copying trigger/sequence/config snapshot
     -> cube: SubmitCubeGpuReadback (0x11a5cde0)
        -> cube capture/projection helpers (GPU helper bodies not all recovered)
ProcessGpuReadbacks (0x11a57590)
  -> render-thread poll closure (0x11a82540)
     -> FRHIGPUTextureReadback::Lock -> row copies -> Unlock
  -> format-dependent pixel/depth handling -> QueueImagePixelsForPublish
```

CPU fallback `CapturePerspectivePixels` 也实际调用 Configure/Prepare/CaptureScene/Distortion，再通过 render-target 虚接口读取 `TArray<FColor>`，并检查 `width*height` 与数组长度一致。它和异步 GPU 路径并存；不能声称所有相机每次必走同一路径。

`ApplyInfraredPostProcess` 仅在 camera type **4** 选取 actor `+0x380` 的材质；从旧 blendable 列表移除旧对象，更新/插入当前材质，权重限制到 `[0,1]`，并设置 postprocess blend weight。**这里没有恢复到“设置目标温度、发射率或单位”的逻辑。** 这是一项对该函数的观察，不是全程序不存在这些能力的证明。

`EnsureRenderTargets` 原始数值包括 `2/10/13/37`。Epic 的 [UE 5.6 EPixelFormat](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Core/EPixelFormat?application_version=5.6) 将其分别定义为 `PF_B8G8R8A8 / PF_FloatRGBA / PF_R32_FLOAT / PF_R8G8B8A8`。这里用文档解释常数，不以文档代替二进制分支证据：透视普通颜色/infrared 的主要 RT 为 2；GPU depth 分支可选 13；cube RT 为 10，projection/distortion RT 可为 37。Semantic 有独立 RT/view-extension。不是所有这些格式都作为 raw 网络载荷发布。

回读 closure 的三个重要事实（E-B；原始调用点 `0x11a825cc / 0x11a82732 / 0x11a82612`）：

1. `Lock` 返回的 row pitch/height 与请求宽高比较；不是盲目假定 GPU 行无 padding。
2. 目标分配 `width * height * bytesPerPixel`，每行从 `rowIndex * rowPitch * bytesPerPixel` 拷贝 `width * bytesPerPixel`。
3. 之后 Unlock、回收 readback、设置完成标志；这些是静态实现，不是本次观测到了 GPU 已完成。

### Encoding → actual queued wire path

`QueueImagePixelsForPublish` 验证像素数量，捕获主题、frame ID、trigger、sequence、尺寸和一个 generation-like counter，移动像素至异步任务。编码闭包 `0x11a82ba0` 在编码前比较这个计数快照；完成闭包 `0x11a83280` 再比较、暂存完成帧并按待完成序列处理，发布前检查弱对象有效性。**不把这次局部观察扩大成所有 reset/race 都已被验证。**

`CompressColors` 及异步闭包交给 ImageWrapper 的是 **`width*height` 个 FColor、每像素 4 字节、format 数值 1、每通道 8 bit**。数值与 Epic [ERGBFormat](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/ImageWrapper/ERGBFormat) 的 BGRA 对应；这是编码输入布局，不是“网络裸 BGRA 格式”。普通图像可选 JPEG/PNG；quality 限制在 1..100；Semantic、depth encoded PNG 的分支强制 PNG。PNG 路径不应被 JPEG quality 的名称误导。

真正选定的发布链：

```text
encode closure 0x11a82ba0
 -> game-thread completion closure 0x11a83280
 -> PublishByteFrame 0x11a5e160
 -> FRobotSensorPublisherHub::Publish 0x11a528e0
 -> selected FZenohRobotSensorPublisher::Publish 0x11a7b530
    [0x11a7b5a7] EnqueueCompressedImage 0xda56320
    [0xda5671c] BuildCompressedImageCDR_Subsystem 0xda51fb0
    [0xda567ae / 0xda567b5] publish queue insertion
 -> FMyProjectZenohIORunnable::Run 0xda5d870
 -> FlushBatch 0xda5d960 -> Dequeue 0xda5ddd0
    [0xda5dd29] z_bytes_from_buf
    [0xda5dd56] z_put
```

**不是**“enqueue 后必调用同步 `PublishCompressedImage`”。后者 `0xda496f0` 是另一个存在的同步接口；本次虽有它的伪代码，也不拿它替代上面实际的 queue → IO 路径。Hub 可配置 publisher/sinks；`GetOrCreatePublisherLocked` 的原始函数体、Zenoh publisher 函数体和 native indirect dispatch 共同支持上述选定路径，但不构成所有运行时 publisher override 的保证。

CDR 原始 helper 顺序与公开 receiver 对齐：四字节 little-endian encapsulation `00 01 00 00`；`int32 sec`、`uint32 nanosec`、CDR string `frame_id`、CDR string `format`、`uint32 encoded_size`、encoded bytes。`0xda51fb0` 的 primitive/string helper `0xda5df60 / 0xda5e290` 也已完整解码，不能只凭函数名声称知道 wire ABI。公开解析器 D4 是独立对照。外层消息没有温度单位、发射率、每像素温度标定、相机内参或 extrinsic；图像尺寸由编码图像本身携带。内部 trigger/sequence 信息不意味着全部进入这个标准 wire header。

## Infrared material/resource investigation

### Exact compiled material reference

ctor 的 `FObjectFinder<UMaterialInterface>` 实际读取原始虚拟地址 `0x10bc2de` 的 UTF-16 字符串（`material-literal.json`；原始文件 offset 同时保留）：

```text
/Game/Sensors/Thermal_Imaging/Material/M_Infrared/
MI_Infrared_Post_Layer_Color.MI_Infrared_Post_Layer_Color
```

这是一个完整 object path，换行仅用于排版，不是两个路径。它比只看到 `InfraredMaterialFinder` 符号强：有实际构造调用、文字地址、原始字节和后续 blendable 使用。但不能据此推断实例的 parent graph 或温度模型。

### P1: actual package index, not loose filename guessing

保留发行包 `downloads/verified/MATRiX_v1.1.2.tar.gz` 的 tar inventory 将主 PAK 绑定到 tar data offset `2449957376`、member size `1640238903`。本次只保留其末尾约 14.95 MB 和相关索引；不把主包解压进 runtime。

PAK footer：version **12**，index encrypted flag **0**；primary index offset `1639646032`、size `279218`。primary SHA1 与 footer 相符；path-hash 和 full-directory 子索引的 SHA1 也分别与主索引相符。自定义限定版本 parser 消耗 full-directory index **200129 / 200129 bytes**，得到 **895 directories / 5326 paths**，数量与主索引 entry count 一致。`pak-inventory.json`、`pak-index-summary.json`、两个原始 index、全目录和选定目录均保留；验证会在字段边界、数量或校验不一致时停止。

已实际定位：

- `UeSim/Content/Sensors/Thermal_Imaging/Material/M_Infrared/MI_Infrared_Post_Layer_Color.{uasset,uexp}`：和构造函数 object path 对应。
- 同目录 `M_Infrared_Post_Layer.{uasset,uexp}`、`MC_Infrared.{uasset,uexp}`。
- `Curve/Curve_Infrared_Color`、`Curve_Infrared_Color1`、`CurveA_Infrared_Color_Map` 的 `.uasset/.uexp`。
- `UeSim/Content/ShaderArchive-Global-SF_VULKAN_SM6-VULKAN_SM6.ushaderbytecode` 和 `ShaderArchive-UeSim_Chunk0-SF_VULKAN_SM6-VULKAN_SM6.ushaderbytecode`。
- `UeSim/Content/Sensors/BP_PTZCamera.{uasset,uexp}`、`BluePrint/Robot/BP_MujocoSceneActor.{uasset,uexp}` 和 `AssetRegistry.bin`。

不能由“同目录基础材质”推断 `MI` 必定继承它，不能由 `Curve` 名字推断温度—颜色映射，不能由 shader archive 文件名推断已定位其中负责该材质的 SPIR-V。索引只证明这些记录确实包装在主包。

### Material body extent and temperature-unit boundary

进一步实际读取了上述 6 对红外资源以及 `BP_PTZCamera`、`BP_MujocoSceneActor` 的共 **16 个 PAK file records**，共 **13,627 bytes stored payload**。每一条都将 encoded-index 的 offset/size/compression 与原始 per-file header 核对，stored payload 的 SHA1 与该 header 一致；全部记录的 encrypted flag 为 false。原始 PAK record 留在 `material-records-win/`，未压缩 payload 另行保留；记录表和字符串观察在 `material-record-selection.json`、`material-record-results.json`。这已经超出仅查看目录/符号，但**仍不是材质图反编译**。

其中 **9 条未压缩、7 条 compression method 1（footer 的 Oodle）**。MI 的 `.uexp` 101 bytes 未压缩，但含名称/导入/父对象映射的 `.uasset` 以 Oodle 存储；base material 的 `.uasset` 和 `.uexp` 都压缩。没有现成且本次已验证的 cooked-unversioned material/Blueprint schema 与 Oodle 解码路径，不安装 decoder，不调用原 UE 可执行文件来绕过这个限制。因此尚不能从导出负载的裸索引/数值恢复 parent graph、shader 映射和 property 单位。

实际未压缩的 `MC_Infrared.uasset`（792 bytes）有 UE package tag `c1 83 2a 9e`，其可见名称包含 **`Infrared_Open`、`MaterialParameterCollection`、`Default__MaterialParameterCollection`**；两个未压缩 curve header 可见 `CurveLinearColor`。这些是**资源名称/类型线索**，不是已证明的温度 scalar、开尔文范围或颜色—温度反函数。`MC_Infrared.uexp` 60 bytes 也已读取；没有在缺少 property schema 时把任意 float/字节解释成温度默认值。未压缩两个 Blueprint `.uexp` 的 `BPTYPE_Normal` 字符串同样不证明恢复了生产 spawn 字节码。

资源读取的首个 WSL forward-gzip 扫描过慢，只对已确认命令的 owned PID 260 发送 SIGTERM；没有产出资源记录。改用**已存在**的 Windows Miniconda Python、同一原始 tar.gz 后完成上述有限记录读取。两个尝试及空的首次输出目录均保留在 `material-read-attempts.json`，不是安装/升级工具或将资源搬到 runtime。

目前输出层实际可主张的量：infrared 的 ImageWrapper 输入是颜色通道值，外层输出为 JPEG/PNG CompressedImage；公开 receiver 以 `cv2.IMREAD_COLOR` 得到 `uint8` 三通道 BGR 并展示。**颜色 DN/8-bit 通道强度没有被证明具有 °C、K、辐亮度或辐照度单位。** 没有可引用的公式将该输出逆变为温度，也没有本次已验证的 emissivity、blackbody response、ambient/reflected-temperature、NETD、绝对温度范围或可追溯 calibration 合同。

这与“底层材质一定没有热参数”是不同断言：后者目前未知。即使材质某处有一个名为 temperature 的 scalar，也还需要追踪其设定者、物理意义、投影/可见性、单位及输出编码才能称为 radiometric sensing。

## Secondary camera contracts: what is actually proved

| 类型 | 二进制已读到什么 | 公开合同/工具对照 | 不能顺势声称什么 |
| --- | --- | --- | --- |
| Infrared (enum 4) | 材质加载/blendable，透视颜色回读，通用 JPEG/PNG encode | D1 标为 CompressedImage；D4 infrared 解码是 uint8 BGR 图像 | raw 温度、°C/K、测温精度、热传导仿真、所有其他 thermal 路径均不存在 |
| Semantic (enum 5) | `PrepareSemanticCapture` 要求 `r.CustomDepth == 3`，注册 label/view-extension 目标；捕获分支强制 PNG；透视 distortion 传 semantic 标志 | D2：R=G=B=class ID、A=255；0 background、10 plane、20 stair、30 other-drive、40 human、50 vehicle、255 unclassified；D4 验证 PNG signature、通道一致、alpha | 概率/识别置信度、instance ID、材质温度；**未恢复 shader 主体，nearest-label sampling 规则目前是文档保证，不是本次逐条 shader 指令证明**；不支持 fisheye/panorama semantic（D2 的边界） |
| Fisheye (enum 3) | cube path；`BuildCubeProjectionSample` circular mask，FOV clamp 181..360°，K1..K4 径向多项式和 20 次二分求射线角；其后 face/UV 选择 | D1 + config 示例：Kannala–Brandt 几何；输出 CompressedImage | 深度、距离、真实镜头标定精度、未观测到的 per-pixel camera model 消息；cube seams/exposure 在真实 GPU 上无误 |
| Panorama RGB (enum 2) | 同 helper 的 equirectangular sin/cos 分支，六面 capture/projection | D1：360×180 equirectangular，CompressedImage | 额外 panorama depth 或 panoramic semantic；本次未恢复所有 GPU projection/tone-map helper |
| PTZRGB | 复用颜色相机；ctor 的 yaw/pitch pivots，采集时 SnapshotCapture；`EnsureConfiguredGimbalStarted` 与 stabilization 有完整原始指令 | D3：配置 `ptzrgb` → Spawn `BP_PTZCamera`；图像 CompressedImage + 独立 UDP `uesim.gimbal` v1.1；`StartGimbal` / `StartSensor` 两个生命周期 | 新物理测量类型、已证明自动变焦、PTZ 状态等于每帧真实曝光外参、UDP 控制有 ACK 就代表图像已交付 |
| Depth (enum 1, 用于对照) | PNG encode R/G packing；单独 `BuildRawDepthPayload` 实际写 `MDEP`, version 2，32-byte header + float32 | D1/D4：PNG `depth_m = R + G/255`；MDEP meters，version 2 带 sec/nsec | 可以把 depth 米制合同嫁接到 infrared；PNG 零值可当作零距离；float32 payload 是温度 |

MDEP v2 的实际布局：`magic[4], int32 version, int32 width, int32 height, float32 max_range_m, float32 horizontal_fov_deg, int32 sec, uint32 nanosec`，随后 `width*height` 个 little-endian float32。`BuildRawDepthPayload` 原始指令与 D4 的 `parse_mdep` 对齐；该函数伪代码含 bad-control warning，头部/拷贝结论也核对了完整指令。深度 PNG 的量化/无效值是产品合同的一部分，不能和视觉颜色图按 topic 猜测混用。

Acoustic：本次保留的公开 sensor guide/templates 没有给出 microphone/pressure stream 合同。**这不是整个二进制/整个产品不存在 acoustic 能力的断言**；音效资源、语音发送工具也不是声压测量合同。

## API lessons for LingTu — semantic lessons, not copied implementation

这些是交给主任务的可复用接口结论，不是本次对 LingTu 进行代码/架构修改：

1. **Measurement kind 和 presentation encoding 分开。** RGB/IR JPEG、semantic label PNG、depth metre float32 即使都来自 camera，也不能只叫 image 后靠 topic 名猜语义。Thermal API 需要真实单位/标定/无效值；没有这些时只能声明 visual infrared/palette image。
2. **Geometry/calibration 与 measurement law 分开。** fisheye/panorama 是投影合同；semantic 是标签合同；PTZ 是运动/控制合同。共享几何不代表共享物理输出。不要把某个外观滤镜当作热传感器模型。
3. **Configuration → binding → sample 三种事实分开。** 读到模板、成功 spawn、开始服务、收到真实 sample 各自只能证明各自的事。图像 `frame_id` 字符串不代替被验证的 mounting transform 和 capture-time pose。
4. **Capture identity 应贯穿异步路径。** 本次实际看到触发/时间/序列快照、generation-like 检查、弱对象检查和完成序列整理；可借鉴其明确时间/生命周期的意图，不复制私有内存布局或 machine code。对 LingTu 仍需由既有 typed contracts/owner 给出 reset 和有效帧规则。
5. **Backpressure 必须可见。** readiness/积压上限参与采集调度，目标 frequency 不等于交付 frequency；对外 API 应区分 requested、effective、dropped/stale 和 observed delivery。静态实现不能替代目标平台负载测试。
6. **GPU staging 和 wire encoding 是不同边界。** row pitch、PF_*、FColor/BGRA 是内部转换事实；网络 payload 由明确 codec/wire schema 定义。不要暴露 UE enum 数值或 MATRiX Zenoh/CDR 作为 LingTu 新的私有 ABI。
7. **控制与观测的生命周期分开。** PTZ 控制成功不证明已捕获；图像能读到不证明云台服务正常；需要 capture-time 状态才可用于 inspection evidence。保持 LingTu 原有 MuJoCo truth/clock、native typed DDS 和 ProductControl 所有权。

当前 Epic [EImageFormat 文档](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/ImageWrapper/EImageFormat) 甚至建议新代码走 ImageWrapperModule/FImage 的较高层接口，而不是逐个 wrapper。这是当前文档的建议，**不是证明本构建调用了新接口**，也不是要求主任务照搬 UE 依赖。

## REA ledger, failures and limits

| 本地标记 | 原始结果 | 真正 Evidence ID / 范围 |
| --- | --- | --- |
| E-A | `rea-warm-run/02-batch_decompile.json` | `ev_14d920d9f51b3f3be7e4f3259697f63125987d3687bdeb378a5f76defb74f178`，8/8：parser、ctor、RT、capture config、start、IR、semantic preparation、CPU pixels |
| E-B | `rea-warm-run/03-batch_decompile.json` | `ev_25af0ec6fb715901152f296d0465877085e4f8956aeb715fc652e26b1595a5bc`，7/7：scheduler、tick、capture、perspective/cube submit、readback process/closure |
| E-C | `rea-warm-run/04-batch_decompile.json` | `ev_b64c67383ab3df700c6f6ca53945e2aa69bf43f0709cfcbcfd0dd07e9288376f`，7/7：queue、codec、两个闭包、byte frame、Zenoh publisher、同步 compressed publisher |
| E-D | `rea-warm-run/05-batch_decompile.json` | `ev_b5bb5a549b52ec7248f1187e79782baedca07a67ca7bc4c30a224550b0386715`，7/7：cube geometry、distortion enqueue、depth raw/PNG、同步 readiness/trigger、settings |
| B-config / B-mount | `disassembly-followup/0xdd38cf0.json`, `0xdd739d0.json` 等 | 匹配 debug 符号边界的完整函数解码；不冒充反编译 |
| B-transport | `disassembly-followup/0xda56320.json`, `disassembly-serializer/0xda51fb0.json`, `disassembly-transport-tail/*.json`，以及 Hub/publisher 函数体 | enqueue → serializer → queue → IO FlushBatch → z_put，包含 serializer 小 helper，均保留 direct call sites |
| P1 | `pak-inventory.json`, `pak-index-summary.json`, 原始 indices，`pak-directory-all.json` | unencrypted PAK v12 的存储目录证据，非 REA Evidence；不是材质图或 shader 反编译 |

`camera-evidence.json` 是成功会话导出的完整 Evidence bundle。Ghidra **12.1.4**；投影 `camera-projection.elf` 120,472 bytes / 29 functions / 135 ranges，SHA256 `762050a21e45c6e4e18c1bc094d2344f721cfd2a8d381e9afaab3d3b4f09f443`。保留原始虚拟地址与选定函数/数据窗口/PLT；每一范围逐字节与原 executable 比较；新 ELF headers/sections/symbol labels 是分析载体，不可运行产品。未选定的 callee/type/vtable/代码与运行时内存不在投影内。

派生方式见任务自有 `make-camera-projection.py`、`make-followup-projection.py`；它们读取既有 builder 并在自有输出生成投影，**没有修改外部原脚本**。8 个 camera-core 原始 disassembly 文件从既有 `parallel-20261012/camera-pipeline/focused-camera-core` 复用；新增函数位于本次其他 disassembly 目录。`function-coverage.json` 按文件列出全部 48 个函数及 direct-call 关系，不把旧证据说成新发现。

必须保留的失败：

- `rea-run/failure.json`：第一批将 import 与 batch 合并在一个 300 秒客户端请求，`Request timed out`；server 日志随后显示该请求实际占用约 **325 秒**。因此不能声称该尝试遵守了硬性服务端 300 秒完成或已拿到可引用结果。客户端取消/清理的边界并非瞬时；没有把这批结果算作成功。
- 观察到上述问题后，用 `rea-camera-warm.mjs` 将 import 的 `list_segments` 与 batch 分开。warmup 约 **255 秒**；四批 decompile 约 **2.24 / 14.78 / 2.45 / 0.73 秒**，都小于 300 秒。这是改变请求后的有界恢复，不是重复同一失败调用。
- `rea-followup-run/01-list_segments.json`：14-function config/mount/transport 投影在 **provider_timeout / timeout_ms=300000** 未就绪；后面三个 batch 未运行，无成功伪代码。投影 96,672 bytes / 14 functions / 115 ranges，SHA256 `bc117bcdf06f444f68805ba703e2e7a8292ee11c8751b92a2f3d99db1cfe299a`；原始指令仍可用。未继续无限重试或安装别的引擎。
- 成功 warm session 的首次 `close_binary` 返回 `cleanup_incomplete`：观察到 `[java]` 与预期 JVM 命令身份不一致。finally 的第二次 close 返回成功；明确检查其 owned PID **504 不存在**。首次错误及完整 Ghidra 日志保留在 `rea-warm-run/07-close_binary.json`，不删除/掩盖。其他会话 finally close 结果也保留。最终自有 helper/WSL JVM 清理核验见 `cleanup-observation.json`。
- `ConfigureCaptureComponents`、`StartSensor`、`BuildRawDepthPayload` 有 bad-control/truncation 警告；`ProcessGpuReadbacks` 有大量类型/低地址伪影。对这些函数，只引用可由完整指令核对的结论；未选定的 GPU shader helper 或虚调用没有凭函数名补成实现。

## Official docs / source-reference evidence

以下上游源以保留 commit 固定，和本地读取内容对照；不是从当前 moving main 推断旧包行为：

- **D1**：[Sensor config tutorial](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/docs/Sensor_Config_Tutorial.md#L80-L196)：类型、位置/角度/轴、格式、深度/鱼眼/全景。文档是声明合同，二进制实现观察单列在上文。
- **D2**：[Semantic camera v1.1.2 rebuild](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/docs/Semantic_Camera.md)：明确 2026-09-23 重打包、class-ID PNG、标签类别、CustomDepth、nearest-label 几何与不支持的 renderer/projection 边界。
- **D3**：[Gimbal UDP/JSON protocol](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/docs/Camera_Gimbal_UDP_JSON_Protocol_CN.md#L77-L143)：协议线版本 1.1、文档版本 1.1.1；ptzrgb 的配置/Blueprint 映射和两个生命周期。文档日期 2026-08，与 runtime v1.1.2 是不同版本轴。
- **D4**：`zsibot/matrix@55ad3fec77deaf1b589d27fa6cc4bfca4553754e:Tools/zenoh_sensor_receiver.py:L314-L339`（[CDR parser](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/Tools/zenoh_sensor_receiver.py#L314-L339)）；`L414-L465`（[depth/color/semantic decoders](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/Tools/zenoh_sensor_receiver.py#L414-L465)）；`L489-L516`（[MDEP](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/Tools/zenoh_sensor_receiver.py#L489-L516)）；`L551-L569`（[infrared image wrapper](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/Tools/zenoh_sensor_receiver.py#L551-L569)）。这是同项目公开接收实现的补充交叉证据，不是另一个生产系统示例，也不证明实际 GPU 已输出。
- Epic enum/API 文档仅作符号解释：EPixelFormat 已请求 **5.6** 版本页；ERGBFormat/EImageFormat 的 5.6 版本页本次工具获取失败，使用的是标为 **5.8** 的当前页，明确有版本漂移风险。以二进制数字、字面 format 和实际数据搬运作为主要依据，不能用这些页面替代目标引擎确切版本鉴定。此调查没有另行完整鉴定原包 UE patch version。

没有用第三方教程作为行为证据，也没有开展依赖选型/替换比较。

`verification.json` 的定向离线检查通过：48 个函数地址唯一、完整解码字节数一致；29 个成功伪代码/identity 和 4 个 Evidence ID 保留；原始 source identity 一致；5,326 个目录条目与主索引数量匹配；16 个资源记录均未加密且 payload 校验匹配。该检查不执行模拟器、不生成伪造 sample，也不将静态检查升级成实际传感器/GPU 交付测试。

## Remaining gaps / retrieval sufficiency

原生相机采集→图像编码→选定 Zenoh queue/IO→z_put，以及不同 output kinds 的 API lessons，证据已足以复用。**完整七阶段生产闭环和 radiometric thermal claim 仍不足。** 下一步若继续，不应再重复 GPS 或把更多名字堆成“全链”：

1. 恢复被选定生产 Blueprint 的实际 class imports/export/script/property assignments，确定类型选择、创建、mount parent、body-follow 更新；对调用点做可引用绑定。只见 `.uasset`/`.uexp` 名称不算完成。
2. 在未加密且工具确实支持的前提下解析红外 MI、parent material、参数集合和曲线的引用/默认值，再定位对应 shader 与设置者；必须区分热状态输入、渲染 proxy 与 palette。压缩不是加密，但本次不安装 decoder、不把未解出的压缩数据解释成图。
3. 获取材质到像素的具体公式和单位/标定合同；若只有 palette/8-bit image，产品接口就必须诚实地停在视觉红外，不提供假温度读数。
4. 单独授权且环境具备时，才做目标 GPU/map 的已知条件捕获、同步、stale/reset、队列饱和和 receiver delivery 验收。本次明确禁止 UE launch，因此没有这些动态证据。

**Reusable takeaway：借鉴清晰的传感器几何、时间快照、异步积压和输出类型边界；不复制 MATRiX 私有机器码、资产或传输 ABI。infrared 的颜色后处理可以解释成视觉效果，但在材质输入与测量单位未闭合前，不能作为 LingTu 测温能力的依据。**
