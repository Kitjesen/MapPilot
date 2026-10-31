# 支撑范围与碰撞余量分离：实现、证据与后续边界

> 历史实验记录：2026-10-27 已从当前导航代码删除逐脚接地证明和承重门槛。本文的旧测试结果保留用于溯源，不是当前运行规则；当前实现见 [组合碰撞模型设计](../design/navigation-body-model-and-motion-posture.md)。

日期：2026-10-25。已修改共享C++接触支撑函数并完成本地Windows Release验证。
这是一次范围受控的判据修复，没有引入足部规划、依赖、配置开关或新的服务。

后续已推进：[显式接触模型的中心支撑准入修订](contact-support-search-20261025.md)。
下文“中心规则未改”描述本次margin修复的边界，后续变更与交付证据单独记录。

## 这次专家式深入处理了什么

先明确待证明的物理命题：避障余量及连续扫掠padding扩大碰撞检查空间，
不应同时扩大轮足必须获得地面支撑的面积。以原始输入中的具体反例验证它，
再修改所有调用者共享的函数，而非通过降低全局阈值让案例通过。

另一个独立命题是“中心下方必须有地面是否必要”。该命题涉及起点、候选、
连接和连通性预检的准入集合，不能在这次局部修复中顺手删掉。本轮只定位并记录，
没有改变它。它仍是明确的待修订项。

## 从哪些原始材料入手

| 材料 | 本轮实际用途 | 不可替代的内容 |
|---|---|---|
| `collision/body.*`、`support.hpp` 的实际公式 | 查清primitive、保守OBB、margin与接触面的关系 | 原始几何/余量定义，不能由效果图猜测 |
| `scene.txt`、`map.ot` | 相同盒体生成地图，逐列检查occupied/free/unknown | 具体高度与自由证据，不能由点云外观猜地面 |
| `nominal.body2`、逐帧BODY2 | 固定与实测构型分别回放 | 实际关节变化后的几何，而非只看标称尺寸 |
| `states.jsonl` 的qpos/qvel、承重、命令 | 对齐物理时刻与拒绝原因 | 事后承重记录，不是预测未来接触的接口 |
| 真正的调用接线 | 区分普通路线、显式contact body与pose blend | 函数存在，不代表所有产品分支都启用 |

上述地图和运动记录来自10组人工Thunder/MuJoCo实验，不是903实机采集。
其机器人模型、策略与实验边界见 [原实验记录](support-gait-20261025.md)。

## 最小代码调整

`src/nav/cpp/collision/support.hpp::observedSupportSurface()`：

1. 必须支撑的范围，取源primitive紧包围范围与**零margin保守OBB**的并集。
   因此没有额外缩紧圆柱/胶囊的角部，也没有放宽为部分投影支撑。
2. 同高度地面占据、正上方已知自由、允许间隙/穿透的规则保留。
3. 已确认支撑后，地面接触豁免范围仍覆盖膨胀几何。它只允许已知占据、
   且顶面不高于接触面的地面体素；上方墙体与Unknown继续走完整碰撞拒绝。
4. BodyQuery与SCAN仍使用原margin和padding进行机体碰撞，没有削减碰撞余量。

`body.hpp/body.cpp` 同步修正说明，区分支撑证据范围与碰撞余量接触地面的范围。
没有增加另一套几何查询器。margin为0时，必需支撑列与返回接触范围保留原逻辑。

另外，`test_body_support_map` 原来误受两个benchmark开关控制，普通测试构建不会生成。
已将其移到正常tests条件下，要求已有OctoPlanner和GTest目标，不再要求打开benchmark。

## 方法和工具实际做了什么

- **解析构造反例**：尺寸明确的足部、体素和墙面；预期由接触/空间关系给出。
  例如足部边界以外的已知自由列不应增加承重需求，进入碰撞余量的墙仍要拒绝。
- **先红后绿**：先让旧函数运行新增测试，保存失败，再改共享函数。
- **性质检查**：改变collision margin不应替代缺失的实际支撑，扫掠padding
  不应额外要求地面；margin=0的范围按构造保留。
- **两个生产调用者检查**：同一fixture同时经过全局BodyQuery与SCAN Grid。
  这证明接线一致，不能作为两套独立物理算法的证明。
- **相同状态回放**：旧/新原生程序分别读取同一地图、模型、时刻和实测构型，
  保留每条CSV以及发生变化的原因。
- 工具：CMake/MSBuild、GTest/CTest、C++原生探针、Python标准库汇总；
  物理接触证据来自已保存MuJoCo记录。本轮没有重新执行物理或使用新第三方碰撞库。

## 验证结果

旧实现：支撑测试25例中22通过、3个预期失败，原有17例全部通过。
失败为余量外圈缺地、扫掠padding扩大支撑、余量内墙先报缺支撑。

修改后：**50/50个受影响CTest检查通过**，本机约4.77秒。
其中25个支撑测试（新增8个）、21个SCAN body测试，以及接触/几何、
全局body smoke和heading reference检查。heading reference内部仍覆盖原64例；
它的支撑/搜索语义未在本轮更改。

| 验收场景 | 修改后结果 |
|---|---|
| 实际未膨胀范围有地面，margin外圈为已知自由 | 接受 |
| 沿地板边缘平移，只有sweep padding越出地板 | 接受 |
| 平地加大避障余量 | 保持合法地面接触 |
| margin内有上方侧墙 | 支撑可建立，但最终occupied_body拒绝 |
| 实际范围下缺地面或Unknown | 保持支撑拒绝 |
| 另一高度楼板试图补足当前落脚面 | 拒绝 |
| 余量中有Unknown | 最终unknown_body拒绝 |

10组原始记录共334个姿态，分别使用固定/实测构型，共668条body判定。
修改前后有8条原因变化，全部在60cm通道：固定构型1条、实测构型7条，
从support_missing变为occupied_body。所有free/rejected布尔结果保持一致。
因此本轮没有证明这10例中的通行率提高，证明的是支撑与碰撞原因分开；
局部解析反例中的误拒则已经消除。

60cm通道t=6秒，实际轮缘距墙约23.1mm，当前保守OBB加margin却越墙约1.39mm。
新程序继续拒绝它是符合当前碰撞包络规则的，不能为了让实验通过再减少margin。

50cm纵向槽没有建立有效初始站立，仍拒绝。横向断口仍保留原接触规则中止的限制。
中心支撑查询本轮不变，不能声称纵向槽已接通正式全局导航。

## 除了你我，还有什么支持这个结论

1. **可手算的几何反例**：给出固定尺寸和占据关系，别人可独立判断预期，不必认可
   我们的整体导航设计。新增测试不是按函数分支抄答案。
2. **可执行回归**：原代码的3个失败与修改后50个通过均保存。任何有同依赖环境的人
   可复跑；测试已进入正常构建条件。未推送或运行远端CI，不能宣称CI已通过。
3. **独立物理记录**：MuJoCo接触不是由我们的支撑谓词生成，能揭示定义偏差。
   但模型、策略、接触阈值仍有限制，不能代替实机验证。
4. **独立代码审阅**：审阅者核对了Unknown、上方障碍、接触高度与margin=0关系，
   没发现本轮范围内需新增防御逻辑的问题。这是代码审阅，不是厂商/真人专家认证。

这些证据支持本次有限修改，不构成对所有地形、控制器和实际步态的完整证明。

## 仍待推进的具体问题

- 中心支撑前置条件仍会挡住两侧实际承重、中心悬空的姿态。修改它时要同时保留
  步高/坡度，并调整连通预检的适用条件，不能只删起点检查。
- 圆轮真实底部接触区域与整个未膨胀投影仍有区别；本轮保留原保守定义。
- 单OBB的pose blend全足迹地面检查没有被本轮contact函数修复替换。
- 当前Go2正式配置是单OBB且无contact部件。本补丁修正显式接触Body共享路径，
  不改变其默认能力范围，不能宣称正式Go2支撑误拒已经全面解决。
- 未来若缩紧碰撞包络，先以解析圆柱/墙距离核验，再跑相同闭环，不能只看接受率。
- 精确落脚控制仍为独立后续能力。本轮没有改Product启停、运动仲裁或硬件配置。

## 结果与复现入口

- [对照页面](../../build/support-margin-20261025/index.html)
- [旧实现失败日志](../../build/support-margin-20261025/red-test.log)
- [修改后50项检查](../../build/support-margin-20261025/green-test.log)
- [逐条判定变化](../../build/support-margin-20261025/changes.json)
- [回放程序](../../tools/bench/support_margin_compare.py)
- [回归源代码](../../tests/nav/cpp/test_body_support_map.cpp)

```powershell
cmake -S src/nav/cpp -B build/nav-cpp/windows-x64-endpoint-cyclone-octo
cmake --build build/nav-cpp/windows-x64-endpoint-cyclone-octo --config Release --target test_body_support_map test_body_contact test_body_geometry test_scan_body octoplanner3d_body_query_smoke octoplanner3d_heading_reference octoplanner3d_support_gait_probe --parallel 4
ctest --test-dir build/nav-cpp/windows-x64-endpoint-cyclone-octo -C Release -R '^(BodySupportMapTest\.|ScanBodyTest\.|test_body_contact$|test_body_geometry$|octoplanner3d_body_query_smoke$|octoplanner3d_heading_reference$)' --output-on-failure
```

Before回放须在旧实现编译后执行，After在新实现编译后执行，使用新的输出目录：

```powershell
uv run python tools/bench/support_margin_compare.py --source build/support-gait-20261025-r1 --out build/support-margin-new-run --native build/nav-cpp/windows-x64-endpoint-cyclone-octo/Release/octoplanner3d_support_gait_probe.exe --label before
# Apply/build the predicate change.
uv run python tools/bench/support_margin_compare.py --source build/support-gait-20261025-r1 --out build/support-margin-new-run --native build/nav-cpp/windows-x64-endpoint-cyclone-octo/Release/octoplanner3d_support_gait_probe.exe --label after
```

旧头文件另存于本地结果目录，便于对照未提交前的实现。本轮保留其他工作区修改。
构建成功；依赖SCAN中已有数值转换/未使用变量警告仍存在，触及的共享支撑代码与
新增测试未产生新的编译诊断。未部署、未推送。
