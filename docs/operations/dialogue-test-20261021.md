# 对话跟随与搜寻：从哪里启动测试

更新：2026-10-21。适用于森哥当前 Windows 工作区和 Go2/NX 的首轮测试准备。
当前状态以 [roadmap](../roadmap.md#当前任务卡对话跟随与搜寻的首轮控制及实机准备) 为准。

**现在先测电脑上的文字控制，再恢复机器狗的静止相机与 SLAM。
新代码还未部署，当前没有已验收的板端跟随／搜寻启动包。**

## 1. 先分清三个对象

| 对象 | 当前状态 | 能测试什么 |
| --- | --- | --- |
| 电脑上的当前源码 `D:\inovxio\brain\lingtu` | 已有对话、选人、查询、取消与网页控制实现；有本地回归 | 控制逻辑和组件接线；模型及运动 endpoint 仍有替身，不能当作实机效果 |
| Go2 上已安装的 `/opt/lingtu/current` | 最后成功核对是 `.93`；2026-10-21 原 IP 的 SSH/5050 均超时，当前运行状态未知 | 恢复连接后先检查该安装版本的相机、深度、SLAM 和静止状态 |
| 新版本 `nav/semantic` | 当前源码已声明，尚未完成本轮 ARM64 构建、模型配置与部署验收 | 部署并完成静止与标定检查后，才测新的真实对话跟随／搜寻 |

`map/camera` 是建图与相机基础模式；新的语义任务入口是 `nav/semantic`。
旧 `.93` 没有当前的 semantic 变体。不要把本地新网页接旧后端后的表现当作新版本验收。

## 2. 电脑上现在就能执行的测试

在 **Windows PowerShell** 中执行，使用已存在的环境，无需重新安装依赖：

```powershell
Set-Location D:\inovxio\brain\lingtu
.venv\Scripts\python.exe -m pytest tests/integration/semantic/test_agent_instruction.py tests/runtime/test_follow_person_selection.py tests/gateway/test_instruction_tasks.py -q
```

2026-10-21 实跑结果：**68 passed**。它检查普通指令接入、人物选择、任务状态和取消等
控制行为。它没有运行真实检测模型，没有启动机器狗，也不是完整仿真。
本轮机器可读结果在 `build/dialogue-start-20261021/local-smoke.xml`。

如需检查网页任务反馈，再在同一终端执行：

```powershell
Set-Location D:\inovxio\brain\lingtu\web
node --experimental-strip-types --test tests/instructionTask.test.ts tests/agentTask.test.ts tests/motionTruthIntegration.test.ts
```

2026-10-21 实跑结果：**46 passed**。它检查任务卡、查询、取消、迟到回复和启动门控，
不证明真实模型、相机或运动已运行。

若只是查看当前网页界面，在仓库根目录执行：

```powershell
Set-Location D:\inovxio\brain\lingtu
npm --prefix web run dev -- --host 127.0.0.1 --strictPort
```

打开 <http://127.0.0.1:3000>。这个命令**只启动网页开发服务**，默认代理到电脑的
`localhost:5050`；没有后端时会显示未连接。它不启动相机、模型、SLAM 或导航。
按 `Ctrl+C` 结束该网页开发服务。不要把网页显示成功当作机器人服务已启动。

## 3. 恢复连接后，先测现机相机与 SLAM

以下使用最后已知地址 `192.168.66.65`；IP 变化时替换地址。
在 Windows PowerShell 登录 NX：

```powershell
ssh -t -o BatchMode=yes -o StrictHostKeyChecking=yes -o HostKeyAlias=192.168.123.18 -o ConnectTimeout=5 -i "$env:USERPROFILE/.ssh/inovxio_deploy" unitree@192.168.66.65
```

登录后，下面的命令在 **机器狗 Linux 终端** 执行。
先读状态；若已有需要保留的建图会话，先保存，再切换：

```bash
sudo bash /opt/lingtu/current/scripts/lingtu status --robot unitree/go2 --env real --json
```

保持机器狗静止，启动现有安装的相机建图模式：

```bash
sudo bash /opt/lingtu/current/scripts/lingtu switch map --robot unitree/go2 --env real --variant camera --json
```

`sudo` 如提示密码，在终端交互输入。现有 SSH 密钥能登录普通账户，不代表能免密提权。
此前非交互启动确实返回过 `sudo: a password is required`；这是系统凭据条件。
密码不要发到聊天里。

启动成功后，在电脑打开 <http://192.168.66.65:5050>，检查相机画面与 SLAM。
在机器人终端可读取详细状态：

```bash
curl --silent --show-error 'http://127.0.0.1:5050/api/v1/readiness?details=true'
curl --silent --show-error http://127.0.0.1:5050/api/v1/camera/status
curl --silent --show-error http://127.0.0.1:5050/api/v1/localization/status
curl --silent --show-error http://127.0.0.1:5050/api/v1/navigation/status
```

应核对 RGB/depth 持续更新、SLAM 状态和位姿新鲜度，以及任务空闲、无运动控制者、
Driver 停止反馈。接口返回认证错误时先处理已部署的访问配置，不能当作算法失败。
启动回执或相机一张画面都不能代替这些状态。

当前源码中的完整只读诊断命令为：

```bash
source /opt/lingtu/config/python.env
sudo env PYTHONPATH=/opt/lingtu/current/src "$LINGTU_PYTHON" -m diagnostics.field.doctor --env real --non-motion --require-camera --json --strict
```

`doctor` 不接收 `--robot`；机器人是在 ProductControl 启动时选择的，`--robot` 用在
ProductControl 的 `status/switch/stop`。旧安装的参数支持仍以其 `--help` 为准。

结束这次完整传感器测试，退出整个 Product：

```bash
sudo bash /opt/lingtu/current/scripts/lingtu stop --robot unitree/go2 --env real --json
```

这会关闭该 Product 的整套服务。它与运行中对话里的“取消当前任务”不同；
普通任务取消不应关闭整个 Product。Product 停止回执也不是电机物理停稳的单独证明。

## 4. 代码还需要收口什么

| 项目 | 要完成的事 | 当前不应做的替代操作 |
| --- | --- | --- |
| 板端检测 | 用已保存的同输入 CPU 参考验证 fixedconv TensorRT 输出，再配置有效 engine 和 C ABI 库；现候选只支持 person | 仅把 `detector` 改为 `yolo_trt` 就承诺找瓶子／杯子 |
| 目标选择与身份 | 配置并验证描述选人和人物外观模型；DINO 加载路径已实现，但默认配置不会自动启用它 | 把“我”默认猜成最近的人，或把短期轨迹 ID 当作长期身份 |
| 相机外参 | 对当前宇树 EDU 架子的实测安装做相机至机体／雷达外参及时间对齐验证 | 只把 `unverified` 改成 `verified` |
| 目标板版本 | 用匹配 ARM64/ABI 的必需依赖构建完整候选，选定模型配置，沿正式安装流程部署 | 用本地 x86 二进制、旧 `.93` 或单独新网页代替新版本 |

当前语义默认配置仍是 YOLO-World，类别为 `person . bottle . cup`。
TensorRT 人物候选与默认语义配置不能不加区分地替换。
NX 已盘点的 GTSAM 4.0.2、系统 GStreamer 1.16.3 尚不满足当前构建契约；
需要准备已有项目依赖要求的 GTSAM 4.3 和匹配 ARM 的视频 runtime。

语音输入／播报、说话人与人物绑定、Follow 暂停／继续、运行中目标纠正仍在未交付项。
首轮先验证文字输入与明确目标，任务切换采用取消并确认停止后再开始新任务。

## 5. 新版本部署并满足条件后，才测试真实对话

需要一张已保存、可定位和导航的地图。以下命令是**后续新版本**的启动入口，
不是让当前旧 `.93` 直接执行；将 `MAP_NAME` 替换成实际保存地图名称：

```bash
sudo bash /opt/lingtu/current/scripts/lingtu switch nav --robot unitree/go2 --env real --variant semantic --map MAP_NAME --json
```

静止检查和相机标定通过后，按监督场景在匹配版本网页里测试：

1. 输入“你现在在做什么”，检查事实状态和 run 身份。
2. 给出明确人物选择或可验证描述，再测试跟随；尚未绑定的“跟随我”会明确拒绝。
3. 输入“停止当前任务”，检查领域停止、原生停止反馈及现场实际效果。
4. 具备 bottle/cup 检测与视觉复核模型后，再测试找物；人物模型不能承担这一项。

每步结果记录到同一验收材料，代码测试、静止实机和监督运动分别判断。
当前任务与未完成条件继续维护在 roadmap，不另建一套任务状态。
