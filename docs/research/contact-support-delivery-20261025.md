# 显式接触支撑修订：FDE 接线与交付记录

日期：2026-10-25。职责：核查正式配置、执行边界和可用Linux验证环境。
本记录与根代理的算法/测试改动配套，不启动机器人，不安装依赖，不改变硬件配置。

## 当前正式接线

```text
RobotConfig.geometry.navigation_body_file
    → assembly.compile_plan 的原生进程环境
    → RunPlan
    → LINGTU_NAV_BODY_FILE
    → parse.cpp / buildNavigationBody()
    → OctoPlanner3D 与 SCAN 的共享 Body
```

- `src/runtime/config.py::GeometryConfig` 默认 `navigation_body_file=""`。
- Go2的 `config/robots/unitree/go2/robot.yaml` 显式为空。注释选择单yaw OBB，
  固定多部件BODY2保留为离线资产。2026-10-25按用户要求将本地每侧水平余量
  改为0.05m后，当前源码水平包络为0.86×0.41m；该配置尚未部署，先前现场的
  0.96×0.51m记录不变。Z范围仍为机身原点下0.10m到上0.24m，不能当作真实脚的接触面。
- Thunder的 `config/robots/doso/thunder_v4/robot.yaml` 未设置该字段，使用空默认。
- `src/lingtu/assembly/compile_plan.py:609` 只在字段非空时解析相对RobotConfig路径，
  确认文件存在并写入 `LINGTU_NAV_BODY_FILE`；空值不会选择离线BODY2。
- `src/nav/cpp/endpoint/nav/app/config/parse.cpp:566` 读取该环境值。
- `buildNavigationBody()` 在文件非空时读取BODY2；否则按RobotConfig尺寸、水平
  hard margin和上下净空构造一个 `navigation_body` box，Body额外margin=0。

所以“有一份多部件离线模型”不能等于“正式默认已经切到该模型”。本轮接触语义
修改应只影响显式required-contact模型的路径，默认Go2/Thunder单OBB保留既有中心
支撑/连接判定。验证仍需以根代理最终实现和新增回归为准。

已进一步调用纯函数 `compile_run_plan('nav','real',robot=...)` 验证当前配置：

| RobotConfig | 进程控制 | 局部规划 | driver backend | RunPlan中的BODY2环境 |
|---|---|---|---|---|
| `unitree/go2` | systemd | scan | go2 | 未设置 |
| `doso/thunder_v4` | systemd | scan | doso | 未设置 |

原始结果保存于 `build/contact-support-20261025/fde/resolved-defaults.json`。
该步骤只编译不可变RunPlan，没有发布计划、切换Product或启动进程。

## 当前运动输出边界

原生 `toDdsFinalVelocity()` 只写 `linear.x/y`、`angular.z`。
driver的 `Velocity` 只有 `vx_mps/vy_mps/yaw_rps`。

- Go2 `move()` 调用 SDK 的 `Move(vx,vy,wz)`；零命令使用 `StopMove()`。
- Doso `move()` 调用 Brainstem 的三轴速度 `move()`。
- 没有将目标足点、摆腿轨迹或接触时序传给这条正式输出链。

本轮几何准入修订不增加落脚控制能力。显式接触模型能连续通过地图查询，仅说明
指定固定接触几何与地形相容；实际控制器是否保持这些接触仍需独立闭环证据。

当前圆弧blend的yaw envelope只支持无接触标签的单Box，显式required-contact模型
不会进入该提交路径，仍按原有分段/停转路线执行。本轮没有为接触模型添加圆弧执行，
也没有把不可达的blend调用当作需要新增防御逻辑的运行时缺陷。

## 现场平台与生命周期条件

`config/system/envs/real.yaml` 声明原生typed DDS、`field_dds_v1` 和systemd进程。
`nav` Product选择OctoPlanner3D、SCAN与原生最终速度出口；ProductControl仍是
公开生命周期入口。不能由新测试工具、Gateway或一个临时Python服务另起运动出口。

现场包要求Linux及实际目标架构（x86_64或aarch64），并使用匹配的CycloneDDS和
设备SDK。`scripts/deploy/README.md` 明确Windows开发二进制不属于Linux现场归档，
修改架构标签也不构成交叉编译。Go2 NX aarch64尚未由本轮x86_64验证替代。

交付继续使用现有路径：

```text
兼容目标的Linux构建
    → install/linux-<arch>/Release
    → 现有native release打包/安装
    → /opt/lingtu/releases/<version> 与 current
    → ProductControl激活既定env下的RunPlan
```

现场需要切换模型时，复用现有RobotConfig字段和正常Product冷启动；不在运行中
改BODY2内容、临时设置另一份环境变量或绕过ProductControl。本轮未执行该切换。

回退复用现有release installer与ProductControl事务：安装器保存旧目标及当前状态，
失败分支恢复旧release/记录并通过ProductControl恢复运行。FDE不另建一套回滚脚本。
在有监督现场验证前保留原发布版本与对应每台设备配置。本轮没有调用安装器或改current。

## 本机独立Linux验证环境

只读探测发现现有WSL Ubuntu可用：

| 项目 | 观测结果 |
|---|---|
| 系统 | Ubuntu 24.04.3 LTS，Linux x86_64 |
| CMake | 3.28.3 |
| 编译器 | GCC/G++ 13.3.0 |
| 构建器 | 系统 Ninja |
| CycloneDDS | 系统CMake包已存在 |
| OctoMap | 系统1.9.7 |
| GTest | 系统CMake包已存在 |
| 工作区 | `/mnt/d/inovxio/brain/lingtu` 可访问 |

独立目录为 `build/contact-support-20261025/fde/linux-x86_64`，没有重用Windows
CMake cache。按现有 `scripts/build/build_nav_endpoint.sh` 的WSL原则隔离Windows
PATH；未下载或安装依赖。配置日志在同级 `configure-linux.log`。

冻结后的实现已经通过Linux相关目标的编译与窄测。窄测针对三类具体失败：
显式接触模型在中心缺地面时被误拒、实际接触面缺失或未知时被误放行、共享支撑及
SCAN Body组件退化。任一失败都应回到对应查询/搜索实现修订，不能切换正式
默认或用速度出口绕过。

本轮有全局`runPlan`、共享支撑及SCAN Body组件回归，没有将同一条新接触路线贯穿
局部规划、完整物理闭环和实机执行；不能将这些组件测试表述为整链执行一致性证明。

| 验证 | 结果 | 原始记录 |
|---|---|---|
| 独立Linux Release配置 | 成功 | `build/contact-support-20261025/fde/configure-linux.log` |
| 六个相关测试目标编译链接 | 成功 | `build/contact-support-20261025/fde/build-linux.log` |
| 冻结后的增量构建 | `ninja: no work to do` | `build/contact-support-20261025/fde/build-linux-final.log` |
| 相关CTest | **50/50通过，25.29秒** | `build/contact-support-20261025/fde/linux-tests.log` |
| 通过用例完整stdout | 已从同次CTest的`LastTest.log`保留，未重跑 | `build/contact-support-20261025/fde/linux-native-test-output.log` |
| 正式`navd`编译链接 | **82/82成功，最终增量构建exit 0** | `build/contact-support-20261025/fde/build-navd-linux.log` |

六个目标为`test_body_support_map`、`test_scan_body`、`test_body_contact`、
`octoplanner3d_body_query_smoke`、`octoplanner3d_heading_smoke`与
`octoplanner3d_heading_reference`。CTest共包含BodySupportMapTest、ScanBodyTest
及上述四个独立测试项；50是CTest条目数，不是物理仿真或实机次数。

正式产物为`build/contact-support-20261025/fde/linux-x86_64/navd`。
`file`确认它是Linux x86-64动态链接ELF；`ldd`列出的OctoMap、CycloneDDS、OpenMP
及系统运行库均可解析，没有`not found`。该检查检测本机缺少动态库的问题，没有启动
`navd`，也不代表目标aarch64板端具备同样的运行库。

本轮保留了一次测试调用的Shell正则转义失败，记录在`linux-tests-invocation.log`。
它发生在CTest启动前，修正调用方式后的正式结果是`linux-tests.log`，没有将调用失败
计为算法失败，也没有删除该原始记录。

Linux构建暴露了已有告警：SCAN上游`polynomial_traj.h`存在有符号/无符号比较、
未使用变量，以及`getMeanVel()`缺少返回值。后者是既有代码缺陷：该非void函数到末尾
没有返回，不能当成无害提示；本次测试通过不证明调用该方法安全。上游优化器及SIMD/CMU实现有未使用
参数。正式端点还出现Route/EndpointLoopContext聚合初始化缺省成员和未使用函数
告警。这些位置不属于本轮接触支撑补丁，未为清理告警扩大改动；构建通过不能表述为
全仓无告警。

独立源码审阅定位该函数为`polynomial_traj.h:201`；检索`src/tests/tools/sim`只发现
定义，没有调用。因此不能用这一条告警解释本轮规划耗时或速度异常；后续可单独做
小补丁修复，本轮不因此重新打开已冻结的接触支撑实施。

正式端点构建另报`nav/app/goal/runtime.cpp:455`的`interrupt()`枚举switch没有处理
`kOperatorTakeover`。独立审阅已确认该枚举在同函数`runtime.cpp:379`的专用if分支
提前处理并return，无法到达455行switch。`safety/stop.cpp:423`先暂停运动输出，
`plan.cpp:881`在停止确认后提交Paused，保留task/epoch并要求显式resume。因此这是
switch穷举告警，没有遗漏操作者接管动作，本轮不添加重复业务分支。此源码核查仍不
等于现场接管验证。

## 目前能推进到哪一步

1. **已完成**：显式接触模型的正/负例组件回归，保留默认无接触OBB的旧语义。
2. **已完成**：独立Linux x86_64相关编译、50项窄测和正式`navd`链接。
3. **未完成**：对所选RobotConfig和真实控制器进行完整物理闭环；地面中心空洞放行不能
   被解释为任意跨沟或步态切换能力。
4. **未完成**：现场aarch64构建、无运动就绪、监督运动验证，按既有Product流程逐级进行。

本轮不发布、不部署、不触发运动。最终算法结果、Windows和Linux证据分别报告。
