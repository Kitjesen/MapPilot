# 客户部署与首次使用

本页面向交付工程师与客户管理员。当前目标是匹配实际 RobotConfig 的 Linux 实机，支持的
发布架构为 aarch64/x86_64；Windows 本机控制台是开发验收入口，不是机器人安装包。

## 客户应拿到什么

由交付方提供同一验收版本的：原生发布包、Web 静态包、该台设备的配置与标定，以及必要的
地图／感知模型。客户不应自行编译 C++、安装前端开发环境或从聊天记录拼配置。
Python 环境现在有统一准备入口；完整离线环境和原生系统依赖镜像仍未形成一份经过目标机
验收的交付物。普通建图／导航对话不需要额外加载人物识别、视觉跟随或向量数据库。

## 首次安装（交付工程师）

前提：已准备与发布包匹配的 Linux 系统和原生运行库，已有 Python 3.10+、uv，能够访问
锁文件所需的软件源，或具有准备好的 uv 缓存。Python 准备不安装原生 SLAM／传感器 SDK 依赖。
升级前先保存需要的地图并保持机器人静止；原生安装器可能恢复已有 Product。

1. 解压原生包和配套 Web 包。Web 放在稳定的版本目录，例如 `/opt/lingtu/web/2.3.0`，不要放进临时解压目录。
2. 在原生发布包目录运行：

   ```bash
   # 只显示计划，不修改环境或启动设备
   python3 scripts/deploy/prepare_runtime.py --package-dir .

   # 创建独立环境、按 uv.lock 安装、用独立解释器检查导入
   sudo python3 scripts/deploy/prepare_runtime.py --package-dir . --apply
   ```

   默认环境为 `/opt/lingtu/venvs/<发布版本>`。脚本拒绝覆盖已有环境，也不放在会被替换的发布目录里。
   它安装 gateway/llm/agent/vision 依赖，输出 `install_command` 与 `service_install_command`；
   执行这些输出命令，分别复用现有 `install_nav.sh` 和服务目录安装器。
   检查导入成功不等于传感器就绪。失败时不激活版本，可能留下未完成的新环境。

3. 设置机器配置和 Web：

   - `/etc/lingtu/control.env`：设置实际 `LINGTU_ROBOT=<vendor/model>`，相机交付才设置对应变体。
   - `/etc/lingtu/gateway.env`：设置 `LINGTU_WEB_DIST=/opt/lingtu/web/2.3.0`，保留已有访问密钥等设置。
   - `/etc/lingtu/models.env`：仅填写需要的模型／搜索密钥。参考包内 `config/system/models.env.example`；
     文件由 root 所有、权限 0600。`lt-host.service` 在启动时加载它。不要将密钥写入模型 YAML、聊天或交付截图。
   - 每台机器的标定／网卡／传感器地址仍由 RobotConfig 和既有机器配置负责，不用默认值假装标定完成。

4. 安装并启用持久管理入口。使用前面输出的同一个 `LINGTU_PYTHON` 绝对路径：

   ```bash
   sudo env LINGTU_PYTHON=/opt/lingtu/venvs/v2.3.0/bin/python \
     bash /opt/lingtu/current/scripts/deploy/thunder/install_catalog_service.sh control
   sudo systemctl enable --now lt-control.service
   ```

   示例版本和机器人路径必须替换成实际交付值。服务目录名称 `thunder` 是现有通用部署资产的位置，
   不能据此把机器人选择改成 Thunder。首次安装不自动选择 Product，不自动开始行走。

## 客户日常怎么用

打开 `http://<机器人地址>:5050/`。持久入口在 Product 停止时仍能管理模式，但停止后当前 Host
也会停止，聊天后端尚非独立常驻服务；这一点仍待补齐。

1. 从网页选择建图；观察实际设备和建图状态，随后按现场操作规程采集。
2. 保存地图，查看保存／处理结果。保存成功、处理完成、可用于导航是不同状态。
3. 选择保存的地图切到导航，等待定位与原生导航状态满足条件。
4. 对话栏选择已配置模型。可先问“有哪些地图？”或“你记住了哪些偏好？”验证只读链路。
5. 地图编辑等写操作按确认卡片执行；可以取消当前对话。运动仍经过既有任务入口和原生最终检查。
6. Codex 在模型连接中走官方设备授权；其他密钥由管理员在服务端设置。授权成功还须验证真实推理。

普通 map/nav 现已声明并装配基础对话 Agent，复用同一模型配置；语义导航仍保留完整感知堆栈，
不会重复创建 Agent。聊天可用不代表“跟随红衣服的人”或“找水”已具备完整实机执行能力。

## 更新、回退与数据

继续使用现有原生安装器／ProductControl，不新增第二套启动顺序。升级前保存地图；
地图、标定、会话、偏好、账号凭据和密钥属于持久机器数据，应按交付范围备份。
当前还没有把这些数据的备份／恢复统一成客户一键操作。
Web 独立版本目录可以按原部署文档回退；Python 环境按版本隔离，不能随软件目录一起删除。

完整安装／回退机制见 [部署说明](../scripts/deploy/README.md)，日常使用见 [操作说明](operations.md)。
这次新增准备入口经过本地合同测试，尚未在干净 Linux 客户机上跑完整安装；不宣称整机一键交付已验收。
