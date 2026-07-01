# S100P 栈盘点报�?
> **Date**: 2026-04-09
> **Hostname**: robot
> **Boot time**: 13:54:23
> **Load avg at scan**: 9.36
> **Scan mode**: 只读，无改动
>
> 配套文档: [GOVERNANCE.md](GOVERNANCE.md) �?六条原则 + 四阶段推�?
---

## 0. Executive Summary

S100P �?*不是之前以为�?3 套栈，而是 5 代累积的 40+ systemd service + 1 �?SSH 手工启动的孤儿进�?*。根源不是技术问题，�?*历次迁移没清�?*�?
- **在跑�?enabled**（核心）: 7 �?service
- **在跑�?disabled**（僵尸）: 5 �?`cortex-*.service`
- **Legacy 未清�?*: 20+ �?service 文件�? 代迁移的残留�?- **SSH 手工孤儿**: 1 �?`lingtu.py nav --daemon`

核心冲突�?*LiDAR 驱动起了两份**，两份都 remap �?`/nav/lidar_scan`，订阅方数据包翻倍�?
---

## 1. 完整 Service 状态矩�?
### 1.1 在跑�?services�?2 个）

| Service | Enabled | Active | 代码位置 | 说明 |
|---------|:---:|:---:|---|---|
| `brainstem.service` | �?| �?| `~/data/brainstem/` | Dart CMS gRPC :13145, 机器人腿部控�?|
| `camera.service` | �?| �?| orbbec_camera launch | 相机 ROS2 节点 |
| `can-setup.service` | �?| �?| shell oneshot | CAN 总线初始�?(can0-can3) |
| `lidar.service` | �?| �?| `~/data/inovxio/lingtu/install/` | Livox MID-360 驱动 (**冲突�?1**) |
| `localization.service` | �?| �?| `~/data/inovxio/lingtu/install/` | Fast-LIO2 SLAM |
| `localizer.service` | �?| �?| `~/data/inovxio/lingtu/install/` | ICP 定位 |
| `nav-lidar-network.service` | �?| exited | network script | LiDAR 网络初始�?(oneshot) |
| `ota-agent.service` | �?| �?| `/opt/nova/ota/` | OTA 拉取 agent |
| `cortex-telemetry.service` | �?| �?| `/opt/nova/cortex/v0.2.8/` | 遥测服务�?*僵尸**）|
| `cortex-safety.service` | �?| �?| `/opt/nova/cortex/v0.2.8/` | 安全服务�?*僵尸**）|
| `cortex-control.service` | �?| �?| `/opt/nova/cortex/v0.2.8/` | 控制服务�?*僵尸**）|
| `cortex-nav-gw.service` | �?| �?| `/opt/nova/cortex/v0.2.8/` | nav gateway�?*僵尸**）|
| `cortex-askme-edge.service` | �?| �?| `/opt/nova/cortex/v0.2.8/` | askme edge�?*僵尸**）|

**"僵尸"定义**：`disabled` �?`active` —�?重启后不会再起来，但现在还活着。说�?*之前被手�?`systemctl start`**，活过一次但�?enable�?
### 1.2 废弃/未用�?5+ 个）

| 类别 | Service | 状�?| 处理建议 |
|---|---|---|---|
| **nav-* 摆设**�? 个）| nav-slam / nav-planning / nav-lidar / nav-grpc / nav-driver / nav-autonomy / nav-monitor / nav-perception / nav-semantic | disabled + inactive | **删除 .service 文件** |
| **lingtu_* 失败迁移**�? 个）| lingtu_camera / lingtu_lidar / lingtu_slam | disabled + inactive | **删除**（WorkingDirectory 指向不存在的 `/opt/nav`）|
| **askme �?*�? 个）| askme / askme-dds-bridge / askme-frame-daemon | disabled + inactive | **删除** |
| **brainstem-ros2-bridge** | | disabled + inactive | 确认后删�?|
| **slam_pgo.service** | | enabled + inactive | ⚠️ 异常：enabled 但没起来，确认用途后处理 |
| **cortex-ops-api** | | disabled + inactive | 删除（但其他 5 �?cortex 还在跑）|
| **ota-daemon.service** | | disabled + inactive | 删除（ota-agent 才是用的那个）|
| **fix-audio.service** | | enabled + **failed** | ⚠️ 需要修复或禁用 |

---

## 2. 在跑的进�?�?代码位置映射

### 2.1 核心进程链（systemd 管理�?
```
systemd.slice
�?├── brainstem.service
�?  └── pid 1729  dart:server.dar  ~/data/brainstem/han_dog/bin/server.dart
�?                (cwd=~/data/brainstem, MEDULLA_STANDALONE=1, port 13145)
�?├── camera.service
�?  └── pid 1277  ros2 launch orbbec_camera gemini_330_series.launch.py
�?├── lidar.service
�?  ├── pid 1730  ros2 run livox_ros_driver2
�?  └── pid 2886  livox_ros_driver2_node
�?                config: ~/data/inovxio/lingtu/install/livox_ros_driver2/...
�?                remap: livox/lidar �?/nav/lidar_scan  �?冲突�?1
�?├── slam.service
�?  ├── pid 1731  ros2 run fastlio2 lio_node
�?  └── pid 2876  fastlio2 lio_node (3.1GB RAM, 23% CPU)
�?                config: ~/data/inovxio/lingtu/install/fastlio2/.../lio_s100p.yaml
�?├── localizer.service
�?  ├── pid 1733  ros2 run localizer localizer_node
�?  └── pid 2900  localizer_node
�?                map: /home/sunrise/data/nova/maps/active/map.pcd
�?├── cortex-telemetry.service  (disabled 僵尸)
�?  └── pid 5449  python3 /opt/nova/cortex/v0.2.8/services/telemetry-hub/run_dev.py
�?├── cortex-safety.service  (disabled 僵尸)
�?  └── pid 5480  python3 .../dog-safety-service/run_dev.py
�?├── cortex-control.service  (disabled 僵尸)
�?  └── pid 5510  python3 .../dog-control-service/run_dev.py
�?├── cortex-nav-gw.service  (disabled 僵尸)
�?  └── pid 5536  python3 .../nav-gateway/run_dev.py
�?├── cortex-askme-edge.service  (disabled 僵尸)
�?  └── pid 5557  python3 .../askme-edge-service/run_dev.py
�?└── ota-agent.service
    └── pid 1294  python3 -m ota_agent.main --config /opt/nova/ota/config.yaml
```

### 2.2 SSH 孤儿进程（绕开 systemd�?
```
/user.slice/user-1000.slice/session-8.scope  (SSH session 8, 已断开)
�?└── pid 9765  python3 lingtu.py nav --daemon    (14:31:01 启动)
    �?        cwd: ~/data/inovxio/lingtu
    �?        PPid: 1  (systemd, 因为 --daemon �?fork detach)
    �?        监听: 5050 (Gateway), 8090 (MCP)
    �?    └── pid 9887  livox_ros_driver2_node  �?冲突�?2
                  二进�? /opt/nova/lingtu/v1.8.0/install/livox_ros_driver2/lib/
                  config: ~/.lingtu/generated/livox/MID360_config.json
                  remap: /livox/lidar �?/nav/lidar_scan  �?冲突�?2 (�?pid 2886 撞车)
```

---

## 3. 代码仓库版本

| 路径 | 版本 | 大小 | 状�?|
|---|---|---|---|
| `~/data/inovxio/lingtu/` | `VERSION=2.0.0` (git main `31bd815b`) | 2.5 GB | 活跃开发分支，本次 session �?push 到此分支 |
| `~/data/brainstem/` | 未查 | 未查 | brainstem 独立项目 |
| `/opt/nova/cortex/current/` �?`v0.2.8` | v0.2.8 | 152 MB | nova-dog runtime 旧项�?|
| `/opt/nova/lingtu/v1.8.0/` | `VERSION=1.7.5` | 272 MB | OTA 部署，但 VERSION 文件没更新（bug）|

**版本号不一致的 bug**: `/opt/nova/lingtu/v1.8.0/VERSION` 写的�?`1.7.5`。部署目录名�?`v1.8.0`，文件内容是 `1.7.5`。说�?OTA 打包脚本没同�?VERSION 文件�?
**OTA 配置**:
```yaml
server_url: "http://192.168.66.62:8000/api"    # 指向你的开发机
check_interval: 30                              # 30 秒拉一�?channel: "stable"
deploy.base_dir: "/opt/nova/ota"
```

---

## 4. 冲突点分�?
### 4.1 LiDAR 驱动双份（确认）

两份 `livox_ros_driver2_node` �?remap �?`/nav/lidar_scan`�?
| 实例 | PID | 启动�?| 二进制路�?| 配置路径 |
|---|---|---|---|---|
| 1 | 2886 | `lidar.service` | `~/data/inovxio/lingtu/install/livox_ros_driver2/` | `~/data/inovxio/lingtu/install/.../MID360_config.json` |
| 2 | 9887 | `lingtu.py nav --daemon` (手工) | `/opt/nova/lingtu/v1.8.0/install/livox_ros_driver2/` | `~/.lingtu/generated/livox/MID360_config.json` |

**影响**�?- `/nav/lidar_scan` 有两�?publisher
- SLAM 订阅方（fastlio2）可能收到双倍点�?- 实际 LiDAR 硬件只有一个，其中一份可能无效连接（但进程还在跑，占 CPU�?
### 4.2 Cortex 栈的 "dog-safety" / "dog-control"

- `cortex-safety.service` (pid 5480) 运行 `/opt/nova/cortex/v0.2.8/services/dog-safety-service/run_dev.py`
- `cortex-control.service` (pid 5510) 运行 `dog-control-service/run_dev.py`

这些�?**NOVA Dog runtime** 项目的服�?—�?�?`D:\inovxio\products\nova-dog/runtime/` 项目里。它们跟 LingTu 导航�?*不是一个项�?*，但都在同一台机器上跑�?
**待验�?*：LingTu 导航栈是否依�?cortex �?gRPC 服务�?- 如果不依�?�?全部可以 stop
- 如果依赖 �?需要明确引用关�?
### 4.3 �?SLAM 实例的异�?
Journal 里看�?`14:44:41` 又启动了一�?`localization.service`（pid 11809）。对�?13:54:29 启动�?pid 2876，说�?SLAM 在运行中**�?systemd 重启过一�?*。原因不明，需要看 `journalctl -u slam.service` 详细日志�?
### 4.4 `lingtu_*.service` 的失败痕�?
```systemd
# /etc/systemd/system/lingtu_slam.service
WorkingDirectory=/opt/nav                    �?不存�?ExecStart=/bin/bash /opt/nav/scripts/services/nav-slam.sh   �?不存�?```

这是**失败的迁移尝�?*—�?有人想把 nav 栈搬�?`/opt/nav/`，但代码没到位，service 文件却留下了�?
### 4.5 OTA 部署没启�?
OTA Agent �?LingTu 拉到�?`/opt/nova/lingtu/v1.8.0/`，但**没有任何 service 启动这个目录**。它�?已下载未部署"的状态。真正在跑的�?`~/data/inovxio/lingtu/install/` 的开发版�?
---

## 5. 五代迁移考古

根据 service 文件命名约定和路径，推测历史�?
```
代次 1: NOVA Cortex (2026-03 之前)
  �?/opt/nova/cortex/v0.1.0 �?v0.2.4 �?v0.2.8
  �?cortex-*.service (6 �?
  �?现状: 5 个仍在跑 (disabled 僵尸)�? �?ops-api 没起

代次 2: �?nav �?(2026-03)
  �?/opt/nav/scripts/services/*.sh (�?/opt/nav 已不存在)
  �?nav-*.service (9 �?
  �?现状: �?disabled + inactive，摆�?
代次 3: 失败�?lingtu_* 迁移
  �?lingtu_camera/lidar/slam.service (3 �?
  �?指向 /opt/nav (不存�?
  �?现状: disabled + inactive，从没成功过

代次 4: 现役 SLAM �?  �?slam/lidar/localizer.service (3 个无前缀)
  �?依赖 ~/data/inovxio/lingtu/install/
  �?现状: 在跑

代次 5: OTA 部署�?LingTu 1.8.0
  �?/opt/nova/lingtu/v1.8.0/
  �?没有对应 service 启动
  �?现状: 文件在磁盘，�?live

旁支: askme �?  �?askme / askme-dds-bridge / askme-frame-daemon
  �?�?disabled，可能是实验残留
```

---

## 6. 决策建议

### 6.1 立即保留（核心，不动�?
| Service | 理由 |
|---|---|
| `brainstem.service` | 机器人腿部控制，动了机器人就�?|
| `camera.service` | 相机是感知基础 |
| `can-setup.service` | 硬件初始化，必须 |
| `nav-lidar-network.service` | LiDAR 网络初始化，oneshot 不影�?|
| `ota-agent.service` | 让它继续，但需要迁到生�?channel（不是你开发机 IP）|

### 6.2 过渡保留（先用，再迁移）

| Service | 迁移目标 |
|---|---|
| `localization.service` / `lidar.service` / `localizer.service` | 代码�?`~/data/inovxio/lingtu/install/` 迁到 `/opt/lingtu/releases/<version>/`，然后重�?ExecStart 指向新路�?|

### 6.3 立即处理（冲突源�?
| 动作 | 对象 | 命令 |
|---|---|---|
| **杀�?* | pid 9765 `lingtu.py nav --daemon` 孤儿 | `kill 9765` + 确认子进�?9887 同步退�?|
| **确认后停** | 5 �?cortex-* 僵尸 | 先验�?LingTu 是否依赖 dog-safety/dog-control �?gRPC |

### 6.4 清理（legacy 文件�?
| 类别 | 文件�?| 动作 |
|---|---|---|
| nav-*.service | 9 | archive �?`/opt/lingtu/archive/legacy-services/` |
| lingtu_*.service | 3 | 删除（指向不存在的路径）|
| askme*.service | 3 | archive |
| brainstem-ros2-bridge | 1 | archive |
| cortex-ops-api | 1 | archive |
| ota-daemon | 1 | archive |
| slam_pgo | 1 | 确认用途，否则 archive |
| fix-audio (failed) | 1 | 诊断 + 修复 or disable |

### 6.5 待确认后才能决定

- [ ] `/opt/nova/cortex/` 是否还需要？（问森哥 nova-dog 项目状态）
- [ ] LingTu 导航栈是否依�?cortex �?`dog-safety` / `dog-control`�?- [ ] `/home/sunrise/data/brainstem/` 的代码是哪个 git commit？对应哪�?brainstem 版本�?- [ ] `/opt/nova/lingtu/v1.8.0/` 为什�?VERSION 文件�?1.7.5？（打包 bug 修复�?- [ ] `~/data/inovxio/lingtu/` �?`/opt/nova/lingtu/v1.8.0/` 代码差异多大�?
---

## 7. 阶段 2 执行清单（立规矩�?
�?`GOVERNANCE.md` 的阶�?2 推进�?
### Step 1: 止血（Day 0�?
```bash
# 1. 杀�?SSH 孤儿进程（消除双 livox�?kill 9765
# 验证 9887 (livox child) 已退�?pgrep -af "lingtu.py nav --daemon"

# 2. �?cortex 僵尸（先验证 LingTu 不依赖）
for svc in cortex-telemetry cortex-safety cortex-control cortex-nav-gw cortex-askme-edge; do
    systemctl stop $svc
done

# 3. 确认载荷（load avg 应该下来�?uptime
```

### Step 2: 代码归位（Day 1-2�?
```bash
# 1. 准备目标目录
sudo mkdir -p /opt/lingtu/releases/v2.0.0-dev
sudo chown -R sunrise:sunrise /opt/lingtu

# 2. �?~/data/inovxio/lingtu/install/ 复制�?/opt/lingtu/releases/v2.0.0-dev/
#    (�?rsync, 避免 git 目录)
rsync -av --exclude='.git' --exclude='build' \
    ~/data/inovxio/lingtu/ /opt/lingtu/releases/v2.0.0-dev/

# 3. �?current symlink
ln -s /opt/lingtu/releases/v2.0.0-dev /opt/lingtu/current

# 4. 配置目录独立
mkdir -p /opt/lingtu/config
cp ~/data/inovxio/lingtu/config/robot_config.yaml /opt/lingtu/config/robot.yaml
```

### Step 3: 重写 systemd services（Day 2�?
创建新的统一入口 `lingtu.target`�?
```systemd
# /etc/systemd/system/lingtu.target
[Unit]
Description=LingTu Navigation Stack
Wants=lingtu-lidar.service lingtu-slam.service lingtu-localizer.service lingtu-nav.service
After=brainstem.service camera.service can-setup.service

[Install]
WantedBy=multi-user.target
```

然后每个 service 改指�?`/opt/lingtu/current/install/...`�?
### Step 4: 清理 legacy（Day 3�?
```bash
# 1. Archive 所有废�?service 文件
sudo mkdir -p /opt/lingtu/archive/legacy-services-2026-04-09
sudo mv /etc/systemd/system/nav-{slam,planning,lidar,grpc,driver,autonomy,monitor,perception,semantic}.service \
        /opt/lingtu/archive/legacy-services-2026-04-09/
sudo mv /etc/systemd/system/lingtu_{camera,lidar,slam}.service \
        /opt/lingtu/archive/legacy-services-2026-04-09/
sudo mv /etc/systemd/system/askme*.service \
        /opt/lingtu/archive/legacy-services-2026-04-09/
sudo mv /etc/systemd/system/brainstem-ros2-bridge.service \
        /opt/lingtu/archive/legacy-services-2026-04-09/
sudo mv /etc/systemd/system/ota-daemon.service \
        /opt/lingtu/archive/legacy-services-2026-04-09/

# 2. Reload systemd
sudo systemctl daemon-reload

# 3. 验证
systemctl list-unit-files | grep -E "nav-|lingtu_|askme" || echo "clean"
```

### Step 5: 启用新栈

```bash
systemctl enable lingtu.target
systemctl start lingtu.target

# 验证
systemctl list-dependencies lingtu.target
```

---

## 8. 未解决问题（下一步调查）

- [ ] `brainstem-ros2-bridge.service` 的用途是什么？
- [ ] `slam_pgo.service` 为什�?enabled �?inactive？（PGO = Pose Graph Optimization，可能是 Fast-LIO2 的后优化模块�?- [ ] `fix-audio.service` 为什�?failed？影响导航吗�?- [ ] OTA channel 应该指向哪里？目前指�?192.168.66.62（你的开发机），生产应该�?GitHub Release
- [ ] brainstem �?`~/data/brainstem/` 的版本跟 `D:\inovxio\brain\brainstem` �?git 版本对应吗？
- [ ] `lingtu.py nav --daemon` 启动时为什么引用了 `/opt/nova/lingtu/v1.8.0/install/livox_ros_driver2/`？代码里硬编码还是环境变量？

---

## 附录 A：关键命令速查

```bash
# 查谁启动的进程（cgroup�?cat /proc/<PID>/cgroup

# 查进程启动时�?ps -o lstart= -p <PID>

# �?service 真实定义
systemctl cat <name>.service

# �?service 状�?systemctl is-enabled <name>.service
systemctl is-active <name>.service

# �?SSH 孤儿进程
ps -eo pid,ppid,sess,cgroup,cmd | grep session

# �?LISTEN 端口来源
ss -tnlp | grep <port>

# 启动顺序
journalctl --no-pager --boot | grep "Started" | head -50
```
