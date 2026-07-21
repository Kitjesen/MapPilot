# Factory 3-Floor Staircase MuJoCo Demo — 交接文档

Status: historical handoff note. Do not treat host addresses or `/tmp` paths in
this file as current deployment instructions. Use `$LINGTU_ROBOT_HOST` for any
new connection command.

## 连接信息

### 机器人 (S100P 四足狗)
- **Host**: historical local-lab S100P target; set `LINGTU_ROBOT_HOST` for new runs
- **用户名**: sunrise
- **密码**: 通过 `S100P_PASSWORD` 环境变量提供，禁止写入仓库
- **平台**: 地瓜 S100P，RDK aarch64 (不是 Jetson)，Ubuntu 22.04，ROS2 Humble
- **显示环境**: GNOME Wayland + Xwayland
  - `DISPLAY=:0`
  - `XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.QRJWL3`
  - `XDG_RUNTIME_DIR=/run/user/1000`
- **SSH 连接**: `ssh sunrise@"$LINGTU_ROBOT_HOST"` (根据当前网络配置设置主机)
- **已安装**: Python3, MuJoCo, Pillow, numpy, ROS2 Humble
- **注意**: SSH 长命令含分号时偶尔 exit 255，建议写脚本 scp 过去再执行

### 本地开发机 (Windows 11)
- **代码仓库**: `D:\inovxio\brain\lingtu\` (git, main branch)
- **工作目录**: `C:\tmp\` (临时脚本和渲染输出)
- **Python**: 需要 paramiko (`pip install paramiko`)

---

## 代码位置

### 本地 Windows

| 路径 | 说明 |
|---|---|
| `D:\inovxio\brain\lingtu\` | 主仓库 (lingtu 导航系统) |
| `D:\inovxio\brain\lingtu\sim\scripts\factory_demo\` | **本 demo 代码 (已同步)** |
| `C:\tmp\factory_stairs_scene.py` | MuJoCo 场景 XML 生成器 (**主要编辑文件**) |
| `C:\tmp\deploy_stairs.py` | 部署脚本 (SSH 上传+渲染+live viewer) |
| `C:\tmp\fs_se.png` | 渲染: SE 全景 |
| `C:\tmp\fs_east.png` | 渲染: 东侧楼梯剖面 |
| `C:\tmp\fs_nw.png` | 渲染: NW 全景 |
| `C:\tmp\fs_stair1.png` | 渲染: 楼梯1近景 |

### 机器人上 (historical lab target)

| 路径 | 说明 |
|---|---|
| `/tmp/factory_global_path.npy` | **C++ ele_planner 规划的全局路径** (226 waypoints, 不要删) |
| `/tmp/factory_stairs_viz.py` | deploy_stairs.py 上传的 live viewer 脚本 |
| `/tmp/render_stairs.py` | deploy_stairs.py 上传的 offscreen 渲染脚本 |
| `/tmp/factory_stairs.log` | live viewer 运行日志 |
| `/tmp/fs_se.png` 等 | offscreen 渲染输出 |
| `/tmp/factory_traj.npy` | live viewer 保存的实际轨迹 |
| `/root/lingtu/` | ROS2 导航系统 workspace (与本 demo 无关，不要动) |

### 路径数据来源

`/tmp/factory_global_path.npy` 是之前用 ROS2 全局规划器 (`global_planner.py` + C++ `ele_planner.so`) 在 3 层工厂 tomogram 上规划出来的。**不需要重新生成**，直接用。

---

## 工作流

```bash
# 1. 在 Windows 上编辑场景
#    修改 C:\tmp\factory_stairs_scene.py

# 2. 一键部署到机器人
cd C:\tmp
python deploy_stairs.py

# 这个脚本会自动:
#   a) SSH 连接 "$LINGTU_ROBOT_HOST"
#   b) 生成场景 XML (调用 factory_stairs_scene.py)
#   c) 上传 live_viz + render 脚本到 /tmp/
#   d) 运行 offscreen render → 下载 4 张 PNG 到 C:\tmp\
#   e) 启动 live viewer (机器人屏幕上显示)
#   f) 监控日志直到 GOAL REACHED

# 3. 检查渲染结果
#    查看 C:\tmp\fs_se.png, fs_east.png, fs_nw.png, fs_stair1.png
```

---

## 当前状态

### 已完成
- 3层工厂场景 MuJoCo XML 生成
- **楼梯几何正确**: 17级台阶 x 18cm，踏板(tread) + 立面(riser) 锯齿轮廓 + 斜梁(stringer) + 栏杆(railing)
- **楼梯间围墙**: 半透明侧墙、背墙、前墙(门洞间)、上下平台、天花板
- **路径对齐**: 楼梯位置与 ele_planner 路径吻合，机器人能沿楼梯上下
- **Live viewer**: 30fps，speed=2.5m/s，trail capped 150，GOAL REACHED ~18s

### 未完成 — 核心问题

**从楼层通往楼梯的走廊/通道没有做出来。**

看渲染图可以明确看到:
- 楼梯间本身有围墙有楼梯 → OK
- 但从楼层主区域(x=0~20)到楼梯间(x=21~27)之间是**完全开放的空旷厂房**
- 机器人直线穿越开放区域到达楼梯，中间没有任何走廊结构
- 真实建筑里应该有封闭走廊连接主区域和楼梯间入口

---

## 场景布局

```
Y
20 ┌──────────────────────────────────────┐
   │             Floor (30x20m)           │
   │   shelf          shelf               │
   │                                      │
10 │        ★START (10.2, 10.0)           │
   │                                      │
   │                  ??? <─ 这里缺走廊    │
 8 │                     ┌───STAIR1────┐  │  楼梯1: y=8.0, x=[21,26.4]
   │                     │  17 steps   │  │
   │                     └─────────────┘  │
6.75│                    ┌───STAIR2────┐  │  楼梯2: y=6.75, x=[20,26.5]
   │                     │  17 steps   │  │
   │                     └─────────────┘  │
 0 └──────────────────────────────────────┘
   0          10        20    25     30    X

3 层楼: Floor1 z=0, Floor2 z=3.06, Floor3 z=6.12
```

## 路径数据 (ele_planner, 226 waypoints)

```
Start: (10.2, 10.0, 0.6)    — 1F 出发
  → 1F 走到楼梯区域: x≈10→26, y≈10→8
Stair 1 climb: x=26.5→21, y≈7.8-8.3, z=0.7→3.1
  → 2F 走到楼梯2: x≈21→26, y≈8→6.75
Stair 2 climb: x=26.5→20, y≈6.7-6.8, z=3.9→6.5
  → 3F 走到终点: x≈20→10, y≈6.75→10
End:   (10.0, 10.0, 6.68)   — 3F 到达
```

---

## 需要做的事

### 核心: 添加走廊连接楼层和楼梯间

在 `factory_stairs_scene.py` 中:

1. **新增 `_corridor()` 函数**
   - 参数: 起点x, 终点x, y_center, width, base_z, height
   - 生成: 两侧墙壁 + 可选天花板
   - 墙壁半透明 (alpha=0.4~0.6) 让路径可见

2. **每层楼都需要走廊**:
   - 1F: 从主区域(x≈18)到 stair1 入口(x=21), y=8.0, 宽度≈2.4m
   - 2F: stair1 出口到 stair2 入口 的过渡通道
   - 3F: stair2 出口到主区域

3. **走廊入口要有门框/开口**，不能是死胡同

4. **确保路径在走廊内部**，不穿墙

### 修改文件

只需要改 `C:\tmp\factory_stairs_scene.py`，然后 `python C:\tmp\deploy_stairs.py` 重新部署验证。

---

## 关键参数

| 参数 | Stair 1→2 | Stair 2→3 |
|---|---|---|
| x_floor_edge (楼梯顶端x) | 21.0 | 20.0 |
| y_center | 8.0 | 6.75 |
| step_d (踏板深度) | 0.32m | 0.38m |
| step_w (楼梯宽度) | 2.4m | 2.4m |
| n_steps | 17 | 17 |
| step_h (每级高度) | 0.18m | 0.18m |
| total_run (水平总长) | 5.44m | 6.46m |
| total_rise (垂直总高) | 3.06m | 3.06m |
| base_z (底部z) | 0.0 | 3.06 |

## 代码结构

`factory_stairs_scene.py` 中的函数:
- `_stair_geoms()` — 生成楼梯台阶几何 (tread+riser+stringer+landing) ✅ 已完成
- `_stair_railings()` — 楼梯栏杆 ✅ 已完成
- `_stairwell()` — 楼梯间围墙 (侧墙+背墙+前墙+平台+天花板) ✅ 已完成
- `_factory_objects()` — 工厂内家具 (货架+箱子+机器) ✅ 已完成
- `_corridor()` — **走廊** ❌ 需要新增
- `generate_scene_xml()` — 组装完整场景 XML

`deploy_stairs.py`:
- 导入 `factory_stairs_scene.generate_scene_xml()` 生成 XML
- 嵌入到 LIVE_VIZ 和 RENDER_SCRIPT 字符串
- paramiko SSH 上传到机器人 `/tmp/`
- 先 offscreen render 4 张图下载回 Windows
- 再 nohup 启动 live viewer 在机器人屏幕上播放
