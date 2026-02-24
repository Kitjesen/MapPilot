# TODO

> 按优先级排列，✅ 已完成 / 🔲 待做 / 🚧 进行中

---

## 近期 (2026 Q1 剩余)

### 国际化 (i18n)
✅ **全 App 中英双语切换**
- ✅ 基础设施：`LocaleProvider` + `SharedPreferences` 持久化
- ✅ 首页 `home_screen.dart` — 6 张卡片 + 连接提示 + KPI 指标
- ✅ 侧边栏 `main_shell_screen.dart` — 导航标签 + 语言切换按钮
- ✅ 横幅 `feature_showcase_banner.dart` — 标语 + 功能卡片
- ✅ 状态页 `status_screen.dart` — 全部指标标签
- ✅ 事件页 `events_screen.dart` — 标题 + 空状态
- ✅ 设置页 `app_settings_screen.dart` — 全部菜单项 + 语言设置入口
- ✅ 地图页 `map_screen.dart` — 任务状态栏、对话框、工具提示
- ✅ 任务面板 `task_panel.dart` — 任务控制、航点管理
- ✅ 固件升级 `firmware_ota_page.dart` — OTA 流程全部文案
- ✅ 地图管理 `map_manager_page.dart` — 保存/加载/删除/重命名
- ✅ 机器人详情 `robot_detail_screen.dart` — 模块列表、状态信息
- ✅ 设备信息 `device_info_page.dart` — 硬件/服务/磁盘
- ✅ 扫描连接 `scan_screen.dart` — 发现设备、连接提示
- ✅ 相机画面 `camera_screen.dart` — 连接状态、全屏控制
- ✅ 遥控页面 `control_screen.dart` — 控制权获取、急停

### 构建 & 测试
- 🔲 colcon 全量构建验证（ROS 2 端 17 个包 + proto 生成）
- 🔲 端到端集成测试（gRPC 通道 → 导航节点 → Flutter App 闭环）

### 字体跨平台
- ✅ Windows: Microsoft YaHei（微软雅黑）
- ✅ Android/iOS/Linux: fontFamilyFallback 回退链（Noto Sans CJK / PingFang SC / WenQuanYi）

---

## 中期 (2026 Q2)

### 任务系统
- 🔲 TaskManager JSON 解析升级（支持复合任务描述）
- ✅ pct_adapters 到达事件回调（航点到达精确触发）
- 🔲 任务历史持久化 + 回放

### 通信 & 可靠性
- ✅ 断联降级策略可配置化（yaml 定义降级级别 × 动作）
- 🔲 rosbag 集成（关键话题自动录制 + 远程导出）

### 导航 & 感知
- 🔲 定位质量阈值标定（ICP fitness score → 速度映射曲线实测调优）
- 🔲 围栏编辑 UI（地图上拖拽设置电子围栏多边形）

### App 体验
- ✅ 运行参数页：参数搜索功能（130+ 参数快速定位）
- 🔲 运行参数页：参数预设方案（室内/室外/低速巡检一键切换）
- 🔲 离线模式优化（无连接时 graceful 降级 + 缓存最后状态）
- 🔲 深色模式审查（确保所有页面在 dark theme 下视觉一致）

---

## 远期 (2026 Q3+)

- 🔲 BehaviorTree 替代有限状态机（更灵活的任务编排）
- 🔲 多机器人协调（集群调度 + 冲突避免 + 统一监控面板）
- 🔲 仿真测试框架（Gazebo/Isaac Sim + CI 回归测试）
- 🔲 3D 地图可视化（点云渲染 + SLAM 建图实时预览）
- 🔲 移动端适配优化（Android/iOS 独立测试 + 手势交互打磨）

---

## 已完成归档

1. ~~将具体算法解耦出来，让我们的系统依赖于的是话题~~
   **已完成** — 见 `docs/TOPIC_CONTRACT.md`，所有模块通过 `/nav/*` 标准接口通信，
   算法通过 launch profile 即插即用 (slam_profile, planner_profile, localizer_profile)。

2. ~~将需要调制的参数解耦出来~~
   **已完成** — 见 `config/robot_config.yaml`，所有机器人物理参数集中存储，
   launch 文件通过 `_robot_config.py` 加载器自动读取。

3. ~~画面改一下，这个意思就是我们的应该是我们的。~~
   **已完成** — DS Logo + 大算 3D NAV 品牌统一。

4. ~~运行参数在线配置~~
   **已完成** — 130+ 参数 Flutter ↔ gRPC ↔ ROS 2 全链路打通，
   参数分 8 组 + 滑块/步进/输入框自适应控件 + 高级折叠区。

5. ~~中英双语基础设施~~
   **已完成** — LocaleProvider + 首页/侧边栏/状态/事件/设置 5 页双语切换，
   侧边栏 + 设置页双入口切换，偏好持久化。

6. ~~字体优化~~
   **已完成** — 全局切换为 Microsoft YaHei（微软雅黑），清晰易读。

7. ~~局部路径规划器性能优化~~
   **已完成** — 20 项 CPU 热路径优化 (O1-O14, P1-P6)，RotLUT + 平方距离 + memset +
   rotDirValid 预计算等，100Hz 主循环显著提速，55 个算法单元测试 + 性能验证套件。

8. ~~全 App 中英双语切换~~
   **已完成** — 15 个页面全部双语化，Android/iOS/Linux 字体回退链，侧边栏/设置双入口。

9. ~~运行参数页：参数搜索功能~~
   **已完成** — 全局搜索框实时过滤 130+ 参数，高亮匹配文本。

10. ~~pct_adapters 到达事件回调~~
    **已完成** — pct_path_adapter (C++/Python) 发布 /nav/planner_status 事件，
    TaskManager 订阅并更新进度追踪 + GetActiveWaypoints 返回真实进度。

11. ~~断联降级策略可配置化~~
    **已完成** — grpc_gateway.yaml 新增 4 参数 (disconnect_warn_sec/speed/stop_sec/action)，
    CheckDisconnection() 完全由配置驱动，支持 "idle" 或 "stop" 两种停车动作。
