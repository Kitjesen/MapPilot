# TODO

> 按优先级排列，✅ 已完成 / 🔲 待做 / 🚧 进行中

---

## 近期 (2026 Q1 剩余)

### 国际化 (i18n)
🚧 **全 App 中英双语切换**
- ✅ 基础设施：`LocaleProvider` + `SharedPreferences` 持久化
- ✅ 首页 `home_screen.dart` — 6 张卡片 + 连接提示 + KPI 指标
- ✅ 侧边栏 `main_shell_screen.dart` — 导航标签 + 语言切换按钮
- ✅ 横幅 `feature_showcase_banner.dart` — 标语 + 功能卡片
- ✅ 状态页 `status_screen.dart` — 全部指标标签
- ✅ 事件页 `events_screen.dart` — 标题 + 空状态
- ✅ 设置页 `app_settings_screen.dart` — 全部菜单项 + 语言设置入口
- 🔲 地图页 `map_screen.dart` — ~45 条字符串（任务状态栏、对话框、工具提示）
- 🔲 任务面板 `task_panel.dart` — ~55 条字符串（任务控制、航点管理）
- 🔲 固件升级 `firmware_ota_page.dart` — ~45 条字符串（OTA 流程全部文案）
- 🔲 地图管理 `map_manager_page.dart` — ~45 条字符串（保存/加载/删除/重命名）
- 🔲 机器人详情 `robot_detail_screen.dart` — ~35 条字符串（模块列表、状态信息）
- 🔲 设备信息 `device_info_page.dart` — ~25 条字符串（硬件/服务/磁盘）
- 🔲 扫描连接 `scan_screen.dart` — ~10 条字符串（发现设备、连接提示）
- 🔲 相机画面 `camera_screen.dart` — ~10 条字符串（连接状态、全屏控制）
- 🔲 遥控页面 `control_screen.dart` — ~5 条字符串（控制权获取、急停）

### 构建 & 测试
- 🔲 colcon 全量构建验证（ROS 2 端 17 个包 + proto 生成）
- 🔲 端到端集成测试（gRPC 通道 → 导航节点 → Flutter App 闭环）

### 字体跨平台
- ✅ Windows: Microsoft YaHei（微软雅黑）
- 🔲 Android/iOS/Linux: 配置 fontFamilyFallback 回退链
  （Android → Noto Sans CJK, iOS → PingFang SC, Linux → WenQuanYi）

---

## 中期 (2026 Q2)

### 任务系统
- 🔲 TaskManager JSON 解析升级（支持复合任务描述）
- 🔲 pct_adapters 到达事件回调（航点到达精确触发）
- 🔲 任务历史持久化 + 回放

### 通信 & 可靠性
- 🔲 断联降级策略可配置化（yaml 定义降级级别 × 动作）
- 🔲 rosbag 集成（关键话题自动录制 + 远程导出）

### 导航 & 感知
- 🔲 定位质量阈值标定（ICP fitness score → 速度映射曲线实测调优）
- 🔲 围栏编辑 UI（地图上拖拽设置电子围栏多边形）

### App 体验
- 🔲 运行参数页：参数搜索功能（130+ 参数快速定位）
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
