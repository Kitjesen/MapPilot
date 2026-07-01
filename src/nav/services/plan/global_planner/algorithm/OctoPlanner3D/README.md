# OctoPlanner3D

基于 PCD 点云与 OctoMap 的轻量级三维全局路径规划器。

本项目采用纯 C++ 实现，不依赖 ROS 生态，可方便集成到现有机器人系统、仿真平台或自主导航项目中。系统实现了从点云地图构建、可通行空间提取、障碍物碰撞检测到三维 A* 全局路径规划的完整流程，可用于四足机器人跨楼层导航、无人机三维路径规划以及室内复杂环境导航研究。

---

## 最新状态

目前已基于本项目封装了 ROS2 可视化测试版本：

**OctoPlanner3D-ROS2**

OctoPlanner3D-ROS2 是基于纯 C++ 版本 OctoPlanner3D 封装的 ROS2 可视化测试版本，主要用于验证 3D 全局路径规划算法在不同场景下的规划效果、性能表现和能力边界。

ROS2 版本地址：

[https://github.com/JackJu-HIT/OctoPlanner3D-ROS2](https://github.com/JackJu-HIT/OctoPlanner3D-ROS2)

需要说明的是，ROS2 版本仅在外层增加了节点封装和 RViz2 可视化能力，核心规划算法仍然保持纯 C++ 实现。

---

## 项目特点

- 基于 PCD 点云构建 OctoMap 三维地图
- 基于 OctoMap 的占据空间建模
- 支持可通行空间 Traversable Space 提取
- 支持基于机器人尺寸的碰撞检测
- 支持三维 26 邻域 A* 全局路径规划
- 支持跨楼层、跨高度差场景路径搜索
- 纯 C++ 实现，不依赖 ROS
- 模块结构清晰，便于集成到现有机器人项目
- 可扩展到四足机器人、无人机、巡检机器人等三维导航场景

---

## 规划流程

```text
PCD 点云
   │
   ▼
OctoMap 建图
   │
   ▼
可通行空间提取
   │
   ▼
碰撞检测
   │
   ▼
3D A* 搜索
   │
   ▼
全局路径
````

---

## 项目结构

```text
OctoPlanner3D
├── octomap
│   ├── include
│   │   └── pcd2octomap_converter.h
│   └── src
│       └── pcd2octomap_converter.cpp
│
├── planner
│   ├── include
│   │   └── global_planner.h
│   └── src
│       └── global_planner.cpp
│
├── pcd_files
│   └── building2_9.pcd
│
└── main.cpp
```

---

## 依赖项

### PCL

用于点云读取与处理。

```bash
sudo apt install libpcl-dev
```

### OctoMap

用于三维占据地图构建与查询。

```bash
sudo apt install liboctomap-dev octovis
```

---

## 编译方法

```bash
mkdir build
cd build
cmake ..
make -j
```

---

## 运行方法

```bash
./globalPlan
```

程序将自动完成：

1. 读取 PCD 点云；
2. 构建 OctoMap 三维地图；
3. 提取可通行空间；
4. 执行三维 A* 路径搜索；
5. 输出规划路径结果。

---

## 结果可视化

### 1. 查看 OctoMap 地图

程序运行后会生成：

```text
result_cleaned.bt
```

可以使用 OctoVis 查看：

```bash
octovis result_cleaned.bt
```

### 2. 查看规划路径

程序运行后会生成：

```text
planned_path.pcd
```

可以使用 PCL Viewer 查看：

```bash
pcl_viewer building2_9.pcd planned_path.pcd
```

其中：

* `building2_9.pcd`：原始环境点云；
* `planned_path.pcd`：规划生成的路径点。

---

## 使用示例

```cpp
octomap_obj = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();

octomap_obj->convert();

global_obj = std::make_shared<global_planner::GlobalPlanner>();

global_obj->setOctomap(octomap_obj->getOctomap());

global_planner::PointPose start;
global_planner::PointPose goal;

global_obj->makePlan(start, goal);

std::vector<global_planner::PointPose> path;
global_obj->getPlannerResults(path);
```

---

## 应用场景

* 四足机器人跨楼层导航
* 无人机三维路径规划
* 室内数字孪生导航
* 巡检机器人自主导航
* 三维环境自主探索
* 学术研究与算法验证

---

## ROS2 可视化版本

如果希望使用 ROS2 和 RViz2 对算法效果进行可视化测试，可以访问 ROS2 版本项目：

[OctoPlanner3D-ROS2](https://github.com/JackJu-HIT/OctoPlanner3D-ROS2)

该版本主要用于：

* RViz2 中显示 OctoMap 体素地图；
* 显示起点、终点和三维规划路径；
* 测试算法在不同地图和复杂三维场景下的规划效果；
* 验证算法性能表现和能力边界。

---

## 参考项目

本项目参考：

* [jie_3d_nav](https://github.com/6-robot/jie_3d_nav)

感谢原作者的开源工作。

---

## License

MIT License

---

## 作者

**Juchunyu**

如果本项目对您的研究或工程开发有所帮助，欢迎 Star ⭐ 支持。
