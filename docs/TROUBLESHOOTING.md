# 故障排除指南

## 编译错误：tf2_ros/buffer_interface.hpp 找不到

### 错误描述

编译 `sensor_scan_generation` 包时出现以下错误：

```
fatal error: tf2_ros/buffer_interface.hpp: No such file or directory
```

### 原因分析

在 ROS 2 Humble 中，`tf2_ros` 包的头文件路径或依赖可能不完整。这通常是因为：
1. `tf2_ros` 包未正确安装
2. 缺少相关的 ROS 2 基础包

### 解决方案

#### 方案 1：安装缺失的 ROS 2 包

```bash
sudo apt update
sudo apt install ros-humble-tf2-ros ros-humble-tf2 ros-humble-tf2-geometry-msgs
sudo apt install libusb-dev ros-humble-desktop-full ros-humble-tf-transformations ros-humble-joy python3-colcon-common-extensions python-is-python3 python3-pip git
pip install transforms3d pyyaml
```
#### 方案 2：安装完整的 ROS 2 基础包

```bash
sudo apt install ros-humble-desktop
```

#### 方案 3：检查并重新安装依赖

```bash
# 清理构建目录
cd ~/data/SLAM/navigation
rm -rf build install log

# 重新安装依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 重新构建
colcon build --packages-skip robot_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### CMake 警告处理

如果遇到 CMP0074 策略警告，可以在 `CMakeLists.txt` 中添加：

```cmake
cmake_policy(SET CMP0074 NEW)
```

或者在构建时使用：

```bash
colcon build --packages-skip robot_driver --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

### 验证

安装完成后，验证 `tf2_ros` 头文件是否存在：

```bash
find /opt/ros/humble -name "buffer_interface.hpp" 2>/dev/null
```

如果找到文件，说明安装成功，可以重新编译。

---

## 编译错误：tf2_geometry_msgs/tf2_geometry_msgs.hpp 找不到

### 错误描述

编译 `visualization_tools` 包时出现以下错误：

```
fatal error: tf2_geometry_msgs/tf2_geometry_msgs.hpp: No such file or directory
```

### 原因分析

`visualization_tools` 包的 `CMakeLists.txt` 中缺少 `tf2_geometry_msgs` 的依赖声明，虽然 `package.xml` 中已声明，但 CMake 配置中没有正确包含。

### 解决方案

#### 方案 1：修复 CMakeLists.txt（推荐）

在 `visualization_tools/CMakeLists.txt` 中添加缺失的依赖：

```cmake
find_package(tf2_geometry_msgs REQUIRED)
```

并在 `ament_target_dependencies` 中添加：

```cmake
ament_target_dependencies(visualizationTools
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
```

#### 方案 2：安装缺失的包

如果包未安装，执行：

```bash
sudo apt update
sudo apt install ros-humble-tf2-geometry-msgs
```

#### 方案 3：使用 rosdep 安装所有依赖

```bash
cd ~/data/SLAM/navigation
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 验证

安装完成后，验证头文件是否存在：

```bash
find /opt/ros/humble -name "tf2_geometry_msgs.hpp" 2>/dev/null
```

如果找到文件，说明安装成功，可以重新编译。
