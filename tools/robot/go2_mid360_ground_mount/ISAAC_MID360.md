# Isaac Lab / Isaac Sim 中的 MID-360 仿真路径

检索日期：2026-09-22。本文是方案调研，没有在本项目运行 Isaac 仿真，也未新增传感器依赖。

| 方案 | 所属环境 | 适用范围 |
|---|---|---|
| aCodeDog/OmniPerception | Isaac Lab 自定义 LidarSensor / RayCaster | 已有 MID-360 `.npy` 扫描数据和 `LivoxPatternCfg`；需修正逐帧序列推进并验证版本适配 |
| Isaac Lab RayCaster + 自定义射线方向 | Isaac Lab | 快速验证安装角、地面覆盖与几何遮挡；普通规则扫描不能代表 MID-360 非重复扫描 |
| RTX LiDAR + 多帧 emitterStateArrays | Isaac Sim，可集成到使用它的 Lab 场景 | 更接近真实扫描时序，需输入或提取每条射线方位、仰角、触发时间和通道；不是已经验证的即插即用包 |
| Livox-SDK/livox_laser_simulation | Gazebo / ROS | 厂商插件仓库含 mid360.csv，可研究其扫描序列；不能直接当作 Isaac Lab 扩展加载 |

对本支架建议先做几何覆盖：相同场景、相同地面、相同扫描序列比较原13°与新35°安装，统计机器人前缘0.5–2 m区域的点数、空白格比例与自身遮挡，不能只看总点数。再加入时间变化的非重复扫描、点时间戳、运动畸变及IMU时序，验证SLAM。两阶段均需把机器人和支架纳入射线可命中的几何；只对地面发射的高度扫描器不能检查自遮挡。

官方Isaac Sim讨论#685提供 emitterStateArrays 路径，但跟帖报告大配置的性能、输出频率和稳定性问题；不能把该帖子当作已稳定交付的MID-360驱动。官方RayCaster概览当前明确说明只支持静态网格；因此运动机器人自身遮挡不能直接用该默认实现保证，应采用静态姿态快照、额外的自遮挡求交或适配动态几何的传感器实现。具体API与采样调度必须按实际Isaac Sim / Lab版本确认。

URDF加载只建立形状和坐标关系，不会自动创建工作的点云传感器。此套件的 `livox_frame` 是设计光学坐标；需要另行创建RayCaster/RTX传感器并正确设置变换，不能把雷达底面原点误作扫描原点。实机标定值仍以实测为准。

## OmniPerception 源码与接入边界

仓库已将我们的开发 fork [Kitjesen/OmniPerception](https://github.com/Kitjesen/OmniPerception) 登记为子模块，
上游为 [aCodeDog/OmniPerception](https://github.com/aCodeDog/OmniPerception)。后续适配提交到我们的 fork，
通过本地 `upstream` remote 同步原项目。
固定提交 `fd37ab2e239113e28fbd816d33f83555e6076e69`（已合并 Isaac Sim 5.0 适配）。获取源码：

```sh
git submodule update --init third_party/research/OmniPerception
```

Isaac Lab 示例位于子模块内 `LidarSensor/LidarSensor/example/isaaclab/isaaclab/`，
扫描数据位于 `LidarSensor/LidarSensor/sensor_pattern/sensor_lidar/scan_mode/mid360.npy`。
配置应使用 `LivoxPatternCfg(sensor_type="mid360", use_simple_grid=False)`，不能以规则网格代替。
上游曾用旋转已有射线代替扫描窗口推进，且扫描文件缺失时回退到随机射线。
我们的 fork 已改为各环境独立推进和 reset，缺失文件直接报错；动态网格位姿改从
PhysX 刚体及固定相对变换计算，修复 Isaac Sim 5.0 的旧 XForm 位姿问题。

已在 Isaac Sim 5.0 / Isaac Lab 2.2.1 / RTX 3090 上通过两环境无渲染测试：
物体移动 0.2 m 时雷达距离变化约 0.2 m，MID-360 有限值观测、安装偏移和旋转、
局部 reset、丢点及编码器反向传播通过。使用独立源码导入，没有覆盖服务器的 Isaac Lab。
这不证明完整 PPO 训练、Go2 自遮挡或密集多环境隔离；扫描内运动畸变仍未实现。
接入方法和验证边界见 [fork 说明](https://github.com/Kitjesen/OmniPerception/blob/main/docs/isaacsim5_rl.md)。

## 来源

- [Isaac Lab RayCaster](https://isaac-sim.github.io/IsaacLab/main/source/overview/core-concepts/sensors/ray_caster.html)
- [Isaac Sim 非重复LiDAR官方仓库讨论](https://github.com/isaac-sim/IsaacSim/discussions/685)
- [RTX LiDAR官方教程](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_series/tutorial_ros2_rtx_lidar.html)
- [Livox 官方仿真插件](https://github.com/Livox-SDK/livox_laser_simulation)
