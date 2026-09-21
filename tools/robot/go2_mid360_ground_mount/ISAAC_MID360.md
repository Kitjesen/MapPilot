# Isaac Lab / Isaac Sim 中的 MID-360 仿真路径

检索日期：2026-09-22。本文是方案调研，没有在本项目运行 Isaac 仿真，也未新增传感器依赖。

| 方案 | 所属环境 | 适用范围 |
|---|---|---|
| Isaac Lab RayCaster + 自定义射线方向 | Isaac Lab | 快速验证安装角、地面覆盖与几何遮挡；普通规则扫描不能代表 MID-360 非重复扫描 |
| RTX LiDAR + 多帧 emitterStateArrays | Isaac Sim，可集成到使用它的 Lab 场景 | 更接近真实扫描时序，需输入或提取每条射线方位、仰角、触发时间和通道；不是已经验证的即插即用包 |
| Livox-SDK/livox_laser_simulation | Gazebo / ROS | 厂商插件仓库含 mid360.csv，可研究其扫描序列；不能直接当作 Isaac Lab 扩展加载 |

对本支架建议先做几何覆盖：相同场景、相同地面、相同扫描序列比较原13°与新35°安装，统计机器人前缘0.5–2 m区域的点数、空白格比例与自身遮挡，不能只看总点数。再加入时间变化的非重复扫描、点时间戳、运动畸变及IMU时序，验证SLAM。两阶段均需把机器人和支架纳入射线可命中的几何；只对地面发射的高度扫描器不能检查自遮挡。

官方Isaac Sim讨论#685提供 emitterStateArrays 路径，但跟帖报告大配置的性能、输出频率和稳定性问题；不能把该帖子当作已稳定交付的MID-360驱动。官方RayCaster概览当前明确说明只支持静态网格；因此运动机器人自身遮挡不能直接用该默认实现保证，应采用静态姿态快照、额外的自遮挡求交或适配动态几何的传感器实现。具体API与采样调度必须按实际Isaac Sim / Lab版本确认。

URDF加载只建立形状和坐标关系，不会自动创建工作的点云传感器。此套件的 `livox_frame` 是设计光学坐标；需要另行创建RayCaster/RTX传感器并正确设置变换，不能把雷达底面原点误作扫描原点。实机标定值仍以实测为准。

## 来源

- [Isaac Lab RayCaster](https://isaac-sim.github.io/IsaacLab/main/source/overview/core-concepts/sensors/ray_caster.html)
- [Isaac Sim 非重复LiDAR官方仓库讨论](https://github.com/isaac-sim/IsaacSim/discussions/685)
- [RTX LiDAR官方教程](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_series/tutorial_ros2_rtx_lidar.html)
- [Livox 官方仿真插件](https://github.com/Livox-SDK/livox_laser_simulation)
