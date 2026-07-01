# Simulation — Virtual Robot Drivers for Testing and Development

This package provides simulated robot drivers that replace real hardware for development, testing, and simulation-based evaluation. Multiple backends support different simulation fidelity levels.

## Files

- **`stub.py`** — StubDriver: mock driver for framework testing; publishes synthetic odometry and sensor data without any hardware or sim engine.
- **`mujoco_driver_module.py`** — MuJoCoDriverModule: full robot simulation via MuJoCo physics; publishes state, odometry, and sensor data.
- **`mujoco_lingtu_stack.py`** — MuJoCo LingTu stack: convenience factory that assembles driver+camera+lidar for LingTu-in-simulation.
- **`mujoco_live_runtime.py`** — Live MuJoCo runtime: real-time simulation loop with hardware-synced clock and external command input.
- **`mujoco_ros2_bridge.py`** - Compatibility entrypoint for `compat.ros2.mujoco_ros2_bridge`.
- **`mujoco_sensor_bridge.py`** — Sensor bridge: converts MuJoCo simulated sensors (lidar, camera, IMU) into standard message types.
- **`mujoco_viz_bridge.py`** - Compatibility entrypoint for `compat.ros2.mujoco_viz_bridge`.
- **`mujoco_scene_metadata.py`** — Scene metadata: defines robot URDF paths, scene XMLs, and camera intrinsics for each scene.
- **`ros2_sim_driver.py`** - Compatibility import for `compat.ros2.sim_driver`, which connects to external ROS2 simulation (Gazebo/GZ) as a LingTu driver.
- **`nova_nav_bridge.py`** - Compatibility entrypoint for `compat.ros2.nova_nav_bridge`.
- **`sim_pointcloud_provider.py`** — Simulated point cloud provider: generates synthetic PointCloud2 from scene geometry for SLAM testing.
- **`test_full_pipeline_s100p.py`** — End-to-end pipeline test: validates full stack from simulation through to control output.
- **`README_bridge.md`** — Bridge-specific documentation for ROS2 simulation setup and configuration.
