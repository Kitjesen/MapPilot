# SLAM Package Tests

Tests for SLAM modules: SLAMModule (Fast-LIO2, Point-LIO, Localizer backends), SlamBridgeModule (external ROS2 SLAM), and point cloud worker.

```bash
python -m pytest src/slam/tests/ -q
```

Markers: `ros2` — tests requiring ROS2 runtime (bridge point cloud worker).
