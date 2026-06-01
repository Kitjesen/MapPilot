# Driver Package Tests

Tests for robot hardware drivers: ThunderDriver (gRPC bridge), stub driver, sim backends (MuJoCo, ROS2), TeleopModule, and RobotProfile.

```bash
python -m pytest src/drivers/tests/ -q
```

Markers: `sim` — tests requiring MuJoCo simulation backend.
