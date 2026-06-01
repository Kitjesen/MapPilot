# Navigation Package Tests

Tests for navigation modules: NavigationModule, GlobalPlannerService, SafetyRingModule, GeofenceManagerModule, CmdVelMux, WaypointTracker, MapManagerModule, and ROS2 WaypointBridge.

```bash
python -m pytest src/nav/tests/ -q
```

Markers: `ros2` — tests requiring ROS2 runtime (waypoint bridge).
