## Robot Driver

This package contains two driver implementations:

### 1. `driver_node.py` — Generic Robot Driver
Generic template driver with watchdog protection. Requires hardware-specific `send_to_motors()` implementation.

### 2. `han_dog_bridge.py` — Han Dog gRPC Bridge (四足机器人)
Bridges the 3d_NAV navigation stack to the Han Dog quadruped robot via `han_dog_message` gRPC.

#### Architecture

```
Flutter App (remote control)
    │ gRPC :50051
    ▼
GrpcGateway (3d_NAV remote_monitoring)
    │ ROS2 /cmd_vel (TwistStamped)
    ▼
★ han_dog_bridge ★
    │ gRPC :13145
    ▼
Han Dog CMS Service (robot onboard)
```

#### Data Flow

| Direction | Source | → | Destination | Description |
|-----------|--------|---|-------------|-------------|
| Command ↓ | `/cmd_vel` (TwistStamped) | → | `CMS.Walk(Vector3)` | 速度归一化到 [-1,1] |
| Command ↓ | `/stop` (Int8) | → | `CMS.SitDown()` / zero vel | 停车控制 |
| Telemetry ↑ | `CMS.ListenImu()` | → | `/Odometry` (Odometry) | IMU 姿态 + 角速度 |
| Telemetry ↑ | `CMS.ListenJoint()` | → | `/robot_state` (RobotState) | 关节角度/速度/力矩 |
| Lifecycle | Auto on start | → | `CMS.Enable()` / `StandUp()` | 自动使能站立 |
| Lifecycle | On shutdown | → | `CMS.SitDown()` / `Disable()` | 安全关机 |

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dog_host` | `127.0.0.1` | Dog CMS gRPC server address |
| `dog_port` | `13145` | Dog CMS gRPC server port |
| `max_linear_speed` | `1.0` | Max linear speed (m/s) for Walk normalization |
| `max_angular_speed` | `1.0` | Max angular speed (rad/s) for Walk normalization |
| `cmd_vel_timeout_ms` | `200.0` | Watchdog timeout (ms) |
| `control_rate` | `50.0` | Control loop frequency (Hz) |
| `auto_enable` | `true` | Auto-enable motors on connect |
| `auto_standup` | `true` | Auto-standup on connect |
| `reconnect_interval` | `3.0` | gRPC reconnect interval (s) |

#### Usage

```bash
# Local connection (dog CMS on same machine)
ros2 run robot_driver han_dog_bridge.py

# Remote connection
ros2 run robot_driver han_dog_bridge.py --ros-args \
    -p dog_host:=192.168.4.100 -p dog_port:=13145

# Full navigation stack launch
ros2 launch navigation_run.launch.py dog_host:=192.168.4.100
```

#### Dependencies

```bash
# han_dog_message Python package
pip3 install han_dog_message   # or: pip3 install -e dog/han_dog_message/python

# gRPC
pip3 install grpcio grpcio-tools

# ROS2 interface package (for RobotState msg)
# Built as part of the 3d_NAV workspace
```

#### Walk Command Normalization

The bridge converts ROS2 velocity commands (m/s, rad/s) to the dog's normalized [-1, 1] Walk vector:

```
Walk.x = clamp(cmd_vel.linear.x  / max_linear_speed,  -1, 1)   # forward/back
Walk.y = clamp(cmd_vel.linear.y  / max_linear_speed,  -1, 1)   # left/right
Walk.z = clamp(cmd_vel.angular.z / max_angular_speed, -1, 1)   # rotation
```

#### Safety Features

- **Watchdog**: No `/cmd_vel` for 200ms → zero velocity sent to dog
- **Reconnection**: Automatic reconnection with configurable interval
- **Safe shutdown**: On node exit → zero vel → sit down → disable motors
- **Stop signal**: `/stop` topic integration (soft=zero vel, hard=sit down)
